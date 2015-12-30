/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2010 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/volume2.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/fresolver.h>

#define TETRAHEDRON_MESH_NO_CACHE
#include "tetra.h"


MTS_NAMESPACE_BEGIN


class ShellMappedDataSourceEx : public VolumeDataSourceEx {
public:
	ShellMappedDataSourceEx(const Properties &props) : VolumeDataSourceEx(props) {
		m_volumeToWorld = props.getTransform("toWorld", Transform());
		m_shellfile = props.getString("shellfile");
		fs::path resolved = Thread::getThread()->getFileResolver()->resolve(m_shellfile);
		if ( !m_shell.load(resolved.string().c_str()) )
			Log(EError, "Failed to load the shell file!");
        else
            Log(EInfo, "Shell mesh loaded: %u tetrahedra, tree depth: %u",
                m_shell.getTetrahedronCount(), m_shell.getTreeDepth());
		m_stepSizeScale = props.getFloat("stepSizeScale", 1.0f);
	}

	ShellMappedDataSourceEx(Stream *stream, InstanceManager *manager) 
	: VolumeDataSourceEx(stream, manager) {
		m_volumeToWorld = Transform(stream);
		m_shellfile = stream->readString();
		if ( !m_shell.load(m_shellfile.c_str()) )
			Log(EError, "Failed to load the shell file!");
        else
            Log(EInfo, "Shell mesh loaded: %u tetrahedra, tree depth: %u",
                m_shell.getTetrahedronCount(), m_shell.getTreeDepth());
		m_stepSizeScale = stream->readFloat();
        m_block = static_cast<VolumeDataSourceEx *>(manager->getInstance(stream));
        configure();
	}

	virtual ~ShellMappedDataSourceEx() {
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);
		m_volumeToWorld.serialize(stream);
		stream->writeString(m_shellfile);
		stream->writeFloat(m_stepSizeScale);
        manager->serialize(stream, m_block.get());
	}
		
	void configure() {
        if (m_block.get() == NULL)
            Log(EError, "No embedded volume specified!");

		m_worldToVolume = m_volumeToWorld.inverse();
        const AABB &aabb = m_block->getAABB();
        m_textureToData = Transform::translate(Vector(aabb.min))*Transform::scale(aabb.getExtents());

        m_stepSize = m_stepSizeScale*m_block->getStepSize();
        m_aabb.reset();
		for ( int i = 0; i < 8; ++i )
			m_aabb.expandBy(m_volumeToWorld(m_shell.getAABB().getCorner(i)));

		std::ostringstream oss;
        oss << "Data AABB: " << aabb.toString() << "\nAABB: " << m_aabb.toString() << '\n';
		oss << "Step size = " << m_stepSize << " (x " << m_stepSizeScale << ")";
		Log(EDebug, oss.str().c_str());
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(VolumeDataSourceEx))) {
            Assert(m_block == NULL);
			m_block = static_cast<VolumeDataSourceEx*>(child);
		} else
			VolumeDataSource::addChild(name, child);
	}

	Float lookupFloat(const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return 0.0f;
        return m_block->lookupFloat(m_textureToData.transformAffine(p));
	}

	Float lookupFloatEx(uint32_t id, const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return 0.0f;
        return m_block->lookupFloatEx(id, m_textureToData.transformAffine(p));
	}
    
	Spectrum lookupSpectrum(const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return Spectrum(0.0f);
        return m_block->lookupSpectrum(m_textureToData.transformAffine(p));
	}

	Spectrum lookupSpectrumEx(uint32_t _id, const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return Spectrum(0.0f);
        return m_block->lookupSpectrumEx(_id, m_textureToData.transformAffine(p));
	}

	Vector lookupVector(const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		Vector norm;
		TangentSpace tang;
		if ( !m_shell.lookupPoint(q, p, norm, tang) ) return Vector(0.0f);

        Vector tmp = m_block->lookupVector(m_textureToData.transformAffine(p));
        Vector ret = m_volumeToWorld(tmp.x*tang.dpdu + tmp.y*tang.dpdv + tmp.z*norm);
		if ( !ret.isZero() ) ret = normalize(ret);
		return ret;
	}

	Vector lookupVectorEx(uint32_t _id, const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		Vector norm;
		TangentSpace tang;
		if ( !m_shell.lookupPoint(q, p, norm, tang) ) return Vector(0.0f);

        Vector tmp = m_block->lookupVectorEx(_id, m_textureToData.transformAffine(p));
        Vector ret = m_volumeToWorld(tmp.x*tang.dpdu + tmp.y*tang.dpdv + tmp.z*norm);
		if ( !ret.isZero() ) ret = normalize(ret);
		return ret;
	}

    void lookupBundle(const Point &_p,
        Float *density, Vector *direction, Spectrum *albedo, Float *gloss) const {
        if ( density ) *density = 0.0f;
        if ( direction ) *direction = Vector(0.0f);
        if ( albedo ) *albedo = Spectrum(0.0f);
        if ( gloss ) *gloss = 0.0f;

		Point q = m_worldToVolume.transformAffine(_p);
		Point p;

        if ( direction ) {
		    Vector norm;
		    TangentSpace tang;
            if ( m_shell.lookupPoint(q, p, norm, tang) ) {
                m_block->lookupBundle(m_textureToData.transformAffine(p), density, direction, albedo, gloss);
                *direction = m_volumeToWorld(direction->x*tang.dpdu + direction->y*tang.dpdv + direction->z*norm);
		        if ( !direction->isZero() ) *direction = normalize(*direction);
            }
        } else {
            if ( m_shell.lookupPoint(q, p) )
                m_block->lookupBundle(m_textureToData.transformAffine(p), density, NULL, albedo, gloss);
        }
    }

    bool supportsBundleLookups() const { return m_block->supportsBundleLookups(); }
	bool supportsFloatLookups() const { return m_block->supportsFloatLookups(); }
    bool supportsSpectrumLookups() const { return m_block->supportsSpectrumLookups(); }
	bool supportsVectorLookups() const { return m_block->supportsVectorLookups(); }
	Float getStepSize() const { return m_stepSize; }
    Float getMaximumFloatValue() const { return m_block->getMaximumFloatValue(); }
    Float getMaximumFloatValueEx(uint32_t id) const { return m_block->getMaximumFloatValueEx(id); }

	MTS_DECLARE_CLASS()

protected:
	std::string m_shellfile;
    TetrahedronMesh m_shell;
    ref<VolumeDataSourceEx> m_block;
	Transform m_worldToVolume, m_volumeToWorld;
    Transform m_textureToData;
	Float m_stepSize, m_stepSizeScale;
};

MTS_IMPLEMENT_CLASS_S(ShellMappedDataSourceEx, false, VolumeDataSourceEx);
MTS_EXPORT_PLUGIN(ShellMappedDataSourceEx, "Shell-mapped data source 2");
MTS_NAMESPACE_END
