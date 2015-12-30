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

#include <mitsuba/render/volume.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/fresolver.h>

#define TETRAHEDRON_MESH_NO_CACHE
#include "tetra.h"


MTS_NAMESPACE_BEGIN


class ShellMappedDataSource : public VolumeDataSource {
public:
	ShellMappedDataSource(const Properties &props) : VolumeDataSource(props) {
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

	ShellMappedDataSource(Stream *stream, InstanceManager *manager) 
	: VolumeDataSource(stream, manager) {
		m_volumeToWorld = Transform(stream);
		m_shellfile = stream->readString();
		if ( !m_shell.load(m_shellfile.c_str()) )
			Log(EError, "Failed to load the shell file!");
        else
            Log(EInfo, "Shell mesh loaded: %u tetrahedra, tree depth: %u",
                m_shell.getTetrahedronCount(), m_shell.getTreeDepth());
		m_stepSizeScale = stream->readFloat();
        m_block = static_cast<VolumeDataSource *>(manager->getInstance(stream));
        configure();
	}

	virtual ~ShellMappedDataSource() {
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
		if (child->getClass()->derivesFrom(MTS_CLASS(VolumeDataSource))) {
            Assert(m_block == NULL);
			m_block = static_cast<VolumeDataSource*>(child);
		} else
			VolumeDataSource::addChild(name, child);
	}

	Float lookupFloat(const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return 0.0f;
        return m_block->lookupFloat(m_textureToData.transformAffine(p));
	}

	Spectrum lookupSpectrum(const Point &_p) const {
		Point q = m_worldToVolume.transformAffine(_p);
		Point p;
		if ( !m_shell.lookupPoint(q, p) ) return Spectrum(0.0f);
        return m_block->lookupSpectrum(m_textureToData.transformAffine(p));
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
	
	bool supportsFloatLookups() const { return m_block->supportsFloatLookups(); }
    bool supportsSpectrumLookups() const { return m_block->supportsSpectrumLookups(); }
	bool supportsVectorLookups() const { return m_block->supportsVectorLookups(); }
	Float getStepSize() const { return m_stepSize; }
    Float getMaximumFloatValue() const { return m_block->getMaximumFloatValue(); }

	MTS_DECLARE_CLASS()

protected:
	std::string m_shellfile;
    TetrahedronMesh m_shell;
    ref<VolumeDataSource> m_block;
	Transform m_worldToVolume, m_volumeToWorld;
    Transform m_textureToData;
	Float m_stepSize, m_stepSizeScale;
};

MTS_IMPLEMENT_CLASS_S(ShellMappedDataSource, false, VolumeDataSource);
MTS_EXPORT_PLUGIN(ShellMappedDataSource, "Shell-mapped data source");
MTS_NAMESPACE_END
