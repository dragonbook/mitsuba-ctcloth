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


MTS_NAMESPACE_BEGIN


class DirectionalAlbedoVolume : public VolumeDataSourceEx {
public:
	DirectionalAlbedoVolume(const Properties &props) : VolumeDataSourceEx(props) {
		if (props.hasProperty("warpAndWeft")) {
			m_warpAndWeft = props.getBoolean("warpAndWeft");
			m_warpAlbedo = props.getSpectrum("warpAlbedo");
			m_weftAlbedo = props.getSpectrum("weftAlbedo");
			m_degree = props.getFloat("degree");
			m_threshold = std::tan(m_degree);
		}
		else m_warpAndWeft = false;
	}

	DirectionalAlbedoVolume(Stream *stream, InstanceManager *manager)
	: VolumeDataSourceEx(stream, manager) {
        m_block = static_cast<VolumeDataSourceEx *>(manager->getInstance(stream));
        configure();
	}

	virtual ~DirectionalAlbedoVolume() {
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);
        manager->serialize(stream, m_block.get());
	}
		
	void configure() {
        if (m_block.get() == NULL)
            Log(EError, "No embedded volume specified!");

        m_stepSize = m_block->getStepSize();
		m_aabb = m_block->getAABB();

		std::ostringstream oss;
        oss << "Data AABB: " << m_aabb.toString() << "\nAABB: " << m_aabb.toString() << '\n';
		oss << "Step size = " << m_stepSize << "\n";
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
		return m_block->lookupFloat(_p);
	}

    Float lookupFloatEx(uint32_t id, const Point &_p) const {
        return m_block->lookupFloatEx(id, _p);
    }

	Spectrum lookupSpectrum(const Point &_p) const {
        return m_block->lookupSpectrum(_p);
	}

	Spectrum lookupSpectrumEx(uint32_t id, const Point &_p) const {
        return m_block->lookupSpectrumEx(id, _p);
	}

	Vector lookupVector(const Point &_p) const {
        Vector ret = m_block->lookupVector(_p);
        if ( !ret.isZero() ) ret = normalize(ret);
        return ret;
	}

	Vector lookupVectorEx(uint32_t id, const Point &_p) const {
        Vector ret = m_block->lookupVectorEx(id, _p);
        if ( !ret.isZero() ) ret = normalize(ret);
        return ret;
	}

    void lookupBundle(const Point &_p,
        Float *density, Vector *direction, Spectrum *albedo, Float *gloss) const {
        if ( density ) *density = 0.0f;
        if ( direction ) *direction = Vector(0.0f);
        if ( albedo ) *albedo = Spectrum(0.0f);
        if ( gloss ) *gloss = 0.0f;

        m_block->lookupBundle(_p, density, direction, albedo, gloss);
		if (albedo && direction && !albedo->isZero() && !direction->isZero()) updateAlbedo(albedo, direction);
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
	inline void updateAlbedo(Spectrum *albedo, Vector *direction) const {
		if (m_warpAndWeft) {
//			if (std::abs(direction->x) - std::abs(direction->y) > 0) *albedo = m_warpAlbedo;
//			else *albedo = m_weftAlbedo;
			if (std::abs(direction->y) > Epsilon && std::abs(direction->x) / std::abs(direction->y) < m_threshold) {
				*albedo = m_weftAlbedo;
			}
			else *albedo = m_warpAlbedo;
		}
		else {
			Spectrum res;
			res.fromLinearRGB(std::abs(direction->x) + Epsilon, std::abs(direction->y) + Epsilon, std::abs(direction->z) + Epsilon);
			*albedo = res;
		}
	}
    ref<VolumeDataSourceEx> m_block;
	Float m_stepSize;
	bool m_warpAndWeft;
	Spectrum m_warpAlbedo, m_weftAlbedo;
	float m_degree;
	float m_threshold;
};

MTS_IMPLEMENT_CLASS_S(DirectionalAlbedoVolume, false, VolumeDataSourceEx);
MTS_EXPORT_PLUGIN(DirectionalAlbedoVolume, "drectional albedo volume data source");
MTS_NAMESPACE_END
