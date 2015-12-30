/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2011 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/volume.h>
#include <mitsuba/core/plugin.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/render/phase.h>
#include <mitsuba/render/medium.h>

#include "microflake_cache.h"

MTS_NAMESPACE_BEGIN

class MicroflakePhaseFunctionEx : public PhaseFunction {
public:
	MicroflakePhaseFunctionEx(const Properties &props) : PhaseFunction(props) {
	}

	MicroflakePhaseFunctionEx(Stream *stream, InstanceManager *manager) 
		: PhaseFunction(stream, manager) {
		configure();
	}

	virtual ~MicroflakePhaseFunctionEx() {
    }

	void configure() {
		PhaseFunction::configure();
        if (m_volume.get() == NULL)
            Log(EError, "No glossness specified!");

		m_type = EAnisotropic | ENonSymmetric;

        Properties props("microflake");
        props.setFloat("stddev", MICROFLAKE_FACTORY_STDDEV_MIN);
        m_dummyPhaseFunction = static_cast<PhaseFunction *> (PluginManager::getInstance()->
			createObject(MTS_CLASS(PhaseFunction), props));
        m_dummyPhaseFunction->configure();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		PhaseFunction::serialize(stream, manager);
	}

	void addChild(const std::string &name, ConfigurableObject *child) {
		if (child->getClass()->derivesFrom(MTS_CLASS(VolumeDataSource))) {
            Assert(m_volume == NULL);
			m_volume = static_cast<VolumeDataSource *>(child);
			Assert(m_volume->supportsFloatLookups());
		} else {
			PhaseFunction::addChild(name, child);
		}
	}

	Float eval(const PhaseFunctionSamplingRecord &pRec) const {
        const Float stddev = m_volume->lookupFloat(pRec.mRec.p);
        return m_factory.get(stddev)->eval(pRec);
	}

	inline Float sample(PhaseFunctionSamplingRecord &pRec, Sampler *sampler) const {
        const Float stddev = m_volume->lookupFloat(pRec.mRec.p);
        return m_factory.get(stddev)->sample(pRec, sampler);
	}

	Float sample(PhaseFunctionSamplingRecord &pRec, 
			Float &pdf, Sampler *sampler) const {
        const Float stddev = m_volume->lookupFloat(pRec.mRec.p);
        return m_factory.get(stddev)->sample(pRec, pdf, sampler);
	}

	bool needsDirectionallyVaryingCoefficients() const { return true; }

	Float sigmaDir(const Point &p, Float cosTheta) const {
        const Float stddev = m_volume->lookupFloat(p);
        return m_factory.get(stddev)->sigmaDir(p, cosTheta);
	}

	Float sigmaDirMax() const {
        return m_dummyPhaseFunction->sigmaDirMax();
	}

	std::string toString() const {
		return "MicroflakePhaseFunctionEx[]";
	}

	MTS_DECLARE_CLASS()

private:
    ref<VolumeDataSource> m_volume;
    ref<PhaseFunction> m_dummyPhaseFunction;
    MicroflakePhaseFunctionFactory m_factory;
};

MTS_IMPLEMENT_CLASS_S(MicroflakePhaseFunctionEx, false, PhaseFunction)
MTS_EXPORT_PLUGIN(MicroflakePhaseFunctionEx, "Spatially varying microflake phase function");
MTS_NAMESPACE_END
