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

#if !defined(__MICROFLAKE_FIBER_CACHE_H)
#define __MICROFLAKE_FIBER_CACHE_H

#include <mitsuba/core/plugin.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/render/phase.h>
#include <map>

MTS_NAMESPACE_BEGIN

#define MICROFLAKE_FACTORY_STDDEV_MIN       0.005f
#define MICROFLAKE_FACTORY_STDDEV_MAX       1.275f
#define MICROFLAKE_FACTORY_STDDEV_DEFAULT   0.5f

class MicroflakePhaseFunctionFactory
{
public:
    inline const PhaseFunction* get(Float stddev) const {
        if ( stddev < MICROFLAKE_FACTORY_STDDEV_MIN || stddev > MICROFLAKE_FACTORY_STDDEV_MAX )
            stddev = MICROFLAKE_FACTORY_STDDEV_DEFAULT;

        t_stddev_map &m = m_cache_phasefunc.get();
        const PhaseFunction* ret = m.get(stddev);
        if ( ret == NULL ) {
            m.add(stddev); ret = m.get(stddev);
        }
        return ret;
    }

private:
    struct t_stddev_map {
        t_stddev_map() {}

        ~t_stddev_map() {
            for ( t_map_data::iterator it = data.begin(); it != data.end(); ++it )
                it->second->decRef();
        }

        void add(Float stddev) {
            Properties props("microflake");
            props.setFloat("stddev", stddev);

			PhaseFunction *content = static_cast<PhaseFunction *> (PluginManager::getInstance()->
					createObject(MTS_CLASS(PhaseFunction), props));
			content->configure();
            content->incRef();

            data.insert(t_map_data::value_type(stddev, content));
        }

        const PhaseFunction* get(float stddev) const {
            t_map_data::const_iterator it = data.find(stddev);
            return it == data.end() ? NULL : it->second;
        }

        typedef std::map<float, PhaseFunction*> t_map_data;
        t_map_data data;
    };

    mutable PrimitiveThreadLocal<t_stddev_map> m_cache_phasefunc;
};

MTS_NAMESPACE_END

#endif /* __MICROFLAKE_FIBER_CACHE_H */
