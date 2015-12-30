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

#include <mitsuba/render/volume2.h>
#include <mitsuba/core/mstream.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/mmap.h>

MTS_NAMESPACE_BEGIN

class GridDataSourceEx_Simple : public VolumeDataSourceEx {
public:
	enum EVolumeType {
		EFloat32 = 1,
		EFloat16 = 2,
		EUInt8 = 3,
		EQuantizedDirections = 4
	};

	GridDataSourceEx_Simple(const Properties &props) 
		: VolumeDataSourceEx(props), m_ready(false) {
		m_volumeToWorld = props.getTransform("toWorld", Transform());

        m_nVolumes = static_cast<uint32_t>(props.getInteger("nfiles"));
        _reserve(m_nVolumes);
        for ( uint32_t i = 0; i < m_nVolumes; ++i ) {
            std::ostringstream oss;
            oss << "filename" << i;
            loadFromFile(props.getString(oss.str()), i);
        }

        m_defultFloatLookupId = static_cast<uint32_t>(props.getInteger("defaultFloatID", -1));
        m_defultSpectrumLookupId = static_cast<uint32_t>(props.getInteger("defaultSpectrumID", -1));
        m_defultVectorLookupId = static_cast<uint32_t>(props.getInteger("defaultVectorID", -1));
	}

	GridDataSourceEx_Simple(Stream *stream, InstanceManager *manager) 
			: VolumeDataSourceEx(stream, manager), m_ready(false) {
		m_volumeToWorld = Transform(stream);
		m_dataAABB = AABB(stream);
        m_nVolumes = stream->readUInt();
        _reserve(m_nVolumes);
        for ( uint32_t i = 0; i < m_nVolumes; ++i )
		    loadFromFile(stream->readString(), i);
        m_defultFloatLookupId = stream->readInt();
        m_defultSpectrumLookupId = stream->readInt();
        m_defultVectorLookupId = stream->readInt();
		configure();
	}

	virtual ~GridDataSourceEx_Simple() {
        for ( uint32_t i = 0; i < m_nVolumes; ++i )
            if ( !m_mmap[i].get() ) delete[] m_data[i];

        delete[] m_filename;
        delete[] m_mmap;
        delete[] m_data;
        delete[] m_volumeType;
        delete[] m_channels;
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSourceEx::serialize(stream, manager);

		m_volumeToWorld.serialize(stream);
		m_dataAABB.serialize(stream);
        stream->writeUInt(m_nVolumes);
        for ( uint32_t i = 0; i < m_nVolumes; ++i )
		    stream->writeString(m_filename[i]);
        stream->writeInt(m_defultFloatLookupId);
        stream->writeInt(m_defultSpectrumLookupId);
        stream->writeInt(m_defultVectorLookupId);
	}

	void configure() {
        if ( !m_ready ) {
		    Vector extents(m_dataAABB.getExtents());
		    m_worldToVolume = m_volumeToWorld.inverse();
		    m_worldToGrid = Transform::scale(Vector(
				    (m_res[0] - 1) / extents[0],
				    (m_res[1] - 1) / extents[1],
				    (m_res[2] - 1) / extents[2])
			    ) * Transform::translate(-Vector(m_dataAABB.min)) * m_worldToVolume;
		    m_stepSize = std::numeric_limits<Float>::infinity();
		    for (int i=0; i<3; ++i)
			    m_stepSize = 0.5f * std::min(m_stepSize, extents[i] / (Float) (m_res[i]-1));
		    m_aabb.reset();
		    for (int i=0; i<8; ++i)
			    m_aabb.expandBy(m_volumeToWorld(m_dataAABB.getCorner(i)));

		    /* Precompute cosine and sine lookup tables */
		    for (int i=0; i<255; i++) {
			    Float angle = (float) i * ((float) M_PI / 255.0f);
			    m_cosPhi[i] = std::cos(2.0f * angle);
			    m_sinPhi[i] = std::sin(2.0f * angle);
			    m_cosTheta[i] = std::cos(angle);
			    m_sinTheta[i] = std::sin(angle);
			    m_densityMap[i] = i/255.0f;
		    }
		    m_cosPhi[255] = m_sinPhi[255] = 0;
		    m_cosTheta[255] = m_sinTheta[255] = 0;
		    m_densityMap[255] = 1.0f;

            m_nFloatLookups = m_nFloat3Lookups = 0;
            for ( uint32_t i = 0; i < m_nVolumes; ++i )
                if ( m_channels[i] == 1 )
                    ++m_nFloatLookups;
                else
                    ++m_nFloat3Lookups;

            m_ready = true;
        }
	}

	void loadFromFile(const std::string &filename, uint32_t id) {
        m_filename[id] = filename;
		fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
        
        m_mmap[id] = new MemoryMappedFile(resolved);
        ref<MemoryStream> stream = new MemoryStream(m_mmap[id]->getData(), m_mmap[id]->getSize());
		stream->setByteOrder(Stream::ELittleEndian);

		char header[3];
		stream->read(header, 3);
		if (header[0] != 'V' || header[1] != 'O' || header[2] != 'L')
			Log(EError, "Encountered an invalid volume data file "
				"(incorrect header identifier)");
		uint8_t version;
		stream->read(&version, 1);
		if (version != 3)
			Log(EError, "Encountered an invalid volume data file "
				"(incorrect file version)");
		int type = stream->readInt();

		int xres = stream->readInt(),
			yres = stream->readInt(),
			zres = stream->readInt();
        if ( id == 0 )
		    m_res = Vector3i(xres, yres, zres);
        else if ( xres != m_res.x || yres != m_res.y || zres != m_res.z )
            Log(EError, "Specified volumes are not well-aligned!");
        m_channels[id] = stream->readInt();

		switch (type) {
			case EFloat32:
				if (m_channels[id] != 1 && m_channels[id] != 3)
					Log(EError, "Encountered an unsupported float32 volume data "
						"file (%i channels, only 1 and 3 are supported)",
						m_channels[id]);
				break;
			case EFloat16:
				Log(EError, "Error: float16 volumes are not yet supported!");
			case EUInt8:
				if (m_channels[id] != 1 && m_channels[id] != 3)
					Log(EError, "Encountered an unsupported uint8 volume data "
						"file (%i channels, only 1 and 3 are supported)", m_channels[id]);
				break;
			case EQuantizedDirections:
				if (m_channels[id] != 3)
					Log(EError, "Encountered an unsupported quantized direction "
							"volume data file (%i channels, only 3 are supported)",
							m_channels[id]);
				break;
			default:
				Log(EError, "Encountered a volume data file of unknown type (type=%i, channels=%i)!", type, m_channels);
		}
        m_volumeType[id] = static_cast<EVolumeType>(type);

		Float xmin = stream->readSingle(),
			  ymin = stream->readSingle(),
			  zmin = stream->readSingle();
		Float xmax = stream->readSingle(),
			  ymax = stream->readSingle(),
			  zmax = stream->readSingle();
        if ( id == 0 )
			m_dataAABB = AABB(Point(xmin, ymin, zmin), Point(xmax, ymax, zmax));
        else if ( std::abs(m_dataAABB.min.x - xmin) > Epsilon || std::abs(m_dataAABB.min.y - ymin) > Epsilon || std::abs(m_dataAABB.min.z - zmin) > Epsilon ||
                  std::abs(m_dataAABB.max.x - xmax) > Epsilon || std::abs(m_dataAABB.max.y - ymax) > Epsilon || std::abs(m_dataAABB.max.z - zmax) > Epsilon )
            Log(EError, "Specified volumes are not well-aligned!");

		Log(EDebug, "Mapped \"%s\" into memory: %ix%ix%i (%i channels), %s, %s", 
			resolved.filename().c_str(), m_res.x, m_res.y, m_res.z, m_channels[id],
			memString(m_mmap[id]->getSize()).c_str(), m_dataAABB.toString().c_str());
		m_data[id] = reinterpret_cast<uint8_t *>((reinterpret_cast<float *>(m_mmap[id]->getData())) + 12);
	}

    Float lookupFloat(const Point &p) const {
        if ( m_defultFloatLookupId < 0 )
            Log(EError, "defaultFloatLookupId unspecified!");
        return lookupFloatEx(m_defultFloatLookupId, p);
    }

    Spectrum lookupSpectrum(const Point &p) const {
        if ( m_defultSpectrumLookupId < 0 )
            Log(EError, "defultSpectrumLookupId unspecified!");
        return lookupSpectrumEx(m_defultSpectrumLookupId, p);
    }

    Vector lookupVector(const Point &p) const {
        if ( m_defultVectorLookupId < 0 )
            Log(EError, "defultVectorLookupId unspecified!");
        return lookupVectorEx(m_defultVectorLookupId, p);
    }

	/**
	 * This is needed since Mitsuba might be 
	 * compiled with either single/double precision
	 */
	struct float3 {
		float value[3];

		inline float3() { }

		inline float3(float a, float b, float c) {
			value[0] = a; value[1] = b; value[2] = c;
		}

		inline float3 operator*(Float v) const {
			return float3(value[0]*v, value[1]*v, value[2]*v);
		}
		
		inline float3 operator+(const float3 &f2) const {
			return float3(value[0]+f2.value[0], value[1]+f2.value[1], value[2]+f2.value[2]);
		}

		inline Spectrum toSpectrum() const {
			Spectrum result;
			result.fromLinearRGB(value[0], value[1], value[2]);
			return result;
		}
		
		inline Vector toVector() const {
			return Vector(value[0], value[1], value[2]);
		}
	
		float operator[](int i) const {
			return value[i];
		}

		inline Matrix3x3 tensor() const {
			return Matrix3x3(
				value[0]*value[0], value[0]*value[1], value[0]*value[2],
				value[1]*value[0], value[1]*value[1], value[1]*value[2],
				value[2]*value[0], value[2]*value[1], value[2]*value[2]
			);
		}
	};

	Float lookupFloatEx(uint32_t id, const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( z1 < 0 || z1 + 1 >= m_res.z ) return 0;

        x1 %= m_res.x; if ( x1 < 0 ) x1 += m_res.x;
        y1 %= m_res.y; if ( y1 < 0 ) y1 += m_res.y;

        const int x2 = (x1 + 1) % m_res.x, y2 = (y1 + 1) % m_res.y, z2 = z1 + 1;
		const Float fx = p.x - std::floor(p.x), fy = p.y - std::floor(p.y), fz = p.z - std::floor(p.z);
        const int ix = fx < 0.5f ? x1 : x2;
        const int iy = fy < 0.5f ? y1 : y2;
        const int iz = fz < 0.5f ? z1 : z2;
        const size_t idx = (iz*m_res.y + iy)*m_res.x + ix;

		switch (m_volumeType[id]) {
			case EFloat32: {
				const float *floatData = (float *) m_data[id];
                return floatData[idx];
			}
			case EUInt8: {
                return m_densityMap[m_data[id][idx]];
			}
			default: return 0.0f;
		}
	}

	Spectrum lookupSpectrumEx(uint32_t id, const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( z1 < 0 || z1 + 1 >= m_res.z) return Spectrum(0.0f);

        x1 %= m_res.x; if ( x1 < 0 ) x1 += m_res.x;
        y1 %= m_res.y; if ( y1 < 0 ) y1 += m_res.y;

	    const int x2 = (x1 + 1) % m_res.x, y2 = (y1 + 1) % m_res.y, z2 = z1 + 1;
        const Float fx = p.x - std::floor(p.x), fy = p.y - std::floor(p.y), fz = p.z - std::floor(p.z);
        const int ix = fx < 0.5f ? x1 : x2;
        const int iy = fy < 0.5f ? y1 : y2;
        const int iz = fz < 0.5f ? z1 : z2;
        const size_t idx = (iz*m_res.y + iy)*m_res.x + ix;

		switch (m_volumeType[id]) {
			case EFloat32: {
				const float3 *spectrumData = (float3 *) m_data[id];
                return spectrumData[idx].toSpectrum();
				}
			case EUInt8: {
                return float3(
                    m_densityMap[m_data[id][3*idx+0]],
                    m_densityMap[m_data[id][3*idx+1]],
                    m_densityMap[m_data[id][3*idx+2]]).toSpectrum();
				}
			default: return Spectrum(0.0f);
		}
	}

	Vector lookupVectorEx(uint32_t id, const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( z1 < 0 || z1 + 1 >= m_res.z) return Vector(0.0f);

        x1 %= m_res.x; if ( x1 < 0 ) x1 += m_res.x;
        y1 %= m_res.y; if ( y1 < 0 ) y1 += m_res.y;

	    const int x2 = (x1 + 1) % m_res.x, y2 = (y1 + 1) % m_res.y, z2 = z1 + 1;
        const Float fx = p.x - std::floor(p.x), fy = p.y - std::floor(p.y), fz = p.z - std::floor(p.z);
        const int ix = fx < 0.5f ? x1 : x2;
        const int iy = fy < 0.5f ? y1 : y2;
        const int iz = fz < 0.5f ? z1 : z2;
        const size_t idx = (iz*m_res.y + iy)*m_res.x + ix;

		Vector value;
		switch (m_volumeType[id]) {
			case EFloat32: {
				const float3 *vectorData = (float3 *) m_data[id];
				value = vectorData[idx].toVector();
				}
				break;
			case EQuantizedDirections: {
				value = lookupQuantizedDirection(idx, id);
				}
				break;
			default: return Vector(0.0f);
		}

		if (!value.isZero())
			return normalize(m_volumeToWorld(value));
		else
			return Vector(0.0f);
	}

    bool supportsFloatLookups() const { return m_defultFloatLookupId >= 0; }
	bool supportsSpectrumLookups() const { return m_defultSpectrumLookupId >= 0; }
	bool supportsVectorLookups() const { return m_defultVectorLookupId >= 0; }
	Float getStepSize() const { return m_stepSize; }
	Float getMaximumFloatValue() const { return 1.0f; }
    Float getMaximumFloatValueEx(uint32_t id) const { return 1.0f; }

	std::string toString() const {
		std::ostringstream oss;
		oss << "GridVolumeEx[" << endl
			<< "  res = " << m_res.toString() << "," << endl
			<< "  aabb = " << m_dataAABB.toString() << endl
            << "  def. float = " << m_defultFloatLookupId << endl
            << "  def. spectrum = " << m_defultSpectrumLookupId << endl
            << "  def. vector = " << m_defultVectorLookupId << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS()
protected: 
	FINLINE Vector lookupQuantizedDirection(size_t index, uint32_t id) const {
		uint8_t theta = m_data[id][2*index], phi = m_data[id][2*index+1];
		return Vector(
			m_cosPhi[phi] * m_sinTheta[theta],
			m_sinPhi[phi] * m_sinTheta[theta],
			m_cosTheta[theta]
		);
	}

protected:
    void _reserve(uint32_t size) {
        m_filename = new std::string[size];
        m_mmap = new ref<MemoryMappedFile>[size];
        m_data = new uint8_t*[size];
        m_volumeType = new EVolumeType[size];
        m_channels = new int[size];
    }

    bool m_ready;

	std::string *m_filename;
    ref<MemoryMappedFile> *m_mmap;
	uint8_t **m_data;
	EVolumeType *m_volumeType;
    int *m_channels;

    uint32_t m_nVolumes;
    uint32_t m_nFloatLookups, m_nFloat3Lookups;
    int m_defultFloatLookupId;
    int m_defultSpectrumLookupId;
    int m_defultVectorLookupId;

	Vector3i m_res;
	Transform m_worldToGrid;
	Transform m_worldToVolume;
	Transform m_volumeToWorld;
	Float m_stepSize;
	AABB m_dataAABB;
	
	Float m_cosTheta[256], m_sinTheta[256];
	Float m_cosPhi[256], m_sinPhi[256];
	Float m_densityMap[256];
};

MTS_IMPLEMENT_CLASS_S(GridDataSourceEx_Simple, false, VolumeDataSourceEx);
MTS_EXPORT_PLUGIN(GridDataSourceEx_Simple, "Grid data source 2 (simple)");
MTS_NAMESPACE_END
