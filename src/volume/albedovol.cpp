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
#include <mitsuba/core/mstream.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/mmap.h>


MTS_NAMESPACE_BEGIN

class AlbedoVolume : public VolumeDataSource {
public:
	enum EVolumeType {
		EFloat32 = 1,
		EFloat16 = 2,
		EUInt8 = 3,
		EQuantizedDirections = 4
	};

	AlbedoVolume(const Properties &props) 
		: VolumeDataSource(props) {
		m_volumeToWorld = props.getTransform("toWorld", Transform());

		if (props.hasProperty("min") && props.hasProperty("max")) {
			/* Optionally allow to use an AABB other than 
			   the one specified by the grid file */
			m_dataAABB.min = props.getPoint("min");
			m_dataAABB.max = props.getPoint("max");
		}

        m_clr0 = props.getSpectrum("c0");
        m_clr1 = props.getSpectrum("c1");
        m_split = props.getInteger("split");

		loadFromFile(props.getString("filename"));
	}

	AlbedoVolume(Stream *stream, InstanceManager *manager) 
			: VolumeDataSource(stream, manager) {
		m_volumeToWorld = Transform(stream);
		m_dataAABB = AABB(stream);
        m_clr0 = Spectrum(stream);
        m_clr1 = Spectrum(stream);
        m_split = stream->readInt();
		loadFromFile(stream->readString());
		configure();
	}

	virtual ~AlbedoVolume() {
		if (!m_mmap) delete[] m_data;
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);

		m_volumeToWorld.serialize(stream);
		m_dataAABB.serialize(stream);
        m_clr0.serialize(stream);
        m_clr1.serialize(stream);
        stream->writeInt(m_split);
		stream->writeString(m_filename);
	}

	void configure() {
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
	}

	void loadFromFile(const std::string &filename) {
		m_filename = filename;
		fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		m_mmap = new MemoryMappedFile(resolved);
		ref<MemoryStream> stream = new MemoryStream(m_mmap->getData(), m_mmap->getSize());
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
		m_res = Vector3i(xres, yres, zres);
		int channels = stream->readInt();

		switch (type) {
			case EUInt8:
				if (channels != 1)
					Log(EError, "Encountered an unsupported uint8 volume data "
						"file (%i channels, only 1 are supported)", channels);
				break;
			default:
				Log(EError, "Encountered a volume data file of unknown type (type=%i, channels=%i)!", type, channels);
		}

		if (!m_dataAABB.isValid()) {
			Float xmin = stream->readSingle(),
				  ymin = stream->readSingle(),
				  zmin = stream->readSingle();
			Float xmax = stream->readSingle(),
				  ymax = stream->readSingle(),
				  zmax = stream->readSingle();
			m_dataAABB = AABB(Point(xmin, ymin, zmin), Point(xmax, ymax, zmax));
		}

		Log(EDebug, "Mapped \"%s\" into memory: %ix%ix%i (%i channels), %s, %s", 
			resolved.filename().c_str(), m_res.x, m_res.y, m_res.z, channels,
			memString(m_mmap->getSize()).c_str(), m_dataAABB.toString().c_str());
		//m_data = (uint8_t *) (((float *) m_mmap->getData()) + 12);
        m_data = reinterpret_cast<uint8_t *>((reinterpret_cast<float *>(m_mmap->getData())) + 12);
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

	Float lookupFloat(const Point &_p) const {
        return 0.0f;
	}

	Spectrum lookupSpectrum(const Point &_p) const {
		const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( z1 < 0 || z1 + 1 >= m_res.z ) return Spectrum(0.0f);

        x1 %= m_res.x; if ( x1 < 0 ) x1 += m_res.x;
        y1 %= m_res.y; if ( y1 < 0 ) y1 += m_res.y;

        const int x2 = (x1 + 1) % m_res.x, y2 = (y1 + 1) % m_res.y, z2 = z1 + 1;
		const Float fx = p.x - std::floor(p.x), fy = p.y - std::floor(p.y), fz = p.z - std::floor(p.z);
        const int ix = fx < 0.5f ? x1 : x2;
        const int iy = fy < 0.5f ? y1 : y2;
        const int iz = fz < 0.5f ? z1 : z2;


        return static_cast<int>(m_data[(iz*m_res.y + iy)*m_res.x + ix]) <= m_split ? m_clr0 : m_clr1;
	}

	Vector lookupVector(const Point &_p) const {
        return Vector(0.0f);
	}

	bool supportsFloatLookups() const { return false; }
	bool supportsSpectrumLookups() const { return true; }
	bool supportsVectorLookups() const { return false; }
	Float getStepSize() const { return m_stepSize; }
	Float getMaximumFloatValue() const { return 1.0f; }

	std::string toString() const {
		return "";
	}

	MTS_DECLARE_CLASS()

protected:
	std::string m_filename;
	uint8_t *m_data;
	Vector3i m_res;
	Transform m_worldToGrid;
	Transform m_worldToVolume;
	Transform m_volumeToWorld;
	Float m_stepSize;
	AABB m_dataAABB;
	ref<MemoryMappedFile> m_mmap;

    Spectrum m_clr0, m_clr1;	
    int m_split;
};

MTS_IMPLEMENT_CLASS_S(AlbedoVolume, false, VolumeDataSource);
MTS_EXPORT_PLUGIN(AlbedoVolume, "Albedo data source");
MTS_NAMESPACE_END
