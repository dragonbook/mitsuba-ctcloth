#include <mitsuba/render/volume2.h>
#include <mitsuba/core/mstream.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/properties.h>
#include <mitsuba/core/mmap.h>


MTS_NAMESPACE_BEGIN


class ImageMapVolume : public VolumeDataSourceEx {
public:
	enum EVolumeType {
		EFloat32 = 1,
		EFloat16 = 2,
		EUInt8 = 3,
		EQuantizedDirections = 4
	};


	ImageMapVolume(const Properties &props) : VolumeDataSourceEx(props), m_ready(false)
    {
		m_volumeToWorld = props.getTransform("toWorld", Transform());

        m_densityFile = props.getString("densityFile");
        m_orientationFile = props.getString("orientationFile");

        if ( props.hasProperty("albedo") )
        {
            if ( props.hasProperty("albedoFile") )
                Log(EError, "Albedo value and albedo volume both declared!");
            m_albedo = props.getSpectrum("albedo");
            m_albedoFile = "";
        }
        else
        {
            m_albedoFile = props.getString("albedoFile");
            m_albedo = Spectrum(0.0f);
        }

		m_mapFile = props.getString("mapFile");
	}


	ImageMapVolume(Stream *stream, InstanceManager *manager) 
		: VolumeDataSourceEx(stream, manager), m_ready(false)
    {
		m_volumeToWorld = Transform(stream);

        m_densityFile = stream->readString();
        m_orientationFile = stream->readString();
        m_albedoFile = stream->readString();
        m_albedo = m_albedoFile == "" ? Spectrum(stream) : Spectrum(0.0f);
		m_imageReso = Vector2i(stream);
		m_pixelSize = Vector2(stream);
		m_mapReso = Vector2i(stream);
        m_mapFile = stream->readString();

		configure();
	}


	virtual ~ImageMapVolume() {
        for ( int i = 0; i < m_nVolumes; ++i )
            if ( !m_mmap[i].get() ) delete[] m_data[i];

        delete[] m_mmap;
        delete[] m_data;
        delete[] m_volumeType;
        delete[] m_channels;
		delete[] m_map;
    }


	void serialize(Stream *stream, InstanceManager *manager) const
    {
		VolumeDataSourceEx::serialize(stream, manager);

        m_volumeToWorld.serialize(stream);

        stream->writeString(m_densityFile);
        stream->writeString(m_orientationFile);
        stream->writeString(m_albedoFile);
        if ( m_albedoFile == "" ) m_albedo.serialize(stream);

		m_imageReso.serialize(stream);
		m_pixelSize.serialize(stream);
		m_mapReso.serialize(stream);
		stream->writeString(m_mapFile);
	}


	void configure()
    {
        if ( !m_ready )
        {
            /* Load stuff */
            _reserve( m_albedoFile != "" ? 3 : 2 );
            loadFromFile(m_densityFile, 0);
            loadFromFile(m_orientationFile, 1);
            if ( m_albedoFile != "" )
                loadFromFile(m_albedoFile, 2);

			/* Load map file */
			ref<FileStream> fs = new FileStream(m_mapFile, FileStream::EReadOnly);
			fs->setByteOrder(Stream::ELittleEndian);

			m_imageReso.x = fs->readInt();
			m_imageReso.y = fs->readInt();
			m_mapReso.x = fs->readInt();
			m_mapReso.y = fs->readInt();

			Log(EDebug, "image reso: (%d, %d)", m_imageReso.x, m_imageReso.y);
			Log(EDebug, "map reso: (%d, %d)", m_mapReso.x, m_mapReso.y);

			m_map = new int[m_mapReso.x*m_mapReso.y*2];
			fs->readIntArray(&m_map[0], m_mapReso.x*m_mapReso.y*2);

			/* the coordiantes in map are in range (1,1) to (imageReso.x, imageReso.y) */
			for (int j = 0; j < m_mapReso.y; ++j) {
				for (int i = 0; i < m_mapReso.x; ++i) {
					if (m_map[j*m_mapReso.x + i] <= 0 || m_map[j*m_mapReso.x + i] > m_imageReso.x) {
						Log(EError, "Invalid pixel coordinate in found in map");
					}
					if (m_map[j*m_mapReso.x + i + 1] <= 0 || m_map[j*m_mapReso.x + i + 1] > m_imageReso.y) {
						Log(EError, "Invalid pixel coordinate in found in map");
					}
				}
			}

			/* compute pixel size */
			Vector extents(m_dataAABB.getExtents());
			m_pixelSize = Vector2(extents.x / m_imageReso.x, extents.y / m_imageReso.y);

            /* Compute transforms */
            m_volumeAABB.min.x = -0.5f*static_cast<Float>(m_mapReso.x)*m_pixelSize.x;	// AABB? what does '*blockSize' means?// understood.
            m_volumeAABB.min.y = -0.5f*static_cast<Float>(m_mapReso.y)*m_pixelSize.y;
            m_volumeAABB.min.z = -0.5f*extents.z;

            m_volumeAABB.max.x = -m_volumeAABB.min.x;
            m_volumeAABB.max.y = -m_volumeAABB.min.y;
            m_volumeAABB.max.z = -m_volumeAABB.min.z;

            Vector volumeExtents = m_volumeAABB.getExtents();

            m_worldToVolume = m_volumeToWorld.inverse();
            m_worldToMap =
                Transform::scale(Vector(
                    static_cast<Float>(m_mapReso.x)/volumeExtents.x, 
                    static_cast<Float>(m_mapReso.y)/volumeExtents.y,
                    1.0f/volumeExtents.z
                ))*
                Transform::translate(Vector(
                    -m_volumeAABB.min.x, -m_volumeAABB.min.y, -m_volumeAABB.min.z // min.x < 0, so understood!
                ))*m_worldToVolume;

            m_aabb.reset();
            for ( int i = 0; i < 8; ++i )
                m_aabb.expandBy(m_volumeToWorld(m_volumeAABB.getCorner(i)));

			/* Log aabb */
			std::ostringstream oss;
			oss << "\nData AABB: " << m_volumeAABB.toString() << "\nAABB: " << m_aabb.toString();
			oss << std::flush;
			Log(EDebug, oss.str().c_str());

			/* Compute step size */
			m_stepSize = std::numeric_limits<Float>::infinity();
			for (int i = 0; i<3; ++i)
				m_stepSize = 0.5f * std::min(m_stepSize, extents[i] / (Float)(m_dataReso[i] - 1));

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

            m_ready = true;
        }
	}


	void loadFromFile(const std::string &filename, uint32_t id) {
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
		    m_dataReso = Vector3i(xres, yres, zres);
        else if ( xres != m_dataReso.x || yres != m_dataReso.y || zres != m_dataReso.z )
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

		// Code below will not be compiled, since 0. AABB is set by blockSize. Note: blockSize.x > 0
#if 0
        else if ( std::abs(m_dataAABB.min.x - xmin) > Epsilon || std::abs(m_dataAABB.min.y - ymin) > Epsilon || std::abs(m_dataAABB.min.z - zmin) > Epsilon ||
                  std::abs(m_dataAABB.max.x - xmax) > Epsilon || std::abs(m_dataAABB.max.y - ymax) > Epsilon || std::abs(m_dataAABB.max.z - zmax) > Epsilon )
            Log(EError, "Specified volumes are not well-aligned!");
#endif

		Log(EDebug, "Mapped \"%s\" into memory: %ix%ix%i (%i channels), %s, %s", 
			resolved.filename().c_str(), m_dataReso.x, m_dataReso.y, m_dataReso.z, m_channels[id],
			memString(m_mmap[id]->getSize()).c_str(), m_dataAABB.toString().c_str());
		m_data[id] = reinterpret_cast<uint8_t *>((reinterpret_cast<float *>(m_mmap[id]->getData())) + 12);
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


    Float lookupFloat(const Point &p) const
    {
        int idx = getCoord(p);
        if ( idx < 0 ) return 0.0;

		switch (m_volumeType[0])
        {
		case EFloat32:
            {
			    const float *floatData = (float *) m_data[0];
                return floatData[idx];
            }
		case EUInt8:
            return m_densityMap[m_data[0][idx]];
		default:
            return 0.0f;
		}
    }


    Spectrum lookupSpectrum(const Point &p) const
    {
        int idx = getCoord(p);
        if ( idx < 0 ) return Spectrum(0.0f);

        if ( m_nVolumes < 3 )
            return m_albedo;
        else
		    switch (m_volumeType[2])
            {
			case EFloat32:
                {
				    const float3 *spectrumData = (float3 *) m_data[2];
                    return spectrumData[idx].toSpectrum();
                }
			case EUInt8:
                return float3(
                    m_densityMap[m_data[2][3*idx+0]],
                    m_densityMap[m_data[2][3*idx+1]],
                    m_densityMap[m_data[2][3*idx+2]]
                ).toSpectrum();
			default:
                return Spectrum(0.0f);
		    }
    }


    Vector lookupVector(const Point &p) const
    {
        int idx = getCoord(p);
        if ( idx < 0 ) return Vector(0.0f);

		Vector value;
		switch (m_volumeType[1])
        {
		case EFloat32:
            {
			    const float3 *vectorData = (float3 *) m_data[1];
			    value = vectorData[idx].toVector();
			    break;
            }
		case EQuantizedDirections:
			value = lookupQuantizedDirection(idx);
			break;
		default:
            return Vector(0.0f);
		}

		if (!value.isZero())
			return normalize(m_volumeToWorld(value));
		else
			return Vector(0.0f);
    }


    void lookupBundle(const Point &p, Float *density, Vector *direction,
        Spectrum *albedo, Float *gloss) const
    {
        Assert( gloss == NULL );

        int idx = getCoord(p);
        if ( idx < 0 )
        {
            if ( density ) *density = 0.0f;
            if ( direction ) *direction = Vector(0.0f);
            if ( albedo ) *albedo = Spectrum(0.0f);
            return;
        }

        if ( density )
		    switch (m_volumeType[0])
            {
			case EFloat32:
                {
				    const float *floatData = (float *) m_data[0];
                    *density = floatData[idx];
			    }
                break;
			case EUInt8:
                *density = m_densityMap[m_data[0][idx]];
                break;
			default:
                *density = 0.0f;
		    }

        if ( direction )
        {
		    Vector value;
		    switch (m_volumeType[1])
            {
		    case EFloat32:
                {
			        const float3 *vectorData = (float3 *) m_data[1];
			        value = vectorData[idx].toVector();
                }
                break;
		    case EQuantizedDirections:
			    value = lookupQuantizedDirection(idx);
			    break;
            default:
                value = Vector(0.0f);
		    }

		    if (!value.isZero())
			    *direction = normalize(m_volumeToWorld(value));
		    else
			    *direction = Vector(0.0f);
        }

        if ( albedo )
        {
            if ( m_nVolumes < 3 )
                *albedo = m_albedo;
            else
            {
		        switch (m_volumeType[2]) {
			    case EFloat32:
                    {
				        const float3 *spectrumData = (float3 *) m_data[2];
                        *albedo = spectrumData[idx].toSpectrum();
                    }
                    break;
			    case EUInt8:
                    *albedo = float3(
                        m_densityMap[m_data[2][3*idx+0]],
                        m_densityMap[m_data[2][3*idx+1]],
                        m_densityMap[m_data[2][3*idx+2]]
                    ).toSpectrum();
                    break;
			    default:
                    *albedo = Spectrum(0.0f);
		        }
            }
        }
    }


	Float lookupFloatEx(uint32_t id, const Point &p) const
    {
        return lookupFloat(p);
	}


	Spectrum lookupSpectrumEx(uint32_t id, const Point &p) const
    {
        return lookupSpectrum(p);
	}


	Vector lookupVectorEx(uint32_t id, const Point &p) const
    {
        return lookupVector(p);
	}

    bool supportsFloatLookups() const { return true; }
	bool supportsSpectrumLookups() const { return true; }
	bool supportsVectorLookups() const { return true; }
    bool supportsBundleLookups() const { return true; }
	Float getStepSize() const { return m_stepSize; }
	Float getMaximumFloatValue() const { return 1.0f; }
    Float getMaximumFloatValueEx(uint32_t id) const { return 1.0f; }

	std::string toString() const {
		std::ostringstream oss;
		oss << "ImageMapVolume[" << endl
			<< "  aabb = " << m_volumeAABB.toString() << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS()

protected: 
	FINLINE Vector lookupQuantizedDirection(size_t index) const {
		uint8_t theta = m_data[1][2*index], phi = m_data[1][2*index+1];
		return Vector(
			m_cosPhi[phi] * m_sinTheta[theta],
			m_sinPhi[phi] * m_sinTheta[theta],
			m_cosTheta[theta]
		);
	}

protected:
    inline void _reserve(int size)
    {
        m_nVolumes = size;
        m_mmap = new ref<MemoryMappedFile>[size];
        m_data = new uint8_t*[size];
        m_volumeType = new EVolumeType[size];
        m_channels = new int[size];
    }

	inline int getCoord(const Point &_p) const {
		// compute pixel location in map
		Point p = m_worldToMap.transformAffine(_p);
		int px = floorToInt(p.x), py = floorToInt(p.y);
		if (px < 0 || px >= m_mapReso.x || py < 0 || py >= m_mapReso.y || p.z < 0.0f || p.z > 1.0f)
			return -1;

		// compute pixel location in image according to location in map
		int pcdx = m_map[py*m_mapReso.x + px];
		int pcdy = m_map[py*m_mapReso.x + px + 1];

		// compute point in data grid volume from pixel location in image
		p.x = (static_cast<Float>(pcdx - 1) + p.x - px) / static_cast<Float>(m_imageReso.x)
			* static_cast<Float>(m_dataReso.x);
		p.y = (static_cast<Float>(pcdy - 1) + p.y - py) / static_cast<Float>(m_imageReso.y)
			* static_cast<Float>(m_dataReso.y);
		p.z = p.z * m_dataReso.z;

		int ix = clamp(floorToInt(p.x), 0, m_dataReso.x - 1);
		int iy = clamp(floorToInt(p.y), 0, m_dataReso.y - 1);
		int iz = clamp(floorToInt(p.z), 0, m_dataReso.z - 1);

		return (iz*m_dataReso.y + iy)*m_dataReso.x + ix;
	}


    bool m_ready;

    std::string m_densityFile;
    std::string m_orientationFile;
    std::string m_albedoFile;
    Spectrum m_albedo;

	Vector2i m_imageReso;
    Vector2 m_pixelSize;	// = m_dataAABB.xy / imageReso
	Vector2i m_mapReso;
	std::string m_mapFile;
	int *m_map;
    AABB m_volumeAABB;


    int m_nVolumes;
    ref<MemoryMappedFile> *m_mmap;
	uint8_t **m_data;
	EVolumeType *m_volumeType;
    int *m_channels;

	Vector3i m_dataReso;
	Transform m_volumeToWorld, m_worldToVolume;
    Transform m_worldToMap;
	Float m_stepSize;
	AABB m_dataAABB;
	
	Float m_cosTheta[256], m_sinTheta[256];
	Float m_cosPhi[256], m_sinPhi[256];
	Float m_densityMap[256];
};

MTS_IMPLEMENT_CLASS_S(ImageMapVolume, false, VolumeDataSourceEx);
MTS_EXPORT_PLUGIN(ImageMapVolume, "Image map volume");
MTS_NAMESPACE_END
