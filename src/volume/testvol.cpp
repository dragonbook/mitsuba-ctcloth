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
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/fresolver.h>
#include <vector>
#include <map>
#include <Eigen/Dense>


MTS_NAMESPACE_BEGIN

/**
 * This class implements a simple binary exchange format
 * for discretized volume data.
 */
class TestDataSource : public VolumeDataSource {
public:
	enum EVolumeType {
        EDensity = 1,
        EDirection = 2,
        EAlbedo = 3,
        EGloss = 4
	};

	TestDataSource(const Properties &props) 
		: VolumeDataSource(props) {
        m_volfile = props.getString("volfile");
        m_idxfile = props.getString("idxfile");
		m_volumeToWorld = props.getTransform("toWorld", Transform());
		m_stepSizeScale = props.getFloat("stepSizeScale", 1.0f);
        loadBlocks(m_volfile);
        loadIndices(m_idxfile);

        if ( props.hasProperty("adjfile") ) {
            m_adjfile = props.getString("adjfile");
            loadAdjustments(m_adjfile);
        } else {
            m_adjfile = ""; m_nAdjBasis0 = m_nAdjBasis1 = 0;
        }
	}

	TestDataSource(Stream *stream, InstanceManager *manager) 
	: VolumeDataSource(stream, manager) {
        m_volfile = stream->readString();
        m_idxfile = stream->readString();
        m_adjfile = stream->readString();
		m_volumeToWorld = Transform(stream);
		m_stepSizeScale = stream->readFloat();
        loadBlocks(m_volfile);
        loadIndices(m_idxfile);
        if ( m_adjfile != "" )
            loadAdjustments(m_adjfile);
        else
            m_nAdjBasis0 = m_nAdjBasis1 = 0;
        configure();
	}

	virtual ~TestDataSource() {
		for ( size_t i = 0; i < m_blocks.size(); ++i ) {
			if (m_blocks[i] != NULL)
				m_blocks[i]->decRef();
		}
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);

        stream->writeString(m_volfile);
        stream->writeString(m_idxfile);
        stream->writeString(m_adjfile);
		m_volumeToWorld.serialize(stream);
		stream->writeFloat(m_stepSizeScale);
	}
		
	void configure() {
        std::ostringstream oss;
        oss << "Total volumes: " << m_blocks.size();

        m_supportsFloatLookups = ( m_volumeType == EDensity || m_volumeType == EGloss );
	    m_supportsSpectrumLookups = ( m_volumeType == EAlbedo );
	    m_supportsVectorLookups = ( m_volumeType == EDirection );
        m_hasAdjustments = (m_nAdjBasis0 > 0) && (m_nAdjBasis1 > 0);

        for ( size_t i = 0; i < m_blocks.size(); ++i )
            switch ( m_volumeType ) {
            case EDensity:
            case EAlbedo:
            case EGloss:
                if ( !m_blocks[i]->supportsFloatLookups() )
                    Log(EError, "Block %d does not support Float lookups!", i);
                break;

            case EDirection:
                if ( !m_blocks[i]->supportsVectorLookups() )
                    Log(EError, "Block %d does not support Vector lookups!", i);
                break;
            }

        Vector extents(m_dataAABB.getExtents());
		m_worldToVolume = m_volumeToWorld.inverse();
		m_worldToGrid = Transform::scale(Vector(m_res.x/extents.x, m_res.y/extents.y, 1.0f/extents.z))
            *Transform::translate(-Vector(m_dataAABB.min))*m_worldToVolume;

        m_aabb.reset();
        for ( int i = 0; i < 8; ++i )
            m_aabb.expandBy(m_volumeToWorld(m_dataAABB.getCorner(i)));
        oss << "\nData AABB: " << m_dataAABB.toString() << "\nAABB: " << m_aabb.toString();

        m_stepSize = std::numeric_limits<Float>::infinity();
        m_tileCorner.resize(m_res.x*m_res.y);
        m_tileExtents.x = extents.x/static_cast<float>(m_res.x);
        m_tileExtents.y = extents.y/static_cast<float>(m_res.y);
        m_tileExtents.z = extents.z;
        for ( int i = 0; i < m_res.y; ++i )
            for ( int j = 0; j < m_res.x; ++j ) {
                int k = i*m_res.x + j;
                m_stepSize = std::min(m_stepSize, m_blocks[m_tileId[k]]->getStepSize());
                m_tileCorner[k].x = m_dataAABB.min.x + m_tileExtents.x*static_cast<float>(j);
                m_tileCorner[k].y = m_dataAABB.min.y + m_tileExtents.y*static_cast<float>(i);
                m_tileCorner[k].z = m_dataAABB.min.z;
            }
        m_stepSize *= m_stepSizeScale;
        oss << "\nStep size = " << m_stepSize << " (x " << m_stepSizeScale << ")";

        if ( m_hasAdjustments ) {
            oss << "\nAdjustment Tile Resolution: " << m_adjRes.x << " x " << m_adjRes.y;
            oss << "\nNumber of Basis (X): " << m_nAdjBasis0;
            oss << "\nNumber of Basis (Y): " << m_nAdjBasis1;
        }

        oss << std::flush;
        Log(EDebug, oss.str().c_str());
	}

    void loadBlocks(const std::string &filename) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        int type = stream->readInt();
        if ( type != EDensity && type != EDirection && type != EAlbedo && type != EGloss )
            Log(EError, "Unknown volume type: %d", type);

        m_volumeType = static_cast<EVolumeType>(type);

        int nblock = stream->readInt();
        m_blocks.resize(nblock);
        for ( int i = 0; i < nblock; ++i ) {
            std::string fn = stream->readString();
            Vector s(stream);
            Properties props("gridvol_simple");
            props.setString("filename", fn);
            props.setTransform("toWorld", Transform::scale(s));

            Log(EDebug, "Loading volume from [%s]", fn.c_str());
			VolumeDataSource *content = static_cast<VolumeDataSource *> (PluginManager::getInstance()->
					createObject(MTS_CLASS(VolumeDataSource), props));
			content->configure();
            content->incRef();
            m_blocks[i] = content;
        }

        Log(EInfo, "%u blocks loaded", m_blocks.size());
    }

    void loadIndices(const std::string &filename) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        m_res = Vector2i(stream);
        m_dataAABB = AABB(stream);

        m_tileId.resize(m_res.x*m_res.y);
        stream->readIntArray(&m_tileId[0], m_res.x*m_res.y);
        for ( size_t i = 0; i < m_tileId.size(); ++i )
            if ( m_tileId[i] >= static_cast<int>(m_blocks.size()) )
                Log(EError, "Invalid block ID: %u", m_tileId[i]);

        std::vector<Float> transforms;
        transforms.resize(3*m_res.x*m_res.y);
        stream->readFloatArray(&transforms[0], 3*m_res.x*m_res.y);

        // Load per-tile transforms
        m_tileTransform.resize(m_res.x*m_res.y);
        for ( int i = 0; i < m_res.x*m_res.y; ++i )
            m_tileTransform[i] = Transform::translate(Vector(transforms[3*i], transforms[3*i + 1], transforms[3*i + 2]));

        if ( m_volumeType == EAlbedo ) {
            // Load id-to-spectrum mapping for albedo volumes
            int nmaps = stream->readInt();
            m_spectrumMaps.resize(nmaps);
            for ( int i = 0; i < nmaps; ++i ) {
                int nitem = stream->readInt();
                for ( int j = 0; j < nitem; ++j ) {
                    int v = stream->readInt();
                    float r = stream->readFloat();
                    float g = stream->readFloat();
                    float b = stream->readFloat();
                    Spectrum s;
                    s.fromLinearRGB(r, g, b);
                    m_spectrumMaps[i].insert(spectrum_map_t::value_type(v, s));
                }
            }

            m_tileMap.resize(m_res.x*m_res.y);
            stream->readIntArray(&m_tileMap[0], m_res.x*m_res.y);
            for ( int i = 0; i < m_res.x*m_res.y; ++i )
                if ( m_tileMap[i] < 0 || m_tileMap[i] >= static_cast<int>(m_spectrumMaps.size()) )
                    Log(EError, "Invalid tile map: %d", m_tileMap[i]);
        }
        else if ( m_volumeType == EGloss ) {
            // Load id-to-float mapping for gloss volumes
            int nmaps = stream->readInt();
            m_floatMaps.resize(nmaps);
            for ( int i = 0; i < nmaps; ++i ) {
                int nitem = stream->readInt();
                for ( int j = 0; j < nitem; ++j ) {
                    unsigned char v = static_cast<unsigned char>(stream->readInt());
                    float s = stream->readFloat();
                    m_floatMaps[i].insert(float_map_t::value_type(v, s));
                }
            }

            m_tileMap.resize(m_res.x*m_res.y);
            stream->readIntArray(&m_tileMap[0], m_res.x*m_res.y);
            for ( int i = 0; i < m_res.x*m_res.y; ++i )
                if ( m_tileMap[i] < 0 || m_tileMap[i] >= static_cast<int>(m_floatMaps.size()) )
                    Log(EError, "Invalid tile map: %d", m_tileMap[i]);
        }

        Log(EInfo, "%d x %d block info. loaded", m_res.x, m_res.y);
    }

    void loadAdjustments(const std::string &filename) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        m_adjRes = Vector2i(stream);

        m_nAdjBasis0 = stream->readInt();
        m_adjAverage0.resize(m_adjRes.x*m_adjRes.y);
        m_adjBasis0.resize(m_adjRes.x*m_adjRes.y, Eigen::VectorXf(m_nAdjBasis0));

        Float *data_buffer = new Float[(m_nAdjBasis0 + 1)*m_adjRes.x*m_adjRes.y];
        stream->readFloatArray(data_buffer, (m_nAdjBasis0 + 1)*m_adjRes.x*m_adjRes.y);
        for ( int i = 0, j = 0; i < m_adjRes.x*m_adjRes.y; ++i ) {
            m_adjAverage0[i] = data_buffer[j++];
            for ( int k = 0; k < m_nAdjBasis0; ++k )
                m_adjBasis0[i](k) = data_buffer[j++];
        }
        delete[] data_buffer;

        int ncoeff0 = stream->readInt();
        if ( ncoeff0 != m_res.x*(m_res.y - 1) )
            Log(EError, "Adjustment/tile data mismatch!");

        m_adjCoeff0.resize(ncoeff0, Eigen::VectorXf(m_nAdjBasis0));
        data_buffer = new Float[m_nAdjBasis0*ncoeff0];
        stream->readFloatArray(data_buffer, m_nAdjBasis0*ncoeff0);
        for ( int i = 0; i < ncoeff0; ++i )
            for ( int j = 0; j < m_nAdjBasis0; ++j )
                m_adjCoeff0[i](j) = data_buffer[i*m_nAdjBasis0 + j];
        delete[] data_buffer;

        m_nAdjBasis1 = stream->readInt();
        m_adjAverage1.resize(m_adjRes.x*m_adjRes.y);
        m_adjBasis1.resize(m_adjRes.x*m_adjRes.y, Eigen::VectorXf(m_nAdjBasis1));

        data_buffer = new Float[(m_nAdjBasis1 + 1)*m_adjRes.x*m_adjRes.y];
        stream->readFloatArray(data_buffer, (m_nAdjBasis1 + 1)*m_adjRes.x*m_adjRes.y);
        for ( int i = 0, j = 0; i < m_adjRes.x*m_adjRes.y; ++i ) {
            m_adjAverage1[i] = data_buffer[j++];
            for ( int k = 0; k < m_nAdjBasis1; ++k )
                m_adjBasis1[i](k) = data_buffer[j++];
        }
        delete[] data_buffer;

        int ncoeff1 = stream->readInt();
        if ( ncoeff1 != (m_res.x - 1)*m_res.y )
            Log(EError, "Adjustment/tile data mismatch!");

        m_adjCoeff1.resize(ncoeff1, Eigen::VectorXf(m_nAdjBasis1));
        data_buffer = new Float[m_nAdjBasis1*ncoeff1];
        stream->readFloatArray(data_buffer, m_nAdjBasis1*ncoeff1);
        for ( int i = 0; i < ncoeff1; ++i )
            for ( int j = 0; j < m_nAdjBasis1; ++j )
                m_adjCoeff1[i](j) = data_buffer[i*m_nAdjBasis1 + j];
        delete[] data_buffer;

        Log(EInfo, "Adjustment info. loaded");
    }

	Float lookupFloat(const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];
            Point p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);

            switch ( m_volumeType ) {
            case EDensity:
                return m_blocks[tid]->lookupFloat(m_tileTransform[id].transformAffine(p0));
            case EGloss: {
                const int idx = floorToInt(
                    255.0f*m_blocks[tid]->lookupFloat(m_tileTransform[id].transformAffine(p0)) + 0.5f
                );

                const float_map_t &map = m_floatMaps[m_tileMap[id]];
                float_map_t::const_iterator it = map.find(idx);
                if ( it != map.end() )
                    return it->second;
                else {
                    if ( idx )
                        Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                    return 0.0f;
                }
            }
            default:
                Log(EError, "Incompatible volume type.");
                return 0.0f;
            }
        }
        else
            return 0.0f;
	}

	Spectrum lookupSpectrum(const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];
            Point p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);

            if ( m_volumeType == EAlbedo ) {
                const int idx = floorToInt(
                    255.0f*m_blocks[tid]->lookupFloat(m_tileTransform[id].transformAffine(p0)) + 0.5f
                );

                const spectrum_map_t &map = m_spectrumMaps[m_tileMap[id]];
                spectrum_map_t::const_iterator it = map.find(idx);
                if ( it != map.end() )
                    return it->second;
                else {
                    if ( idx )
                        Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                    return Spectrum(0.0f);
                }
            }
            else {
                Log(EError, "Incompatible volume type.");
                return Spectrum(0.0f);
            }
        }
        else
            return Spectrum(0.0f);
	}

	Vector lookupVector(const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];
            Point p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);

            if ( m_volumeType == EDirection )
                return m_blocks[tid]->lookupVector(m_tileTransform[id].transformAffine(p0));
            else {
                Log(EError, "Incompatible volume type.");
                return Vector(0.0f);
            }
        }
        else
            return Vector(0.0f);
	}
	
	bool supportsFloatLookups() const { return m_supportsFloatLookups; }
	bool supportsSpectrumLookups() const { return m_supportsSpectrumLookups; }
	bool supportsVectorLookups() const { return m_supportsVectorLookups; }
	Float getStepSize() const { return m_stepSize; }
	Float getMaximumFloatValue() const { return 1.0f; }

	MTS_DECLARE_CLASS()

protected:
    void adjustPoint(int tx, int ty, Point &p0) const {
        int ax = static_cast<int>(static_cast<float>(m_adjRes.x)*p0.x/m_tileExtents.x);
        int ay = static_cast<int>(static_cast<float>(m_adjRes.y)*p0.y/m_tileExtents.y);
        Float delta = 0.0f;
        int p, q;

        //if ( ax < 0 || ax >= m_adjRes.x || ay < 0 || ay >= m_adjRes.y ) {
        //    Log(EWarn, "Invalid adjust coord: (%d, %d)", ax, ay);
        //    return;
        //}
        ax = std::min(std::max(ax, 0), m_adjRes.x - 1);
        ay = std::min(std::max(ay, 0), m_adjRes.y - 1);

        if ( ty > 0 ) {
            p = (m_adjRes.y - 1 - ay)*m_adjRes.x + ax;
            q = (ty - 1)*m_res.x + tx;
            delta -= m_adjAverage0[p] + m_adjBasis0[p].dot(m_adjCoeff0[q]);
        }

        if ( ty + 1 < m_res.y ) {
            p = ay*m_adjRes.x + ax;
            q = ty*m_res.x + tx;
            delta += m_adjAverage0[p] + m_adjBasis0[p].dot(m_adjCoeff0[q]);
        }

        if ( tx > 0 ) {
            p = ay*m_adjRes.x + m_adjRes.x - 1 - ax;
            q = ty*(m_res.x - 1) + tx - 1;
            delta -= m_adjAverage1[p] + m_adjBasis1[p].dot(m_adjCoeff1[q]);
        }

        if ( tx + 1 < m_res.x ) {
            p = ay*m_adjRes.x + ax;
            q = ty*(m_res.x - 1) + tx;
            delta += m_adjAverage1[p] + m_adjBasis1[p].dot(m_adjCoeff1[q]);
        }

        p0.z -= delta;
    }

    std::string m_idxfile, m_volfile, m_adjfile;
    EVolumeType m_volumeType;

    std::vector<VolumeDataSource*> m_blocks;
    Vector2i m_res;
    std::vector<int> m_tileId;
    std::vector<Transform> m_tileTransform;
    std::vector<Vector> m_tileCorner;

    Vector2i m_adjRes;
    int m_nAdjBasis0, m_nAdjBasis1;
    std::vector<Float> m_adjAverage0;
    std::vector<Float> m_adjAverage1;
    std::vector<Eigen::VectorXf> m_adjBasis0, m_adjCoeff0;
    std::vector<Eigen::VectorXf> m_adjBasis1, m_adjCoeff1;

	Transform m_worldToGrid;
	Transform m_worldToVolume;
	Transform m_volumeToWorld;
	bool m_supportsFloatLookups;
	bool m_supportsSpectrumLookups;
	bool m_supportsVectorLookups;
    AABB m_dataAABB;
	Float m_stepSize, m_stepSizeScale;
    Vector m_tileExtents;
    bool m_hasAdjustments;

    typedef std::map<int, float> float_map_t;
    typedef std::map<int, Spectrum> spectrum_map_t;

    std::vector<float_map_t> m_floatMaps;
    std::vector<spectrum_map_t> m_spectrumMaps;
    std::vector<int> m_tileMap;
};

MTS_IMPLEMENT_CLASS_S(TestDataSource, false, VolumeDataSource);
MTS_EXPORT_PLUGIN(TestDataSource, "Test data source");
MTS_NAMESPACE_END
