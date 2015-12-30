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
#include <mitsuba/core/fstream.h>
#include <mitsuba/core/fresolver.h>
#include <map>
#include <Eigen/Dense>

MTS_NAMESPACE_BEGIN

#define UNIFIED_DENSITY_VALUE       0.5f
#define UNIFIED_DENSITY_THRESHOLD   0.3f


class TestDataSourceEx : public VolumeDataSourceEx {
public:
	enum EVolumeType {
        EDensity = 1,
        EDirection = 2,
        EAlbedo = 3,
        EGloss = 4
	};

	TestDataSourceEx(const Properties &props) 
		: VolumeDataSourceEx(props), m_ready(false) {
        m_volfile[0] = props.getString("volfile_density");
        m_volfile[1] = props.getString("volfile_orientation");
        m_volfile[2] = props.getString("volfile_segment");

        m_idxfile[0] = props.getString("idxfile");
        m_idxfile[1] = props.getString("idxfile_gloss", "");
		m_volumeToWorld = props.getTransform("toWorld", Transform());
		m_stepSizeScale = props.getFloat("stepSizeScale", 1.0f);
        m_unifiedDensity = props.getBoolean("densityUnification", false);

        loadBlockInfo(m_volfile[0], EDensity);
        loadBlockInfo(m_volfile[1], EDirection);
        loadBlockInfo(m_volfile[2], EAlbedo);
        loadBlocks();

        m_tileId = NULL; m_tileTransform = NULL;
        m_tileCorner = NULL;

        loadIndices(m_idxfile[0], EAlbedo);
        if ( m_idxfile[1] != "" )
            loadIndices(m_idxfile[1], EGloss);
        else {
            m_nFloatMaps = 0;
            m_floatMaps = NULL;
            m_tileFloatMap = NULL;
        }

        if ( props.hasProperty("adjfile") ) {
            m_adjfile = props.getString("adjfile");
            loadAdjustments(m_adjfile);
        } else {
            m_adjfile = ""; m_nAdjBasis0 = m_nAdjBasis1 = 0;
        }
	}

	TestDataSourceEx(Stream *stream, InstanceManager *manager) 
	: VolumeDataSourceEx(stream, manager), m_ready(false) {
        m_volfile[0] = stream->readString();
        m_volfile[1] = stream->readString();
        m_volfile[2] = stream->readString();
        m_idxfile[0] = stream->readString();
        m_idxfile[1] = stream->readString();
        m_adjfile = stream->readString();
		m_volumeToWorld = Transform(stream);
		m_stepSizeScale = stream->readFloat();
        m_unifiedDensity = stream->readBool();

        loadBlockInfo(m_volfile[0], EDensity);
        loadBlockInfo(m_volfile[1], EDirection);
        loadBlockInfo(m_volfile[2], EAlbedo);
        loadBlocks();

        m_tileId = NULL; m_tileTransform = NULL;
        m_tileCorner = NULL;

        loadIndices(m_idxfile[0], EAlbedo);
        if ( m_idxfile[1] != "" )
            loadIndices(m_idxfile[1], EGloss);
        else {
            m_nFloatMaps = 0;
            m_floatMaps = NULL;
            m_tileFloatMap = NULL;
        }

        if ( m_adjfile != "" )
            loadAdjustments(m_adjfile);
        else
            m_nAdjBasis0 = m_nAdjBasis1 = 0;

        configure();
	}

	virtual ~TestDataSourceEx() {
		for ( size_t i = 0; i < m_nblocks; ++i ) {
			if (m_blocks[i].get()) m_blocks[i]->decRef();
		}

        delete[] m_tileId; delete[] m_tileTransform; delete[] m_tileCorner;

        delete[] m_spectrumMaps; delete[] m_tileSpectrumMap;
        if ( m_nFloatMaps ) {
            delete[] m_floatMaps; delete[] m_tileFloatMap;
        }

        if ( m_hasAdjustments ) {
            delete[] m_adjAverage0; delete[] m_adjBasis0; delete[] m_adjCoeff0;
            delete[] m_adjAverage1; delete[] m_adjBasis1; delete[] m_adjCoeff1;
        }
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		VolumeDataSource::serialize(stream, manager);

        stream->writeString(m_volfile[0]);
        stream->writeString(m_volfile[1]);
        stream->writeString(m_volfile[2]);
        stream->writeString(m_idxfile[0]);
        stream->writeString(m_idxfile[1]);
        stream->writeString(m_adjfile);
		m_volumeToWorld.serialize(stream);
		stream->writeFloat(m_stepSizeScale);
        stream->writeBool(m_unifiedDensity);
	}
		
	void configure() {
        if ( !m_ready ) {
            std::ostringstream oss;
            oss << "Total volumes: " << m_nblocks;

            m_hasAdjustments = (m_nAdjBasis0 > 0) && (m_nAdjBasis1 > 0);

            Vector extents(m_dataAABB.getExtents());
		    m_worldToVolume = m_volumeToWorld.inverse();
		    m_worldToGrid = Transform::scale(Vector(m_res.x/extents.x, m_res.y/extents.y, 1.0f/extents.z))
                *Transform::translate(-Vector(m_dataAABB.min))*m_worldToVolume;

            m_aabb.reset();
            for ( int i = 0; i < 8; ++i )
                m_aabb.expandBy(m_volumeToWorld(m_dataAABB.getCorner(i)));
            oss << "\nData AABB: " << m_dataAABB.toString() << "\nAABB: " << m_aabb.toString();

            m_stepSize = std::numeric_limits<Float>::infinity();
            m_tileCorner = new Vector[m_res.x*m_res.y];
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

            m_pixelSize.x = static_cast<Float>(m_adjRes.x)/m_tileExtents.x;
            m_pixelSize.y = static_cast<Float>(m_adjRes.y)/m_tileExtents.y;

            oss << std::flush;
            Log(EDebug, oss.str().c_str());

            m_ready = true;
        }
	}

    void loadBlockInfo(const std::string &filename, EVolumeType desiredType) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        int type = stream->readInt();
        if ( type != desiredType )
            Log(EError, "Undesired volume type (%d) for [%s]", type, filename.c_str());

        if ( desiredType == EDensity ) {
            m_nblocks = static_cast<size_t>(stream->readInt());
            m_blockInfo = new t_blockInfo[m_nblocks];
        }
        else if ( m_nblocks != static_cast<size_t>(stream->readInt()) )
            Log(EError, "The block description files specified different number of volumes!");

        for ( size_t i = 0; i < m_nblocks; ++i ) {
            t_blockInfo &bi = m_blockInfo[i];
            bi.fn[static_cast<int>(desiredType) - 1] = stream->readString();
            if ( desiredType == EDensity )
                bi.s = Vector(stream);
            else {
                Vector s0(stream);
                if ( std::abs(bi.s.x - s0.x) > Epsilon ||
                     std::abs(bi.s.y - s0.y) > Epsilon ||
                     std::abs(bi.s.z - s0.z) > Epsilon )
                    Log(EError, "The block description files specified inconsistent scales!");
            }
        }
    }

    void loadBlocks() {
        m_blocks = new ref<VolumeDataSourceEx>[m_nblocks];
        for ( size_t i = 0; i < m_nblocks; ++i ) {
            Properties props("gridvol2_simple");
            props.setInteger("nfiles", 3);
            for ( int j = 0; j < 3; ++j ) {
                std::ostringstream oss;
                oss << "filename" << j;
                props.setString(oss.str(), m_blockInfo[i].fn[j]);
            }
            props.setTransform("toWorld", Transform::scale(m_blockInfo[i].s));

            VolumeDataSourceEx *content = static_cast<VolumeDataSourceEx*> (PluginManager::getInstance()->
				createObject(MTS_CLASS(VolumeDataSourceEx), props));
            content->configure();
            content->incRef();
            m_blocks[i] = content;
        }

        delete[] m_blockInfo;

        if ( m_unifiedDensity )
            Log(EInfo, "%u blocks loaded (with densities unified)", m_nblocks);
        else
            Log(EInfo, "%u blocks loaded", m_nblocks);
    }

    void loadIndices(const std::string &filename, EVolumeType desiredType) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        m_res = Vector2i(stream);
        m_dataAABB = AABB(stream);

        if ( m_tileId == NULL )
            m_tileId = new int[m_res.x*m_res.y];
        stream->readIntArray(m_tileId, m_res.x*m_res.y);
        for ( int i = 0; i < m_res.x*m_res.y; ++i )
            if ( m_tileId[i] >= static_cast<int>(m_nblocks) )
                Log(EError, "Invalid block ID: %u", m_tileId[i]);

        // Load per-tile transforms
        Float *transforms = new Float[3*m_res.x*m_res.y];
        stream->readFloatArray(&transforms[0], 3*m_res.x*m_res.y);

        if ( m_tileTransform == NULL )
            m_tileTransform = new Transform[m_res.x*m_res.y];
        for ( int i = 0; i < m_res.x*m_res.y; ++i )
            m_tileTransform[i] = Transform::translate(Vector(transforms[3*i], transforms[3*i + 1], transforms[3*i + 2]));

        delete[] transforms;

        if ( desiredType == EAlbedo ) {
            // Load id-to-spectrum mapping for albedo volumes
            m_nSpectrumMaps = stream->readInt();
            m_spectrumMaps = new spectrum_map_t[m_nSpectrumMaps];
            for ( size_t i = 0; i < m_nSpectrumMaps; ++i ) {
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

            m_tileSpectrumMap = new int[m_res.x*m_res.y];
            stream->readIntArray(m_tileSpectrumMap, m_res.x*m_res.y);
            for ( int i = 0; i < m_res.x*m_res.y; ++i )
                if ( m_tileSpectrumMap[i] < 0 || m_tileSpectrumMap[i] >= static_cast<int>(m_nSpectrumMaps) )
                    Log(EError, "Invalid tile map: %d", m_tileSpectrumMap[i]);
        }
        else if ( desiredType == EGloss ) {
            // Load id-to-float mapping for gloss volumes
            m_nFloatMaps = stream->readInt();
            m_floatMaps = new float_map_t[m_nFloatMaps];
            for ( size_t i = 0; i < m_nFloatMaps; ++i ) {
                int nitem = stream->readInt();
                for ( int j = 0; j < nitem; ++j ) {
                    int v = stream->readInt();
                    float s = stream->readFloat();
                    m_floatMaps[i].insert(float_map_t::value_type(v, s));
                }
            }

            m_tileFloatMap = new int[m_res.x*m_res.y];
            stream->readIntArray(m_tileFloatMap, m_res.x*m_res.y);
            for ( int i = 0; i < m_res.x*m_res.y; ++i )
                if ( m_tileFloatMap[i] < 0 || m_tileFloatMap[i] >= static_cast<int>(m_nFloatMaps) )
                    Log(EError, "Invalid tile map: %d", m_tileFloatMap[i]);
        }
        else
            Log(EError, "badness");

        Log(EInfo, "%d x %d block info. loaded", m_res.x, m_res.y);
    }

    void loadAdjustments(const std::string &filename) {
        fs::path resolved = Thread::getThread()->getFileResolver()->resolve(filename);
		ref<FileStream> stream = new FileStream(resolved, FileStream::EReadOnly);
		stream->setByteOrder(Stream::ELittleEndian);

        m_adjRes = Vector2i(stream);

        m_nAdjBasis0 = stream->readInt();
        m_adjAverage0 = new Float[m_adjRes.x*m_adjRes.y];
        m_adjBasis0 = new Eigen::VectorXf[m_adjRes.x*m_adjRes.y];
        for ( int i = 0; i < m_adjRes.x*m_adjRes.y; ++i )
            m_adjBasis0[i].resize(m_nAdjBasis0);

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

        m_adjCoeff0 = new Eigen::VectorXf[ncoeff0];
        for ( int i = 0; i < ncoeff0; ++i )
            m_adjCoeff0[i].resize(m_nAdjBasis0);
        data_buffer = new Float[m_nAdjBasis0*ncoeff0];
        stream->readFloatArray(data_buffer, m_nAdjBasis0*ncoeff0);
        for ( int i = 0; i < ncoeff0; ++i )
            for ( int j = 0; j < m_nAdjBasis0; ++j )
                m_adjCoeff0[i](j) = data_buffer[i*m_nAdjBasis0 + j];
        delete[] data_buffer;

        m_nAdjBasis1 = stream->readInt();
        m_adjAverage1 = new Float[m_adjRes.x*m_adjRes.y];
        m_adjBasis1 = new Eigen::VectorXf[m_adjRes.x*m_adjRes.y];
        for ( int i = 0; i < m_adjRes.x*m_adjRes.y; ++i )
            m_adjBasis1[i].resize(m_nAdjBasis1);

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

        m_adjCoeff1 = new Eigen::VectorXf[ncoeff1];
        for ( int i = 0; i < ncoeff1; ++i )
            m_adjCoeff1[i].resize(m_nAdjBasis1);
        data_buffer = new Float[m_nAdjBasis1*ncoeff1];
        stream->readFloatArray(data_buffer, m_nAdjBasis1*ncoeff1);
        for ( int i = 0; i < ncoeff1; ++i )
            for ( int j = 0; j < m_nAdjBasis1; ++j )
                m_adjCoeff1[i](j) = data_buffer[i*m_nAdjBasis1 + j];
        delete[] data_buffer;

        Log(EInfo, "Adjustment info. loaded");
    }

	Float lookupFloatEx(uint32_t _id, const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            Point p0;
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];

            p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);
            p0 = m_tileTransform[id].transformAffine(p0);

            if ( _id == 0 )
                return m_blocks[tid]->lookupFloatEx(0, p0);
            else {
                const int idx = floorToInt(255.0f*m_blocks[tid]->lookupFloatEx(2, p0) + 0.5f);
                const float_map_t &map = m_floatMaps[m_tileFloatMap[id]];
                float_map_t::const_iterator it = map.find(idx);
                if ( it != map.end() )
                    return it->second;
                else {
                    if ( idx ) Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                    return 0.0f;
                }
            }
        }
        else
            return 0.0f;
	}

	Spectrum lookupSpectrumEx(uint32_t _id, const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            Point p0;
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];

            p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);
            p0 = m_tileTransform[id].transformAffine(p0);

            const int idx = floorToInt(
                255.0f*m_blocks[tid]->lookupFloatEx(2, p0) + 0.5f
            );

            const spectrum_map_t &map = m_spectrumMaps[m_tileSpectrumMap[id]];
            spectrum_map_t::const_iterator it = map.find(idx);
            if ( it != map.end() )
                return it->second;
            else {
                if ( idx )
                    Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                return Spectrum(0.0f);
            }
        }
        else
            return Spectrum(0.0f);
	}

	Vector lookupVectorEx(uint32_t _id, const Point &_p) const {
        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            Point p0;
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];
            p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);
            p0 = m_tileTransform[id].transformAffine(p0);

            return m_blocks[tid]->lookupVectorEx(1, p0);
        }
        else
            return Vector(0.0f);
	}

    virtual void lookupBundle(const Point &_p,
        Float *density, Vector *direction, Spectrum *albedo, Float *gloss) const {

        if ( density ) *density = 0.0f;
        if ( direction ) *direction = Vector(0.0f);
        if ( albedo ) *albedo = Spectrum(0.0f);
        if ( gloss ) *gloss = 0.0f;

        const Point p = m_worldToGrid.transformAffine(_p);
		int x1 = floorToInt(p.x), y1 = floorToInt(p.y), z1 = floorToInt(p.z);
        if ( x1 >= 0 && x1 < m_res.x && y1 >= 0 && y1 < m_res.y && z1 == 0 ) {
            Point p0;
            const int id = y1*m_res.x + x1;
            const int tid = m_tileId[id];
            p0 = m_worldToVolume.transformAffine(_p) - m_tileCorner[id];
            if ( m_hasAdjustments ) adjustPoint(x1, y1, p0);
            p0 = m_tileTransform[id].transformAffine(p0);

            bool done = false;
            if ( density ) {
                float d = m_blocks[tid]->lookupFloatEx(0, p0);
                if ( m_unifiedDensity ) d = d > UNIFIED_DENSITY_THRESHOLD ? UNIFIED_DENSITY_VALUE : 0.0f;
                done = ( (*density = d) < Epsilon );
            }
            if ( !done && direction ) {
                *direction = m_blocks[tid]->lookupVectorEx(1, p0);
                done = direction->isZero();
            }

            int idx = -1;
            if ( !done && albedo ) {
                idx = floorToInt(255.0f*m_blocks[tid]->lookupFloatEx(2, p0) + 0.5f);
                const spectrum_map_t &map = m_spectrumMaps[m_tileSpectrumMap[id]];
                spectrum_map_t::const_iterator it = map.find(idx);
                if ( it != map.end() )
                    *albedo = it->second;
                else {
                    if ( idx )
                        Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                    *albedo = Spectrum(0.0f);
                }
                done = albedo->isZero();
            }

            if ( !done && gloss ) {
                if ( idx < 0 ) idx = floorToInt(255.0f*m_blocks[tid]->lookupFloatEx(2, p0) + 0.5f);
                const float_map_t &map = m_floatMaps[m_tileFloatMap[id]];
                float_map_t::const_iterator it = map.find(idx);
                if ( it != map.end() )
                    *gloss = it->second;
                else {
                    if ( idx ) Log(EWarn, "Failed to lookup index %u for block (%d %d)", idx, x1, y1);
                    *gloss = 0.0f;
                }
            }
        }
    }

    Float lookupFloat(const Point &_p) const { return lookupFloatEx(3, _p); }
    Spectrum lookupSpectrum(const Point &_p) const { return lookupSpectrumEx(2, _p); }
    Vector lookupVector(const Point &_p) const { return lookupVectorEx(1, _p); }
	
	bool supportsFloatLookups() const { return true; }
	bool supportsSpectrumLookups() const { return true; }
	bool supportsVectorLookups() const { return true; }
    bool supportsBundleLookups() const { return true; }
	Float getStepSize() const { return m_stepSize; }

	Float getMaximumFloatValue() const {
        return m_unifiedDensity ? UNIFIED_DENSITY_VALUE : 1.0f;
    }

    Float getMaximumFloatValueEx(uint32_t id) const {
        return m_unifiedDensity ? UNIFIED_DENSITY_VALUE : 1.0f;
    }

	MTS_DECLARE_CLASS()

protected:
    void adjustPoint(int tx, int ty, Point &p0) const {
        int ax = static_cast<int>(p0.x*m_pixelSize.x);
        int ay = static_cast<int>(p0.y*m_pixelSize.y);
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

    bool m_ready;

    std::string m_volfile[3];
    std::string m_idxfile[2];
    std::string m_adjfile;
    bool m_unifiedDensity;

    size_t m_nblocks;
    ref<VolumeDataSourceEx> *m_blocks;

    struct t_blockInfo {
        std::string fn[3];
        Vector s;
    };
    t_blockInfo *m_blockInfo;

    Vector2i m_res;
    int *m_tileId;
    Transform *m_tileTransform;
    Vector *m_tileCorner;

    Vector2i m_adjRes;
    int m_nAdjBasis0, m_nAdjBasis1;
    Float *m_adjAverage0;
    Float *m_adjAverage1;
    Eigen::VectorXf *m_adjBasis0, *m_adjCoeff0;
    Eigen::VectorXf *m_adjBasis1, *m_adjCoeff1;

	Transform m_worldToGrid;
	Transform m_worldToVolume;
	Transform m_volumeToWorld;

    AABB m_dataAABB;
	Float m_stepSize, m_stepSizeScale;
    Vector m_tileExtents;
	Vector2 m_pixelSize;
    bool m_hasAdjustments;

    typedef std::map<int, float> float_map_t;
    typedef std::map<int, Spectrum> spectrum_map_t;

    size_t m_nFloatMaps;
    float_map_t *m_floatMaps;

    size_t m_nSpectrumMaps;
    spectrum_map_t *m_spectrumMaps;

    int *m_tileFloatMap, *m_tileSpectrumMap;
};

MTS_IMPLEMENT_CLASS_S(TestDataSourceEx, false, VolumeDataSourceEx);
MTS_EXPORT_PLUGIN(TestDataSourceEx, "Test data source 2");
MTS_NAMESPACE_END
