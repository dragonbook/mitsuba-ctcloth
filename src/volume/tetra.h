#ifdef __TETRAHEDRON2_H
#error "badness"
#endif

#ifndef __TETRAHEDRON_H
#define __TETRAHEDRON_H

#include <mitsuba/mitsuba.h>
#include <mitsuba/render/trimesh.h>
#include <mitsuba/core/tls.h>


MTS_NAMESPACE_BEGIN


struct Tetrahedron
{
	uint32_t idx[4];

	inline AABB getAABB(const Point *positions) const
	{
		AABB result(positions[idx[0]]);
		result.expandBy(positions[idx[1]]);
		result.expandBy(positions[idx[2]]);
		result.expandBy(positions[idx[3]]);
		return result;
	}

	inline bool inside(const Point *positions, const Point &p, Point4 &bb) const
	{
		Vector3 a(positions[idx[0]] - p);
		Vector3 b(positions[idx[1]] - p);
		Vector3 c(positions[idx[2]] - p);
		Vector3 d(positions[idx[3]] - p);

		Float v = std::abs(dot(a - d, cross(b - d, c - d)));
		Float v0 = std::abs(dot(b, cross(c, d)));
		Float v1 = std::abs(dot(a, cross(c, d)));
		Float v2 = std::abs(dot(a, cross(b, d)));
		Float v3 = std::abs(dot(a, cross(b, c)));

		bb.x = v0/v;
		bb.y = v1/v;
		bb.z = v2/v;
		bb.w = v3/v;

		return bb.x + bb.y + bb.z + bb.w < 1.0f + Epsilon;
	}
};


class TetrahedronMesh
{
public:
	TetrahedronMesh()
	{
		m_vertexCount = m_tetrahedronCount = 0;
		m_vtxPosition = m_vtxTexcoord = NULL;
		m_vtxNormal = NULL; m_vtxTangent = NULL;
		m_tetra = NULL;

		m_list = NULL; m_tree = NULL;
		m_treeSize = 0;
	}

	~TetrahedronMesh()
	{
		if ( m_vtxPosition ) delete[] m_vtxPosition;
		if ( m_vtxTexcoord ) delete[] m_vtxTexcoord;
		if ( m_vtxNormal ) delete[] m_vtxNormal;
		if ( m_vtxTangent ) delete[] m_vtxTangent;
		if ( m_tetra ) delete[] m_tetra;

		if ( m_list ) delete[] m_list;
		if ( m_tree ) delete[] m_tree;
	}

	bool load(const char *fname)
	{
		#define VERIFY_VALUE(x, y)	{ if ( (x) != (y) ) return false; }

		m_aabb = AABB();

		FILE *fin = fopen(fname, "rt");
		if ( fin )
		{
			VERIFY_VALUE(fscanf(fin, "%u %u", &m_vertexCount, &m_tetrahedronCount), 2);

			m_vtxPosition = new Point[m_vertexCount];
			m_vtxTexcoord = new Point[m_vertexCount];
			m_vtxNormal = new Vector[m_vertexCount];
			m_vtxTangent = new TangentSpace[m_vertexCount];
			m_tetra = new Tetrahedron[m_tetrahedronCount];

			uint32_t i;
			for ( i = 0; i < m_vertexCount; ++i )
			{
				VERIFY_VALUE(fscanf(fin, "%f %f %f", &m_vtxPosition[i].x, &m_vtxPosition[i].y, &m_vtxPosition[i].z), 3);
				VERIFY_VALUE(fscanf(fin, "%f %f %f", &m_vtxTexcoord[i].x, &m_vtxTexcoord[i].y, &m_vtxTexcoord[i].z), 3);
				VERIFY_VALUE(fscanf(fin, "%f %f %f", &m_vtxNormal[i].x, &m_vtxNormal[i].y, &m_vtxNormal[i].z), 3);
				VERIFY_VALUE(fscanf(fin, "%f %f %f", &m_vtxTangent[i].dpdu.x, &m_vtxTangent[i].dpdu.y, &m_vtxTangent[i].dpdu.z), 3);
				VERIFY_VALUE(fscanf(fin, "%f %f %f", &m_vtxTangent[i].dpdv.x, &m_vtxTangent[i].dpdv.y, &m_vtxTangent[i].dpdv.z), 3);
			}
			for ( i = 0; i < m_tetrahedronCount; ++i )
				VERIFY_VALUE(fscanf(fin, "%u %u %u %u", &m_tetra[i].idx[0], &m_tetra[i].idx[1], &m_tetra[i].idx[2], &m_tetra[i].idx[3]), 4);

			fclose(fin);

			for ( i = 0; i < m_tetrahedronCount; ++i )
				m_aabb.expandBy(m_tetra[i].getAABB(m_vtxPosition));

			// build BVH
			m_list = new uint32_t[m_tetrahedronCount];
			m_tree = new _node_type[2*m_tetrahedronCount];

			AABB *abl = new AABB[m_tetrahedronCount], aabb;
			for ( i = 0; i < m_tetrahedronCount; ++i )
			{
				m_list[i] = i;
				abl[i] = m_tetra[i].getAABB(m_vtxPosition);
				aabb.expandBy(abl[i]);
			}
			std::random_shuffle(m_list, m_list + m_tetrahedronCount);

			_node_type &root = m_tree[1];
			root.L = 0; root.R = m_tetrahedronCount;
			root.Lch = root.Rch = -1;
			root.aabb = aabb;
			m_treeSize = 1;
			m_treeDepth = build(0, 1, abl);
			SAssert( m_treeDepth <= MAX_TREE_DEPTH );

			delete[] abl;
			return true;
		}
		else
			return false;

		#undef VERIFY_VALUE
	}

	inline AABB getAABB() const
	{
		return m_aabb;
	}

	bool lookupPoint_BruteForce(const Point &p, Point &tex) const
	{
		Point4 bb;
		for ( uint32_t i = 0; i < m_tetrahedronCount; ++i )
			if ( m_tetra[i].inside(m_vtxPosition, p, bb) )
			{
				tex = m_vtxTexcoord[m_tetra[i].idx[0]]*bb.x
					+ m_vtxTexcoord[m_tetra[i].idx[1]]*bb.y
					+ m_vtxTexcoord[m_tetra[i].idx[2]]*bb.z
					+ m_vtxTexcoord[m_tetra[i].idx[3]]*bb.w;
				return true;
			}
		return false;
	}

	bool lookupPoint(const Point &p, Point &tex) const
	{
		uint32_t id;
		Point4 bb;

		//if ( !lookup(p, 1, id, bb) ) return false;
		if ( !lookup(p, id, bb) ) return false;
		tex = m_vtxTexcoord[m_tetra[id].idx[0]]*bb.x
			+ m_vtxTexcoord[m_tetra[id].idx[1]]*bb.y
			+ m_vtxTexcoord[m_tetra[id].idx[2]]*bb.z
			+ m_vtxTexcoord[m_tetra[id].idx[3]]*bb.w;

		return true;
	}

	bool lookupPoint(const Point &p, Point &tex, Vector &normal, TangentSpace &tangent) const
	{
		uint32_t id;
		Point4 bb;

		//if ( !lookup(p, 1, id, bb) ) return false;
		if ( !lookup(p, id, bb) ) return false;
		tex = m_vtxTexcoord[m_tetra[id].idx[0]]*bb.x
			+ m_vtxTexcoord[m_tetra[id].idx[1]]*bb.y
			+ m_vtxTexcoord[m_tetra[id].idx[2]]*bb.z
			+ m_vtxTexcoord[m_tetra[id].idx[3]]*bb.w;
		normal = m_vtxNormal[m_tetra[id].idx[0]]*bb.x
			   + m_vtxNormal[m_tetra[id].idx[1]]*bb.y
			   + m_vtxNormal[m_tetra[id].idx[2]]*bb.z
			   + m_vtxNormal[m_tetra[id].idx[3]]*bb.w;
		tangent.dpdu = m_vtxTangent[m_tetra[id].idx[0]].dpdu*bb.x
					 + m_vtxTangent[m_tetra[id].idx[1]].dpdu*bb.y
					 + m_vtxTangent[m_tetra[id].idx[2]].dpdu*bb.z
					 + m_vtxTangent[m_tetra[id].idx[3]].dpdu*bb.w;
		tangent.dpdv = m_vtxTangent[m_tetra[id].idx[0]].dpdv*bb.x
					 + m_vtxTangent[m_tetra[id].idx[1]].dpdv*bb.y
					 + m_vtxTangent[m_tetra[id].idx[2]].dpdv*bb.z
					 + m_vtxTangent[m_tetra[id].idx[3]].dpdv*bb.w;

		return true;
	}

	inline uint32_t getTreeSize() const
	{
		return m_treeSize;
	}

	inline uint32_t getTreeDepth() const
	{
		return m_treeDepth;
	}

    inline uint32_t getTetrahedronCount() const
    {
        return m_tetrahedronCount;
    }

protected:
	struct _node_type
	{
		_node_type()
		{
			L = R = 0;
			Lch = Rch = -1;
		}

		_node_type(const _node_type& in)
		{
			L = in.L; R = in.R;
			Lch = in.Lch; Rch = in.Rch;
			aabb = in.aabb;
		}

		uint32_t L, R;
		int Lch, Rch;
		AABB aabb;
	};

	//struct _cache_struct
	//{
	//	inline _cache_struct()
	//	{
	//		data = new uint32_t[MAX_TREE_DEPTH + 1];
	//		data[0] = data[1] = 1;
	//	}

	//	~_cache_struct()
	//	{
	//		delete[] data;
	//	}

	//	inline uint32_t &operator[] (size_t idx)
	//	{
	//		return data[idx];
	//	}

	//	uint32_t *data;
	//};

	static const uint32_t INVALID_INDEX = 0xffffffff;
	static const uint32_t MAX_TREE_DEPTH = 100;

	size_t build(uint32_t dep, uint32_t r, const AABB *abl)
	{
        if ( dep == MAX_TREE_DEPTH /*|| m_tree[r].R - m_tree[r].L <= 5*/ )
            return 1;

		int axis = m_tree[r].aabb.getLargestAxis();
		Float pos = 0.5f*(m_tree[r].aabb.min[axis] + m_tree[r].aabb.max[axis]);

		int L = m_tree[r].L, R = m_tree[r].R;
		while ( L < R )
		{
			while ( L < R && pos - abl[m_list[L]].getCenter()[axis] > -Epsilon ) ++L;
			while ( L < R && abl[m_list[R - 1]].getCenter()[axis] - pos > -Epsilon ) --R;
			if ( L < R ) std::swap(m_list[L], m_list[R - 1]);
		}

		if ( L == static_cast<int>(m_tree[r].L) || L == static_cast<int>(m_tree[r].R) )
			return 1;
		
		_node_type nodeL, nodeR;
		int i;

		nodeL.L = m_tree[r].L; nodeL.R = L;
		nodeL.Lch = nodeL.Rch = -1;
		for ( i = m_tree[r].L; i < L; ++i )
			nodeL.aabb.expandBy(abl[m_list[i]]);
		nodeR.L = L; nodeR.R = m_tree[r].R;
		nodeR.Lch = nodeR.Rch = -1;
		for ( i = static_cast<int>(L); i < static_cast<int>(m_tree[r].R); ++i )
			nodeR.aabb.expandBy(abl[m_list[i]]);

		m_tree[++m_treeSize] = nodeL;
		m_tree[++m_treeSize] = nodeR;
		m_tree[r].Lch = static_cast<int>(m_treeSize) - 1;
		m_tree[r].Rch = static_cast<int>(m_treeSize);
		return std::max(build(dep + 1, m_tree[r].Lch, abl), build(dep + 1, m_tree[r].Rch, abl)) + 1;
	}


	uint32_t lookup(const Point &p, uint32_t r, uint32_t &id, Point4 &bb /*, uint32_t *stack = NULL*/) const
	{
		//if ( stack ) *stack = r;
		if ( !m_tree[r].aabb.contains(p) ) return 0;

		uint32_t j;
		if ( m_tree[r].Lch < 0 )
		{
			for ( uint32_t i = m_tree[r].L; i < m_tree[r].R; ++i )
			{
				j = m_list[i];
				if ( m_tetra[j].inside(m_vtxPosition, p, bb) )
				{
					id = j;
#ifndef TETRAHEDRON_MESH_NO_CACHE
                    m_cache_tetra.get() = j;
#endif
					return 1;
				}
			}
		}
		else
		{
			uint32_t ret;

			j = m_tree[r].Lch;
			ret = lookup(p, j, id, bb /*, stack ? stack + 1 : NULL*/);
			if ( ret ) return ret + 1;

			j = m_tree[r].Rch;
			ret = lookup(p, j, id, bb /*, stack ? stack + 1 : NULL*/);
			if ( ret ) return ret + 1;
		}

		return 0;
	}

	inline bool lookup(const Point &p, uint32_t &id, Point4 &bb) const
	{
#ifndef TETRAHEDRON_MESH_NO_CACHE
		uint32_t val = m_cache_tetra.get();
		if ( val <= m_treeSize && m_tetra[val].inside(m_vtxPosition, p, bb) )
		{
			id = val; return true;
		}
#endif

#if 0
		_cache_struct &cache = m_cache_path.get();
		uint32_t i, j;

		i = cache[0];
		if ( j = lookup(p, cache[i], id, bb, &cache[i]) )
		{
			cache[0] = i + j - 1;
			return true;
		}

		for ( i = cache[0] - 1; i > 0; --i )
		{
			j = cache[i];
			cache[i + 1] = m_tree[j].Lch + m_tree[j].Rch - cache[i + 1];
			if ( j = lookup(p, cache[i + 1], id, bb, &cache[i + 1]) )
			{
				cache[0] = i + j;
				return true;
			}
		}

		cache[0] = cache[1] = 1;
		return false;
#endif
		return lookup(p, 1, id, bb);
	}

	AABB m_aabb;

	uint32_t m_vertexCount, m_tetrahedronCount;
	Point *m_vtxPosition, *m_vtxTexcoord;
	Vector *m_vtxNormal;
	TangentSpace *m_vtxTangent;
	Tetrahedron *m_tetra;

	// BVH
	uint32_t *m_list;
	uint32_t m_treeSize, m_treeDepth;
	_node_type *m_tree;

	// per-thread cache
#ifdef TETRAHEDRON_MESH_NO_CACHE
    #pragma message("Tetrahedron mesh compiled without per-thread cache")
#else
	mutable PrimitiveThreadLocal<uint32_t> m_cache_tetra;
    //mutable PrimitiveThreadLocal<uint32_t> m_cache_hit, m_cache_query;
	//mutable PrimitiveThreadLocal<_cache_struct> m_cache_path;
#endif
};


MTS_NAMESPACE_END


#endif
