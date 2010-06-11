/*
 * Pathfinding on Polygonal Map Demonstration
 * written by Fahrezal Effendi <exavolt@gmail.com>
 * Comments, suggestions, bug-reports are welcome.
 *
 * License
 * -------
 * This software is distributed under the WTFPL.
 *
 * Compiling
 * ---------
 * Requires OpenGL (or MESA) 1.2 or above, GLU 1.2 or above and GLUT.
 *
 * linux-gcc:
 *   g++ -O2 -o demo_gl polypath.cxx demo_gl.cxx -L/usr/X11R6/lib -lGL -lGLU -lglut
 * win32-gcc/mingw:
 *   g++ -O2 -o demo_gl.exe polypath.cxx demo_gl.cxx -L. -lopengl32 -lglu32 -lglut32
 *
 * Other platforms with GCC should have similiar command.
 *
 * References
 * ----------
 * - Amit's Thoughts on Path-Finding and A-Star 
 *   http://theory.stanford.edu/~amitp/GameProgramming/
 * - A* Pathfinding for Beginners by Patrick Leste
 *   http://www.policyalmanac.org/games/aStarTutorial.htm
 * - Core Techniques and Algorithms in Game Programming
 *   http://www.tar.hu/gamealgorithms/
 * - 3D Game Engine Design
 *   by David H. Eberly
 *   http://www.geometrictools.com/
 *
 */

// TODO:
// - winding check
// - limited search; limits: region, distance, node count
// - user data/labeling
// - possible costs: area, turn angle (turn angle limit)
// - OPTIMIZE!!

#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cassert>
#include <limits>
#include <set>
#include <vector>
#include <map>
#include "polypath.h"


namespace polypath
{


//- VID --------------------------

//! Vertex address on a map
class VID
{
public:
	VID() : sid(-1), cid(-1) {}
	VID(short __s, short __c) : sid(__s), cid(__c) {}
	VID(const VID& __v) : sid(__v.sid), cid(__v.cid) {}

	VID& operator=(const VID& __v) { sid = __v.sid; cid = __v.cid; return *this; }

	VID& set(short __s, short __c) { sid = __s; cid = __c; return *this; }

public:
	short sid; //!< Shape ID
	short cid; //!< Corner ID
};

inline bool operator==(const VID& v1, const VID& v2) {
	return v1.sid==v2.sid && v1.cid==v2.cid;
}
inline bool operator!=(const VID& v1, const VID& v2) {
	return !(v1==v2);
}


//- Edge --------------------------

//! Polygon's edge
/**
 *  This structure also contains information about corner that formed 
 *  with another edge that shares first vertex of this edge.
 */
class Edge
{
public:
	Edge() : vdiff(0,0), length(0), vdir(1,0), angabs(0), angrel(0), convex(false) {}
	Edge(const Edge& __e) : vdiff(__e.vdiff), length(__e.length), 
		vdir(__e.vdir), angabs(__e.angabs), angrel(__e.angrel), convex(__e.convex) {}
	Edge(const Vec2& __p1, const Vec2& __p2);

	Edge& operator=(const Edge& __e)
	{
		vdiff  = __e.vdiff;
		length = __e.length;
		vdir   = __e.vdir;
		angabs = __e.angabs;
		angrel = __e.angrel;
		convex = __e.convex;
		return *this;
	}

	void determineConvexity(const Edge& __eprev);
	bool isConvex() const { return convex; }

public:
	Vec2    vdiff;
	float   length;
	Vec2    vdir;    //!< Direction vector
	float   angabs;  //!< Absolute angle (relative to X axis)
	float   angrel;  //!< Angle relative to previous edge
	bool    convex;
};


//- Shape --------------------------

//! Polygon
class Shape
{
public:
	Shape();

	float getRevolutionAngle() const;

	// Atomic
	// Need to call rebuildEdges and recalcBoundingRectangle
	void assignVertices(const std::vector<Vec2>& vlist);

	void rebuildEdges();
	void recalcBoundingRectangle();

public:
	std::vector<Vec2>   verts;
	std::vector<Edge>   edges;
	Vec2    bbmin;
	Vec2    bbmax;
};


//- Link --------------------------

//! Connection between two vertices
struct Link
{
	VID     vtx1;    //!< First vertex Id
	VID     vtx2;    //!< Second vertex Id
	Vec2    vdiff;   //!< The difference of destination from source
	float   length;  //!< Length of this link; distance from source to destination
};


//- Node --------------------------

//! Search node
struct Node
{
	VID   vid;  //!< Vertex for this node
	VID   vpr;  //!< Parent vertex id
	float gval; //!< Accumulated distance from source point
	float hval; //!< Estimated distance to destination point

	Node() : vid(-1,-1), vpr(-1,-1), gval(0.0), hval(0.0) {}
	Node(const Node& __n)
		: vid(__n.vid), vpr(__n.vpr), gval(__n.gval), hval(__n.hval) {}

	Node& operator=(const Node& __n)
	{
		vid  = __n.vid;
		vpr  = __n.vpr;
		gval = __n.gval;
		hval = __n.hval;
		return *this;
	}
};

#ifndef NODE_SEARCH_LIST_AS_VECTOR
struct node_sorter_less_f
{
	bool operator()(const Node& __x, const Node& __y) const {
		return __x.hval+__x.gval < __y.hval+__y.gval;
	}
};
struct node_sorter_less_g
{
	bool operator()(const Node& __x, const Node& __y) const {
		return __x.gval < __y.gval;
	}
};
#endif

//- MapInst --------------------------

class MapInst
{
public:
	MapInst();
	MapInst(float offs);

	void clear();

	bool getShapeVertices(std::vector<std::vector<Vec2> >& sverts) const;

	void computeLinks();
	void buildLinks(const Vec2& pt, int pid, std::vector<Link>& outlinks) const;

	bool findIntersection(const Vec2& vs, const Vec2& vd, bool closest) const;
	size_t findLink(const VID& v1, const VID& v2) const;
	size_t findNode(const VID& v) const;

	int computePath(const Vec2& srcpt, const Vec2& dstpt, 
		std::vector<Vec2>* pathpoints, std::vector<Vec2>* visitedpoints, 
		std::vector<VID>* pathnodes, PathStats* stats) const;

public:
	std::vector<Shape> shapes;  //!< Offseted shape copy
	std::vector<Link>  links;
	std::vector<VID>   nodes;
	float              offset;  //!< Shapes' edge offset

private:
	// Note: although these variables are temporarily used when performing the search,
	// they are placed as member for performance reason.
#ifndef NODE_SEARCH_LIST_AS_VECTOR
	mutable std::set<Node, node_sorter_less_f> _openNodes;
	mutable std::set<Node, node_sorter_less_g> _visitedNodes;
#else
	mutable std::vector<Node> _openNodes;
	mutable std::vector<Node> _visitedNodes;
#endif
};



//! Ray vs Ray intersection test
/*inline*/ bool find_intersection(const Vec2& origin0, const Vec2& dir0,
    const Vec2& origin1, const Vec2& dir1, Vec2& result)
{
	float det = dir1.det(dir0);
	Vec2 diff = origin1 - origin0;
	float sqrlen = dir0.sqrmag();

	if (det*det > float(0.00001)*sqrlen*dir1.sqrmag()) {
	    float idet = float(1.0)/det;
		result.x = (dir1.x * diff.y - dir1.y * diff.x)*idet;
		result.y = (dir0.x * diff.y - dir0.y * diff.x)*idet;
	    return true;
	}

	return false;
}


//- Edge --------------------------

Edge::Edge(const Vec2& __p1, const Vec2& __p2)
{
	vdiff = __p2-__p1;
	length = vdiff.magnitude();
	if (length != float(0.0))
		vdir = vdiff / length;
	angabs = std::atan2(vdiff.y, vdiff.x);
	if (angabs < 0.0)
		angabs += 2.0*double(M_PI);
	convex = false;
}

void Edge::determineConvexity(const Edge& __eprev)
{
	angrel = -std::atan2(__eprev.vdir.y*vdir.x - __eprev.vdir.x*vdir.y, 
		__eprev.vdir.x*vdir.x + __eprev.vdir.y*vdir.y);
	convex = vdiff.det(__eprev.vdiff) >= float(0);
}


//- Shape --------------------------

Shape::Shape() : bbmin(0,0), bbmax(0,0) {}

float Shape::getRevolutionAngle() const
{
	float res = 0.0;
	std::vector<Edge>::const_iterator ied;
	for (ied = edges.begin(); ied != edges.end(); ++ied) {
		res += (*ied).angrel;
	}
	return res;
}

void Shape::assignVertices(const std::vector<Vec2>& vlist)
{
	verts.assign(vlist.begin(), vlist.end());
	edges.clear();
	bbmin.set(0,0);
	bbmax.set(0,0);
}

void Shape::rebuildEdges()
{
	edges.clear();
	if (verts.size() > 1)
	{
		std::vector<Vec2>::const_iterator iptf,iptn;
		for (iptf=verts.begin(), iptn=verts.begin()+1; iptn != verts.end(); ++iptf, ++iptn)
			edges.push_back(Edge(*iptf, *iptn));
		edges.push_back( Edge( verts.back(), verts.front()));

		std::vector<Edge>::iterator ied;
		for (ied = edges.begin(); ied != edges.end(); ++ied) {
			const Edge& eref = ((edges.begin() == ied) ? edges.back() : *(ied - 1));
			(*ied).determineConvexity(eref);
		}
	}
}

void Shape::recalcBoundingRectangle()
{
	if (verts.empty()) {
		bbmin.set(0,0);
		bbmax.set(0,0);
		return;
	}
	bbmin = bbmax = verts.front();
	std::vector<Vec2>::const_iterator ipt;
	for (ipt=verts.begin()+1; ipt != verts.end(); ++ipt) {
		if ((*ipt).x < bbmin.x) bbmin.x = (*ipt).x;
		else if (bbmax.x < (*ipt).x) bbmax.x = (*ipt).x;
		if ((*ipt).y < bbmin.y) bbmin.y = (*ipt).y;
		else if (bbmax.y < (*ipt).y) bbmax.y = (*ipt).y;
	}
}



//- MapInst --------------------------

MapInst::MapInst()
: offset(0.0)
{
}

MapInst::MapInst(float offs)
: offset(offs)
{
}

void MapInst::clear()
{
	shapes.clear();
	links.clear();
	nodes.clear();
}

bool MapInst::getShapeVertices(std::vector<std::vector<Vec2> >& sverts) const
{
	if (shapes.empty())
		return false;
	std::vector<Shape>::const_iterator ish;
	for (ish = shapes.begin(); ish != shapes.end(); ++ish)
		sverts.push_back((*ish).verts);
	return true;
}

size_t MapInst::findLink(const VID& v1, const VID& v2) const
{
	std::vector<Link>::const_iterator iln;
	size_t iil;
	for (iln = links.begin(), iil = 0; iln != links.end(); ++iln, ++iil)
		if ((iln->vtx1 == v1 && iln->vtx2 == v2) || (iln->vtx1 == v2 && iln->vtx2 == v1))
			return iil;
	return std::numeric_limits<size_t>::max();
}
size_t MapInst::findNode(const VID& v) const
{
	std::vector<VID>::const_iterator ino;
	size_t iin;
	for (ino = nodes.begin(), iin = 0; ino != nodes.end(); ++ino, ++iin)
		if ((*ino) == v)
			return iin;
	return std::numeric_limits<size_t>::max();
}

//! Computes visibility graph
void MapInst::computeLinks()
{
	std::vector<Shape>::const_iterator ish0, ish1, ish2;
	std::vector<Vec2>::const_iterator ipt0, ipt1, ipt2;
	std::vector<Edge>::const_iterator ied;
	short iis0, iip0, iis1, iip1, iis2, iip2;
	VID ivsrc, ivdst;
	Vec2 vdiff, isp;
	Link link;
	bool obstd, neighbor;

	nodes.clear();
	links.clear();

	// Early out
	if (shapes.empty())
		return;

	for (ish0 = shapes.begin(), iis0 = 0; ish0 != shapes.end(); ++ish0, ++iis0)
	{
		for (ipt0 = (*ish0).verts.begin(), iip0 = 0; ipt0 != (*ish0).verts.end(); ++ipt0, ++iip0)
		{
			ivsrc.set(iis0, iip0);

			// Test for convex corners
			if (!(*ish0).edges[iip0].isConvex())
				continue;

			for (ish1 = ish0, iis1 = iis0; ish1 != shapes.end(); ++ish1, ++iis1)
			{
				// Iteration starting points
				if (iis0 == iis1) {
					ipt1 = ipt0+1;
					iip1 = iip0+1;
				} else {
					ipt1 = (*ish1).verts.begin();
					iip1 = 0;
				}

				for ( ; ipt1 != (*ish1).verts.end(); ++ipt1, ++iip1)
				{
					ivdst.set(iis1, iip1);

					// Test for convex corners
					if (!(*ish1).edges[iip1].isConvex())
						continue;
					// Find link for inter-shape
					if (iis0 != iis1 && findLink(ivsrc, ivdst) != std::numeric_limits<size_t>::max())
						continue;

					// Calculate the difference between second point and first point
					vdiff = *ipt1 - *ipt0;

					// Test for neighboring points
					neighbor = (iis0 == iis1) && 
						((iip1 - iip0) <= 1 || (iip0 == 0 && iip1 == ((int)(*ish0).verts.size() - 1)));

					// Neighboring vertices: just add the edge to list if not obstructed
					// Otherwise, perform some other tests
					if (!neighbor)
					{
						const Edge& ed0a = ish0->edges[iip0];
						const Edge& ed0b = iip0 == 0 ? ish0->edges.back() : ish0->edges[iip0-1];

						if (iis0 == iis1)
						{
							bool cw1 = vdiff.det(ed0a.vdiff) >= float(0);
							bool cw2 = vdiff.det(ed0b.vdiff) >= float(0);
							// Check whether the link cuts across the shape itself
							bool inner = (ed0b.vdiff.det(ed0a.vdiff) >= float(0) ? (cw1 || cw2) : (cw1 && cw2));
							if (inner)
								continue;
						}

						// Both edges connected to first vertex must be in same side relative to link span
						if ((ed0a.vdiff.det(vdiff) >= float(0)) != (vdiff.det(ed0b.vdiff) >= float(0)))
							continue;

						const Edge& ed1a = ish1->edges[iip1];
						const Edge& ed1b = iip1 == 0 ? ish1->edges.back() : ish1->edges[iip1-1];
						// Both edges connected to second vertex must be in same side relative to link span
						if ((ed1a.vdiff.det(vdiff) >= float(0)) != (vdiff.det(ed1b.vdiff) >= float(0)))
							continue;

						// Reject overlapping links
						if ((ed0a.vdiff.det(vdiff) == float(0)) || (ed1b.vdiff.det(vdiff) == float(0)))
							continue;
					}

					// Test for any intersections

					obstd = false;

					for (ish2 = shapes.begin(), iis2 = 0; !obstd && ish2 != shapes.end(); ++ish2, ++iis2)
					{
						if ((*ish2).verts.size() < 2)
						    continue;

						ipt2 = (*ish2).verts.begin();
						ied = (*ish2).edges.begin();
						for (iip2 = 0; ipt2 != (*ish2).verts.end(); ++ipt2, ++ied, ++iip2)
						{
							if ((iis2 == iis0 && iip2 == iip0) || (iis2 == iis1 && iip2 == iip1))
								continue;

							if (!find_intersection(*ipt0, vdiff, *ipt2, ied->vdiff, isp))
								continue;
							if (isp.x > float(0.00001) && isp.x < float(0.99999) && 
								isp.y > float(0.00001) && isp.y < float(0.99999))
							{
								// Obstructed
								obstd = true;
								break;
							}
						}
					}

					if (!obstd) {
						// Not obstructed
						// Add new link to the list
						link.vtx1   = ivsrc;
						link.vtx2   = ivdst;
						link.vdiff  = vdiff;
						link.length = vdiff.magnitude();
						links.push_back(link);
					}
				}
			}
		}
	}

	// Build node list
	{
		std::vector<Link>::iterator iln;
		for (iln = links.begin(); iln != links.end(); ++iln)
		{
			if (findNode((*iln).vtx1) == std::numeric_limits<size_t>::max()) {
				nodes.push_back((*iln).vtx1);
				continue;
			}
			if (findNode((*iln).vtx2) == std::numeric_limits<size_t>::max()) {
				nodes.push_back((*iln).vtx2);
			}
		}
	}
}

bool MapInst::findIntersection(const Vec2& vs, const Vec2& vd, bool closest) const
{
	std::vector<Shape>::const_iterator ish;
	std::vector<Vec2>::const_iterator ipt;
	std::vector<Edge>::const_iterator ied;
	Vec2 isp, vdiff = vd - vs;
	float lowt = std::numeric_limits<float>::max();
	short shid = -1;
	short edid = -1;
	short iis, iip;

	for (ish = shapes.begin(), iis = 0; ish != shapes.end(); ++ish, ++iis)
	{
		if ((*ish).verts.size() < 2)
		    continue;
		ipt = (*ish).verts.begin();
		ied = (*ish).edges.begin();
		for (iip = 0; ipt != (*ish).verts.end(); ++ipt, ++ied, ++iip)
		{
			if (!find_intersection(vs, vdiff, *ipt, ied->vdiff, isp))
				continue;
			if (isp.x >= float(0.0) && isp.x <= float(1.0) &&
				isp.y >= float(0.0) && isp.y <= float(1.0))
			{
				if (!closest)
					return true;
				if (isp.x < lowt) {
					lowt = isp.x;
					shid = iis;
					edid = iip;
				}
			}
		}
	}

	return shid >= 0;
}

//! Builds links (visibility graph) from a point to vertices on the map
//! Note: This method will append the links found into result variable
void MapInst::buildLinks(const Vec2& pt, int pid, std::vector<Link>& result) const
{
	std::vector<Shape>::const_iterator ish0, ish2;
	std::vector<Vec2>::const_iterator ipt0, ipt2;
	std::vector<Edge>::const_iterator ied;
	short iis0, iip0, iis2, iip2;
	bool obst;
	Vec2 vdiff, isp;
	Link link;

	// Early out
	if (shapes.empty())
		return;

	for (ish0 = shapes.begin(), iis0 = 0; ish0 != shapes.end(); ++ish0, ++iis0)
	{
		if ((*ish0).verts.size() < 2)
			continue;
		for (ipt0 = (*ish0).verts.begin(), iip0 = 0; ipt0 != (*ish0).verts.end(); ++ipt0, ++iip0)
		{
			// Test for convex corners
			if (!(*ish0).edges[iip0].isConvex())
				continue;

			vdiff = pt - *ipt0;

			{
				const Edge& ed0a = ish0->edges[iip0];
				const Edge& ed0b = iip0 == 0 ? ish0->edges.back() : ish0->edges[iip0-1];
				if ((ed0a.vdiff.det(vdiff) >= float(0)) != (vdiff.det(ed0b.vdiff) >= float(0)))
					continue;
				if (ed0a.vdiff.det(vdiff) == float(0))
					continue;
			}

			obst = false;

			for (ish2 = shapes.begin(), iis2 = 0; !obst && ish2 != shapes.end(); ++ish2, ++iis2)
			{
				if ((*ish2).verts.size() < 2)
				    continue;

				ipt2 = (*ish2).verts.begin();
				ied = (*ish2).edges.begin();
				for (iip2 = 0; ipt2 != (*ish2).verts.end(); ++ipt2, ++ied, ++iip2)
				{
					if (iis2 == iis0 && iip2 == iip0)
						continue;
					if (!find_intersection(*ipt0, vdiff, *ipt2, ied->vdiff, isp))
						continue;
					if (isp.x > float(0.00001) && isp.x < float(0.99999) && 
						isp.y > float(0.00001) && isp.y < float(0.99999))
					{
						obst = true;
						break;
					}
				}
			}

			if (!obst) {
				link.vtx1.set(iis0, iip0);
				link.vtx2.set(  -1,  pid);
				link.vdiff  = vdiff;
				link.length = vdiff.magnitude();
				result.push_back(link);
			}
		}
	}
}

// The main path computation routine
// Note: this method will append nodes/points to the output variable
int MapInst::computePath(const Vec2& srcpt, const Vec2& dstpt, 
	std::vector<Vec2>* pathpoints, std::vector<Vec2>* visitedpoints, 
	std::vector<VID>* pathnodes, PathStats* pst) const
{
	if (!pathpoints && !pathnodes)
		return 0;

	PathStats stats;

	memset(&stats, 0, sizeof(PathStats));

	// First of all, check whether destination is visible from source point.
	// If so, there's no need to search the visibility graph.
	if (shapes.empty() || !findIntersection(srcpt, dstpt, false))
	{
		stats.path_length = (dstpt - srcpt).magnitude();
		stats.path_cost   = stats.path_length; // There's no additional cost besides length
		if (pathpoints) {
			pathpoints->push_back(srcpt);
			pathpoints->push_back(dstpt);
		}
		if (pathnodes) {
			pathnodes->push_back(VID(-1,0));
			pathnodes->push_back(VID(-1,1));
		}
		if (pst)
			memcpy(pst, &stats, sizeof(PathStats));
		return 0;
	}

	{
		std::vector<Link> srclinks;
		std::vector<Link> dstlinks;
		Node node, ntmp;
		Vec2 vdiff, vn;
		std::vector<Link>::const_iterator iln;
		VID srcvid(-1, 0); // Id for source point: invalid shape with zero for corner index
		VID dstvid(-1, 1); // Id for destination point: invalid shape with one for corner index

		_openNodes.clear();
		_visitedNodes.clear();

		buildLinks(srcpt, 0, srclinks);
		if (srclinks.empty())
		{
			if (pst)
				memcpy(pst, &stats, sizeof(PathStats));
			return 1;
		}
		stats.links_source = srclinks.size();
		buildLinks(dstpt, 1, dstlinks);
		if (dstlinks.empty())
		{
			if (pst)
				memcpy(pst, &stats, sizeof(PathStats));
			return 2;
		}
		stats.links_dest = dstlinks.size();

		for (iln = srclinks.begin(); iln != srclinks.end(); ++iln) {
			vn = shapes[(*iln).vtx1.sid].verts[(*iln).vtx1.cid];
			ntmp.gval = (*iln).length;
			ntmp.hval = (dstpt - vn).fast_mag();
			ntmp.vid  = (*iln).vtx1;
			ntmp.vpr  = srcvid;
#ifndef NODE_SEARCH_LIST_AS_VECTOR
			_openNodes.insert(ntmp);
#else
			_openNodes.push_back(ntmp);
#endif
			stats.nodes_added++;
		}

		node.gval = 0.0f;
		node.hval = (dstpt - srcpt).fast_mag();
		node.vid  = srcvid;
		node.vpr  = VID(-1,-1); // Invalid parent

		std::vector<Node>::iterator ino, in2;
		bool found = false;
		float pathlen = 0.0f;
#ifndef NODE_SEARCH_LIST_AS_VECTOR
		std::set<Node, node_sorter_less_f>::iterator ion;
		std::set<Node, node_sorter_less_g>::iterator ivn;
#else
		std::vector<Node>::const_iterator ion;
		std::vector<Node>::iterator ivn;
#endif

		while (1)
		{
			// Add the current node to visited list so it will be skipped
#ifndef NODE_SEARCH_LIST_AS_VECTOR
			_visitedNodes.insert(node);
#else
			_visitedNodes.push_back(node);
#endif
			if (visitedpoints)
				visitedpoints->push_back(shapes[node.vid.sid].verts[node.vid.cid]);

			// Check whether current node has connection to destination point
			for (iln = dstlinks.begin(); iln != dstlinks.end(); ++iln)
			{
				if (node.vid == (*iln).vtx1) {
					pathlen = node.gval + (*iln).length;
					found = true;
					break;
				}
			}

			// The current node is connected to destination point, we have the conclusion
			if (found)
				break;

			// No more nodes in the open list
			if (_openNodes.empty())
				break;

			// Take the next node from open list that have smallest (estimated) total cost
#ifndef NODE_SEARCH_LIST_AS_VECTOR
			// The list is always sorted. Just take the front
			node = *(_openNodes.begin()); // no front() for std::set
			_openNodes.erase(_openNodes.begin()); // no pop_front() for std::set
#else
			in2 = _openNodes.begin();
			for (ino = _openNodes.begin()+1; ino != _openNodes.end(); ++ino) {
				stats.nodes_searched++;
				if ((*ino).hval+(*ino).gval < (*in2).hval+(*in2).gval)
					in2 = ino;
			}
			node = *in2;
			_openNodes.erase(in2);
#endif

			// Find other nodes that connected to current node and add them to open list
			for (iln = links.begin(); iln != links.end(); ++iln)
			{
				// Check for connection
				if ((*iln).vtx1 == node.vid)
					ntmp.vid = (*iln).vtx2;
				else if ((*iln).vtx2 == node.vid)
					ntmp.vid = (*iln).vtx1;
				else
					// Not connected, continue with next link
					continue;

				if ((*iln).vtx1 == node.vpr || (*iln).vtx2 == node.vpr)
					continue;

				// Accumulate the cost
				ntmp.gval = (*iln).length + node.gval;
				ntmp.vpr  = node.vid;

				// Check whether the node has been visited
				for (ivn = _visitedNodes.begin(); ivn != _visitedNodes.end(); ++ivn)
					if ((*ivn).vid == ntmp.vid)
						// Found, no need to continue
						break;

				// Found in visited, continue
				if (ivn != _visitedNodes.end())
					continue;

				bool reins = false;

				// Check whether the node already in open list
				for (ion = _openNodes.begin(); ion != _openNodes.end(); ++ion)
				{
					if ((*ion).vid != ntmp.vid)
						continue;

					// Compare g values: if less, change the parent
					// This step is optional
					if (ntmp.gval < (*ion).gval) {
#ifndef NODE_SEARCH_LIST_AS_VECTOR
						// A std::set iterator is constant, so delete and reinsert
						_openNodes.erase(ion);
						ion = _openNodes.end();
						reins = true;
#else
						// Assign new parent and cost value
						(*ion).gval = ntmp.gval;
						(*ion).vpr  = node.vid;
#endif
					}
					// Found, no need to continue
					break;
				}

				// Found in open, continue
				if (ion != _openNodes.end())
					continue;

				// Not found in visited or in open, insert into open list
				vn = shapes[ntmp.vid.sid].verts[ntmp.vid.cid];
				ntmp.hval = (dstpt - vn).fast_mag();
				ntmp.vpr  = node.vid;
#ifndef NODE_SEARCH_LIST_AS_VECTOR
				_openNodes.insert(ntmp);
#else
				_openNodes.push_back(ntmp);
#endif
				if (!reins)
					stats.nodes_added++;
			}
		}

		if (found)
		{
			if (pathpoints)
				pathpoints->push_back(dstpt);
			if (pathnodes)
				pathnodes->push_back(VID(-1,1));
			while (1)
			{
				if (pathpoints) {
					vn = shapes[node.vid.sid].verts[node.vid.cid];
					pathpoints->push_back(vn);
				}
				if (pathnodes)
					pathnodes->push_back(node.vid);
				if (node.vpr == srcvid)
					break;
				for (ivn = _visitedNodes.begin(); ivn != _visitedNodes.end(); ++ivn)
				{
					if ((*ivn).vid == node.vpr) {
						node = *ivn;
						break;
					}
				}
			}
			if (pathpoints)
				pathpoints->push_back(srcpt);
			if (pathnodes)
				pathnodes->push_back(VID(-1,0));
		}

		stats.nodes_left    = _openNodes.size();
		stats.nodes_visited = _visitedNodes.size();
		stats.path_length   = pathlen;
		stats.path_cost     = stats.path_length;

		if (pst)
			memcpy(pst, &stats, sizeof(PathStats));

		return 0;
	}
}


//- AgentDef --------------------------

Agent::Agent()
: radius(0.0)
{
}

float Agent::calculateTurnCost(float deg) const
{
	return 0.0f;
}


//- MapDef --------------------------

MapDef::MapDef()
: clamp_max(false)
, next_shid(1)
{
}

MapDef::~MapDef()
{
	clear();
}

void MapDef::clear()
{
	std::map<float, MapInst*>::iterator imp;
	for (imp = maps.begin(); imp != maps.end(); ++imp)
		delete (*imp).second;
	maps.clear();
	shapes.clear();
}

void MapDef::initInstance(float offset)
{
	std::map<float, MapInst*>::iterator imp;

	if (offset < 0.0f)
		return;

	imp = maps.find(offset);
	if (imp != maps.end())
		return;

	maps[offset] = new MapInst(offset);
	MapInst* map = maps[offset];

	if (offset == float(0.0)) {
		std::map<int, std::vector<Vec2> >::const_iterator ivv;
		for (ivv = shapes.begin(); ivv != shapes.end(); ++ivv) {
			map->shapes.push_back(Shape());
			Shape& rsh = map->shapes.back();
			rsh.assignVertices(ivv->second);
			rsh.rebuildEdges();
			rsh.recalcBoundingRectangle();
		}
	} else {
		std::vector<Shape> tsh;

		{
			std::map<int, std::vector<Vec2> >::const_iterator ivv;
			for (ivv = shapes.begin(); ivv != shapes.end(); ++ivv) {
				tsh.push_back(Shape());
				Shape& rsh = tsh.back();
				rsh.assignVertices(ivv->second);
				rsh.rebuildEdges();
				rsh.recalcBoundingRectangle();
			}
		}

		std::vector<Shape>::const_iterator ish;
		std::vector<Edge>::const_iterator ied;
		std::vector<Vec2>::const_iterator ivx;
		std::vector< std::pair<Vec2,Vec2> > rays;
		std::pair<Vec2,Vec2> ray;
		std::vector< std::pair<Vec2,Vec2> >::const_iterator ira;
		Vec2 isp;
		std::vector<Vec2> npts;

		for (ish = tsh.begin(); ish != tsh.end(); ++ish)
		{
			rays.clear();
			npts.clear();

			for (ied = (*ish).edges.begin(), ivx = (*ish).verts.begin(); 
				ied != (*ish).edges.end(); ++ied, ++ivx) {
				if ((*ied).angrel > (double(M_PI)+double(M_PI_2)) || (*ied).angrel < -double(M_PI_2)) {
					const Edge& eref = (ied == (*ish).edges.begin()) ? (*ish).edges.back() : *(ied-1);
					float angabs = (*ied).angrel*0.5 + eref.angabs;
					Vec2 vdir(std::cos(angabs), std::sin(angabs));
					ray.first  = (*ivx) + Vec2(-vdir.y, vdir.x) * offset;
					ray.second = vdir;
					rays.push_back(ray);
				}
				ray.first  = (*ivx) + Vec2(-(*ied).vdir.y, (*ied).vdir.x) * offset;
				ray.second = (*ied).vdir;
				rays.push_back(ray);
			}

			for (ira = rays.begin(); ira != rays.end(); ++ira) {
				const std::pair<Vec2,Vec2>& rref = (ira == rays.begin()) ? 
					rays.back() : *(ira-1);
				find_intersection(rref.first, rref.second, (*ira).first, (*ira).second, isp);
				npts.push_back((*ira).first+(*ira).second*isp.y);
			}

			map->shapes.push_back(Shape());
			Shape& rsh = map->shapes.back();
			rsh.assignVertices(npts);
			rsh.rebuildEdges();
			rsh.recalcBoundingRectangle();
		}
	}

	map->computeLinks();
}

void MapDef::clearInstances()
{
	std::map<float, MapInst*>::iterator imp;
	for (imp = maps.begin(); imp != maps.end(); ++imp)
		delete (*imp).second;
	maps.clear();
}

void MapDef::rebuildInstances()
{
	if (maps.empty())
		return;

	std::vector<float> offs;
	std::vector<float>::const_iterator iof;

	getInstanceOffsets(offs);
	clearInstances();

	for (iof = offs.begin(); iof != offs.end(); ++iof)
		initInstance(*iof);
}

int MapDef::computePath(float offset, const Vec2& srcpt, const Vec2& dstpt, 
	std::vector<Vec2>* pathpoints, PathStats* pst) const
{
	return computePath(offset, srcpt, dstpt, pathpoints, NULL, pst);
}

int MapDef::computePath(float offset, const Vec2& srcpt, const Vec2& dstpt, 
	std::vector<Vec2>* pathpoints, std::vector<Vec2>* visited, PathStats* pst) const
{
	if (shapes.empty()) {
		if (pst) {
			memset(pst, 0, sizeof(PathStats));
			pst->path_length = (dstpt - srcpt).magnitude();
			pst->path_cost   = pst->path_length; // There's no additional cost besides length
		}
		if (pathpoints) {
			pathpoints->push_back(srcpt);
			pathpoints->push_back(dstpt);
		}

		return 0;

	} else {
		std::map<float, MapInst*>::const_iterator imp;

		for (imp = maps.begin(); imp != maps.end(); ++imp)
			if ((*imp).first >= offset)
				break;

		if (imp == maps.end()) {
			if (!clamp_max)
				return 0;

			imp = maps.end();
			--imp;
		}

		return (*imp).second->computePath(srcpt, dstpt, pathpoints, visited, NULL, pst);
	}
}

bool MapDef::getInstanceOffsets(std::vector<float>& offs) const
{
	if (maps.empty())
		return false;

	std::map<float, MapInst*>::const_iterator imp;
	for (imp = maps.begin(); imp != maps.end(); ++imp)
		offs.push_back((*imp).first);

	return true;
}

int MapDef::addShape(const std::vector<Vec2>& vlist)
{
	shapes[next_shid] = vlist;
	next_shid++;
	return next_shid - 1;
}

void MapDef::removeShape(int shid)
{
	std::map<int, std::vector<Vec2> >::iterator ish;
	ish = shapes.find(shid);
	if (ish != shapes.end())
		shapes.erase(ish);
}

void MapDef::clearShapes()
{
	shapes.clear();
}

bool MapDef::getInstanceShapeVertices(float offset, std::vector<std::vector<Vec2> >& sverts) const
{
	std::map<float, MapInst*>::const_iterator imp;
	imp = maps.find(offset);
	return (imp == maps.end()) ? false : (*imp).second->getShapeVertices(sverts);
}


}; // namespace polypath
