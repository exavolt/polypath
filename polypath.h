
#ifndef __POLYPATH_H
#define __POLYPATH_H

#include <cstdlib>
#include <cmath>
#include <vector>
#include <map>


#define POLYPATH_API 


namespace polypath
{

class Vec2
{
public:
	Vec2() {}
	Vec2(const float& __x, const float& __y) : x(__x), y(__y) {}
	Vec2(const Vec2& __v) : x(__v.x), y(__v.y) {}

	Vec2& operator=(const Vec2& __v) { x=__v.x; y=__v.y; return *this; }

	Vec2& set(const float& __x, const float& __y) { x = __x; y = __y; return *this; }

	Vec2 operator+(const Vec2& __v) const { return Vec2(x+__v.x, y+__v.y); }
	Vec2 operator-(const Vec2& __v) const { return Vec2(x-__v.x, y-__v.y); }
	Vec2 operator*(const float& __c) const { return Vec2(x*__c, y*__c); }
	Vec2 operator/(const float& __c) const { return (__c == float(0.0)) ? *this : Vec2(x/__c, y/__c); }

	Vec2& operator+=(const Vec2& __v) { x+=__v.x; y+=__v.y; return *this; }
	Vec2& operator*=(const float& __c) { x*=__c; y*=__c; return *this; }
	Vec2& operator/=(const float& __c) { if (__c != float(0.0)) { x/=__c; y/=__c; } return *this; }

	float dot(const Vec2& __v) const { return x*__v.x + y*__v.y; }
	float det(const Vec2& __v) const { return x*__v.y - y*__v.x; }
	float sqrmag() const     { return dot(*this); }
	float magnitude() const  { return (float)std::sqrt(sqrmag()); };
	float fast_mag() const   { return (float)std::sqrt(sqrmag()); }; //TODO: fast sqrt

	Vec2& normalize()      { *this /= magnitude(); return *this; };

public:
	float x, y;
};


//- PathStats --------------------------

struct PathStats
{
	int   nodes_searched;
	int   nodes_added;
	int   nodes_visited;
	int   nodes_left;
	int   links_source;
	int   links_dest;
	float path_length;
	float path_cost;
};


class Agent
{
public:
	float radius;

public:
	Agent();

	//! Calculates the cost of turning.
	/**
	 *	The default implementation will return zero (no cost).
	 */
	virtual float calculateTurnCost(float deg) const;
};
/*
class AgentDef : public Agent
{
protected:
	float turn_angle_limit_right; // positive
	float turn_angle_limit_left; // negative
	float turn_angle_cost_mult;

protected:
	float calculateTurnCost(float deg) const;
}:
*/

class MapInst;


//- MapDef --------------------------

class POLYPATH_API MapDef
{
public:
	MapDef();
	~MapDef();


	//! Adds new shape
	/**
	 *  @return Returns the identifier number for the shape.
	 *
	 *  @note Note that the instances will NOT be updated.
	 *  Use rebuildInstances when finished adding/removing shapes.
	 *  @see removeShape, clearShapes, rebuildInstances
	 */
	int addShape(const std::vector<Vec2>& vlist);

	//! Removes existing shape from the map.
	/**
	 *  @param shid Shape identifier number.
	 *
	 *  @note Note that the instances will NOT be updated.
	 *  Use rebuildInstances when finished adding/removing shapes.
	 *  @see addShape, clearShapes, rebuildInstances
	 */
	void removeShape(int shid);

	//! Clears all shapes from the map.
	/**
	 *  @note Note that the instances will NOT be updated.
	 *  Use rebuildInstances when finished adding/removing shapes.
	 *  @see addShape, removeShape, rebuildInstances
	 */
	void clearShapes();


	void initInstance(float offset);
	void clearInstances();
	void rebuildInstances();
	bool getInstanceOffsets(std::vector<float>& offs) const;
	//Note: this could be slow and does a lot of memory allocations
	bool getInstanceShapeVertices(float offset, std::vector<std::vector<Vec2> >& sverts) const;

	int computePath(float offset, const Vec2& srcpt, const Vec2& dstpt, 
		std::vector<Vec2>* pathpoints, PathStats* pst) const;
	int computePath(float offset, const Vec2& srcpt, const Vec2& dstpt, 
		std::vector<Vec2>* pathpoints, std::vector<Vec2>* visited, PathStats* pst) const;

	void clear();

private:
	std::map<int, std::vector<Vec2> >  shapes; //!< Blueprint (original shapes)
	std::map<float, MapInst*>   maps;   //!< Map list
	int next_shid;

public:
	bool clamp_max; //!< Use the largest offset found instead of bailing out
};


}; // namespace polypath

#endif // __POLYPATH_H
