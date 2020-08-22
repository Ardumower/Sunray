// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  mapping
*/

#ifndef SUNRAY_MAP_H
#define SUNRAY_MAP_H

#include <Arduino.h>


// waypoint type
enum WayType {WAY_PERIMETER, WAY_EXCLUSION, WAY_DOCK, WAY_MOW, WAY_FREE};
typedef enum WayType WayType;


class Point
{
  public:
    float x;
    float y;       
    Point();
    Point(float ax, float ay);
    virtual void init();
    virtual void setXY(float ax, float ay);
    virtual void assign(Point &fromPoint);
};


class Polygon
{
  public:
    Point *points;    
    short numPoints;    
    Polygon();
    Polygon(short aNumPoints);
    virtual ~Polygon();
    virtual void init();
    virtual void alloc(short aNumPoints);
    virtual void dealloc();
    virtual void dump();
};

class PolygonList // owns polygons!
{
   public:
     Polygon *polygons;    
     short numPolygons;     
     PolygonList();
     PolygonList(short aNumPolygons);
     virtual ~PolygonList();
     virtual void init();
     virtual void alloc(short aNumPolygons);
     virtual void dealloc();
     virtual void dump();
     virtual int numPoints();
};

class Node   // nodes just hold references to points and other nodes
{
  public:
    Point *point;
    Node *parent;
    bool opened;
    bool closed;
    float g;
    float h;
    float f;
    Node();
    Node(Point *aPoint, Node *aParentNode);
    virtual void init();
    virtual void dealloc();
};


class NodeList  // owns nodes!
{
  public:
    Node *nodes;    
    short numNodes;     
    NodeList();
    NodeList(short aNumNodes);
    virtual ~NodeList();
    virtual void init();
    virtual void alloc(short aNumNodes);
    virtual void dealloc();    
};



// there are three types of points used as waypoints:
// mowing points:     fixed and transfered by the phone
// docking points:    fixed and transfered by the phone
// free points:       dynamic, connects the above, computed by the Arduino based on the situation 
//                    (obstacles, docking, etc.)
//
// there are additional static points (not used as waypoints but for computing the free points):
// exclusion points:  fixed and transfered by the phone
// perimeter points:  fixed and transfered by the phone



class Map
{
  public:    
    // current waypoint mode (dock, mow, free)
    WayType wayMode;
    
    // the line defined by (lastTargetPoint, targetPoint) is the current line to drive
    Point targetPoint; // target point
    Point lastTargetPoint; // last target point
    //int targetPointIdx; // index of target point    
    bool trackReverse; // get to target in reverse?
    bool trackSlow;    // get to target slowly?
    bool useGPSfloatForPosEstimation;    // use GPS float solution for position estimation?
    bool useGPSfloatForDeltaEstimation;  // use GPS float solution for delta estimation?
    bool useIMU; // allow using IMU?
    
    // keeps track of the progress in the different point types
    int mowPointsIdx;    // next mowing point in mowing point polygon
    int dockPointsIdx;   // next dock point in docking point polygon
    int freePointsIdx;   // next free point in free point polygon
    int percentCompleted; 
    int exclusionPointsCount;        
    
    Polygon points;
    Polygon perimeterPoints;
    Polygon mowPoints;    
    Polygon dockPoints;
    Polygon freePoints;
    PolygonList exclusions;     
    PolygonList obstacles;     
    PolygonList pathFinderObstacles;
    NodeList pathFinderNodes;
           
    bool shouldDock;  // start docking?
    bool shouldMow;  // start mowing?
        
    void begin();    
    void run();    
    // set point coordinate
    bool setPoint(int idx, float x, float y);    
    // set number points for waytype
    bool setWayCount(WayType type, int count);
    // set number points for exclusion 
    bool setExclusionLength(int idx, int len);
    // choose progress (0..100%) in mowing point list
    void setMowingPointPercent(float perc);
    // skip next mowing point
    void skipNextMowingPoint();
    // set last target point
    void setLastTargetPoint(float stateX, float stateY);
    // distance to target waypoint
    float distanceToTargetPoint(float stateX, float stateY);    
    float distanceToLastTargetPoint(float stateX, float stateY);
    // go to next waypoint
    bool nextPoint(bool sim);
    // next point is straight and not a sharp curve?   
    bool nextPointIsStraight();
    // set robot state position to docking position
    void setRobotStatePosToDockingPos(float &x, float &y, float &delta);
    void setIsDocked(bool flag);
    void startDocking(float stateX, float stateY);
    void startMowing(float stateX, float stateY);
    void addObstacle(float stateX, float stateY);
    void dump();    
  private:
    bool findObstacleSafeMowPoint();
    bool nextMowPoint(bool sim);
    bool nextDockPoint(bool sim);
    bool nextFreePoint(bool sim);        
    bool findPath(Point &src, Point &dst);
    float distance(Point &src, Point &dst);        
    float pointsAngle(float x1, float y1, float x2, float y2);
    float scalePI(float v);
    float distancePI(float x, float w);
    float distanceManhattan(Point &pos0, Point &pos1);
    float calcHeuristic(Point &pos0, Point &pos1);
    float scalePIangles(float setAngle, float currAngle);
    bool lineIntersects (Point &p0, Point &p1, Point &p2, Point &p3);        
    bool linePolygonIntersection( Point &src, Point &dst, Polygon &poly);
    float polygonArea(Polygon &poly);
    void polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist);
    int findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx);
    bool pointIsInsidePolygon( Polygon &polygon, float x, float y);
};



#endif
