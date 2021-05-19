// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  mapping
*/

#ifndef SUNRAY_MAP_H
#define SUNRAY_MAP_H

#include <Arduino.h>
#include <SD.h>


// waypoint type
enum WayType {WAY_PERIMETER, WAY_EXCLUSION, WAY_DOCK, WAY_MOW, WAY_FREE};
typedef enum WayType WayType;


class Point
{
  public:
    short px = 0; // cm
    short py = 0; // cm       
    Point() = default;
    Point(float ax, float ay); // meter
    float x() const;  // meter
    float y() const;  // meter
    void setXY(float ax, float ay); // meter
    void assign(Point &fromPoint); 
    long crc() const;
    bool read(File &file);
    bool write(File &file);
};


class Polygon
{
  public:
    Point *points = nullptr;    
    short numPoints = 0;    
    Polygon() = default;
    Polygon(short aNumPoints);
    ~Polygon();
    bool alloc(short aNumPoints);
    void dealloc();
    void dump() const;
    long crc() const;
    bool read(File &file);
    bool write(File &file);
};

class PolygonList // owns polygons!
{
   public:
     Polygon *polygons = nullptr;    
     short numPolygons = 0;     
     PolygonList() = default;
     PolygonList(short aNumPolygons);
     ~PolygonList();
     bool alloc(short aNumPolygons);
     void dealloc();
     void dump() const;
     int numPoints() const;
     long crc() const;
     bool read(File &file);
     bool write(File &file);
};

class Node   // nodes just hold references to points and other nodes
{
  public:
    Point *point = nullptr;
    Node *parent = nullptr;
    bool opened = false;
    bool closed = false;
    float g = 0;
    float h = 0;
    float f = 0;
    Node() = default;
    Node(Point *aPoint, Node *aParentNode);
    void init();
    void dealloc();
};


class NodeList  // owns nodes!
{
  public:
    Node *nodes = nullptr;    
    short numNodes = 0;     
    NodeList() = default;
    NodeList(short aNumNodes);
    ~NodeList();
    bool alloc(short aNumNodes);
    void dealloc();    
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
    
    Polygon points;
    Polygon perimeterPoints;
    Polygon mowPoints;    
    Polygon dockPoints;
    Polygon freePoints;
    PolygonList exclusions;     
    PolygonList obstacles;     
    PolygonList pathFinderObstacles;
    NodeList pathFinderNodes;
    File mapFile;
    int exclusionPointsCount;        
           
    bool shouldDock;  // start docking?
    bool shouldMow;  // start mowing?       
    
    long mapCRC;  // map data CRC
        
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
    // move progress to last mowing point
    void repeatLastMowingPoint();
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
    bool isUndocking();
    bool startDocking(float stateX, float stateY);
    bool startMowing(float stateX, float stateY);
    bool addObstacle(float stateX, float stateY);
    bool mowingCompleted();
    bool findObstacleSafeMowPoint(Point &findPathToPoint);
    void clearObstacles();
    void clearMap();
    void dump();    
    bool load();
    bool save();
    void stressTest();
    long calcMapCRC();
  private:
    void finishedUploadingMap();
    void checkMemoryErrors();
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
    bool polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist);
    int findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx);
    bool pointIsInsidePolygon( Polygon &polygon, Point &pt);
    void findPathFinderSafeStartPoint(Point &src, Point &dst);
    bool linePolygonIntersectPoint( Point &src, Point &dst, Polygon &poly, Point &sect);
    bool lineLineIntersection(Point &A, Point &B, Point &C, Point &D, Point &pt);
    bool isPointInBoundingBox(Point &pt, Point &A, Point &B);
    int linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly);
    void testIntegerCalcs();
};



#endif
