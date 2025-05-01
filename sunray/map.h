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

// a point on the map
class Point
{
  public:
    #ifdef __linux__
      float px; // cm    // Linux: float
      float py; // cm  
    #else
      short px; // cm   // Arduino: 2 bytes (max +- 327 meter)
      short py; // cm 
    #endif
    Point();
    Point(float ax, float ay); // meter
    float x();  // meter
    float y();  // meter
    void init();
    void setXY(float ax, float ay); // meter
    void assign(Point &fromPoint); 
    long crc();
    bool read(File &file);
    bool write(File &file);
};

// a closed loop of points
class Polygon
{
  public:
    Point *points;    
    short numPoints;    
    Polygon();
    Polygon(short aNumPoints);
    ~Polygon();
    void init();
    bool alloc(short aNumPoints);
    void dealloc();
    void dump();
    long crc();
    void getCenter(Point &pt);
    bool read(File &file);
    bool write(File &file);
};

// a list of polygons
class PolygonList // owns polygons!
{
   public:
     Polygon *polygons;    
     short numPolygons;     
     PolygonList();
     PolygonList(short aNumPolygons);
     ~PolygonList();
     void init();
     bool alloc(short aNumPolygons);
     void dealloc();
     void dump();
     int numPoints();
     long crc();
     bool read(File &file);
     bool write(File &file);
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
    void init();
    void dealloc();
};

// a list of nodes
class NodeList  // owns nodes!
{
  public:
    Node *nodes;    
    short numNodes;     
    NodeList();
    NodeList(short aNumNodes);
    ~NodeList();
    void init();
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
    bool useGPSfixForPosEstimation;  // use GPS fix solution for position estimation?
    bool useGPSfixForDeltaEstimation;  // use GPS fix solution for delta estimation?    
    bool useIMU; // allow using IMU?
    
    // keeps track of the progress in the different point types
    int mowPointsIdx;    // next mowing point in mowing point polygon
    int dockPointsIdx;   // next dock point in docking point polygon
    int freePointsIdx;   // next free point in free point polygon
    int percentCompleted;
    
    Polygon points;      // all points in one list (mowPoints, perimeterPoints, dockPoints) transfered to robot
    Polygon perimeterPoints;  // all perimeter points
    Polygon mowPoints;        // all mowing points
    Polygon dockPoints;       // all docking points
    Polygon freePoints;
    PolygonList exclusions;   // list of exclusion points
    PolygonList obstacles;     
    PolygonList pathFinderObstacles;
    NodeList pathFinderNodes;
    File mapFile;
    int exclusionPointsCount;        
           
    bool shouldDock;  // start docking?
    bool shouldRetryDock; // retry docking?
    bool shouldMow;  // start mowing?       
    
    long mapCRC;  // map data CRC
        
    void begin();    
    void run();    

    // --------mapping ----------------------------------
    // set point coordinate
    bool setPoint(int idx, float x, float y);    
    // set number points for waytype
    bool setWayCount(WayType type, int count);
    // set number points for exclusion 
    bool setExclusionLength(int idx, int len);
    void clearMap();
    void dump();    
    bool load();
    bool save();
    void stressTest();
    void stressTestMapTransfer();
    long calcMapCRC();

    // -------mowing operation--------------------------------------
    bool checkpoint(float x, float y);
    // call to inform mapping to start mowing  
    bool startMowing(float stateX, float stateY);    
    // has mowing completed?
    bool mowingCompleted();    
    // given some point, check and modify it to get obstacle-safe mowing point
    bool findObstacleSafeMowPoint(Point &findPathToPoint);    
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
    bool nextPoint(bool sim,float stateX, float stateY);
    // next point is straight and not a sharp curve?   
    bool nextPointIsStraight();
    // get docking position and orientation
    bool getDockingPos(float &x, float &y, float &delta, int idx = -1);
    
    // ------docking------------------------------------------
    // if docked manually, call this to inform mapping that robot has been docked
    void setIsDocked(bool flag);
    // is robot on docking points and undocking?
    bool isUndocking();
    // is robot on docking points and docking?
    bool isDocking();
    bool isBetweenLastAndNextToLastDockPoint();
    bool isTargetingLastDockPoint();
    bool isTargetingNextToLastDockPoint();    
    // call this to inform mapping to start docking
    bool startDocking(float stateX, float stateY);
    // retry docking (have robot drive back to first docking point)
    bool retryDocking(float stateX, float stateY);
    
    // -----virtual obstacles----------------------------------
    bool addObstacle(float stateX, float stateY);    
    void clearObstacles();
    
    // -----misc-----------------------------------------------
    bool pointIsInsidePolygon( Polygon &polygon, Point &pt);
    bool findPath(Point &src, Point &dst);    
    void generateRandomMap();    
    // check if given point is inside perimeter (and outside exclusions) of current map 
    bool isInsidePerimeterOutsideExclusions(Point &pt);
  private:
    void finishedUploadingMap();
    void checkMemoryErrors();
    bool nextMowPoint(bool sim);
    bool nextDockPoint(bool sim);
    bool nextFreePoint(bool sim);        
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
    void findPathFinderSafeStartPoint(Point &src, Point &dst);
    bool linePolygonIntersectPoint( Point &src, Point &dst, Polygon &poly, Point &sect);
    bool lineLineIntersection(Point &A, Point &B, Point &C, Point &D, Point &pt);
    bool isPointInBoundingBox(Point &pt, Point &A, Point &B);
    int linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly);
    void testIntegerCalcs();
};



#endif
