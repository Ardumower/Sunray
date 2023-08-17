// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  mapping
*/

#ifndef SUNRAY_MAP_H // Preprocessor guard to prevent multiple inclusions of this header file
#define SUNRAY_MAP_H // Define the preprocessor symbol for this header file

#include <Arduino.h> // Include Arduino core library
#include <SD.h>      // Include SD card library

// Define enumeration for waypoint types
enum WayType {WAY_PERIMETER, WAY_EXCLUSION, WAY_DOCK, WAY_MOW, WAY_FREE};
typedef enum WayType WayType; // Define a type alias for the WayType enumeration

// Class definition for a Point
class Point
{
  public:
    short px; // x-coordinate in centimeters
    short py; // y-coordinate in centimeters
    Point(); // Default constructor
    Point(float ax, float ay); // Constructor with x and y parameters in meters
    float x();  // Method to get x-coordinate in meters
    float y();  // Method to get y-coordinate in meters
    void init(); // Initialization method
    void setXY(float ax, float ay); // Method to set x and y coordinates in meters
    void assign(Point &fromPoint); // Method to assign values from another Point object
    long crc(); // Method to calculate CRC (Cyclic Redundancy Check)
    bool read(File &file); // Method to read Point from a file
    bool write(File &file); // Method to write Point to a file
};

// Class definition for a Polygon
class Polygon
{
  public:
    Point *points;    // Pointer to an array of Points
    short numPoints;  // Number of Points in the Polygon
    Polygon(); // Default constructor
    Polygon(short aNumPoints); // Constructor with number of points parameter
    ~Polygon(); // Destructor
    void init(); // Initialization method
    bool alloc(short aNumPoints); // Method to allocate memory for points
    void dealloc(); // Method to deallocate memory for points
    void dump(); // Method to dump (print) the Polygon data
    long crc(); // Method to calculate CRC (Cyclic Redundancy Check)
    bool read(File &file); // Method to read Polygon from a file
    bool write(File &file); // Method to write Polygon to a file
};

// Class definition for a PolygonList, which owns polygons
class PolygonList
{
   public:
     Polygon *polygons;    // Pointer to an array of Polygons
     short numPolygons;    // Number of Polygons in the list
     PolygonList(); // Default constructor
     PolygonList(short aNumPolygons); // Constructor with number of polygons parameter
     ~PolygonList(); // Destructor
     void init(); // Initialization method
     bool alloc(short aNumPolygons); // Method to allocate memory for polygons
     void dealloc(); // Method to deallocate memory for polygons
     void dump(); // Method to dump (print) the PolygonList data
     int numPoints(); // Method to get the total number of points in all polygons
     long crc(); // Method to calculate CRC (Cyclic Redundancy Check)
     bool read(File &file); // Method to read PolygonList from a file
     bool write(File &file); // Method to write PolygonList to a file
};

// Class definition for a Node, which holds references to points and other nodes
class Node
{
  public:
    Point *point; // Pointer to a Point object
    Node *parent; // Pointer to a parent Node
    bool opened; // Flag to indicate if the node is opened
    bool closed; // Flag to indicate if the node is closed
    float g; // Cost from the start node to this node
    float h; // Heuristic cost from this node to the goal
    float f; // Total cost (g + h)
    Node(); // Default constructor
    Node(Point *aPoint, Node *aParentNode); // Constructor with Point and parent Node parameters
    void init(); // Initialization method
    void dealloc(); // Method to deallocate resources
};

// Class definition for a NodeList, which owns nodes
class NodeList
{
  public:
    Node *nodes;    // Pointer to an array of Nodes
    short numNodes; // Number of Nodes in the list
    NodeList(); // Default constructor
    NodeList(short aNumNodes); // Constructor with number of nodes parameter
    ~NodeList(); // Destructor
    void init(); // Initialization method
    bool alloc(short aNumNodes); // Method to allocate memory for nodes
    void dealloc(); // Method to deallocate memory for nodes
};

// Comment describing three types of points used as waypoints and additional static points
// mowing points:     fixed and transferred by the phone
// docking points:    fixed and transferred by the phone
// free points:       dynamic, connects the above, computed by the Arduino based on the situation 
//                    (obstacles, docking, etc.)
// exclusion points:  fixed and transferred by the phone
// perimeter points:  fixed and transferred by the phone

// Class definition for a Map
class Map
{
  public:    
    WayType wayMode; // Current waypoint mode (dock, mow, free)
    
    // The line defined by (lastTargetPoint, targetPoint) is the current line to drive
    Point targetPoint; // Target point
    Point lastTargetPoint; // Last target point
    // int targetPointIdx; // Index of target point (commented out)
    bool trackReverse; // Flag to indicate if the target should be reached in reverse
    bool trackSlow;    // Flag to indicate if the target should be reached slowly
    bool useGPSfloatForPosEstimation;    // Flag to use GPS float solution for position estimation
    bool useGPSfloatForDeltaEstimation;  // Flag to use GPS float solution for delta estimation
    bool useIMU; // Flag to allow using IMU (Inertial Measurement Unit)
    
    // Variables to keep track of the progress in the different point types
    int mowPointsIdx;    // Next mowing point in mowing point polygon
    int dockPointsIdx;   // Next dock point in docking point polygon
    int freePointsIdx;   // Next free point in free point polygon
    int percentCompleted; // Percentage of completion
    
    Polygon points; // General points
    Polygon perimeterPoints; // Perimeter points
    Polygon mowPoints; // Mowing points
    Polygon dockPoints; // Docking points
    Polygon freePoints; // Free points
    PolygonList exclusions; // Exclusion polygons
    PolygonList obstacles; // Obstacle polygons
    PolygonList pathFinderObstacles; // Pathfinding obstacle polygons
    NodeList pathFinderNodes; // Pathfinding nodes
    File mapFile; // File object for the map
    int exclusionPointsCount; // Count of exclusion points
        
    bool shouldDock;  // Flag to start docking
    bool shouldMow;  // Flag to start mowing
    
    long mapCRC;  // Map data CRC (Cyclic Redundundancy Check)
        
    void begin(); // Method to begin map operations
    void run(); // Method to run map operations
    bool setPoint(int idx, float x, float y); // Method to set point coordinate
    bool setWayCount(WayType type, int count); // Method to set number of points for a waypoint type
    bool setExclusionLength(int idx, int len); // Method to set number of points for exclusion
    void setMowingPointPercent(float perc); // Method to choose progress (0..100%) in mowing point list
    void skipNextMowingPoint(); // Method to skip next mowing point
    void repeatLastMowingPoint(); // Method to move progress to last mowing point
    void setLastTargetPoint(float stateX, float stateY); // Method to set last target point
    float distanceToTargetPoint(float stateX, float stateY); // Method to calculate distance to target waypoint
    float distanceToLastTargetPoint(float stateX, float stateY); // Method to calculate distance to last target point
    bool nextPoint(bool sim); // Method to go to next waypoint
    bool nextPointIsStraight(); // Method to check if next point is straight and not a sharp curve
    void setRobotStatePosToDockingPos(float &x, float &y, float &delta); // Method to set robot state position to docking position
    void setIsDocked(bool flag); // Method to set docking status
    bool isUndocking(); // Method to check if undocking
    bool startDocking(float stateX, float stateY); // Method to start docking
    bool startMowing(float stateX, float stateY); // Method to start mowing
    bool addObstacle(float stateX, float stateY); // Method to add obstacle
    bool mowingCompleted(); // Method to check if mowing is completed
    bool findObstacleSafeMowPoint(Point &findPathToPoint); // Method to find safe mowing point around obstacle
    void clearObstacles(); // Method to clear obstacles
    void clearMap(); // Method to clear map
    void dump(); // Method to dump (print) map data
    bool load(); // Method to load map from file
    bool save(); // Method to save map to file
    void stressTest(); // Method to perform stress test
    long calcMapCRC(); // Method to calculate map CRC (Cyclic Redundancy Check)
  private:
    void finishedUploadingMap(); // Private method called when map upload is finished
    void checkMemoryErrors(); // Private method to check for memory errors
    bool nextMowPoint(bool sim); // Private method to go to next mowing point
    bool nextDockPoint(bool sim); // Private method to go to next docking point
    bool nextFreePoint(bool sim); // Private method to go to next free point
    bool findPath(Point &src, Point &dst); // Private method to find path between two points
    float distance(Point &src, Point &dst); // Private method to calculate distance between two points
    float pointsAngle(float x1, float y1, float x2, float y2); // Private method to calculate angle between points
    float scalePI(float v); // Private method to scale value to PI range
    float distancePI(float x, float w); // Private method to calculate distance in PI range
    float distanceManhattan(Point &pos0, Point &pos1); // Private method to calculate Manhattan distance between points
    float calcHeuristic(Point &pos0, Point &pos1); // Private method to calculate heuristic for pathfinding
    float scalePIangles(float setAngle, float currAngle); // Private method to scale angles to PI range
    bool lineIntersects (Point &p0, Point &p1, Point &p2, Point &p3); // Private method to check if lines intersect
    bool linePolygonIntersection(Point &src, Point &dst, Polygon &poly); // Private method to check if line intersects polygon
    float polygonArea(Polygon &poly); // Private method to calculate polygon area
    bool polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist); // Private method to offset polygon
    int findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx); // Private method to find next neighbor node
    bool pointIsInsidePolygon(Polygon &polygon, Point &pt); // Private method to check if point is inside polygon
    void findPathFinderSafeStartPoint(Point &src, Point &dst); // Private method to find safe start point for pathfinding
    bool linePolygonIntersectPoint(Point &src, Point &dst, Polygon &poly, Point &sect); // Private method to find intersection point of line and polygon
    bool lineLineIntersection(Point &A, Point &B, Point &C, Point &D, Point &pt); // Private method to find intersection point of two lines
    bool isPointInBoundingBox(Point &pt, Point &A, Point &B); // Private method to check if point is inside bounding box
    int linePolygonIntersectionCount(Point &src, Point &dst, Polygon &poly); // Private method to count intersections between line and polygon
    void testIntegerCalcs(); // Private method to test integer calculations
};

#endif // End of preprocessor guard for SUNRAY_MAP_H

