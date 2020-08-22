// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH


#include "map.h"
#include "robot.h"
#include "config.h"
#include <Arduino.h>



Point::Point(){
  init();
}

void Point::init(){
  x = 0;
  y = 0;
}

Point::Point(float ax, float ay){
  x = ax;
  y = ay;
}

void Point::assign(Point &fromPoint){
  x = fromPoint.x;
  y = fromPoint.y;
}

void Point::setXY(float ax, float ay){
  x = ax;
  y = ay;
}

// -----------------------------------
Polygon::Polygon(){  
  init();
}


Polygon::Polygon(short aNumPoints){ 
  init();
  alloc(aNumPoints);  
}

void Polygon::init(){  
  numPoints = 0;  
  points = NULL;
}

Polygon::~Polygon(){
  // dealloc();
}

void Polygon::alloc(short aNumPoints){
  if (aNumPoints == numPoints) return;
  Point* newPoints = new Point[aNumPoints];  
  if (points != NULL){
    memcpy(newPoints, points, sizeof(Point)* min(numPoints,aNumPoints) );        
    delete[] points;    
  } 
  points = newPoints;              
  numPoints = aNumPoints;
}

void Polygon::dealloc(){
  if (points == NULL) return;  
  delete[] points;  
  points = NULL;
  numPoints = 0;  
}

void Polygon::dump(){
  for (int i=0; i < numPoints; i++){
    CONSOLE.print("(");
    CONSOLE.print(points[i].x);
    CONSOLE.print(",");
    CONSOLE.print(points[i].y);
    CONSOLE.print(")");   
    if (i < numPoints-1) CONSOLE.print(",");
  }
  CONSOLE.println();
}

// -----------------------------------

PolygonList::PolygonList(){
  init();
}
  
PolygonList::PolygonList(short aNumPolygons){
  init();
  alloc(aNumPolygons);  
}

void PolygonList::init(){
  numPolygons = 0;
  polygons = NULL;  
}

PolygonList::~PolygonList(){
  //dealloc();
}

void PolygonList::alloc(short aNumPolygons){  
  if (aNumPolygons == numPolygons) return;
  Polygon* newPolygons = new Polygon[aNumPolygons];  
  if (polygons != NULL){
    memcpy(newPolygons, polygons, sizeof(Polygon)* min(numPolygons, aNumPolygons));        
    if (aNumPolygons < numPolygons){
      for (int i=aNumPolygons; i < numPolygons; i++){
        //polygons[i].dealloc();        
      }  
    }
    delete[] polygons;    
  } 
  polygons = newPolygons;              
  numPolygons = aNumPolygons;  
}

void PolygonList::dealloc(){
  if (polygons == NULL) return;
  for (int i=0; i < numPolygons; i++){
    polygons[i].dealloc();        
  }  
  delete[] polygons;
  polygons = NULL;
  numPolygons = 0;  
}

int PolygonList::numPoints(){
  int num = 0;
  for (int i=0; i < numPolygons; i++){
     num += polygons[i].numPoints;
  }
  return num;
}

void PolygonList::dump(){
  for (int i=0; i < numPolygons; i++){
    CONSOLE.print(i);
    CONSOLE.print(":");
    polygons[i].dump();
  }  
  CONSOLE.println();
}


// -----------------------------------

Node::Node(){
  init();
}

Node::Node(Point *aPoint, Node *aParentNode){
  init();
  point = aPoint;
  parent = aParentNode;  
};

void Node::init(){
  g = 0;
  h = 0;  
  f = 0;
  opened = false;
  closed = false;
  point = NULL;
  parent = NULL;
}

void Node::dealloc(){
}


// -----------------------------------


NodeList::NodeList(){
  init();
}
  
NodeList::NodeList(short aNumNodes){
  init();
  alloc(aNumNodes);  
}

void NodeList::init(){
  numNodes = 0;
  nodes = NULL;  
}

NodeList::~NodeList(){
  //dealloc();
}

void NodeList::alloc(short aNumNodes){  
  if (aNumNodes == numNodes) return;
  Node* newNodes = new Node[aNumNodes];  
  if (nodes != NULL){
    memcpy(newNodes, nodes, sizeof(Node)* min(numNodes, aNumNodes));        
    if (aNumNodes < numNodes){
      for (int i=aNumNodes; i < numNodes; i++){
        //nodes[i].dealloc();        
      }  
    }
    delete[] nodes;    
  } 
  nodes = newNodes;              
  numNodes = aNumNodes;  
}

void NodeList::dealloc(){
  if (nodes == NULL) return;
  for (int i=0; i < numNodes; i++){
    nodes[i].dealloc();        
  }  
  delete[] nodes;
  nodes = NULL;
  numNodes = 0;  
}



// ---------------------------------------------------------------------

// rescale to -PI..+PI
float Map::scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d);
  else if (d < -PI) return (2*PI+d);
  else return d;
}


// scale setangle, so that both PI angles have the same sign
float Map::scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}


// compute course (angle in rad) between two points
float Map::pointsAngle(float x1, float y1, float x2, float y2){
  float dX = x2 - x1;
  float dY = y2 - y1;
  float angle = scalePI(atan2(dY, dX));           
  return angle;
}



// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float Map::distancePI(float x, float w)
{
  // cases:
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;
  return d; 
}

// This is the Manhattan distance
float Map::distanceManhattan(Point &pos0, Point &pos1){
  float d1 = abs (pos1.x - pos0.x);
  float d2 = abs (pos1.y - pos0.y);
  return d1 + d2;
}



void Map::begin(){
  wayMode = WAY_MOW;
  trackReverse = false;
  trackSlow = false;
  useGPSfloatForPosEstimation = true;
  useGPSfloatForDeltaEstimation = true;
  useIMU = true;
  mowPointsIdx = 0;
  freePointsIdx = 0;
  dockPointsIdx = 0;  
}

void Map::dump(){
  CONSOLE.println("map dump");  
  CONSOLE.print("points: ");
  points.dump();
  CONSOLE.print("perimeter pts: ");
  CONSOLE.println(perimeterPoints.numPoints);
  //perimeterPoints.dump();
  CONSOLE.print("exclusion pts: ");
  CONSOLE.println(exclusionPointsCount);  
  CONSOLE.print("exclusions: ");  
  CONSOLE.println(exclusions.numPolygons);  
  //exclusions.dump();  
  CONSOLE.print("dock pts: ");
  CONSOLE.println(dockPoints.numPoints);
  //dockPoints.dump();
  CONSOLE.print("mow: ");  
  CONSOLE.println(mowPoints.numPoints);  
  //mowPoints.dump();
  CONSOLE.print("first mow point:");
  CONSOLE.print(mowPoints.points[0].x);
  CONSOLE.print(",");
  CONSOLE.println(mowPoints.points[0].y);
  CONSOLE.print("free pts: ");
  CONSOLE.println(freePoints.numPoints);
}
  
// set point
bool Map::setPoint(int idx, float x, float y){  
  if (idx == 0){   
    points.dealloc();    
    perimeterPoints.dealloc();    
    exclusions.dealloc();
    dockPoints.dealloc();      
    mowPoints.dealloc();    
    freePoints.dealloc();
    obstacles.dealloc();
    pathFinderObstacles.dealloc();
    pathFinderNodes.dealloc();
  }    
  points.alloc(idx+1);
  points.points[idx].setXY(x, y);      
  return true;
}


// set number points for point type
bool Map::setWayCount(WayType type, int count){
  switch (type){
    case WAY_PERIMETER:            
      perimeterPoints.alloc(count);
      for (int i=0; i < count; i++){
        perimeterPoints.points[i].assign( points.points[i] );
      }
      break;
    case WAY_EXCLUSION:      
      exclusionPointsCount = count;                  
      break;
    case WAY_DOCK:    
      dockPoints.alloc(count);      
      for (int i=0; i < count; i++){
        dockPoints.points[i].assign( points.points[perimeterPoints.numPoints + exclusionPointsCount + i] );
      }
      break;
    case WAY_MOW:          
      mowPoints.alloc(count);
      for (int i=0; i < count; i++){
        mowPoints.points[i].assign( points.points[perimeterPoints.numPoints + exclusionPointsCount + dockPoints.numPoints + i] );
      }
      if (exclusionPointsCount == 0){
        points.dealloc();
        dump();
      }
      break;    
    case WAY_FREE:      
      break;
    default: 
      return false;       
  }
  mowPointsIdx = 0;
  dockPointsIdx = 0;
  freePointsIdx = 0;  
  return true;
}


// set number exclusion points for exclusion
bool Map::setExclusionLength(int idx, int len){  
  /*CONSOLE.print("setExclusionLength ");
  CONSOLE.print(idx);
  CONSOLE.print("=");
  CONSOLE.println(len);*/
    
  exclusions.alloc(idx+1);
  exclusions.polygons[idx].alloc(len);
  
  
  int ptIdx = 0;  
  for (int i=0; i < idx; i++){    
    ptIdx += exclusions.polygons[i].numPoints;    
  }    
  for (int j=0; j < len; j++){
    exclusions.polygons[idx].points[j].assign( points.points[perimeterPoints.numPoints + ptIdx] );        
    ptIdx ++;
  }
  CONSOLE.print("ptIdx=");
  CONSOLE.print(ptIdx);
  CONSOLE.print(" exclusionPointsCount=");
  CONSOLE.print(exclusionPointsCount);
  CONSOLE.println();
  if (ptIdx == exclusionPointsCount){
    points.dealloc();
    dump();
  }
  
  //CONSOLE.print("exclusion ");
  //CONSOLE.print(idx);
  //CONSOLE.print(": ");
  //CONSOLE.println(exclusionLength[idx]);   
  return true;
}


// set desired progress in mowing points list
// 1.0 = 100%
// TODO: use path finder for valid free points to target point
void Map::setMowingPointPercent(float perc){
  mowPointsIdx = (int)( ((float)mowPoints.numPoints) * perc);
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }
}

void Map::skipNextMowingPoint(){
  mowPointsIdx++;
  if (mowPointsIdx >= mowPoints.numPoints) {
    mowPointsIdx = mowPoints.numPoints-1;
  }  
}

void Map::run(){
  switch (wayMode){
    case WAY_DOCK:      
      if (dockPointsIdx < dockPoints.numPoints){
        targetPoint.assign( dockPoints.points[dockPointsIdx] );
      }
      break;
    case WAY_MOW:
      if (mowPointsIdx < mowPoints.numPoints){
        targetPoint.assign( mowPoints.points[mowPointsIdx] );
      }
      break;
    case WAY_FREE:      
      if (freePointsIdx < freePoints.numPoints){
        targetPoint.assign(freePoints.points[freePointsIdx]);
      }
      break;
  } 
  percentCompleted = (((float)mowPointsIdx) / ((float)mowPoints.numPoints) * 100.0);
}

float Map::distanceToTargetPoint(float stateX, float stateY){  
  float dX = targetPoint.x - stateX;
  float dY = targetPoint.y - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

float Map::distanceToLastTargetPoint(float stateX, float stateY){  
  float dX = lastTargetPoint.x - stateX;
  float dY = lastTargetPoint.y - stateY;
  float targetDist = sqrt( sq(dX) + sq(dY) );    
  return targetDist;
}

// check if path from last target to target to next target is a curve
bool Map::nextPointIsStraight(){
  if (wayMode != WAY_MOW) return false;
  if (mowPointsIdx+1 >= mowPoints.numPoints) return false;     
  Point nextPt;
  nextPt.assign(mowPoints.points[mowPointsIdx+1]);  
  float angleCurr = pointsAngle(lastTargetPoint.x, lastTargetPoint.y, targetPoint.x, targetPoint.y);
  float angleNext = pointsAngle(targetPoint.x, targetPoint.y, nextPt.x, nextPt.y);
  angleNext = scalePIangles(angleNext, angleCurr);                    
  float diffDelta = distancePI(angleCurr, angleNext);                 
  //CONSOLE.println(fabs(diffDelta)/PI*180.0);
  return ((fabs(diffDelta)/PI*180.0) < 20);
}


// set robot state (x,y,delta) to final docking state (x,y,delta)
void Map::setRobotStatePosToDockingPos(float &x, float &y, float &delta){
  if (dockPoints.numPoints < 2) return;
  Point dockFinalPt;
  Point dockPrevPt;
  dockFinalPt.assign(dockPoints.points[ dockPoints.numPoints-1]);  
  dockPrevPt.assign(dockPoints.points[ dockPoints.numPoints-2]);
  x = dockFinalPt.x;
  y = dockFinalPt.y;
  delta = pointsAngle(dockPrevPt.x, dockPrevPt.y, dockFinalPt.x, dockFinalPt.y);  
}             

// mower has been docked
void Map::setIsDocked(bool flag){
  if (dockPoints.numPoints < 2) return;
  if (flag){
    wayMode = WAY_DOCK;
    dockPointsIdx = dockPoints.numPoints-2;
    //targetPointIdx = dockStartIdx + dockPointsIdx;                     
    trackReverse = true;             
    trackSlow = true;
    useGPSfloatForPosEstimation = false;  
    useGPSfloatForDeltaEstimation = false;
    useIMU = true; // false
  } else {
    wayMode = WAY_FREE;
    dockPointsIdx = 0;    
    trackReverse = false;             
    trackSlow = false;
    useGPSfloatForPosEstimation = true;    
    useGPSfloatForDeltaEstimation = true;
    useIMU = true;
  }  
}

void Map::startDocking(float stateX, float stateY){
  shouldDock = true;
  shouldMow = false;
  if (dockPoints.numPoints > 0){
    // find valid path to docking point      
    //freePoints.alloc(0);
    Point src;
    Point dst;
    src.setXY(stateX, stateY);    
    dst.assign(dockPoints.points[0]);    
    findPath(src, dst);
    wayMode = WAY_FREE;          
  }    
}

void Map::startMowing(float stateX, float stateY){
  shouldDock = false;
  shouldMow = true;    
  if (mowPoints.numPoints > 0){
    // find valid path to mowing point    
    //freePoints.alloc(0);
    Point src;
    Point dst;
    src.setXY(stateX, stateY);
    if (wayMode == WAY_DOCK){
      src.assign(dockPoints.points[0]);
    } else {
      wayMode = WAY_FREE;      
      freePointsIdx = 0;    
    }        
    if (findObstacleSafeMowPoint()){
      dst.assign(mowPoints.points[mowPointsIdx]);
      findPath(src, dst);       
    }
  }  
}

// add dynamic octagon obstacle in front of robot on line going from robot to target point
void Map::addObstacle(float stateX, float stateY){    
  float d1 = 0.1;   // distance from center to nearest octagon edges
  float d2 = 3*d1;  // distance from center to farest octagon edges
  
  float angleCurr = pointsAngle(stateX, stateY, targetPoint.x, targetPoint.y);
  float r = d2 + 0.05;
  float x = stateX + cos(angleCurr) * r;
  float y = stateY + sin(angleCurr) * r;
  
  CONSOLE.print("addObstacle ");
  CONSOLE.print(x);
  CONSOLE.print(",");
  CONSOLE.println(y);
  int idx = obstacles.numPolygons;
  obstacles.alloc(idx+1);
  obstacles.polygons[idx].alloc(8);
  
  obstacles.polygons[idx].points[0].setXY(x-d2, y-d1);
  obstacles.polygons[idx].points[1].setXY(x-d1, y-d2);
  obstacles.polygons[idx].points[2].setXY(x+d1, y-d2);
  obstacles.polygons[idx].points[3].setXY(x+d2, y-d1);
  obstacles.polygons[idx].points[4].setXY(x+d2, y+d1);
  obstacles.polygons[idx].points[5].setXY(x+d1, y+d2);
  obstacles.polygons[idx].points[6].setXY(x-d1, y+d2);
  obstacles.polygons[idx].points[7].setXY(x-d2, y+d1);         
}


// check if mowing point is inside any obstacle, and if so, find next mowing point (outside any obstacles)
bool Map::findObstacleSafeMowPoint(){  
  bool safe;  
  Point dst;  
  while (true){
    safe = true;  
    dst.assign(mowPoints.points[mowPointsIdx]);
    CONSOLE.print("findObstacleSafeMowPoint checking ");    
    CONSOLE.print(dst.x);
    CONSOLE.print(",");
    CONSOLE.println(dst.y);
    for (int idx=0; idx < obstacles.numPolygons; idx++){
      if (pointIsInsidePolygon( obstacles.polygons[idx], dst.x, dst.y)){
        safe = false;
        break;
      }
    }
    if (safe) return true;    
    if (mowPointsIdx >= mowPoints.numPoints){
      CONSOLE.println("findObstacleSafeMowPoint error: no more mowing points reachable due to obstacles");
      return false;
    } 
    mowPointsIdx++;
  }
}

// go to next point
// sim=true: only simulate (do not change data)
bool Map::nextPoint(bool sim){
  CONSOLE.print("nextPoint sim=");
  CONSOLE.print(sim);
  CONSOLE.print(" wayMode=");
  CONSOLE.println(wayMode);
  if (wayMode == WAY_DOCK){
    return (nextDockPoint(sim));
  } 
  else if (wayMode == WAY_MOW) {
    return (nextMowPoint(sim));
  } 
  else if (wayMode == WAY_FREE) {
    return (nextFreePoint(sim));
  } else return false;
}


// get next mowing point
bool Map::nextMowPoint(bool sim){  
  if (shouldMow){
    if (mowPointsIdx+1 < mowPoints.numPoints){
      // next mowing point       
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) mowPointsIdx++;
      //if (!sim) targetPointIdx++;      
      return true;
    } else {
      // finished mowing;
      mowPointsIdx = 0;            
      return false;
    }         
  } else if ((shouldDock) && (dockPoints.numPoints > 0)) {      
      // go docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) freePointsIdx = 0; 
      if (!sim) wayMode = WAY_FREE;      
      return true;    
  } else return false;  
}

// get next docking point  
bool Map::nextDockPoint(bool sim){    
  if (shouldDock){
    // should dock  
    if (dockPointsIdx+1 < dockPoints.numPoints){
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx++;              
      if (!sim) trackReverse = false;              
      if (!sim) trackSlow = true;
      if (!sim) useGPSfloatForPosEstimation = false;    
      if (!sim) useGPSfloatForDeltaEstimation = false;    
      if (!sim) useIMU = true;     // false      
      return true;
    } else {
      // finished docking
      return false;
    } 
  } else if (shouldMow){
    // should undock
    if (dockPointsIdx > 0){
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx--;              
      if (!sim) trackReverse = true;              
      if (!sim) trackSlow = true;      
      return true;
    } else {
      // finished undocking
      if ((shouldMow) && (mowPoints.numPoints > 0 )){
        if (!sim) lastTargetPoint.assign(targetPoint);
        //if (!sim) targetPointIdx = freeStartIdx;
        if (!sim) wayMode = WAY_FREE;      
        if (!sim) trackReverse = false;              
        if (!sim) trackSlow = false;
        if (!sim) useGPSfloatForPosEstimation = true;    
        if (!sim) useGPSfloatForDeltaEstimation = true;    
        if (!sim) useIMU = true;    
        return true;
      } else return false;        
    }  
  }
}

// get next free point  
bool Map::nextFreePoint(bool sim){
  // free points
  if (freePointsIdx+1 < freePoints.numPoints){
    if (!sim) lastTargetPoint.assign(targetPoint);
    if (!sim) freePointsIdx++;                  
    return true;
  } else {
    // finished free points
    if ((shouldMow) && (mowPoints.numPoints > 0 )){
      // start mowing
      if (!sim) lastTargetPoint.assign(targetPoint);      
      if (!sim) wayMode = WAY_MOW;
      return true;  
    } else if ((shouldDock) && (dockPoints.numPoints > 0)){      
      // start docking
      if (!sim) lastTargetPoint.assign(targetPoint);
      if (!sim) dockPointsIdx = 0;      
      if (!sim) wayMode = WAY_DOCK;      
      return true;
    } else return false;
  }  
}

void Map::setLastTargetPoint(float stateX, float stateY){
  lastTargetPoint.setXY(stateX, stateY);
}


// ------------------------------------------------------ 


float Map::distance(Point &src, Point &dst) {
  return sqrt(sq(src.x-dst.x)+sq(src.y-dst.y));  
}



// checks if point is inside  polygon
// The algorithm is ray-casting to the right. Each iteration of the loop, the test point is checked against
// one of the polygon's edges. The first line of the if-test succeeds if the point's y-coord is within the
// edge's scope. The second line checks whether the test point is to the left of the line
// If that is true the line drawn rightwards from the test point crosses that edge.
// By repeatedly inverting the value of c, the algorithm counts how many times the rightward line crosses the
// polygon. If it crosses an odd number of times, then the point is inside; if an even number, the point is outside.
bool Map::pointIsInsidePolygon( Polygon &polygon, float x, float y)
{
  int i, j, c = 0;
  int nvert = polygon.numPoints;
  if (nvert == 0) return false;
  Point pti;
  Point ptj;  
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    pti.assign(polygon.points[i]);
    ptj.assign(polygon.points[j]);    
    if ( ((pti.y>y) != (ptj.y>y)) &&
     (x < (ptj.x-pti.x) * (y-pti.y) / (ptj.y-pti.y) + pti.x) )
       c = !c;
  }
  return (c % 2 != 0);
}      

// checks if two lines intersect (or if single line points touch with line or line points)
// (p0,p1) 1st line
// (p2,p3) 2nd line
bool Map::lineIntersects (Point &p0, Point &p1, Point &p2, Point &p3) {
  /*CONSOLE.print("lineIntersects ((");
  CONSOLE.print(p0.x);  
  CONSOLE.print(",");
  CONSOLE.print(p0.y);
  CONSOLE.print("),(");
  CONSOLE.print(p1.x);
  CONSOLE.print(",");
  CONSOLE.print(p1.y);
  CONSOLE.print("))   ((");   
  CONSOLE.print(p2.x);  
  CONSOLE.print(",");
  CONSOLE.print(p2.y);
  CONSOLE.print("),(");
  CONSOLE.print(p3.x);
  CONSOLE.print(",");
  CONSOLE.print(p3.y);
  CONSOLE.print(")) ");*/  
  float s1x = p1.x - p0.x;
  float s1y = p1.y - p0.y;
  float s2x = p3.x - p2.x;
  float s2y = p3.y - p2.y;
  float s = (-s1y * (p0.x - p2.x) + s1x * (p0.y - p2.y)) / (-s2x * s1y + s1x * s2y);
  float t = (s2x * (p0.y - p2.y) - s2y * (p0.x - p2.x)) / (-s2x * s1y + s1x * s2y);
  bool res =  ((s >= 0) && (s <= 1) && (t >= 0) && (t <= 1));
  //CONSOLE.println(res);  
  return res;
}
    
  
// determines if a line intersects (or touches) a polygon
bool Map::linePolygonIntersection( Point &src, Point &dst, Polygon &poly) {      
  //Poly testpoly = poly;
  //if (allowtouch) testpoly = this.polygonOffset(poly, -0.02);      
  Point p1;
  Point p2;
  for (int i = 0; i < poly.numPoints; i++) {      
    p1.assign( poly.points[i] );
    p2.assign( poly.points[ (i+1) % poly.numPoints] );             
    if (lineIntersects(p1, p2, src, dst)) {        
      return true;
    }
    //if (this.doIntersect(p1, p2, src, dst)) return true;
    //if (this.lineLineIntersection(p1, p2, src, dst) != null) return true;      
  }       
  return false;
}
      
  
// Calculates the area of a polygon.
float Map::polygonArea(Polygon &poly){
  float a = 0;
  int i;
  int l;
  Point v0;
  Point v1;
  for (i = 0, l = poly.numPoints; i < l; i++) {
    v0.assign( poly.points[i] );
    v1.assign( poly.points[i == l - 1 ? 0 : i + 1] );
    a += v0.x * v1.y;
    a -= v1.x * v0.y;
  }
  return a / 2;
}
  

  
  
// offset polygon points by distance
// https://stackoverflow.com/questions/54033808/how-to-offset-polygon-edges
void Map::polygonOffset(Polygon &srcPoly, Polygon &dstPoly, float dist){    
  bool orient = (polygonArea(srcPoly) >= 0);
  //console.log('orient',orient);
  //var orient2 = ClipperLib.Clipper.Orientation(poly);
  //if (orient != orient2){
  //  console.log('polygonOffset bogus!');   
  //}        
  dstPoly.alloc(srcPoly.numPoints);
  Point p1;
  Point p2;
  Point p3;  
  for (int idx1 = 0; idx1 < srcPoly.numPoints; idx1++){
    int idx2 = idx1-1;
    if (idx2 < 0) idx2 = srcPoly.numPoints-1;
    int idx3 = idx1+1;
    if (idx3 > srcPoly.numPoints-1) idx3 = 0;      
    p2.assign(srcPoly.points[idx2]); // previous           
    p1.assign(srcPoly.points[idx1]); // center
    p3.assign(srcPoly.points[idx3]); // next                 
    float a3 = atan2(p3.y - p1.y, p3.x - p1.x);            
    float a2 = atan2(p2.y - p1.y, p2.x - p1.x);      
    float angle = a2 + (a3-a2)/2;      
    if (a3 < a2) angle-=PI;      
    if (!orient) angle+=PI;    
    dstPoly.points[idx1].setXY( p1.x + dist * cos(angle), p1.y + dist * sin(angle) );     
  }  
}

      
float Map::calcHeuristic(Point &pos0, Point &pos1) {
  return distanceManhattan(pos0, pos1);
  //return distance(pos0, pos1) ;  
}
  
  
int Map::findNextNeighbor(NodeList &nodes, PolygonList &obstacles, Node &node, int startIdx) {
  //var x = node.pos.X;
  //var y = node.pos.Y;   
  for (int idx = startIdx+1; idx < nodes.numNodes; idx++){
    if (nodes.nodes[idx].opened) continue;
    if (nodes.nodes[idx].point == node.point) continue;     
    Point *pt = nodes.nodes[idx].point;            
    //if (pt.visited) continue;
    //if (this.distance(pt, node.pos) > 10) continue;
    bool safe = true;
    for (int idx2 = 0; idx2 < obstacles.numPolygons; idx2++){
       if (linePolygonIntersection (*node.point, *pt, obstacles.polygons[idx2])) {
         safe = false;
         break;
       }
    }
    if (safe) {          
      //pt.visited = true;
      //var anode = {pos: pt, parent: node, f:0, g:0, h:0};          
      //ret.push(anode);
      return idx;
    }            
  }       
  return -1;
}  


// astar path finder 
// https://briangrinstead.com/blog/astar-search-algorithm-in-javascript/
bool Map::findPath(Point &src, Point &dst){
  CONSOLE.print("findPath (");
  CONSOLE.print(src.x);
  CONSOLE.print(",");
  CONSOLE.print(src.y);
  CONSOLE.print(") (");
  CONSOLE.print(dst.x);
  CONSOLE.print(",");
  CONSOLE.print(dst.y);
  CONSOLE.println(")");  
      
  if (ENABLE_PATH_FINDER){
    CONSOLE.println("path finder is enabled");      
    
    // create path-finder obstacles    
    int idx = 0;
    pathFinderObstacles.alloc(1 + exclusions.numPolygons + obstacles.numPolygons);        
    
    polygonOffset(perimeterPoints, pathFinderObstacles.polygons[idx], 0.02);      
    idx++;
    
    for (int i=0; i < exclusions.numPolygons; i++){
      polygonOffset(exclusions.polygons[i], pathFinderObstacles.polygons[idx], -0.02);
      idx++;
    }      
    for (int i=0; i < obstacles.numPolygons; i++){
      polygonOffset(obstacles.polygons[i], pathFinderObstacles.polygons[idx], -0.02);
      idx++;
    }  
    
    //CONSOLE.println("perimeter");
    //perimeterPoints.dump();
    //CONSOLE.println("exclusions");
    //exclusions.dump();
    //CONSOLE.println("obstacles");
    //obstacles.dump();
    //CONSOLE.println("pathFinderObstacles");
    //pathFinderObstacles.dump();
    
    // create nodes
    pathFinderNodes.alloc(exclusions.numPoints() + obstacles.numPoints() + perimeterPoints.numPoints + 2);        
    for (int i=0; i < pathFinderNodes.numNodes; i++){
      pathFinderNodes.nodes[i].init();
    }
    // exclusion nodes
    idx = 0;
    for (int i=0; i < exclusions.numPolygons; i++){
      for (int j=0; j < exclusions.polygons[i].numPoints; j++){    
        pathFinderNodes.nodes[idx].point = &exclusions.polygons[i].points[j];
        idx++;
      }
    }
    // obstacle nodes    
    for (int i=0; i < obstacles.numPolygons; i++){
      for (int j=0; j < obstacles.polygons[i].numPoints; j++){    
        pathFinderNodes.nodes[idx].point = &obstacles.polygons[i].points[j];
        idx++;
      }
    }
    // perimeter nodes
    for (int j=0; j < perimeterPoints.numPoints; j++){    
      pathFinderNodes.nodes[idx].point = &perimeterPoints.points[j];
      idx++;
    }      
    // start node
    Node *start = &pathFinderNodes.nodes[idx];
    start->point = &src;
    start->opened = true;
    idx++;
    // end node
    Node *end = &pathFinderNodes.nodes[idx];
    end->point = &dst;    
    idx++;
    //CONSOLE.print("nodes=");
    //CONSOLE.print(nodes.numNodes);
    //CONSOLE.print(" idx=");
    //CONSOLE.println(idx);
    
    
    //CONSOLE.print("sz=");
    //CONSOLE.println(sizeof(visitedPoints));
    
    int timeout = 300;    
    Node *currentNode = NULL;
    
    CONSOLE.println("starting path-finder");
    while(true) {       
      timeout--;      
      if (timeout == 0){
        CONSOLE.println("timeout");
        break;
      }
      // Grab the lowest f(x) to process next
      int lowInd = -1;
      for(int i=0; i<pathFinderNodes.numNodes; i++) {
        if ((pathFinderNodes.nodes[i].opened) && ((lowInd == -1) || (pathFinderNodes.nodes[i].f < pathFinderNodes.nodes[lowInd].f))) { lowInd = i; }
      }
      //CONSOLE.print("lowInd=");
      //CONSOLE.println(lowInd);
      if (lowInd == -1) break;
      currentNode = &pathFinderNodes.nodes[lowInd]; 
      // console.log('ol '+openList.length + ' cl ' + closedList.length + ' ' + currentNode.pos.X + ',' + currentNode.pos.Y);
      // End case -- result has been found, return the traced path
      if (distance(*currentNode->point, *end->point) < 0.02) break;        
      // Normal case -- move currentNode from open to closed, process each of its neighbors      
      currentNode->opened = false;
      currentNode->closed = true;
      //console.log('cn  pos'+currentNode.pos.X+','+currentNode.pos.Y);            
      //console.log('neighbors '+neighbors.length);      
      int neighborIdx = -1;
      //CONSOLE.print("currentNode ");
      //CONSOLE.print(currentNode->point->x);
      //CONSOLE.print(",");
      //CONSOLE.println(currentNode->point->y);
      while (true) {        
        neighborIdx = findNextNeighbor(pathFinderNodes, pathFinderObstacles, *currentNode, neighborIdx); 
        if (neighborIdx == -1) break;
        Node* neighbor = &pathFinderNodes.nodes[neighborIdx];        
        if (neighbor->closed) continue;                
        //CONSOLE.print("neighbor ");
        //CONSOLE.print(neighbor->point->x);
        //CONSOLE.print(",");
        //CONSOLE.println(neighbor->point->y);
        //this.debugPaths.push( [currentNode.pos, neighbor.pos] );
        // g score is the shortest distance from start to current node, we need to check if
        //   the path we have arrived at this neighbor is the shortest one we have seen yet
        //var gScore = currentNode.g + 1; // 1 is the distance from a node to it's neighbor
        float gScore = currentNode->g + distance(*currentNode->point, *neighbor->point);
        bool gScoreIsBest = false;
        bool found = neighbor->opened;        
        if (!found){          
          // This the the first time we have arrived at this node, it must be the best
          // Also, we need to take the h (heuristic) score since we haven't done so yet 
          gScoreIsBest = true;
          neighbor->h = calcHeuristic(*neighbor->point, *end->point);
          neighbor->opened = true;
        }
        else if(gScore < neighbor->g) {
          // We have already seen the node, but last time it had a worse g (distance from start)
          gScoreIsBest = true;
        } 
        if(gScoreIsBest) {
          // Found an optimal (so far) path to this node.   Store info on how we got here and
          //  just how good it really is...
          neighbor->parent = currentNode;
          neighbor->g = gScore;
          neighbor->f = neighbor->g + neighbor->h;
          //neighbor.debug = "F: " + neighbor.f + "<br />G: " + neighbor.g + "<br />H: " + neighbor.h;
        }
      }
    } 
    
    CONSOLE.print("finish nodes=");
    CONSOLE.println(pathFinderNodes.numNodes);
      
    if ((currentNode != NULL) && (distance(*currentNode->point, *end->point) < 0.02)) {
      Node *curr = currentNode;
      int nodeCount = 0;
      while(curr) {                
        nodeCount++;
        curr = curr->parent;        
      }      
      freePoints.alloc(nodeCount);      
      curr = currentNode;
      int idx = nodeCount-1;
      while(curr) {                                
        freePoints.points[idx].assign( *curr->point );
        CONSOLE.print("node pt=");
        CONSOLE.print(curr->point->x);
        CONSOLE.print(",");
        CONSOLE.println(curr->point->y);
        idx--;
        curr = curr->parent;                
      }            
    } else {
      // No result was found
      CONSOLE.println("no path");      
      freePoints.alloc(2);
      freePoints.points[0].assign(src);    
      freePoints.points[1].assign(dst);        
    }       
  } else {
    freePoints.alloc(2);
    freePoints.points[0].assign(src);    
    freePoints.points[1].assign(dst);        
  }    
  freePointsIdx=0;  
  
  return true;  
}


  
  
  
