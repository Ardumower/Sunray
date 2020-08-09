// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH


// TODO:  
// + obstacle avoidance ( see function 'addDynamicObstacle' further below )

#include "map.h"
#include "config.h"
#include <Arduino.h>


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
  mowPointsCount = 0;
  perimeterPointsCount = 0;
  exclusionPointsCount = 0;
  dockPointsCount = 0;
  freePointsCount = 0;
  storeIdx = 0;
  targetPointIdx = 0;
  for (int i=0; i < MAX_POINTS; i++){
    points[i].x=0;
    points[i].y=0;
  }  
}

void Map::dump(){
  CONSOLE.println("map dump");
  CONSOLE.print("perimeter pts: ");
  CONSOLE.println(perimeterPointsCount);
  CONSOLE.print("exclusion pts: ");
  CONSOLE.println(exclusionPointsCount);  
  CONSOLE.print("dock pts: ");
  CONSOLE.println(dockPointsCount);
  CONSOLE.print("mow: ");  
  CONSOLE.println(mowPointsCount);
  CONSOLE.print("first mow point:");
  CONSOLE.print(points[mowStartIdx].x);
  CONSOLE.print(",");
  CONSOLE.println(points[mowStartIdx].y);
  CONSOLE.print("free pts: ");
  CONSOLE.println(freePointsCount);
}

// set point
bool Map::setPoint(int idx, float x, float y){  
  if (idx >= MAX_POINTS) return false;  
  if ((idx != 0) && (idx != (storeIdx+1))) return false;  
  mowPointsIdx = 0;  
  dockPointsIdx = 0;
  freePointsIdx = 0;
  points[idx].x = x;
  points[idx].y = y;
  storeIdx = idx;
  return true;
}


// set number points for point type
bool Map::setWayCount(WayType type, int count){
  switch (type){
    case WAY_PERIMETER:      
      perimeterPointsCount = count;      
      break;
    case WAY_EXCLUSION:      
      exclusionPointsCount = count;      
      break;
    case WAY_DOCK:    
      dockPointsCount = count;      
    case WAY_MOW:      
      mowPointsCount = count;            
      break;    
    case WAY_FREE:
      freePointsCount = count;
      break;
    default: 
      return false;       
  }
  dockStartIdx = perimeterPointsCount + exclusionPointsCount;
  mowStartIdx = dockStartIdx + dockPointsCount;
  freeStartIdx = mowStartIdx + mowPointsCount;
  targetPointIdx = mowStartIdx;
}


// set number exclusion points for exclusion
bool Map::setExclusionLength(int idx, int len){
  if (idx >= MAX_EXCLUSIONS) return false;
  int startIdx = perimeterPointsCount;
  if (idx > 0) startIdx = exclusionStartIdx[idx-1] + exclusionLength[idx-1];          
  exclusionCount = idx + 1;
  exclusionStartIdx[idx] = startIdx;            
  exclusionLength[idx] = len;
    
  //LinkedList<pt_t> exclusionPts = LinkedList<pt_t>();
  //exclusions.add(exclusionPts);   
  
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
  mowPointsIdx = (int)( ((float)mowPointsCount) * perc);
  if (mowPointsIdx >= mowPointsCount) {
    mowPointsIdx = mowPointsCount-1;
  }
  targetPointIdx = mowStartIdx + mowPointsIdx;
}

void Map::skipNextMowingPoint(){
  mowPointsIdx++;
  if (mowPointsIdx >= mowPointsCount) {
    mowPointsIdx = mowPointsCount-1;
  }
  targetPointIdx = mowStartIdx + mowPointsIdx;
}

void Map::run(){
  targetPoint = points[targetPointIdx];  
  percentCompleted = (((float)mowPointsIdx) / ((float)mowPointsCount) * 100.0);
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
  if (mowPointsIdx+1 >= mowPointsCount) return false;     
  pt_t nextPt = points[mowStartIdx + mowPointsIdx+1];  
  float angleCurr = pointsAngle(lastTargetPoint.x, lastTargetPoint.y, targetPoint.x, targetPoint.y);
  float angleNext = pointsAngle(targetPoint.x, targetPoint.y, nextPt.x, nextPt.y);
  angleNext = scalePIangles(angleNext, angleCurr);                    
  float diffDelta = distancePI(angleCurr, angleNext);                 
  //CONSOLE.println(fabs(diffDelta)/PI*180.0);
  return ((fabs(diffDelta)/PI*180.0) < 20);
}


// set robot state (x,y,delta) to final docking state (x,y,delta)
void Map::setRobotStatePosToDockingPos(float &x, float &y, float &delta){
  if (dockPointsCount < 2) return;
  pt_t dockFinalPt = points[dockStartIdx + dockPointsCount-1];
  pt_t dockPrevPt = points[dockStartIdx + dockPointsCount-2];
  x = dockFinalPt.x;
  y = dockFinalPt.y;
  delta = pointsAngle(dockPrevPt.x, dockPrevPt.y, dockFinalPt.x, dockFinalPt.y);  
}             

// mower has been docked
void Map::setIsDocked(bool flag){
  if (dockPointsCount < 2) return;
  if (flag){
    wayMode = WAY_DOCK;
    dockPointsIdx = dockPointsCount-2;
    targetPointIdx = dockStartIdx + dockPointsIdx;                     
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
  if (dockPointsCount > 0){
    // find valid path to docking point      
    clearFreePoints();
    pt_t src;
    src.x = stateX;
    src.y = stateY;
    pt_t dst = points[dockStartIdx];
    findPath(src, dst);
    wayMode = WAY_FREE;      
    targetPointIdx = freeStartIdx;    
  }    
}

void Map::startMowing(float stateX, float stateY){
  shouldDock = false;
  shouldMow = true;    
  if (mowPointsCount > 0){
    // find valid path to mowing point    
    clearFreePoints();
    pt_t src;
    src.x = stateX;
    src.y = stateY;
    if (wayMode == WAY_DOCK){
      src = points[dockStartIdx];
    } else {
      wayMode = WAY_FREE;      
      targetPointIdx = freeStartIdx;     
    }    
    pt_t dst = points[mowStartIdx + mowPointsIdx];
    findPath(src, dst);       
  }  
}

void Map::addDynamicObstacle(float x, float y){
  // TODO: 
  // 1. add obstacle position to some dynamic obstacle list 
  // 2. add intersection checking for dynamic obstacles to 'getObstaclePolygonPoint' and 'getObstaclePolygonLen'   
  // 3. compute new destination point (that is outside of any dynamic obstacle/exclusion/perimeter)
  // 4. re-trigger findpath like in startMowing/startDocking for new dst 
  // 5. switch to wayMode 'WAY_FREE'
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
    if (mowPointsIdx+1 < mowPointsCount){
      // next mowing point       
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) mowPointsIdx++;
      if (!sim) targetPointIdx++;      
      return true;
    } else {
      // finished mowing;
      mowPointsIdx = 0;      
      targetPointIdx = mowStartIdx;                
      return false;
    }         
  } else if ((shouldDock) && (dockPointsCount > 0)) {      
      // go docking
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) targetPointIdx = freeStartIdx; 
      if (!sim) wayMode = WAY_FREE;      
      return true;    
  } else return false;  
}

// get next docking point  
bool Map::nextDockPoint(bool sim){    
  if (shouldDock){
    // should dock  
    if (dockPointsIdx+1 < dockPointsCount){
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) dockPointsIdx++;              
      if (!sim) trackReverse = false;              
      if (!sim) trackSlow = true;
      if (!sim) useGPSfloatForPosEstimation = false;    
      if (!sim) useGPSfloatForDeltaEstimation = false;    
      if (!sim) useIMU = true;     // false
      if (!sim) targetPointIdx++;      
      return true;
    } else {
      // finished docking
      return false;
    } 
  } else if (shouldMow){
    // should undock
    if (dockPointsIdx > 0){
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) dockPointsIdx--;              
      if (!sim) trackReverse = true;              
      if (!sim) trackSlow = true;
      if (!sim) targetPointIdx--;      
      return true;
    } else {
      // finished undocking
      if ((shouldMow) && (mowPointsCount > 0 )){
        if (!sim) lastTargetPoint = targetPoint;
        if (!sim) targetPointIdx = freeStartIdx;
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
  if (freePointsIdx+1 < freePointsCount){
    if (!sim) lastTargetPoint = targetPoint;
    if (!sim) freePointsIdx++;              
    if (!sim) targetPointIdx++;      
    return true;
  } else {
    // finished free points
    if ((shouldMow) && (mowPointsCount > 0 )){
      // start mowing
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) targetPointIdx = mowStartIdx + mowPointsIdx;              
      if (!sim) wayMode = WAY_MOW;
      return true;  
    } else if ((shouldDock) && (dockPointsCount > 0)){      
      // start docking
      if (!sim) lastTargetPoint = targetPoint;
      if (!sim) dockPointsIdx = 0;
      if (!sim) targetPointIdx = dockStartIdx;                
      if (!sim) wayMode = WAY_DOCK;      
      return true;
    } else return false;
  }  
}

void Map::setLastTargetPoint(float stateX, float stateY){
  lastTargetPoint.x = stateX; 
  lastTargetPoint.y = stateY;
}


// -------free points-----------------------------------------------
void Map::addFreePoint(pt_t pt){
  if (freeStartIdx + freePointsCount >= MAX_POINTS) {
    CONSOLE.println("ERROR addFreePoint error");
    return;
  }
  points[freeStartIdx + freePointsCount] = pt;  
  freePointsCount++;
  CONSOLE.print("addFreePoint ");
  CONSOLE.print(pt.x);
  CONSOLE.print(",");
  CONSOLE.println(pt.y);
}

void Map::clearFreePoints(){
  freePointsCount=0;
  freePointsIdx=0;  
}

bool Map::includesFreePoint(pt_t pt){
  pt_t fp;
  for (int idx=0; idx < freePointsCount; idx++){
    fp = points[freeStartIdx + idx];
    if ((fp.x == pt.x) && (fp.y == pt.y)) return true;    
  }
  return false;
}


// ---------obstacle polygons (perimeter&exclusions&dynamic obstacles)---------------------------------

// TODO: add dynamic obstacles here as well
bool Map::getObstaclePolygonPoint(int polyIdx, int ptIdx, pt_t &pt){
  if (polyIdx > exclusionCount){
    return false;
  } else if (polyIdx == exclusionCount) {
    // perimeter 
    if (ptIdx >= perimeterPointsCount) return false;
    pt = points[perimeterStartIdx + ptIdx];    
    return true;
  } else {
    // exclusion
    if (ptIdx >= exclusionLength[polyIdx]) return false;
    pt = points[exclusionStartIdx[polyIdx] + ptIdx];    
    return true;
  }
}

int Map::getObstaclePolygonLen(int polyIdx){
  if (polyIdx > exclusionCount){
    return 0;
  } else if (polyIdx == exclusionCount) {
    // perimeter 
    return perimeterPointsCount;
  } else {
    // exclusion
    return exclusionLength[polyIdx];
  }
}


// ------------------------------------------------------ 

// checks if point is inside bounding box given by points A, B
bool Map::isPointInBoundingBox(pt_t pt, pt_t A, pt_t B){
  float minX = min(A.x, B.x);
  float minY = min(A.y, B.y);
  float maxX = max(A.x, B.x);
  float maxY = max(A.y, B.y);    
  if (pt.x < minX-0.02) return false;
  if (pt.y < minY-0.02) return false;
  if (pt.x > maxX+0.02) return false;
  if (pt.y > maxY+0.02) return false;
  return true;
}

float Map::distance(pt_t src, pt_t dst) {
  return sqrt(sq(src.x-dst.x)+sq(src.y-dst.y));  
}


// calculates intersection point of two lines 
// (A,B) 1st line
// (C,D) 2nd line  
// https://www.geeksforgeeks.org/program-for-point-of-intersection-of-two-lines/
bool Map::lineLineIntersection(pt_t A, pt_t B, pt_t C, pt_t D, pt_t &pt)  { 
  //console.log('lineLineIntersection', A,B,C,D);
  if ((distance(A, C) < 0.02) || (distance(A, D) < 0.02)) { 
    pt.x = A.x;
    pt.y = A.y;
    return true;
  } 
  if ((distance(B, C) < 0.02) || (distance(B, D) < 0.02)){
    pt.x = B.x;
    pt.y = B.y;
    return true;
  }   
  // Line AB represented as a1x + b1y = c1 
  float a1 = B.y - A.y; 
  float b1 = A.x - B.x; 
  float c1 = a1*(A.x) + b1*(A.y); 
  // Line CD represented as a2x + b2y = c2 
  float a2 = D.y - C.y; 
  float b2 = C.x - D.x; 
  float c2 = a2*(C.x)+ b2*(C.y);   
  float determinant = a1*b2 - a2*b1;   
  if (determinant == 0)  { 
      // The lines are parallel.         
      //console.log('lines are parallel');
      return false;
  } else { 
      float x = (b2*c1 - b1*c2)/determinant; 
      float y = (a1*c2 - a2*c1)/determinant;             
      pt.x = x;
      pt.y = y;
      if (!isPointInBoundingBox(pt, A, B)) return false; // not in bounding box of 1st line
      if (!isPointInBoundingBox(pt, C, D)) return false; // not in bounding box of 2nd line
      return true;
  } 
} 
      

// check if two lines intersect
// (p0,p1) 1st line
// (p2,p3) 2nd line
bool Map::lineIntersects (pt_t p0, pt_t p1, pt_t p2, pt_t p3) {
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
  bool res =  ((s > 0) && (s < 1) && (t > 0) && (t < 1));
  //CONSOLE.println(res);  
  return res;
}
  

// travel waypoints around polygon from index srcSrc to index srcDst in forward or reverse direction and optionally
// adding waypoints to freepoint list
// returns distance traveled
float Map::goPathOnPolygon(int polyIdx, int idxSrc, int idxDst, bool goForward, bool add){  
    if (add){
      CONSOLE.print("goPathOnPolygon polyIdx=");
      CONSOLE.print(polyIdx);
      CONSOLE.print(" idxSrc=");
      CONSOLE.print(idxSrc);
      CONSOLE.print(" idxDst=");
      CONSOLE.println(idxDst);
    }
    float dist = 0;
    int len = getObstaclePolygonLen(polyIdx);        
    if ( (idxSrc < 0) || (idxDst < 0) || (idxSrc >= len) || (idxDst >= len) ){
      CONSOLE.println("goPathOnPolygon invalid indexes");            
      return 0;
    }    
    int idx = idxSrc;    
    pt_t pt;    
    pt_t lastPt;   
    getObstaclePolygonPoint(polyIdx, idx, pt);                 
    if (add) {
      if (!goForward) addFreePoint(pt);        
    }
    while(true){      
      if (goForward){
        if (idx == len-1) idx = 0;
          else idx++;
      } else {
        if (idx == 0) idx = len-1; 
          else idx--;
      }      
      lastPt = pt;           
      getObstaclePolygonPoint(polyIdx, idx, pt);                             
      if (add) {
        if (goForward){
          addFreePoint(pt);
        } else {
          if (idx != idxDst){
            addFreePoint(pt);
          }          
        }        
      }      
      dist += distance(lastPt, pt);
      if (idx == idxDst) break;
    }    
    return dist;
}
  
  
// find shortest and 2nd shortest intersections between polygon and line (src, dst) 
bool Map::getPolyIntersections(int polyIdx, pt_t src, pt_t dst, pt_t &secPt1, pt_t &secPt2, int &polyIdx1, int &polyIdx2 ){
    CONSOLE.print("getPolyIntersections polyIdx=");
    CONSOLE.println(polyIdx);       
    int nextIdx = 0;
    float dist1 = 9999;
    float dist2 = 9999;
    pt_t pt;
    pt_t nextPt;
    pt_t secPt;
    pt_t lastSecPt;
    lastSecPt.x = 9999;
    lastSecPt.y = 9999;    
    secPt1.x = 9999;
    secPt1.y = 9999;
    secPt2.x = 9999;
    secPt2.y = 9999;
    int len = getObstaclePolygonLen(polyIdx);        
    if (len < 2) return false;    
    for (int idx=0; idx < len; idx++){                    
      if (idx == len-1) nextIdx = 0;
        else nextIdx = idx+1;                  
      getObstaclePolygonPoint(polyIdx, idx, pt);
      getObstaclePolygonPoint(polyIdx, nextIdx, nextPt);          
      if (lineLineIntersection(src, dst, pt, nextPt, secPt)){
         CONSOLE.println("intersection");
         if ( (distance(secPt, lastSecPt) > 0.02) && (distance(secPt, secPt1) > 0.02) && (distance(secPt, secPt2) > 0.02) ){                  
           float dist = distance(src, secPt);
           CONSOLE.print("dist=");
           CONSOLE.println(dist);
           if (dist < dist2){
             if (dist < dist1){
               if (dist1 < dist2){
                 dist2 = dist1;
                 polyIdx2 = polyIdx1;
                 secPt2 = secPt1;
               } 
               dist1 = dist;
               polyIdx1 = idx;
               secPt1 = secPt;                 
             } else {
               dist2 = dist;
               polyIdx2 = idx;
               secPt2 = secPt;
             }
           }                                   
         } else CONSOLE.println("ignored");
         lastSecPt = secPt;
      }
    }    
    bool res = ((dist1 < 9999) && (dist2 < 9999));    
    CONSOLE.print("getPolyIntersections res=");    
    CONSOLE.println(res);
    return res;
}
  
  
// find path using obstacles from src to dst  
// (using 'rubber-band' path-finder)
bool Map::findPath(pt_t src, pt_t dst){
  CONSOLE.print("findPath (");
  CONSOLE.print(src.x);
  CONSOLE.print(",");
  CONSOLE.print(src.y);
  CONSOLE.print(") (");
  CONSOLE.print(dst.x);
  CONSOLE.print(",");
  CONSOLE.print(dst.y);
  CONSOLE.println(")");  

  pt_t currPt = src;
  int timeout = 10;      
  clearFreePoints();  
  addFreePoint(src);      
  if (ENABLE_PATH_FINDER){
    CONSOLE.println("path finder is enabled");      
    while(true){
      bool intersects = false;          
      float minDist = 9999; 
      int polyIdx1 = 0;       
      int polyIdx2 = 0;      
      pt_t secPt1;
      pt_t secPt2;
      int polyIdx = 0;
      pt_t sp1;
      pt_t sp2;
      int pi1 = 0;
      int pi2 = 0;
      for (int idx = 0; idx <= exclusionCount; idx++){                    
        if (getPolyIntersections(idx, currPt, dst, sp1, sp2, pi1, pi2)){       
          float dist = distance(currPt, sp1);
          if (dist < minDist){
            polyIdx = idx;
            minDist = dist;                       
            secPt1 = sp1;
            secPt2 = sp2;
            polyIdx1 = pi1;
            polyIdx2 = pi2;
          }           
        }
      }
      intersects = (minDist < 9999);                
      if (intersects) {
        // intersection                                                
        addFreePoint(secPt1);        
        float dist1 = goPathOnPolygon(polyIdx, polyIdx1, polyIdx2, true, false);            
        float dist2 = goPathOnPolygon(polyIdx, polyIdx1, polyIdx2, false, false);
        goPathOnPolygon(polyIdx, polyIdx1, polyIdx2, (dist1 < dist2), true);            
        addFreePoint(secPt2);                        
        currPt = secPt2;                                   
      } else break;      
      watchdogReset();           
      timeout--;
      if (timeout == 0) {
        CONSOLE.println("findPath timeout");
        break;
      }
    }
  }
  addFreePoint(dst);
  return true;  
}    
  
  
  
// checks if line between src and dst is crossing specified polygon 
bool Map::lineObstaclePolygonIntersection ( int polyIdx, pt_t src, pt_t dst) {
  int ptIdx = 0;
  int len = getObstaclePolygonLen(polyIdx);        
  while (true){
    pt_t pt1;
    pt_t pt2;    
    if (!getObstaclePolygonPoint(polyIdx, ptIdx, pt1)) break;
    if (!getObstaclePolygonPoint(polyIdx, (ptIdx+1) % len, pt2)) break;    
    if (lineIntersects(pt1, pt2, src, dst)) return true;        
    ptIdx++;          
  }  
  return false;
}  

// checks if point is inside obstacle polygon
// The algorithm is ray-casting to the right. Each iteration of the loop, the test point is checked against
// one of the polygon's edges. The first line of the if-test succeeds if the point's y-coord is within the
// edge's scope. The second line checks whether the test point is to the left of the line
// If that is true the line drawn rightwards from the test point crosses that edge.
// By repeatedly inverting the value of c, the algorithm counts how many times the rightward line crosses the
// polygon. If it crosses an odd number of times, then the point is inside; if an even number, the point is outside.
bool Map::isInsideObstaclePolygon( int polyIdx, float x, float y)
{
  int i, j, c = 0;
  int nvert = getObstaclePolygonLen(polyIdx);        
  if (nvert == 0) return false;
  pt_t pti;
  pt_t ptj;  
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if (!getObstaclePolygonPoint(polyIdx, i, pti)) break;
    if (!getObstaclePolygonPoint(polyIdx, j, ptj)) break;
    if ( ((pti.y>y) != (ptj.y>y)) &&
     (x < (ptj.x-pti.x) * (y-pti.y) / (ptj.y-pti.y) + pti.x) )
       c = !c;
  }
  return (c % 2 != 0);
}


// check if line crosses any obstacles and is not inside any exclusion and is not outside perimeter
bool Map::lineIsObstacleSafe(pt_t src, pt_t dst){
  // perimeter
  if (!isInsideObstaclePolygon(exclusionCount, src.x+(dst.x-src.x)/2, src.y+(dst.y-src.y)/2)) return false;
  if (lineObstaclePolygonIntersection (exclusionCount, src, dst)) return false;              
  // exclusions
  for (int polyIdx = 0; polyIdx < exclusionCount; polyIdx++){                      
     if (isInsideObstaclePolygon(polyIdx, src.x+(dst.x-src.x)/2, src.y+(dst.y-src.y)/2)) return false;     
     if (lineObstaclePolygonIntersection (polyIdx, src, dst)) return false;              
  }
  return true;
}


/*
// find valid path from src to dst that does not intersect with perimeter and exclusions
bool Map::findPath(pt_t src, pt_t dst){
  CONSOLE.print("findPath (");
  CONSOLE.print(src.x);
  CONSOLE.print(",");
  CONSOLE.print(src.y);
  CONSOLE.print(") (");
  CONSOLE.print(dst.x);
  CONSOLE.print(",");
  CONSOLE.print(dst.y);
  CONSOLE.println(")");  
  //for (int polyIdx = 0; polyIdx <= exclusionCount; polyIdx++){      
  //  CONSOLE.print("polyIdx=");
  //  CONSOLE.print(polyIdx);
  //  int len = getObstaclePolygonLen(polyIdx);      
  //  pt_t pt;
  //  for (int ptIdx = 0; ptIdx < len; ptIdx++){
  //    if (!getObstaclePolygonPoint(polyIdx, ptIdx, pt)) break;  
  //    CONSOLE.print("(");
  //    CONSOLE.print(pt.x);
  //    CONSOLE.print(",");
  //    CONSOLE.print(pt.y);
  //    CONSOLE.print(")");
  //  }
  //  CONSOLE.println();
  //} 
  pt_t currPt = src;
  clearFreePoints();  
  addFreePoint(src);      
  if (ENABLE_PATH_FINDER){
    CONSOLE.println("path finder is enabled");
    while (true){
      if (lineIsObstacleSafe(currPt, dst)) break;      
      pt_t minPt;
      float minDist = 9999;
      //var d1 = distance(currPt, dst);
      // find next point that is 'safe' and closer to destination
      for (int polyIdx = 0; polyIdx <= exclusionCount; polyIdx++){      
        int len = getObstaclePolygonLen(polyIdx);      
        pt_t pt;       
        for (int ptIdx = 0; ptIdx < len; ptIdx++){
          if (!getObstaclePolygonPoint(polyIdx, ptIdx, pt)) break;            
          if (!includesFreePoint(pt)) {                        
            if (lineIsObstacleSafe(currPt, pt)) {                      
              float d2 = distance(pt, dst);
              //var d3 = distance(currPt, pt);                    
              if (d2 < minDist) {              
                minDist = d2;
                minPt = pt;
              }  
            }                
          }                
        }
      }
      
      //console.log('d1',d1,'minDist',minDist,'minPt',minPt);            
      if (minDist >= 9999) {
        CONSOLE.println("PATH ERROR: no path found");        
        break;
      }
      CONSOLE.print("next path point: ");
      CONSOLE.print(minPt.x);
      CONSOLE.print(",");
      CONSOLE.println(minPt.y);
      addFreePoint(minPt);
      currPt = minPt;            
      watchdogReset();   
    }          
  }  
  addFreePoint(dst);
  return true;
}

*/





