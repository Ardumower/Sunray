// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  test pathfinder
*/

#include "PathFinderTest.h"
#include <Arduino.h>
#include "../../map.h"
#include "../../robot.h"


bool PathFinderTest::pointIsValid(Point &pt){
  if (!maps.pointIsInsidePolygon( maps.perimeterPoints, pt)) return false;    

  for (int idx=0; idx < maps.obstacles.numPolygons; idx++){
    if (!maps.pointIsInsidePolygon( maps.obstacles.polygons[idx], pt)) return false;
  }

  for (int idx=0; idx < maps.exclusions.numPolygons; idx++){
    if (maps.pointIsInsidePolygon( maps.exclusions.polygons[idx], pt)) return false;
  }    
  return true;
}


bool PathFinderTest::findValidPoint(Point &pt){
  int timeout = 1000;
  float d = 30.0;
  while (timeout > 0){
    pt.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    if (pointIsValid(pt)) return true;
  }
  CONSOLE.println("findValidPoint failed!");
  return false;  
}


void PathFinderTest::run(){
  CONSOLE.println("PathFinderTest::run");

  if (maps.perimeterPoints.numPoints == 0) {
    CONSOLE.println("PathFinderTest::run - no map, nothing to test");  
    return;
  }

  Point src;
  Point dst;
  int numTests = 0;
  int numTestsFailed = 0; 
  float d = 30.0;  
  for (int i=0 ; i < 1000; i++){
    CONSOLE.print("PathFinderTest::run loop ");
    CONSOLE.println(i);
    for (int j=0 ; j < 20; j++){
      //addObstacle( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    }
    if (!findValidPoint(src)) break;
    if (!findValidPoint(dst)) break;
    numTests++;
    bool res = maps.findPath(src, dst);
    if (!res) numTestsFailed++;    
    //clearObstacles();
  }  
  CONSOLE.print("PathFinderTest::run #tests ");
  CONSOLE.print(numTests);
  CONSOLE.print("  #failed ");
  CONSOLE.println(numTestsFailed);
}



