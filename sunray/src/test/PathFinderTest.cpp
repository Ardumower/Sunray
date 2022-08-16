// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  test pathfinder
*/

#include "PathFinderTest.h"
#include <Arduino.h>
#include "../../map.h"
#include "../../robot.h"



bool PathFinderTest::findValidPoint(Point &pt){
  int timeout = 10000;
  float d = 30.0;
  while (timeout > 0){
    pt.setXY( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    if (maps.isInsidePerimeterOutsideExclusions(pt)) return true;
    timeout--;
  }
  CONSOLE.println("findValidPoint failed!");
  return false;  
}


// for the current map, run path finder on all perimeter points
// the test is considered as failed if path has not length 2
void PathFinderTest::runPerimeterPathTest(){
  CONSOLE.println("PathFinderTest::runPerimeterPathTest");

  Point src;
  Point dst;    
  int numTests = 0;
  int numTestsFailed = 0;   
  for (int i=0; i < maps.perimeterPoints.numPoints-1; i++){
    src.assign(maps.perimeterPoints.points[i]);
    dst.assign(maps.perimeterPoints.points[i+1]);    
    numTests++;
    bool res = maps.findPath(src, dst);
    if (res) {
      if (maps.freePoints.numPoints != 2) numTestsFailed++;
    } else numTestsFailed++;
  }
  CONSOLE.print("PathFinderTest::runPerimeterPathTest #tests ");
  CONSOLE.print(numTests);
  CONSOLE.print("  #failed ");
  CONSOLE.println(numTestsFailed);
}


// for the current map, generate random source and destination points which are inside perimeter (and outside exclusions) of current map
// and run path finder to find a path from source to destination - the test is considered as failed if not path was found
void PathFinderTest::runRandomPathTest(){
  CONSOLE.println("PathFinderTest::runRandomPathTest");

  if (maps.perimeterPoints.numPoints == 0) {
    CONSOLE.println("PathFinderTest::runRandomPathTest - no map, nothing to test");  
    return;
  }

  Point src;
  Point dst;
  int numTests = 0;
  int numTestsFailed = 0; 
  float d = 30.0;  
  for (int i=0 ; i < 10000; i++){
    CONSOLE.print("PathFinderTest::runRandomPathTest loop ");
    CONSOLE.println(i);
    for (int j=0 ; j < 20; j++){
      //addObstacle( ((float)random(d*10))/10.0-d/2, ((float)random(d*10))/10.0-d/2 );
    }
    //src.setXY(-2.33, 19.41);
    //dst.setXY(-3.18, 19.39);
    if (!findValidPoint(src)) break;
    if (!findValidPoint(dst)) break;
    numTests++;
    bool res = maps.findPath(src, dst);
    if (!res) numTestsFailed++;    
    //clearObstacles();
  }  
  CONSOLE.print("PathFinderTest::runRandomPathTest #tests ");
  CONSOLE.print(numTests);
  CONSOLE.print("  #failed ");
  CONSOLE.println(numTestsFailed);
}



