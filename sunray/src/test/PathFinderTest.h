// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
/*
  test path finder
*/

#ifndef SUNRAY_PATHFINDER_TEST
#define SUNRAY_PATHFINDER_TEST

#include <Arduino.h>
#include "../../map.h"


class PathFinderTest
{
  public:
    void runPerimeterPathTest();
    void runRandomPathTest();
    bool findValidPoint(Point &pt);
    bool pointIsValid(Point &pt);
};


#endif

