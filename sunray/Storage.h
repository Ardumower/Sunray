// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef STORAGE_H
#define STORAGE_H


#include <Arduino.h>
#include <SD.h>

class Storage {
public:
  bool loadState();
  bool saveState();

private:
  double calcStateCRC();
  void dumpState();
  File stateFile;
  double stateCRC = 0;
};

extern Storage storage;


#endif
