/*
  Server.h - Base class that provides Server
*/

#ifndef server_h
#define server_h

#include "Print.h"

class Server : public Print {
public:
  virtual void begin() =0;
};

#endif
