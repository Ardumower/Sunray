#include "src/navigation/CameraGpsPosition.h"
#include "src/navigation/ReflectorDockingGate.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>

int main() {
  CameraGpsPosition gps;
  float x=0, y=0;
  assert(!gps.update(CameraGpsPosition::Invalid, 1, 0, 100, x, y) && x==0);
  assert(!gps.update(CameraGpsPosition::Fixed, 1, 0, 200, x, y));
  assert(!gps.update(CameraGpsPosition::Fixed, 1, 0, 400, x, y));
  assert(gps.update(CameraGpsPosition::Fixed, 1, 0, 600, x, y));
  assert(std::fabs(x-.02f)<1e-6 && y==0);
  // A single jump towards the exit cannot complete undocking.
  const float before=x;
  assert(!gps.update(CameraGpsPosition::Fixed, 1.8f, 0, 800, x, y) && x==before);
  ReflectorDockingGate gate;
  gate.update(false,true,true,x,y,3,0,0,true,800,.25f,2,0);
  assert(!gate.complete());
  assert(!gps.update(CameraGpsPosition::Fixed, 1, 0, 1000, x, y));
  assert(!gps.update(CameraGpsPosition::Fixed, 1, 0, 1200, x, y));
  assert(gps.update(CameraGpsPosition::Fixed, 1, 0, 1400, x, y));
  // Consistent fixes converge; duplicate samples cannot accelerate correction.
  const float duplicate=x;
  assert(!gps.update(CameraGpsPosition::Fixed, 1, 0, 1400, x, y) && x==duplicate);
  for(unsigned long t=1600;t<=12000;t+=200) gps.update(CameraGpsPosition::Fixed,1,0,t,x,y);
  assert(std::fabs(x-1)<1e-5);
  gate.update(false,true,true,x,y,3,0,.5f,true,12000,.25f,2,0);
  assert(gate.complete());
  assert(!gps.update(CameraGpsPosition::Fixed,1.1f,0,14000,x,y)); // stale: reacquire
  assert(!gps.update(CameraGpsPosition::Invalid,1.1f,0,14200,x,y));
  assert(!gps.update(CameraGpsPosition::Fixed,1.1f,0,14400,x,y));
  assert(!gps.update(CameraGpsPosition::Fixed,1.1f,0,14600,x,y));
  assert(gps.update(CameraGpsPosition::Fixed,1.1f,0,14800,x,y));
  const float valid=x;
  assert(!gps.update(CameraGpsPosition::Fixed,10,0,15000,x,y) && x==valid);
  assert(!gps.update(CameraGpsPosition::Fixed,std::numeric_limits<float>::quiet_NaN(),0,15200,x,y));
  gps.reset(); x=0; y=0;
  gps.update(CameraGpsPosition::Fixed,1,1,100,x,y); gps.update(CameraGpsPosition::Fixed,1,1,300,x,y);
  assert(gps.update(CameraGpsPosition::Fixed,1,1,500,x,y));
  assert(std::fabs(std::hypot(x,y)-.02f)<1e-6); // vector bound, not per axis
  // Float also converges, at half the maximum correction speed.
  gps.reset(); x=0; y=0;
  assert(!gps.update(CameraGpsPosition::Float,1,0,100,x,y));
  assert(!gps.update(CameraGpsPosition::Float,1,0,300,x,y));
  assert(gps.update(CameraGpsPosition::Float,1,0,500,x,y));
  assert(std::fabs(x-.01f)<1e-6 && gps.activeQuality(500)==CameraGpsPosition::Float);
  const float beforeFloatJump=x;
  assert(!gps.update(CameraGpsPosition::Float,1.8f,0,700,x,y) && x==beforeFloatJump);
  for(unsigned long t=900;t<40000;t+=200) gps.update(CameraGpsPosition::Float,1,0,t,x,y);
  assert(std::fabs(x-1)<.001f);
  const float settled=x;
  assert(!gps.update(CameraGpsPosition::Fixed,1.1f,0,40000,x,y) && x==settled);
  assert(!gps.update(CameraGpsPosition::Fixed,1.1f,0,40200,x,y));
  assert(gps.update(CameraGpsPosition::Fixed,1.1f,0,40400,x,y));
  assert(gps.activeQuality(40400)==CameraGpsPosition::Fixed);
  assert(gps.activeQuality(41401)==CameraGpsPosition::Invalid);
  assert(!gps.update(CameraGpsPosition::Float,1.1f,0,40600,x,y));
  assert(!gps.update(CameraGpsPosition::Invalid,1.1f,0,40800,x,y));
  assert(!gps.update(CameraGpsPosition::Float,1.1f,0,41000,x,y));
  assert(!gps.update(CameraGpsPosition::Float,1.1f,0,41200,x,y));
  assert(gps.update(CameraGpsPosition::Float,1.1f,0,41400,x,y));
  const float beforeInvalid=x;
  assert(!gps.update(CameraGpsPosition::Float,10,0,41600,x,y) && x==beforeInvalid);
  std::cout << "PASS camera GPS position: weighted float/fix convergence, quality changes, stale/jump rejection and bounded exit\n";
}
