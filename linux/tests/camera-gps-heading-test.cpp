#include "src/navigation/CameraGpsHeading.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>

float angleError(float a,float b) { return std::atan2(std::sin(a-b),std::cos(a-b)); }
int main() {
  for (int reverse=0;reverse<2;++reverse) {
    CameraGpsHeading h; float yaw=-1.8f; int corrections=0;
    const float heading=2.77f, sign=reverse ? -1.f : 1.f;
    for(unsigned long i=0;i<400;++i) {
      const float travel=i*.004f;
      const float before=yaw;
      if(h.update(true,sign*travel*std::cos(heading),sign*travel*std::sin(heading),
                  travel,sign*.02f,0,i*200,yaw)) ++corrections;
      assert(std::fabs(angleError(yaw,before))<=.5236f);
    }
    assert(corrections>=4 && std::fabs(angleError(yaw,heading))<.001f);
  }
  // Standstill drift, float, turning and zero wheel movement cannot supply yaw.
  for(int mode=0;mode<4;++mode) {
    CameraGpsHeading h; float yaw=1;
    for(unsigned long i=0;i<200;++i)
      assert(!h.update(mode!=0,i*.004f,0,mode==3?0:i*.004f,
                       mode==1?0:.02f,mode==2?.2f:0,i*200,yaw));
    assert(yaw==1);
  }
  CameraGpsHeading h; float yaw=1;
  for(unsigned long i=0;i<50;++i) h.update(true,i*.004f,0,i*.004f,.02f,0,i*200,yaw);
  assert(!h.update(true,1,0,.2f,.02f,0,10000,yaw) && yaw==1); // jump resets baseline
  assert(!h.update(true,1.01f,0,.21f,.02f,0,12000,yaw)); // stale
  assert(!h.update(true,1.02f,0,.22f,-.02f,0,12200,yaw)); // direction change
  assert(!h.update(false,1.03f,0,.23f,-.02f,0,12400,yaw));
  assert(!h.update(true,std::numeric_limits<float>::quiet_NaN(),0,.24f,.02f,0,12600,yaw));
  // Wraparound takes the short correction, not a full turn.
  h.reset(); yaw=3.12f;
  for(unsigned long i=0;i<100;++i) {
    float d=i*.004f; h.update(true,d*std::cos(-3.12f),d*std::sin(-3.12f),d,.02f,0,i*200,yaw);
  }
  assert(std::fabs(angleError(yaw,-3.12f))<.001f);
  std::cout << "PASS camera heading: forward/reverse, wrong initial yaw, angular bound, wrap, standstill/float/turn/jump/stale rejection\n";
}
