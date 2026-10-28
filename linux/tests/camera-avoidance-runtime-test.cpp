#include "CameraAvoidanceRuntime.h"
#include "src/camera/GrassOverlay.h"
#include <cassert>
#include <chrono>
#include <thread>
#include <iostream>
static uint64_t now(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static bool supply=false;
static uint64_t suppliedStamp=0;
bool GrassOverlay::available(){return true;}
bool GrassOverlay::mask(std::array<uint8_t,3600>& labels,uint64_t& stamp){if(!supply)return false;labels.fill(1);if(!suppliedStamp || now()-suppliedStamp>100)suppliedStamp=now();stamp=suppliedStamp;return true;}
int main(){
 using namespace CameraAvoidanceRuntime;
 CameraAvoidance::MapSnapshot allowed;allowed.perimeter={{-10,-10},{110,-10},{110,10},{-10,10}};
 float v=.3,w=.1;
 update({}, {5,0},.3,allowed,v,w);assert(v==.3f&&w==.1f);
 setEnabled(true);update({}, {5,0},.3,allowed,v,w);assert(v==0&&w==0);
 supply=true;
 // Polling an asynchronous result must not block the control update.
 auto waitFor=[&](const std::function<bool()>& tick){
  auto deadline=now()+1500;while(now()<deadline){if(tick())return true;std::this_thread::sleep_for(std::chrono::milliseconds(2));}return false;
 };
 assert(waitFor([&]{v=.3;w=0;update({}, {5,0},.3,allowed,v,w);return v>0;}));
 supply=false;std::this_thread::sleep_for(std::chrono::milliseconds(1100));
 update({}, {5,0},.3,allowed,v,w);assert(v==0&&w==0);assert(!discardTarget());
 setEnabled(true);supply=true;
 CameraAvoidance::MapSnapshot disconnected;disconnected.perimeter={{-10,-10},{1,-10},{1,10},{-10,10}};
 assert(waitFor([&]{update({}, {5,0},.3,disconnected,v,w);return discardTarget();}));
 update({}, {.5,0},.3,disconnected,v,w);assert(!discardTarget());
 update({2,0,0},{5,0},.3,disconnected,v,w);assert(!discardTarget());
 setEnabled(true);CameraAvoidance::Point lineStart{0,0};
 assert(waitFor([&]{update({0,.5,0},{100,0},.3,disconnected,v,w,&lineStart);return discardTarget();}));
 // Disabling must invalidate a result that was submitted just before stopping.
 setEnabled(true);update({}, {5,0},.3,allowed,v,w);setEnabled(false);
 std::this_thread::sleep_for(std::chrono::milliseconds(20));v=.12;w=.02;
 update({}, {5,0},.3,allowed,v,w);assert(v==.12f&&w==.02f);
 setEnabled(false);assert(!enabled());assert(obstacles().empty());
 std::cout<<"PASS default-off, enable, fresh perception, stale stop and disable reset\n";
}
