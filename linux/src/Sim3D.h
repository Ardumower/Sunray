#pragma once
#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max
#include <cstdint>
#include <string>
#include <vector>
namespace Sim3D {
bool enabled();
std::string robotId();
struct State {double x=0,y=0,yaw=0,leftTicks=0,rightTicks=0,linear=0,angular=0,roll=0,pitch=0;bool bumper=false,charging=false;};
void command(double left,double right,double mow,double wheelbase,double ticksPerMeter,unsigned flags);
void teleport(double x,double y,double yaw);
void obstacle(double x,double y,double radius);
bool poll(State& state);
// Atomic latest-frame file in tmpfs; stale and repeated frames are rejected.
bool frame(std::vector<uint8_t>& jpeg,uint64_t& sequence,uint64_t& ageMs);
}

#pragma pop_macro("max")
#pragma pop_macro("min")
