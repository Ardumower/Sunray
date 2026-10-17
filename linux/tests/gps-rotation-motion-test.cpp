#include <cassert>
#include <cmath>
#include <cstdio>
#define GPS_MOTION_DETECTION true
#define LOC_GPS 1
#define EVT_NO_ROBOT_MOTION_OBSTACLE 1
unsigned long now=1000,nextGPSMotionCheckTime=0,overallMotionTimeout=0;
unsigned long millis(){return now;}
float lastGPSMotionX=0,lastGPSMotionY=0;
struct {float stateX=10,stateY=10;int stateLocalizationMode=LOC_GPS;} stateEstimator;
struct {int statMowGPSMotionTimeoutCounter=0;} stats;
struct {void println(const char*){}} CONSOLE;
struct {void event(int){}} Logger;
bool rotating=true,obstacle=false;
bool robotShouldRotate(){return rotating;}
void updateGPSMotionCheckTime(){nextGPSMotionCheckTime=now+10000;}
void resetOverallMotionTimeout(){overallMotionTimeout=now+10000;}
void triggerObstacle(){obstacle=true;}
float sq(float v){return v*v;}
bool check(){
#include "gps-motion-production.inc"
 return false;
}
int main(){
 // A slow full turn can take over a minute without translating.
 for(now=1000;now<=180000;now+=1000){assert(!check());assert(!obstacle);}
 rotating=false;
 now=180001;assert(!check());
 now=189999;assert(!check());
 now=190001;assert(check());assert(obstacle); // blocked translation still trips
 obstacle=false;stateEstimator.stateX+=.2f;now=200002;assert(!check());assert(!obstacle);
 std::puts("PASS production GPS motion: prolonged rotation, fresh translation baseline, blocked/moving translation");
}
