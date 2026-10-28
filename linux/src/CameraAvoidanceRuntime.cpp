#include "CameraAvoidanceRuntime.h"
#include "src/camera/GrassOverlay.h"
#include <chrono>
#include <deque>
#include <cstdio>
#include "config.h"
#undef min
#undef max
#ifndef CAMERA_AVOIDANCE_HEIGHT_M
#define CAMERA_AVOIDANCE_HEIGHT_M .4
#endif
#ifndef CAMERA_AVOIDANCE_FORWARD_M
#define CAMERA_AVOIDANCE_FORWARD_M .5
#endif
#ifndef CAMERA_AVOIDANCE_HFOV_DEG
#define CAMERA_AVOIDANCE_HFOV_DEG 75
#endif
#ifndef CAMERA_AVOIDANCE_DOWN_PITCH_DEG
#define CAMERA_AVOIDANCE_DOWN_PITCH_DEG 0
#endif
#ifndef CAMERA_AVOIDANCE_WIDTH_M
#define CAMERA_AVOIDANCE_WIDTH_M .8
#endif
#ifndef CAMERA_AVOIDANCE_LENGTH_M
#define CAMERA_AVOIDANCE_LENGTH_M 1.0
#endif
namespace CameraAvoidanceRuntime {
using namespace CameraAvoidance;
static Planner planner;
static std::unique_ptr<CameraPlanWorker> worker;
static bool on=false,following=false,discardCurrent=false,rejoining=false;
static Point currentGoal,rejoinGoal,navigationGoal,currentLine;
static bool hasLine=false;
static double pathDeviation=0,lastAngular=0,maxControlMs=0,lastPlanMs=0,lastQueueMs=0;
static uint64_t stamp=0,lastRun=0,lastLog=0,lastSubmit=0,epoch=0,requestId=0,mapVersion=0,acceptedId=0;
static unsigned staleResults=0;
struct Sample{uint64_t time;Pose pose;};
static std::deque<Sample> history;
static uint64_t nowMs(){return planningNow();}
static void invalidate(){++epoch;if(worker)worker->invalidate(epoch);lastSubmit=0;acceptedId=0;planner.route.clear();following=false;rejoining=false;discardCurrent=false;}
bool supported(){
#ifdef SUNRAY_RKNN
 return true;
#else
 return false;
#endif
}
bool enabled(){return on;}
void setEnabled(bool value){
 on=value&&supported();if(on&&!worker)worker.reset(new CameraPlanWorker());
 invalidate();planner.reset();stamp=0;history.clear();lastRun=0;pathDeviation=0;lastAngular=0;mapVersion=0;
}
void idle(){if(lastRun||following||!planner.route.empty())invalidate();lastRun=0;lastAngular=0;history.clear();pathDeviation=0;}
bool discardTarget(){return on&&discardCurrent;}
bool active(){return on&&following;}
double deviation(){return pathDeviation;}
const std::vector<Obstacle>& obstacles(){return planner.obstacles;}
bool targetBlocked(Point p){return on&&planner.contains(p);}
void update(Pose pose,Point goal,double desired,const MapSnapshot& map,float& linear,float& angular,const Point* lineStart){
 if(!on)return;
 const auto began=std::chrono::steady_clock::now();
 struct Measure {std::chrono::steady_clock::time_point start;~Measure(){maxControlMs=std::max(maxControlMs,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());}} measure{began};
 discardCurrent=false;const auto now=nowMs();
 const auto version=map.fingerprint();
 const bool lineChanged=hasLine!=bool(lineStart) || (lineStart&&distance(currentLine,*lineStart)>.001);
 if((lastRun&&now-lastRun>2000)||distance(currentGoal,goal)>.001||version!=mapVersion||lineChanged){
  invalidate();currentGoal=goal;mapVersion=version;hasLine=bool(lineStart);if(lineStart)currentLine=*lineStart;
 }
 pathDeviation=0;
 if(following && !planner.route.empty()){
  pathDeviation=1e9;Point from=planner.routeStart;
  for(auto pt:planner.route){pathDeviation=std::min(pathDeviation,segmentDistance({pose.x,pose.y},from,pt));from=pt;}
 }
 // Assess deviation from the accepted path before accepting any new path.
 if(pathDeviation>1.0){linear=angular=0;return;}
 const double dt=lastRun?std::min(.2,(now-lastRun)/1000.):.02;lastRun=now;
 planner.cfg.height=CAMERA_AVOIDANCE_HEIGHT_M;planner.cfg.forward=CAMERA_AVOIDANCE_FORWARD_M;
 planner.cfg.hfov=CAMERA_AVOIDANCE_HFOV_DEG;planner.cfg.downPitch=CAMERA_AVOIDANCE_DOWN_PITCH_DEG;
 planner.cfg.width=CAMERA_AVOIDANCE_WIDTH_M;planner.cfg.length=CAMERA_AVOIDANCE_LENGTH_M;
 history.push_back({now,pose});while(history.size()>150)history.pop_front();
 std::array<uint8_t,3600> labels;
 if(GrassOverlay::mask(labels,stamp)&&now>=stamp&&now-stamp<1000){
  auto capture=history.front();for(auto sample:history){if(sample.time>stamp)break;capture=sample;}
  if(capture.time<=stamp+50)planner.observe(labels.data(),stamp,capture.pose);
 }
 const bool ready=GrassOverlay::available()&&planner.observationStamp()&&now>=planner.observationStamp()&&now-planner.observationStamp()<1000;
 auto allowed=[&](Point p){return map.allows(p);};
 planner.setControlPose(pose);
 if(auto result=worker->take()){
  const auto& r=result->input;
  bool valid=!result->failed && r.epoch==epoch && r.id>acceptedId && r.mapVersion==mapVersion &&
      distance(r.goal,goal)<.001 && now>=r.created && now-r.created<=1000 &&
      distance({r.pose.x,r.pose.y},{pose.x,pose.y})<1.0 && ready;
  // A negative answer must refer to current perception and a nearly unchanged
  // start. A positive path is instead revalidated against the freshest contours.
  if(valid && !result->found)valid=r.planner.revision==planner.revision && distance({r.pose.x,r.pose.y},{pose.x,pose.y})<.15;
  if(valid && result->found){
   Planner candidate=planner;candidate.adoptPath(r.planner);candidate.setControlPose(pose);
   valid=candidate.visibleRoute(pose,allowed);
  }
  if(valid){
   planner.adoptPath(r.planner);planner.setControlPose(pose);acceptedId=r.id;
   rejoining=r.rejoining;rejoinGoal=r.rejoinGoal;navigationGoal=result->navigationGoal;
   following=result->found&&r.following;
   discardCurrent=result->discard;
   lastPlanMs=result->computeMs;lastQueueMs=result->queueMs;
  }else ++staleResults;
 }
 const bool usable=ready && planner.visibleRoute(pose,allowed);
 if(!usable){linear=angular=0;lastAngular=0;}
 else if(following || rejoining || !planner.obstacles.empty()){
  following=true;planner.steer(pose,desired,dt,allowed,lastAngular,linear,angular);
 }else lastAngular=angular;
 // One latest request replaces any waiting request. No control-thread waits,
 // no Arduino/map references, no unbounded planning backlog.
 const uint64_t interval=usable?200:50;
 if(ready && !discardCurrent && (!lastSubmit || now-lastSubmit>=interval)){
  auto request=std::make_shared<PlanRequest>();request->id=++requestId;request->epoch=epoch;
  request->created=now;request->mapVersion=mapVersion;request->pose=pose;request->goal=goal;
  request->hasLine=bool(lineStart);if(lineStart)request->lineStart=*lineStart;
  request->following=following;request->rejoining=rejoining;request->rejoinGoal=rejoinGoal;
  request->planner=planner;request->map=map;
  if(worker->submit(std::move(request)))lastSubmit=now;
 }
 if(now-lastLog>2000){
  std::printf("CAM avoidance async ready=%d detour=%d blocked=%d obstacles=%zu route=%zu lin=%.3f ang=%.3f rejoin=%d aim=%.2f,%.2f overlap=%d controlMaxMs=%.2f planMs=%.2f queueMs=%.0f stale=%u\n",
   ready,planner.active,!usable,planner.obstacles.size(),planner.route.size(),linear,angular,rejoining,navigationGoal.x,navigationGoal.y,
   planner.contains({pose.x,pose.y}),maxControlMs,lastPlanMs,lastQueueMs,staleResults);
  maxControlMs=0;lastLog=now;
 }
}
}
