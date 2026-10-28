#pragma once
#include "src/navigation/CameraAvoidance.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
namespace CameraAvoidance {
// Owned geometry only: the planning thread must never dereference Map/Arduino
// state or a lambda capturing the live control stack.
struct MapSnapshot {
 Polygon perimeter;
 std::vector<Polygon> forbidden;
 bool entry=false;
 Point entryStart,entryEnd;
 double entryLimit=0;
 bool allows(Point p) const {
  if(perimeter.size()<3)return false;
  if(!inside(p,perimeter) && !(entry && entryCorridor(p,entryStart,entryEnd,entryLimit)))return false;
  for(const auto& polygon:forbidden)if(inside(p,polygon))return false;
  return true;
 }
 uint64_t fingerprint() const {
  uint64_t h=1469598103934665603ULL;
  auto number=[&](int64_t n){h^=uint64_t(n);h*=1099511628211ULL;};
  auto point=[&](Point p){number(int64_t(std::llround(p.x*10000)));number(int64_t(std::llround(p.y*10000)));};
  auto polygon=[&](const Polygon& p){number(p.size());for(auto q:p)point(q);};
  polygon(perimeter);number(forbidden.size());for(const auto& p:forbidden)polygon(p);
  number(entry);if(entry){point(entryStart);point(entryEnd);number(int64_t(std::llround(entryLimit*10000)));}
  return h;
 }
};
struct PlanRequest {
 uint64_t id=0,epoch=0,created=0,mapVersion=0;
 Pose pose;
 Point goal,lineStart,rejoinGoal;
 bool hasLine=false,following=false,rejoining=false;
 Planner planner;
 MapSnapshot map;
};
struct PlanResult {
 PlanRequest input;
 bool found=false,discard=false,failed=false;
 Point navigationGoal;
 double computeMs=0,queueMs=0;
};
inline uint64_t planningNow(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline PlanResult solveCameraPlan(PlanRequest request){
 PlanResult result;result.input=std::move(request);auto& r=result.input;auto& p=r.planner;
 const auto started=std::chrono::steady_clock::now();result.queueMs=planningNow()-r.created;
 auto allowed=[&](Point q){return r.map.allows(q);};
 Point aim=r.goal;
 if(r.hasLine && (segmentDistance({r.pose.x,r.pose.y},r.lineStart,r.goal)>.20 || r.following || r.rejoining)){
  if(!r.rejoining || distance({r.pose.x,r.pose.y},r.rejoinGoal)<1.0)r.rejoinGoal=p.rejoinPoint(r.pose,r.lineStart,r.goal);
  r.rejoining=true;aim=r.rejoinGoal;
 }
 bool found=p.plan(r.pose,aim,r.created,allowed);
 if(r.hasLine && !r.rejoining && p.active){
  r.rejoinGoal=p.rejoinPoint(r.pose,r.lineStart,r.goal);r.rejoining=true;aim=r.rejoinGoal;
  found=p.plan(r.pose,aim,r.created,allowed);
 }
 for(int attempt=0;!found && p.noRoute && r.rejoining && distance(aim,r.goal)>.1 && attempt<3;++attempt){
  const double remaining=distance(r.rejoinGoal,r.goal),step=std::min(2.,remaining);
  r.rejoinGoal={r.rejoinGoal.x+(r.goal.x-r.rejoinGoal.x)*step/remaining,r.rejoinGoal.y+(r.goal.y-r.rejoinGoal.y)*step/remaining};
  aim=r.rejoinGoal;found=p.plan(r.pose,aim,r.created,allowed);
 }
 if(!found && p.contains({r.pose.x,r.pose.y}))found=p.planEscape(r.pose,allowed);
 result.discard=!found && p.noRoute && distance(aim,r.goal)<.1 && allowed({r.pose.x,r.pose.y}) && !p.contains({r.pose.x,r.pose.y});
 result.found=found;result.navigationGoal=aim;
 r.following=found && (p.active || r.following || r.rejoining || !p.obstacles.empty());
 result.computeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
 return result;
}
class CameraPlanWorker {
public:
 using Solver=std::function<PlanResult(PlanRequest)>;
 explicit CameraPlanWorker(Solver solve=solveCameraPlan):solve(std::move(solve)),thread([this]{run();}){}
 ~CameraPlanWorker(){stopping.store(true);wake.notify_all();if(thread.joinable())thread.join();}
 CameraPlanWorker(const CameraPlanWorker&)=delete;
 CameraPlanWorker& operator=(const CameraPlanWorker&)=delete;
 void invalidate(uint64_t value){epoch.store(value);}
 bool submit(std::shared_ptr<PlanRequest> request){
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);if(!lock)return false;
  pending=std::move(request);wake.notify_one();return true;
 }
 std::shared_ptr<PlanResult> take(){
  std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);if(!lock)return {};
  auto r=std::move(completed);return r;
 }
private:
 Solver solve;
 std::atomic<bool> stopping{false};
 std::atomic<uint64_t> epoch{0};
 std::mutex mutex;
 std::condition_variable wake;
 std::shared_ptr<PlanRequest> pending;
 std::shared_ptr<PlanResult> completed;
 std::thread thread;
 void run(){
  while(!stopping.load()){
   std::shared_ptr<PlanRequest> request;
   {std::unique_lock<std::mutex> lock(mutex);wake.wait(lock,[&]{return stopping.load()||pending;});if(stopping.load())break;request=std::move(pending);}
   if(request->epoch!=epoch.load())continue;
   const auto requestEpoch=request->epoch;
   std::shared_ptr<PlanResult> result;
   try{result=std::make_shared<PlanResult>(solve(std::move(*request)));}
   catch(...){result=std::make_shared<PlanResult>();result->input.id=request->id;result->input.epoch=requestEpoch;result->failed=true;}
   if(requestEpoch!=epoch.load())continue;
   {std::lock_guard<std::mutex> lock(mutex);completed=std::move(result);}
  }
 }
};
}
