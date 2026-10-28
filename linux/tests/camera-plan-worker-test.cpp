#include "CameraPlanWorker.h"
#include <cassert>
#include <iostream>
using namespace CameraAvoidance;
int main(){
 std::atomic<bool> entered{false},release{false};std::atomic<int> calls{0};
 CameraPlanWorker worker([&](PlanRequest request){
  ++calls;entered.store(true);
  while(!release.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
  PlanResult r;r.input=std::move(request);r.found=true;return r;
 });
 worker.invalidate(1);
 auto first=std::make_shared<PlanRequest>();first->epoch=1;first->id=1;assert(worker.submit(first));
 auto deadline=planningNow()+1000;while(!entered.load()&&planningNow()<deadline)std::this_thread::yield();assert(entered.load());
 const auto started=std::chrono::steady_clock::now();
 for(int i=2;i<=100;++i){auto q=std::make_shared<PlanRequest>();q->epoch=1;q->id=i;worker.submit(q);assert(!worker.take());}
 const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
 assert(ms<20); // the blocked solver cannot hold the control-thread mailbox
 worker.invalidate(2);auto newest=std::make_shared<PlanRequest>();newest->epoch=2;newest->id=101;assert(worker.submit(newest));
 release.store(true);std::shared_ptr<PlanResult> result;
 deadline=planningNow()+1000;
 while(planningNow()<deadline){result=worker.take();if(result)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 assert(result&&result->input.id==101&&result->input.epoch==2);assert(calls.load()==2);
 // Solve only owned geometry; later modifications to the live snapshot cannot
 // change a request already copied for the worker.
 PlanRequest request;request.epoch=2;request.created=planningNow();request.goal={7,0};
 request.map.perimeter={{-2,-8},{12,-8},{12,8},{-2,8}};
 request.planner.obstacles.push_back({{{2,-1},{4,-1},{4,1},{2,1}},1});
 auto copy=request;request.map.perimeter.clear();auto solved=solveCameraPlan(std::move(copy));
 assert(solved.found&&solved.input.planner.route.size()>1);
 assert(!request.map.allows({0,0}));assert(solved.input.map.allows({0,0}));
 std::cout<<"PASS nonblocking mailbox ("<<ms<<" ms), latest request only, epoch invalidation and owned map snapshots\n";
}
