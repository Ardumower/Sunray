#include "src/camera/PacedFrameWorker.h"
#include <atomic>
#include <cassert>
#include <iostream>
#include <vector>
using Clock=std::chrono::steady_clock;
int main(){
 std::vector<Clock::time_point> times;std::vector<int> frames;
 PacedFrameWorker<int> worker(20,[&](const int& frame){times.push_back(Clock::now());frames.push_back(frame);});
 const auto start=Clock::now();int sequence=0;
 while(Clock::now()-start<std::chrono::seconds(1)){
  worker.submit(++sequence);worker.setRate(20); // Same subscription must not reset phase.
  std::this_thread::sleep_for(std::chrono::milliseconds(19));
 }
 std::this_thread::sleep_for(std::chrono::milliseconds(120));worker.stop();
 assert(times.size()>=18 && times.size()<=22);
 for(size_t i=1;i<times.size();++i){assert(frames[i]>frames[i-1]);assert(times[i]-times[i-1]<std::chrono::milliseconds(90));}
 assert(frames.back()==sequence);
 std::atomic<int> calls{0};std::atomic<bool> release{false},entered{false};
 PacedFrameWorker<int> blocked(50,[&](const int&){++calls;entered=true;while(!release.load())std::this_thread::yield();});
 blocked.submit(1);while(!entered)std::this_thread::yield();
 const auto began=Clock::now();for(int i=2;i<1000;++i)blocked.submit(i);
 assert(Clock::now()-began<std::chrono::milliseconds(100));
 release=true;std::this_thread::sleep_for(std::chrono::milliseconds(100));blocked.stop();assert(calls==2);
 std::atomic<int> changedCalls{0};
 PacedFrameWorker<int> changed(1,[&](const int&){++changedCalls;});
 changed.submit(1);changed.setRate(50);
 std::this_thread::sleep_for(std::chrono::milliseconds(80));changed.stop();assert(changedCalls==1);
 std::cout<<"PASS fixed cadence with irregular capture, latest-only delivery, no duplicate images, nonblocking capture\n";
}
