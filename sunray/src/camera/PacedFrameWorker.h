#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

// A fixed cadence, independent of capture arrival. Only the newest pending
// frame is consumed at a deadline; missing frames are never duplicated.
template<class Frame> class PacedFrameWorker {
  using Clock=std::chrono::steady_clock;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false,pending_=false,changed_=false;
  std::chrono::microseconds period_;
  Frame frame_;
  std::function<void(const Frame&)> process_;
  std::thread thread_;
  static std::chrono::microseconds period(int fps){return std::chrono::microseconds(1000000/(fps>0 && fps<=1000?fps:5));}
  void run(){
    std::unique_lock<std::mutex> lock(mutex_);
    auto step=period_;auto next=Clock::now()+step;
    while(!stopping_){
      wake_.wait_until(lock,next,[this]{return stopping_||changed_;});
      if(stopping_)return;
      if(changed_){changed_=false;step=period_;next=Clock::now()+step;continue;}
      Frame frame;const bool have=pending_;
      if(have){frame=std::move(frame_);pending_=false;}
      lock.unlock();
      if(have)process_(frame);
      lock.lock();
      // Preserve phase; skip missed deadlines after slow processing rather
      // than bursting old frames or adding processing time to every period.
      next+=step;
      const auto now=Clock::now();
      if(next<=now)next+=step*((now-next)/step+1);
    }
  }
public:
  PacedFrameWorker(int fps,std::function<void(const Frame&)> process)
    :period_(period(fps)),process_(std::move(process)),thread_([this]{run();}){}
  ~PacedFrameWorker(){stop();}
  void setRate(int fps){
    std::lock_guard<std::mutex> lock(mutex_);const auto value=period(fps);
    if(value!=period_){period_=value;changed_=true;wake_.notify_one();}
  }
  void submit(Frame frame){
    std::lock_guard<std::mutex> lock(mutex_);
    if(!stopping_){frame_=std::move(frame);pending_=true;}
  }
  void stop(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;pending_=false;}
    wake_.notify_one();if(thread_.joinable())thread_.join();
  }
};
