#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

// One in-flight frame and one replaceable pending frame. Processing never holds
// the mailbox lock; camera capture cannot wait for image analysis.
template<class Frame> class LatestFrameWorker {
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false, pending_=false;
  Frame frame_;
  std::function<void(const Frame&)> process_;
  std::thread thread_;
  void run() {
    for (;;) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this]{return stopping_ || pending_;});
        if(stopping_)return;
        frame=std::move(frame_);pending_=false;
      }
      process_(frame);
    }
  }
public:
  explicit LatestFrameWorker(std::function<void(const Frame&)> process)
      :process_(std::move(process)),thread_([this]{run();}){}
  ~LatestFrameWorker(){stop();}
  void submit(Frame frame) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if(stopping_)return;
      frame_=std::move(frame);pending_=true;
    }
    wake_.notify_one();
  }
  void stop() {
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;pending_=false;}
    wake_.notify_one();
    if(thread_.joinable())thread_.join();
  }
};
