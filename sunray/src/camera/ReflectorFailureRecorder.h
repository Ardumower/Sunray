#pragma once
#ifdef __linux__
#include "ReflectorStripsDetector.h"
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/stat.h>

// Bounded asynchronous original-JPEG recorder. No disk I/O on the camera thread.
// Fixed slots bound disk usage across restarts; a full queue drops diagnostics,
// never camera frames. At most one image per 60 seconds on found -> missing,
// no periodic failure writes or deferred snapshots of suppressed transitions.
class ReflectorFailureRecorder {
  struct Frame {
    std::vector<uint8_t> jpeg;
    ReflectorStripsObservation observation;
    unsigned long monotonicMs=0;
    long long unixMs=0;
    int width=0,height=0;
    float linear=0,angular=0;
    bool beforeLoss=false;
  };
  std::string directory_;
  size_t slots_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Frame> queue_;
  bool stopping_=false;
  std::thread writer_;
  bool previousFound_=false,haveSaved_=false;
  unsigned long lastSaved_=0;
  static constexpr size_t maxJpegBytes=2*1024*1024;
  std::string path(size_t slot,const char* extension) const {
    char name[64];std::snprintf(name,sizeof(name),"/frame-%03zu.%s",slot,extension);
    return directory_+name;
  }
  void run() {
    if(mkdir(directory_.c_str(),0700)!=0){
      struct stat st{};
      if(stat(directory_.c_str(),&st)!=0 || !S_ISDIR(st.st_mode)){
        std::fprintf(stderr,"CAM failure recorder: cannot open %s\n",directory_.c_str());return;
      }
    }
    size_t slot=0;time_t newest=0;
    for(size_t i=0;i<slots_;++i){
      struct stat st{};
      if(stat(path(i,"jpg").c_str(),&st)==0 && st.st_mtime>=newest){newest=st.st_mtime;slot=(i+1)%slots_;}
    }
    for(;;){
      Frame f;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock,[this]{return stopping_ || !queue_.empty();});
        if(queue_.empty())return;
        f=std::move(queue_.front());queue_.pop_front();
      }
      const std::string jpg=path(slot,"jpg"),meta=path(slot,"json");
      // Remove stale metadata before replacing the corresponding image.
      std::remove(meta.c_str());
      FILE* file=std::fopen(jpg.c_str(),"wb");
      if(!file){std::fprintf(stderr,"CAM failure recorder: write failed\n");continue;}
      bool ok=std::fwrite(f.jpeg.data(),1,f.jpeg.size(),file)==f.jpeg.size();
      if(std::fclose(file)!=0)ok=false;
      if(!ok){std::remove(jpg.c_str());continue;}
      file=std::fopen(meta.c_str(),"w");
      if(file){
        std::fprintf(file,"{\"unix_ms\":%lld,\"monotonic_ms\":%lu,\"width\":%d,\"height\":%d,\"before_loss\":%s,\"found\":%s,\"strips\":%d,\"confidence\":%.5f,\"error\":%.5f,\"linear_set\":%.5f,\"angular_set\":%.5f}\n",
          f.unixMs,f.monotonicMs,f.width,f.height,f.beforeLoss?"true":"false",f.observation.found?"true":"false",
          f.observation.stripCount,f.observation.confidence,f.observation.horizontalError,f.linear,f.angular);
        std::fclose(file);
      }
      slot=(slot+1)%slots_;
    }
  }
public:
  explicit ReflectorFailureRecorder(const std::string& directory="camera_failures",size_t slots=128)
    :directory_(directory),slots_(slots?slots:1),writer_([this]{run();}){}
  ~ReflectorFailureRecorder(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}
    wake_.notify_one();writer_.join();
  }
  void observe(const uint8_t* jpeg,size_t size,int width,int height,unsigned long now,
               const ReflectorStripsObservation& observation,float linear,float angular){
    if(!jpeg || !size || size>maxJpegBytes)return;
    const bool lost=previousFound_ && !observation.found;
    previousFound_=observation.found;
    if(!lost || (haveSaved_ && now-lastSaved_<60000UL))return;
    Frame f;f.jpeg.assign(jpeg,jpeg+size);f.width=width;f.height=height;f.monotonicMs=now;
    f.unixMs=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    f.observation=observation;f.linear=linear;f.angular=angular;
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);
    if(!lock.owns_lock() || queue_.size()>=4)return;
    queue_.push_back(std::move(f));lastSaved_=now;haveSaved_=true;wake_.notify_one();
  }
};
#endif
