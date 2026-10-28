#pragma once
#ifdef __linux__
// Include before Arduino's min/max macros, or temporarily undefine them.
#include <atomic>
#include <cstdio>
#include <array>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>
#include <pthread.h>

// The network thread exclusively owns the transport. Robot commands are passed
// to the control thread, never executed by the network thread. One request at a
// time bounds backlog; old requests and all session data expire on disconnect.
class CloudWorker {
public:
  struct Transport {
    std::function<bool()> connect, connected;
    std::function<void()> close;
    std::function<bool(std::string&)> receive;
    std::function<bool(const std::string&)> send;
    std::function<bool(const unsigned char*, size_t)> binary;
    std::function<bool(const std::string&, std::string&)> auxiliary;
  };
  struct Request { std::string text; unsigned long long session=0; };
  explicit CloudWorker(Transport t, unsigned retryMs=2000): transport(std::move(t)), retryMs(retryMs) {}
  ~CloudWorker(){ stopAndJoin(); }
  void stopAndJoin(){ stop.store(true); wake.notify_all(); if(thread.joinable()) thread.join(); }
  void start(){ thread=std::thread([this]{run();}); }
  // Control/camera callbacks never wait for a mutex held by another thread.
  bool take(Request& out){
    std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
    if(!lock || !pending || taken || !online.load()) return false;
    if(Clock::now()-receivedAt>std::chrono::milliseconds(500)) { replyReady=true; response.clear(); pending=false; wake.notify_all(); return false; }
    out={incoming,generation}; taken=true; return true;
  }
  bool reply(unsigned long long session,const std::string& value){
    std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
    if(!lock) return false;
    if(session!=generation || !pending || !taken || !online.load()) return true; // discard obsolete reply
    response=value; replyReady=true; pending=false; wake.notify_all(); return true;
  }
  void camera(const unsigned char* data,size_t length){
    if(length>1024*1024 || !online.load()) return;
    std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
    // Keep one latest frame per normal/texture view, so back-to-back views
    // cannot overwrite one another while the network thread is busy.
    const bool texture=length>=16 && data[0]=='S' && data[1]=='R' &&
        data[2]=='B' && data[3]=='X' && (data[6]&0x80) && data[7]==101;
    if(lock && online.load()) {
      const int view=texture?1:0;if(!frames[view].empty())++overwritten;
      frames[view].assign(data,data+length);frameQueued[view]=Clock::now();
    }
  }
  bool connected() const {return online.load();}
private:
  using Clock=std::chrono::steady_clock;
  Transport transport;
  unsigned retryMs;
  std::thread thread;
  std::atomic<bool> stop{false},online{false};
  std::mutex mutex;
  std::condition_variable wake;
  unsigned long long generation=0;
  bool pending=false,taken=false,replyReady=false;
  std::string incoming,response;
  std::array<std::vector<unsigned char>,2> frames;
  Clock::time_point receivedAt;
  std::array<Clock::time_point,2> frameQueued;
  std::atomic<unsigned> overwritten{0};
  unsigned sentFrames=0;
  double queueMaxMs=0,sendMaxMs=0,commandWaitMaxMs=0;
  Clock::time_point lastVideoStats=Clock::now();
  bool flushFrames(){
    std::array<std::vector<unsigned char>,2> images;
    std::array<Clock::time_point,2> queued;
    {std::lock_guard<std::mutex> lock(mutex);images.swap(frames);queued=frameQueued;}
    for(size_t i=0;i<images.size();++i)if(!images[i].empty()){
      const auto started=Clock::now();
      const double queueMs=std::chrono::duration<double,std::milli>(started-queued[i]).count();
      if(queueMs>queueMaxMs)queueMaxMs=queueMs;
      if(!transport.binary(images[i].data(),images[i].size()))return false;
      const double sendMs=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
      if(sendMs>sendMaxMs)sendMaxMs=sendMs;++sentFrames;
    }
    if(Clock::now()-lastVideoStats>=std::chrono::seconds(10)){
      if(sentFrames)std::fprintf(stderr,"CAM cloud frames=%u queueMaxMs=%.1f sendMaxMs=%.1f commandWaitMaxMs=%.1f overwritten=%u\n",sentFrames,queueMaxMs,sendMaxMs,commandWaitMaxMs,overwritten.exchange(0));
      sentFrames=0;queueMaxMs=sendMaxMs=commandWaitMaxMs=0;lastVideoStats=Clock::now();
    }
    return true;
  }
  void clear(){
    online.store(false);
    std::lock_guard<std::mutex> lock(mutex);
    ++generation;pending=taken=replyReady=false;incoming.clear();response.clear();for(auto& frame:frames)frame.clear();
  }
  void wait(unsigned ms){std::unique_lock<std::mutex> lock(mutex);wake.wait_for(lock,std::chrono::milliseconds(ms),[this]{return stop.load();});}
  void run(){
    // A broken TLS socket must not terminate the robot process via SIGPIPE.
    sigset_t signals;sigemptyset(&signals);sigaddset(&signals,SIGPIPE);
    pthread_sigmask(SIG_BLOCK,&signals,nullptr);
    while(!stop.load()){
      if(!transport.connect()){ clear();transport.close();wait(retryMs);continue; }
      if(stop.load()) break;
      {std::lock_guard<std::mutex> lock(mutex);++generation;online.store(true);}
      auto lastReceive=Clock::now();
      while(!stop.load() && transport.connected()){
        std::string message;
        if(transport.receive(message) && !message.empty()){
          if(message.size()>65536) break;
          lastReceive=Clock::now();
          std::string answer;
          if(transport.auxiliary && transport.auxiliary(message,answer)){
            if(!answer.empty() && !transport.send(answer)) break;
            continue;
          }
          const auto commandStarted=Clock::now();
          {
            std::lock_guard<std::mutex> lock(mutex);
            incoming=std::move(message);receivedAt=commandStarted;pending=true;taken=false;replyReady=false;
          }
          bool answered=false,videoOk=true;
          while(!stop.load() && Clock::now()-commandStarted<std::chrono::seconds(2)){
            {
              std::unique_lock<std::mutex> lock(mutex);
              wake.wait_for(lock,std::chrono::milliseconds(5),[this]{return replyReady||stop.load();});
              if(replyReady){answer=std::move(response);replyReady=false;incoming.clear();answered=true;break;}
            }
            // A status/obstacle response may take time in the control thread.
            // Keep video moving on this same transport thread while it runs.
            if(!(videoOk=flushFrames()))break;
          }
          const double waited=std::chrono::duration<double,std::milli>(Clock::now()-commandStarted).count();
          if(waited>commandWaitMaxMs)commandWaitMaxMs=waited;
          if(stop.load() || !answered || !videoOk)break;
          if(!answer.empty() && !transport.send(answer)) break;
        }
        if(!flushFrames())break;
        if(Clock::now()-lastReceive>std::chrono::seconds(180)) break;
        wait(5);
      }
      clear();transport.close();wait(retryMs);
    }
    clear();transport.close();
  }
};
#endif
