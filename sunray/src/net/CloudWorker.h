#pragma once
#ifdef __linux__
// Include before Arduino's min/max macros, or temporarily undefine them.
#include <atomic>
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
    if(lock && online.load()) frame.assign(data,data+length); // latest frame only
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
  std::vector<unsigned char> frame;
  Clock::time_point receivedAt;
  void clear(){
    online.store(false);
    std::lock_guard<std::mutex> lock(mutex);
    ++generation;pending=taken=replyReady=false;incoming.clear();response.clear();frame.clear();
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
          {
            std::unique_lock<std::mutex> lock(mutex);
            incoming=std::move(message);receivedAt=Clock::now();pending=true;taken=false;replyReady=false;
            wake.wait_for(lock,std::chrono::seconds(2),[this]{return replyReady||stop.load();});
            if(stop.load() || !replyReady) break;
            answer=std::move(response);replyReady=false;incoming.clear();
          }
          if(!answer.empty() && !transport.send(answer)) break;
        }
        std::vector<unsigned char> image;
        {std::lock_guard<std::mutex> lock(mutex);image.swap(frame);}
        if(!image.empty() && !transport.binary(image.data(),image.size())) break;
        if(Clock::now()-lastReceive>std::chrono::seconds(15)) break;
        wait(5);
      }
      clear();transport.close();wait(retryMs);
    }
    clear();transport.close();
  }
};
#endif
