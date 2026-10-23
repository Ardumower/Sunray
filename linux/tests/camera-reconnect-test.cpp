#include <functional>

#include <atomic>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
using namespace std::chrono;
struct ReflectorStripsDetector {
 std::atomic<int> clears{0};
 static ReflectorStripsDetector& instance(){static ReflectorStripsDetector d;return d;}
 void clear(){++clears;}
};
struct CameraStreamer {
 std::atomic<bool> running_{true},streamingRequested_{true},healthy{false};
 std::atomic<int> reqFps_{5},index{0},attempts{0},frames{0};
 int captureIndex(){return index.load();}
 void buildAndSendFrame(){++frames;}
 void runCaptureSession(){++attempts;while(healthy && running_)std::this_thread::sleep_for(milliseconds(5));}
 void runLoop();
};
#include "retry.inc"
static void until(std::function<bool()> ready,int ms){
 auto deadline=steady_clock::now()+milliseconds(ms);
 while(!ready() && steady_clock::now()<deadline)std::this_thread::sleep_for(milliseconds(5));
 assert(ready());
}
struct MMapBuffer {void* start=nullptr;size_t length=0;};
static int unmapped=0,streamoff=0;
static int unmapSpy(void*,size_t){++unmapped;return 0;}
static int ioctl(int,int,void*){++streamoff;return 0;}
#define V4L2_BUF_TYPE_VIDEO_CAPTURE 1
#define VIDIOC_STREAMOFF 2
#define munmap unmapSpy
#include "resources.inc"
#undef munmap
int main(){
 CameraStreamer c;
 auto start=steady_clock::now();std::thread worker([&]{c.runLoop();});
 until([&]{return c.attempts==1;},500);
 std::this_thread::sleep_for(milliseconds(300));assert(c.attempts==1);
 until([&]{return c.attempts>=2;},2200);
 assert(steady_clock::now()-start>=seconds(2));
 c.healthy=true;until([&]{return c.attempts>=3;},2300);
 assert(c.frames>0 && ReflectorStripsDetector::instance().clears>=2);
 int n=c.attempts;std::this_thread::sleep_for(milliseconds(200));assert(c.attempts==n);
 c.healthy=false;until([&]{return ReflectorStripsDetector::instance().clears>=3;},500);
 c.streamingRequested_=false;int frames=c.frames;
 c.index=1;c.healthy=true;until([&]{return c.attempts>n;},500);
 assert(c.frames==frames);c.running_=false;worker.join();
 CameraStreamer stopped;stopped.streamingRequested_=false;
 std::thread waiting([&]{stopped.runLoop();});until([&]{return stopped.attempts==1;},500);
 auto stop=steady_clock::now();stopped.running_=false;waiting.join();
 assert(steady_clock::now()-stop<milliseconds(300));assert(stopped.frames==0);
 int pipes[2];assert(pipe(pipes)==0);
 {CaptureResources r(pipes[0]);r.buffers.resize(3);r.buffers[0].start=(void*)1;r.buffers[0].length=4096;}
 assert(fcntl(pipes[0],F_GETFD)==-1 && unmapped==1 && streamoff==0);close(pipes[1]);
 assert(pipe(pipes)==0);{CaptureResources r(pipes[0]);r.streaming=true;}
 assert(fcntl(pipes[0],F_GETFD)==-1 && streamoff==1);close(pipes[1]);
 puts("PASS production retry loop: 2s retry, delayed camera, active session, disconnect, index switch, prompt stop, subscriber-only fallback, marker clear, partial setup cleanup");
}
