#include "src/camera/ReflectorFailureRecorder.h"
#include <cassert>
#include <fstream>
#include <iterator>
#include <unistd.h>
static std::string read(const std::string& path){std::ifstream f(path);return {std::istreambuf_iterator<char>(f),std::istreambuf_iterator<char>()};}
static void waitFor(const std::string& path,const std::string& text){
 for(int i=0;i<200;++i){if(read(path).find(text)!=std::string::npos)return;std::this_thread::sleep_for(std::chrono::milliseconds(5));}
 assert(false && "recorder output timeout");
}
int main(){
 char name[]="/tmp/reflector-recorder-XXXXXX";assert(mkdtemp(name));std::string dir=name;
 uint8_t good[]={0xff,0xd8,1,0xff,0xd9},bad[]={0xff,0xd8,2,0xff,0xd9};
 ReflectorStripsObservation o;o.found=true;o.stripCount=3;
 {
  ReflectorFailureRecorder recorder(dir,3);
  recorder.observe(good,sizeof(good),640,360,1000,o,.02f,.01f);
  o.found=false;o.stripCount=0;
  recorder.observe(bad,sizeof(bad),640,360,1100,o,.02f,.01f);
  waitFor(dir+"/frame-001.json","\"monotonic_ms\":1100");
  assert(read(dir+"/frame-000.jpg")==std::string((char*)good,sizeof(good)));
  assert(read(dir+"/frame-001.jpg")==std::string((char*)bad,sizeof(bad)));
  assert(read(dir+"/frame-000.json").find("\"before_loss\":true")!=std::string::npos);
  recorder.observe(bad,sizeof(bad),640,360,1200,o,0,0);
  recorder.observe(bad,sizeof(bad),640,360,2200,o,0,0);
  waitFor(dir+"/frame-002.json","\"monotonic_ms\":2200");
  recorder.observe(bad,sizeof(bad),640,360,3300,o,0,0);
  waitFor(dir+"/frame-000.json","\"monotonic_ms\":3300");
 }
 {
  ReflectorFailureRecorder recorder(dir,3);
  recorder.observe(bad,sizeof(bad),640,360,4400,o,0,0);
 }
 for(int i=0;i<3;++i){char f[64];std::snprintf(f,sizeof(f),"/frame-%03d",i);unlink((dir+f+".jpg").c_str());unlink((dir+f+".json").c_str());}
 assert(rmdir(dir.c_str())==0);
 // An unavailable directory must not interrupt or block camera processing.
 {ReflectorFailureRecorder recorder(dir+"/missing");for(int i=0;i<20;++i)recorder.observe(bad,sizeof(bad),640,360,1000+i*1100,o,0,0);}
 std::puts("PASS original JPEG, pre-loss frame, metadata, throttle, bounded rotation, restart, unavailable storage");
}
