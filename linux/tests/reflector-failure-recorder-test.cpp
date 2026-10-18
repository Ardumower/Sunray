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
 ReflectorStripsObservation o;
 {
  ReflectorFailureRecorder recorder(dir,3);
  // Startup darkness and a whole night without a marker cause no image writes.
  for(int i=0;i<100;++i)recorder.observe(bad,sizeof(bad),640,360,1000+i*60000,o,0,0);
 }
 assert(read(dir+"/frame-000.jpg").empty());
 {
  ReflectorFailureRecorder recorder(dir,3);
  for(int i=0;i<4;++i){
   o.found=true;o.stripCount=3;recorder.observe(good,sizeof(good),640,360,1000+i*1000,o,0,0);
   o.found=false;o.stripCount=0;recorder.observe(bad,sizeof(bad),640,360,1100+i*1000,o,.02f,.01f);
   char file[64];std::snprintf(file,sizeof(file),"/frame-%03d",i%3);
   waitFor(dir+file+".json","\"monotonic_ms\":"+std::to_string(1100+i*1000));
   assert(read(dir+file+".jpg")==std::string((char*)bad,sizeof(bad)));
   assert(read(dir+file+".json").find("\"before_loss\":false")!=std::string::npos);
  }
  // Persistent failure does not advance/overwrite the ring.
  for(int i=0;i<100;++i)recorder.observe(bad,sizeof(bad),640,360,5000+i*60000,o,0,0);
 }
 assert(read(dir+"/frame-000.json").find("\"monotonic_ms\":4100")!=std::string::npos);
 {
  ReflectorFailureRecorder recorder(dir,3);
  // Restart while still missing is not a detection-loss event.
  recorder.observe(bad,sizeof(bad),640,360,10000000,o,0,0);
 }
 assert(read(dir+"/frame-000.json").find("\"monotonic_ms\":4100")!=std::string::npos);
 for(int i=0;i<3;++i){char f[64];std::snprintf(f,sizeof(f),"/frame-%03d",i);unlink((dir+f+".jpg").c_str());unlink((dir+f+".json").c_str());}
 assert(rmdir(dir.c_str())==0);
 {ReflectorFailureRecorder recorder(dir+"/missing");o.found=true;recorder.observe(good,sizeof(good),640,360,1000,o,0,0);o.found=false;recorder.observe(bad,sizeof(bad),640,360,1100,o,0,0);}
 std::puts("PASS transition-only original JPEG, metadata, bounded rotation, startup/night silence, restart, unavailable storage");
}
