// Opt-in test against the Godot world only. Stop sunray-sim3d first.
#include "src/navigation/CameraAvoidance.h"
#include "src/camera/GrassOverlay.h"
#include "src/camera/SafeJpeg.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fstream>
#include <cstring>
#include <deque>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
using namespace CameraAvoidance;
static uint64_t now(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv){
 if((argc!=4&&argc!=5) || std::system("systemctl is-active --quiet sunray-sim3d")==0){std::cerr<<"Requires stopped simulator and original x y yaw as arguments\n";return 2;}
 int fd=socket(AF_INET,SOCK_DGRAM,0);sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(19091);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);connect(fd,(sockaddr*)&a,sizeof(a));
 timeval timeout{1,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
 unsigned seq=0;bool molehill=argc==5;const double sx=molehill?45:42,sy=molehill?-12:-10;const Point goal{molehill?51.:50.,sy};Pose pose{sx,sy,0};
 auto command=[&](double v,double w,bool restore=false){
  std::ostringstream s;s<<"S3D1 "<<++seq<<" "<<v-w*.24<<" "<<v+w*.24<<" 0 .48 1346.7 0 "<<(restore?2:1)<<" "<<(restore?std::string(argv[1]):std::to_string(sx))<<" "<<(restore?std::string(argv[2]):std::to_string(sy))<<" "<<(restore?argv[3]:"0")<<" "<<(restore?2:1)<<" 45 -10 "<<(restore||molehill?0:.6);
  auto msg=s.str();send(fd,msg.data(),msg.size(),0);
  for(int tries=0;tries<100;tries++){
   char buf[512];auto n=recv(fd,buf,sizeof(buf)-1,0);if(n<0)return false;buf[n]=0;
   std::istringstream in(buf);std::string magic;unsigned reply;double enc1,enc2,linear,angular;int bump;Pose current;
   in>>magic>>reply>>current.x>>current.y>>current.yaw>>enc1>>enc2>>linear>>angular>>bump;
   if(magic!="S3D1"||reply!=seq)continue;pose=current;
   if(!restore&&bump){std::cerr<<"COLLISION\n";return false;}return true;
  }return false;
 };
 command(0,0);Planner planner;GrassOverlay grass;uint64_t stamp=0,frameSeq=0;double turn=0;bool arrived=false,healthy=true;int pauses=0;
 auto allowed=[](Point p){return p.x>39&&p.x<53&&p.y>-16&&p.y<-6;};
 std::deque<std::pair<uint64_t,Pose>> history;
 const auto start=now();
 for(int tick=0;tick<900;tick++){
  history.push_back({now(),pose});while(history.size()>100)history.pop_front();
  std::ifstream file("/run/sunray-sim3d/camera.frame",std::ios::binary);std::vector<uint8_t> frame((std::istreambuf_iterator<char>(file)),{});
  if(frame.size()>20&&frame[0]=='S'&&frame[1]=='R'){
   uint64_t id=0;std::memcpy(&id,frame.data()+4,8);
   if(id!=frameSeq){std::vector<uint8_t> rgb;int w,h;if(SafeJpeg::decode(frame.data()+20,frame.size()-20,rgb,w,h)){uint64_t capture=0;std::memcpy(&capture,frame.data()+12,8);
    auto wall=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    grass.submit(rgb.data(),w,h,now()-std::min<uint64_t>(1000,std::max<int64_t>(0,wall-capture)));frameSeq=id;}}
  }
  std::array<uint8_t,3600> labels;if(GrassOverlay::mask(labels,stamp)){
   Pose at=history.front().second;for(auto s:history){if(s.first>stamp)break;at=s.second;}planner.observe(labels.data(),stamp,at);
  }
  float v=0,w=0;
  if(grass.available()&&stamp&&now()-stamp<1000&&planner.plan(pose,goal,now(),allowed))planner.steer(pose,.35,.05,allowed,turn,v,w);
  if(v<.01)pauses++;
  if(!command(v,w)){healthy=false;break;}
  if(tick%40==0){
   std::vector<uint8_t> overlay;uint64_t when=0;double ms=0;
   if(grass.latest(overlay,when,ms)){std::ofstream ppm("/tmp/camera-avoidance-overlay.ppm",std::ios::binary);ppm<<"P6\n640 360\n255\n";ppm.write((char*)overlay.data(),overlay.size());}
   std::ofstream polys("/tmp/camera-avoidance-polygons.txt");polys<<"pose "<<pose.x<<" "<<pose.y<<" "<<pose.yaw<<"\n";
   for(auto& o:planner.obstacles){for(auto p:o.points)polys<<p.x<<","<<p.y<<" ";polys<<"\n";}
  }
  if(tick%40==0)std::cout<<"t="<<(now()-start)/1000.<<" pose="<<pose.x<<","<<pose.y<<" objects="<<planner.obstacles.size()<<" route="<<planner.route.size()<<" v="<<v<<" w="<<w<<std::endl;
  if(distance({pose.x,pose.y},goal)<.25){arrived=true;break;}
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
 }
 command(0,0);command(0,0,true);close(fd);
 std::cout<<"Godot RGB -> grass model -> contours -> planner -> physics: arrived="<<arrived<<" healthy="<<healthy<<" pauses="<<pauses<<std::endl;
 return arrived&&healthy?0:1;
}
