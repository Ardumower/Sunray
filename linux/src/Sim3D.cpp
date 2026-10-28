#include "Sim3D.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <locale>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netpacket/packet.h>
#ifndef SIM3D_TEST
#include "../../sunray/config.h"
#endif
#ifndef SIM3D_ENABLE
#define SIM3D_ENABLE false
#endif
#ifndef SIM3D_FRAME_FILE
#define SIM3D_FRAME_FILE "/run/sunray-sim3d/camera.frame"
#endif
namespace Sim3D {
using Clock=std::chrono::steady_clock;
namespace {
struct Link {
 int fd=-1;uint64_t seq=0,lastSeq=0,reset=0,obs=0;double left=0,right=0,mow=0,base=.39,tpm=1000,tx=0,ty=0,ta=0,ox=0,oy=0,orr=0;unsigned flags=0;
 Clock::time_point sent{},received{};State latest;
 Link(){fd=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(fd>=0){sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(19091);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);if(connect(fd,(sockaddr*)&a,sizeof(a))){close(fd);fd=-1;}}}
 ~Link(){if(fd>=0)close(fd);}
};
Link& link(){static Link value;return value;}
uint64_t le64(const uint8_t* p){uint64_t r=0;for(int i=7;i>=0;--i)r=(r<<8)|p[i];return r;}
}
bool enabled(){return SIM3D_ENABLE;}
std::string robotId(){
 // Stable physical Ethernet MAC, with deterministic fallback to another adapter.
 std::string fallback; ifaddrs* all=nullptr;if(getifaddrs(&all)!=0)return "sunray-sim3d";
 for(auto* p=all;p;p=p->ifa_next){if(!p->ifa_addr||p->ifa_addr->sa_family!=AF_PACKET||!strcmp(p->ifa_name,"lo"))continue;
 auto* a=(sockaddr_ll*)p->ifa_addr;if(a->sll_halen!=6)continue;char s[32];snprintf(s,sizeof(s),"%02x:%02x:%02x:%02x:%02x:%02x",a->sll_addr[0],a->sll_addr[1],a->sll_addr[2],a->sll_addr[3],a->sll_addr[4],a->sll_addr[5]);
 if(p->ifa_name[0]=='e'){fallback=s;break;}if(fallback.empty())fallback=s;
 }freeifaddrs(all);return fallback.empty()?"sunray-sim3d":fallback;
}
void command(double l,double r,double m,double wb,double tpm,unsigned flags){auto& c=link();c.left=l;c.right=r;c.mow=m;c.base=wb;c.tpm=tpm;c.flags=flags;}
void teleport(double x,double y,double yaw){auto& c=link();c.tx=x;c.ty=y;c.ta=yaw;++c.reset;}
void obstacle(double x,double y,double radius){auto& c=link();c.ox=x;c.oy=y;c.orr=radius;++c.obs;}
bool poll(State& state){
 auto& c=link();const auto now=Clock::now();if(c.fd<0)return false;
 if(now-c.sent>=std::chrono::milliseconds(50)){
  std::ostringstream s;s.imbue(std::locale::classic());s.precision(12);
  s<<"S3D1 "<<++c.seq<<' '<<c.left<<' '<<c.right<<' '<<c.mow<<' '<<c.base<<' '<<c.tpm<<' '<<c.flags<<' '<<c.reset<<' '<<c.tx<<' '<<c.ty<<' '<<c.ta<<' '<<c.obs<<' '<<c.ox<<' '<<c.oy<<' '<<c.orr;
  const auto msg=s.str();send(c.fd,msg.data(),msg.size(),MSG_DONTWAIT);c.sent=now;
 }
 for(int n=0;n<16;n++){
  char b[1024];const int bytes=recv(c.fd,b,sizeof(b)-1,MSG_DONTWAIT);if(bytes<=0)break;b[bytes]=0;
  State next;unsigned long long seq;int bumper,charge;char tail;
  const int fields=sscanf(b,"S3D1 %llu %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %c",&seq,&next.x,&next.y,&next.yaw,&next.leftTicks,&next.rightTicks,&next.linear,&next.angular,&bumper,&charge,&next.roll,&next.pitch,&tail);
  if(fields!=12||seq<=c.lastSeq||seq>c.seq)continue;
  bool valid=true;for(double v:{next.x,next.y,next.yaw,next.leftTicks,next.rightTicks,next.linear,next.angular,next.roll,next.pitch})valid=valid&&std::isfinite(v);
  if(!valid||next.leftTicks<0||next.rightTicks<0||std::abs(next.linear)>5||std::abs(next.angular)>20)continue;
  next.bumper=bumper!=0;next.charging=charge!=0;c.latest=next;c.lastSeq=seq;c.received=now;
 }
 if(c.received.time_since_epoch().count()==0||now-c.received>std::chrono::milliseconds(500))return false;
 state=c.latest;return true;
}
bool frame(std::vector<uint8_t>& jpeg,uint64_t& sequence,uint64_t& ageMs){
 std::ifstream f(SIM3D_FRAME_FILE,std::ios::binary|std::ios::ate);if(!f)return false;
 auto size=f.tellg();if(size<24||size>4*1024*1024)return false;f.seekg(0);
 uint8_t h[20];if(!f.read((char*)h,sizeof(h))||memcmp(h,"SRF1",4))return false;
 const auto seq=le64(h+4),stamp=le64(h+12);
 const auto now=uint64_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
 if(stamp>now||now-stamp>500||seq==sequence)return false;
 std::vector<uint8_t> out(size_t(size)-sizeof(h));if(!f.read((char*)out.data(),out.size()))return false;
 if(out[0]!=0xff||out[1]!=0xd8)return false;
 sequence=seq;ageMs=now-stamp;jpeg.swap(out);return true;
}
}
