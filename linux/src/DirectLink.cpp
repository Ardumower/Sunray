#include "DirectLink.h"
#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef DIRECT_LINK_PORT
#define DIRECT_LINK_PORT 18891
#endif

namespace DirectLink {
using Clock=std::chrono::steady_clock;
static double now(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static uint32_t nowMs(){return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count());}
// Comm::processCmd removes the final checksum field even when checking is off.
// Use the full AT wire format so the angular argument survives parsing.
static std::string motorCommand(double linear, double angular){
 char payload[80];std::snprintf(payload,sizeof(payload),"AT+M,%.4f,%.4f",linear,angular);
 uint8_t checksum=0;for(const char* p=payload;*p;++p)checksum+=static_cast<uint8_t>(*p);
 char suffix[8];std::snprintf(suffix,sizeof(suffix),",0x%02X",checksum);
 return std::string(payload)+suffix;
}
struct Bridge {
 std::atomic<bool> stop{false}, connected{false};
 std::atomic<uint32_t> lease{0}, cloudUntil{0};
 std::mutex mutex;
 std::thread thread;
 std::array<std::vector<unsigned char>,2> frames;
 std::string drive;
 double received=0;
 bool moving=false;
 ~Bridge(){stop=true;if(thread.joinable())thread.join();}
 void receive(const std::string& line){
  double timestamp=0,linear=0,angular=0;char extra=0;
  if(std::sscanf(line.c_str(),"D %lf %lf %lf %c",&timestamp,&linear,&angular,&extra)!=3 ||
     !std::isfinite(timestamp)||!std::isfinite(linear)||!std::isfinite(angular)||
     std::abs(now()-timestamp)>.25 || std::abs(linear)>.6 || std::abs(angular)>1.0)return;
  const auto command=motorCommand(linear,angular);
  std::lock_guard<std::mutex> lock(mutex);
  received=now();drive=command;moving=linear!=0||angular!=0;lease=nowMs()+DRIVE_TIMEOUT_MS+500;
 }
 void disconnect(){
  connected=false;
  std::lock_guard<std::mutex> lock(mutex);
  for(auto& frame:frames)frame.clear();
  if(moving){drive=motorCommand(0,0);received=now();moving=false;}
 }
 void run(){
  int server=socket(AF_INET,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);if(server<0)return;
  int yes=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes));
  sockaddr_in address={};address.sin_family=AF_INET;address.sin_port=htons(DIRECT_LINK_PORT);address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if(bind(server,reinterpret_cast<sockaddr*>(&address),sizeof(address))<0||listen(server,1)<0){close(server);return;}
  while(!stop){
   int fd=accept4(server,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
   if(fd<0){std::this_thread::sleep_for(std::chrono::milliseconds(20));continue;}
   connected=true;std::string incoming;std::vector<unsigned char> output;size_t sent=0;double began=now();
   while(!stop){
    char buffer[2048];const auto n=recv(fd,buffer,sizeof(buffer),0);
    if(n==0 || (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK))break;
    if(n>0){incoming.append(buffer,n);if(incoming.size()>8192)break;
     size_t pos;while((pos=incoming.find('\n'))!=std::string::npos){receive(incoming.substr(0,pos));incoming.erase(0,pos+1);}}
    if(output.empty()){
     std::unique_lock<std::mutex> lock(mutex,std::try_to_lock);
     if(lock)for(auto& frame:frames)if(!frame.empty()){
      const uint32_t length=htonl(frame.size());output.resize(4);std::memcpy(output.data(),&length,4);
      output.insert(output.end(),frame.begin(),frame.end());frame.clear();sent=0;began=now();break;
     }
    }
    if(!output.empty()){
     auto n=send(fd,output.data()+sent,output.size()-sent,MSG_NOSIGNAL);
     if(n>0){sent+=n;if(sent==output.size())output.clear();}
     else if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK)break;
     if(!output.empty() && now()-began>.25)break; // No stale image backlog.
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
   }
   close(fd);disconnect();
  }
  close(server);disconnect();
 }
};
static Bridge& bridge(){static Bridge value;return value;}
void start(){auto& b=bridge();if(!b.thread.joinable())b.thread=std::thread([&b]{b.run();});}
bool ownsDrive(){const auto until=bridge().lease.load();return until!=0 && static_cast<int32_t>(until-nowMs())>0;}
bool takeDrive(std::string& command){
 auto& b=bridge();std::unique_lock<std::mutex> lock(b.mutex,std::try_to_lock);if(!lock)return false;
 if(!b.drive.empty()){
  if(now()-b.received>.25){b.drive.clear();b.moving=false;command=motorCommand(0,0);}
  else {command=std::move(b.drive);b.drive.clear();}
  return true;
 }
 if(b.moving && now()-b.received>DRIVE_TIMEOUT_MS/1000.0){b.moving=false;command=motorCommand(0,0);return true;}
 return false;
}
void camera(const unsigned char* data,size_t length){
 auto& b=bridge();if(!b.connected || length>1024*1024)return;
 std::unique_lock<std::mutex> lock(b.mutex,std::try_to_lock);
 if(lock)b.frames[length>=16 && data[7]==101?1:0].assign(data,data+length);
}
void setCloudCamera(bool enabled){bridge().cloudUntil=enabled?0:nowMs()+15000;}
bool cloudCamera(){const auto until=bridge().cloudUntil.load();return until==0 || static_cast<int32_t>(nowMs()-until)>=0;}
bool rpc(const std::string& message,std::string& answer){
 if(message.compare(0,7,"DIRECT:")!=0)return false;
 if(message.size()<41||message.size()>24000||message[39]!=':')return true;
 for(size_t i=7;i<39;i++)if(!((message[i]>='0'&&message[i]<='9')||(message[i]>='a'&&message[i]<='f')))return true;
 const auto prefix=message.substr(0,40);answer=prefix+"{\"ok\":false,\"error\":\"Direct link unavailable\"}";
 int fd=socket(AF_INET,SOCK_STREAM|SOCK_CLOEXEC,0);if(fd<0)return true;
 timeval timeout={5,0};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
 sockaddr_in addr={};addr.sin_family=AF_INET;addr.sin_port=htons(18892);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 if(connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){close(fd);return true;}
 auto body=message.substr(40);auto request="POST /rpc HTTP/1.0\r\nX-Sunray-IPC: 1\r\nContent-Type: application/json\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
 size_t sent=0;while(sent<request.size()){auto n=send(fd,request.data()+sent,request.size()-sent,MSG_NOSIGNAL);if(n<=0){close(fd);return true;}sent+=n;}
 std::string response;char buffer[8192];bool complete=false;auto deadline=now()+5;
 while(response.size()<32768 && now()<deadline){auto n=recv(fd,buffer,sizeof(buffer),0);if(n==0){complete=true;break;}if(n<0)break;response.append(buffer,n);}
 close(fd);auto split=response.find("\r\n\r\n");
 if(complete && response.compare(0,12,"HTTP/1.0 200")==0 && split!=std::string::npos)answer=prefix+response.substr(split+4);
 return true;
}
}
