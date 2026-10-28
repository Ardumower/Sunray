#include "DirectLink.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <thread>
#include <string>
#include <iostream>
static void wait(int ms){std::this_thread::sleep_for(std::chrono::milliseconds(ms));}
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
// Validate the checksum and strip it exactly as Comm::processCmd does, then
// check that both motor arguments remain (the previous format lost angular).
static std::string payload(const std::string& command){
 const auto idx=command.rfind(',');assert(idx!=std::string::npos);
 assert(command.substr(idx+1,2)=="0x");
 unsigned char sum=0;for(size_t i=0;i<idx;++i)sum+=command[i];
 assert(std::stoul(command.substr(idx+1),nullptr,16)==sum);
 return command.substr(0,idx);
}
int main(){
 DirectLink::start();wait(80);
 int fd=socket(AF_INET,SOCK_STREAM,0);sockaddr_in a={};a.sin_family=AF_INET;a.sin_port=htons(18893);a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
 assert(connect(fd,(sockaddr*)&a,sizeof(a))==0);wait(30);
 auto sendDrive=[&](double stamp,const char* values){auto s="D "+std::to_string(stamp)+" "+values+"\n";assert(send(fd,s.data(),s.size(),0)==(ssize_t)s.size());wait(30);};
 std::string cmd;
 sendDrive(now()-2,"0.2 0");assert(!DirectLink::takeDrive(cmd));
 sendDrive(now(),"nan 0");assert(!DirectLink::takeDrive(cmd));
 sendDrive(now(),"8 0");assert(!DirectLink::takeDrive(cmd));
 sendDrive(now(),"0.2 0.1");assert(DirectLink::takeDrive(cmd)&&payload(cmd)=="AT+M,0.2000,0.1000");assert(DirectLink::ownsDrive());
 for(const auto values:{"0 1","0 -1"}){
  sendDrive(now(),values);assert(DirectLink::takeDrive(cmd));
  assert(payload(cmd)==(std::string(values)=="0 1"?"AT+M,0.0000,1.0000":"AT+M,0.0000,-1.0000"));
 }
 wait(2100);assert(!DirectLink::takeDrive(cmd));assert(DirectLink::ownsDrive());
 wait(1050);assert(DirectLink::takeDrive(cmd)&&payload(cmd)=="AT+M,0.0000,0.0000");assert(!DirectLink::takeDrive(cmd));
 sendDrive(now(),"0.1 0");wait(260);assert(DirectLink::takeDrive(cmd)&&payload(cmd)=="AT+M,0.0000,0.0000");
 sendDrive(now(),"0.1 0");assert(DirectLink::takeDrive(cmd));close(fd);wait(50);assert(DirectLink::takeDrive(cmd)&&payload(cmd)=="AT+M,0.0000,0.0000");
 DirectLink::setCloudCamera(false);assert(!DirectLink::cloudCamera());DirectLink::setCloudCamera(true);assert(DirectLink::cloudCamera());
 std::cout<<"Direct-link expiry, validation and disconnect tests passed\n";
}
