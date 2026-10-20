#pragma once
#ifdef __linux__
#include <string>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// Invoked only on the cloud network thread. Never dispatches a motion command.
// Service handlers enqueue training; this RPC has bounded I/O and response size.
inline bool textureRpc(const std::string& message,std::string& answer){
  if(message.compare(0,6,"TRAIN:")!=0)return false;
  if(message.size()<40 || message.size()>4200 || message[38]!=':')return true;
  for(size_t i=6;i<38;i++)if(!((message[i]>='0'&&message[i]<='9')||(message[i]>='a'&&message[i]<='f')))return true;
  const std::string prefix=message.substr(0,39);
  answer=prefix+"{\"ok\":false,\"error\":\"Robot training service unavailable\"}";
  int fd=socket(AF_INET,SOCK_STREAM,0);if(fd<0)return true;
  timeval timeout={0,300000};setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
  sockaddr_in addr={};addr.sin_family=AF_INET;addr.sin_port=htons(18741);addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
  if(connect(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){close(fd);return true;}
  const auto body=message.substr(39);
  const auto request="POST /rpc HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
  size_t sent=0;while(sent<request.size()){ssize_t n=send(fd,request.data()+sent,request.size()-sent,MSG_NOSIGNAL);if(n<=0){close(fd);return true;}sent+=n;}
  std::string response;char buffer[8192];bool complete=false;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(750);
  while(response.size()<256*1024 && std::chrono::steady_clock::now()<deadline){ssize_t n=recv(fd,buffer,sizeof(buffer),0);if(n==0){complete=true;break;}if(n<0)break;response.append(buffer,n);}
  close(fd);auto split=response.find("\r\n\r\n");
  if(complete && response.compare(0,12,"HTTP/1.0 200")==0 && split!=std::string::npos)answer=prefix+response.substr(split+4);
  return true;
}
#endif
