#pragma once
#include <cstdio>
#include <string>
// No shell process or Arduino Stream::readString EOF timeout. The device-tree
// model is immutable during operation and bounded; read it only once.
inline const std::string& linuxBoardName(){
  static const std::string name=[] {
    std::string result="Linux";
    FILE* f=std::fopen("/sys/firmware/devicetree/base/model","rb");
    if(f){char buffer[256]={};const size_t n=std::fread(buffer,1,sizeof(buffer)-1,f);std::fclose(f);
      if(n){result+=' ';result+=buffer;}}
    return result;
  }();
  return name;
}
