#pragma once
#ifdef __linux__
#include <vector>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

// Latest-frame exchange in tmpfs, never a camera archive. Both processes run
// under the same account. The overlay includes its original capture timestamp.
namespace TextureFiles {
inline std::string directory(){const char* p=std::getenv("SUNRAY_TEXTURE_RUNTIME");return p?p:"/dev/shm/sunray-texture";}
inline void publish(const std::vector<uint8_t>& jpeg,uint64_t stamp){
  const auto dir=directory();mkdir(dir.c_str(),0700);
  const auto path=dir+"/frame.bin", tmp=path+".tmp";
  FILE* f=fopen(tmp.c_str(),"wb");if(!f)return;
  uint8_t header[8];for(int i=0;i<8;i++)header[i]=stamp>>(8*i);
  bool ok=fwrite(header,1,8,f)==8 && fwrite(jpeg.data(),1,jpeg.size(),f)==jpeg.size();
  if(fclose(f)!=0)ok=false;
  if(ok)rename(tmp.c_str(),path.c_str());else unlink(tmp.c_str());
}
inline bool overlay(std::vector<uint8_t>& jpeg,uint64_t now,uint64_t& stamp){
  FILE* f=fopen((directory()+"/overlay.bin").c_str(),"rb");if(!f)return false;
  struct stat st;uint8_t header[8];
  if(fstat(fileno(f),&st)<0 || st.st_size<9 || st.st_size>1024*1024 || fread(header,1,8,f)!=8){fclose(f);return false;}
  stamp=0;for(int i=0;i<8;i++)stamp|=uint64_t(header[i])<<(8*i);
  if(stamp>now || now-stamp>2000){fclose(f);return false;}
  jpeg.resize(st.st_size-8);bool ok=fread(jpeg.data(),1,jpeg.size(),f)==jpeg.size();fclose(f);return ok;
}
}
#endif
