// Replay chronological JPEG paths from stdin without resetting acquisition.
#include "src/camera/ReflectorStripsDetector.h"
#include "src/camera/SafeJpeg.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
int main(){
  auto& detector=ReflectorStripsDetector::instance();detector.clear();
  std::string path;unsigned count=0,missing=0;
  while(std::getline(std::cin,path)){
    if(path.empty())continue;
    std::ifstream file(path,std::ios::binary);
    std::vector<uint8_t> jpeg((std::istreambuf_iterator<char>(file)),{}),rgb;
    int width=0,height=0;
    if(!SafeJpeg::decode(jpeg.data(),jpeg.size(),rgb,width,height)){
      std::cerr<<"Invalid JPEG: "<<path<<"\n";return 1;
    }
    const unsigned long timestamp=1000+count*200;
    detector.processRgb(rgb.data(),width,height,timestamp);++count;
    if(count>=3 && !detector.observation(timestamp+150).found){
      ++missing;std::cerr<<"Lost acquisition: "<<path<<"\n";
    }
  }
  std::cout<<"Sequence frames="<<count<<" unavailable after acquisition="<<missing<<"\n";
  return count>=3 && !missing?0:1;
}
