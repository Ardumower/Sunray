#include "src/camera/CameraLightLogger.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <unistd.h>
int main(){
  std::vector<uint8_t> image(12*8*3,0);
  auto dark=CameraLightLogger::measure(image.data(),12,8);
  assert(dark.mean==0 && dark.dark==1 && dark.bright==0);
  std::fill(image.begin(),image.end(),255);
  auto bright=CameraLightLogger::measure(image.data(),12,8);
  assert(bright.mean==255 && bright.bright==1 && bright.p50==255);
  for(int y=0;y<8;++y)for(int x=0;x<12;++x)for(int c=0;c<3;++c)image[(y*12+x)*3+c]=((y/4)*3+x/4)*40;
  auto grid=CameraLightLogger::measure(image.data(),12,8);
  for(int i=0;i<6;++i)assert(grid.region[i]==i*40 && grid.regionP80[i]==i*40);
  assert(grid.mean==100 && grid.p10==0 && grid.p90==200);
  char tmp[]="/tmp/camera-light-test-XXXXXX";assert(mkdtemp(tmp));assert(chdir(tmp)==0);
  {
    CameraLightLogger log;
    v4l2_buffer b{};b.sequence=10;b.timestamp.tv_sec=1;log.capture(b);
    b.sequence=13;b.timestamp.tv_usec=100000;log.capture(b);
    log.invalid();log.decoded();log.detection(true);log.detection(false);log.timeout();
    std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    log.sample(-1,0,0,image.data(),12,8);
    std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    log.timeout();log.sample(-1,0,0);
  }
  std::ofstream("camera_light/disabled").close();
  {
    CameraLightLogger disabled;
    std::this_thread::sleep_for(std::chrono::milliseconds(1050));
    assert(!disabled.due());
    disabled.sample(-1,0,0,image.data(),12,8);
  }
  std::cout<<tmp<<"\n";
}
