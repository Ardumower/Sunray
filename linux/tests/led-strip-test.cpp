#include "src/camera/LedStripDetector.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <fstream>
#include <vector>

static const int W=640,H=360;
using Image=std::vector<uint8_t>;
static Image render(const std::function<double(int,int)>& pixel) {
  Image a(W*H*3);
  for(int y=0;y<H;++y) for(int x=0;x<W;++x) {
    auto v=(uint8_t)std::max(0.0,std::min(255.0,pixel(x,y)));
    for(int c=0;c<3;++c) a[(y*W+x)*3+c]=v;
  }
  return a;
}
static double flare(int x,int y,double cx=320,double slope=-.08,double width=65) {
  if(y<65) return 25;
  double half=width*std::sqrt((y-65.0)/(H-65.0));
  double d=std::fabs(x-(cx+slope*(y-200)))/(half+1);
  return std::min(255.0,25+290*std::exp(-std::pow(d,4)));
}
static void require(bool ok,const char* message) {
  if(!ok) {std::fprintf(stderr,"FAIL: %s\n",message);std::exit(1);}
}
static void check(const char* name,const Image& a,bool expected) {
  auto& d=LedStripDetector::instance();d.clear();
  for(int i=0;i<3;++i) {
    d.processRgb(a.data(),W,H,1000+100*i);
    require(d.observation(1000+100*i).found==(expected && i==2),name);
  }
  if(expected) require(std::fabs(d.observation(1200).horizontalError)<.04,name);
  require(!d.observation(1800).found,"expiry");d.clear();
  require(!d.observation(1200).found,"clear");
  std::printf("PASS %s\n",name);
}
int main(int argc,char**argv) {
  if(argc==4) {
    int w=std::atoi(argv[2]),h=std::atoi(argv[3]);require(w>0&&h>0,"dimensions");
    Image a(w*h*3);std::ifstream f(argv[1],std::ios::binary);f.read((char*)a.data(),a.size());
    require(f.gcount()==(long)a.size(),"RGB input size");auto& d=LedStripDetector::instance();
    for(int i=0;i<3;++i)d.processRgb(a.data(),w,h,1000+100*i);
    auto o=d.observation(1200);std::printf("found=%d error=%.4f confidence=%.3f\n",o.found,o.horizontalError,o.confidence);return 0;
  }
  for(double scale: {.35,.6,1.0,1.2})
    check("bloom/exposure",render([=](int x,int y){return flare(x,y)*scale;}),true);
  check("black",render([](int,int){return 0;}),false);
  check("white",render([](int,int){return 255;}),false);
  check("white rectangle",render([](int x,int y){return x>250&&x<390&&y>80?255:25;}),false);
  check("one edge",render([](int x,int){return x>320?255:25;}),false);
  check("upper only",render([](int x,int y){return y<130?flare(x,y):25;}),false);
  check("diagonal",render([](int x,int y){return flare(x,y,320,.5);}),false);
  check("ambiguous blooms",render([](int x,int y){return std::max(flare(x,y,160,-.08,40),flare(x,y,480,-.08,40));}),false);
  check("round reflection",render([](int x,int y){return 25+230*std::exp(-std::pow((x-320)/110.0,2)-std::pow((y-230)/80.0,2));}),false);
  auto left=render([](int x,int y){return flare(x,y,220);});
  auto right=render([](int x,int y){return flare(x,y,420);});
  auto& d=LedStripDetector::instance();d.clear();
  for(int i=0;i<6;++i){d.processRgb((i%2?left:right).data(),W,H,1000+i*100);require(!d.observation(1000+i*100).found,"alternating candidates must not acquire");}
  d.clear();d.processRgb(left.data(),W,H,1000);d.processRgb(left.data(),W,H,1100);d.processRgb(left.data(),W,H,2000);
  require(!d.observation(2000).found,"stale acquisition");
  std::puts("PASS temporal consistency");
}
