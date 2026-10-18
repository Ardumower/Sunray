#pragma once
#include <cstdint>
#include <cmath>
namespace GrassObstacle {
// Normalized trapezoid: top (15%,75%)..(85%,75%), bottom (0%,100%)..(100%,100%).
inline float leftEdge(float v){return 0.6f*(1.0f-v);}
inline bool contains(float u,float v){return v>=0.75f&&v<=1.0f&&u>=leftEdge(v)&&u<=1.0f-leftEdge(v);}
inline bool cellInside(int x,int y){return contains((x+0.5f)/80.0f,(y+0.5f)/45.0f);}
constexpr uint64_t FreshMs=1000;
inline bool fresh(uint64_t stamp,uint64_t now){return stamp && now>=stamp && now-stamp<FreshMs;}
inline bool detect(const uint8_t* labels,bool wasBlocked=false){
  int count=0,total=0;
  for(int y=0;y<45;y++)for(int x=0;x<80;x++)if(cellInside(x,y)){++total;if(labels[y*80+x]==0)++count;}
  // Hysteresis: assert at 10%, release below 5% obstacle-class pixels.
  return count*100 >= (wasBlocked?5:10)*total;
}
inline float limit(float speed,bool enabled,bool automaticGps,bool reverse,bool blocked,bool fresh){
  if(enabled&&automaticGps&&!reverse&&blocked&&fresh&&speed>0.1f)return 0.1f;
  return speed;
}
inline bool border(int x,int y){
  const float u=x/639.0f,v=y/359.0f;
  if(!contains(u,v))return false;
  const float left=leftEdge(v);
  return y<274||y>=356||u-left<4.0f/640.0f||1.0f-left-u<4.0f/640.0f;
}
inline void drawContour(uint8_t* rgb,int w,int h,int state){
  if(!rgb||w<2||h<2)return;
  for(int y=0;y<h;y++)for(int x=0;x<w;x++)if(border(x*639/(w-1),y*359/(h-1))){
    auto* pixel=rgb+(y*w+x)*3;
    pixel[0]=state==0?0:255;pixel[1]=state==1?0:(state==0?255:180);pixel[2]=0;
  }
}

}
