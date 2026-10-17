#pragma once
#include <cmath>

// Bounded in-place scan, separate from camera tracking and GPS route following.
class DockMarkerSearch {
public:
  enum State { Inactive, Settling, Turning, Finished, Stalled };
  State state=Inactive;
  float lastYaw=0,turn=0,progressTurn=0;
  unsigned long started=0,progressAt=0;
  bool active()const{return state==Settling || state==Turning;}
  void reset(){state=Inactive;}
  static bool eligible(float x,float y,float ax,float ay,float bx,float by){
    const float dx=bx-ax,dy=by-ay,len2=dx*dx+dy*dy;
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(ax)||!std::isfinite(ay)||
       !std::isfinite(bx)||!std::isfinite(by)||!std::isfinite(len2)||len2<.0025f)return false;
    const float t=((x-ax)*dx+(y-ay)*dy)/len2;
    if(t<0 || t>1)return false; // station side of handoff, not beyond the station
    const float ex=x-ax-t*dx,ey=y-ay-t*dy;
    return ex*ex+ey*ey<=4.0f;
  }
  void start(float yaw,unsigned long now){
    state=std::isfinite(yaw)?Settling:Stalled;
    lastYaw=yaw;turn=progressTurn=0;started=progressAt=now;
  }
  State update(float yaw,unsigned long now){
    if(!active())return state;
    if(!std::isfinite(yaw) || now-started>=300000UL)return state=Stalled;
    if(state==Settling){
      lastYaw=yaw;progressAt=now;
      if(now-started>=500)state=Turning;
      return state;
    }
    const float delta=std::atan2(std::sin(yaw-lastYaw),std::cos(yaw-lastYaw));
    lastYaw=yaw;
    if(std::fabs(delta)>.35f)return state=Stalled; // pose jump cannot count as a scan
    turn+=delta; // commanded positive rotation; oscillation must not count as a full turn
    if(turn>=6.283185307f)return state=Finished;
    if(turn-progressTurn>=.03f){progressTurn=turn;progressAt=now;}
    if(now-progressAt>=15000UL)state=Stalled;
    return state;
  }
};
