#pragma once
#include <cmath>
#include "IoLightsProtocol.h"
namespace IoLights {
struct Input {
  bool station=false, charging=false, full=false, driving=false, dockPath=false;
  float linear=0, angular=0, voltage=0, emptyVoltage=23, fullVoltage=29;
  uint8_t brightness=5, chargeSelection=3;
};
class Policy {
  uint8_t candidate=NoBlink;
  uint32_t candidateAt=0,lastSentAt=0,lastState=0;
  bool sent=false;
public:
  State select(const Input& in,uint32_t now){
    State s;s.brightness=in.brightness;s.selection=in.chargeSelection;
    uint8_t wanted=NoBlink;
    if(std::isfinite(in.linear) && std::isfinite(in.angular) &&
       std::fabs(in.linear)<0.03f && std::fabs(in.angular)>0.06f)
      wanted=in.angular>0?Left:Right;
    if(in.station || in.dockPath)wanted=NoBlink;
    if(wanted!=candidate){candidate=wanted;candidateAt=now;}
    if(in.station){
      if(in.full){s.mode=Charged;s.percent=100;}
      else if(in.charging){
        s.mode=Charge;
        // Approximation only: Sunray has no measured battery SOC.
        float pct=0;
        if(std::isfinite(in.voltage) && in.fullVoltage>in.emptyVoltage)
          pct=100*(in.voltage-in.emptyVoltage)/(in.fullVoltage-in.emptyVoltage);
        s.percent=pct<0?0:pct>=99?99:uint8_t(pct);
      }
    }else if(in.driving || wanted!=NoBlink || std::fabs(in.linear)>=0.03f){
      s.mode=Road;s.front=s.rear=true;s.selection=3;
      s.blink=in.dockPath?uint8_t(Hazard):((uint32_t(now-candidateAt)>=250)?candidate:uint8_t(NoBlink));
    }
    return s;
  }
  bool due(const State& s,uint32_t now){
    const uint32_t value=s.pack();
    if(sent && value==lastState && uint32_t(now-lastSentAt)<1000)return false;
    sent=true;lastSentAt=now;lastState=value;return true;
  }
};
}
