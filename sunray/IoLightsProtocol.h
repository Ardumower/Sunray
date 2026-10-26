#pragma once
#include <stdint.h>
// Shared wire format; keep identical in Sunray and the IO board.
namespace IoLights {
constexpr uint16_t CanId = 700;
constexpr uint8_t Node = 1;
enum Command : uint8_t { StateCommand=32, Brightness=33, Front=34, Rear=35,
  Indicators=36, Charging=37, Full=38, Off=39, Test=40 };
enum Mode : uint8_t { Dark=0, Road=1, Charge=2, Charged=3, SegmentTest=4 };
enum Blink : uint8_t { NoBlink=0, Left=1, Right=2, Hazard=3 };
// Payload: [mode | front<<3 | rear<<4 | blink<<5, brightness%, percent, selection].
// selection: charge/full 1=front,2=rear,3=both; test physical segment 0..3.
struct State {
  uint8_t mode=Dark, brightness=5, percent=0, selection=3, blink=NoBlink;
  bool front=false, rear=false;
  uint32_t pack() const {
    return uint32_t(mode | (front?8:0) | (rear?16:0) | (blink<<5)) |
      (uint32_t(brightness)<<8) | (uint32_t(percent)<<16) | (uint32_t(selection)<<24);
  }
  static State unpack(uint32_t v) {
    State s; s.mode=v&7; s.front=v&8; s.rear=v&16; s.blink=(v>>5)&3;
    s.brightness=(v>>8)&255; s.percent=(v>>16)&255; s.selection=v>>24; return s;
  }
};
inline bool valid(const State& s) {
  return s.mode<=SegmentTest && s.brightness<=100 && s.percent<=100 &&
    (s.mode==SegmentTest ? s.selection<4 : s.selection>=1 && s.selection<=3);
}
// Only addressed SET packets, no broadcasts, no firmware-update commands or ACK traffic.
inline bool apply(uint32_t id,uint8_t len,const uint8_t* p,State& state) {
  if(id!=CanId || len!=8)return false;
  const uint16_t addr=uint16_t(p[0])|(uint16_t(p[1])<<8);
  const unsigned src=addr&63;
  if((addr&0xf000) || src==0 || src==Node || src>=63 || ((addr>>6)&63)!=Node || p[2]!=2)return false;
  State next=state;
  if(p[3]==StateCommand){
    if(p[4]&128)return false;
    next=State::unpack(uint32_t(p[4])|(uint32_t(p[5])<<8)|(uint32_t(p[6])<<16)|(uint32_t(p[7])<<24));
  } else {
    if(p[6] || p[7])return false;
    switch(p[3]) {
      case Brightness: if(p[5])return false; next.brightness=p[4]; break;
      case Front: case Rear:
        if(p[4]>1 || p[5])return false;
        next.mode=Road; next.selection=3;
        if(p[3]==Front)next.front=p[4];else next.rear=p[4]; break;
      case Indicators:
        if(p[4]>3 || p[5])return false;
        next.mode=Road;next.selection=3;next.blink=p[4];break;
      case Charging:
        next.mode=p[4]==100?Charged:Charge;next.percent=p[4];next.selection=p[5];next.blink=0;break;
      case Full:
        if(p[5])return false;
        next.mode=Charged;next.percent=100;next.selection=p[4];next.blink=0;break;
      case Off:
        if(p[4] || p[5])return false;
        next=State();next.brightness=state.brightness;break;
      case Test:
        if(p[5])return false;
        next.mode=SegmentTest;next.selection=p[4];next.blink=0;break;
      default:return false;
    }
  }
  if(!valid(next))return false;
  state=next;return true;
}
}
