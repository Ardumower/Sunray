#include "src/camera/CameraBrightnessWatchdog.h"
#include <cassert>
#include <cstdio>
int main(){
  CameraBrightnessWatchdog w;
  for(int t=0;t<60;++t)assert(!w.update(10,t)); // dark at startup: one full minute
  assert(w.update(10,60));
  w.newSession();
  for(int t=61;t<121;++t)assert(!w.update(10,t));
  assert(w.update(10,121)); // reset remains bounded after reopening
  w.newSession();
  for(int t=0;t<=255;++t)assert(!w.update(255-t,t)); // sunset
  w.newSession();
  assert(!w.update(100,300));
  for(int t=301;t<=360;++t)assert(!w.update(25,t));
  assert(w.update(25,361));assert(w.reference()==100 && w.darkPeak()==25);
  for(int t=362;t<500;++t)assert(!w.update(25,t)); // no reset loop in darkness
  w.newSession();assert(!w.update(120,500));
  for(int t=501;t<530;++t)assert(!w.update(20,t));
  assert(!w.update(40,530)); // a bright image cancels the minute
  for(int t=531;t<600;++t)assert(!w.update(20,t));
  w.newSession();assert(!w.update(100,600));assert(!w.update(10,601));
  for(int t=610;t<680;++t)assert(!w.update(10,t)); // capture gap breaks continuity
  w.newSession();assert(!w.update(100,700));assert(!w.update(10,701));
  w.newSession();for(int t=702;t<762;++t)assert(!w.update(10,t));
  assert(w.update(10,762)); // reopening gets a new assumed bright baseline
  assert(!w.update(100,800));
  for(int t=801;t<=860;++t)assert(!w.update(10,t));
  assert(w.update(10,861));
  assert(!w.update(100,862));
  for(int t=863;t<=922;++t)assert(!w.update(10,t));
  assert(w.update(10,923)); // second confirmed transition, >60s after first
  uint8_t pixels[]={0,0,0,25,25,25,100,100,100};
  assert(CameraBrightnessWatchdog::peak(pixels,3,1)==100);
  puts("PASS brightness watchdog: startup darkness, sunset, sustained drop, recovery, gaps, session restart, reset interval, pixel peak");
}
