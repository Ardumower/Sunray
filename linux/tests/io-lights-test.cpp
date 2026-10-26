#include "../../sunray/IoLightsPolicy.h"
#include <cassert>
#include <cstdio>
using namespace IoLights;
int main(){
 Policy p;Input i;assert(p.select(i,0).mode==Dark);
 i.driving=true;State s=p.select(i,0);assert(s.mode==Road && s.front && s.rear && s.brightness==5);
 assert(p.due(s,0));assert(!p.due(s,999));assert(p.due(s,1000));
 i.angular=.1;s=p.select(i,100);assert(s.blink==NoBlink);s=p.select(i,350);assert(s.blink==Left);
 i.angular=-.1;assert(p.select(i,400).blink==NoBlink);assert(p.select(i,650).blink==Right);
 i.linear=.1;assert(p.select(i,651).blink==NoBlink);
 i.dockPath=true;assert(p.select(i,652).blink==Hazard);
 i.station=true;assert(p.select(i,653).mode==Dark);
 i.charging=true;i.voltage=26;s=p.select(i,654);assert(s.mode==Charge && s.percent==50 && s.blink==NoBlink);
 i.voltage=30;s=p.select(i,655);assert(s.mode==Charge && s.percent==99);
 i.full=true;assert(p.select(i,656).mode==Charged);
 i.station=false;i.dockPath=false;i.linear=0;i.angular=0;assert(p.select(i,657).mode==Road);
 i.driving=false;assert(p.select(i,658).mode==Dark);
 Policy wrap;i.angular=.1;wrap.select(i,0xffffff80);assert(wrap.select(i,122).blink==Left);
 puts("Sunray LED policy OK");
}
