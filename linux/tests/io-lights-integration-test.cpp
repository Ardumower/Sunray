#include "IoLightsPolicy.h"
#include "LedScenes.h"
#include <cassert>
#include <cstdio>
int main(){
 IoLights::Policy policy;IoLights::Input input;IoLights::Scenes scenes;scenes.begin(0);
 input.driving=true;input.dockPath=true;
 auto s=policy.select(input,100);auto packed=s.pack();
 uint8_t frame[8]={124,0,2,32};for(unsigned i=0;i<4;i++)frame[4+i]=packed>>(8*i);
 IoLights::State received;assert(IoLights::apply(700,8,frame,received));scenes.set(received,100);
 IoLights::Color pixels[96];scenes.render(100,pixels);
 for(unsigned seg=0;seg<4;seg++){auto c=pixels[LedLayout::pixel(seg,LedLayout::IndicatorPixels[seg][0])];assert(c.r==12 && c.g==9 && c.b==0);}
 scenes.render(600,pixels);for(unsigned seg=0;seg<4;seg++)assert(pixels[LedLayout::pixel(seg,LedLayout::IndicatorPixels[seg][0])].r==0);
 input.station=input.charging=true;input.voltage=26;packed=policy.select(input,700).pack();
 for(unsigned i=0;i<4;i++)frame[4+i]=packed>>(8*i);
 assert(IoLights::apply(700,8,frame,received));scenes.set(received,700);scenes.render(700,pixels);
 assert(pixels[6].r>0 && pixels[18].g>0 && pixels[12].r==0 && pixels[0].g==0);
 puts("Sunray -> CAN -> IO board integration OK");
}
