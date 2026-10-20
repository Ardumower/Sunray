#include "src/camera/ImageDarkness.h"
#include <cassert>
#include <iostream>
#include <limits>
int main(int argc,char**){
 ImageDarkness d;double r[6]={205,232,177,83,72,66}; // current sunny dock/shadow
 for(int t=0;t<60;++t)d.update(r,t);assert(!d.dark());assert(d.brightness()>115);
 for(int i=0;i<6;i++)r[i]=40;r[5]=255; // one bright reflector cannot hide night
 for(int t=60;t<70;++t)d.update(r,t);assert(!d.dark());
 for(int t=70;t<100;++t)d.update(r,t);assert(d.dark());
 double daylight[6]={205,232,177,83,72,66};
 for(int t=100;t<120;++t)d.update(daylight,t);assert(d.dark());
 for(int t=120;t<180;++t)d.update(daylight,t);assert(!d.dark());
 double darkCorner[6]={0,140,140,140,140,140};
 for(int t=180;t<240;++t)d.update(darkCorner,t);assert(!d.dark());
 darkCorner[0]=std::numeric_limits<double>::quiet_NaN();d.update(darkCorner,241);assert(!d.dark());
 ImageDarkness gap;for(int i=0;i<6;i++)r[i]=0;gap.update(r,0);gap.update(r,100);assert(!gap.dark());
 if(argc>1){ImageDarkness replay;double t;bool old=false;
  while(std::cin>>t>>r[0]>>r[1]>>r[2]>>r[3]>>r[4]>>r[5]){replay.update(r,t);if(old!=replay.dark()){std::cout<<std::fixed<<t<<" "<<replay.dark()<<"\n";old=replay.dark();}}
 }else std::cout<<"PASS full-image darkness: sunny dock shadow, night, highlight, dark corner, hysteresis and missing frames\n";
}
