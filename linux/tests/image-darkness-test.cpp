#include "src/camera/ImageDarkness.h"
#include <cassert>
#include <iostream>
int main(int argc,char**){
 ImageDarkness d;double r[6]={255,255,255,140,140,140};
 for(int t=0;t<60;++t)d.update(r,t);assert(!d.dark());
 r[3]=r[4]=40;r[5]=255; // one reflector cannot hide a dark scene
 for(int t=60;t<70;++t)d.update(r,t);assert(!d.dark());
 for(int t=70;t<100;++t)d.update(r,t);assert(d.dark());
 r[3]=r[4]=140;
 for(int t=100;t<120;++t)d.update(r,t);assert(d.dark());
 for(int t=120;t<180;++t)d.update(r,t);assert(!d.dark());
 ImageDarkness gap;r[3]=r[4]=0;gap.update(r,0);gap.update(r,100);assert(!gap.dark());
 if(argc>1){ImageDarkness replay;double t;bool old=false;
  while(std::cin>>t>>r[0]>>r[1]>>r[2]>>r[3]>>r[4]>>r[5]){replay.update(r,t);if(old!=replay.dark()){std::cout<<std::fixed<<t<<" "<<replay.dark()<<"\n";old=replay.dark();}}
 }else std::cout<<"PASS dark warning: persistence, hysteresis, bright outlier and missing frames\n";
}
