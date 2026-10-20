#pragma once
#include <algorithm>
#include <cmath>

// Scene-brightness warning, not a calibrated ambient-light/lux measurement.
// Use all six image regions. A trimmed mean drops one bright highlight and
// one dark corner, while broad daylight above a shaded dock still counts.
class ImageDarkness {
  bool initialized_=false,dark_=false;
  double previous_=0,filtered_=0,lowSeconds_=0,highSeconds_=0;
public:
  bool dark()const{return dark_;}
  double brightness()const{return filtered_;}
  void update(const double* regions,double seconds){
    double sorted[6];
    for(int i=0;i<6;++i){if(!std::isfinite(regions[i]))return;sorted[i]=regions[i];}
    if(!std::isfinite(seconds))return;
    std::sort(sorted,sorted+6);
    const double scene=(sorted[1]+sorted[2]+sorted[3]+sorted[4])*.25;
    if(!initialized_){initialized_=true;previous_=seconds;filtered_=scene;return;}
    const double dt=seconds-previous_;previous_=seconds;
    if(dt<=0 || dt>3){lowSeconds_=highSeconds_=0;filtered_=scene;return;}
    filtered_+=(1-std::exp(-dt/5.0))*(scene-filtered_);
    lowSeconds_=filtered_<85?lowSeconds_+dt:0;
    highSeconds_=filtered_>115?highSeconds_+dt:0;
    if(lowSeconds_>=20)dark_=true;
    if(highSeconds_>=45)dark_=false;
  }
};
