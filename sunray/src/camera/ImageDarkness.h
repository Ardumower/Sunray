#pragma once
#include <algorithm>
#include <cmath>

// Scene-brightness warning, not a calibrated ambient-light/lux measurement.
// The median of three lower image regions reduces sky/highlight influence.
class ImageDarkness {
  bool initialized_=false,dark_=false;
  double previous_=0,filtered_=0,lowSeconds_=0,highSeconds_=0;
public:
  bool dark()const{return dark_;}
  double brightness()const{return filtered_;}
  void update(const double* regions,double seconds){
    double lower[3]={regions[3],regions[4],regions[5]};std::sort(lower,lower+3);
    if(!std::isfinite(lower[1]) || !std::isfinite(seconds))return;
    if(!initialized_){initialized_=true;previous_=seconds;filtered_=lower[1];return;}
    const double dt=seconds-previous_;previous_=seconds;
    if(dt<=0 || dt>3){lowSeconds_=highSeconds_=0;filtered_=lower[1];return;}
    filtered_+=(1-std::exp(-dt/5.0))*(lower[1]-filtered_);
    lowSeconds_=filtered_<85?lowSeconds_+dt:0;
    highSeconds_=filtered_>115?highSeconds_+dt:0;
    if(lowSeconds_>=20)dark_=true;
    if(highSeconds_>=45)dark_=false;
  }
};
