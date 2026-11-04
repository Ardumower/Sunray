#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <cstdint>
#include <cstddef>

// Feed the peak luminance of an unmodified camera image once per second.
// A rapid bright-to-dark transition must remain dark for a full minute.
// Opening a camera assumes a bright baseline, so dark-at-start is recovered too.
// Capture gaps discard observations; the reset budget survives session restarts.
class CameraBrightnessWatchdog {
  struct Sample {double time,peak;};
  std::deque<Sample> recent_;
  double previous_=-1, darkSince_=-1, reference_=0, darkPeak_=0;
  double lastAttempt_=-1;
  bool assumeBright_=true;
public:
  static int peak(const uint8_t* rgb,int w,int h){
    int result=0;
    for(std::size_t i=0,n=std::size_t(w)*std::size_t(h)*3;i<n;i+=3)
      result=std::max(result,(77*rgb[i]+150*rgb[i+1]+29*rgb[i+2])>>8);
    return result;
  }
  void newSession(bool assumeBright=true){recent_.clear();previous_=-1;darkSince_=-1;assumeBright_=assumeBright;}
  double reference()const{return reference_;}
  double darkPeak()const{return darkPeak_;}
  bool update(double peak,double seconds){
    if(!std::isfinite(peak)||!std::isfinite(seconds)||peak<0||peak>255)return false;
    if(previous_>=0 && (seconds<=previous_ || seconds-previous_>3))newSession(false);
    previous_=seconds;
    while(!recent_.empty() && seconds-recent_.front().time>5)recent_.pop_front();
    if(peak>=30)darkSince_=-1;
    else if(darkSince_<0){
      double high=assumeBright_?100:0;for(const auto& s:recent_)high=std::max(high,s.peak);
      if(high>=80){darkSince_=seconds;reference_=high;darkPeak_=peak;}
    }
    assumeBright_=false;
    if(darkSince_>=0){
      darkPeak_=std::max(darkPeak_,peak);
      if(seconds-darkSince_>=60 && (lastAttempt_<0 || seconds-lastAttempt_>=60)){
        lastAttempt_=seconds;newSession(false);return true;
      }
    }
    recent_.push_back({seconds,peak});
    return false;
  }
};
