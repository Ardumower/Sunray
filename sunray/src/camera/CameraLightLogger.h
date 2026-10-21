#pragma once
#ifdef __linux__
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/videodev2.h>

// Passive diagnostics. All files are written on a separate thread; a bounded
// queue and disk reserve keep diagnostics from delaying camera navigation.
class CameraLightLogger {
  using Clock=std::chrono::steady_clock;
  struct Record { std::string line, day, imageName; std::vector<uint8_t> jpeg; };
  std::mutex mutex_; std::condition_variable wake_; std::deque<Record> queue_;
  const bool enabled_=access("camera_light/disabled",F_OK)!=0;
  bool stopping_=false; std::thread writer_;
  Clock::time_point start_=Clock::now(), lastImage_=start_-std::chrono::seconds(300);
  uint64_t frames_=0, gaps_=0, invalid_=0, decodes_=0, detected_=0, found_=0, timeouts_=0, dropped_=0;
  uint32_t sequence_=0; bool haveSequence_=false;
  double previousTimestamp_=0, timestampSpan_=0, timestampMaxGap_=0;
  unsigned timestampIntervals_=0;
  double smoothFps_=-1, smoothLight_=-1;
  struct Control { int value=0; unsigned flags=0; bool valid=false; };
  static Control control(int fd, unsigned id) {
    Control result; v4l2_queryctrl q{};q.id=id;
    v4l2_control c{};c.id=id;
    if(ioctl(fd,VIDIOC_QUERYCTRL,&q)==0 && !(q.flags&V4L2_CTRL_FLAG_DISABLED) && ioctl(fd,VIDIOC_G_CTRL,&c)==0){
      result.value=c.value;result.flags=q.flags;result.valid=true;
    }
    return result;
  }
  static void field(std::ostringstream& s,const char* name,Control c){
    s<<",\""<<name<<"\":"; if(c.valid)s<<c.value;else s<<"null";
    s<<",\""<<name<<"_flags\":"<<c.flags;
  }
  void writeLoop(){
    bool warned=false;
    for(;;){
      Record r;
      {std::unique_lock<std::mutex> lock(mutex_);wake_.wait(lock,[this]{return stopping_||!queue_.empty();});
       if(queue_.empty())return;
       r=std::move(queue_.front());queue_.pop_front();}
      mkdir("camera_light",0755);
      struct statvfs space{};
      if(statvfs("camera_light",&space)!=0 || (uint64_t)space.f_bavail*space.f_frsize<256ULL*1024*1024){
        if(!warned)std::fprintf(stderr,"CAM light logger: no writable storage or less than 256 MiB free; logging suspended\n");
        warned=true;continue;
      }
      warned=false;
      bool imageOk=false;
      if(!r.jpeg.empty()){
        const std::string path="camera_light/"+r.imageName;
        FILE* f=std::fopen(path.c_str(),"wb");
        if(f){imageOk=std::fwrite(r.jpeg.data(),1,r.jpeg.size(),f)==r.jpeg.size();if(std::fclose(f)!=0)imageOk=false;}
      }
      // A reference is only advertised when its file was successfully written.
      r.line+=",\"reference_jpeg\":"+(imageOk?"\""+r.imageName+"\"":"null")+"}\n";
      const std::string path="camera_light/"+r.day+".jsonl";
      FILE* f=std::fopen(path.c_str(),"ab");
      if(f){if(std::fwrite(r.line.data(),1,r.line.size(),f)!=r.line.size())std::fprintf(stderr,"CAM light logger: write failed\n");std::fclose(f);}
      else std::fprintf(stderr,"CAM light logger: cannot open log\n");
    }
  }
public:
  struct Light {
    double mean=0,dark=0,bright=0,p10=0,p50=0,p90=0,region[6]={},regionP80[6]={};
  };
  static Light measure(const uint8_t* rgb,int w,int h){
    Light out;if(!rgb || w<=0 || h<=0)return out;
    unsigned histogram[256]={},counts[6]={},regionHistogram[6][256]={};unsigned n=0;
    const int step=std::max(1,std::min(w,h)/120);
    for(int y=0;y<h;y+=step)for(int x=0;x<w;x+=step){
      const auto* p=rgb+((size_t)y*w+x)*3;
      const int l=(77*p[0]+150*p[1]+29*p[2])>>8;
      ++histogram[l];++n;out.mean+=l;out.dark+=l<24;out.bright+=l>239;
      int region=std::min(1,y*2/h)*3+std::min(2,x*3/w);out.region[region]+=l;++counts[region];++regionHistogram[region][l];
    }
    if(!n)return out;
    out.mean/=n;out.dark/=n;out.bright/=n;
    for(int i=0;i<6;++i)if(counts[i])out.region[i]/=counts[i];
    for(int r=0;r<6;++r)if(counts[r]){
      unsigned total=0;
      for(int i=0;i<256;++i){total+=regionHistogram[r][i];if(total>=counts[r]*.8){out.regionP80[r]=i;break;}}
    }
    unsigned sum=0;bool a=false,b=false,c=false;
    for(int i=0;i<256;++i){sum+=histogram[i];if(!a&&sum>=n*.1){out.p10=i;a=true;}if(!b&&sum>=n*.5){out.p50=i;b=true;}if(!c&&sum>=n*.9){out.p90=i;c=true;}}
    return out;
  }
  CameraLightLogger():writer_([this]{writeLoop();}){}
  ~CameraLightLogger(){ {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;}wake_.notify_one();writer_.join(); }
  bool due()const{return enabled_ && Clock::now()-start_>=std::chrono::seconds(1);}
  void capture(const v4l2_buffer& b){
    if(!enabled_)return;
    ++frames_;
    if(haveSequence_){uint32_t delta=b.sequence-sequence_;if(delta>1 && delta<100000)gaps_+=delta-1;}
    sequence_=b.sequence;haveSequence_=true;
    double t=b.timestamp.tv_sec+b.timestamp.tv_usec/1e6;
    if(previousTimestamp_>0 && t>previousTimestamp_){double d=t-previousTimestamp_;timestampSpan_+=d;timestampMaxGap_=std::max(timestampMaxGap_,d);++timestampIntervals_;}
    previousTimestamp_=t;
  }
  void invalid(){++invalid_;}
  void decoded(){++decodes_;}
  void detection(bool found){++detected_;found_+=found;}
  void timeout(){++timeouts_;}
  void sample(int fd,int index,int nav,const uint8_t* rgb=nullptr,int w=0,int h=0,
              const uint8_t* jpeg=nullptr,size_t size=0,double confidence=0){
    if(!due())return;
    auto now=Clock::now();double seconds=std::chrono::duration<double>(now-start_).count();
    const double fps=frames_/seconds,alpha=1-std::exp(-seconds/60.0);
    smoothFps_=smoothFps_<0?fps:smoothFps_+alpha*(fps-smoothFps_);
    auto unixMs=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    time_t epoch=unixMs/1000;tm utc{};gmtime_r(&epoch,&utc);char day[16];strftime(day,sizeof(day),"%Y-%m-%d",&utc);
    Record r;r.day=day;
    std::ostringstream s;s.precision(7);
    s<<"{\"schema\":1,\"unix_ms\":"<<unixMs<<",\"monotonic_ms\":"<<std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
     <<",\"camera_index\":"<<index<<",\"nav_mode\":"<<nav<<",\"window_s\":"<<seconds
     <<",\"capture_fps\":"<<fps<<",\"capture_fps_ema60\":"<<smoothFps_
     <<",\"buffer_timestamp_fps\":";if(timestampSpan_>0)s<<timestampIntervals_/timestampSpan_;else s<<"null";
    s<<",\"max_frame_gap_ms\":"<<timestampMaxGap_*1000<<",\"sequence_gaps\":"<<gaps_
     <<",\"invalid_frames\":"<<invalid_<<",\"decoded_frames\":"<<decodes_<<",\"select_timeouts\":"<<timeouts_
     <<",\"detection_count\":"<<detected_<<",\"found_count\":"<<found_<<",\"confidence\":"<<confidence
     <<",\"queue_dropped\":"<<dropped_<<",\"image_valid\":"<<(rgb?"true":"false");
    field(s,"exposure_raw_100us",control(fd,V4L2_CID_EXPOSURE_ABSOLUTE));
    field(s,"gain_raw",control(fd,V4L2_CID_GAIN));field(s,"auto_exposure",control(fd,V4L2_CID_EXPOSURE_AUTO));
    field(s,"dynamic_framerate",control(fd,V4L2_CID_EXPOSURE_AUTO_PRIORITY));
    if(rgb){
      Light l=measure(rgb,w,h);smoothLight_=smoothLight_<0?l.mean:smoothLight_+alpha*(l.mean-smoothLight_);
      s<<",\"width\":"<<w<<",\"height\":"<<h<<",\"luma_mean\":"<<l.mean<<",\"luma_ema60\":"<<smoothLight_
       <<",\"dark_fraction\":"<<l.dark<<",\"bright_fraction\":"<<l.bright
       <<",\"p10\":"<<l.p10<<",\"p50\":"<<l.p50<<",\"p90\":"<<l.p90<<",\"regions_2x3\":[";
      for(int i=0;i<6;++i)s<<(i?",":"")<<l.region[i];
      s<<"]";
      if(jpeg && size<=2*1024*1024 && now-lastImage_>=std::chrono::seconds(300)){
        r.imageName=std::to_string(unixMs)+"-cam"+std::to_string(index)+".jpg";r.jpeg.assign(jpeg,jpeg+size);lastImage_=now;
      }
    }
    r.line=s.str();
    {std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);if(lock.owns_lock()&&queue_.size()<8){queue_.push_back(std::move(r));wake_.notify_one();}else ++dropped_;}
    start_=now;frames_=gaps_=invalid_=decodes_=detected_=found_=timeouts_=0;timestampSpan_=timestampMaxGap_=0;timestampIntervals_=0;
  }
};
#endif
