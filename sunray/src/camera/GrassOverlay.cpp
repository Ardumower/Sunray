#ifdef __linux__
#include "GrassOverlay.h"
#include "GrassObstacle.h"
#include <atomic>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#ifdef SUNRAY_RKNN
#include <rknn_api.h>
#endif
namespace {
std::atomic<uint64_t> observation{0};
std::mutex maskMutex;
std::array<uint8_t,3600> maskLabels{};
uint64_t maskStamp=0;
std::atomic<bool> modelReady{false}; // capture timestamp * 2 + blocked bit
uint64_t monotonicMs(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
}
bool GrassOverlay::available(){return modelReady.load();}
bool GrassOverlay::mask(std::array<uint8_t,3600>& labels,uint64_t& stamp){
 std::unique_lock<std::mutex> lock(maskMutex,std::try_to_lock);
 if(!lock||!maskStamp||maskStamp==stamp)return false;labels=maskLabels;stamp=maskStamp;return true;
}
int GrassOverlay::obstacleState(){
  const uint64_t value=observation.load(), stamp=value>>1, now=monotonicMs();
  return GrassObstacle::fresh(stamp,now) ? int(value&1) : -1;
}
bool GrassOverlay::obstaclePresent(){return obstacleState()==1;}
struct GrassOverlay::Impl {
  std::mutex mutex;
  std::condition_variable ready;
  std::thread worker;
  bool stop=false;
  std::vector<uint8_t> pending, result;
  uint64_t pendingTime=0,resultTime=0;
  double ms=0;
  ~Impl(){ {std::lock_guard<std::mutex> lock(mutex);stop=true;}ready.notify_one();if(worker.joinable())worker.join();observation.store(0); }
  void run(){
#ifdef SUNRAY_RKNN
    char exe[4096];const auto len=readlink("/proc/self/exe",exe,sizeof(exe)-1);
    std::string path="models/grass.rknn";
    if(len>0){exe[len]=0;std::string s(exe);path=s.substr(0,s.find_last_of('/'))+"/models/grass.rknn";}
    if(const char* env=std::getenv("SUNRAY_GRASS_MODEL"))path=env;
    rknn_context ctx=0;
    int ret=rknn_init(&ctx,const_cast<char*>(path.c_str()),0,0,nullptr);
    if(ret<0){fprintf(stderr,"GRASS model unavailable: %s (%d)\n",path.c_str(),ret);return;}
    rknn_input_output_num io={};rknn_tensor_attr attr={};
    if(rknn_query(ctx,RKNN_QUERY_IN_OUT_NUM,&io,sizeof(io))<0||io.n_input!=1||io.n_output!=1||
       rknn_query(ctx,RKNN_QUERY_OUTPUT_ATTR,&attr,sizeof(attr))<0||attr.n_dims!=4||
       attr.fmt!=RKNN_TENSOR_NCHW||attr.dims[0]!=1||attr.dims[1]!=5||attr.dims[2]!=45||attr.dims[3]!=80||
       rknn_set_core_mask(ctx,RKNN_NPU_CORE_0)<0){fprintf(stderr,"GRASS unsupported model\n");rknn_destroy(ctx);return;}
    fprintf(stderr,"GRASS ready: %s\n",path.c_str());
    modelReady=true;
    unsigned frames=0; bool blocked=false; uint64_t lastStamp=0;
    while(true){
      std::vector<uint8_t> frame;uint64_t stamp;
      {std::unique_lock<std::mutex> lock(mutex);ready.wait(lock,[this]{return stop||!pending.empty();});if(stop)break;frame.swap(pending);stamp=pendingTime;}
      rknn_input in={};in.buf=frame.data();in.size=frame.size();in.type=RKNN_TENSOR_UINT8;in.fmt=RKNN_TENSOR_NHWC;
      const auto start=std::chrono::steady_clock::now();
      if(rknn_inputs_set(ctx,1,&in)<0||rknn_run(ctx,nullptr)<0){fprintf(stderr,"GRASS inference failed\n");break;}
      rknn_output out={};out.want_float=1;
      if(rknn_outputs_get(ctx,1,&out,nullptr)<0){fprintf(stderr,"GRASS output failed\n");break;}
      const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      if(out.size!=5*45*80*sizeof(float)){rknn_outputs_release(ctx,1,&out);break;}
      const float* scores=static_cast<const float*>(out.buf);
      uint8_t labels[45*80];
      for(int i=0;i<45*80;i++){int best=0;for(int c=1;c<5;c++)if(scores[c*45*80+i]>scores[best*45*80+i])best=c;labels[i]=best;}
      rknn_outputs_release(ctx,1,&out);
      if(stamp<lastStamp || stamp-lastStamp>=GrassObstacle::FreshMs)blocked=false;
      {std::lock_guard<std::mutex> lock(maskMutex);std::copy(labels,labels+3600,maskLabels.begin());maskStamp=stamp;}
      blocked=GrassObstacle::detect(labels,blocked);lastStamp=stamp;
      observation.store((stamp<<1)|(blocked?1:0));
      // Obstacle, grass, excluded region, non-grass, restricted region.
      const uint8_t colors[5][3]={{235,65,65},{45,210,65},{160,90,220},{230,155,40},{70,130,230}};
      for(int y=0;y<360;y++)for(int x=0;x<640;x++)for(int c=0;c<3;c++){
        const auto i=(y*640+x)*3+c;frame[i]=(frame[i]*20+colors[labels[(y/8)*80+x/8]][c]*80)/100;
        if(GrassObstacle::border(x,y)) frame[i]=blocked?(c==0?255:0):(c==1?255:0);
      }
      {std::lock_guard<std::mutex> lock(mutex);result.swap(frame);resultTime=stamp;ms=elapsed;}
      if (++frames == 1) fprintf(stderr,"GRASS first live overlay complete: %.1f ms, timestamp %llu\n",elapsed,(unsigned long long)stamp);
    }
    modelReady=false;
    rknn_destroy(ctx);
#else
    fprintf(stderr,"GRASS unavailable: build without RKNN\n");
#endif
  }
};
GrassOverlay::GrassOverlay():impl(new Impl){}
GrassOverlay::~GrassOverlay()=default;
void GrassOverlay::submit(const uint8_t* rgb,int w,int h,uint64_t stamp){
  if(!rgb||w<=0||h<=0)return;
  std::vector<uint8_t> frame(640*360*3);
  for(int y=0;y<360;y++)for(int x=0;x<640;x++)for(int c=0;c<3;c++)frame[(y*640+x)*3+c]=rgb[((y*h/360)*w+x*w/640)*3+c];
  {std::lock_guard<std::mutex> lock(impl->mutex);impl->pending.swap(frame);impl->pendingTime=stamp;
    if(!impl->worker.joinable())impl->worker=std::thread([this]{impl->run();});}
  impl->ready.notify_one();
}
bool GrassOverlay::latest(std::vector<uint8_t>& rgb,uint64_t& stamp,double& ms){
  std::lock_guard<std::mutex> lock(impl->mutex);if(impl->result.empty())return false;
  rgb=impl->result;stamp=impl->resultTime;ms=impl->ms;return true;
}
#endif
