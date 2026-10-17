#ifdef __linux__
#include "ReflectorStripsDetector.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include "../../config.h"
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifndef DOCK_REFLECTOR_STRIPS_ROI_TOP
#define DOCK_REFLECTOR_STRIPS_ROI_TOP 0.40f
#endif
#ifndef DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN
#define DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN 0.055f
#endif
#ifndef DOCK_REFLECTOR_STRIPS_MAX_SLOPE
#define DOCK_REFLECTOR_STRIPS_MAX_SLOPE 0.25f
#endif
#ifndef DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO
#define DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO 1.6f
#endif
#ifndef DOCK_REFLECTOR_STRIPS_MAX_SPACING_RATIO
#define DOCK_REFLECTOR_STRIPS_MAX_SPACING_RATIO 1.45f
#endif
#ifndef DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES
#define DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES 3
#endif
#ifndef DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS
#define DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS 500
#endif
namespace {
struct Strip {float x,y,width,height,slope,top,bottom;bool cropped,leftCut,rightCut;};
struct Marker {float x[3],width,span,quality;};
static bool similar(float a,float b,float ratio){return std::max(a,b)<=std::min(a,b)*ratio;}
}
ReflectorStripsDetector& ReflectorStripsDetector::instance(){static ReflectorStripsDetector detector;return detector;}
void ReflectorStripsDetector::clear(){
  acquireCount_=0;found_.store(false,std::memory_order_release);
  confidence_.store(0);stripCount_.store(0);verticalSpan_.store(0);timestampMs_.store(0);
}
void ReflectorStripsDetector::processRgb(const uint8_t* rgb,int width,int height,unsigned long timestampMs){
  if(!rgb || width<32 || height<32){clear();return;}
  const int w=std::min(width,640),h=height*w/width;
  if(h<16){clear();return;}
  const int roi=std::max(0,std::min(h-1,int(h*DOCK_REFLECTOR_STRIPS_ROI_TOP)));
  std::vector<float> light(w*h),neutral(w*h);
  float maximum=0;
  // Average pooling prevents individual hot pixels from becoming strips.
  for(int y=0;y<h;++y)for(int x=0;x<w;++x){
    float sum=0,white=0;int n=0;
    for(int yy=y*height/h;yy<(y+1)*height/h;++yy)for(int xx=x*width/w;xx<(x+1)*width/w;++xx){
      const auto* p=rgb+((size_t)yy*width+xx)*3;
      const int lo=std::min(p[0],std::min(p[1],p[2])),hi=std::max(p[0],std::max(p[1],p[2]));
      sum+=lo;if(hi-lo<=hi*.35f)white+=lo;++n;
    }
    light[y*w+x]=sum/n;neutral[y*w+x]=white/n;
    if(y>=roi)maximum=std::max(maximum,neutral[y*w+x]);
  }
  // Small absolute floors reject darkness without excluding dim reflectors;
  // relative thresholds, triple geometry and temporal acquisition still apply.
  if(maximum<12){clear();return;}
  std::vector<Marker> markers;
  // Exposure-relative levels retain dim reflectors as well as saturated ones.
  // Resolve the intermediate levels too: blurred distant strips can have only
  // a narrow threshold interval between background attachment and disappearance.
  // A candidate must pass geometry and dark-gap checks at one of these levels.
  for(float fraction : {.25f,.30f,.35f,.40f,.45f,.50f,.55f,.60f,.70f,.85f}){
    const float threshold=std::max(6.0f,maximum*fraction);
    // Search complete components, including their portion above the lower-image
    // anchor. A close marker may extend upwards while still standing below it.
    // Bridge only short vertical interruptions (e.g. a cable); never bridge the
    // horizontal dark gaps separating the three strips.
    std::vector<uint8_t> mask(w*h,0);
    for(int i=0;i<w*h;++i)mask[i]=neutral[i]>=threshold;
    const int maxGap=std::max(1,int(h*.04f));
    for(int x=0;x<w;++x){
      int previous=-1;
      for(int y=0;y<h;++y)if(neutral[y*w+x]>=threshold){
        if(previous>=0 && y-previous-1<=maxGap)
          for(int yy=previous+1;yy<y;++yy)mask[yy*w+x]=1;
        previous=y;
      }
    }
    std::vector<uint8_t> seen(w*h,0);std::vector<int> queue;
    std::vector<Strip> strips;
    for(int start=0;start<w*h;++start){
      if(seen[start] || !mask[start])continue;
      queue.clear();queue.push_back(start);seen[start]=1;
      int minx=w,maxx=0,miny=h,maxy=0;
      std::vector<int> left(h,w),right(h,-1),count(h,0);
      std::vector<float> sumx(h,0);
      for(size_t i=0;i<queue.size();++i){
        const int x=queue[i]%w,y=queue[i]/w;
        minx=std::min(minx,x);maxx=std::max(maxx,x);miny=std::min(miny,y);maxy=std::max(maxy,y);
        left[y]=std::min(left[y],x);right[y]=std::max(right[y],x);sumx[y]+=x;++count[y];
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
          const int nx=x+dx,ny=y+dy;
          if(nx<0 || nx>=w || ny<0 || ny>=h)continue;
          const int next=ny*w+nx;
          if(!seen[next] && mask[next]){seen[next]=1;queue.push_back(next);}
        }
      }
      const float bh=maxy-miny+1;
      // At charging contact the marker can fill almost the entire frame.
      // A very close marker can extend beyond the top/bottom image edges.
      // Permit that only for a long visible segment, and below require all
      // three strips to span most of the image. Side clipping has additional
      // outer-strip checks when forming a triple below.
      // Width is bounded separately; the triple geometry and aspect ratio
      // continue to reject broad bright panels.
      const bool cropped=miny<=0 || maxy>=h-1;
      const bool leftCut=minx<2,rightCut=maxx>=w-2;
      const bool close=cropped && bh>=h*.85f;
      if((cropped && bh<h*.70f) || maxy<roi || ((leftCut || rightCut) && !close) ||
         bh<h*DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN || maxy<h*.50f)continue;
      double sx=0,sy=0,sxy=0,syy=0;float widths=0;int rows=0;
      for(int y=miny;y<=maxy;++y)if(count[y]){
        const float x=sumx[y]/count[y];++rows;sx+=x;sy+=y;sxy+=x*y;syy+=y*y;widths+=right[y]-left[y]+1;
      }
      const float bw=widths/rows;
      if(bw<1.5f || bw>w*(close?.22f:.16f) || bh<bw*(close?2.5f:3.f) || rows<bh*.9f || queue.size()<widths*.55f)continue;
      const double denominator=rows*syy-sy*sy;
      const float slope=denominator>0?(rows*sxy-sx*sy)/denominator:0;
      const float intercept=(sx-slope*sy)/rows;
      if(std::fabs(slope)>DOCK_REFLECTOR_STRIPS_MAX_SLOPE)continue;
      double residual=0;int edges=0;
      for(int y=miny;y<=maxy;++y)if(count[y]){
        const float x=sumx[y]/count[y],d=x-(slope*y+intercept);residual+=d*d;
        const int offset=std::max(2,int(bw*.6f));
        // At close range the outer strips can be fully visible but leave only
        // a narrow dark margin. Sample that margin instead of discarding a row
        // when the preferred contrast offset would fall outside the image.
        // A clipped outer side is checked through its visible inner edge;
        // the triple validation below limits this to a near-field marker.
        const int l=std::max(0,left[y]-offset),r=std::min(w-1,right[y]+offset);
        const float core=light[y*w+int(x)],contrast=std::max(4.0f,core*.15f);
        if((leftCut || core-light[y*w+l]>=contrast) && (rightCut || core-light[y*w+r]>=contrast))++edges;
      }
      if(residual>rows*std::pow(std::max(1.2f,bw*.25f),2) || edges<rows*.7f)continue;
      strips.push_back({float(sx/rows),float(sy/rows),bw,bh,slope,float(miny),float(maxy),cropped,leftCut,rightCut});
    }
    // Bound combinatorial work in highly textured scenes; abstain at that level.
    if(strips.size()>64)continue;
    std::sort(strips.begin(),strips.end(),[](const Strip&a,const Strip&b){return a.x<b.x;});
    for(size_t i=0;i<strips.size();++i)for(size_t j=i+1;j<strips.size();++j)for(size_t k=j+1;k<strips.size();++k){
      const Strip* a[3]={&strips[i],&strips[j],&strips[k]};
      // Only OUTER strips may touch side edges; never invent a missing strip
      // or accept a clipped centre. Two clipped sides need the stricter
      // full-height and visible-width checks below.
      if(a[0]->rightCut || a[1]->leftCut || a[1]->rightCut || a[2]->leftCut)continue;
      const bool bothSidesCut=a[0]->leftCut && a[2]->rightCut;
      const bool sideCut=a[0]->leftCut || a[2]->rightCut;
      float effectiveW[3]={a[0]->width,a[1]->width,a[2]->width};
      if(sideCut){
        // Side clipping is a special case for a marker filling the frame.
        // All three strips must also reach a vertical image edge. Otherwise
        // bright scenery at a side border plus two real strips can form a
        // spurious second triple while the actual marker's ends are visible.
        if(!a[0]->cropped || !a[1]->cropped || !a[2]->cropped)continue;
        if(a[0]->height<h*.85f || a[1]->height<h*.85f || a[2]->height<h*.85f)continue;
        if(bothSidesCut){
          // No complete outer width is available. Require all three strips
          // to span BOTH vertical borders and substantial, comparable visible
          // outer segments. Use only a conservative lower bound on full width;
          // the directly measured middle strip remains the steering target.
          if(a[0]->top>0 || a[1]->top>0 || a[2]->top>0 ||
             a[0]->bottom<h-1 || a[1]->bottom<h-1 || a[2]->bottom<h-1)continue;
          if(a[0]->width<a[1]->width*.5f || a[2]->width<a[1]->width*.5f ||
             !similar(a[0]->width,a[2]->width,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO))continue;
          const float outerWidth=std::max(std::max(a[0]->width,a[2]->width),
              a[1]->width/float(DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO));
          effectiveW[0]=effectiveW[2]=outerWidth;
        }else{
          const int n=a[0]->leftCut?0:2;
          // Compare the outer strips symmetrically: wide-angle optics can make
          // the centre strip wider than either outer strip at very close range.
          const float expectedW=a[2-n]->width;
          if(a[n]->width<expectedW*.60f ||
             !similar(std::max(a[n]->width,expectedW),expectedW,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO))continue;
          // Perspective can make the border-touching outer strip wider than
          // its opposite. Never shrink an actually observed width.
          effectiveW[n]=std::max(a[n]->width,expectedW);
        }
      }
      float minH=a[0]->height,maxH=minH,minW=a[0]->width,maxW=minW,top=a[0]->top,bottom=a[0]->bottom;
      for(int n=1;n<3;++n){minH=std::min(minH,a[n]->height);maxH=std::max(maxH,a[n]->height);minW=std::min(minW,a[n]->width);maxW=std::max(maxW,a[n]->width);top=std::max(top,a[n]->top);bottom=std::min(bottom,a[n]->bottom);}
      minW=std::min(effectiveW[0],std::min(effectiveW[1],effectiveW[2]));
      maxW=std::max(effectiveW[0],std::max(effectiveW[1],effectiveW[2]));
      if((a[0]->cropped || a[1]->cropped || a[2]->cropped) && minH<h*.70f)continue;
      // In the extreme near field, perspective/lens distortion can make the
      // centre wider than both outer strips. Compare it to their mean, while
      // still requiring the outer pair to agree. This rule applies equally
      // just before and just after an outer strip leaves the image border.
      const float outerMean=(effectiveW[0]+effectiveW[2])*.5f;
      const bool nearWidths=minH>=h*.85f &&
        similar(effectiveW[0],effectiveW[2],DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) &&
        similar(effectiveW[1],outerMean,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO);
      if(!similar(minH,maxH,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) ||
         (!similar(minW,maxW,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) && !nearWidths) || bottom-top<maxH*.70f)continue;
      if(std::fabs(a[0]->slope-a[1]->slope)>.12f || std::fabs(a[2]->slope-a[1]->slope)>.12f)continue;
      const float y=(top+bottom)*.5f;
      Marker m;float shift[3]={0,0,0};
      if(a[0]->leftCut)shift[0]=-(effectiveW[0]-a[0]->width)*.5f;
      if(a[2]->rightCut)shift[2]=(effectiveW[2]-a[2]->width)*.5f;
      for(int n=0;n<3;++n)m.x[n]=a[n]->x+a[n]->slope*(y-a[n]->y)+shift[n];
      const float d1=m.x[1]-m.x[0],d2=m.x[2]-m.x[1];
      if(d1<maxW*1.5f || d2<maxW*1.5f || std::max(d1,d2)>minW*10 || !similar(d1,d2,DOCK_REFLECTOR_STRIPS_MAX_SPACING_RATIO))continue;
      // Require dark gaps over most of the common height, not three highlights
      // on one white object. This also excludes four aligned stripes: they
      // produce multiple different plausible triplets and remain ambiguous.
      int dark=0,total=0;
      for(int yy=int(top);yy<=int(bottom);++yy){
        float xs[3];for(int n=0;n<3;++n)xs[n]=a[n]->x+a[n]->slope*(yy-a[n]->y)+shift[n];
        ++total;
        if(xs[0]<0 || xs[1]<0 || xs[2]<0 || xs[0]>=w || xs[1]>=w || xs[2]>=w)continue;
        const float core=std::min(light[yy*w+int(xs[0])],std::min(light[yy*w+int(xs[1])],light[yy*w+int(xs[2])]));
        if(light[yy*w+int((xs[0]+xs[1])*.5f)]<core*.8f && light[yy*w+int((xs[1]+xs[2])*.5f)]<core*.8f)++dark;
      }
      if(dark<total*.8f)continue;
      m.width=minW;m.span=minH/h;m.quality=(minH/maxH+minW/maxW+std::min(d1,d2)/std::max(d1,d2))/3;
      bool duplicate=false;
      for(auto& old:markers){
        bool same=true;for(int n=0;n<3;++n)if(std::fabs(old.x[n]-m.x[n])>std::max(2.0f,std::min(old.width,m.width)*.5f))same=false;
        if(same){if(m.quality>old.quality)old=m;duplicate=true;break;}
      }
      if(!duplicate)markers.push_back(m);
    }
  }
  if(markers.size()!=1){clear();return;}
  const Marker& marker=markers.front();
  const float error=(marker.x[1]-w*.5f)/(w*.5f);
  const unsigned long previous=timestampMs_.load(std::memory_order_relaxed);
  if(acquireCount_ && (timestampMs-previous>DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS || std::fabs(error-horizontalError_.load())>.08f))acquireCount_=0;
  if(acquireCount_<DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES)++acquireCount_;
  horizontalError_.store(error);confidence_.store(marker.quality);stripCount_.store(3);verticalSpan_.store(marker.span);timestampMs_.store(timestampMs);
  found_.store(acquireCount_>=DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES,std::memory_order_release);
}
ReflectorStripsObservation ReflectorStripsDetector::observation(unsigned long nowMs)const{
  ReflectorStripsObservation r;
  const unsigned long timestamp=timestampMs_.load();
  r.ageMs=timestamp?nowMs-timestamp:(unsigned long)-1;
  r.horizontalError=horizontalError_.load();r.confidence=confidence_.load();r.stripCount=stripCount_.load();r.verticalSpan=verticalSpan_.load();
  r.found=found_.load(std::memory_order_acquire)&&timestamp&&r.ageMs<=DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS;
  return r;
}
#endif
