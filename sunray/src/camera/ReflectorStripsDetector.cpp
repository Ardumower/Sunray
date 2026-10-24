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
#define DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN 0.04f
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
struct GradientData {
  struct Edge {float x,slope;int score;};
  std::vector<float> values;
  std::vector<int> positivePeak,negativePeak;
  std::vector<Edge> edges[2];
  std::vector<Strip> strips[2];
  bool complete=false;
};
static bool similar(float a,float b,float ratio){return std::max(a,b)<=std::min(a,b)*ratio;}
// Near-field fallback based on paired vertical edges. No sun/shadow layout or
// absolute brightness distribution is assumed. Missing edge rows (e.g. cables)
// are tolerated only when both real sides persist over most of a long strip.
static std::vector<Strip> gradientStrips(const std::vector<float>& light,const std::vector<float>& neutral,int w,int h,bool localContrast,GradientData& data){
  if(data.complete)return data.strips[localContrast?1:0];
  auto& gradient=data.values;
  auto& edges=data.edges;
  using Edge=GradientData::Edge;
  const float centreY=(h-1)*.5f;
  // Share image gradients and line votes between the two contrast models.
  if(gradient.empty()){
  gradient.assign(w*h,0);
  for(int y=0;y<h;++y)for(int x=3;x<w-3;++x){
    float g=0;for(int d=1;d<=3;++d)g+=light[y*w+x+d]-light[y*w+x-d];
    gradient[y*w+x]=g/3;
  }
  // Peak locations depend only on the image, not on a proposed edge pair.
  // Precompute once instead of repeating fourteen comparisons per pair/row.
  data.positivePeak.resize(w*h);data.negativePeak.resize(w*h);
  for(int y=0;y<h;++y)for(int x=3;x<w-3;++x){
    float positive=0,negative=0;int l=x,r=x;
    for(int dx=-3;dx<=3;++dx){
      const float g=gradient[y*w+x+dx];
      if(g>positive){positive=g;l=x+dx;}
      if(-g>negative){negative=-g;r=x+dx;}
    }
    data.positivePeak[y*w+x]=l;data.negativePeak[y*w+x]=r;
  }
  for(int polarity=0;polarity<2;++polarity){
    const int sign=polarity==0?1:-1;
    std::vector<Edge> all;
    // Extract local peaks once; voting for each slope must not rescan every
    // background pixel. This keeps extra line hypotheses affordable.
    struct Peak {int x,y;};
    std::vector<Peak> peaks;
    for(int y=0;y<h;++y)for(int x=4;x<w-4;++x){
      const float g=sign*gradient[y*w+x];
      if(g>=4 && g>=sign*gradient[y*w+x-1] && g>sign*gradient[y*w+x+1])peaks.push_back({x,y});
    }
    for(int step=-10;step<=10;++step){
      const float slope=step*.025f;std::vector<int> votes(w,0);
      for(const auto& peak:peaks){
        const int centre=int(std::lround(peak.x-slope*(peak.y-centreY)));
        for(int dx=-2;dx<=2;++dx)if(centre+dx>=0 && centre+dx<w)++votes[centre+dx];
      }
      for(int x=1;x<w-1;++x)if(votes[x]>=std::max(12.f,h*.08f) && votes[x]>=votes[x-1] && votes[x]>votes[x+1])all.push_back({float(x),slope,votes[x]});
    }
    std::sort(all.begin(),all.end(),[](const Edge&a,const Edge&b){return a.score>b.score;});
    std::vector<int> perColumn((w+15)/16,0);
    for(const auto&e:all){
      bool duplicate=false;for(const auto&old:edges[polarity])if(std::fabs(e.x-old.x)<8 && std::fabs(e.slope-old.slope)<.075f)duplicate=true;
      // Reserve candidates across the image. Long edges in the fence/backing
      // must not evict shorter marker edges from another part of the frame.
      // Keep alternate slopes too: a shadow can dominate the initial vote.
      const int column=int(e.x)/16;
      if(!duplicate && perColumn[column]<12){
        edges[polarity].push_back(e);++perColumn[column];
      }
    }
  }
  } // cached gradients and line votes
  // Each line hypothesis participates in many pairs. Measure its row positions
  // and local gradient once, rather than repeating rounding/peak lookup for
  // every possible partner. This preserves all candidates and thresholds.
  struct EdgeRow { int predicted,actual; float strength; };
  std::vector<std::vector<EdgeRow>> edgeRows[2];
  std::vector<std::vector<uint64_t>> strongRows[2];
  for(int side=0;side<2;++side){
    edgeRows[side].resize(edges[side].size());
    strongRows[side].resize(edges[side].size(),std::vector<uint64_t>((h+63)/64,0));
    for(size_t i=0;i<edges[side].size();++i){
      const auto& edge=edges[side][i];auto& rows=edgeRows[side][i];rows.resize(h);
      for(int y=0;y<h;++y){
        const int x=std::lround(edge.x+edge.slope*(y-centreY));
        if(x<6 || x>=w-6){rows[y]={x,-1,0};continue;}
        const int actual=side?data.negativePeak[y*w+x]:data.positivePeak[y*w+x];
        rows[y]={x,actual,std::max(0.f,(side?-1.f:1.f)*gradient[y*w+actual])};
        if(rows[y].strength>=4)strongRows[side][i][y/64]|=uint64_t(1)<<(y%64);
      }
    }
  }
  std::vector<float> supportScores[2];
  for(size_t li=0;li<edges[0].size();++li)for(size_t ri=0;ri<edges[1].size();++ri){
    const auto& left=edges[0][li];const auto& right=edges[1][ri];
    const float width=right.x-left.x;
    if(width<std::max(3.f,w*.008f) || width>w*.16f || std::fabs(left.slope-right.slope)>.075f)continue;
    // A valid strip needs both edges on at least 65% of its minimum height.
    // The bitset intersection is an upper bound on that support: skip only
    // pairs that cannot possibly pass the unchanged detailed row checks.
    unsigned possibleRows=0;
    for(size_t word=0;word<strongRows[0][li].size();++word)
      possibleRows+=__builtin_popcountll(strongRows[0][li][word]&strongRows[1][ri][word]);
    const float minimumHeight=std::max(std::max(h*.08f,h*float(DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN)),width*3);
    if(possibleRows<minimumHeight*.65f)continue;
    std::vector<int> supported[2];
    std::vector<float> rowX(h),rowWidth(h);
    for(int y=0;y<h;++y){
      const auto& leftRow=edgeRows[0][li][y];const auto& rightRow=edgeRows[1][ri][y];
      const int l=leftRow.predicted,r=rightRow.predicted;
      if(leftRow.actual<0 || rightRow.actual<0 || r-l<width*.7f || r-l>width*1.3f)continue;
      const int actualL=leftRow.actual,actualR=rightRow.actual;
      const float positive=leftRow.strength,negative=rightRow.strength;
      if(actualL<5 || actualR>=w-5 || actualR-actualL<width*.7f || actualR-actualL>width*1.3f)continue;
      const int centre=(actualL+actualR)/2;
      const float core=light[y*w+centre];
      const int inside=std::max(1,std::min(5,(actualR-actualL)/3));
      const float leftCore=std::max(core,light[y*w+actualL+inside]);
      const float rightCore=std::max(core,light[y*w+actualR-inside]);
      if(positive<4 || negative<4 || neutral[y*w+centre]<core*.70f)continue;
      const float outsideLeft=light[y*w+actualL-5],outsideRight=light[y*w+actualR+5];
      if(core-outsideLeft>=std::max(4.f,core*.03f) && core-outsideRight>=std::max(4.f,core*.03f))supported[0].push_back(y);
      if(leftCore-outsideLeft>=std::max(4.f,leftCore*.03f) && rightCore-outsideRight>=std::max(4.f,rightCore*.03f))supported[1].push_back(y);
      rowX[y]=(actualL+actualR)*.5f;rowWidth[y]=actualR-actualL;
    }
    // The expensive paired-edge sampling is shared by both contrast models.
    for(int model=0;model<2;++model){
    const auto& supportedRows=supported[model];
    auto& strips=data.strips[model];auto& stripSupport=supportScores[model];
    int rows=0,top=h,bottom=0;double sumY=0;
    // Isolated matching background pixels must not extend a strip's ends and
    // dilute its support. Keep the longest supported segment, bridging a cable
    // gap of at most one strip width. This scales with the marker distance.
    size_t bestBegin=0,bestEnd=0,begin=0;
    for(size_t end=0;end<supportedRows.size();++end){
      if(end && supportedRows[end]-supportedRows[end-1]>std::max(4.f,width))begin=end;
      if(end+1-begin>bestEnd-bestBegin){bestBegin=begin;bestEnd=end+1;}
    }
    for(size_t i=bestBegin;i<bestEnd;++i){
      const int y=supportedRows[i];++rows;sumY+=y;
      top=std::min(top,y);bottom=std::max(bottom,y);
    }
    const float height=bottom-top+1;
    if(height<std::max(h*.08f,h*float(DOCK_REFLECTOR_STRIPS_MIN_VERTICAL_SPAN)) ||
       rows<height*.65f || height<width*3 ||
       bottom<h*.50f || bottom<h*DOCK_REFLECTOR_STRIPS_ROI_TOP)continue;
    const float y=sumY/rows;double sx=0,sxy=0,syy=0,sw=0;
    // Fit geometry to supported pixels, not to quantized Hough hypotheses.
    for(size_t i=bestBegin;i<bestEnd;++i){int yy=supportedRows[i];sx+=rowX[yy];sxy+=(yy-y)*rowX[yy];syy+=(yy-y)*(yy-y);sw+=rowWidth[yy];}
    const float slope=syy?sxy/syy:0,x=sx/rows,measuredWidth=sw/rows;
    double residual=0;for(size_t i=bestBegin;i<bestEnd;++i){int yy=supportedRows[i];residual+=std::pow(rowX[yy]-x-slope*(yy-y),2);}
    if(std::fabs(slope)>DOCK_REFLECTOR_STRIPS_MAX_SLOPE || residual>rows*std::pow(std::max(1.2f,measuredWidth*.15f),2))continue;
    const Strip candidate{x,y,measuredWidth,height,slope,float(top),float(bottom),top==0||bottom==h-1,false,false};
    // Prefer continuous measured support, not length alone. Sparse background
    // matches must not extend a good strip and replace its real endpoints.
    const float support=float(rows)*rows/height;
    bool duplicate=false;
    for(size_t i=0;i<strips.size();++i){
      auto& old=strips[i];
      if(std::fabs(x-old.x)<width*.25f && similar(width,old.width,1.2f) && std::fabs(slope-old.slope)<.05f && std::fabs(height-old.height)<std::max(width,height*.15f)){
        if(support>stripSupport[i]){old=candidate;stripSupport[i]=support;}
        duplicate=true;break;
      }
    }
    if(!duplicate){strips.push_back(candidate);stripSupport.push_back(support);}
  }
  } // both contrast models
  // Contrast variants may expose only a fragment of a strip that the other
  // variant measures completely. Do not let that fragment combine with a
  // shorter background object into a second, spurious marker.
  std::vector<Strip> completeStrips[2];
  for(int model=0;model<2;++model)for(size_t i=0;i<data.strips[model].size();++i){
    const Strip& a=data.strips[model][i];bool fragment=false;
    for(int other=0;other<2 && !fragment;++other)for(size_t j=0;j<data.strips[other].size();++j){
      const Strip& b=data.strips[other][j];
      if(b.height<=a.height*1.25f || supportScores[other][j]<=supportScores[model][i]*1.15f ||
         !similar(a.width,b.width,1.3f) || std::fabs(a.slope-b.slope)>.12f)continue;
      const float overlap=std::min(a.bottom,b.bottom)-std::max(a.top,b.top)+1;
      const float bx=b.x+b.slope*(a.y-b.y);
      if(overlap>=a.height*.90f && std::fabs(a.x-bx)<std::min(a.width,b.width)*.30f){fragment=true;break;}
    }
    if(!fragment)completeStrips[model].push_back(a);
  }
  for(int model=0;model<2;++model)data.strips[model].swap(completeStrips[model]);
  // A boundary model may measure a full strip while the other measures its
  // illumination differently. Validate both contrast models on the shared
  // geometric candidates instead of discarding a valid line combination.
  std::vector<Strip> combined=data.strips[0];
  for(const auto& a:data.strips[1]){
    bool duplicate=false;
    for(const auto& b:combined)if(std::fabs(a.x-b.x)<std::min(a.width,b.width)*.15f && similar(a.width,b.width,1.15f) && similar(a.height,b.height,1.1f) && std::fabs(a.slope-b.slope)<.03f){duplicate=true;break;}
    if(!duplicate)combined.push_back(a);
  }
  data.strips[0]=combined;data.strips[1]=combined;
  data.complete=true;
  return data.strips[localContrast?1:0];
}

}
ReflectorStripsDetector& ReflectorStripsDetector::instance(){static ReflectorStripsDetector detector;return detector;}
void ReflectorStripsDetector::clear(){
  std::lock_guard<std::mutex> lock(writerMutex_);
  clearUnlocked();
}
void ReflectorStripsDetector::clearUnlocked(){
  acquireCount_=0;found_.store(false,std::memory_order_release);
  confidence_.store(0);stripCount_.store(0);verticalSpan_.store(0);horizontalSpan_.store(0);timestampMs_.store(0);
}
void ReflectorStripsDetector::processRgb(const uint8_t* rgb,int width,int height,unsigned long timestampMs){
  std::lock_guard<std::mutex> lock(writerMutex_);
  if(!rgb || width<32 || height<32){clearUnlocked();return;}
  const int w=std::min(width,640),h=height*w/width;
  if(h<16){clearUnlocked();return;}
  const int roi=std::max(0,std::min(h-1,int(h*DOCK_REFLECTOR_STRIPS_ROI_TOP)));
  std::vector<float> light(w*h),neutral(w*h),dawnNeutral(w*h);
  float maximum=0;int dawnPixels=0;
  // Average pooling prevents individual hot pixels from becoming strips.
  for(int y=0;y<h;++y)for(int x=0;x<w;++x){
    float sum=0,white=0,channels[3]={0,0,0};int n=0;
    for(int yy=y*height/h;yy<(y+1)*height/h;++yy)for(int xx=x*width/w;xx<(x+1)*width/w;++xx){
      const auto* p=rgb+((size_t)yy*width+xx)*3;
      const int lo=std::min(p[0],std::min(p[1],p[2])),hi=std::max(p[0],std::max(p[1],p[2]));
      sum+=lo;if(hi-lo<=hi*.35f)white+=lo;
      for(int c=0;c<3;++c)channels[c]+=p[c];++n;
    }
    light[y*w+x]=sum/n;neutral[y*w+x]=white/n;
    // In shaded low-light areas, chroma noise and a blue cast can fragment
    // white reflectors. Classify the pooled RGB sample, with a bounded colour
    // allowance only for dark pixels; normal passes keep strict per-pixel white.
    // Dawn captures have red/blue casts with min/max channel ratios down to 0.38.
    // Keep at least 35% in every pooled channel; saturated colours stay excluded.
    const float pooledLo=std::min(channels[0],std::min(channels[1],channels[2]))/n;
    const float pooledHi=std::max(channels[0],std::max(channels[1],channels[2]))/n;
    dawnNeutral[y*w+x]=(pooledHi<=128 && pooledHi-pooledLo<=pooledHi*.65f)
      ?std::max(neutral[y*w+x],pooledLo):neutral[y*w+x];
    if(dawnNeutral[y*w+x]>=12 && dawnNeutral[y*w+x]>neutral[y*w+x]*2)++dawnPixels;
    if(y>=roi)maximum=std::max(maximum,neutral[y*w+x]);
  }
  // Small absolute floors reject darkness without excluding dim reflectors;
  // relative thresholds, triple geometry and temporal acquisition still apply.
  if(maximum<12){clearUnlocked();return;}
  std::vector<Marker> markers;
  GradientData gradientData;
  // Future optimization if measured runtime again exceeds the freshness budget:
  // separate full-image SEARCH from TRACK of a freshly acquired triple. TRACK
  // could validate edges near the previous positions, widths and slopes before
  // doing full Hough voting. Require all three strips in the current frame;
  // retain contrast, geometry, ambiguity and capture-age checks. Missing strips,
  // excessive change or expiry must invalidate tracking and return to SEARCH,
  // preserving the existing navigation stop gate (no predicted/held detections).
  // Periodic full-image validation and fourth-strip/distractor regressions are
  // needed so an ROI cannot hide competing markers. This is not implemented;
  // the search-order selection below still evaluates the full image.
  // Preserve ordinary-pass ambiguity rejection (including clipped fourth strips),
  // then try local edges before the costly short-gap and dawn fallback scans.
  // Broad low-light chroma rejection predicts that the dawn pass will be useful.
  // Try it before Hough voting in that case. The ordinary pass stays first for
  // clipped fourth-strip ambiguity, and all passes still examine the full image.
  const bool preferDawn=dawnPixels>w*h/50;
  const int normalOrder[]={0,3,4,1,2},dawnOrder[]={0,2,3,4,1};
  for(int passIndex=0;passIndex<5;++passIndex){
  const int gapPass=(preferDawn?dawnOrder:normalOrder)[passIndex];
  // Both edge contrast models must agree on a unique triple. A shadowed
  // fourth strip found by the local model must invalidate a three-strip subset.
  if(!markers.empty() && gapPass!=3 && gapPass!=4)continue;
  const bool edgePass=gapPass==3 || gapPass==4;
  const auto& maskLight=gapPass>=2?dawnNeutral:neutral;
  for(float fraction : {.05f,.075f,.10f,.15f,.20f,.25f,.30f,.35f,.40f,.45f,.50f,.55f,.60f,.70f,.85f}){
    if(edgePass && fraction!=.25f)continue;
    if(gapPass!=2 && fraction<.25f)continue;
    std::vector<Strip> strips;
    if(edgePass)strips=gradientStrips(light,dawnNeutral,w,h,gapPass==4,gradientData);
    else {
    const float threshold=std::max(6.0f,maximum*fraction);
    // Search complete components, including their portion above the lower-image
    // anchor. A close marker may extend upwards while still standing below it.
    // Bridge only short vertical interruptions (e.g. a cable); never bridge the
    // horizontal dark gaps separating the three strips.
    std::vector<uint8_t> mask(w*h,0);
    for(int i=0;i<w*h;++i)mask[i]=maskLight[i]>=threshold;
    const int maxGap=gapPass!=1?std::max(1,int(h*.04f)):2;
    for(int x=0;x<w;++x){
      int previous=-1;
      for(int y=0;y<h;++y)if(maskLight[y*w+x]>=threshold){
        if(previous>=0 && y-previous-1<=maxGap)
          for(int yy=previous+1;yy<y;++yy)mask[yy*w+x]=1;
        previous=y;
      }
    }
    std::vector<uint8_t> seen(w*h,0);std::vector<int> queue;
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
    } // threshold component pass
    // Bound combinatorial work in highly textured scenes; abstain at that level.
    // The edge pool combines two bounded models; allow their union rather
    // than dropping an otherwise valid frame just above the single-model cap.
    if(strips.size()>(edgePass?128u:64u))continue;
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
        // Relaxed low-light colour must not join a reflector to bright scenery
        // at an image edge. In that fallback, accept side clipping only when
        // all three measured strips span both vertical image boundaries.
        if(gapPass==2 && (a[0]->top>0 || a[1]->top>0 || a[2]->top>0 ||
           a[0]->bottom<h-1 || a[1]->bottom<h-1 || a[2]->bottom<h-1))continue;
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
          // Unequal clipping does not imply unequal physical strip widths.
          // Keep one substantial outer segment and at least an 18%-width
          // of the other; two tiny edge remnants still cannot form a marker.
          if(std::min(a[0]->width,a[2]->width)<a[1]->width*.18f ||
             std::max(a[0]->width,a[2]->width)<a[1]->width*.5f)continue;
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
      const int clippedEnds=int(a[0]->cropped)+int(a[1]->cropped)+int(a[2]->cropped);
      // A single paired edge may continue into background above a real strip.
      // Keep the long-span rule when multiple strips actually leave the frame.
      // Sunlight can erase unequal upper sections while the three physical
      // lower ends still line up. Accept only a long shared measured segment.
      const float endSpread=std::max(a[0]->bottom,std::max(a[1]->bottom,a[2]->bottom))-bottom;
      const bool alignedLowerEdges=edgePass && endSpread<=minW*.25f &&
        bottom-top>=minW*3 && minH>=h*.45f && bottom>=h*.85f;
      if(clippedEnds && minH<h*.70f && (!edgePass || clippedEnds!=1) && !alignedLowerEdges)continue;
      // In the extreme near field, perspective/lens distortion can make the
      // centre wider than both outer strips. Compare it to their mean, while
      // still requiring the outer pair to agree. This rule applies equally
      // just before and just after an outer strip leaves the image border.
      const float outerMean=(effectiveW[0]+effectiveW[2])*.5f;
      const bool nearWidths=minH>=h*.85f &&
        similar(effectiveW[0],effectiveW[2],DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) &&
        similar(effectiveW[1],outerMean,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO);
      // Local illumination allows more background candidates. Require tighter
      // agreement of the three measured sizes in this additional model.
      if(edgePass && !similar(minW,maxW,1.3f))continue;
      if(gapPass==4 && !similar(minH,maxH,1.3f))continue;
      if(!similar(minH,maxH,alignedLowerEdges?1.8f:DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) ||
         (!similar(minW,maxW,DOCK_REFLECTOR_STRIPS_MAX_SIZE_RATIO) && !nearWidths) || bottom-top<(edgePass?std::max(minH*.80f,(alignedLowerEdges?maxH*.55f:maxH*.70f-3.f)):maxH*.70f))continue;
      // At full-height, side-clipped contact range the wide-angle image
      // bends outer strips away from the centre. Keep the stricter parallel
      // check everywhere else; all individual slope/contrast checks remain.
      const float slopeDifference=bothSidesCut?.20f:.12f;
      if(std::fabs(a[0]->slope-a[1]->slope)>slopeDifference ||
         std::fabs(a[2]->slope-a[1]->slope)>slopeDifference)continue;
      const float y=(top+bottom)*.5f;
      Marker m;float shift[3]={0,0,0};
      if(a[0]->leftCut)shift[0]=-(effectiveW[0]-a[0]->width)*.5f;
      if(a[2]->rightCut)shift[2]=(effectiveW[2]-a[2]->width)*.5f;
      for(int n=0;n<3;++n)m.x[n]=a[n]->x+a[n]->slope*(y-a[n]->y)+shift[n];
      const float d1=m.x[1]-m.x[0],d2=m.x[2]-m.x[1];
      // Paired edges also occur on distant fence and frame parts. Require a
      // compact, regularly spaced triple rather than joining those objects.
      const float maxGapWidths=edgePass?6.f:10.f;
      const float spacingRatio=edgePass?std::min(1.3f,float(DOCK_REFLECTOR_STRIPS_MAX_SPACING_RATIO)):float(DOCK_REFLECTOR_STRIPS_MAX_SPACING_RATIO);
      if(d1<maxW*1.5f || d2<maxW*1.5f || std::max(d1,d2)>minW*maxGapWidths || !similar(d1,d2,spacingRatio))continue;
      // Require dark gaps over most of the common height, not three highlights
      // on one white object. This also excludes four aligned stripes: they
      // produce multiple different plausible triplets and remain ambiguous.
      int dark=0,darkLocal=0,total=0;
      for(int yy=int(top);yy<=int(bottom);++yy){
        // Contrast must sample visible strip cores, not reconstructed centres
        // which may legitimately lie outside the frame after side clipping.
        float xs[3];for(int n=0;n<3;++n)xs[n]=a[n]->x+a[n]->slope*(yy-a[n]->y);
        ++total;
        if(xs[0]<0 || xs[1]<0 || xs[2]<0 || xs[0]>=w || xs[1]>=w || xs[2]>=w)continue;
        const float core=std::min(light[yy*w+int(xs[0])],std::min(light[yy*w+int(xs[1])],light[yy*w+int(xs[2])]));
        bool gapsDark=true,localGapsDark=true;
        for(int gap=0;gap<2;++gap){
          if(edgePass){
            // A sun/shadow boundary may cross the gap itself. Compare each
            // facing edge to its own neighbouring background, never a shaded
            // strip to unrelated sunlit pixels halfway across the gap.
            const int leftEdge=int(xs[gap]+a[gap]->width*.5f);
            const int rightEdge=int(xs[gap+1]-a[gap+1]->width*.5f);
            const int offset=std::max(1,std::min(5,int(minW/3)));
            if(leftEdge-offset<0 || rightEdge+offset>=w || leftEdge+5>=w || rightEdge-5<0){localGapsDark=false;continue;}
            const float lc=light[yy*w+leftEdge-offset],rc=light[yy*w+rightEdge+offset];
            if(lc-light[yy*w+leftEdge+5]<std::max(4.f,lc*.03f) ||
               rc-light[yy*w+rightEdge-5]<std::max(4.f,rc*.03f))localGapsDark=false;
          }
          float reference=core;
          if(gapPass==4){
            // Compare a gap to the facing inner edges, not to shaded centres
            // on the other side of a strip. Each boundary is locally measured.
            const int l=std::max(0,std::min(w-1,int(xs[gap]+a[gap]->width*.35f)));
            const int r=std::max(0,std::min(w-1,int(xs[gap+1]-a[gap+1]->width*.35f)));
            reference=std::max(core,std::min(light[yy*w+l],light[yy*w+r]));
          }
          const float limit=edgePass?reference-std::max(4.f,reference*.03f):core*.8f;
          if(light[yy*w+int((xs[gap]+xs[gap+1])*.5f)]>=limit)gapsDark=false;
        }
        if(gapsDark)++dark;
        if(edgePass && localGapsDark)++darkLocal;
      }
      if(dark<total*.8f && darkLocal<total*.8f)continue;
      m.width=minW;m.span=minH/h;m.quality=(minH/maxH+minW/maxW+std::min(d1,d2)/std::max(d1,d2))/3;
      // Unequal visible ends provide weaker evidence than a complete triple.
      // Do not let an overlap-tolerant background hypothesis displace it.
      if(edgePass && bottom-top<maxH*.70f)m.quality-=.05f;
      bool duplicate=false;
      for(auto& old:markers){
        bool same=true;for(int n=0;n<3;++n)if(std::fabs(old.x[n]-m.x[n])>std::max(2.0f,std::min(old.width,m.width)*.5f))same=false;
        if(same){if(m.quality>old.quality)old=m;duplicate=true;break;}
      }
      if(!duplicate)markers.push_back(m);
    }
  }
  }
  const unsigned long previous=timestampMs_.load(std::memory_order_relaxed);
  if(markers.size()>1 && found_.load(std::memory_order_relaxed) &&
     timestampMs-previous<=DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS){
    int match=-1,matches=0;
    for(size_t i=0;i<markers.size();++i){
      const auto& m=markers[i];
      bool same=similar(m.width/w,previousStripWidth_,1.3f) &&
                similar(m.span,previousStripSpan_,1.3f);
      for(int n=0;n<3;++n)if(std::fabs(m.x[n]/w-previousStripX_[n])>
          std::max(2.f/w,previousStripWidth_*.5f))same=false;
      if(same){match=int(i);++matches;}
    }
    // Associate only one fully observed, fresh three-strip candidate. Never
    // predict an absent strip or retain a stale observation through a loss.
    if(matches==1){const Marker selected=markers[match];markers.assign(1,selected);}
  }
  if(markers.size()>1){
    std::sort(markers.begin(),markers.end(),[](const Marker& a,const Marker& b){return a.quality>b.quality;});
    // A background edge can borrow two genuine strips. Resolve only a strong
    // geometric winner over substantially worse overlapping hypotheses. Two
    // independent markers, or four equally plausible strips, remain ambiguous.
    bool dominant=markers[0].quality>=.90f && markers[0].quality-markers[1].quality>=.10f;
    for(size_t i=1;i<markers.size() && dominant;++i){
      int shared=0;
      for(float x:markers[0].x){
        bool matches=false;
        for(float other:markers[i].x)if(std::fabs(x-other)<=std::min(markers[0].width,markers[i].width)*.5f)matches=true;
        if(matches)++shared;
      }
      if(shared<2)dominant=false;
    }
    if(dominant)markers.resize(1);
  }
  if(markers.size()!=1){clearUnlocked();return;}
  const Marker& marker=markers.front();
  const float error=(marker.x[1]-w*.5f)/(w*.5f);
  if(acquireCount_ && (timestampMs-previous>DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS || std::fabs(error-horizontalError_.load())>.08f))acquireCount_=0;
  if(acquireCount_<DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES)++acquireCount_;
  for(int n=0;n<3;++n)previousStripX_[n]=marker.x[n]/w;
  previousStripWidth_=marker.width/w;previousStripSpan_=marker.span;
  horizontalError_.store(error);confidence_.store(marker.quality);stripCount_.store(3);verticalSpan_.store(marker.span);horizontalSpan_.store((marker.x[2]-marker.x[0])/w);timestampMs_.store(timestampMs);
  found_.store(acquireCount_>=DOCK_REFLECTOR_STRIPS_ACQUIRE_FRAMES,std::memory_order_release);
}
ReflectorStripsObservation ReflectorStripsDetector::observation(unsigned long nowMs)const{
  ReflectorStripsObservation r;
  const unsigned long timestamp=timestampMs_.load();
  r.ageMs=timestamp?nowMs-timestamp:(unsigned long)-1;
  r.horizontalError=horizontalError_.load();r.confidence=confidence_.load();r.stripCount=stripCount_.load();r.verticalSpan=verticalSpan_.load();r.horizontalSpan=horizontalSpan_.load();
  r.found=found_.load(std::memory_order_acquire)&&timestamp&&r.ageMs<=DOCK_REFLECTOR_STRIPS_LOST_TIMEOUT_MS;
  return r;
}
#endif
