#include "src/camera/ReflectorStripsDetector.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <random>
using Image=std::vector<uint8_t>;
const int W=640,H=360;
static void require(bool ok,const char* name){if(!ok){std::fprintf(stderr,"FAIL %s\n",name);std::exit(1);}}
static void strip(Image& a,int x,int top,int width,int height,float gain=1,float slope=0){
 for(int y=top;y<top+height;++y)for(int xx=int(x+slope*(y-top))-width/2;xx<int(x+slope*(y-top))+(width+1)/2;++xx)
  if(y>=0&&y<H&&xx>=0&&xx<W)for(int c=0;c<3;++c)a[(y*W+xx)*3+c]=std::min(255,int(210*gain));
}
static Image scene(int count=3,float gain=1,int centre=320,float slope=0){
 Image a(W*H*3,uint8_t(20*gain));
 for(int i=0;i<count;++i)strip(a,centre+(i-1)*30,210,8,80,gain,slope);
 return a;
}
static void check(const char* name,const Image&a,bool expected,float error=0){
 auto& d=ReflectorStripsDetector::instance();d.clear();
 for(int i=0;i<3;++i){d.processRgb(a.data(),W,H,1000+i*100);require(d.observation(1000+i*100).found==(expected&&i==2),name);}
 auto o=d.observation(1200);if(expected){require(o.stripCount==3,name);require(std::fabs(o.horizontalError-error)<.04,name);}
 require(!d.observation(1800).found,"expiry");d.clear();require(!d.observation(1200).found,"clear");std::printf("PASS %s\n",name);
}
int main(int argc,char**argv){
 if(argc==4){int w=std::atoi(argv[2]),h=std::atoi(argv[3]);require(w>0&&h>0,"dimensions");Image a(w*h*3);std::ifstream f(argv[1],std::ios::binary);f.read((char*)a.data(),a.size());require(f.gcount()==(long)a.size(),"input");auto&d=ReflectorStripsDetector::instance();for(int i=0;i<3;++i)d.processRgb(a.data(),w,h,1000+i*100);auto o=d.observation(1200);std::printf("found=%d strips=%d error=%.4f confidence=%.3f span=%.3f\n",o.found,o.stripCount,o.horizontalError,o.confidence,o.verticalSpan);return 0;}
 for(float gain:{.1f,.2f,.4f,.7f,1.f,1.3f})check("exposure",scene(3,gain),true);
 for(float gain:{.08f,.1f,.2f}){
  Image dim=scene(3,gain);std::mt19937 rng(42);std::uniform_int_distribution<int> noise(-2,2);
  for(auto& value:dim)value=uint8_t(std::max(0,std::min(255,int(value)+noise(rng))));
  check("dim reflectors with sensor noise",dim,true);
 }
 for(int seed=0;seed<12;++seed){
  Image noiseImage(W*H*3);std::mt19937 rng(seed);std::uniform_int_distribution<int> noise(0,30);
  for(auto& value:noiseImage)value=uint8_t(noise(rng));
  check("dark sensor noise rejected",noiseImage,false);
 }
 check("below usable light rejected",scene(3,.03f),false);
 // Dawn: a bright patch outside the marker must not set its minimum brightness.
 for(int count:{1,2,3,4}){
  Image dawn=scene(count,.15f);
  for(int y=150;y<170;++y)for(int x=40;x<65;++x)for(int c=0;c<3;++c)dawn[(y*W+x)*3+c]=240;
  check("dawn shaded strips beside bright scenery",dawn,count==3);
 }
 // The low-threshold fallback must still reject coloured objects and noise.
 Image coloured=scene(3,.15f);
 for(int y=150;y<170;++y)for(int x=40;x<65;++x)for(int c=0;c<3;++c)coloured[(y*W+x)*3+c]=240;
 for(int y=210;y<290;++y)for(int x=316;x<324;++x)coloured[(y*W+x)*3]=180;
 check("dawn coloured centre is not a reflector",coloured,false);
 // Blue cast at dawn can fail the per-pixel whiteness gate even when the
 // three-strip geometry is intact. Keep strong colour and missing strips out.
 for(int count:{2,3,4}){
  Image cast=scene(count,.25f);
  for(size_t i=0;i<cast.size();i+=3)if(cast[i]>30){cast[i]=52;cast[i+1]=65;cast[i+2]=95;}
  for(int y=150;y<170;++y)for(int x=40;x<65;++x)for(int c=0;c<3;++c)cast[(y*W+x)*3+c]=240;
  check("dawn blue cast triple geometry",cast,count==3);
 }
 // Mixed red/blue dawn casts and chroma noise must not erase a real strip.
 // The same allowance must still reject missing/fourth strips and saturation.
 for(int count:{2,3,4})for(int seed=0;seed<3;++seed)for(int green:{32,38}){
  Image cast=scene(count,.25f);std::mt19937 rng(seed);
  std::uniform_int_distribution<int> noise(-3,3);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   auto* p=&cast[(y*W+x)*3];if(p[0]<=30)continue;
   p[0]=(x<320?80:55)+noise(rng);p[1]=green+noise(rng);p[2]=(x<320?60:80)+noise(rng);
  }
  for(int y=150;y<170;++y)for(int x=40;x<65;++x)for(int c=0;c<3;++c)cast[(y*W+x)*3+c]=240;
  check("mixed dawn casts with chroma noise",cast,count==3);
 }
 for(bool bright:{false,true}){
  Image cast=scene(3,.25f);
  for(size_t i=0;i<cast.size();i+=3)if(cast[i]>30){cast[i]=bright?200:80;cast[i+1]=bright?90:20;cast[i+2]=bright?130:55;}
  check("saturated or bright coloured strips rejected",cast,false);
 }

 // Sunlit backing joins the tops; the shadowed lower segments remain separate.
 for(int count:{2,3,4}){
  Image shadow(W*H*3,20);
  for(int y=0;y<190;y++)for(int x=0;x<W;x++)for(int c=0;c<3;c++)shadow[(y*W+x)*3+c]=220;
  for(int i=0;i<count;i++)strip(shadow,int(320+(i-(count-1)*.5f)*170),20,60,335,1.3f);
  for(int y=190;y<H;y++)for(int x=0;x<W;x++)for(int c=0;c<3;c++)if(shadow[(y*W+x)*3+c]>100)shadow[(y*W+x)*3+c]=80;
  check("sunlit backing and shadowed marker",shadow,count==3);
 }
 // Edge fallback must not depend on which half is bright or where a shadow falls.
 for(int boundary:{90,180,270})for(bool brightTop:{false,true})for(float gain:{.3f,.6f,1.f}){
  Image varied(W*H*3);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   bool bright=(y<boundary)==brightTop;
   bool marker=y>=20 && y<355 && (std::abs(x-150)<30 || std::abs(x-320)<30 || std::abs(x-490)<30);
   uint8_t value=uint8_t((marker?(bright?255:80):(bright?220:20))*gain);
   for(int c=0;c<3;c++)varied[(y*W+x)*3+c]=value;
  }
  check("gradient marker independent of shadow position and exposure",varied,true);
 }
 // A shadow may split a reflector vertically: its centre can be darker than
 // the sunny gap beside it while both physical outer edges remain measurable.
 for(int count:{2,3,4})for(float gain:{.3f,1.f})for(bool mirror:{false,true}){
  Image split(W*H*3);
  const int first=int(320-(count-1)*.5f*125);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   const int xx=mirror?W-1-x:x;
   bool marker=false;
   for(int n=0;n<count;++n)if(std::abs(xx-(first+n*125))<30)marker=true;
   marker=marker && y>=25 && y<345;
   const bool shade=y<140 || xx<first+5;
   const uint8_t value=uint8_t((marker?(shade?80:255):(shade?20:220))*gain);
   for(int c=0;c<3;++c)split[(y*W+x)*3+c]=value;
  }
  char label[120];
  std::snprintf(label,sizeof(label),"shadow through strip centre count=%d gain=%.1f mirror=%d",count,gain,mirror);
  check(label,split,count==3);
 }
 // A shorter neighbouring object must not make a complete triple ambiguous.
 for(float gain:{.3f,1.f})for(bool mirror:{false,true}){
  Image backing(W*H*3,uint8_t(230*gain));
  for(int y=140;y<320;++y)for(int x=0;x<W;++x){
   const int xx=mirror?W-1-x:x;
   bool marker=std::abs(xx-200)<15 || std::abs(xx-320)<15 || std::abs(xx-440)<15;
   bool shorterBackground=y>=200 && std::abs(xx-80)<15;
   if(marker || shorterBackground)for(int c=0;c<3;++c)backing[(y*W+x)*3+c]=uint8_t(255*gain);
  }
  check("short background edge cannot form competing triple",backing,true);
 }
 // On undocking the complete marker shrinks below 70% of image height.
 // Keep three-strip discrimination through that transition, also in dim light.
 for(int length:{185,215,250,280})for(int count:{2,3,4})for(float gain:{.3f,1.f}){
  Image receding(W*H*3);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   bool marker=false;
   for(int n=0;n<count;++n)if(std::abs(x-int(320+(n-(count-1)*.5f)*125))<20)marker=true;
   marker=marker && y>=50 && y<50+length;
   const bool bright=y<170;
   const uint8_t value=uint8_t((marker?(bright?255:80):(bright?220:20))*gain);
   for(int c=0;c<3;++c)receding[(y*W+x)*3+c]=value;
  }
  check("receding gradient marker below seventy percent height",receding,count==3);
 }
 // Scale the same low-contrast geometry across the camera approach/exit.
 // Both light levels defeat the ordinary 15% contrast gate, forcing edges.
 for(int length:{60,90,120,150,175,220,300})for(int count:{2,3,4})for(float gain:{.3f,1.f}){
  const int width=length/6,spacing=width*3,top=320-length;
  Image scaled(W*H*3);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   bool marker=false;
   for(int n=0;n<count;++n)if(std::abs(x-int(320+(n-(count-1)*.5f)*spacing))<width/2)marker=true;
   marker=marker && y>=top && y<320;
   const bool bright=y<top+length/2;
   const uint8_t value=uint8_t((marker?(bright?255:90):(bright?220:75))*gain);
   for(int c=0;c<3;c++)scaled[(y*W+x)*3+c]=value;
  }
  check("scaled low-contrast gradient triple",scaled,count==3);
 }
 // Numerous long non-neutral background edges must not evict the shorter
 // true marker from a global top-N Hough list.
 for(int count:{2,3,4}){
  Image clutter(W*H*3);
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   bool marker=false;
   for(int n=0;n<count;++n)if(std::abs(x-int(320+(n-(count-1)*.5f)*60))<10)marker=true;
   marker=marker && y>=180 && y<300;
   uint8_t value=marker?(y<240?255:90):(y<240?220:75);
   for(int c=0;c<3;++c)clutter[(y*W+x)*3+c]=value;
   if(x<200 || x>440){
    clutter[(y*W+x)*3]=(x/3)%2?70:30;
    clutter[(y*W+x)*3+1]=clutter[(y*W+x)*3+2]=200;
   }
  }
  check("small gradient marker among long background edges",clutter,count==3);
 }
 check("left",scene(3,1,180),true,(180-320.f)/320);
 check("right",scene(3,1,460),true,(460-320.f)/320);
 check("camera roll",scene(3,1,320,.15),true,.02);
 check("one strip",scene(1),false);check("two strips",scene(2),false);check("four strips ambiguous",scene(4),false);
 check("blank",Image(W*H*3,0),false);check("white",Image(W*H*3,255),false);
 Image a(W*H*3,20);strip(a,280,210,8,80);strip(a,300,210,8,80);strip(a,360,210,8,80);check("uneven spacing",a,false);
 a.assign(W*H*3,20);strip(a,290,210,8,80);strip(a,320,210,8,30);strip(a,350,210,8,80);check("unequal length",a,false);
 a.assign(W*H*3,20);strip(a,290,210,8,80);strip(a,320,210,20,80);strip(a,350,210,8,80);check("unequal width",a,false);
 a.assign(W*H*3,20);for(int x:{290,320,350})strip(a,x,20,8,80);check("upper only",a,false);
 a.assign(W*H*3,20);for(int y:{210,240,270})strip(a,320,y,80,8);check("horizontal stripes",a,false);
 a.assign(W*H*3,20);for(int x:{290,320,350})strip(a,x,300,8,80);check("cropped marker",a,false);
 check("diagonal",scene(3,1,320,.5),false);
 a=scene(3,1,150);for(int x:{460,490,520})strip(a,x,210,8,80);check("two markers ambiguous",a,false);
 // A distant, dim triple has only a narrow usable threshold interval:
 // low levels attach the right strip to background, high levels lose the left.
 a.assign(W*H*3,20);
 for(int x:{290,320,350})strip(a,x,190,6,28,x==290?132.f/210:145.f/210);
 for(int y=160;y<175;++y)for(int x=60;x<160;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=255;
 for(int y=200;y<204;++y)for(int x=353;x<405;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=110;
 for(int y=180;y<230;++y)for(int x=400;x<430;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=110;
 check("distant narrow threshold interval",a,true);
 // At 640x360 a real distant marker has thresholded heights 22/18/23.
 // The former 5.5% height floor intermittently discarded the centre strip.
 a.assign(W*H*3,20);
 strip(a,430,193,3,22);strip(a,441,195,3,18);strip(a,452,193,3,23);
 check("distant unequal thresholded ends",a,true,(441-320.f)/320);
 a.assign(W*H*3,20);
 for(int x:{430,441,452})strip(a,x,193,3,14);
 check("too short distant highlights",a,false);
 // A large vertical closing radius joins a far strip to background below;
 // the short-gap fallback must recover the actual narrow triple.
 a.assign(W*H*3,20);
 for(int x:{430,441,452})strip(a,x,200,3,18);
 strip(a,452,225,25,15);
 check("distant strip above bright scenery",a,true,(441-320.f)/320);
 // At charging contacts the strips exceed both the old 75% height and
 // 8% width limits; scale alone must not lose an otherwise valid triple.
 for(int count:{2,3,4}){
  a.assign(W*H*3,20);
  for(int i=0;i<count;++i)strip(a,int(320+(i-(count-1)*.5f)*140),35,60,300);
  check(count==3?"contact full-height triple":(count==2?"contact two strips":"contact four strips"),a,count==3);
 }
 a.assign(W*H*3,20);for(int x:{180,320,460})strip(a,x,5,60,350);
 check("almost full-height triple",a,true);
 a.assign(W*H*3,20);for(int x:{180,320,460})strip(a,x,35,60,300);
 for(int x=429;x<=490;++x)for(int y=100+(x-429)/2;y<110+(x-429)/2;++y)
  for(int c=0;c<3;++c)a[(y*W+x)*3+c]=20;
 check("contact triple with cable",a,true);
 // Near markers cross the former ROI boundary; all three full ends remain visible.
 a.assign(W*H*3,20);for(int x:{245,320,395})strip(a,x,95,24,200);
 check("near marker crosses ROI",a,true);
 // A diagonal dark cable interrupts the right reflector by at most 10 rows.
 for(int x=380;x<=410;++x)for(int y=195+(x-380)/3;y<205+(x-380)/3;++y)
  for(int c=0;c<3;++c)a[(y*W+x)*3+c]=20;
 check("near marker with cable",a,true);
 // All three may have a short interruption, but must remain three distinct strips.
 a.assign(W*H*3,20);for(int x:{245,320,395})strip(a,x,95,24,200);
 for(int y=190;y<200;++y)for(int x=220;x<420;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=20;
 check("three strips with small occlusion",a,true);
 // A large missing part must not be invented by closing.
 a.assign(W*H*3,20);for(int x:{245,320,395})strip(a,x,95,24,200);
 for(int y=145;y<245;++y)for(int x=380;x<410;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=20;
 check("large occlusion rejected",a,false);
 a.assign(W*H*3,20);for(int x:{245,320})strip(a,x,95,24,200);
 check("near two strips rejected",a,false);
 a.assign(W*H*3,20);for(int x:{170,245,320,395})strip(a,x,95,24,200);
 check("near four strips ambiguous",a,false);
 a.assign(W*H*3,20);for(int x:{245,320,395})strip(a,x,-10,24,200);
 check("top cropped marker rejected",a,false);
 // At contact, only the ends may leave the frame; all three long segments
 // and the separating dark gaps must still be visible.
 for(int top:{-30,30,-60}){
  a.assign(W*H*3,20);for(int x:{180,320,460})strip(a,x,top,60,420);
  check("contact cropped ends",a,true);
 }
 for(int count:{2,4}){
  a.assign(W*H*3,20);
  for(int i=0;i<count;++i)strip(a,int(320+(i-(count-1)*.5f)*140),-30,60,420);
  check("cropped missing or ambiguous strips",a,false);
 }
 a.assign(W*H*3,20);for(int x:{180,320,460})strip(a,x,30,60,400);
 for(int y=100;y<112;++y)for(int x=425;x<495;++x)
  for(int c=0;c<3;++c)a[(y*W+x)*3+c]=20;
 check("cropped contact with cable",a,true);
 a.assign(W*H*3,20);strip(a,180,30,60,400);strip(a,320,130,60,200);strip(a,460,30,60,400);
 check("cropped triple requires three long segments",a,false);
 a.assign(W*H*3,20);for(int x:{20,160,300})strip(a,x,-30,60,420);
 check("one outer strip partly cropped at contact",a,true,-.5f);
 // Fully visible outer strips with a narrow dark border: contrast probes
 // must stay inside the image; full-height clipped sides have separate checks.
 a.assign(W*H*3,20);for(int x:{43,320,597})strip(a,x,-30,80,420);
 check("contact narrow outer margins",a,true);
 a.assign(W*H*3,20);for(int x:{43,597})strip(a,x,-30,80,420);
 check("narrow margins missing middle",a,false);
 a.assign(W*H*3,20);for(int x:{40,320,600})strip(a,x,-30,80,420);
 check("full-height triple touching both sides",a,true);
 a.assign(W*H*3,20);for(int x:{43,320,597})strip(a,x,-30,80,420);
 for(int y=0;y<H;++y)for(int x=0;x<5;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=210;
 for(int y=0;y<H;++y)for(int x=W-5;x<W;++x)for(int c=0;c<3;++c)a[(y*W+x)*3+c]=210;
 check("full-height triple without outer margins",a,true);
 a.assign(W*H*3,20);
 strip(a,60,-30,80,420);strip(a,330,-30,120,420);strip(a,626,-30,80,420);
 check("extreme close wide centre and cropped right",a,true,10.f/320);
 a.assign(W*H*3,20);
 strip(a,14,-30,80,420);strip(a,310,-30,120,420);strip(a,580,-30,80,420);
 check("extreme close cropped left",a,true,-10.f/320);
 a.assign(W*H*3,20);
 strip(a,60,-30,80,420);strip(a,330,-30,120,420);strip(a,650,-30,80,420);
 check("too little outer strip remains",a,false);
 a.assign(W*H*3,20);strip(a,60,-30,80,420);strip(a,330,-30,120,420);
 check("extreme close missing third strip",a,false);
 a.assign(W*H*3,20);for(int x:{10,190,370,550})strip(a,x,-30,80,420);
 check("four strips with clipped outer ambiguous",a,false);
 a.assign(W*H*3,20);strip(a,40,-30,90,420);strip(a,328,-30,110,420);strip(a,572,-30,70,420);
 check("one clipped outer wider by perspective",a,true,8.f/320);
 a.assign(W*H*3,20);strip(a,60,-30,140,420);strip(a,328,-30,110,420);strip(a,572,-30,70,420);
 check("one clipped outer excessively wide",a,false);
 // Both outer strips may be clipped after contact recovery, but cannot be
 // tiny remnants or unrelated side panels beside a shorter middle strip.
 for(float gain:{.1f,.2f,1.f}){
  a.assign(W*H*3,uint8_t(20*gain));
  strip(a,25,-30,100,420,gain);strip(a,332,-30,120,420,gain);strip(a,615,-30,100,420,gain);
  check("both sides cropped wide middle",a,true,12.f/320);
 }
 // Contact view with asymmetric clipping: one outer is only ~30% of
 // the middle width. Its reconstructed centre can be outside the image.
 for(float gain:{.1f,.2f,1.f})for(bool mirror:{false,true}){
  a.assign(W*H*3,uint8_t(20*gain));
  strip(a,mirror?640:0,-30,150,420,gain);
  strip(a,mirror?287:353,-30,136,420,gain);
  strip(a,mirror?0:640,-30,80,420,gain);
  check("asymmetric clipped contact",a,true,(mirror?-33.f:33.f)/320);
 }
 // Contact with a ~20%-width right remnant and diverging outer slopes.
 for(float gain:{.1f,.2f,1.f}){
  a.assign(W*H*3,uint8_t(20*gain));
  strip(a,42,-30,100,420,gain,.114f);
  strip(a,390,-30,130,420,gain,-.056f);
  strip(a,667,-30,100,420,gain,-.15f);
  check("contact thin outer and perspective slopes",a,true,59.f/320);
 }
 a.assign(W*H*3,20);
 strip(a,0,-30,150,420);strip(a,353,-30,136,420);strip(a,680,-30,100,420);
 check("contact outer tiny remnant rejected",a,false);
 a.assign(W*H*3,20);strip(a,0,-30,100,420);strip(a,332,-30,120,420);strip(a,640,-30,100,420);
 check("both sides too little visible",a,false);
 a.assign(W*H*3,20);strip(a,25,-30,100,420);strip(a,332,5,120,350);strip(a,615,-30,100,420);
 check("both sides require middle full height",a,false);
 a.assign(W*H*3,20);strip(a,25,-30,100,420);strip(a,332,-30,120,420);
 check("both side mode cannot invent third",a,false);
 a.assign(W*H*3,20);for(int x:{20,220,420,620})strip(a,x,-30,80,420);
 check("four full height strips with both sides cropped",a,false);
 // Bright scenery at the border must not borrow two genuine strips to
 // create ambiguity when their top/bottom ends are still visible.
 for(float gain:{.1f,.2f,1.f})for(bool mirror:{false,true}){
  a.assign(W*H*3,uint8_t(20*gain));
  for(int x:{170,320,470})strip(a,x,20,55,320,gain);
  strip(a,mirror?635:5,-20,70,400,gain);
  check("bright border beside complete marker",a,true);
 }
 // The outer strip must remain detectable as it transitions off the border.
 for(float gain:{.2f,.5f,1.f})for(int right:{590,600,610,620}){
  a.assign(W*H*3,uint8_t(20*gain));
  strip(a,55,-30,90,420,gain);strip(a,340,-30,116,420,gain);strip(a,right,-30,72,420,gain);
  check("near width perspective and border transition",a,true,20.f/320);
 }
 a.assign(W*H*3,20);strip(a,55,-30,90,420);strip(a,340,-30,140,420);strip(a,590,-30,50,420);
 check("near outer width mismatch rejected",a,false);
 a.assign(W*H*3,20);strip(a,90,-30,50,420);strip(a,340,-30,130,420);strip(a,590,-30,50,420);
 check("near excessive centre width rejected",a,false);
 auto&d=ReflectorStripsDetector::instance();d.clear();auto left=scene(3,1,180),right=scene(3,1,460);
 for(int i=0;i<6;++i){d.processRgb((i%2?left:right).data(),W,H,1000+i*100);require(!d.observation(1000+i*100).found,"candidate jump");}
 d.clear();a=scene();for(int i=0;i<3;++i)d.processRgb(a.data(),W,H,1000+i*100);
 a=scene(2);d.processRgb(a.data(),W,H,1300);require(!d.observation(1300).found,"missing strip stops immediately");
 d.clear();a=scene();d.processRgb(a.data(),W,H,1000);d.processRgb(a.data(),W,H,1100);d.processRgb(a.data(),W,H,2000);require(!d.observation(2000).found,"stale acquisition");
 // A newly visible fourth edge cannot hijack an already acquired marker.
 // Association still requires all three measured strips and expires normally.
 d.clear();a=scene();
 for(int i=0;i<3;++i)d.processRgb(a.data(),W,H,1000+i*100);
 auto four=scene(4);
 d.processRgb(four.data(),W,H,1300);
 require(d.observation(1300).found && std::fabs(d.observation(1300).horizontalError)<.01f,"fresh full-triple association");
 d.processRgb(four.data(),W,H,2000);
 require(!d.observation(2000).found,"ambiguous association cannot survive expiry");
 d.clear();a=scene();
 for(int i=0;i<3;++i)d.processRgb(a.data(),W,H,1000+i*100);
 a=scene(2);d.processRgb(a.data(),W,H,1300);
 require(!d.observation(1300).found,"association cannot invent a missing third strip");
 std::puts("PASS acquisition, loss, ambiguity and stale frames");
}
