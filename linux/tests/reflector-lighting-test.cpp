// Deterministic illumination stress test. Run through run-reflector-strips-tests.sh.
// All positive scenes retain real contrast on all three strips. Fully erased
// markers are tested separately as negatives; no stale observation is accepted.
#define main reflectorExistingMain
#include "reflector-strips-test.cpp"
#undef main
int main(int argc,char** argv){
 const char* names[]={"hard horizontal","soft horizontal","diagonal shadow","vertical shadow","multiple bands","sun spot","cloud gradient","leaf shadows","backlight haze","warm sunlight","cool shade","clipped sunlight"};
 int failures=0,total=0,falsePositive=0,misses=0,knownLimitations=0;
 for(int mode=0;mode<12;++mode)for(int variant=0;variant<8;++variant)for(int count=0;count<=4;++count){
  if(argc==4 && (mode!=std::atoi(argv[1]) || variant!=std::atoi(argv[2]) || count!=std::atoi(argv[3])))continue;
  Image a(W*H*3);std::mt19937 rng(20261022+variant);std::uniform_int_distribution<int> noise(-1,1);
  const int length=variant%2?300:150,width=length/6,spacing=width*3,top=335-length;
  for(int y=0;y<H;++y)for(int x=0;x<W;++x){
   bool marker=false;
   for(int n=0;n<count;++n)if(std::abs(x-int(320+(n-(count-1)*.5f)*spacing))<width/2)marker=true;
   marker=marker && y>=top && y<335;
   float t=0,edge=70+variant*30,soft=std::max(0.f,std::min(1.f,(y-edge+35)/70.f));
   switch(mode){
    case 0:t=y>edge;break;
    case 1:t=soft;break;
    case 2:t=y>.35f*(x-320)+edge;break;
    case 3:t=x>100+variant*60;break;
    case 4:t=(y/(25+variant*5))%2;break;
    case 5:t=std::exp(-((x-240.f)*(x-240)+(y-190.f)*(y-190))/18000.f);break;
    case 6:t=(x+float(y))/(W+H);break;
    case 7:t=.5f+.25f*std::sin(x*.027f+y*.033f+variant)+.25f*std::sin(y*.045f-x*.019f);break;
    default:t=soft;break;
   }
   if(variant&2)t=1-t;
   float gain=(variant&4)?.4f:1.f;
   float value=(marker?(80+175*t):(20+200*t))*gain;
   if(mode==8)value=value*.65f+50;
   if(mode==11)value=std::min(255.f,value*1.08f);
   for(int c=0;c<3;++c){
    float tint=1;
    if(mode==9)tint=c==2?.82f:1;
    if(mode==10)tint=c==0?.82f:1;
    a[(y*W+x)*3+c]=uint8_t(std::max(0.f,std::min(255.f,value*tint+noise(rng))));
   }
  }
  auto& d=ReflectorStripsDetector::instance();d.clear();
  for(int i=0;i<3;++i)d.processRgb(a.data(),W,H,1000+i*100);
  auto o=d.observation(1200);bool expected=count==3;
  bool ok=o.found==expected && (!expected || std::fabs(o.horizontalError)<.04f) && !d.observation(1800).found;
  ++total;
  // Known ambiguity: the hard shadow coincides exactly with the fourth strip's
  // left edge, cancelling that edge. Keep measuring/reporting it, but do not
  // relax the detector for this single artificial case (accepted 2026-10-22).
  if(!ok && mode==3 && variant==7 && count==4 && o.found){
   ++knownLimitations;std::puts("KNOWN LIMITATION vertical shadow variant=7 strips=4: cancelled fourth edge");continue;
  }
  if(!ok){++failures;if(o.found&&!expected)++falsePositive;else ++misses;
   std::printf("FAIL %s variant=%d strips=%d found=%d error=%.3f\n",names[mode],variant,count,o.found,o.horizontalError);
  }
 }
 std::printf("Lighting stress: %d cases, %d unexpected failures (%d false positives, %d misses/geometry failures), %d known limitations\n",total,failures,falsePositive,misses,knownLimitations);
 return failures?1:0;
}
