#include "WifiHeatmap.h"
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef WIFI_HEATMAP_TEST
#include "../../sunray/config.h"
#endif
#include "../../sunray/WifiHeatmapConfig.h"

namespace WifiHeatmap {
namespace {
const char* schema="unix_s,x_m,y_m,rtk_e_m,rtk_n_m,latitude_deg,longitude_deg,absolute,origin_lat,origin_lon,wifi_status,rssi_dbm,frequency_mhz,ssid,bssid,rtk_age_ms,probe_ms";
std::string utc(int64_t t){time_t ts=t;tm value{};gmtime_r(&ts,&value);char b[32];strftime(b,sizeof(b),"%Y-%m-%dT%H:%M:%SZ",&value);return b;}
std::string csv(const std::string& s){std::string out="\"";for(unsigned char c:s){if(c=='"')out+='"';if(c>=32&&c!=127)out+=char(c);else{char b[5];snprintf(b,sizeof(b),"\\x%02x",c);out+=b;}}return out+'"';}
std::string json(const std::string& s){std::string out="\"";for(unsigned char c:s){if(c=='"'||c=='\\')out+='\\';if(c>=32)out+=char(c);else{char b[7];snprintf(b,sizeof(b),"\\u%04x",c);out+=b;}}return out+'"';}
bool writeAll(int fd,const std::string& s){size_t n=0;while(n<s.size()){ssize_t r=write(fd,s.data()+n,s.size()-n);if(r<0&&errno==EINTR)continue;if(r<=0)return false;n+=size_t(r);}return true;}
bool header(std::istream& in,int64_t& started){std::string first,columns;long long parsed=0;if(!std::getline(in,first)||!std::getline(in,columns)||sscanf(first.c_str(),"# sunray_wifi_heatmap_v1,start_unix=%lld",&parsed)!=1||parsed<1577836800||columns!=schema)return false;started=parsed;return true;}
bool valid(const Position& p){return p.fixed&&std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.east)&&std::isfinite(p.north)&&std::isfinite(p.originLat)&&std::isfinite(p.originLon)&&(!p.hasLatLon||(std::isfinite(p.lat)&&std::isfinite(p.lon)&&std::abs(p.lat)<=90&&std::abs(p.lon)<=180));}
bool fresh(const Position& p,Clock::time_point now){return valid(p)&&now>=p.received&&now-p.received<=std::chrono::milliseconds(1500);}
std::vector<std::string> fields(const std::string& line){std::vector<std::string> out;std::string part;bool quoted=false;for(size_t i=0;i<line.size();i++){char c=line[i];if(c=='"'){if(quoted&&i+1<line.size()&&line[i+1]=='"'){part+='"';i++;}else quoted=!quoted;}else if(c==','&&!quoted){out.push_back(part);part.clear();}else part+=c;}out.push_back(part);if(quoted)out.clear();return out;}
}
Reading parseLink(const std::string& text,bool success){
 Reading r;if(!success||text.size()>8192)return r;
 std::istringstream in(text);std::string line;bool connected=false,disconnected=false;
 while(std::getline(in,line)){
  const auto first=line.find_first_not_of(" \t\r");if(first==std::string::npos)continue;line.erase(0,first);
  if(line=="Not connected.")disconnected=true;
  if(line.compare(0,13,"Connected to ")==0){std::istringstream s(line.substr(13));s>>r.bssid;connected=r.bssid.size()==17;}
  if(line.compare(0,6,"SSID: ")==0)r.ssid=line.substr(6,256);
  if(line.compare(0,6,"freq: ")==0){int value=0;if(sscanf(line.c_str()+6,"%d",&value)==1&&value>=2000&&value<=72000)r.frequency=value;}
  if(line.compare(0,8,"signal: ")==0){double value=0;char unit[8]={};if(sscanf(line.c_str()+8,"%lf %7s",&value,unit)==2&&strcmp(unit,"dBm")==0&&std::isfinite(value)&&value<0&&value>=-150){r.rssi=int(std::lround(value));r.hasRssi=true;}}
 }
 if(connected&&!disconnected)r.state=Reading::Connected;
 else if(disconnected&&!connected){r=Reading();r.state=Reading::Disconnected;}
 return r;
}
Reading queryLink(const std::string& interface){
 if(interface.empty()||interface.size()>15||interface.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-")!=std::string::npos)return {};
 const auto cmd="LC_ALL=C timeout -k 1s 1s iw dev "+interface+" link 2>/dev/null";
 FILE* f=popen(cmd.c_str(),"r");if(!f)return {};
 std::string text;char b[512];while(fgets(b,sizeof(b),f)){if(text.size()<8192)text+=b;}
 return parseLink(text,pclose(f)==0);
}
Store::~Store(){if(fd>=0)close(fd);if(lockFd>=0)close(lockFd);}
bool Store::create(int64_t now){
 std::string temp=config.file+".tmp.XXXXXX";std::vector<char> name(temp.begin(),temp.end());name.push_back(0);
 int next=mkstemp(name.data());if(next<0)return false;
 const std::string text="# sunray_wifi_heatmap_v1,start_unix="+std::to_string(now)+",start_utc="+utc(now)+"\n"+schema+"\n";
 bool ok=flock(next,LOCK_EX|LOCK_NB)==0&&writeAll(next,text)&&fsync(next)==0;
 if(ok)ok=rename(name.data(),config.file.c_str())==0;
 if(!ok){close(next);unlink(name.data());return false;}
 if(fd>=0)close(fd);fd=next;started=now;haveLast=false;return true;
}
bool Store::maintain(int64_t now){
 if(now<1577836800||!std::isfinite(config.distance)||config.distance<=0||!config.days)return false;
 if(lockFd<0){lockFd=open((config.file+".lock").c_str(),O_RDWR|O_CREAT|O_NOFOLLOW,0600);if(lockFd<0)return false;if(flock(lockFd,LOCK_EX|LOCK_NB)!=0){close(lockFd);lockFd=-1;return false;}}
 if(fd<0){
  fd=open(config.file.c_str(),O_RDWR|O_APPEND|O_NOFOLLOW);
  if(fd<0){if(errno==ENOENT)return create(now);return false;}
  struct stat st{};
  if(fstat(fd,&st)||!S_ISREG(st.st_mode)||flock(fd,LOCK_EX|LOCK_NB)){close(fd);fd=-1;return false;}
  std::ifstream in(config.file);if(!header(in,started)){close(fd);fd=-1;return false;}
  std::streamoff complete=in.tellg();std::string line;
  while(std::getline(in,line)){
   if(in.eof())break; // incomplete tail from an interrupted write
   complete=in.tellg();auto f=fields(line);
   if(f.size()!=17){close(fd);fd=-1;return false;}
   try{last.x=std::stod(f[1]);last.y=std::stod(f[2]);last.east=std::stod(f[3]);last.north=std::stod(f[4]);last.absolute=f[7]=="1";last.originLat=std::stod(f[8]);last.originLon=std::stod(f[9]);last.fixed=true;haveLast=valid(last);}catch(...){close(fd);fd=-1;return false;}
  }
  if(complete<st.st_size && ftruncate(fd,complete)!=0)return false;
 }
 if(now<started)return false; // do not silently mix data after wall-clock rollback
 if(now-started>=int64_t(config.days)*86400)return create(now);
 return true;
}
bool Store::eligible(const Position& p)const{
 if(!valid(p))return false;if(!haveLast)return true;
 if(p.absolute!=last.absolute||p.originLat!=last.originLat||p.originLon!=last.originLon)return true;
 return std::hypot(p.x-last.x,p.y-last.y)>=config.distance;
}
bool Store::append(const Position& p,const Reading& r,int64_t now,unsigned ageMs,unsigned probeMs){
 if(!maintain(now)||!eligible(p))return false;
 std::ostringstream row;row.imbue(std::locale::classic());row<<std::setprecision(12)<<now<<','<<p.x<<','<<p.y<<','<<p.east<<','<<p.north<<',';
 if(p.hasLatLon)row<<p.lat;row<<',';if(p.hasLatLon)row<<p.lon;
 row<<','<<p.absolute<<','<<p.originLat<<','<<p.originLon<<','<<(r.state==Reading::Connected?"connected":r.state==Reading::Disconnected?"disconnected":"unknown")<<',';
 if(r.state==Reading::Connected&&r.hasRssi)row<<r.rssi;
 row<<',';if(r.state==Reading::Connected&&r.frequency)row<<r.frequency;
 row<<','<<csv(r.ssid)<<','<<csv(r.bssid)<<','<<ageMs<<','<<probeMs<<'\n';
 const off_t end=lseek(fd,0,SEEK_END);if(end<0)return false;
 if(!writeAll(fd,row.str())){ftruncate(fd,end);return false;}
 last=p;haveLast=true;return true;
}
struct Worker::Impl {
 Config config;Store store;std::function<Reading()> query;
 std::mutex mutex;std::condition_variable wake;Position latest;uint64_t epoch=0;bool stop=false;
 std::thread thread;
 Impl(Config c,std::function<Reading()> q):config(c),store(c),query(q?q:[c]{return queryLink(c.interface);}),thread([this]{run();}){}
 ~Impl(){{std::lock_guard<std::mutex> lock(mutex);stop=true;}wake.notify_all();thread.join();}
 void run(){
  unsigned errors=0;
  for(;;){
   Position before;uint64_t beforeEpoch;
   {std::lock_guard<std::mutex> lock(mutex);if(stop)return;before=latest;beforeEpoch=epoch;}
   const int64_t now=time(nullptr);
   if(!store.maintain(now)){if(errors++%60==0)fprintf(stderr,"WIFI_HEATMAP: log unavailable (path/permissions/header/time/lock)\n");}
   else if(fresh(before,Clock::now())&&store.eligible(before)){
    const auto begin=Clock::now();Reading reading;
    try { reading=query(); } catch (...) { reading=Reading(); }
    const auto end=Clock::now();
    Position after;uint64_t afterEpoch;{std::lock_guard<std::mutex> lock(mutex);after=latest;afterEpoch=epoch;if(stop)return;}
    // Keep positions and RSSI contemporaneous; reject fix loss during the query.
    if(beforeEpoch==afterEpoch&&fresh(after,end)&&std::hypot(after.x-before.x,after.y-before.y)<=.5){
     const auto probe=std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
     if(probe>500)reading=Reading(); // slow/failed probes are unknown, never RF holes
     const auto age=std::chrono::duration_cast<std::chrono::milliseconds>(end-after.received).count();
     store.append(after,reading,time(nullptr),unsigned(age),unsigned(probe));
    }
   }
   std::unique_lock<std::mutex> lock(mutex);if(wake.wait_for(lock,std::chrono::seconds(1),[this]{return stop;}))return;
  }
 }
};
Worker::Worker(Config c,std::function<Reading()> q):impl(new Impl(std::move(c),std::move(q))){}
Worker::~Worker()=default;
void Worker::publish(Position p){p.received=Clock::now();std::lock_guard<std::mutex> lock(impl->mutex);if(!p.fixed&&impl->latest.fixed)impl->epoch++;impl->latest=p;}

std::string page(const Config& config,const std::string& generation,uint64_t offset,uint64_t end){
 std::ifstream in(config.file);if(!in)return "{\"ok\":false,\"error\":\"No heatmap file yet\"}";
 int64_t started=0;if(!header(in,started))return "{\"ok\":false,\"error\":\"Invalid heatmap file\"}";
 const auto first=uint64_t(in.tellg());in.seekg(0,std::ios::end);const auto size=uint64_t(in.tellg());
 const std::string id=std::to_string(started);
 if(offset==0){if(generation!="0"||end!=0)return "{\"ok\":false,\"error\":\"Invalid first page\"}";offset=first;end=size;}
 else if(generation!=id||offset<first||offset>end||end>size)return "{\"ok\":false,\"error\":\"Heatmap changed; reload\"}";
 in.seekg(offset);std::string rows;unsigned count=0;
 while(offset<end&&count<64){
  std::string line;std::getline(in,line);if(!in||in.eof()){offset=end;break;}
  const auto next=uint64_t(in.tellg());if(next>end){offset=end;break;}offset=next;
  if(line.size()>4096)return "{\"ok\":false,\"error\":\"Invalid heatmap row\"}";
  auto row=fields(line);if(row.size()!=17)return "{\"ok\":false,\"error\":\"Invalid heatmap row\"}";
  if(count++)rows+=',';rows+='[';for(size_t i=0;i<row.size();i++){if(i)rows+=',';rows+=json(row[i]);}rows+=']';
 }
 return "{\"ok\":true,\"generation\":"+json(id)+",\"startedUtc\":"+json(utc(started))+",\"end\":"+std::to_string(end)+",\"next\":"+(offset<end?std::to_string(offset):"null")+",\"rows\":["+rows+"]}";
}
}
namespace {
WifiHeatmap::Config settings(){WifiHeatmap::Config c;c.file=WIFI_HEATMAP_FILE;c.interface=WIFI_HEATMAP_INTERFACE;c.distance=WIFI_HEATMAP_MIN_DISTANCE_M;c.days=WIFI_HEATMAP_MAX_AGE_DAYS;return c;}
}
void linuxWifiHeatmapPosition(const WifiHeatmap::Position& position){
 if(!WIFI_HEATMAP_ENABLE)return;
 static WifiHeatmap::Worker worker(settings());worker.publish(position);
}
bool linuxWifiHeatmapRpc(const std::string& request,std::string& response){
 if(request.compare(0,5,"WIFI:")!=0)return false;
 if(request.size()>160||request.size()<43||request[37]!=':')return true;
 for(size_t i=5;i<37;i++)if(!((request[i]>='0'&&request[i]<='9')||(request[i]>='a'&&request[i]<='f')))return true;
 const std::string prefix=request.substr(0,38);
 response=prefix+"{\"ok\":false,\"error\":\"WiFi heatmap logging is disabled in robot config\"}";
 if(!WIFI_HEATMAP_ENABLE)return true;
 std::string args=request.substr(38);for(char c:args)if((c<'0'||c>'9')&&c!=':')return true;
 auto a=args.find(':'),b=args.find(':',a==std::string::npos?0:a+1);
 if(a==std::string::npos||b==std::string::npos||args.find(':',b+1)!=std::string::npos)return true;
 try{size_t used1=0,used2=0;auto off=std::stoull(args.substr(a+1,b-a-1),&used1);auto end=std::stoull(args.substr(b+1),&used2);if(used1!=b-a-1||used2!=args.size()-b-1)return true;response=prefix+WifiHeatmap::page(settings(),args.substr(0,a),off,end);}catch(...){response=prefix+"{\"ok\":false,\"error\":\"Invalid heatmap request\"}";}
 return true;
}
