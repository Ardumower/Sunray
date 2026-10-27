#include "WifiHeatmap.h"
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <atomic>
#include <thread>
using namespace WifiHeatmap;
int main(){
 char dir[]="/tmp/sunray-wifi-test-XXXXXX";assert(mkdtemp(dir));
 Config c;c.file=std::string(dir)+"/samples.csv";
 auto r=parseLink("Connected to aa:bb:cc:dd:ee:ff (on wlan0)\n\tSSID: a,\"b\"\n\tfreq: 2412\n\tsignal: -67 dBm\n",true);
 assert(r.state==Reading::Connected&&r.rssi==-67&&r.ssid=="a,\"b\"");
 auto disconnected=parseLink("Not connected.\n",true);assert(disconnected.state==Reading::Disconnected&&!disconnected.hasRssi);
 assert(parseLink("Not connected.\n",false).state==Reading::Unknown);
 const int64_t now=1800000000;Position p;p.fixed=true;
 {
  Store store(c);assert(store.maintain(now));Store other(c);assert(!other.maintain(now));
  p.fixed=false;assert(!store.append(p,r,now,10,2));p.fixed=true;
  assert(store.append(p,r,now,10,2));p.x=.99;assert(!store.append(p,r,now,10,2));
  p.x=1;assert(store.append(p,disconnected,now,10,2));
  p.x=std::numeric_limits<double>::quiet_NaN();assert(!store.eligible(p));p.x=1;
 }
 {std::ofstream tail(c.file,std::ios::app);tail<<"partial";}
 {
  Store store(c);assert(store.maintain(now));assert(!store.eligible(p));
  for(int i=2;i<70;i++){p.x=i;assert(store.append(p,disconnected,now,10,2));}
  const auto result=page(c,"0",0,0);assert(result.find("\"ok\":true")!=std::string::npos);assert(result.find("\"next\":null")==std::string::npos);
  assert(result.find("a,\\\"b\\\"")!=std::string::npos);
  assert(page(c,"wrong",1000,1000).find("\"ok\":false")!=std::string::npos);
  assert(!store.maintain(now-1));assert(store.maintain(now+30*86400));
  assert(page(c,"0",0,0).find("\"rows\":[]")!=std::string::npos);
 }
 // A blocked WLAN probe must not hold the control-thread mutex; fix loss
 // during the probe must invalidate the pending sample even after reacquisition.
 Config slow=c;slow.file=std::string(dir)+"/slow.csv";
 std::atomic<bool> entered{false},release{false};
 {
  Worker worker(slow,[&]{entered=true;while(!release)std::this_thread::sleep_for(std::chrono::milliseconds(1));return r;});
  worker.publish(p);
  auto deadline=Clock::now()+std::chrono::seconds(3);
  while(!entered&&Clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(entered);
  const auto begin=Clock::now();p.fixed=false;worker.publish(p);p.fixed=true;worker.publish(p);
  assert(Clock::now()-begin<std::chrono::milliseconds(100));release=true;
 }
 assert(page(slow,"0",0,0).find("\"rows\":[]")!=std::string::npos);
 unlink(slow.file.c_str());unlink((slow.file+".lock").c_str());
 std::string response;assert(!linuxWifiHeatmapRpc("AT+S",response));
 assert(linuxWifiHeatmapRpc("WIFI:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:0:0:0",response));assert(response.find("disabled")!=std::string::npos);
 unlink(c.file.c_str());unlink((c.file+".lock").c_str());rmdir(dir);
 std::cout<<"WiFi parse, RTK gate, spacing, locking, restart, tail recovery, rotation, pagination, default-off: OK\n";
}
