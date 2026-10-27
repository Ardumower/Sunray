#pragma once
#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max
#include <chrono>
#include <cstdint>
#include <string>
#include <functional>
#include <memory>
#include <utility>

namespace WifiHeatmap {
using Clock=std::chrono::steady_clock;
struct Position {
 bool fixed=false,absolute=false,hasLatLon=false;
 double east=0,north=0,x=0,y=0,lat=0,lon=0,originLat=0,originLon=0;
 Clock::time_point received{};
};
struct Reading {
 enum State { Unknown=0, Connected=1, Disconnected=2 } state=Unknown;
 bool hasRssi=false;
 int rssi=0,frequency=0;
 std::string ssid,bssid;
};
struct Config {std::string file="wifi-heatmap.csv",interface="wlan0";double distance=1;unsigned days=30;};
Reading parseLink(const std::string&,bool success);
Reading queryLink(const std::string& interface);
class Store {
 Config config;int fd=-1,lockFd=-1;int64_t started=0;bool haveLast=false;Position last;
 bool create(int64_t now);
public:
 explicit Store(Config c):config(std::move(c)){}
 ~Store();
 bool maintain(int64_t now);
 bool eligible(const Position&) const;
 bool append(const Position&,const Reading&,int64_t now,unsigned ageMs,unsigned probeMs);
};
class Worker {
 struct Impl;std::unique_ptr<Impl> impl;
public:
 explicit Worker(Config,std::function<Reading()> query={});
 ~Worker();
 void publish(Position);
};
// Pages of at most 64 rows, stable byte limit across requests; network thread only.
std::string page(const Config&,const std::string& generation,uint64_t offset,uint64_t end);
}
void linuxWifiHeatmapPosition(const WifiHeatmap::Position&);
bool linuxWifiHeatmapRpc(const std::string& request,std::string& response);

#pragma pop_macro("max")
#pragma pop_macro("min")
