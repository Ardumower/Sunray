// Real UBLOX parser/configuration, fake UART and clock; no robot commands.
#include <cassert>
#include <chrono>
#include <deque>
#include <map>
#include <vector>
#include <iostream>
#include "../../sunray/src/net/CloudWorker.h"
#include "../../sunray/src/ublox/ublox.h"
#include "../../sunray/events.h"
#include "../../sunray/robot.h"
#include "../../sunray/motor.h"
#undef min
#undef max
#undef round
#undef yield

static uint32_t now;
unsigned long millis() { return now; }
void delay(uint32_t) { assert(false && "GPS startup must never delay"); }
void delayMicroseconds(uint32_t) { assert(false); }
void thread_yield() { assert(false); }
static unsigned errors;
EventLogger::EventLogger() {}
void EventLogger::event(EventCode code) { assert(code==EVT_ERROR_GPS_NOT_CONNECTED); ++errors; }
EventLogger Logger;
UBLOX gps;
TestMotorDriver motorDriver;
TestRobotDriver robotDriver;

struct Receiver : HardwareSerial {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  std::vector<std::map<uint32_t,uint32_t>> configs;
  uint32_t baud=0, receiverBaud=115200, generation=0;
  bool online=false, blocked=false, ack=true, nak=false, version=true;
  unsigned polls=0, writes=0;
  void begin(uint32_t b) override { baud=b; }
  uint32_t connectionGeneration() const override { return generation; }
  int available() override { return rx.size(); }
  int read() override { assert(!rx.empty()); int b=rx.front(); rx.pop_front(); return b; }
  void packet(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload, bool corrupt=false){
    std::vector<uint8_t> p={0xb5,0x62,cls,id,(uint8_t)payload.size(),(uint8_t)(payload.size()>>8)};
    p.insert(p.end(),payload.begin(),payload.end());
    uint8_t a=0,b=0;
    for (size_t i=2;i<p.size();++i){ a+=p[i]; b+=a; }
    p.push_back(a); p.push_back(corrupt ? b^1 : b);
    rx.insert(rx.end(),p.begin(),p.end());
  }
  void position(){ std::vector<uint8_t> p(64); p[0]=1; p[60]=7|(2<<3); packet(1,0x3c,p); }
  size_t write(uint8_t b) override {
    if (blocked) return 0;
    ++writes; tx.push_back(b);
    if (tx.size()<8 || tx.size()!=size_t(8+tx[4]+256*tx[5])) return 1;
    uint8_t a=0,c=0;
    for(size_t i=2;i<tx.size()-2;++i){a+=tx[i];c+=a;}
    assert(a==tx[tx.size()-2] && c==tx.back());
    if(online && baud==receiverBaud){
      if(tx[2]==6 && tx[3]==8){ ++polls; packet(6,8,{200,0,1,0,0,0}); }
      else if(tx[2]==0x0a && tx[3]==4){ if(version) packet(0x0a,4,std::vector<uint8_t>(40,0)); }
      else if(tx[2]==6 && tx[3]==0x8a){
        assert(tx[6]==0 && tx[7]==1 && tx[8]==0 && tx[9]==0); // RAM only
        std::map<uint32_t,uint32_t> values;
        for(size_t at=10;at<tx.size()-2;){
          uint32_t key=0,value=0;
          for(int i=0;i<4;++i) key|=uint32_t(tx[at++])<<(8*i);
          unsigned storage=key>>28, width=storage<=2?1:storage==3?2:4;
          for(unsigned i=0;i<width;++i) value|=uint32_t(tx[at++])<<(8*i);
          values[key]=value;
        }
        configs.push_back(values);
        if(values.count(0x40520001)) receiverBaud=values.at(0x40520001);
        if(ack) packet(5,nak?0:1,{6,0x8a});
      } else assert(false && "unexpected command");
    }
    tx.clear(); return 1;
  }
};

static void step(UBLOX& gps, uint32_t ms=10){
  now+=ms;
  const auto before=std::chrono::steady_clock::now();
  gps.run();
  assert(std::chrono::steady_clock::now()-before < std::chrono::milliseconds(50));
}
static void ready(UBLOX& gps){
  for(unsigned i=0;i<3000 && gps.isConfiguring();++i) step(gps);
  assert(!gps.isConfiguring()); assert(gps.solution==SOL_INVALID);
}
static void checkConfig(const Receiver& r, size_t first=0){
  assert(r.configs.size()==first+10);
  assert(r.configs[first].at(0x10530005)==1); // correction radio UART2 stays enabled
  assert(r.configs[first+2].at(0x2091005c)==0); // TIMEUTC disabled (not timeout=2000)
  assert(r.configs[first+4].at(0x10750004)==1); // RTCM3 input from radio
  assert(r.configs[first+4].at(0x40530001)==115200);
  assert(r.configs[first+7].at(0x30210001)==200);
  assert(r.configs[first+7].at(0x30210002)==1);
  assert(r.configs[first+8].at(0x20910090)==1);
  assert(r.configs[first+9].at(0x2091008e)==1);
}
struct TestMotor : Motor {
  using Motor::speedPWM;
  using Motor::motorMowPWMSet;
  using Motor::motorLeftRpmSet;
  using Motor::motorRightRpmSet;
};
int main(){
  now=0;
  Receiver uart; gps.begin(uart,115200);
  TestMotor motor{}; motor.begin();
  motor.releaseBrakesWhenZero=true; motor.motorReleaseBrakesTime=0;
  now=100;
  motor.setLinearAngularSpeed(0.5,0.2,true); motor.setMowState(true);
  motor.speedPWM(100,100,200);
  assert(motor.linearSpeedSet==0 && motor.angularSpeedSet==0 && motor.motorMowPWMSet==0);
  assert(motorDriver.left==0 && motorDriver.right==0 && motorDriver.mow==0 && !motorDriver.release);
  assert(gps.isConfiguring() && gps.solution==SOL_INVALID && uart.writes==0);
  // Main/control-side cloud work continues throughout missing hardware.
  std::atomic<bool> connected{false}, delivered{false};
  CloudWorker worker({[&]{connected=true;return true;},[&]{return connected.load();},
    [&]{connected=false;},[&](std::string& s){if(delivered.exchange(true))return false;s="AT+V";return true;},
    [](const std::string&){return true;},[](const unsigned char*,size_t){return true;}},5);
  worker.start(); bool command=false;
  for(unsigned i=0;i<1600;++i){
    step(gps);
    CloudWorker::Request request;
    if(worker.take(request)){command=true;worker.reply(request.session,"V,test");}
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  worker.stopAndJoin();
  assert(command && gps.isConfiguring() && gps.solution==SOL_INVALID && errors>=1);
  uart.online=true; uart.position(); step(gps); assert(gps.solution==SOL_INVALID);
  ready(gps); checkConfig(uart);
  uart.position(); step(gps); assert(gps.solution==SOL_FIXED);
  motor.setLinearAngularSpeed(0.5,0.2,false); motor.setMowState(true);
  motor.speedPWM(100,100,200);
  assert(motor.linearSpeedSet>0 && motor.motorMowPWMSet!=0 && motorDriver.left!=0);
  std::cout<<"PASS motor and brake interlock during pending GPS configuration\n";
  std::cout<<"PASS absent/late receiver, cloud request, ten RAM packets and radio settings\n";

  // A lost fix with live UBX traffic must not reconfigure the receiver.
  for(unsigned i=0;i<12;++i){uart.packet(2,0x32,std::vector<uint8_t>(8));step(gps,1000);}
  assert(!gps.isConfiguring() && gps.solution==SOL_INVALID && uart.configs.size()==10);
  // A brief USB close/reopen must reconfigure, even before the silence timeout.
  ++uart.generation; uart.position(); step(gps);
  assert(gps.isConfiguring() && gps.solution==SOL_INVALID);
  motor.run();
  assert(motor.motorLeftRpmSet==0 && motor.motorRightRpmSet==0 && motor.motorMowPWMSet==0);
  assert(motorDriver.left==0 && motorDriver.right==0 && motorDriver.mow==0 && !motorDriver.release);
  ready(gps); checkConfig(uart,10);
  uart.position();step(gps);assert(gps.solution==SOL_FIXED);
  step(gps,10001);assert(gps.isConfiguring() && gps.solution==SOL_INVALID);
  ready(gps);checkConfig(uart,20);
  std::cout<<"PASS USB reconnect and silent stream reconfigure; loss of FIX alone does not\n";

  Receiver fallback; fallback.online=true; fallback.receiverBaud=38400; fallback.version=false;
  UBLOX fg; fg.begin(fallback,115200); ready(fg);
  assert(fallback.configs.front().at(0x40520001)==115200);checkConfig(fallback,1);
  std::cout<<"PASS fallback baud and missing optional MON-VER\n";

  Receiver rejected; rejected.online=true;rejected.nak=true;
  UBLOX ng;ng.begin(rejected,115200);
  for(unsigned i=0;i<50;++i)step(ng);
  assert(ng.isConfiguring() && rejected.configs.size()==2);
  size_t old=rejected.configs.size();
  for(unsigned i=0;i<900;++i)step(ng);
  assert(rejected.configs.size()==old); // ten-second retry pause, not a busy loop
  rejected.nak=false;ready(ng);checkConfig(rejected,2);
  std::cout<<"PASS NAK, two attempts, delayed retry and recovery\n";

  Receiver stale;stale.online=true;stale.ack=false;
  UBLOX sg;sg.begin(stale,115200);
  while(stale.configs.empty())step(sg);
  stale.packet(5,1,{6,0x8a},true);stale.packet(5,1,{6,8});stale.packet(5,1,{6});
  for(unsigned i=0;i<30;++i)step(sg);
  assert(stale.configs.size()==1 && sg.isConfiguring() && sg.chksumErrorCounter==1);
  for(unsigned i=0;i<500;++i)step(sg);
  assert(stale.configs.size()==2 && sg.isConfiguring());
  std::cout<<"PASS corrupt/unrelated/short ACKs and missing-ACK timeout\n";

  Receiver blocked;blocked.blocked=true;UBLOX bg;bg.begin(blocked,115200);
  unsigned previous=errors;
  for(unsigned i=0;i<250;++i)step(bg);
  assert(errors==previous+1 && bg.isConfiguring());
  blocked.blocked=false;blocked.online=true;ready(bg);checkConfig(blocked);
  // Input flood is bounded per run, keeping command polling possible.
  blocked.rx.assign(3000,0);step(bg);assert(blocked.rx.size()==1976);
  std::cout<<"PASS TX backpressure, recovery and bounded receive work\n";

  now=0xfffffff0u;
  Receiver wrap;wrap.online=true;UBLOX wg;wg.begin(wrap,115200);ready(wg);checkConfig(wrap);
  wrap.position();step(wg);assert(wg.solution==SOL_FIXED);
  step(wg,3001);assert(wg.solution==SOL_INVALID);
  std::cout<<"PASS clock wraparound and position watchdog\n";
}
