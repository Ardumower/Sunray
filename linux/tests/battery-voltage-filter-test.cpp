#include "battery.h"
#include "config.h"
#include "robot.h"
#include <cassert>
#include <cstdio>
#include <cmath>
unsigned long testNow=0;
BatteryDriverFake batteryDriver;
BuzzerFake buzzer;
ConsoleFake CONSOLE;
static void near(float a,float b,float epsilon=.002f){assert(std::fabs(a-b)<epsilon);}
static void start(Battery& b,float voltage){
 batteryDriver.voltage=voltage;batteryDriver.charge=0;testNow=0;b.begin();b.run();
 assert(!b.shouldGoHome());testNow=2000;b.run();near(b.batteryVoltage,voltage);
 near(b.batteryVoltageGoHome,voltage);
}
static void advance(Battery& b,unsigned duration,float voltage){
 batteryDriver.voltage=voltage;const unsigned long until=testNow+duration;
 while(testNow<until){testNow+=50;b.run();}
}
int main(){
 // Exact elapsed-time response, duplicate timestamps and uint32 clock wrap.
 BatteryVoltageFilter f;near(f.update(26,100,180000),26);
 near(f.update(22,10100,180000),22+4*std::exp(-10.f/180));
 const float before=f.update(22,10100,180000);near(f.update(18,10100,180000),before);
 BatteryVoltageFilter wrapped;wrapped.update(26,0xfffff000u,180000);
 near(wrapped.update(22,uint32_t(0xfffff000u+10000u),180000),before);
 BatteryVoltageFilter delayed;delayed.update(26,0,180000);
 near(delayed.update(22,180000,180000),22+4*std::exp(-1.f));
 near(delayed.update(0,180050,180000),0);near(delayed.update(25,180100,180000),25);
 delayed.reset();near(delayed.update(21,180150,180000),21);
 Battery b{};start(b,26);assert(!b.shouldGoHome());
 advance(b,10000,22);
#ifdef GO_HOME_VOLTAGE_FILTER_TAU_MS
 near(b.batteryVoltageGoHome,25.78384f);assert(!b.shouldGoHome());
 const float rawAfterDip=b.batteryVoltageGoHome;
 advance(b,30000,26);assert(b.batteryVoltageGoHome>rawAfterDip && !b.shouldGoHome());
 start(b,25);advance(b,10000,21);assert(b.batteryVoltage<23 && !b.shouldGoHome());
 near(b.batteryVoltageGoHome,24.78384f);
 start(b,26);advance(b,180000,22);near(b.batteryVoltageGoHome,22+4*std::exp(-1.f));
 assert(!b.shouldGoHome());advance(b,70000,22);assert(b.shouldGoHome());
 // A long return filter cannot delay the existing undervoltage decision.
 start(b,26);advance(b,30000,18);assert(b.underVoltage());assert(!b.shouldGoHome());
 // Boot on an already low pack must not wait for three minutes.
 start(b,22);assert(b.shouldGoHome());
 // Charging changes the display filter, not the return filter's time constant.
 start(b,26);batteryDriver.charge=30;advance(b,10000,22);
 assert(b.chargerConnected());assert(b.batteryVoltage<22.1f);near(b.batteryVoltageGoHome,25.78384f);
 puts("PASS production battery: 10s dip attenuated, sustained low return, independent undervoltage, startup/reset, charging and clock wrap");
#else
 near(b.batteryVoltageGoHome,b.batteryVoltage);
 advance(b,30000,22);assert(b.shouldGoHome());near(b.batteryVoltageGoHome,b.batteryVoltage);
 puts("PASS existing profiles retain the original return voltage filter");
#endif
}
