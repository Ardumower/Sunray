#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT HUP INT TERM
cp sunray/battery.cpp sunray/battery.h sunray/BatteryVoltageFilter.h "$work/"
cat > "$work/Arduino.h" <<'EOF'
#pragma once
#include <cmath>
#include <algorithm>
using std::abs;using std::min;
extern unsigned long testNow;
inline unsigned long millis(){return testNow;}
#define F(x) x
EOF
cat > "$work/robot.h" <<'EOF'
#pragma once
struct BatteryDriverFake {
 float voltage=26,charge=0;
 float getBatteryVoltage(){return voltage;}
 float getChargeVoltage(){return charge;}
 float getChargeCurrent(){return 1;}
 void enableCharging(bool){}
 void keepPowerOn(bool){}
};
struct ConsoleFake {template<class T>void print(T){} template<class T>void println(T){}};
extern ConsoleFake CONSOLE;
struct BuzzerFake {void sound(int,bool){}};
extern BatteryDriverFake batteryDriver;
extern BuzzerFake buzzer;
#define SND_OVERCURRENT 1
#define DEBUG(...) ((void)0)
#define DEBUGLN(...) ((void)0)
EOF
for file in helper.h motor.h buzzer.h; do touch "$work/$file"; done
cat > "$work/config.h" <<'EOF'
#define GO_HOME_VOLTAGE 23.f
#define BAT_UNDERVOLTAGE 18.9f
#define BAT_FULL_CURRENT .2f
#define BAT_FULL_VOLTAGE 29.f
#define BAT_FULL_SLOPE .002f
#define BAT_SWITCH_OFF_UNDERVOLTAGE true
#define BAT_SWITCH_OFF_IDLE false
EOF
for mode in legacy long; do
 if [ "$mode" = long ]; then echo '#define GO_HOME_VOLTAGE_FILTER_TAU_MS 180000UL' >> "$work/config.h"; fi
 "${CXX:-c++}" -std=c++11 -Wall -Wextra ${CXXFLAGS:-} -I"$work" linux/tests/battery-voltage-filter-test.cpp "$work/battery.cpp" -o "$work/test"
 "$work/test"
done
