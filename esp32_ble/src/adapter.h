#ifndef _ARDUMOWER_ADAPTER_H
#define _ARDUMOWER_ADAPTER_H

#include "../config.h"

#ifdef USE_MQTT


#include <list>
#include <functional>
#include <ArduinoJson.h>
#include "encrypt.h"
#include "checksum.h"

namespace ArduMower {

// does not change during runtime
class Properties {
  public:
    String firmware;
    String version;

    String toJson();
};

// changes during runtime
class Stats {
  public:
    int statIdleDuration;
    int statChargeDuration;
    float statMaxControlCycleTime;
    int serial_buffer_size;
    int freeMemory;
    int resetCause;
    int chksumErrorCounter;
    int dgpsChecksumErrorCounter;
    int statGpsJumps;
    int statMowGpsMotionTimeoutCounter;
    int statMowDuration;
    int statMowDurationFloat;
    int statMowDurationFix;
    int statMowFloatToFixRecoveries;
    int statMowDistanceTraveled;
    float statMowMaxDgpsAge;
    int statMowDurationInvalid;
    int statMowInvalidRecoveries;
    int statImuRecoveries;
    int statMowObstacles;
    int statMowSonarCounter;
    int statMowBumperCounter;
    float statTempMin;
    float statTempMax;

    Stats()
      : statIdleDuration(0), statChargeDuration(0), statMaxControlCycleTime(0), serial_buffer_size(0), freeMemory(0), resetCause(0),
      chksumErrorCounter(0), dgpsChecksumErrorCounter(0), statGpsJumps(0), statMowGpsMotionTimeoutCounter(0),
      statMowDuration(0), statMowDurationFloat(0), statMowDurationFix(0), statMowFloatToFixRecoveries(0), statMowDistanceTraveled(0), statMowMaxDgpsAge(0), statMowDurationInvalid(0), statMowInvalidRecoveries(0),
      statImuRecoveries(0), statMowObstacles(0), statMowSonarCounter(0), statMowBumperCounter(0), statTempMin(0), statTempMax(0)
      {}
    
    String toJson();
};

namespace State {
class Point {
  public:
    float x, y;

    Point()
      : x(0), y(0)
    {};

    JsonObject toJsonObject();
};

class Position : public Point {
  public:
    float delta;
    int solution;
    float age;
    float accuracy;
    int visibleSatellites, visibleSatellitesDgps;
    int mowPointIndex;

    Position()
      : delta(0),
        solution(0), age(0), accuracy(0), visibleSatellites(0), visibleSatellitesDgps(0),
        mowPointIndex(0)
    {};

    JsonObject toJsonObject();
};

class State {
  public:
    float batteryVoltage;
    Position position;
    Point target;
    int job;
    int sensor;
    float amps;
    int mapCrc;

    State()
      : batteryVoltage(0), job(0), sensor(0), amps(0), mapCrc(0)
    {}

    String toJson();
};

};

class Adapter {
  private:
    Stream& io;
    Encrypt enc;

    // uart traffic buffers
    String txBuffer, rxBuffer;
    bool txBufferDirty, rxBufferDirty;

    // data freshness
    uint32_t timeAtv, timeAts, timeAtt;

    // mutators
    void processTxBuffer(uint32_t now);
    void processRxBuffer(uint32_t now);
    void decryptBuffer(String& buffer);

    void processATVResponse(uint32_t now, String& res);
    void processATSResponse(uint32_t now, String& res);
    void processATTResponse(uint32_t now, String& res);

    void processCSVResponse(String& res, std::function<void(int, String&)> fn);

  public:
    // business end
    Properties props;
    Stats stats;
    State::State state;
    std::list<std::function<void(State::State&)>> stateListeners;
    std::list<std::function<void(Properties&)>> propertiesListeners;
    std::list<std::function<void(Stats&)>> statsListeners;

    Adapter(Stream& _io, int encryptPass, bool encryptionOn)
      : io(_io),
      txBuffer(""), rxBuffer(""), txBufferDirty(false), rxBufferDirty(false),
      timeAtv(0), timeAts(0), timeAtt(0)
    {
      enc.setPassword(encryptPass);
    };

    void loop(uint32_t now);

    // ESP sends data to ArduMower
    void tx(String& s);
    void tx(char c);

    // ESP receives data from ArduMower
    void rx(char c);

    void addStateListener(std::function<void(State::State& s)> fn) {
      stateListeners.push_back(fn);
    }

    void addPropertiesListeners(std::function<void(Properties& p)> fn) {
      propertiesListeners.push_back(fn);
    }
    
    void addStatsListeners(std::function<void(Stats& s)> fn) {
      statsListeners.push_back(fn);
    }

    uint32_t ageAtv(uint32_t now) { return now - timeAtv; }
    uint32_t ageAts(uint32_t now) { return now - timeAts; }
    uint32_t ageAtt(uint32_t now) { return now - timeAtt; }

    void sendCommand(String cmd);
};

};

#endif

#endif
