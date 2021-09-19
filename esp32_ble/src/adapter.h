#ifndef _ARDUMOWER_ADAPTER_H
#define _ARDUMOWER_ADAPTER_H

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
    uint32_t timeAtv, timeAts;

    // mutators
    void processTxBuffer(uint32_t now);
    void processRxBuffer(uint32_t now);
    void decryptBuffer(String& buffer);

    void processATVResponse(uint32_t now, String& res);
    void processATSResponse(uint32_t now, String& res);

    void processCSVResponse(String& res, std::function<void(int, String&)> fn);

  public:
    // business end
    Properties props;
    State::State state;
    std::list<std::function<void(State::State&)>> stateListeners;
    std::list<std::function<void(Properties&)>> propertiesListeners;

    Adapter(Stream& _io, int encryptPass, bool encryptionOn)
      : io(_io),
      txBuffer(""), rxBuffer(""), txBufferDirty(false), rxBufferDirty(false),
      timeAtv(0), timeAts(0)
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

    uint32_t ageAtv(uint32_t now) { return now - timeAtv; }
    uint32_t ageAts(uint32_t now) { return now - timeAts; }

    void sendCommand(String cmd);
};

};


#endif
