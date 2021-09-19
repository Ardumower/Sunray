#ifndef _ARDUMOWER_ADAPTER_H
#define _ARDUMOWER_ADAPTER_H

#include <list>
#include <ArduinoJson.h>
#include "encrypt.h"
#include "checksum.h"

namespace ArduMower {

// does not change during runtime
class Properties {
  public:
    String firmware;
    String version;
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

void ArduMower::Adapter::loop(uint32_t now) {
  if (txBufferDirty) processTxBuffer(now);

  if (rxBufferDirty) processRxBuffer(now);
}

void ArduMower::Adapter::tx(String& s) {
  txBuffer += s;
  txBufferDirty = true;
}

void ArduMower::Adapter::tx(char c) {
  if (c == 0) return;
  
  txBuffer += c;
  txBufferDirty = true;
}

void ArduMower::Adapter::rx(char c) {
  if (c == 0) return;
  
  rxBuffer += c;
  rxBufferDirty = true;
}

void ArduMower::Adapter::processTxBuffer(uint32_t now) {
  txBufferDirty = false;

  const int index = txBuffer.indexOf("\r\n");
  if (index == -1) return;

  String buffer = txBuffer.substring(0, index);
  String rest = txBuffer.substring(index + 2);

  String s = buffer.substring(0, 4);

  // the phone app sends the "AT+V" command
  if (s == "AT+V") return;

  decryptBuffer(buffer);

  // TODO read waypoints, stuff

  txBuffer = rest;
  if (rest.length() > 0) txBufferDirty = true;
}

void ArduMower::Adapter::processRxBuffer(uint32_t now) {
  rxBufferDirty = false;

  const int index = rxBuffer.indexOf("\r\n");
  if (index == -1) return;

  String buffer = rxBuffer.substring(0, index);
  String rest = rxBuffer.substring(index + 2);

  rxBuffer = rest;
  if (rest.length() > 0) {
    rxBufferDirty = true;
  }

  if (buffer.length() < 1) return;

  unsigned char crc = 0;
  bool checkCrc = false;

  if (buffer.length() >= 5) {
    String crcVal = buffer.substring(buffer.length() - 5);
    if (crcVal.startsWith(",0x")) {
      crc += ((0xf * crcVal[3 + 0]) + crcVal[3 + 1]);
      checkCrc = true;
    }
  }

  const auto index2 = buffer.indexOf(",");
  String responseType = buffer;
  String remainder = "";

  if (index2 != -1) {
    responseType = buffer.substring(0, index2);
    remainder = buffer.substring(index2 + 1);
  }

  if (responseType == "V") processATVResponse(now, remainder);
  else if (responseType == "S") processATSResponse(now, remainder);
  else CONSOLE.printf("adapter::unknown-response-type (%s)\n", responseType);
}

void ArduMower::Adapter::decryptBuffer(String& buffer) {
  char* buf = strdup(buffer.c_str());
  enc.encrypt(buf, strlen(buf));
  free(buf);
  buffer = String(buf);
}

void ArduMower::Adapter::processATVResponse(uint32_t now, String& res) {
  processCSVResponse(res, [ = ](int index, String & val) {
    // [V,]Ardumower Sunray,1.0.189,1,73,0x58\r
    switch (index) {
      case 0:
        props.firmware = val;
        break;
      case 1:
        props.version = val;
        break;
      case 2:
        enc.setOn(val.toInt() == 1);
        break;
      case 3:
        enc.setChallenge(val.toInt());
        break;
    }
  });

  timeAtv = now;
  
  for (auto it : propertiesListeners) {
    it(props);
  }
}

void ArduMower::Adapter::processATSResponse(uint32_t now, String& res) {
  processCSVResponse(res, [ = ](int index, String & val) {
    switch (index) {
      case 0:
        state.batteryVoltage = val.toFloat();
        break;
      case 1:
        state.position.x = val.toFloat();
        break;
      case 2:
        state.position.y = val.toFloat();
        break;
      case 3:
        state.position.delta = val.toFloat();
        break;
      case 4:
        state.position.solution = val.toInt();
        break;
      case 5:
        state.job = val.toInt();
        break;
      case 6:
        state.position.mowPointIndex = val.toInt();
        break;
      case 7:
        state.position.age = val.toFloat();
        break;
      case 8:
        state.sensor = val.toInt();
        break;

      case 9:
        state.target.x = val.toFloat();
        break;
      case 10:
        state.target.y = val.toFloat();
        break;

      case 11:
        state.position.accuracy = val.toFloat();
        break;
      case 12:
        state.position.visibleSatellites = val.toInt();
        break;
      case 13:
        state.amps = val.toFloat();
        break;
      case 14:
        state.position.visibleSatellitesDgps = val.toInt();
        break;
      case 15:
        state.mapCrc = val.toInt();
        break;
    }
  });

  timeAts = now;
  
  for (auto it : stateListeners) {
    it(state);
  }
}

void ArduMower::Adapter::processCSVResponse(String& res, std::function<void(int, String&)> fn) {
  int index = -1;

  while (res.length() > 0) {
    index++;

    const auto delimiter = res.indexOf(",");
    String val = delimiter == -1 ? res : res.substring(0, delimiter);

    fn(index, val);

    if (delimiter != -1) res = res.substring(delimiter + 1);
    else res = "";
  }
}

void ArduMower::Adapter::sendCommand(String cmd) {
  Checksum chk;
  chk.update(cmd);

  char* buf;
  asprintf(&buf, "%s,0x%02x", cmd.c_str(), chk);
  enc.encrypt(buf, strlen(buf));

  String result = String(buf);
  result += "\r\n";
  io.print(result.c_str());
  
  free(buf);
}

String ArduMower::State::State::toJson() {
  String result = "";
  DynamicJsonDocument doc(1024);

  doc["battery_voltage"] = batteryVoltage;
  doc["position"] = position.toJsonObject();
  doc["target"] = target.toJsonObject();
  doc["job"] = job;
  doc["sensor"] = sensor;
  doc["amps"] = amps;
  doc["map_crc"] = mapCrc;

  serializeJson(doc, result);
  
  return result;
}

JsonObject ArduMower::State::Point::toJsonObject() {
  DynamicJsonDocument doc(1024);
  doc["x"] = x;
  doc["y"] = y;

  return doc.as<JsonObject>();
}

JsonObject ArduMower::State::Position::toJsonObject() {
  DynamicJsonDocument doc(1024);
  doc["x"] = x;
  doc["y"] = y;
  doc["delta"] = delta;
  doc["solution"] = solution;
  doc["age"] = age;
  doc["accuracy"] = accuracy;
  doc["visible_satellites"] = visibleSatellites;
  doc["visible_satellites_dgps"] = visibleSatellitesDgps;
  doc["mow_point_index"] = mowPointIndex;

  return doc.as<JsonObject>();
}

#endif
