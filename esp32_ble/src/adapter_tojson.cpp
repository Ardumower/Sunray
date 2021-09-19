#include "adapter.h"


String ArduMower::Properties::toJson() {
  String result = "";
  DynamicJsonDocument doc(1024);

  doc["firmware"] = firmware;
  doc["version"] = version;

  serializeJson(doc, result);
  
  return result;
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
