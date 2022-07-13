#include "adapter.h"
#include "../config.h"

#ifdef USE_MQTT



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

String ArduMower::Stats::toJson() {
  String result = "";
  DynamicJsonDocument doc(1024);
  
  doc["duration_idle"] = statIdleDuration;
  doc["duration_charge"] = statChargeDuration;
  doc["duration_mow"] = statMowDuration;
  doc["duration_mow_invalid"] = statMowDurationInvalid;
  doc["duration_mow_float"] = statMowDurationFloat;
  doc["duration_mow_fix"] = statMowDurationFix;
  
  doc["distance_mow_traveled"] = statMowDistanceTraveled;
  
  doc["counter_gps_chk_sum_errors"] = chksumErrorCounter;
  doc["counter_dgps_chk_sum_errors"] = dgpsChecksumErrorCounter;
  doc["counter_invalid_recoveries"] = statMowInvalidRecoveries;
  doc["counter_float_recoveries"] = statMowFloatToFixRecoveries;
  doc["counter_gps_jumps"] = statGpsJumps;
  doc["counter_gps_motion_timeout"] = statMowGpsMotionTimeoutCounter;
  doc["counter_imu_triggered"] = statImuRecoveries;
  doc["counter_sonar_triggered"] = statMowSonarCounter;
  doc["counter_bumper_triggered"] = statMowBumperCounter;
  doc["counter_obstacles"] = statMowObstacles;
  
  doc["time_max_cycle"] = statMaxControlCycleTime;
  doc["time_max_dpgs_age"] = statMowMaxDgpsAge;
  
  doc["serial_buffer_size"] = serial_buffer_size;
  doc["free_memory"] = freeMemory;
  doc["reset_cause"] = resetCause;
  
  doc["temp_min"] = statTempMin;
  doc["temp_max"] = statTempMax;
  
  serializeJson(doc, result);
  
  return result;
}
#endif