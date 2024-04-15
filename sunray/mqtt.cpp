#include "mqtt.h"
#include "config.h"
#include "robot.h"
#include "StateEstimator.h"
#include "LineTracker.h"
#include "Stats.h"
#include "src/op/op.h"
#include "reset.h"
#include "timetable.h"

// mqtt
#define MSG_BUFFER_SIZE	(50)
char mqttMsg[MSG_BUFFER_SIZE];
unsigned long nextMQTTPublishTime = 0;
unsigned long nextMQTTLoopTime = 0;



void mqttReconnect() {
  // Loop until we're reconnected
  if (!mqttClient.connected()) {
    CONSOLE.println("MQTT: Attempting connection...");
    // Create a random client ID
    String clientId = "sunray-ardumower";
    // Attempt to connect
#ifdef MQTT_USER
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
#else
    if (mqttClient.connect(clientId.c_str())) {
#endif
      CONSOLE.println("MQTT: connected");
      // Once connected, publish an announcement...
      //mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      CONSOLE.println("MQTT: subscribing " MQTT_TOPIC_PREFIX "/cmd");
      mqttClient.subscribe(MQTT_TOPIC_PREFIX "/cmd");
    } else {
      CONSOLE.print("MQTT: failed, rc=");
      CONSOLE.print(mqttClient.state());
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  CONSOLE.print("MQTT: Message arrived [");
  CONSOLE.print(topic);
  CONSOLE.print("] ");
  String cmd = ""; 
  for (int i = 0; i < length; i++) {
    cmd += (char)payload[i];    
  }
  CONSOLE.println(cmd);
  if (cmd == "dock") {
    setOperation(OP_DOCK, false);
  } else if (cmd ==  "stop") {
    setOperation(OP_IDLE, false);
  } else if (cmd == "start"){
    setOperation(OP_MOW, false);
  }
}

// define a macro so avoid repetitive code lines for sending single values via MQTT
#define MQTT_PUBLISH(VALUE, FORMAT, TOPIC) \
      snprintf (mqttMsg, MSG_BUFFER_SIZE, FORMAT, VALUE); \
      mqttClient.publish(MQTT_TOPIC_PREFIX TOPIC, mqttMsg);    


// process MQTT input/output (subcriber/publisher)
void processWifiMqttClient()
{
  if (!ENABLE_MQTT) return; 
  if (millis() >= nextMQTTPublishTime){
    nextMQTTPublishTime = millis() + 20000;
    if (mqttClient.connected()) {
      updateStateOpText();
      // operational state
      //CONSOLE.println("MQTT: publishing " MQTT_TOPIC_PREFIX "/status");      
      MQTT_PUBLISH(stateOpText.c_str(), "%s", "/op")
      MQTT_PUBLISH(maps.percentCompleted, "%d", "/progress")

      // GPS related information
      snprintf (mqttMsg, MSG_BUFFER_SIZE, "%.2f, %.2f", gps.relPosN, gps.relPosE);          
      mqttClient.publish(MQTT_TOPIC_PREFIX "/gps/pos", mqttMsg);
      MQTT_PUBLISH(gpsSolText.c_str(), "%s", "/gps/sol")
      MQTT_PUBLISH(gps.iTOW, "%lu", "/gps/tow")
      
      MQTT_PUBLISH(gps.lon, "%.8f", "/gps/lon")
      MQTT_PUBLISH(gps.lat, "%.8f", "/gps/lat")
      MQTT_PUBLISH(gps.height, "%.1f", "/gps/height")
      MQTT_PUBLISH(gps.relPosN, "%.4f", "/gps/relNorth")
      MQTT_PUBLISH(gps.relPosE, "%.4f", "/gps/relEast")
      MQTT_PUBLISH(gps.relPosD, "%.2f", "/gps/relDist")
      MQTT_PUBLISH((millis()-gps.dgpsAge)/1000.0, "%.2f","/gps/ageDGPS")
      MQTT_PUBLISH(gps.accuracy, "%.2f", "/gps/accuracy")
      MQTT_PUBLISH(gps.groundSpeed, "%.4f", "/gps/groundSpeed")
      
      // power related information      
      MQTT_PUBLISH(battery.batteryVoltage, "%.2f", "/power/battery/voltage")
      MQTT_PUBLISH(motor.motorsSenseLP, "%.2f", "/power/motor/current")
      MQTT_PUBLISH(battery.chargingVoltage, "%.2f", "/power/battery/charging/voltage")
      MQTT_PUBLISH(battery.chargingCurrent, "%.2f", "/power/battery/charging/current")

      // map related information
      MQTT_PUBLISH(maps.targetPoint.x(), "%.2f", "/map/targetPoint/X")
      MQTT_PUBLISH(maps.targetPoint.y(), "%.2f", "/map/targetPoint/Y")
      MQTT_PUBLISH(stateX, "%.2f", "/map/pos/X")
      MQTT_PUBLISH(stateY, "%.2f", "/map/pos/Y")
      MQTT_PUBLISH(stateDelta, "%.2f", "/map/pos/Dir")

      // statistics
      MQTT_PUBLISH((int)statIdleDuration, "%d", "/stats/idleDuration")
      MQTT_PUBLISH((int)statChargeDuration, "%d", "/stats/chargeDuration")
      MQTT_PUBLISH((int)statMowDuration, "%d", "/stats/mow/totalDuration")
      MQTT_PUBLISH((int)statMowDurationInvalid, "%d", "/stats/mow/invalidDuration")
      MQTT_PUBLISH((int)statMowDurationFloat, "%d", "/stats/mow/floatDuration")
      MQTT_PUBLISH((int)statMowDurationFix, "%d", "/stats/mow/fixDuration")
      MQTT_PUBLISH((int)statMowFloatToFixRecoveries, "%d", "/stats/mow/floatToFixRecoveries")
      MQTT_PUBLISH((int)statMowObstacles, "%d", "/stats/mow/obstacles")
      MQTT_PUBLISH((int)statMowGPSMotionTimeoutCounter, "%d", "/stats/mow/gpsMotionTimeouts")
      MQTT_PUBLISH((int)statMowBumperCounter, "%d", "/stats/mow/bumperEvents")
      MQTT_PUBLISH((int)statMowSonarCounter, "%d", "/stats/mow/sonarEvents")
      MQTT_PUBLISH((int)statMowLiftCounter, "%d", "/stats/mow/liftEvents")
      MQTT_PUBLISH(statMowMaxDgpsAge, "%.2f", "/stats/mow/maxDgpsAge")
      MQTT_PUBLISH(statMowDistanceTraveled, "%.1f", "/stats/mow/distanceTraveled")
      MQTT_PUBLISH((int)statMowInvalidRecoveries, "%d", "/stats/mow/invalidRecoveries")
      MQTT_PUBLISH((int)statImuRecoveries, "%d", "/stats/imuRecoveries")
      MQTT_PUBLISH((int)statGPSJumps, "%d", "/stats/gpsJumps")      
      MQTT_PUBLISH(statTempMin, "%.1f", "/stats/tempMin")
      MQTT_PUBLISH(statTempMax, "%.1f", "/stats/tempMax")
      MQTT_PUBLISH(stateTemp, "%.1f", "/stats/curTemp")

    } else {
      mqttReconnect();  
    }
  }
  if (millis() > nextMQTTLoopTime){
    nextMQTTLoopTime = millis() + 20000;
    mqttClient.loop();
  }
}


