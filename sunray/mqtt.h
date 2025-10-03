#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>

class MqttService {
public:
  void begin();
  void reconnect();
  void callback(char* topic, uint8_t* payload, unsigned int length);
  void process();

private:
  static const size_t MSG_BUFFER_SIZE = 50;
  char mqttMsg[MSG_BUFFER_SIZE];
  unsigned long nextMQTTPublishTime = 0;
  unsigned long nextMQTTLoopTime = 0;
};




#endif
