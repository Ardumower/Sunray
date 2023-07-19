#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>

void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void processWifiMqttClient();


#endif
