#ifndef HTTPSERVER_H
#define HTTPSERVER_H

#include <Arduino.h>

void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void processWifiMqttClient();


#endif
