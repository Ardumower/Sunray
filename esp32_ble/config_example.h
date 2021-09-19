/*
1. Copy this file into 'config.h'

2. Configure the options below and finally compile and upload this project.
*/

#define NAME "Ardumower"

#define BLE_MTU 20   // max. transfer bytes per BLE frame
#define BLE_MIN_INTERVAL 2    // connection parameters (tuned for high speed/high power consumption - see: https://support.ambiq.com/hc/en-us/articles/115002907792-Managing-BLE-Connection-Parameters)
#define BLE_MAX_INTERVAL 10
#define BLE_LATENCY      0
#define BLE_TIMEOUT      30  

#define MQTT_ENABLED    false
#define MQTT_PREFIX     "/ardumower/"
#define MQTT_HOSTNAME   "mqtt-server.home.lan"
#define MQTT_CLIENT_ID  NAME
#define MQTT_USERNAME   ""
#define MQTT_PASSWORD   ""

#define ENCRYPTION_PASSWORD   123456
#define ENCRYPTION_ENABLED    true

//IP WiFi:
//#define WIFI_STATIC_IP true  // activate this for static IP
#define WIFI_STATIC_IP false // activate this for dynamic IP
#define WIFI_STATIC_IP_LOCAL  10,0,100,11
#define WIFI_STATIC_IP_GW     10,0,100,1
#define WIFI_STATIC_IP_SUBNET 255,255,255,0
#define WIFI_STATIC_IP_DNS1   8,8,8,8
#define WIFI_STATIC_IP_DNS2   8,8,4,4

#define WIFI_STA_SSID   "yourSSID"      // WiFi SSID      (leave empty ("") to not use WiFi)
#define WIFI_STA_PSK    "yourPASSWORD"  // WiFi password  (leave empty ("") to not use WiFi)
#define WIFI_TIMEOUT_FIRST_RESPONSE  800   // fast response times (500), for more reliable choose: 800     
#define WIFI_TIMEOUT_RESPONSE        400   // fast response times (100), for more reliable choose: 400

#define pinGpioRx   16    // UART2 / GPIO16 / IO16
#define pinGpioTx   17    // UART2 / GPIO17 / IO17

//#define pinGpioRx   9   // UART1 / GPIO9  / SD2
//#define pinGpioTx   10  // UART1 / GPIO10 / SD3

//#define pinGpioRx   3   // UART0 / GPIO3  / RXD0
//#define pinGpioTx   1   // UART0 / GPIO1  / TXD0

#define pinLED   2

#define CONSOLE Serial  // where to send/receive console messages for debugging etc.
#define UART Serial2    // where to send/receive UART data

