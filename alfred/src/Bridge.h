/* Arduino bridge
*/

#ifndef BRIDGE_H_
#define BRIDGE_H_

#include <Arduino.h>
#include <Stream.h>

class BridgeClass {
  public:
    BridgeClass(Stream &_stream);
    void begin();

    // Methods to handle key/value datastore
    void put(const char *key, const char *value);
    void put(const String &key, const String &value){
      put(key.c_str(), value.c_str());
    }
    unsigned int get(const char *key, uint8_t *buff, unsigned int size);
    unsigned int get(const char *key, char *value, unsigned int maxlen){
      return get(key, reinterpret_cast<uint8_t *>(value), maxlen);
    }
    uint16_t getBridgeVersion(){
      return 0;
    }
};

// This subclass uses a serial port Stream
class SerialBridgeClass : public BridgeClass {
  public:
    SerialBridgeClass(Stream &_serial) : BridgeClass(_serial) {
      // Empty
    }
    void begin(unsigned long baudrate=0) {
      BridgeClass::begin();
    }
};

extern SerialBridgeClass Bridge;
extern void checkForRemoteSketchUpdate(uint8_t pin=0);

#endif /* BRIDGE_H_ */

#include <Process.h>
#include <BridgeServer.h>
