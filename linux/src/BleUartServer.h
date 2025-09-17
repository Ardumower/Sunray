//
// BLE GATT server offering UART service 


#ifndef BLE_UART_SERVER_H
#define BLE_UART_SERVER_H

#include <Arduino.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#include "Stream.h"
#include "HardwareSerial.h"

extern "C" {
  #include <bluetooth/bluetooth.h>
  #include <bluetooth/hci.h>
  #include <bluetooth/hci_lib.h>
  #include <bluetooth/l2cap.h>

  #include "ble/uuid.h"
  #include "ble/mainloop.h"
  //#include "ble/util.h"
  //#include "ble/att.h"
  //#include "ble/queue.h"
  #include "ble/timeout.h"
  #include "ble/gatt-db.h"
  #include "ble/gatt-server.h"
}

#define BLE_BUF_SZ 8192

class BleUartServer: public HardwareSerial{
  protected:
	  bool verbose;
    int fd;
    uint16_t mtu;
    struct bt_att *att;
	  struct gatt_db *db;
	  struct bt_gatt_server *gatt;
    int sec;
	  uint8_t src_type;  
    bdaddr_t src_addr;
    uint16_t handle;
    struct gatt_db_attribute *service;
    struct gatt_db_attribute *msrmt;
    uint16_t msrmt_handle;	
    pthread_t thread_id;
    virtual void populateUartService();
    virtual void populateDb();
    virtual bool listen();
    virtual bool createGattServer();
    virtual void destroyGattServer();
  public:
    int rxReadPos;
    int rxWritePos;
    int txReadPos;
    int txWritePos;
    byte rxBuf[BLE_BUF_SZ];
    byte txBuf[BLE_BUF_SZ];
    bool msrmt_enabled;
    pthread_mutex_t txMutex;
    pthread_mutex_t rxMutex;
    
    BleUartServer();
    virtual ~BleUartServer() { end(); };
    virtual bool begin();
    virtual bool begin(int baudrate);
    virtual void end();
    virtual void run();
    virtual void notify();	  

    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush();

    virtual size_t write(const uint8_t c) override;
    
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool() { return true; }
};

//extern BleUartServer BLE;


#endif
