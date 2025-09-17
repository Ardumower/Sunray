#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include "Arduino.h"

#include "LinuxSerial.h"


//#define LINUX_SERIAL_FIFO  1



void *linuxSerialRxThreadFun(void *user_data)
{
    LinuxSerial *ser = (LinuxSerial*)user_data;
	  while (true){
	    ser->runRx();
      //delayMicroseconds(500);
      delay(1);
	  }
	  return NULL;
}


void *linuxSerialTxThreadFun(void *user_data)
{
    LinuxSerial *ser = (LinuxSerial*)user_data;
	  while (true){
	    ser->runTx();
      //delayMicroseconds(500);
      delay(1);
	  }
	  return NULL;
}


void LinuxSerial::begin(const char *devicePath){    
  open(devicePath);
}

void LinuxSerial::begin(const char *devicePath, uint32_t baudrate){      
  if (!open(devicePath)) return;
  setBaudrate(baudrate); 
}

void LinuxSerial::begin(uint32_t baudrate){
  setBaudrate(baudrate);
} 

  
bool LinuxSerial::setBaudrate(uint32_t baudrate){
  if(!_stream) return false;    
  ::printf("setting baudrate %s %d...\n", devPath.c_str(), baudrate);
  struct termios newtermios;  
  tcgetattr(_stream, &_termios);
  memset(&newtermios, 0, sizeof(struct termios));
  int baud;
  switch(baudrate){
      case 50:  baud = B50; break;
      case 110: baud = B110; break;
      case 300: baud = B300; break;
      case 600: baud = B600; break;
      case 1200: baud = B1200; break;
      case 2400: baud = B2400; break;
      case 4800: baud = B4800; break;
      case 9600: baud = B9600; break;
      case 19200: baud = B19200; break;
      case 38400: baud = B38400; break;
      case 56700: baud = B57600; break;
      case 115200: baud = B115200; break;      
      default: baud = B115200; break;
  }
  int databits = CS8;
  int stopbits = 0;
  int parity = 0;
  int protocol = 0;
  newtermios.c_cflag = baud | stopbits | parity | databits | CLOCAL | CREAD | protocol;
  newtermios.c_cc[VMIN] = 1;
  tcflush(_stream, TCIOFLUSH);
  tcsetattr(_stream, TCSANOW, &newtermios);
  tcflush(_stream, TCIOFLUSH);
  fcntl(_stream, F_SETFL, O_NONBLOCK);
  return true;
}

bool LinuxSerial::open(const char *devicePath){    
  //return false;
  struct termios newtermios;
  devPath = devicePath;
  ::printf("opening serial port %s...\n", devicePath);
  if ((_stream = ::open(devicePath, O_RDWR | O_NOCTTY | O_NONBLOCK)) <= 0)
  { 
    ::printf("could not open serial port %s\n", devicePath);
    return false;
  }
  #ifdef LINUX_SERIAL_FIFO
    if (thread_rx_id == 0){    
      //::printf("LINUX_SERIAL_FIFO: starting serial threads...");
      pthread_create(&thread_rx_id, NULL, linuxSerialRxThreadFun, (void*)this);	
      pthread_create(&thread_tx_id, NULL, linuxSerialTxThreadFun, (void*)this);	
    }
  #endif
  return true;
}


void LinuxSerial::end(){
  if(_stream)
  {
    /* reset old settings */
    ::printf("closing serial port %s...\n", devPath.c_str());
    tcsetattr(_stream, TCSANOW, &_termios);
    close(_stream);
    _stream = 0;
  }
}


int LinuxSerial::peek(){
    return 0;
}


void LinuxSerial::flush(){
    //console_flush();
}


// RX FIFO  
bool LinuxSerial::runRx(){
	if(!_stream) return false;  
  //if (fifoTx.available() != 0) return true; // give TX FIFO highest priority for USB communication 

  // ----------- read from serial into RX FIFO ---------------
	while (true){    
    int bytes_avail = 0;
    ioctl(_stream, FIONREAD, &bytes_avail);
    if (bytes_avail == 0) break;

    if (bytes_avail > 4095) bytes_avail = 4095;
    byte buffer[4095];    
    size_t size = bytes_avail;
    int j = ::read(_stream, &buffer, size);    
      //if(j < 0){
    //  if(errno == EAGAIN) {
    //     break;
    if (j != bytes_avail){
			fprintf(stderr, "LINUX SERIAL: error reading\n");      
      break;
    }    
		for (int i=0; i < bytes_avail; i++){
      if (!fifoRx.write(buffer[i])){
        // fifoRx overflow
        fprintf(stderr, "LINUX SERIAL: FIFO RX overflow\n");
        break;
      }
      frameCounterRx++;
    }
		//delayMicroseconds(500);
  }
  return true;
}

// TX FIFO 
bool LinuxSerial::runTx(){
	if(!_stream) return false;    
  if (fifoTx.available() == 0) return true;

  // ---------- write to serial from TX FIFO ---------------
	//frame.secs = tv.tv_sec;
	//frame.usecs = tv.tv_usec;
	//byte data;
	byte buffer[4095];    
  
  int count = 0; 
  while (fifoTx.available() != 0){
    byte data;
    fifoTx.read(data);
    buffer[count] = data;  
    count ++;
    if (count == 4095) break;                
    if (fifoTx.available() == 0) delay(1); // add timeout
  }
  
  //fprintf(stderr, "LINUX SERIAL: sending %d\n", count);
  int j = ::write(_stream, buffer, count);    
  if(j < 0)
  {
      if(errno == EAGAIN) {
        perror("LINUX SERIAL: ERROR writing LinuxSerial");
      }
      //delayMicroseconds(500);    
	}
	return true;
}




#ifdef LINUX_SERIAL_FIFO  // ---------------------------------------------------------------


int LinuxSerial::available(){
  if(!_stream) return 0;  
  return (fifoRx.available()); 
}

int LinuxSerial::read(){
	byte data;
	if (!fifoRx.read(data)) return 0;
    
  return data;
}


size_t LinuxSerial::write(uint8_t c){
  if(!_stream) return 0;      
	
    if (!fifoTx.write(c)){
		// fifoTx overflow
		fprintf(stderr, "LINUX SERIAL: FIFO TX overflow\n");
		return 0;
	}
	frameCounterTx++;
  return 1;
}

    
size_t LinuxSerial::write(const uint8_t *buffer, size_t size){
  if(!_stream) return 0;      
  while (fifoTx.available() != 0); // wait until last packet was sent  

  int count = 0;
  for (int i=0; i < size; i++){
    int c = write(buffer[i]);
    if (c == 0) break;
    count++;
  }
  return count;
}





#else  // ---------------------------------------- no LINUX_SERIAL_FIFO ---------------------------------------

int LinuxSerial::available(){
  int bytes_avail = 0;
  ioctl(_stream, FIONREAD, &bytes_avail);
  return bytes_avail;
}


int LinuxSerial::read(){
  char buffer = 0;
  size_t size = 1;
  int j = ::read(_stream, &buffer, size);
  if(j < 0)
  {
    if(errno == EAGAIN)
      return 0;
    else
      return buffer;
  }
  return buffer;
}


size_t LinuxSerial::write(uint8_t c){
  size_t size = 1;
  char *buffer = (char*)&c;
  int j = ::write(_stream, buffer, size);    
  if(j < 0)
  {
      if(errno == EAGAIN)
        return 0;
      else
        return j;
  }
  return j;
}

  
size_t LinuxSerial::write(const uint8_t *buffer, size_t size){
  int j = ::write(_stream, buffer, size);    
  if(j < 0)
  {
      if(errno == EAGAIN)
        return 0;
      else
        return j;
  }
  return j;
}


#endif // --------------------------------------------------------------------------------


