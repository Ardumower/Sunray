/*
  SPI Master library for Linux
*/

//#include "pins_arduino.h"
#include "SPI.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/stat.h>

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>


int SPIClass::fd = 0;
uint8_t SPIClass::mode=0; 
uint32_t SPIClass::speed=0;
uint16_t SPIClass::delay=0;
uint8_t SPIClass::bits=0;               


void SPIClass::begin() {
  //pinMode(9, ALT0);
  //pinMode(10, ALT0);
  //pinMode(11, ALT0);

  char filename[50];
  int busAddressMajor = 0;
  int busAddressMinor = 0;
  
	sprintf(filename, "/dev/spidev%d.%d", busAddressMajor, busAddressMinor);

  delay = 0;             //  SPI driver latency: https://www.raspberrypi.org/forums/viewtopic.php?f=44&t=19489
  bits = 8;

  //  Previously: parse_opts(argc, argv);
  setDataMode(0);
  setClock(500000);
  printf("spi mode: %d\n", mode);
  printf("bits per word: %d\n", bits);
  printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

    //  Open SPI device.
  fd = open(filename, O_RDWR);
  if (fd < 0) { 
    perror("error: can't open SPI bus"); 
    return;
  }

  //  Set SPI read and write bits per word.
  int ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
  if (ret == -1) { perror("can't set bits per word"); }
  ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
  if (ret == -1) { perror("can't get bits per word"); }
}

void SPIClass::end() {
  //pinMode(9, INPUT);
  //pinMode(10, INPUT);
  //pinMode(11, INPUT);
}

void SPIClass::setDataMode(uint32_t amode){
  mode = amode;
  //  Set SPI mode to read and write.
  int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
  if (ret == -1) { perror("can't set spi mode"); }
  ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
  if (ret == -1) { perror("can't get spi mode"); }
}

void SPIClass::setClockDivider(uint32_t rate){
}

void SPIClass::setClock(uint32_t rate){
  speed = rate; 
  //  Set SPI read and write max speed.
  int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  if (ret == -1) { perror("can't set max speed hz"); }
  ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
  if (ret == -1) { perror("can't get max speed hz"); }
}

uint8_t SPIClass::transfer(uint8_t data) {

  byte res = 0;

	struct spi_ioc_transfer tr;
  tr.tx_buf = (unsigned long) &data;
	tr.rx_buf =  (unsigned long) NULL;
	tr.len = 1;
  tr.delay_usecs = delay;
  tr.speed_hz = speed;
  tr.bits_per_word = bits;
	
	int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    //  Check SPI result.
	if (ret < 1) { perror("spi_transmit failed"); }

  tr.tx_buf = (unsigned long) NULL;
	tr.rx_buf =  (unsigned long) &res;	  
  tr.len = 1;
  tr.delay_usecs = delay;
  tr.speed_hz = speed;
  tr.bits_per_word = bits;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    //  Check SPI result.
	if (ret < 1) { perror("spi_receive failed"); }
  
  return res;
}


void SPIClass::beginTransaction(SPISettings settings){
  setDataMode(settings.mode);
  setClock(settings.freq);
}

SPIClass SPI;

