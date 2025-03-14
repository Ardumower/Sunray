// Ardumower Sunray 

// Linux CAN bus class (with FIFO)

// docs:  https://www.kernel.org/doc/html/next/networking/can.html


#ifdef __linux__

#include "linuxcan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>


#include <Arduino.h>


//#define CAN_DEBUG 1


// https://github.com/spotify/linux/blob/master/include/asm-generic/sockios.h
#define SIOCGSTAMP_KERNEL	0x8906		/* Get stamp (timeval) */



void *canRxThreadFun(void *user_data)
{
    LinuxCAN *can = (LinuxCAN*)user_data;
	while (true){
	  can->runRx();
      delayMicroseconds(500);
	}
	return NULL;
}


void *canTxThreadFun(void *user_data)
{
    LinuxCAN *can = (LinuxCAN*)user_data;
	while (true){
	  can->runTx();
      delayMicroseconds(500);
	}
	return NULL;
}



LinuxCAN::LinuxCAN(){
  sock = -1;
  frameCounterRx = 0;
  frameCounterTx = 0;
}

bool LinuxCAN::begin(){  
   	/*#ifndef __arm__
		sock = -1;
		Serial.println("ERROR starting CAN socket (disabled on non-ARM systems)");
		return false;
	#endif*/ 
	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("ERROR starting CAN socket");
		return false;
	}

  	struct ifreq ifr;
  	strcpy(ifr.ifr_name, "slcan0" );
  	if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0){
		perror("cannot open slcan0...");
	  	strcpy(ifr.ifr_name, "can0" );
  		if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0){
			perror("cannot open can0...");	  	
		}
	}

  	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("ERROR binding CAN socket");
		sock = -1;
		return false;
	}

	Serial.println("linuxcan: server listening");
  	pthread_create(&thread_rx_id, NULL, canRxThreadFun, (void*)this);	
  	pthread_create(&thread_tx_id, NULL, canTxThreadFun, (void*)this);	
	return true;
}

bool LinuxCAN::available(){
	return (fifoRx.available()); 
}

bool LinuxCAN::read(can_frame_t &frame){	
	can_frame_t fframe;
	if (!fifoRx.read(fframe)) return false;
    
	//printf("can: rx_frames=%d\n", frameCounterRx);

	frame.idx = fframe.idx;
	frame.secs = fframe.secs;
	frame.usecs = fframe.usecs;
	
	frame.can_id = fframe.can_id;
	frame.can_dlc = fframe.can_dlc;
	for (int i=0; i < sizeof(frame.data); i++) frame.data[i] = fframe.data[i];

	#if defined(CAN_DEBUG)
		printf("frame %d, secs: %d, usecs: %d, CAN: 0x%03X [%d] ", frame.idx, frame.secs, frame.usecs, frame.can_id, frame.can_dlc);
		for (int i = 0; i < frame.can_dlc; i++)
			printf("%02X ",frame.data[i]);
		printf("\r\n");
	#endif

	return true;
}


bool LinuxCAN::runRx(){
	if (sock < 0) return false;
	struct can_frame frame;		
	// ----------- read from CAN bus into RX FIFO ---------------
	while (true){
		int nbytes = ::read(sock, &frame, sizeof(struct can_frame));		
		
		if (nbytes == 0) return true;
		struct timeval tv;
		ioctl(sock, SIOCGSTAMP_KERNEL, &tv);
		
		can_frame_t fframe;
		fframe.idx = frameCounterRx;
		fframe.secs = tv.tv_sec;
		fframe.usecs = tv.tv_usec;
		fframe.can_id = frame.can_id;
		fframe.can_dlc = frame.can_dlc;
		for (int i=0; i < sizeof(fframe.data); i++) fframe.data[i] = frame.data[i]; 
		if (!fifoRx.write(fframe)){
			// fifoRx overflow
			fprintf(stderr, "CAN: FIFO RX overflow\n");
		}
		frameCounterRx++;
	}
	return true;
}

bool LinuxCAN::runTx(){
	if (sock < 0) return false;
	// ---------- write to CAN bus from TX FIFO ---------------
	//frame.secs = tv.tv_sec;
	//frame.usecs = tv.tv_usec;
	can_frame_t fframe;
	while (fifoTx.read(fframe)){
		struct can_frame fr; 
		fr.can_id = fframe.can_id;
		fr.can_dlc = fframe.can_dlc;
		for (int i=0; i < 8; i++) fr.data[i] = fframe.data[i]; 		
		if (::write(sock, &fr, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
			perror("ERROR writing CAN socket");
			break;			
		} 		
		delayMicroseconds(500);
	}
	return true;
}


bool LinuxCAN::write(can_frame_t frame){
  	if (sock < 0) return false; 
	//Serial.println("LinuxCAN::write");
	
	frame.idx = frameCounterTx;
	//frame.secs = tv.tv_sec;
	//frame.usecs = tv.tv_usec;
    if (!fifoTx.write(frame)){
		// fifoRx overflow
		fprintf(stderr, "CAN: FIFO TX overflow\n");
		return false;
	}
	frameCounterTx++;
  	return true;
}


bool LinuxCAN::close(){
	if (sock < 0) return false;
	if (::close(sock) < 0) {
			perror("ERROR closing CAN socket");
			return false;
	}
	return true;
}



#endif

