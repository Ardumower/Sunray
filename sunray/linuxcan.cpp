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



void *canThreadFun(void *user_data)
{
    LinuxCAN *can = (LinuxCAN*)user_data;
	while (true){
	  can->run();
      //usleep(300);
	}
	return NULL;
}



LinuxCAN::LinuxCAN(){
  sock = -1;
}

bool LinuxCAN::begin(){  
   	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("ERROR starting CAN socket");
		return false;
	}

  	struct ifreq ifr;
  	strcpy(ifr.ifr_name, "can0" );
  	ioctl(sock, SIOCGIFINDEX, &ifr);

  	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("ERROR binding CAN socket");
		return false;
	}

	Serial.println("linuxcan: server listening");
  	pthread_create(&thread_id, NULL, canThreadFun, (void*)this);
	return true;
}

bool LinuxCAN::available(){
	return (fifoRxStart != fifoRxEnd); 
}

bool LinuxCAN::read(can_frame_t &frame){	
	if (fifoRxStart == fifoRxEnd) return false;
	//printf("can: rx_frames=%d\n", frameCounterRx);

	frame.idx = fifoRx[fifoRxStart].idx;
	frame.secs = fifoRx[fifoRxStart].secs;
	frame.usecs = fifoRx[fifoRxStart].usecs;
	
	frame.can_id = fifoRx[fifoRxStart].can_id;
	frame.can_dlc = fifoRx[fifoRxStart].can_dlc;
	for (int i=0; i < sizeof(frame.data); i++) frame.data[i] = fifoRx[fifoRxStart].data[i];
	if (fifoRxStart == CAN_FIFO_FRAMES_RX-1) fifoRxStart = 0; 
	  else fifoRxStart++;

	#if defined(CAN_DEBUG)
		printf("frame %d, secs: %d, usecs: %d, CAN: 0x%03X [%d] ", frame.idx, frame.secs, frame.usecs, frame.can_id, frame.can_dlc);
		for (int i = 0; i < frame.can_dlc; i++)
			printf("%02X ",frame.data[i]);
		printf("\r\n");
	#endif

	return true;
}


bool LinuxCAN::run(){
	struct can_frame frame;	
	int nbytes = ::read(sock, &frame, sizeof(struct can_frame));		
	
	if (nbytes <= 0) {
		//perror("ERROR reading CAN socket");
		return false;
	}
	struct timeval tv;
	ioctl(sock, SIOCGSTAMP_KERNEL, &tv);
	

	int nextFifoRxEnd = fifoRxEnd;
	if (nextFifoRxEnd == CAN_FIFO_FRAMES_RX-1) nextFifoRxEnd = 0; 
	  else nextFifoRxEnd++;
	
	if (nextFifoRxEnd != fifoRxStart){
		// no fifoRx overflow 
		fifoRx[fifoRxEnd].idx = frameCounterRx;
		fifoRx[fifoRxEnd].secs =tv.tv_sec;
		fifoRx[fifoRxEnd].usecs = tv.tv_usec;

		fifoRx[fifoRxEnd].can_id = frame.can_id;
		fifoRx[fifoRxEnd].can_dlc = frame.can_dlc;
		for (int i=0; i < sizeof(frame.data); i++) fifoRx[fifoRxEnd].data[i] = frame.data[i]; 
		fifoRxEnd = nextFifoRxEnd;
	} else {
		// fifoRx overflow
		fprintf(stderr, "CAN: FIFO RX overflow\n");
		fifoRxEnd = fifoRxStart;
	}

	frameCounterRx++;
	
	return true;
}

bool LinuxCAN::write(can_frame_t frame){
  	struct can_frame fr; 
	fr.can_id = frame.can_id;
	fr.can_dlc = frame.can_dlc;
	for (int i=0; i < 8; i++) fr.data[i] = frame.data[i]; 
	if (::write(sock, &fr, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("ERROR writing CAN socket");
		return false;
	} else {
		frameCounterTx++;
	}
  	return true;
}


bool LinuxCAN::close(){
	if (::close(sock) < 0) {
			perror("ERROR closing CAN socket");
			return false;
	}
	return true;
}



#endif

