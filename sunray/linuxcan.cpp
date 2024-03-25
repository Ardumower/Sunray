// Ardumower Sunray 

// Linux CAN bus 


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
  return true;
}


bool LinuxCAN::read(can_frame_t &frame){	
	struct can_frame fr;
	int nbytes = ::read(sock, &fr, sizeof(struct can_frame));

	if (nbytes < 0) {
		perror("ERROR reading CAN socket");
		return false;
	}
	//printf("nbytes=%d\n", nbytes);

	//printf("CAN: 0x%03X [%d] ",fr.can_id, fr.can_dlc);
	//for (int i = 0; i < fr.can_dlc; i++)
	//	printf("%02X ",fr.data[i]);
	//printf("\r\n");

	frame.can_id = fr.can_id;
	frame.can_dlc = fr.can_dlc;
	for (int i=0; i < 8; i++) frame.data[i] = fr.data[i]; 

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

