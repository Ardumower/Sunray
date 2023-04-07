/*
  Arduino.c -  Partial implementation of the Wiring API for the Raspberry Pi
  Copyright (c) 2015 Hristo Gochkov.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "Arduino.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/stat.h>


int pwm_export(int pwmio)
{
	int efd;
	char buf[50];
	int pwmfd, ret;

	/* Quick test if it has already been exported */
	sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/enable", pwmio);
	efd = open(buf, O_WRONLY);
	if(efd != -1) {
		// already exported
		close(efd);
		return 0;
	}

	efd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY | O_SYNC);

	if(efd != -1) {
		sprintf(buf, "%d", pwmio); 
		ret = write(efd, buf, strlen(buf));
		if(ret < 0) {
			perror("error: PWM export failed");
			return -2;
		}
		close(efd);
	} else {
		// If we can't open the export file, we probably
		// dont have any pwmio permissions
		perror("error: no PWM permission");
		return -1;
	}
	return 0;
}


void pwm_unexport(int pwmio)
{
	int pwmfd, ret;
	char buf[50];
	pwmfd = open("/sys/class/pwm/pwmchip0/unexport", O_WRONLY | O_SYNC);
	sprintf(buf, "%d", pwmio);
	ret = write(pwmfd, buf, strlen(buf));
	close(pwmfd);
}

int pwm_getfd(int pwmio)
{
	char in[3] = {0, 0, 0};
	char buf[50];
	int pwmfd;
	sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwmio);
	pwmfd = open(buf, O_RDWR | O_SYNC);
	if(pwmfd < 0) {
		fprintf(stderr, "Failed to open pwm %d value\n", pwmio);
		perror("pwm failed");
	}

	return pwmfd;
}



void analogWriteRange(uint32_t range){
}

void analogWriteDiv(uint16_t div){
}

uint32_t analogWriteSetup(uint32_t frequency, uint32_t range){
    return 0;
}

void analogWriteInit(){
}

//  https://www.linkedin.com/pulse/pwm-linux-user-space-vahid-gharaee/
void analogWrite(uint8_t pwm, uint16_t val){
	char buf[50];
	int nread, ret, pwmfd;
	pwm_export(pwm);
  
  int period = 10000000;  // 10 ms pulse width
  int duty = ((float)val) / 1023.0 * period;    

  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/period", pwm);
	pwmfd = open(buf, O_WRONLY);
	if(pwmfd < 0) {
      perror("error opening period");
      return;  
  }
  snprintf(buf, 8, "%d", period);
  ret = write(pwmfd, buf, 8);
  if (ret < 0){  
    perror("failed to set period");
    close(pwmfd);
    return;
  }
  close(pwmfd);

  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwm);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0) {
    perror("error opening duty_cycle");
    return;
  }
  snprintf(buf, 8, "%d", duty);
  ret = write(pwmfd, buf, 8);
  if(ret < 0) {
      perror("failed to set duty_cylce");
      close(pwmfd);
      return;
  }
  close(pwmfd);  	  

  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/enable", pwm);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0) {
    perror("error opening enable");
    return;
  }
  snprintf(buf, 2, "%d", 1);
  ret = write(pwmfd, buf, 1);
  if(ret < 0) {
      perror("failed to set enable");
      close(pwmfd);
      return;
  }
  close(pwmfd);  	  

}




