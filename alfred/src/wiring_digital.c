/*
  Arduino.c -  Partial implementation of the Wiring API for Linux
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


//  https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
//  https://github.com/embeddedarm/ts4900-utils/blob/master/src/gpiolib.c

int gpio_export(int gpio)
{
	int efd;
	char buf[50];
	int gpiofd, ret;

	/* Quick test if it has already been exported */
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	efd = open(buf, O_WRONLY);
	if(efd != -1) {
		// already exported
		close(efd);
		return 0;
	}

	efd = open("/sys/class/gpio/export", O_WRONLY | O_SYNC);

	if(efd != -1) {
		sprintf(buf, "%d", gpio); 
		ret = write(efd, buf, strlen(buf));
		if(ret < 0) {
			perror("error: GPIO export failed");
			return -2;
		}
		close(efd);
	} else {
		// If we can't open the export file, we probably
		// dont have any gpio permissions
		perror("error: no GPIO permission");
		return -1;
	}
	return 0;
}


void gpio_unexport(int gpio)
{
	int gpiofd, ret;
	char buf[50];
	gpiofd = open("/sys/class/gpio/unexport", O_WRONLY | O_SYNC);
	sprintf(buf, "%d", gpio);
	ret = write(gpiofd, buf, strlen(buf));
	close(gpiofd);
}

int gpio_getfd(int gpio)
{
	char in[3] = {0, 0, 0};
	char buf[50];
	int gpiofd;
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	gpiofd = open(buf, O_RDWR | O_SYNC);
	if(gpiofd < 0) {
		fprintf(stderr, "Failed to open gpio %d value\n", gpio);
		perror("gpio failed");
	}

	return gpiofd;
}


int gpio_setedge(int gpio, int rising, int falling)
{
	int ret = 0;
	char buf[50];
	sprintf(buf, "/sys/class/gpio/gpio%d/edge", gpio);
	int gpiofd = open(buf, O_WRONLY);
	if(gpiofd < 0) {
		perror("Couldn't open IRQ file");
		ret = -1;
	}

	if(gpiofd && rising && falling) {
		if(4 != write(gpiofd, "both", 4)) {
			perror("Failed to set IRQ to both falling & rising");
			ret = -2;
		}
	} else {
		if(rising && gpiofd) {
			if(6 != write(gpiofd, "rising", 6)) {
				perror("Failed to set IRQ to rising");
				ret = -2;
			}
		} else if(falling && gpiofd) {
			if(7 != write(gpiofd, "falling", 7)) {
				perror("Failed to set IRQ to falling");
				ret = -3;
			}
		}
	}

	close(gpiofd);
}


// ---------------------------------------------------

void pinMode(uint8_t gpio, uint8_t mode){
  printf("skipping pinMode: %d\n", gpio);
  return;
  gpio_export(gpio);
  int ret = 0;
	char buf[50];
	sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
	int gpiofd = open(buf, O_WRONLY | O_SYNC);
	if(gpiofd < 0) {
		perror("Couldn't open direction file");
		ret = -1;
	}
	if((mode == OUTPUT) && (gpiofd)){
		if (4 != write(gpiofd, "out", 4)) {
			fprintf(stderr, "Couldn't set GPIO %d direction to out: %s\n", 
				gpio,
				strerror(errno));
			ret = -2;
		}
	}  else if(gpiofd) {
		if(2 != write(gpiofd, "in", 2)) {
			fprintf(stderr, "Couldn't set GPIO %d direction to in: %s\n", 
				gpio,
				strerror(errno));
			ret = -3;
		}
	}

	close(gpiofd);
	//return ret;
}

void digitalWrite(uint8_t gpio, uint8_t val){
  	//printf("skipping digitalWrite: %d\n", gpio);
	//return;
	char buf[50];
	int nread, ret, gpiofd;
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	gpiofd = open(buf, O_RDWR);
	if(gpiofd > 0) {
		snprintf(buf, 2, "%d", val ? 1 : 0);
		ret = write(gpiofd, buf, 2);
		if(ret < 0) {
			perror("failed to set gpio");
			return;
      		//return 1;
	 	 }
		close(gpiofd);
		//if(ret == 2) return 0;
	}
	//return 1;
}

int digitalRead(uint8_t gpio){
  	//printf("skipping digitalRead: %d\n", gpio);
	//return 0;	
  	char in[3] = {0, 0, 0};
	char buf[50];
	int nread, gpiofd;
	sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
	gpiofd = open(buf, O_RDWR | O_SYNC);
	if(gpiofd < 0) {
		fprintf(stderr, "Failed to open gpio %d value\n", gpio);
		perror("gpio failed");
	}

	do {
		nread = read(gpiofd, in, 1);
	} while (nread == 0);
	if(nread == -1){
		perror("GPIO Read failed");
		return -1;
	}
	
	close(gpiofd);
	return atoi(in);
}

