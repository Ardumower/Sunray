#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Arduino.h"
#include "Console.h"


bool LinuxConsole::begin(){    
    printf("LinuxConsole::begin\n");
    printf("opening TTY...\n");
    keyboard = 0;
    //keyboard = open("/dev/tty",O_RDONLY|O_NONBLOCK);    
    printf("TTY=%d\n", keyboard);    
    return true;
}

bool LinuxConsole::begin(int baudrate){
    return begin();
}    

void LinuxConsole::end(){
}

int LinuxConsole::available(){
    //return 0;     
    //if (keyboard <= 0) return 0;
    struct timeval timeout;
    fd_set readfd;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    FD_ZERO(&readfd);
    FD_SET(keyboard, &readfd);
    int ret = select(keyboard+1, &readfd, NULL, NULL, &timeout);
    return (ret > 0);    
    //return (FD_ISSET(keyboard, &readfd));
}

int LinuxConsole::peek(){
    return 0; 
    //if(idemonitor_connected() && idemonitor_available())
    //    return idemonitor_peek();
    //return console_peek();
}

int LinuxConsole::read(){
    //return 0;
    //return getchar();          
    //if (keyboard <= 0) return 0;
    char ch = '\0';
    ::read(keyboard, &ch, 1);
    return ch;
}

void LinuxConsole::flush(){
    //console_flush();
}

size_t LinuxConsole::write(uint8_t c){
    //if(idemonitor_connected())
    //    idemonitor_write((char*)&c, 1);
    //console_write(c);
    ::printf("%c",c);
    return 1;
}


LinuxConsole Console;



