#!/bin/bash

# example custom script  

# ---serial tcp bridge----

# physical serial port path  ( find out with:  ls /dev/serial/by-id/* )
#SERIAL_PATH=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_9C:9E:6E:C3:16:DC-if00
SERIAL_PATH=/dev/ttyACM0
TCP_PORT=12345

VIRTUAL_TTY0="/dev/ttyV0"
VIRTUAL_TTY1="/dev/ttyV1"


##   serial ----------------------------------------------------tcp  ----+------- tcp-broker
##                                                                       |
##   virtual serial ----------------   virtual serial  -------- tcp  ----|
##                                                                       |
##                                                              tcp  ----+


# tcp broker
echo "=======tcp-broker"
ncat --broker --listen -p $TCP_PORT  &
SOCAT1_PID=$!
sleep 1

# tcp zum serial
echo "=======tcp to serial"
socat -d -d -lpA_tcp_to_serial   TCP:localhost:$TCP_PORT    $SERIAL_PATH,raw,echo=0  &
SOCAT2_PID=$!
sleep 1

# virtual serial line (mit endpunkten ttyV0 und ttyV1)
echo "=======virtual serial line"
socat -d -d -lpA_virtual_virtual   PTY,link=$VIRTUAL_TTY0,raw,echo=0   PTY,link=$VIRTUAL_TTY1,raw,echo=0  &
SOCAT3_PID=$!
sleep 1

# tcp zum virtual serial  
echo "=======tcp to virtual serial"
socat -d -d -lpC_tcp_to_virtual_serial   TCP:localhost:$TCP_PORT   $VIRTUAL_TTY0  &
SOCAT4_PID=$!


# Fange das SIGINT-Signal (CTRL+C) ab und beende beide socat-Prozesse
trap "kill $SOCAT1_PID $SOCAT2_PID $SOCAT3_PID $SOCAT4_PID; exit" INT

# Warte, bis einer der Prozesse beendet wird (oder auf CTRL+C)
#wait


# -----test commands-----------------------------

# serial:
#   sudo cat /dev/ttyV1
#   screen /dev/ttyUSB0
#   killall screen

# tcp:
#   sudo lsof | grep LISTEN
#   nc localhost 12345


