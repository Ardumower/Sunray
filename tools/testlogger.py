#!/usr/bin/env python
# -*- coding: utf-8 -*-

# UDP test client - sends UDP packets
# to be run with Anaconda/Miniconda Python 3.7

import socket
import time

host = "192.168.2.30"
port = 4210

message = b'hello world\r\n'

addr = (host, port)
UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Endlosschleife fuer Empfang, Abbruch durch Strg-C
try:
  while True:
    UDPSock.sendto(message, addr)    
    time.sleep(2.0)
    
except KeyboardInterrupt:
  UDPSock.close()
  print ("terminating ...")
  exit(0)

