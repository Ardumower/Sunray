#!/usr/bin/env python
# -*- coding: utf-8 -*-

# UDP server - receives UDP packets from Arduino
# to be run with Anaconda/Miniconda Python 3.7

import socket

host = ""
port = 4210

bufsize = 99

addr = (host, port)
UDPSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDPSock.bind(addr)
UDPSock.setblocking(0)

print("waiting for messages (port: " + str(port) + ") ...")

# Endlosschleife fuer Empfang, Abbruch durch Strg-C
try:
  while True:
    try: 
      (data, addr) = UDPSock.recvfrom(bufsize)
    except socket.error:
      pass
    else:   
      print (data.decode('utf-8'), end='', flush=True)      
except KeyboardInterrupt:
  UDPSock.close()
  print ("terminating ...")
  exit(0)
  
