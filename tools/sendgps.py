# sends ublox GPS packets (recorded .ubx files) to Arduino 
# to be run with Anaconda/Miniconda Python 3.7

dosend = True
# dosend = False

from time import sleep
import serial

if dosend:
  ser = serial.Serial("COM21", 115200, timeout=0.5)

with open("d:/temp/COM25_200902_084412.ubx", "rb") as f:
    
    if dosend:
      sleep(5)
    idx = 0    
    
    while True:
      byte = f.read(1)
      if not byte:
        break
      if (byte == b'\xb5'):
        print('\n\n', end='', flush=True)
        sleep(0.2)
      print(byte.hex()+',', end='', flush=True)              
      
      if dosend: 
        ser.write(byte)        
        
      idx = idx + 1
      #if (idx % 64 == 0):
      #  sleep(5)
        
        



