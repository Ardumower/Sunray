# robot HTTP client

import socket
import requests
import time


ROBOT_CON_PASS = "123456" # robot connection password
#ROBOT_HOST = "192.168.2.68" # some remote machine
ROBOT_HOST = "127.0.0.1"  # local host
ROBOT_PORT = 80  # The port used by the robot HTTP server
URL = 'http://' + ROBOT_HOST + ':' + str(ROBOT_PORT)
DEBUG = False

GPS_INVALID = 0
GPS_FLOAT   = 1
GPS_FIX     = 2



class RobotHttpClient():
    def __init__(self):
        self.robotVoltage = 0
        self.robotX = 0
        self.robotY = 0
        self.robotGpsSol = 0
        self.robotState = 0
        self.robotObstacles = 0
        self.robotSonarCounter = 0
        self.robotBumperCounter = 0
        self.robotGPSNoMotionCounter = 0

        self.encMode = 0
        self.encChlg = 0
        self.lastRequestTime = time.time()

    def encrypt(self, msg):
        encryptPass = int(ROBOT_CON_PASS)
        encryptKey = encryptPass % self.encChlg
        res = ''
        for ch in msg:
            code = ord(ch)
            if ((code >=32) and (code <= 126)):   # ASCII between 32..126
                code = code + encryptKey
                if (code >= 127): code = 32 + (code - 127)  
            res += chr(code)
        return res

    def addCRC(self, msg):
        crc = 0
        # print(s.encode())    
        for ch in msg:
            crc = (crc + ord(ch)) & 255
        msg += "," + hex(crc)
        return msg

    def sendRobot(self, msg):
        self.lastRequestTime = time.time()
        if DEBUG: print('sending ',msg)
        #print(URL + '  -->   '  + msg)        
        msg = self.addCRC(msg)
        if (self.encMode == 1): msg = self.encrypt(msg)    
        headers = {'Content-type': 'text/plain'}    
        resp = requests.post(url = URL, data = msg + '\r\n', headers=headers, timeout=5)
        if DEBUG: print("Received " + str(resp.text))        
        arr = resp.text.split(',')
        return arr

    def requestVersion(self):    
        self.encMode = 0
        try:
            arr = self.sendRobot('AT+V')
            if len(arr) < 5:
                print('ERROR: invalid answer from robot (requestVersion)')
                return False
            self.encMode = int(arr[3])
            self.encChlg = int(arr[4])
            #print(encMode, encChlg)
            return True
        except Exception as e:
            print("Error requesting robot version: " + str(e))
            return False

    def requestSummary(self):
        if self.encChlg == 0:
            self.requestVersion()
        if self.encChlg == 0:            
            return
        try:
            arr = self.sendRobot('AT+S')
            if len(arr) < 7:
                print('ERROR invalid answer from robot (requestSummary)')
                return False            
            #print(arr)
            self.robotVoltage = float(arr[1])
            self.robotX = float(arr[2])
            self.robotY = float(arr[3])
            self.robotGpsSol = int(arr[5])
            self.robotState = int(arr[6])    
            #print('volt', robotVoltage, 'x', robotX, 'y', robotY, 'gps', robotGpsSol, 'state', robotState)
            return True
        except Exception as e:
            print("Error requesting robot summary: " + str(e))
            return False        

    def requestStatistics(self):
        if self.encChlg == 0:
            self.requestVersion()
        if self.encChlg == 0:            
            return
        try:
            arr = self.sendRobot('AT+T')
            if len(arr) < 7:
                print('ERROR invalid answer from robot (requestStatistics)')
                return False            
            #print(arr)
            self.robotObstacles = int(arr[18])
            self.robotSonarCounter = int(arr[22])            
            self.robotBumperCounter = int(arr[23])
            self.robotGPSNoMotionCounter = int(arr[24])  
            return True
        except Exception as e:
            print("Error requesting robot summary: " + str(e))
            return False        


if __name__ == "__main__":
    robot = RobotHttpClient()
    resp = robot.requestVersion()    
    if resp:
        resp = robot.requestSummary()
        print('volt', robot.robotVoltage, 'x', robot.robotX, 'y', robot.robotY, 'gps', robot.robotGpsSol, 'state', robot.robotState)
        resp = robot.requestStatistics()
        print('obst', robot.robotObstacles, 'sonar', robot.robotSonarCounter, 'bump', robot.robotBumperCounter, 'gps', robot.robotGPSNoMotionCounter)
            
        