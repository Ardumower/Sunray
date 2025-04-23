# log wifi signal quality/ping statistics
# 
# https://forum.ardumower.de/threads/advanced-topic-generate-wifi-gps-heatmaps-with-sunray-on-alfred-or-ardumower-with-connected-raspberry-pi.25078/

import robot_http_client
import net_tools
import time
import datetime
import random
import math

SIM = False
LOG = 'wifi_signal_log.txt'

lastPrintTime = 0
lastAP = ""
lastRobotX = 0
lastRobotY = 0

simGoRight = True
simSignal = -90
simSignalUp = True
simPing = 0
simPingUp = True

robot = robot_http_client.RobotHttpClient()
nt = net_tools.NetTools()


def addFile(line):
    file=open(LOG,"a+")        
    file.write(line)
    file.close()



print("press CTRL+C to exit script at any time")

nt.resetStats()
lastPrintTime = time.time()
        
nt.findGatewayAddr()
#gatewayAddr = '25.8.8.8'
#readInterfaceStats()
counter = 0 

dt = datetime.datetime.now()
ts = dt.strftime('%y-%m-%d_%H_%M_%S')
addFile('# -------------------------------------------------------------------------------------------\n')

nt.findWifiInterface()

line = '# robot: %s  interface: %s   gateway: %s   ' % (robot_http_client.ROBOT_HOST, nt.interface, nt.gatewayAddr) 
print(line)
addFile(line + '\n')

line = '# scanning network for network devices...'
print(line)
addFile(line + '\n')

lines = nt.detectNetDevices()
for line in lines:
    line = '# ' + line 
    print(line)
    addFile(line + '\n')    

line = '# measuring network device signal...'
print(line)
addFile(line + '\n')



while (True): 
    if SIM:
        # simulation    
        if simPingUp:
            simPing +=0.1
            if simPing > 10: simPingUp = False 
        else:
            simPing -=0.1
            if simPing < 1: simPingUp = True

        if simSignalUp:
            simSignal += 1
            if simSignal > -30: simSignalUp = False            
        else:
            simSignal -= 1        
            if simSignal < -90: simSignalUp = True
        nt.signalLevel = (100.0 - 2.0*robot.robotX**2 - 2.0*robot.robotY**2) / 100.0 * -60.0 -30.0 
        nt.pingTime = simPing

        if simGoRight: 
            robot.robotX += 0.25 + random.random() / 100.0        
            if robot.robotX > 10: 
                robot.robotY += 0.25 + random.random() / 100.0
                simGoRight = False
        else:
            robot.robotX -= 0.25 - random.random() / 100.0
            if robot.robotX < -10:
                robot.robotY += 0.25 + random.random() / 100.0
                simGoRight = True
        if robot.robotY > 10:
            robot.robotY = 0    
    else:
        nt.detectLinkQuality()
        nt.pingGateway()    
        if time.time() > robot.lastRequestTime + 3.0:
            if not robot.requestSummary():
                line = '# error reading robot summary'
                print(line)
                addFile(line + '\n')

    nt.computeStats()

    #mac = nt.getMAC(interface)
    ap = nt.getAccessPointMAC()

    distTraveled = math.sqrt( (robot.robotX-lastRobotX)**2 + (robot.robotY-lastRobotY)**2 )    

    if (ap != lastAP) or (distTraveled > 0.25) or (nt.signalCounter > 1000):
        lastPrintTime = time.time()
        nt.finishStats()
                
        dt = datetime.datetime.now()
        ts = dt.strftime('%y-%m-%d %H:%M:%S')        
        line = ts + '  AP ' + ap 
        line += '  state %d  volt %.1f  GPS(x=%.2f,y=%.2f,sol=%d,sv=%d,dsv=%d)'  \
           '  sig dBm(avg=%.0f min=%.0f max=%.0f)  qty(avg=%.0f min=%.0f max=%.0f)  ping sec(err=%d avg=%.3f min=%.3f max=%.3f)'   \
           % (robot.robotState, robot.robotVoltage, robot.robotX, robot.robotY, robot.robotGpsSol, robot.gpsNumSV, robot.gpsNumSVdgps, nt.signalLevelAvg, nt.signalLevelMin, \
           nt.signalLevelMax, nt.linkQualityAvg, nt.linkQualityMin, nt.linkQualityMax, nt.pingErrors, nt.pingTimeAvg, nt.pingTimeMin, nt.pingTimeMax) 
        print(line)
        
        line = '%s,%s,%d,%.1f,%.2f,%.2f,%d,%d,%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%d,%.3f,%.3f,%.3f'  \
           % (ts, ap, robot.robotState, robot.robotVoltage, robot.robotX, robot.robotY, robot.robotGpsSol, robot.gpsNumSV, robot.gpsNumSVdgps, nt.signalLevelAvg, nt.signalLevelMin, \
           nt.signalLevelMax, nt.linkQualityAvg, nt.linkQualityMin, nt.linkQualityMax, nt.pingErrors, nt.pingTimeAvg, nt.pingTimeMin, nt.pingTimeMax)
        addFile(line + '\n')
        
        nt.resetStats()
        lastAP = ap
        lastRobotX = robot.robotX
        lastRobotY = robot.robotY

    if not SIM:
        time.sleep(1.0)
    

