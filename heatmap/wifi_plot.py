# plot wifi signal quality/ping statistics
#
# https://forum.ardumower.de/threads/advanced-topic-generate-wifi-gps-heatmaps-with-sunray-on-alfred-or-ardumower-with-connected-raspberry-pi.25078/

'''

run with:  python3 wifi_plot.py

reading log...
seen access points: 
['D8:47:32:A9:FB:BC', '80:3F:5D:BF:C8:05']
analysing log (3121x17)...
saving image: output/wifi_heatmap_signal_min.jpg (825x785)
saving image: output/wifi_heatmap_ping_avg.jpg (825x785)
saving image: output/wifi_heatmap_ping_max.jpg (825x785)
saving image: output/wifi_heatmap_ap.jpg (825x785)
saving image: output/wifi_heatmap_gps.jpg (825x785)
analysing log done!


'seen access points' will give a list of MAC addresses of all seen AP's - 1st AP (0), 2nd AP (1), 3rd AP (2) etc.

'''

import robot_http_client
import heat_image_cv
import cv2
import numpy as np
import datetime
import time
import getopt, sys
import os
import shutil

LOG = 'wifi_signal_log.txt'

heatSigMin = heat_image_cv.HeatImageCV('signal_min', -90, -30, 'dbM signalLevelMin', cv2.COLORMAP_JET, False)  # dBm (-30 good ... -90 bad)        
heatPingAvg = heat_image_cv.HeatImageCV('ping_avg', 0, 3, 'sec pingTimeAvg', cv2.COLORMAP_JET, False)  #  ping time (0 good ... 10 bad)
heatPingMax = heat_image_cv.HeatImageCV('ping_max', 0, 5, 'sec pingTimeMax', cv2.COLORMAP_JET, False)  #  ping time (0 good ... 10 bad)
heatAP = heat_image_cv.HeatImageCV('ap', 0, 20, 'access point', None, True)  #  access points (0..30) 
heatGPS = heat_image_cv.HeatImageCV('gps', 0, 3, ['gps invalid', 'gps float', 'gps fix'], None, True)  #  GPS solution (0..30) 
heatNumSV = heat_image_cv.HeatImageCV('gps_sv', 20, 50, '#SV', cv2.COLORMAP_JET, False)  #  number satellites (0 bad ... 50 good)
heatNumDSV = heat_image_cv.HeatImageCV('gps_dsv', 20, 50, '#DSV', cv2.COLORMAP_JET, False)  #  number DGPS satellites (0 bad ... 50 good)


data = None  
apList = []
heatList = [heatSigMin, heatPingAvg, heatPingMax, heatAP, heatGPS]


def readLog():
    global data, apList
    print('reading log...')
    file = open(LOG, 'r')

    lineCounter = 0
    lstlst = []

    for line in file:
        line = line.strip()
        if line.startswith('#'): 
            # print(line)
            continue
        lst = line.split(',')
        #print(len(arr))
        ts, ap, robotState, robotVoltage, robotX, robotY, robotGpsSol, gpsNumSV, gpsNumSVdgps, signalLevelAvg, signalLevelMin, signalLevelMax, linkQualityAvg, linkQualityMin, linkQualityMax, pingErrors, pingTimeAvg, pingTimeMin, pingTimeMax = lst                        
                        
        if not ap in apList: apList.append(ap)         
        apIdx = apList.index(ap)        
        dt = datetime.datetime.strptime(ts, '%y-%m-%d %H:%M:%S')
        timestamp = dt.timestamp()
        lst = timestamp, apIdx, robotState, robotVoltage, robotX, robotY, robotGpsSol, gpsNumSV, gpsNumSVdgps, signalLevelAvg, signalLevelMin, signalLevelMax, linkQualityAvg, linkQualityMin, linkQualityMax, pingErrors, pingTimeAvg, pingTimeMin, pingTimeMax                        
        
        if (int(robotGpsSol) < 1) or (int(robotGpsSol) > 2): continue         
        lstlst.append(lst)
        
        lineCounter += 1
        #time.sleep(0.1)
    data = np.array(lstlst, dtype=float)
    print("seen access points: ")
    print(apList)


def analyseLog():
    print('analysing log (%dx%d)...' % (data.shape) )  

    resolution = 0.2  # 0.2m resolution

    minX = int(round(min(data[:,4]) / resolution ))     
    maxX = int(round(max(data[:,4]) / resolution ))     
    minY = int(round(min(data[:,5]) / resolution ))     
    maxY = int(round(max(data[:,5]) / resolution ))     

    heatSigMin.resizeMinMax(minX, maxX, minY, maxY)                    
    heatPingAvg.resizeMinMax(minX, maxX, minY, maxY)
    heatPingMax.resizeMinMax(minX, maxX, minY, maxY)
    heatAP.resizeMinMax(minX, maxX, minY, maxY)
    heatGPS.resizeMinMax(minX, maxX, minY, maxY)
    heatNumSV.resizeMinMax(minX, maxX, minY, maxY)
    heatNumDSV.resizeMinMax(minX, maxX, minY, maxY)
    

    lineCounter = 0
    lastPrintTime = time.time()

    for row in data:                
        ts, apIdx, robotState, robotVoltage, robotX, robotY, robotGpsSol, gpsNumSV, gpsNumSVdgps, signalLevelAvg, signalLevelMin, signalLevelMax, linkQualityAvg, linkQualityMin, linkQualityMax, pingErrors, pingTimeAvg, pingTimeMin, pingTimeMax = row                        
        
        robotState = int(robotState)        
        robotVoltage = float(robotVoltage)
        robotX = float(robotX)
        robotY = float(robotY)
        robotGpsSol = int(robotGpsSol)
        gpsNumSV = int(gpsNumSV)
        gpsNumSVdgps = int(gpsNumSVdgps)        
        signalLevelAvg = float(signalLevelAvg)
        signalLevelMin = float(signalLevelMin)
        signalLevelMax = float(signalLevelMax)
        linkQualityAvg = float(linkQualityAvg)
        linkQualityMin = float(linkQualityMin)
        linkQualityMax = float(linkQualityMax)
        pingErrors = int(pingErrors)
        pingTimeAvg = float(pingTimeAvg)
        pingTimeMin = float(pingTimeMin)
        pingTimeMax = float(pingTimeMax)
        apIdx = int(apIdx)

        line  = '%s  AP# %s  state %d  volt %.1f  GPS(x=%.2f,y=%.2f,sol=%d,numsv=%d,numdsv=%d)'  \
           '  sig dBm(avg=%.0f min=%.0f max=%.0f)  qty(avg=%.0f min=%.0f max=%.0f)  ping sec(err=%d avg=%.3f min=%.3f max=%.3f)'   \
           % (ts, apIdx, robotState, robotVoltage, robotX, robotY, robotGpsSol, gpsNumSV, gpsNumSVdgps, signalLevelAvg, signalLevelMin, signalLevelMax, linkQualityAvg, linkQualityMin, linkQualityMax, pingErrors, pingTimeAvg, pingTimeMin, pingTimeMax) 
        #print(line)        
        if time.time() > lastPrintTime + 10:
            lastPrintTime = time.time()
            print(str(int(lineCounter / len(data)*100.0)) + '% ')

        imgX = int(round(robotX / resolution )) 
        imgY = int(round(robotY / resolution ))

        # draw dBm (-30 good ... -90 bad)
        heatSigMin.drawSignal(imgX, imgY, signalLevelMin)
        # draw ping time (0 good ... 10 bad)
        heatPingAvg.drawSignal(imgX, imgY, pingTimeAvg)
        heatPingMax.drawSignal(imgX, imgY, pingTimeMax)
        # draw access point number        
        heatAP.drawSignal(imgX, imgY, apIdx)
        # draw GPS solution        
        heatGPS.drawSignal(imgX, imgY, robotGpsSol)                
        # draw number satellites        
        heatNumSV.drawSignal(imgX, imgY, gpsNumSV)                
        heatNumDSV.drawSignal(imgX, imgY, gpsNumSVdgps)                

        lineCounter += 1
       
    heatSigMin.saveImage()
    heatPingAvg.saveImage()    
    heatPingMax.saveImage()
    heatAP.saveImage()    
    heatGPS.saveImage()
    heatNumSV.saveImage()
    heatNumDSV.saveImage()
        
    print('analysing log done!')    


argumentList = sys.argv[1:]
 
# Options
options = "hg:"
 
# Long options
long_options = ["help", "gif", "mp4"]


# Python program to demonstrate
# command line arguments
 
 
import getopt, sys
 
 
# Remove 1st argument from the
# list of command line arguments
argumentList = sys.argv[1:]
 
# Options
options = "hgv:"
 
# Long options
long_options = ["help", "gif", "video"]
 
readLog()


try:
    # Parsing argument
    arguments, values = getopt.getopt(argumentList, options, long_options)
     
    # checking each argument
    for currentArgument, currentValue in arguments:
 
        if currentArgument in ("-h", "--help"):
            print ("--gif generate animated gif")
            print ("--mp4 generate video")                    
        elif currentArgument in ("-g", "--gif"):            
            print ("will generate animated gif (warning: will consume CPU and huge amount of memory, run this on a high-end PC!)")
            #folder = os.path.join(os.getcwd(), 'output')
            #folder = os.path.join(folder, 'gif')            
            #shutil.rmtree(folder)
            for hmap in heatList: 
                hmap.makeGif = True
                hmap.resize = 3   # reduce if your PC is stalling             
        elif currentArgument in ("-v", "--video"):
            print ("will generate video (warning: will consume CPU, run this on a high-end PC!)")            
            for hmap in heatList: 
                hmap.makeVideo = True

             
except getopt.error as err:
    # output error, and return with an error code
    print (str(err))

analyseLog()

