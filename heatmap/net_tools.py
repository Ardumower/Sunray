# wifi network tools (signal quality, ping etc.)

import socket
import struct
import subprocess


class NetTools():
    def __init__(self):
        self.gatewayAddr = '8.8.8.8'
        self.interface = 'wlan0'

        self.linkQuality = 0
        self.signalLevel = 0
        self.signalNoise = 0
        self.signalCounter = 0

        self.pingCounter = 0
        self.pingTime = 0
        self.pingErrors = 0

        self.lastPrintTime = 0
        self.lastAP = ""


    def detectNetDevices(self):
        installed = False
        while (True):
            try:
                cmd = 'sudo arp-scan --interface=' + self.interface + ' --localnet'
                output = subprocess.check_output(cmd, shell=True)
                output = output.decode()
                break
            except:
                if installed: return([]) 
                print('arp-scan not found - installing...')
                #print(output)
                try:
                    installed = True
                    cmd = 'sudo apt-get install arp-scan -y'
                    output = subprocess.check_output(cmd, shell=True)
                    output = output.decode()
                    print('arp-scan installation done')
                except:
                    print('error installing arp-scan!')
                    return([]) 
        #print(output)         
        output = output.split('\n')[2:]
        output = output[:-4]    
        return output       


    def getMAC(self):
        try:
            mac = open('/sys/class/net/'+self.interface+'/address').readline()
        except:
            mac = "00:00:00:00:00:00"
        return mac[0:17]


    def getAccessPointMAC(self):
        cmd = 'iwconfig'
        output = subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)    
        output = output.decode()
        mac = output.partition('Access Point: ')[2]
        mac = mac.partition('\n')[0].strip()    
        return mac


    def findWifiInterface(self):
        cmd = 'cat /proc/net/wireless'
        output = subprocess.check_output(cmd, shell=True)    
        output = output.decode()
        output = output.split('\n')[-2]
        output = " ".join(output.split())
        #print(output)
        cols = output.split(" ")
        #print(cols)
        self.interface = cols[0].replace(':', '')
        #print(interface)


    def findGatewayAddr(self):
        with open("/proc/net/route") as fh:
            # skip header
            next(fh)
            route_list = []
            for line in fh:
                routes = line.strip().split()
                destination = socket.inet_ntoa(struct.pack("<L", int(routes[1], 16)))
                if destination != "0.0.0.0":
                    continue
                gateway = socket.inet_ntoa(struct.pack("<L", int(routes[2], 16)))
                mask = socket.inet_ntoa(struct.pack("<L", int(routes[7], 16)))
                metric = routes[6]
                interface = routes[0]
                route_table = (destination, gateway, mask, metric, interface)
                route_list.append(route_table)
            #pp(route_list)
            if len(route_list) > 0:
                self.gatewayAddr = route_list[0][1]
            else:
                print('error finding gateway!')


    def pingGateway(self):
        numPings = 1
        pingTimeout = 5
        addr =  self.gatewayAddr
        cmd = 'ping -c %s -W %s %s' % (numPings, pingTimeout, addr)
        #print(cmd)
        try:
            output = subprocess.check_output(cmd, shell=True)
            output = output.decode()
            output = output.split('\n')[-3:]
            # -1 is a blank line, -3 & -2 contain the actual results
            xmit_stats = output[0].split(",")
            timing_stats = output[1].split("=")[1].split("/")
            packet_loss = float(xmit_stats[2].split("%")[0])
            self.pingTime = float(timing_stats[2]) / 1000.0            
            #pingTimeMin = float(timing_stats[0])
            #pingTimeAvg = float(timing_stats[1])
            #pingTimeMax = float(timing_stats[2])
        except:
            self.pingTime = -1
            self.pingErrors += 1    
        #print(pingTimeAvg, pingTimeMin, pingTimeMax)


    def readInterfaceStats(self):
        lines = open("/proc/net/dev", "r").readlines()
        columnLine = lines[1]
        _, receiveCols , transmitCols = columnLine.split("|")
        receiveCols = map(lambda a:"recv_"+a, receiveCols.split())
        transmitCols = map(lambda a:"trans_"+a, transmitCols.split())
        cols = receiveCols+transmitCols
        faces = {}
        for line in lines[2:]:
            if line.find(":") < 0: continue
            face, data = line.split(":")
            faceData = dict(zip(cols, data.split()))
            faces[face] = faceData
        pp(faces)


    def detectLinkQuality(self):
        cmd = 'cat /proc/net/wireless'
        output = subprocess.check_output(cmd, shell=True)    
        output = output.decode()
        output = output.split('\n')[-2]
        output = " ".join(output.split())
        #print(output)
        cols = output.split(" ")
        #print(cols)
        self.linkQuality = float(cols[2])
        self.signalLevel = float(cols[3])
        self.signalNoise = float(cols[4])
        #print(linkQuality, signalLevel, signalNoise)


    def resetStats(self):
        self.pingErrors = 0      
        self.pingTimeMin = 9999999
        self.pingTimeMax = - 9999999
        self.pingTimeAvg = 0
        self.pingCounter = 0

        self.linkQualityAvg = 0
        self.linkQualityMin = 9999999
        self.linkQualityMax = -9999999

        self.signalLevelAvg = 0
        self.signalLevelMin = 9999999
        self.signalLevelMax = -9999999
        self.signalCounter = 0


    def computeStats(self):
        if self.pingTime >= 0:
            self.pingCounter += 1
            self.pingTimeAvg += self.pingTime 
            self.pingTimeMin = min(self.pingTimeMin, self.pingTime)
            self.pingTimeMax = max(self.pingTimeMax, self.pingTime)    

        self.linkQualityAvg += self.linkQuality 
        self.linkQualityMin = min(self.linkQualityMin, self.linkQuality)
        self.linkQualityMax = max(self.linkQualityMax, self.linkQuality)    

        self.signalLevelAvg += self.signalLevel 
        self.signalLevelMin = min(self.signalLevelMin, self.signalLevel)
        self.signalLevelMax = max(self.signalLevelMax, self.signalLevel)    

        self.signalCounter += 1


    def finishStats(self):
        self.linkQualityAvg /= self.signalCounter
        self.signalLevelAvg /= self.signalCounter
        if self.pingCounter > 0:            
            self.pingTimeAvg /= self.pingCounter



if __name__ == "__main__":
    nt = NetTools()
    nt.findWifiInterface()
    nt.findGatewayAddr()
    print('interface: %s   gateway: %s   ' % (nt.interface, nt.gatewayAddr)) 

    print('MAC: %s  AP: %s'  % (nt.getMAC(), nt.getAccessPointMAC()) )

    nt.detectLinkQuality()    
    print('signal level dbM: %.0f' % (nt.signalLevel))

    nt.pingGateway()
    print('gateway ping time sec: %.3f' % (nt.pingTime) )
    
    print('found network devices: ')
    lines = nt.detectNetDevices()
    for line in lines:
        print(line)



            
