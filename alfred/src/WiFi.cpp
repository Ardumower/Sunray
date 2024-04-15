/*
  WiFi.h - Library for Linux
  
*/

//#include "utility/wifi_drv.h"
#include "WiFi.h"

#include <Process.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timeb.h>

#include <errno.h>
#include <sys/types.h>
#include <signal.h>


#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>   
#include <linux/wireless.h>   


WiFiClass::WiFiClass()
{
	numNetworks = 0;
	rssi = 0;
	//iface = "wlo1";
	iface = "wlan0";
	//scanNetworks();	
	//getStatus();
	//begin("ssid", "pass");	
}


void WiFiClass::init()
{
    //WiFiDrv::wifiDriverInit();
}

uint8_t WiFiClass::getSocket()
{
    return 0;
	//return NO_SOCKET_AVAIL
}

char* WiFiClass::firmwareVersion()
{
	return NULL;
}

void WiFiClass::begin()
{
	findWifiInterface();
	byte mac[6];
	macAddress(&mac[0]);	
	accessPointMacAddress(&mac[0]);
	signalLevel();
} 

int WiFiClass::begin(char* ssid)
{
	return 0;
}

int WiFiClass::begin(char* ssid, uint8_t key_idx, const char *key)
{
	return 0;
}

int WiFiClass::startWifiProtectedSetup(){
	Serial.print("starting WiFi protected setup - NOTE: press WPS button on your router first...");
	//Serial.println(passphrase);		
	Process p;
    p.runShellCommand("wpa_cli -i " + iface + " remove_network all");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " wps_pbc");
	Serial.println(p.readString());
	return 0;
}

int WiFiClass::begin(char* ssid, const char *passphrase)
{
	Serial.print("connecting WiFi...SSID=");
	Serial.println(ssid);
	//Serial.println(passphrase);		
	Process p;
    p.runShellCommand("wpa_cli -i " + iface + " remove_network all");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " add_network");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " set_network 0 ssid '\"" + ssid + "\"'");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " set_network 0 psk '\"" + passphrase + "\"'");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " list_networks");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " enable_network 0");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " reconnect");
	Serial.println(p.readString());
	p.runShellCommand("wpa_cli -i " + iface + " save_config");	
	Serial.println(p.readString()); 
	return 0;
}

void WiFiClass::config(IPAddress local_ip)
{
	//WiFiDrv::config(1, (uint32_t)local_ip, 0, 0);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server)
{
	//WiFiDrv::config(1, (uint32_t)local_ip, 0, 0);
	//WiFiDrv::setDNS(1, (uint32_t)dns_server, 0);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway)
{
	//WiFiDrv::config(2, (uint32_t)local_ip, (uint32_t)gateway, 0);
	//WiFiDrv::setDNS(1, (uint32_t)dns_server, 0);
}

void WiFiClass::config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet)
{
	//WiFiDrv::config(3, (uint32_t)local_ip, (uint32_t)gateway, (uint32_t)subnet);
	//WiFiDrv::setDNS(1, (uint32_t)dns_server, 0);
}

void WiFiClass::setDNS(IPAddress dns_server1)
{
	//WiFiDrv::setDNS(1, (uint32_t)dns_server1, 0);
}

void WiFiClass::setDNS(IPAddress dns_server1, IPAddress dns_server2)
{
	//WiFiDrv::setDNS(2, (uint32_t)dns_server1, (uint32_t)dns_server2);
}

int WiFiClass::disconnect()
{
    //return WiFiDrv::disconnect();
	return 0;
}

void WiFiClass::findWifiInterface(){
	printf("WiFiClass::findWifiInterface\n");
	// Create a channel to the NET kernel. 
  	int fd = socket(AF_INET, SOCK_DGRAM, 0);
    // Get list of active devices 	
	char buff[1024];
	struct ifconf ifc;
	ifc.ifc_len = sizeof(buff);
	ifc.ifc_buf = buff;
	if(ioctl(fd, SIOCGIFCONF, &ifc) < 0) {
		printf("ERROR finding active WiFi interface (SIOCGIFCONF)\n");
		return;
	}
	struct ifreq *ifr;
	ifr = ifc.ifc_req;
	// use the first match 
  	struct iwreq		wrq;
	for(int i = ifc.ifc_len / sizeof(struct ifreq); --i >= 0; ifr++){
		strncpy(wrq.ifr_name, ifr->ifr_name, IFNAMSIZ);
		if (0 == ioctl(fd, SIOCGIWAP, &wrq)) {    
			iface = ifr->ifr_name;
			printf("found WiFi interface: %s\n", iface.c_str());
		}
	}
}

uint8_t* WiFiClass::macAddress(uint8_t* mac)
{		
	int fd;
    struct ifreq ifr;
    unsigned char *_mac = NULL;
    memset(&ifr, 0, sizeof(ifr));
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name , iface.c_str() , IFNAMSIZ-1);
    if (0 == ioctl(fd, SIOCGIFHWADDR, &ifr)) {
        _mac = (unsigned char *)ifr.ifr_hwaddr.sa_data;
        //display mac address
        printf("MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n" , _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
		memcpy(mac, _mac, 6);
	}
    close(fd);
	//uint8_t* _mac = WiFiDrv::getMacAddress();	 
    return mac;
}


uint8_t* WiFiClass::accessPointMacAddress(uint8_t* mac)
{		
	int fd;
    struct iwreq wrq;
    unsigned char *_mac = NULL;
    memset(&wrq, 0, sizeof(wrq));
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    strncpy(wrq.ifr_name, iface.c_str(), IFNAMSIZ-1);
    if (0 == ioctl(fd, SIOCGIWAP, &wrq)) {
        _mac = (unsigned char *)wrq.u.ap_addr.sa_data;
        //display mac address
        printf("AP_MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n" , _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);
		memcpy(mac, _mac, 6); 
    }
    close(fd);
	//uint8_t* _mac = WiFiDrv::getMacAddress();
    return mac;
}

int WiFiClass::signalLevel(){
	int fd;
    struct iwreq wrq;
    memset(&wrq, 0, sizeof(wrq));
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    strncpy(wrq.ifr_name, iface.c_str(), IFNAMSIZ-1);
    struct iw_statistics iwstats;
	memset(&iwstats, 0, sizeof(iwstats));
	wrq.u.data.pointer = (caddr_t)&iwstats;
	wrq.u.data.length = sizeof(iwstats);
	wrq.u.data.flags = 1; // Clear updated flag 
	if (0 == ioctl(fd, SIOCGIWSTATS, &wrq)) {
		if (iwstats.qual.updated & IW_QUAL_DBM){
        	printf("WiFi signal level: %d dBm\n" , iwstats.qual.level - 256); // (qual, level, noise, updated)
		}
	}
    close(fd);
	return iwstats.qual.level - 256;
}


void WiFiClass::getStatus(){
	Serial.println("WiFiClass::getStatus");
	Process p;
    p.runShellCommand("wpa_cli -i " + iface + " status");
	String key = "";	
	String value = "";
	bool keyComplete = false;
	while (p.available()){
		char ch = p.read();
		if (ch == '='){
			keyComplete = true;
		} 
		else if (ch == '\n'){
			if (key == "ssid") ssid = value;			
			if (key == "bssid") bssid = value;
			if (key == "key_mgmt") encryption = value;
			if (key == "ip_address") ipaddr = value;		
			keyComplete = false;
			key = "";
			value = "";
		} else if (ch != '\r'){
			if (keyComplete) value += ch;
			  else key += ch;			
		}
		
	}
	Serial.print("BSSID=");
	Serial.println(bssid);
	Serial.print("SSID=");
	Serial.println(ssid);
	Serial.print("IPADDR=");
	Serial.println(ipaddr);
}

   
IPAddress WiFiClass::localIP()
{
	getStatus();
	IPAddress ret;
	ret.fromString(ipaddr);
	return ret;
}

IPAddress WiFiClass::subnetMask()
{
	IPAddress ret;
	//WiFiDrv::getSubnetMask(ret);
	return ret;	
}

IPAddress WiFiClass::gatewayIP()
{
	IPAddress ret;
	//WiFiDrv::getGatewayIP(ret);
	return ret;
}

char* WiFiClass::SSID()
{
	getStatus();
	return (char*)ssid.c_str(); 
}

uint8_t* WiFiClass::BSSID(uint8_t* bssid)
{
	return NULL;
	/*uint8_t* _bssid = WiFiDrv::getCurrentBSSID();
	memcpy(bssid, _bssid, WL_MAC_ADDR_LENGTH);
    return bssid;*/
}

int32_t WiFiClass::RSSI()
{
    return rssi;
}

uint8_t WiFiClass::encryptionType()
{
	return 0;
}


int8_t WiFiClass::scanNetworks()
{
	Serial.println("WiFiClass::scanNetworks");
	Process p;
    p.runShellCommand("wpa_cli -i " + iface + " scan");
    p.runShellCommand("wpa_cli -i " + iface + " scan_result");
	Serial.println("WiFiClass::scanNetworks completed");
	
	numNetworks = 0;
	int line = 0;
	int col = 0;
	bool newCol = false;
	bool newLine = false;
	String colValue = "";		
	while (p.available()){
		char ch = p.read();
		if (ch == '\t'){
			newCol = true;
		} 
		else if (ch == '\n'){
			newCol = true;
			newLine = true;						
		} else if (ch != '\r') {
			colValue += ch;			
		}

		if (newCol){	
			col++;								
			if (col == 1){
				networkBSSID[numNetworks] = colValue;
			} else if (col == 3){
				networkRSSI[numNetworks] = colValue.toInt();
			} else if (col == 2){
				networkEncryption[numNetworks] = colValue;
			}
			else if (col == 5){					
				networkSSID[numNetworks] = colValue; 
			}															
			newCol = false;
			colValue = "";		
			if (newLine) {
				col = 0;
				if (line > 0){
					if (numNetworks < MAX_NETWORKS-1) numNetworks++;
				}
				newLine = false;
				line++;
			}			
		}
	}
	
	Serial.print("WiFiClass::numNetworks=");
	Serial.println(numNetworks);

	for (int i=0; i < numNetworks; i++){
		Serial.print(networkBSSID[i]);
		Serial.print(",");
		Serial.print(networkRSSI[i]);
		Serial.print(",");
		Serial.println(networkSSID[i]);
	}

	return numNetworks;
}

char* WiFiClass::SSID(uint8_t networkItem)
{
	if (networkItem >= numNetworks) return 0;
	return (char*)networkSSID[networkItem].c_str();
}

int32_t WiFiClass::RSSI(uint8_t networkItem)
{
	if (networkItem >= numNetworks) return 0;
	return networkRSSI[networkItem];
}

uint8_t WiFiClass::encryptionType(uint8_t networkItem)
{
	return 0;
}

uint8_t WiFiClass::status()
{
    return 0;
}

int WiFiClass::hostByName(const char* aHostname, IPAddress& aResult)
{
	return 0;
}

WiFiClass WiFi;
