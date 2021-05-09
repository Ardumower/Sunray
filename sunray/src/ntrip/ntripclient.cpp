#ifdef __linux__

#include "../../config.h"
#include "ntripclient.h"


void NTRIPClient::begin(){
  CONSOLE.println("Requesting SourceTable.");
  if(reqSrcTbl(host,httpPort)){
    char buffer[512];
    delay(5);
    while(available()){
      readLine(buffer,sizeof(buffer));
      CONSOLE.print(buffer); 
    }
  }
  else{
    CONSOLE.println("SourceTable request error");
  }
  CONSOLE.print("Requesting SourceTable is OK\n");
  stop(); //Need to call "stop" function for next request.
  
  CONSOLE.println("Requesting MountPoint's Raw data");
  if(!reqRaw(NTRIP_HOST,NTRIP_PORT,NTRIP_MOUNT,NTRIP_USER,NTRIP_PASS)){
    delay(15000);
    //ESP.restart();
  }
  CONSOLE.println("Requesting MountPoint is OK");}
}


void NTRIPClient::run(){
  while(available()) {
    char ch = read();        
    //CONSOLE.print(ch);
    NTRIP.write(ch);  // send to GPS receiver (NTRIP port)        
  }
}


bool NTRIPClient::reqSrcTbl(char* host,int &port)
{
  if(!connect(host,port)){
      CONSOLE.print("Cannot connect to ");
      CONSOLE.println(host);
      return false;
  }/*p = String("GET ") + String("/") + String(" HTTP/1.0\r\n");
  p = p + String("User-Agent: NTRIP Enbeded\r\n");*/
  print(
      "GET / HTTP/1.0\r\n"
      "User-Agent: NTRIPClient for Arduino v1.0\r\n"
      );
  unsigned long timeout = millis();
  while (available() == 0) {
     if (millis() - timeout > 5000) {
        CONSOLE.println("Client Timeout !");
        stop();
        return false;
     }
     delay(10);
  }
  char buffer[50];
  readLine(buffer,sizeof(buffer));
  if(strncmp((char*)buffer,"SOURCETABLE 200 OK",17))
  {
    CONSOLE.print((char*)buffer);
    return false;
  }
  return true;
    
}
bool NTRIPClient::reqRaw(char* host,int &port,char* mntpnt,char* user,char* psw)
{
    if(!connect(host,port))return false;
    String p="GET /";
    String auth="";
    CONSOLE.println("Request NTRIP");
    
    p = p + mntpnt + String(" HTTP/1.0\r\n"
        "User-Agent: NTRIPClient for Arduino v1.0\r\n"
    );
    
    if (strlen(user)==0) {
        p = p + String(
            "Accept: */*\r\n"
            "Connection: close\r\n"
            );
    }
    else {
        auth = base64::encode(String(user) + String(":") + psw);
        #ifdef Debug
        CONSOLE.println(String(user) + String(":") + psw);
        #endif

        p = p + String("Authorization: Basic ");
        p = p + auth;
        p = p + String("\r\n");
    }
    p = p + String("\r\n");
    print(p);
    #ifdef Debug
    CONSOLE.println(p);
    #endif
    unsigned long timeout = millis();
    while (available() == 0) {
        if (millis() - timeout > 20000) {
            CONSOLE.println("Client Timeout !");
            return false;
        }
        delay(10);
    }
    char buffer[50];
    readLine(buffer,sizeof(buffer));
    if(strncmp((char*)buffer,"ICY 200 OK",10))
    {
      CONSOLE.print((char*)buffer);
      return false;
    }
    return true;
}
bool NTRIPClient::reqRaw(char* host,int &port,char* mntpnt)
{
    return reqRaw(host,port,mntpnt,"","");
}
int NTRIPClient::readLine(char* _buffer,int size)
{
  int len = 0;
  while(available()) {
    _buffer[len] = read();
    len++;
    if(_buffer[len-1] == '\n' || len >= size) break;
  }
  _buffer[len]='\0';

  return len;
}


#endif
