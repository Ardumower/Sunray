#include "BleUartServer.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//#include <termios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>

extern "C" {
	#include <bluetooth/bluetooth.h>
	#include <bluetooth/hci.h>
	#include <bluetooth/hci_lib.h>
	#include <bluetooth/l2cap.h>

	#include "ble/uuid.h"
	#include "ble/mainloop.h"
	#include "ble/util.h"
	#include "ble/att.h"
	#include "ble/queue.h"
	#include "ble/timeout.h"
	#include "ble/gatt-db.h"
	#include "ble/gatt-server.h"
}

#include <Arduino.h>

//#define BLE_PROTOCOL_DABBLE   1  // choose this for Dabble Bluetooth protocol (Dabble App)  (disable for Sunray App)
//#define BLE_PROTOCOL_SUNRAY   1  // choose this for Sunray Bluetooth protocol (Sunray App)  (disable for Dabble App)


#define ATT_CID 4


#ifdef BLE_PROTOCOL_DABBLE
  // Dabble App
  #define UUID_CUSTOM_SERVICE    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // serv: custom service 
  #define UUID_CUSTOM_CHAR_RX    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // char: custom char (props: read, write, notify, write_no_respons) 
  #define UUID_CUSTOM_CHAR_TX    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // char: custom char (props: read, write, notify, write_no_respons) 
#else
  // Sunray App
  #define UUID_CUSTOM_SERVICE    "0000ffe0-0000-1000-8000-00805f9b34fb"  // serv: custom service 
  #define UUID_CUSTOM_CHAR       "0000ffe1-0000-1000-8000-00805f9b34fb"  // char: custom char (props: read, write, notify, write_no_respons) 
#endif


#define PRLOG(...) \
	do { \
		::printf(__VA_ARGS__); \
		print_prompt(); \
	} while (0)


#define COLOR_OFF	"\x1B[0m"
#define COLOR_RED	"\x1B[0;91m"
#define COLOR_GREEN	"\x1B[0;92m"
#define COLOR_YELLOW	"\x1B[0;93m"
#define COLOR_BLUE	"\x1B[0;94m"
#define COLOR_MAGENTA	"\x1B[0;95m"
#define COLOR_BOLDGRAY	"\x1B[1;30m"
#define COLOR_BOLDWHITE	"\x1B[1;37m"

#ifndef __arm__
	void baswap(bdaddr_t *dst, const bdaddr_t *src)
	{
		register unsigned char *d = (unsigned char *) dst;
		register const unsigned char *s = (const unsigned char *) src;
		register int i;
		for (i = 0; i < 6; i++)
			d[i] = s[5-i];
	}

	int ba2str(const bdaddr_t *ba, char *str){
		uint8_t b[6];
		baswap((bdaddr_t *) b, ba);
		return sprintf(str, "%2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X",
			b[0], b[1], b[2], b[3], b[4], b[5]);
		return 0;
	} 	
#endif


void *bleThreadFun(void *user_data)
{
    BleUartServer *server = (BleUartServer*)user_data;
	while (true){
		server->run();
	}
	return NULL;
}

static void print_prompt(void)
{
	::printf(COLOR_BLUE "[GATT server]" COLOR_OFF "# ");
	fflush(stdout);
}

static void att_debug_cb(const char *str, void *user_data)
{
	const char *prefix = (const char*)user_data;
	PRLOG(COLOR_BOLDGRAY "%s" COLOR_BOLDWHITE "%s\n" COLOR_OFF, prefix, str);
}

static void gatt_debug_cb(const char *str, void *user_data)
{
	const char *prefix = (const char*)user_data;
	PRLOG(COLOR_GREEN "%s%s\n" COLOR_OFF, prefix, str);
}

static void att_disconnect_cb(int err, void *user_data)
{
	::printf("BLE: Device disconnected: %s\n", strerror(err));
	mainloop_quit();
}

static void signal_cb(int signum, void *user_data)
{
	switch (signum) {
	case SIGINT:
	case SIGTERM:
		mainloop_quit();
		break;
	default:
		break;
	}
}


// client read operation
static void msrmt_ccc_read_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{	
	::printf("BLE: msrmt_ccc_read_cb id=%d ofs=%d\n", id, offset);
	BleUartServer *server = (BleUartServer*)user_data;
	//#define VALUE  "V,sunray,1.0,0x10"
	//gatt_db_attribute_read_result(attrib, id, 0, VALUE, strlen(VALUE));	
	uint8_t value[2];
	value[0] = server->msrmt_enabled ? 0x01 : 0x00;
	value[1] = 0x00;
	gatt_db_attribute_read_result(attrib, id, 0, value, 2);
}


// client write operation
static void msrmt_ccc_write_cb(struct gatt_db_attribute *attrib,
					unsigned int id, uint16_t offset,
					const uint8_t *value, size_t len,
					uint8_t opcode, struct bt_att *att,
					void *user_data)
{
	BleUartServer *server = (BleUartServer*)user_data;
	pthread_mutex_lock( &server->rxMutex );  	
	uint8_t ecode = 0;
	//::printf("BLE: msrmt_ccc_write_cb len=%d\n", (int)len);
	if (len == 2){
		if (value[0] == 0x00){	  	
			server->msrmt_enabled = false;
			::printf("BLE: notify off\n");
		} else if (value[0] == 0x01) {
			::printf("BLE: notify on\n");
			server->msrmt_enabled = true;
		} 
		//update_hr_msrmt_simulation(server);
	} else {
		for (int i=0; i < len; i++){
			if ( ((server->rxWritePos +1) % BLE_BUF_SZ) == server->rxReadPos){
			  ::printf("BLE: rxBuf overflow!\n");
			  break;
			}			
			server->rxBuf[server->rxWritePos] = value[i];                          // push it to the ring buffer  
			server->rxWritePos = (server->rxWritePos + 1) % BLE_BUF_SZ; 			
		}
	}
	gatt_db_attribute_write_result(attrib, id, ecode);
	pthread_mutex_unlock( &server->rxMutex );  
}


bool BleUartServer::begin(){
  ::printf("BLE: begin\n");
  return true;
}

bool BleUartServer::begin(int baudrate){
  return begin();
}

void BleUartServer::end(){
}

int BleUartServer::available(){
  pthread_mutex_lock( &rxMutex );
  int i = 0;
  int pos = rxReadPos;
  while (pos != rxWritePos){
	i++;
    pos = (pos + 1) % BLE_BUF_SZ;	
  }
  pthread_mutex_unlock( &rxMutex );   
  return i; 
}

int BleUartServer::peek(){
    return 0;
}

int BleUartServer::read(){
  pthread_mutex_lock( &rxMutex );
  int value = 0;
  if (rxReadPos != rxWritePos){
    value = rxBuf[rxReadPos];
    rxReadPos = (rxReadPos + 1) % BLE_BUF_SZ;
  } 
  pthread_mutex_unlock( &rxMutex );   
  return value;
}

void BleUartServer::flush(){
    //console_flush();
}


static void destroy_packet_cb(void *user_data){
  BleUartServer *server = (BleUartServer*)user_data;
  server->notify(); 
}


size_t BleUartServer::write(uint8_t c){
	pthread_mutex_lock( &txMutex );
	if ( ((txWritePos +1) % BLE_BUF_SZ) == txReadPos){
		::printf("BLE: txBuf overflow!\n");
		return 0;
	}			
	txBuf[txWritePos] = c;                          // push it to the ring buffer  
	txWritePos = (txWritePos + 1) % BLE_BUF_SZ;	 			
	pthread_mutex_unlock( &txMutex );
	if (char(c) == '\n'){
	  notify();
	}
	return 1;
}

void BleUartServer::notify(){	  
	pthread_mutex_lock( &txMutex );	
	String s = "";
	int maxLen = 16; //bt_att_get_mtu(att) - 1;	
	while (txReadPos != txWritePos){
		s = s + char(txBuf[txReadPos]);
		txReadPos = (txReadPos + 1) % BLE_BUF_SZ;
		if (s.length() >= maxLen) break; 
	}
	if ((s.length() > 0) && (msrmt_enabled)) {
		//::printf("BLE: write %d\n", s.length());
		// https://stackoverflow.com/questions/35200626/how-do-i-send-a-long-notification-with-bluez-example
		bt_gatt_server_send_notification(gatt, msrmt_handle, (const uint8_t*)s.c_str(), s.length(), false, this, destroy_packet_cb);			
	}
	pthread_mutex_unlock( &txMutex );	
}



void BleUartServer::populateUartService()
{
	bt_uuid_t uuid;
	
	/* Add Uart Service */
	//bt_uuid16_create(&uuid, UUID_CUSTOM_SERVICE);
	bt_string_to_uuid(&uuid, UUID_CUSTOM_SERVICE);
	service = gatt_db_add_service(db, &uuid, true, 16);
	handle = gatt_db_attribute_get_handle(service);

	/*
	 * Uart Characteristic. 
	 */
	#ifdef BLE_PROTOCOL_DABBLE
		::printf("BLE_PROTOCOL_DABBLE\n");
		bt_string_to_uuid(&uuid, UUID_CUSTOM_CHAR_RX);
		msrmt = gatt_db_service_add_characteristic(service, &uuid,
							BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
							BT_GATT_CHRC_PROP_WRITE | BT_GATT_CHRC_PROP_NOTIFY,
							msrmt_ccc_read_cb, msrmt_ccc_write_cb, this);

		bt_string_to_uuid(&uuid, UUID_CUSTOM_CHAR_TX);
		msrmt = gatt_db_service_add_characteristic(service, &uuid,
							BT_ATT_PERM_WRITE ,
							BT_GATT_CHRC_PROP_WRITE | BT_GATT_CHRC_PROP_WRITE_WITHOUT_RESP,
							msrmt_ccc_read_cb, msrmt_ccc_write_cb, this);
		
		msrmt_handle = gatt_db_attribute_get_handle(msrmt);
	#else 
		::printf("BLE_PROTOCOL_SUNRAY\n");		
		//bt_uuid16_create(&uuid, UUID_CUSTOM_CHAR);	
		bt_string_to_uuid(&uuid, UUID_CUSTOM_CHAR);
		msrmt = gatt_db_service_add_characteristic(service, &uuid,
							BT_ATT_PERM_READ | BT_ATT_PERM_WRITE ,
							BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_WRITE | BT_GATT_CHRC_PROP_NOTIFY | BT_GATT_CHRC_PROP_WRITE_WITHOUT_RESP,
							msrmt_ccc_read_cb, msrmt_ccc_write_cb, this);
		
		msrmt_handle = gatt_db_attribute_get_handle(msrmt);
	#endif

	bt_uuid16_create(&uuid, GATT_CLIENT_CHARAC_CFG_UUID);
	gatt_db_service_add_descriptor(service, &uuid,
					BT_ATT_PERM_READ | BT_ATT_PERM_WRITE,
					msrmt_ccc_read_cb,
					msrmt_ccc_write_cb, this);

	gatt_db_service_set_active(service, true);
}



void BleUartServer::populateDb(){
  //populateGapService();
	//populateGattService();
	populateUartService();
}

bool BleUartServer::listen()
{
	::printf("BLE: listen\n");
  int sk, nsk;
	struct sockaddr_l2 srcaddr, addr;
	socklen_t optlen;
	struct bt_security btsec;
	char ba[18];

	sk = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
	if (sk < 0) {
		perror("BLE: Failed to create L2CAP socket");
		return false;
	}

	/* Set up source address */
	memset(&srcaddr, 0, sizeof(srcaddr));
	srcaddr.l2_family = AF_BLUETOOTH;
	srcaddr.l2_cid = htobs(ATT_CID);
	srcaddr.l2_bdaddr_type = src_type;
	bacpy(&srcaddr.l2_bdaddr, &src_addr);

	if (bind(sk, (struct sockaddr *) &srcaddr, sizeof(srcaddr)) < 0) {
		perror("BLE: Failed to bind L2CAP socket");
		goto fail;
	}

	/* Set the security level */
	memset(&btsec, 0, sizeof(btsec));
	btsec.level = sec;
	if (setsockopt(sk, SOL_BLUETOOTH, BT_SECURITY, &btsec,
							sizeof(btsec)) != 0) {
		::printf("BLE: Failed to set L2CAP security level\n");
		goto fail;
	}

	if (::listen(sk, 10) < 0) {
		perror("BLE: Listening on socket failed");
		goto fail;
	}

	::printf("BLE: Started listening on ATT channel. Waiting for connections\n");

	memset(&addr, 0, sizeof(addr));
	optlen = sizeof(addr);
	nsk = accept(sk, (struct sockaddr *) &addr, &optlen);
	if (nsk < 0) {
		perror("BLE: Accept failed");
		goto fail;
	}

	ba2str(&addr.l2_bdaddr, ba);
	::printf("BLE: Connect from %s\n", ba);
	close(sk);
	fd = nsk;
  return true;

fail:
	close(sk);
	return false;
}



bool BleUartServer::createGattServer()
{
  	::printf("BLE: createGattServer\n");

	att = bt_att_new(fd, false);
	if (!att) {
		::printf("BLE: Failed to initialze ATT transport layer\n");
		goto fail;
	}

	if (!bt_att_set_close_on_unref(att, true)) {
		::printf("BLE: Failed to set up ATT transport layer\n");
		goto fail;
	}

	if (!bt_att_register_disconnect(att, att_disconnect_cb, NULL, NULL)) {
		::printf("BLE: Failed to set ATT disconnect handler\n");
		goto fail;
	}

	db = gatt_db_new();
	if (!db) {
		::printf("BLE: Failed to create GATT database\n");
		goto fail;
	}

	gatt = bt_gatt_server_new(db, att, mtu, 0);
	if (!gatt) {
		::printf("BLE: Failed to create GATT server\n");
		goto fail;
	}

	if (verbose) {
		::printf("BLE: verbose\n");
		bt_att_set_debug(att, BT_ATT_DEBUG_VERBOSE, att_debug_cb, (void*)"att: ", NULL);
		bt_gatt_server_set_debug(gatt, gatt_debug_cb, (void*)"server: ", NULL);
	}

	/* bt_gatt_server already holds a reference */
	populateDb();
	
	return true;

fail:
	gatt_db_unref(db);
	bt_att_unref(att);
	return false;
}

void BleUartServer::destroyGattServer()
{
  ::printf("BLE: destroyGattServer\n");
	bt_gatt_server_unref(gatt);
	gatt_db_unref(db);
}


void BleUartServer::run(){
	::printf("BLE: run\n");
	rxWritePos = rxReadPos = 0;                               // initialize the circular buffer  
	txWritePos = txReadPos = 0;                               // initialize the circular buffer  
	msrmt_enabled = false;
	verbose = false;
	mtu = 0;	
	sec = BT_SECURITY_LOW;
	src_type = BDADDR_LE_PUBLIC;  
	bdaddr_t bdaddr_any =  {{0, 0, 0, 0, 0, 0}};
	bacpy(&src_addr, &bdaddr_any);
	::printf("BLE: l2cap le att listen and accept\n");
	if (!listen()){
		::printf("BLE: Failed to accept L2CAP ATT connection\n");
		return;
	}
	mainloop_init();
	if (!createGattServer()){
		::printf("BLE: no server\n");
		close(fd);
		return;
	}
	::printf("BLE: Running GATT server\n");
	print_prompt();
	mainloop_run_with_signal(signal_cb, NULL);
	::printf("BLE: Shutting down...\n");
	destroyGattServer();
}

BleUartServer::BleUartServer(){ 
	//printf("Before Thread\n");
    txMutex = PTHREAD_MUTEX_INITIALIZER;
	rxMutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_create(&thread_id, NULL, bleThreadFun, (void*)this);
    //pthread_join(thread_id, NULL);
    //printf("After Thread\n");
};
    

//BleUartServer BLE;




