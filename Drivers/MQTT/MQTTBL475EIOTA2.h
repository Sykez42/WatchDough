/*
 * MQTTBL475EIOTA2.h
 *
 *  Created on: 23 mag 2020
 *      Author: Nick
 */

#ifndef INC_MQTTBL475EIOTA2_H_
#define INC_MQTTBL475EIOTA2_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include "wifi.h"
#include "stm32l475e_iot01.h"
#include "stdio.h"
#include "stdint.h"

#define MYTLS_SEND_DEFAULTINTERVAL 			10000
#define MYTLS_RECV_DEFAULTINTERVAL 			10000
#define MYTLS_MAXTIMEOUT		   			32767
#define MY_ES_WIFI_MAX_SINGLE_READ_LEN		ES_WIFI_PAYLOAD_SIZE
#define MY_ES_WIFI_MAX_SINGLE_SEND_LEN		ES_WIFI_PAYLOAD_SIZE
#define MY_ES_WIFI_MAX_PAYLOAD				ES_WIFI_DATA_SIZE-800


typedef struct Timer Timer;

struct Timer {
	unsigned long systick_period;
	unsigned long end_time;
};

typedef struct Network Network;

struct Network
{
	uint32_t my_socket;
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
	void (*disconnect) (Network*);
};

char TimerIsExpired(Timer*);
void TimerCountdownMS(Timer*, unsigned int);
void TimerCountdown(Timer*, unsigned int);
int TimerLeftMS(Timer*);

void TimerInit(Timer*);

int bl475eiota2_read(Network* n, uint8_t* buffer, int len, int timeout_ms);
int bl475eiota2_write(Network* n, uint8_t* buffer, int len, int timeout_ms);
void bl475eiota2_clientDisconnect(Network* n);
void NetworkInit(Network* n);
WIFI_Status_t ClientJoinWiFiNetwork(const char* myssid, const char* mypassword);

WIFI_Status_t ClientConnectToIP(Network* n, uint8_t *myremoteipaddr, uint16_t myremoteport);
WIFI_Status_t ClientConnectNViaDNS(Network* n, const char *myremoteaddr, uint16_t myremoteport);
//int TLSConnectNetwork(Network*, char*, int, SlSockSecureFiles_t*, unsigned char, unsigned int, char);


#ifdef __cplusplus
}
#endif

#endif /* INC_MQTTBL475EIOTA2_H_ */
