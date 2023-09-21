/*
 * MQTTBL475EIOTA2.c
 *
 *  Created on: 23 mag 2020
 *      Author: Nick
 */


#include "MQTTBL475EIOTA2.h"



unsigned long MilliTimer;

void SysTickIntHandler(void)
{
	MilliTimer++;
}

char TimerIsExpired(Timer* timer)
{
	long left = timer->end_time - MilliTimer;
	return (left < 0);
}


void TimerCountdownMS(Timer* timer, unsigned int timeout)
{
	timer->end_time = MilliTimer + timeout;
}


void TimerCountdown(Timer* timer, unsigned int timeout)
{
	timer->end_time = MilliTimer + (timeout * 1000);
}


int TimerLeftMS(Timer* timer)
{
	long left = timer->end_time - MilliTimer;
	return (left < 0) ? 0 : left;
}


void TimerInit(Timer* timer)
{
	timer->end_time = 0;
}


/**
  * @brief  EXTI line detection callback.
  * @param  n: Pointer to Network structure, as defined in the header file.
  * @retval None
  */
int bl475eiota2_read(Network* n, uint8_t* buffer, int len, int timeout_ms) {
	WIFI_Status_t ret;
	uint16_t recvLen = 0;
	if( len>MY_ES_WIFI_MAX_PAYLOAD )
	{
		printf("ERROR: the requested read is greater than the maximum allowed buffer ()!\nSkipping receive command");
		return -1;
	}
	size_t residual_recvLen = len;
	size_t iter_recvLen = 0;
	size_t already_recv = 0;
	unsigned char *curr_buf = buffer;
	while(residual_recvLen>0)
	{
		recvLen = 0;
		// Limit read to MY_ES_WIFI_MAX_PAYLOAD bytes
		iter_recvLen = (residual_recvLen>MY_ES_WIFI_MAX_SINGLE_READ_LEN) ? MY_ES_WIFI_MAX_SINGLE_READ_LEN : residual_recvLen;
		ret = WIFI_ReceiveData((uint8_t) n->my_socket, curr_buf, (uint16_t) iter_recvLen, &recvLen, timeout_ms);
		if(ret!=WIFI_STATUS_OK || ((recvLen==0) && (residual_recvLen>0)))
		{
			return -1;
		}
		already_recv += recvLen;
		residual_recvLen -= recvLen;
		curr_buf += recvLen;
	}
	return (int) already_recv;
}


int bl475eiota2_write(Network* n, uint8_t* buffer, int len, int timeout_ms) {
	WIFI_Status_t ret;
  	uint16_t sentLen = 0;
  	if( len>MY_ES_WIFI_MAX_PAYLOAD )
	{
		printf("ERROR: the requested read is greater than the maximum allowed buffer ()!\nSkipping receive command");
		return -1;
	}
	size_t residual_sentLen = len;
	size_t iter_sentLen = 0;
	size_t already_sent = 0;
	unsigned char *curr_buf = (unsigned char *) buffer;
	while(residual_sentLen>0)
	{
		sentLen = 0;
		// Limit read to MY_ES_WIFI_MAX_PAYLOAD bytes
		iter_sentLen = (residual_sentLen>MY_ES_WIFI_MAX_SINGLE_SEND_LEN) ? MY_ES_WIFI_MAX_SINGLE_SEND_LEN : residual_sentLen;
		ret = WIFI_SendData((uint8_t) n->my_socket, curr_buf, (uint16_t) iter_sentLen, &sentLen, timeout_ms);
		if(ret!=WIFI_STATUS_OK)
		{
			return -1;
		}
		already_sent += sentLen;
		residual_sentLen -= sentLen;
		curr_buf += sentLen;
	}
	return (int) already_sent;
}


void bl475eiota2_clientDisconnect(Network* n) {
	WIFI_CloseClientConnection(n->my_socket);
}


void NetworkInit(Network* n) {
	n->my_socket = (uint32_t) 0;
	n->mqttread = bl475eiota2_read;
	n->mqttwrite = bl475eiota2_write;
	n->disconnect = bl475eiota2_clientDisconnect;
}

/*
int TLSConnectNetwork(Network *n, char* addr, int port, SlSockSecureFiles_t* certificates, unsigned char sec_method, unsigned int cipher, char server_verify) {
	SlSockAddrIn_t sAddr;
	int addrSize;
	int retVal;
	unsigned long ipAddress;

	retVal = sl_NetAppDnsGetHostByName(addr, strlen(addr), &ipAddress, AF_INET);
	if (retVal < 0) {
		return -1;
	}

	sAddr.sin_family = AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)port);
	sAddr.sin_addr.s_addr = sl_Htonl(ipAddress);

	addrSize = sizeof(SlSockAddrIn_t);

	n->my_socket = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
	if (n->my_socket < 0) {
		return -1;
	}

	SlSockSecureMethod method;
	method.secureMethod = sec_method;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
	if (retVal < 0) {
		return retVal;
	}

	SlSockSecureMask mask;
	mask.secureMask = cipher;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(mask));
	if (retVal < 0) {
		return retVal;
	}

	if (certificates != NULL) {
		retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES, certificates->secureFiles, sizeof(SlSockSecureFiles_t));
		if(retVal < 0)
		{
			return retVal;
		}
	}

	retVal = sl_Connect(n->my_socket, ( SlSockAddr_t *)&sAddr, addrSize);
	if( retVal < 0 ) {
		if (server_verify || retVal != -453) {
			sl_Close(n->my_socket);
			return retVal;
		}
	}

	SysTickIntRegister(SysTickIntHandler);
	SysTickPeriodSet(80000);
	SysTickEnable();

	return retVal;
}
*/

WIFI_Status_t ClientJoinWiFiNetwork(const char* myssid, const char* mypassword)
{
	WIFI_Status_t ret = WIFI_Connect(myssid, mypassword, WIFI_ECN_WPA2_PSK);
	return ret;
}



WIFI_Status_t ClientConnectToIP(Network* n, uint8_t *myremoteipaddr, uint16_t myremoteport)
{
	WIFI_Status_t ret = WIFI_OpenClientConnection(n->my_socket, WIFI_TCP_PROTOCOL, \
									"TCP_CLIENT", myremoteipaddr, myremoteport, 0);
	return ret;
}

WIFI_Status_t ClientConnectNViaDNS(Network* n, const char *myremoteaddr, uint16_t myremoteport)
{
	uint8_t resolvedip[] = {0,0,0,0};
	WIFI_Status_t ret = WIFI_GetHostAddress(myremoteaddr, resolvedip);
	ret = WIFI_OpenClientConnection(n->my_socket, WIFI_TCP_PROTOCOL, \
									"TCP_CLIENT", resolvedip, myremoteport, 0);
	return ret;
}
