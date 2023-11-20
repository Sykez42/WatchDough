/*
 * MQTT_handler.c
 *
 *  Created on: 20 nov 2023
 *      Author: merca
 */


#include "MQTT_handler.h"

uint16_t nmsg = 0;
Network network;

MQTTClient client;

MQTTConnackData connackdata;
MQTTSubackData subData;

MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;  //MQTTConnect.h
connectData.clientID.cstring = "WatchDough_GM";
connectData.willFlag = 0;
connectData.keepAliveInterval = 10;
connectData.cleansession = 1;

void messageArrived(MessageData* data)
{
	snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"Message arrived on topic %s: \"%s\"\r\n", data->topicName->lenstring.data,(char *) data->message->payload);
	HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
}


void connection_init(){
	//Network and MQTT Initialization

	NetworkInit(&network); //MQTTBL475EIOTA2 driver

	if(WIFI_Init() ==  WIFI_STATUS_OK){
		snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> WIFI Module Initialized.\r\n");
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);

		if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK){
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\r\r\n",
			MAC_Addr[0],MAC_Addr[1],MAC_Addr[2],MAC_Addr[3],MAC_Addr[4],MAC_Addr[5]);
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
		}
		else{
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> ERROR : CANNOT get MAC address\r\r\n");
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
				}
	}

	else{
		snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> ERROR : WIFI Module cannot be initialized.\r\n");
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}

	WIFI_Status_t retwifi = ClientJoinWiFiNetwork(SSID, PASSWORD);

	if(retwifi!=WIFI_STATUS_OK){
		snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> retwifi=%x.\r\n",retwifi);
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}
	else{
	    snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> WIFI Module Initialized.\r\n");
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}


	while (connTrials--){
		WIFI_Status_t retcon = ClientConnectNViaDNS(&network, myaddress, myport);
		if(retcon!=WIFI_STATUS_OK){
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> retcon=%x.\r\n",retcon);
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}
		else{
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"> Connected to broker TCP port\r\n");
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
			break;
			}
		HAL_Delay(2000);
	}

	MQTTClientInit(&client, &network, WIFI_WRITE_TIMEOUT, WiFiTxData, WIFI_TX_MAXSIZE, WiFiRxData, WIFI_TX_MAXSIZE);


	if ((rc = MQTTConnectWithResults(&client, &connectData, &connackdata)) != 0){
		snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"Error in connection: return code is %d\r\n", rc);
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}
	else{
		snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"MQTT Connected\r\n");
		HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	}


}

void subscribe_and_check(const char subtopic){

	if((rc = MQTTSubscribeWithResults(&client, subtopic, 2, messageArrived, &subData)) != 0){
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"Error in subscription: return code is %d\r\n", rc);
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
		}
		else{
			snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"MQTT topic %s subscribed\r\n",subtopic);
			HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
		}


}

void public_on_topic(MQTTMessage message, void *payload, int qos, int retained, const char pubtopic){

	message.qos = qos;
	message.retained = retained;
	message.payload = payload;
	sprintf(payload, "message number %d", ++nmsg);
	message.payloadlen = strlen(payload);
	snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"Publishing message \"%s\"\r\n", payload);
	HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	rc = MQTTPublish(&client, pubtopic, &message);
	snprintf((char*)msgseriale,SERIAL_MSG_MAXSIZE,"Return code from MQTT publish is %d\r\n", rc);
	HAL_UART_Transmit(&huart1,msgseriale,strlen((char*)msgseriale),1000);
	HAL_Delay(6000);

}
