/*
 * MQTT_handler.h
 *
 *  Created on: 20 nov 2023
 *      Author: merca
 */

#ifndef INC_MQTT_HANDLER_H_
#define INC_MQTT_HANDLER_H_

#include "commons.h"
#include "MQTTClient.h"

const char myaddress[] = "test.mosquitto.org";
uint16_t myport = 1883;

const char pubtopic_temperature[] = "watchdough/temp";
const char pubtopic_humidity[] = "watchdough/hum";
const char subtopic_mode[] = "watchdough/mode";  //AUTO 1 ; MANUAL 0
const char subtopic_lamp[]="watchdough/lamp"; //LAMP ON 1; LAMP OFF 0

char payload[MQTT_PAYLOAD_SIZE];
uint8_t WiFiRxData[WIFI_RX_MAXSIZE];
uint8_t WiFiTxData[WIFI_TX_MAXSIZE];
uint8_t msgseriale[SERIAL_MSG_MAXSIZE];

//functions

void messageArrived(MessageData* data); //debug function that prints info about received MQTT messages
void connection_init(); //initializes WIFI connection and MQTT
void subscribe_and_check(const char subtopic); //subscribes and checks subscription
void public_on_topic();

#endif /* INC_MQTT_HANDLER_H_ */
