/*
 * commons.h
 *
 *  Created on: Oct 7, 2023
 *      Author: merca
 */

#ifndef INC_COMMONS_H_
#define INC_COMMONS_H_

#define LOCAL_TEMP_DELTA 5 //for board heating
#define LAMP_HYSTERESIS 2 //temperature hysteresis for lamps

//wifi and MQTT data

#define SSID     					"DojoGioGioCasaHouse"
#define PASSWORD 					"palle_555_666"
#define WIFI_WRITE_TIMEOUT 			10000
#define WIFI_READ_TIMEOUT  			10000
#define CONNECTION_TRIAL_MAX        10
#define SERIAL_MSG_MAXSIZE			240
#define WIFI_RX_MAXSIZE				140
#define WIFI_TX_MAXSIZE				140
#define MQTT_PAYLOAD_SIZE			30


//temperature handling using ints
struct temperature{

	int temp_int;
	int temp_dec;

};

#endif /* INC_COMMONS_H_ */
