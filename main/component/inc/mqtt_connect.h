/*
 * mqtt.h
 */
#ifndef MAIN_MQTT_APP_H_
#define MAIN_MQTT_APP_H_
#include "mqtt_client.h"

#define WIFI_SSID      "1111"
#define WIFI_PASS      "01245678"

#define MAX_LENGTH_TOPIC 30
#define MAX_LENGTH_DATA 10

#define ID1_TOPIC_PUB 		    "ID/ID1"
#define ID2_TOPIC_PUB 		    "ID/ID2"
#define ID3_TOPIC_PUB 		    "ID/ID3"
#define CONNECT_TOPIC_PUB 		"Status/Connected"
#define DISCONNECT_TOPIC_PUB 	"Status/Disconnected"

#define TOPIC_SUB 		        "ChangeID/#"
#define ID1_TOPIC_SUB 		    "ChangeID/ID1"
#define ID2_TOPIC_SUB 		    "ChangeID/ID2"
#define ID3_TOPIC_SUB 		    "ChangeID/ID3"
#define CHANGE_MODE_TOPIC_SUB   "ChangeMode"
/**
 * Mqtt config
 */
#define MQTT_ADDRESS 		"mqtt.flespi.io"
// #define MQTT_ADDRESS 		"192.168.137.154"
#define MQTT_PORT 		1883

typedef struct data_t{   
    char* topic;
    int topic_len;
    char* data;
    int data_len;
} data_t;

typedef struct MQTT_Handler_Struct{   
    esp_mqtt_client_handle_t client;
    esp_mqtt_client_config_t* mqtt_cfg;    
} MQTT_Handler_Struct;

void receive_mqtt_data(data_t );
void change_mode_WtoB_task(void* );
void log_error_if_nonzero(const char *, int );
void mqtt_event_handler(void *, esp_event_base_t , int32_t , void *);
bool mqtt_client_publish(MQTT_Handler_Struct* , char* , char *);
void mqtt_init_start(MQTT_Handler_Struct* );
void mqtt_receive_task(void*);
void mqtt_stop_deinit(MQTT_Handler_Struct *);
#endif /* MAIN_MQTT_APP_H_ */













