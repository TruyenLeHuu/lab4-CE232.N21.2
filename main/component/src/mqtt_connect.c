#include <stdio.h>

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "wifi_connect.h"

#include "mqtt_connect.h"

#include "ble_connect.h"

#include "ssd1306.h"

#include "cJSON.h"


static const char *TAG = "mqtt connection";

uint8_t char1_value[];
uint8_t char2_value[];
uint8_t char3_value[];

MQTT_Handler_Struct mqtt_h;

void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
void change_mode_WtoB_task(void* arg)
{
    mqtt_stop_deinit(&mqtt_h);

    wifi_stop_deinit();

    ble_init_enable();

    vTaskDelete(NULL);
};
void receive_mqtt_data(data_t data)
{
    ESP_LOGW(TAG,"TOPIC = %s",  data.topic);
    ESP_LOGW(TAG,"TOPIC_LENGTH = %d",  data.topic_len);
    ESP_LOGW(TAG,"DATA = %s",   data.data);
    ESP_LOGW(TAG,"DATA_LENGTH = %d",  data.data_len);
    char topic[MAX_LENGTH_TOPIC];
    char data_send[MAX_LENGTH_DATA];
    snprintf(topic, MAX_LENGTH_TOPIC, "%.*s", data.topic_len, data.topic);
    snprintf(data_send, MAX_LENGTH_DATA, "%.*s", data.data_len, data.topic + data.topic_len);
    if (strcmp(topic, ID1_TOPIC_SUB) == 0)
    {
        strcpy((char*)char1_value, data_send);
    } else if (strcmp(topic, ID2_TOPIC_SUB) == 0)
    {
        strcpy((char*)char2_value, data_send);
    } else if (strcmp(topic, ID3_TOPIC_SUB) == 0)
    {
        strcpy((char*)char3_value, data_send);
    } else if (strcmp(topic, CHANGE_MODE_TOPIC_SUB) == 0 && strcmp(data_send, "Yes") == 0)
    {
        xTaskCreatePinnedToCore(change_mode_WtoB_task, "change_mode_WtoB_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    }
    // ssd1306_display_ID(char1_value, char2_value, char3_value);
}
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    MQTT_Handler_Struct *mqtt_t = (MQTT_Handler_Struct *)handler_args;
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, TOPIC_SUB, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, CHANGE_MODE_TOPIC_SUB, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        mqtt_client_publish(mqtt_t, ID1_TOPIC_PUB, (char*) char1_value);
        mqtt_client_publish(mqtt_t, ID2_TOPIC_PUB, (char*) char2_value);
        mqtt_client_publish(mqtt_t, ID3_TOPIC_PUB, (char*) char3_value);
        mqtt_client_publish(mqtt_t, CONNECT_TOPIC_PUB, "Esp32");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED: 
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        data_t data;
        data.topic = event->topic;
        data.data = event->data;
        data.data_len = event->data_len;
        data.topic_len = event->topic_len;
        receive_mqtt_data(data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
bool mqtt_client_publish(MQTT_Handler_Struct *mqtt_t, char *topic, char *publish_string)
{
    if (mqtt_t->client)
    {
        int msg_id = esp_mqtt_client_publish(mqtt_t->client, topic, publish_string, 0, 1, 0);
        ESP_LOGI(TAG, "Sent publish returned msg_id=%d", msg_id);
        return true;
    }
    return false;
}
void mqtt_init_start(MQTT_Handler_Struct *mqtt_t)
{
    mqtt_t->client = esp_mqtt_client_init(mqtt_t->mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_t->client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_t);
    esp_mqtt_client_start(mqtt_t->client);
}
void mqtt_stop_deinit(MQTT_Handler_Struct *mqtt_t)
{
    esp_mqtt_client_disconnect(mqtt_t->client);
    esp_mqtt_client_stop(mqtt_t->client);
}
