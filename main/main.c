/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/********************************************************************************
*
* This file is for gatt server. It can send adv data, and get connected by client.
*
*********************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "sdkconfig.h"
#include "ssd1306.h"

#include "ble_connect.h"

#include "mqtt_connect.h"

#include "wifi_connect.h"

esp_mqtt_client_config_t mqtt_cfg_t = {
            .host = MQTT_ADDRESS,
            .port = MQTT_PORT,
            .username = "eXUeRHMZWmaE7CqW9nj2Peio1iKtDrNFdyHS3jGNtGgq6wa6KjYpn5CdbcuCs87v",
            .password = "",
            .client_id = "ESP32",
            .lwt_topic = DISCONNECT_TOPIC_PUB,
            .lwt_msg = "Esp32",
            .lwt_msg_len = 0,
            .lwt_qos = 1,
            .lwt_retain = 0
};
extern MQTT_Handler_Struct mqtt_h =
{
    .mqtt_cfg = &mqtt_cfg_t,
};

void app_main(void)
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    /* Initialize the underlying TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());
    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* Creates default WIFI STA */
    esp_netif_create_default_wifi_sta();
    /* Release bt controller memory */ 
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    /* Initialize ble */
    ble_init_enable();
}
