/*
 * ble.h
 */
#ifndef MAIN_BLE_APP_H_
#define MAIN_BLE_APP_H_
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SVC_INST_ID                 0

/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,
    IDX_CHAR_CFG_C,

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,
    IDX_CHAR_CFG_D,
    
    HRS_IDX_NB,
};

/* The max length of characteristic value. When the gatt client write or prepare write,
*  the data length must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.
*/
#define ID_LEN          500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

void change_mode_BtoW_task(void* );
void show_bonded_devices(void);
void gap_event_handler(esp_gap_ble_cb_event_t , esp_ble_gap_cb_param_t *);
void example_prepare_write_event_env(esp_gatt_if_t , prepare_type_env_t *, esp_ble_gatts_cb_param_t *);
void example_exec_write_event_env(prepare_type_env_t *, esp_ble_gatts_cb_param_t *);
void gatts_profile_event_handler(esp_gatts_cb_event_t , esp_gatt_if_t , esp_ble_gatts_cb_param_t *);
void gatts_event_handler(esp_gatts_cb_event_t , esp_gatt_if_t , esp_ble_gatts_cb_param_t *);
void ble_init_enable(void);
void ble_disable_deinit(void);
#endif