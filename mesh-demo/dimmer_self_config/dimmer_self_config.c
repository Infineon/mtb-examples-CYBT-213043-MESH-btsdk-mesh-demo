/*
* Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
* Cypress Semiconductor Corporation. All Rights Reserved.
*
* This software, including source code, documentation and related
* materials ("Software"), is owned by Cypress Semiconductor Corporation
* or one of its subsidiaries ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*/

/** @file
 *
 * This demo application shows a simple dimmer implementation with self
 * configuration feature. The feature allows fast configuration of
 * the device. The SELF_CONFIG feature works in concert with the
 * embedded provisioner. For embedded provisioner, the configuration needs
 * to be hardcoded and then each device needs to be configured after
 * provisioning according to hardcoded configuration. For the self
 * configuration, the rules are hardcoded in a shared embedded_provisioner.h.
 * After provisioning is the provisioner sends new device the application key,
 * binds vendor specific model to the keys, and then sends vendor specific
 * command to self configure. The code snippets under #if defined(SELF_CONFIG)
 * can be copied to any application to allow the feature. To build the app
 * use SELF_CONFIG=1 in the build command line, or in the make file.  For example,
 *
 * demo.mesh.dimmer_self_config-CYW920719Q40EVB_01 SELF_CONFIG=1 download
 *
 * The app is based on the snip/mesh/mesh_level_client which implements
 * BLE Mesh Generic Level Client model.
 * Normally a dimmer has at least 2 buttons to turn the light on and off.
 * This application performs dimming using a single button available on the
 * EVK.  On a short push, the level is toggled between 0% and 100%.
 * When a button is pushed and not released, the level is changed
 * every 0.5 seconds from 0 to 100 in 8 steps 12.5% each.  The button
 * can be released before the level reaches 100% and in this case
 * consecutive push or long push will continue increasing the level.
 * If level reaches 100% the next button control will decrease the level.
 *
 * By default application does not support Relay, Proxy or Friend features
 * It can also be compiled to support a Low Power Node by adding
 * #define LOW_POWER_NODE  1
 *
 * Features demonstrated
 *  - Button usage on the EVK
 *  - Controlling of a BLE light bulb using BLE Mesh Set Level messages
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to the Mesh Evaluation Board /
 *    CYW920819EVB-02 Evaluation Kit
 * 2. Build and download Mesh_EmbeddedProvisioner application (Mesh Evaluation
 *    Board / CYW920819EVB-02 Evaluation Kit)
 * 3. The Embedded Provisioner will automatically provision and tell this app
 *    to self configure
 * 4. Push/release the application button on the dimmer.  The LED on the light
 *    side should turn on.
 * 5. Push/release the application button on the dimmer.  The LED on the light
 *    side should turn off.
 * 6. Push and hold the application button on the dimmer.  The LED on the light
 *    side should gradually go from off to on within 4 seconds.
 * 7. Push and hold the application button on the dimmer.  The LED on the light
 *    side should gradually go from on to off within 4 seconds.
 * 8. Try pushing and holding button for less than 4 seconds, and all other
 *    combinations.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_timer.h"
#include "button_control.h"
#include "wiced_memory.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_wdog.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_mesh_model_utils.h"
#include "wiced_bt_mesh_provision.h"
#include "mesh_application.h"
#if defined(SELF_CONFIG)
#include "embedded_provisioner.h"
#endif

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3005
#define MESH_VID                0x0002
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_app_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data);
#if defined(SELF_CONFIG)
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void self_configure(uint16_t node_addr);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
#if defined(SELF_CONFIG)
    // need those for self config as well to send messages to local device
    WICED_BT_MESH_MODEL_CONFIG_CLIENT,
    WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT,
#endif
//#if defined(REMOTE_PROVISION_SERVER_SUPPORTED)
//    WICED_BT_MESH_MODEL_REMOTE_PROVISION_SERVER,
//#endif
    WICED_BT_MESH_MODEL_LEVEL_CLIENT,
#if defined(SELF_CONFIG)
    { MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, mesh_vendor_server_message_handler, NULL, NULL },
#endif
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_LEVEL_CLIENT_ELEMENT_INDEX   0

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_LOW_POWER, // A bit field indicating the device features. In Low Power mode no Relay, no Proxy and no Friend
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window = 0,                                        // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 2,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 2,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 3,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 100,                               // Receive delay in 1ms units to be requested by the Low Power node.
        .poll_timeout          = 36000                              // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features           = 0,                                        // no, support for proxy, friend, or relay
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window         = 0,                                // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len          = 0,                                // Length of the buffer for the cache
        .max_lpn_num            = 0                                 // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#endif
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,          // application initialization
    button_hardware_init,   // hardware initialization
    NULL,                   // GATT connection status
    NULL,                   // attention processing
    NULL,                   // notify period set
    NULL,                   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

#if defined(SELF_CONFIG)
/*
 * if model is present in the device configuration, it will be subscribed to receive group multicasts
 */
uint16_t models_configured_for_sub[] =
{
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCENE_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_TIME_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SCHEDULER_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_HUE_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_SATURATION_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_TEMPERATURE_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ADMIN_PROPERTY_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_MANUFACT_PROPERTY_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_USER_PROPERTY_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_CLIENT_PROPERTY_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SETUP_SRV,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT,
};

/*
 * if model is present in the device configuration, it will be configured for publications
 */
uint16_t models_configured_for_pub[] =
{
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LEVEL_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_ONOFF_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_POWER_LEVEL_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_LOCATION_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LIGHTNESS_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_CTL_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_HSL_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_XYL_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_LIGHT_LC_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_CLNT,
    MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV,
};
static void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t* p_event, void* p_data);
static void configure_model_app_bind(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx);
static void configure_model_sub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t group_addr);
static void configure_model_pub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx, uint16_t pub_addr, uint32_t pub_period, uint8_t pub_ttl, uint8_t pub_rxmit_count, uint16_t pub_rxmit_interval, uint8_t pub_credentials);
static void configure_default_transition_time(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint16_t app_key_idx, uint32_t deftt);
static wiced_bt_mesh_event_t* mesh_configure_create_event(uint16_t dst, wiced_bool_t retransmit);
static void mesh_vendor_server_process_data(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
extern void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t* p_event, void* p_data);
#endif

#if defined(SELF_CONFIG)
void mesh_self_config_timer_cb(TIMER_PARAM_TYPE arg);
wiced_timer_t self_config_timer;
uint8_t self_configuration_done = WICED_FALSE;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    // Set Debug trace level for mesh_models_lib and mesh_provisioner_lib
    wiced_bt_mesh_models_set_trace_level(WICED_BT_MESH_CORE_TRACE_INFO);
#endif
#if 0
    // Set Debug trace level for all modules but Info level for CORE_AES_CCM module
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_ALL, WICED_BT_MESH_CORE_TRACE_DEBUG);
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_CORE_AES_CCM, WICED_BT_MESH_CORE_TRACE_INFO);
#endif
#if defined(SELF_CONFIG)
    WICED_BT_TRACE("Dimmer with Self Config\n");
    extern uint8_t wiced_bt_mesh_core_adv_tx_power;
    WICED_BT_TRACE("tx_power:%d to 0\n", wiced_bt_mesh_core_adv_tx_power);
    wiced_bt_mesh_core_adv_tx_power = 0;
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Dimmer";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_CONTROL_DEVICE_SLIDER;

    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len         = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data      = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len         = 2;
        buf[0]                         = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1]                         = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data      = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }
//#if defined(REMOTE_PROVISION_SERVER_SUPPORTED)
//    wiced_bt_mesh_remote_provisioning_server_init();
//#endif

#if defined(SELF_CONFIG)
    wiced_bt_mesh_config_client_init(mesh_config_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_default_transition_time_client_init(0, mesh_default_transition_time_client_message_handler, is_provisioned);
#endif
    button_control_init();
    wiced_bt_mesh_model_level_client_init(MESH_LEVEL_CLIENT_ELEMENT_INDEX, mesh_app_message_handler, is_provisioned);
}

/*
* Process event received from the Level Server.
*/
void mesh_app_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_level_status_data_t *p_data)
{
    WICED_BT_TRACE("level clt msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->status.tx_flag);
        break;

    case WICED_BT_MESH_LEVEL_STATUS:
        WICED_BT_TRACE("level present:%d target:%d remaining time:%d\n", p_data->present_level, p_data->target_level, p_data->remaining_time);
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

#if defined(SELF_CONFIG)
/*
 * This function is called when core receives a valid message for the define Vendor
 * Model (MESH_VENDOR_CYPRESS_COMPANY_ID/MESH_VENDOR_SELF_CONFIG_MODEL_ID) combination.  The function shall return TRUE if it
 * was able to process the message, and FALSE if the message is unknown.  In the latter case the core
 * will call other registered models.
 */
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    uint8_t buffer[8];
    uint8_t *p = buffer;
    uint16_t src;

    WICED_BT_TRACE("mesh_vendor_server_message_handler: opcode:%x model_id:%x\n", p_event->opcode, p_event->model_id);

    // 0xffff model_id means request to check if that opcode belongs to that model
    if (p_event->model_id == 0xffff)
    {
        return (p_event->opcode == MESH_VENDOR_CYPRSESS_OPCODE_CONFIG);
    }

    src = p_event->src;
    WICED_BT_TRACE("subcode:%x from:%04x\n", p_data[0], src);

    if ((p_event->opcode != MESH_VENDOR_CYPRSESS_OPCODE_CONFIG) || (p_data[0] != MESH_VENDOR_OPCODE_CONFIGURE_SET))
    {
        wiced_bt_mesh_release_event(p_event);
        return WICED_TRUE;
    }
    UINT8_TO_STREAM(p, MESH_VENDOR_OPCODE_CONFIGURE_STATUS);
    wiced_bt_mesh_models_utils_send(wiced_bt_mesh_create_reply_event(p_event), NULL, WICED_FALSE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint8_t)(p - buffer), NULL);

    // it can be a retransmission
    if (self_configuration_done)
        return WICED_TRUE;

    self_configuration_done = WICED_TRUE;

    self_configure(wiced_bt_mesh_core_get_local_addr());

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, src, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event)
    {
        p_event->retrans_cnt = 1;       // Try 1 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
    }
    p = buffer;
    UINT8_TO_STREAM(p, MESH_VENDOR_OPCODE_CONFIGURE_COMPLETE);
    wiced_bt_mesh_models_utils_send(p_event, NULL, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
    return WICED_TRUE;
}

void mesh_self_config_timer_cb(TIMER_PARAM_TYPE arg)
{
    uint16_t src = (uint16_t)arg;
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[30];
    uint8_t *p = buffer;

    self_configure(wiced_bt_mesh_core_get_local_addr());

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, src, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event)
    {
        p_event->retrans_cnt = 1;       // Try 1 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
    }
    UINT8_TO_STREAM(p, MESH_VENDOR_OPCODE_CONFIGURE_COMPLETE);

    wiced_bt_mesh_models_utils_send(p_event, NULL, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
}

/*
 * Process event received from the Configuration Server.
 */
void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t* p_event, void* p_data)
{
    switch (event)
    {
    default:
        WICED_BT_TRACE("ignored\n");
        wiced_bt_mesh_release_event(p_event);
        break;
    }
}

uint8_t cur_element_idx = 0;
uint8_t cur_model_idx = 0;

void self_configure_next_op(TIMER_PARAM_TYPE arg)
{
    int i, element_idx, model_idx;
    uint16_t node_addr = (uint16_t)arg;

    WICED_BT_TRACE("self_configure_next_op element:%d model:%d\n", cur_element_idx, cur_model_idx);

    for (element_idx = cur_element_idx; element_idx < mesh_config.elements_num; element_idx++)
    {
        wiced_bt_mesh_core_config_element_t* p_element = &mesh_config.elements[element_idx];

        for (model_idx = cur_model_idx; model_idx < p_element->models_num; model_idx++)
        {
            wiced_bt_mesh_core_config_model_t* p_model = &p_element->models[model_idx];

            if ((p_model->company_id == MESH_COMPANY_ID_UNUSED) ||
                ((p_model->company_id == MESH_COMPANY_ID_BT_SIG) &&
                 ((p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_SRV) ||
                  (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT) ||
                  (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_REMOTE_PROVISION_SRV) ||
                  (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_REMOTE_PROVISION_CLNT) ||
                  (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_DIRECTED_FORWARDING_SRV) ||
                  (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_DIRECTED_FORWARDING_CLNT))))
                continue;

            configure_model_app_bind(node_addr, p_model, element_idx, EMBEDDED_PROV_APP_KEY_IDX);

            // check if this model needs to be configured for subscriptions
            if (p_model->company_id != MESH_COMPANY_ID_BT_SIG)
            {
                configure_model_sub(node_addr, p_model, element_idx, EMBEDDED_PROV_GROUP_ADDR);
            }
            else
            {
                for (i = 0; i < sizeof(models_configured_for_sub) / sizeof(models_configured_for_sub[0]); i++)
                {
                    if ((p_model->company_id == MESH_COMPANY_ID_BT_SIG) &&
                        (p_model->model_id == models_configured_for_sub[i]))
                    {
                        configure_model_sub(node_addr, p_model, element_idx, EMBEDDED_PROV_GROUP_ADDR);
                        break;
                    }
                }
            }

            // check if this model needs to be configured for publication
            if (p_model->company_id != MESH_COMPANY_ID_BT_SIG)
            {
                configure_model_pub(node_addr, p_model, element_idx, EMBEDDED_PROV_APP_KEY_IDX, EMBEDDED_PROV_GROUP_ADDR, EMBEDDED_PROV_PUB_PERIOD, EMBEDDED_PROV_PUB_TTL, EMBEDDED_PROV_PUB_REXMIT_COUNT, EMBEDDED_PROV_PUB_REXMIT_INTERVAL, EMBEDDED_PROV_PUB_CREDENTIALS);
            }
            else
            {
                for (i = 0; i < sizeof(models_configured_for_pub) / sizeof(models_configured_for_pub[0]); i++)
                {
                    if ((p_model->company_id == MESH_COMPANY_ID_BT_SIG) &&
                        (p_model->model_id == models_configured_for_pub[i]))
                    {
                        configure_model_pub(node_addr, p_model, element_idx, EMBEDDED_PROV_APP_KEY_IDX, EMBEDDED_PROV_GROUP_ADDR, EMBEDDED_PROV_PUB_PERIOD, EMBEDDED_PROV_PUB_TTL, EMBEDDED_PROV_PUB_REXMIT_COUNT, EMBEDDED_PROV_PUB_REXMIT_INTERVAL, EMBEDDED_PROV_PUB_CREDENTIALS);
                        break;
                    }
                }
            }

            if ((p_model->company_id == MESH_COMPANY_ID_BT_SIG) &&
                (p_model->model_id == WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_SRV))
            {
                configure_default_transition_time(node_addr + element_idx, p_model, EMBEDDED_PROV_APP_KEY_IDX, EMBEDDED_PROV_DEF_TRANSITION_TIME);
            }
            cur_model_idx++;
            wiced_start_timer(&self_config_timer, 10);
            WICED_BT_TRACE("done company:%04x model:%04x\n", p_model->company_id, p_model->model_id);
            return;
        }
        cur_model_idx = 0;
        WICED_BT_TRACE("done element:%d\n", element_idx);
    }
    if (node_addr == EMBEDDED_PROV_LOCAL_ADDR)
    {
        extern void utilslib_delayUs(UINT32 delay);

        // Delay 100ms and reboot
        for (i = 0; i < 100; i++)
            utilslib_delayUs(1000);

        wiced_hal_wdog_reset_system();
    }
}

/*
 * Self provision to this network.
 */
void self_configure(uint16_t node_addr)
{
    cur_model_idx = 0;
    cur_element_idx = 0;
    wiced_init_timer(&self_config_timer, self_configure_next_op, (TIMER_PARAM_TYPE)node_addr, WICED_MILLI_SECONDS_TIMER);
    self_configure_next_op((TIMER_PARAM_TYPE)node_addr);
}

/*
 * Send Config Model App Bind message to local or remote device.
 */
void configure_model_app_bind(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx)
{
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_config_model_app_bind_data_t data;

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != wiced_bt_mesh_core_get_local_addr())) == NULL)
    {
        WICED_BT_TRACE("model app bind no mem\n");
        return;
    }
    memset(&data, 0, sizeof(data));

    data.operation = OPERATION_BIND;
    data.element_addr = node_addr + element_idx;
    data.company_id = p_model->company_id;
    data.model_id = p_model->model_id;
    data.app_key_idx = app_key_idx;

    wiced_bt_mesh_config_model_app_bind(p_event, &data);
}

/*
 * Send Config Model Subscription Add message to local or remote device.
 */
void configure_model_sub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t group_addr)
{
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_config_model_subscription_change_data_t data;

    WICED_BT_TRACE("Configure sub:%04x %04x\n", p_model->company_id, p_model->model_id);

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != wiced_bt_mesh_core_get_local_addr())) == NULL)
    {
        WICED_BT_TRACE("model sub no mem\n");
        return;
    }
    memset(&data, 0, sizeof(data));

    data.operation = OPERATION_ADD;
    data.element_addr = node_addr + element_idx;
    data.company_id = p_model->company_id;;
    data.model_id = p_model->model_id;
    data.addr[0] = group_addr & 0xff;
    data.addr[1] = (group_addr >> 8) & 0xff;

    wiced_bt_mesh_config_model_subscription_change(p_event, &data);
}

/*
 * Send Config Config Model Publication Set message to local or remote device.
 */
void configure_model_pub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx, uint16_t pub_addr, uint32_t pub_period, uint8_t pub_ttl, uint8_t pub_rxmit_count, uint16_t pub_rxmit_interval, uint8_t pub_credentials)
{
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_config_model_publication_set_data_t data;

    WICED_BT_TRACE("Configure pub:%04x %04x\n", p_model->company_id, p_model->model_id);

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != wiced_bt_mesh_core_get_local_addr())) == NULL)
    {
        WICED_BT_TRACE("model pub no mem\n");
        return;
    }
    memset(&data, 0, sizeof(data));

    data.element_addr = node_addr + element_idx;
    data.company_id = p_model->company_id;;
    data.model_id = p_model->model_id;
    data.publish_addr[0] = pub_addr & 0xff;
    data.publish_addr[1] = (pub_addr >> 8) & 0xff;
    data.app_key_idx = app_key_idx;
    data.publish_period = pub_period;
    data.publish_ttl = pub_ttl;
    data.publish_retransmit_count = pub_rxmit_count;
    data.publish_retransmit_interval = pub_rxmit_interval;
    data.credential_flag = pub_credentials;

    wiced_bt_mesh_config_model_publication_set(p_event, &data);
}

/*
 * Send Generic Default Transition Time Set message to local or remote device.
 */
void configure_default_transition_time(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint16_t app_key_idx, uint32_t deftt)
{
    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_default_transition_time_data_t data;

    data.time = deftt;

    p_event = wiced_bt_mesh_create_event(0, p_model->company_id, p_model->model_id, node_addr, app_key_idx);
    if (p_event == NULL)
        return;

    if (node_addr == wiced_bt_mesh_core_get_local_addr())
    {
        p_event->retrans_cnt = 4;       // Try 5 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
        p_event->reply_timeout = 80;    // wait for the reply 4 seconds
    }
    wiced_bt_mesh_model_default_transition_time_client_send_set(p_event, &data);
}

/*
 * Create event to send configuration message
 */
wiced_bt_mesh_event_t* mesh_configure_create_event(uint16_t dst, wiced_bool_t retransmit)
{
    wiced_bt_mesh_event_t* p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_CLNT, dst, 0xFFFF);
    if (p_event != NULL)
    {
        if (retransmit)
        {
            p_event->retrans_cnt = 4;       // Try 5 times (this is in addition to network layer retransmit)
            p_event->retrans_time = 20;     // Every 1 sec
            p_event->reply_timeout = 200;   // wait for the reply 10 seconds
        }
    }
    return p_event;
}
#endif
