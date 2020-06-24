/*
* Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * This demo application shows an implementation of a dimmable light which
 * also serves as an embedded provisioner and/or self configured device.
 * Behavior of the application depends on compilation flags
 * EMBEDDED_PROVISION and SELF_CONFIG.
 *
 * The SELF_CONFIG flag enables vendor specific functionality to speed up
 * configuration. Instead of waiting for all Model App bind, Model Sub,
 * Model Pub commands, the device publishes support for Self Config
 * Vendor Specific Model. When Set message is received, the device
 * binds all models to an app key, configures device for subscriptions and
 * publications.
 *
 * The EMBEDDED_PROVISION flag enables functionality to search for
 * unprovisioned devices, and then provision them and configured in to
 * the network.  The app uses Remote Provisioning Service feature to add
 * devices to the network. On startup the app goes through all network
 * devices and tells them to scan for unprovisioned devices. When a new device
 * is found by, the app starts a guard timeout to allow other provisioners
 * to report the same device and when timeout expires, the app selects the
 * best provisioner and performs provisioning using that remote provisioning
 * server.
 *
 * To decide if an unprovisioned device needs to be added to the network,
 * the app can either use RSSI or can use a magic number in the UUID. See
 * mesh_provsioner_validate_unprovision_device function in the mesh_scanner.
 * It is expected for a production app to update this function to use
 * preprogrammed list of UUIDs and some OOB data.
 *
 * After the new device is provisioned, the app performs configuration. To
 * do that the app adds an application key, binds vendor specific model to
 * this application key and then sends the command to self configure. The
 * configuration values are hardcoded in the embedded_provisioner.h file.
 *
 * If application is build with both EMBEDDED_PROVISION and SELF_CONFIG
 * compilation flags set, the device would start up in SELF_CONFIG mode.
 * On a button push, the device becomes an embedded provisioner and
 * will stay in this role until factory reset. A network should only have a
 * single  embedded provisioner.
 *
 * The app is based on the snip/mesh/mesh_light_lightness sample which
 * implements BLE Mesh Light Lightness Server model. Because Light Lightness
 * Server model extends Generic OnOff and Generic Level, the dimmable
 * light can be controlled by a Switch (Generic OnOff Client), a Dimmer
 * (Generic Level Client), or by an application which implements Light
 * Lightness Client.  The WICED Mesh Models library takes care of the
 * translation of the OnOff and Level messages and the only messages
 * that the application layer needs to process is those of the Light
 * Lightness Model.
 *
 * Features demonstrated
 *  - Embedded Provisioner and Self Config vendor specific features
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download several applications (to the WICED boards)
 * 2. Build and download a controlling application (to another WICED board)
 *    (for example, apps/demo/dimmer_self_config project)
 * 3. Push the application button on one of the boards so that device becomes
 *    the provsioner
 * 3. This application on the provisioner board will detect unprovisioned
 *    devices, provision them into the network and configure.
 * 4. Push/release the application button on the dimmer_self_config app.
 *    The LEDs on other boards should turn on.
 */
#define RSSI_TEST           1
#define EMBEDDED_PROVISION  1
#define SELF_CONFIG         1

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_nvram.h"
#include "wiced_platform.h"
#include "led_control.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "wiced_hal_rand.h"
#include "wiced_hal_wdog.h"
#include "wiced_hal_nvram.h"
#include "wiced_firmware_upgrade.h"
#include "wiced_bt_mesh_model_utils.h"
#include "wiced_bt_mesh_provision.h"
#if defined(MESH_DFU_SUPPORTED)
#include "wiced_bt_mesh_dfu.h"
#endif
#include "mesh_application.h"
#include "embedded_provisioner.h"
#if defined(EMBEDDED_PROVISION)
#include "mesh_scanner.h"
#endif
#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

uint8_t app_key[16];

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x321F
#define MESH_VID                0x0002
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t* p_data, uint32_t length);
static void mesh_app_factory_reset(void);
static void mesh_app_attention(uint8_t element_idx, uint8_t time);
static void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data);
static void mesh_app_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_t *p_data);
#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void self_configure(uint16_t node_addr);
#endif
#if (defined(SELF_CONFIG) && defined(EMBEDDED_PROVISION))
void button_hardware_init(void);
void button_interrupt_handler(void* user_data, uint8_t pin);
#endif
/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]        = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]            = { 'A', '1', '9', 0 };
uint8_t mesh_system_id[8]                                                         = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
    // need those for self config as well to send messages to local device
    WICED_BT_MESH_MODEL_CONFIG_CLIENT,
    WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT,
#endif
#if defined(EMBEDDED_PROVISION)
    WICED_BT_MESH_MODEL_HEALTH_CLIENT,
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_CLIENT,
#endif
#if defined(REMOTE_PROVISION_SERVER_SUPPORTED)
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_SERVER,
#endif
#if defined(MESH_DFU_SUPPORTED)
    WICED_BT_MESH_MODEL_FW_DISTRIBUTOR_UPDATE_SERVER,
#endif
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SERVER,
#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
    { MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, mesh_vendor_server_message_handler, NULL, NULL },
#endif
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_LIGHT_LIGHTNESS_SERVER_ELEMENT_INDEX   0

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
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_RELAY,     // No friend no proxy for this app
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 20,                                // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
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
#if (defined(SELF_CONFIG) && defined(EMBEDDED_PROVISION))
    button_hardware_init,   // If device can be both provisioner and self config, app will use button push to set device as a provisioner
#else
    NULL,                   // default SDK button processing
#endif
    NULL,                   // GATT connection status
    mesh_app_attention,     // attention processing
    NULL,                   // notify period set
    mesh_app_proc_rx_cmd,   // WICED HCI command
    NULL,                   // LPN sleep
    mesh_app_factory_reset, // factory reset
};

uint8_t last_known_brightness = 0;
uint8_t attention_brightness = 0;
uint8_t attention_time = 0;

wiced_timer_t attention_timer;
static void attention_timer_cb(TIMER_PARAM_TYPE arg);

#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
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
extern void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t* p_event, void* p_data);
#endif

#if defined(SELF_CONFIG)
void self_configuration_execute(uint16_t addr);
wiced_timer_t self_config_timer;
uint8_t self_configuration_done = WICED_FALSE;
#endif

#if defined(EMBEDDED_PROVISION)
static void create_network(void);
static void configure_net_beacon_set(uint16_t node_addr, uint8_t beacon_state);
static void configure_app_key_add(uint16_t node_addr, uint16_t net_key_idx, uint8_t* p_app_key, uint16_t app_key_idx);
static void configure_model_app_bind(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx);
static void configure_model_sub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t group_addr);
static void configure_model_pub(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint8_t element_idx, uint16_t app_key_idx, uint16_t pub_addr, uint32_t pub_period, uint8_t pub_ttl, uint8_t pub_rxmit_count, uint16_t pub_rxmit_interval, uint8_t pub_credentials);
static void configure_default_transition_time(uint16_t node_addr, wiced_bt_mesh_core_config_model_t* p_model, uint16_t app_key_idx, uint32_t deftt);
static void configure_vs_self_config(uint16_t node_addr);
static void node_reset(uint16_t node_addr);

static void mesh_embedded_provisioner_scan_restart(TIMER_PARAM_TYPE arg);

static const uint8_t* embedded_prov_get_dev_key_callback(uint16_t addr, uint16_t* p_net_key_idx);
static void mesh_provisioner_process_provision_end(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_status_data_t* p_data);
static void mesh_provisioner_process_link_report(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_link_report_data_t* p_data);
static void mesh_configure_app_key_status(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_config_appkey_status_data_t* p_data);
static void mesh_configure_model_app_bind_status(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_config_model_app_bind_status_data_t* p_data);
static uint16_t select_node_to_reset(void);
static void mesh_node_reset_status(wiced_bt_mesh_event_t* p_event);
static void mesh_node_reset_failed(wiced_bt_mesh_event_t* p_event);

static wiced_bt_mesh_event_t* mesh_configure_create_event(uint16_t dst, wiced_bool_t retransmit);
static void mesh_provisioner_process_device_capabilities(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_provision_device_capabilities_data_t *p_data);
static void mesh_provisioner_process_device_get_oob_data(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_provision_device_oob_request_data_t *p_data);
extern int find_rpr_idx(uint16_t rpr_addr);

uint8_t static_oob[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
#endif

mesh_embedded_provisioner_state_t *p_provisioner_state = NULL;
extern wiced_bool_t mesh_config_client;

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
    int i;
#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
    wiced_result_t result;
    mesh_node_t node;

#if defined(SELF_CONFIG) && defined(EMBEDDED_PROVISION)
    WICED_BT_TRACE("Embedded provisioner and Self Config\n");
#elif defined(EMBEDDED_PROVISION)
    WICED_BT_TRACE("Embedded provisioner\n");
#elif defined(SELF_CONFIG)
    WICED_BT_TRACE("Self Config\n");
#endif
    extern uint8_t wiced_bt_mesh_core_adv_tx_power;
    WICED_BT_TRACE("tx_power:%d to 0\n", wiced_bt_mesh_core_adv_tx_power);
    wiced_bt_mesh_core_adv_tx_power = 0;
#endif
    WICED_BT_TRACE("%s %s\n", __DATE__, __TIME__);
#if 1
    // Set Debug trace level for mesh_models_lib and mesh_provisioner_lib
    wiced_bt_mesh_models_set_trace_level(WICED_BT_MESH_CORE_TRACE_INFO);
#endif
#if 1
    // Set Debug trace level for all modules but Info level for CORE_AES_CCM module
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_ALL, WICED_BT_MESH_CORE_TRACE_DEBUG);
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_CORE_AES_CCM, WICED_BT_MESH_CORE_TRACE_NO);
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Dimmable Light";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_LIGHT_CEILING;

    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        // If device can only be a provisioner, create the network and reboot.
#if defined(EMBEDDED_PROVISION) && !defined(SELF_CONFIG)
        p_provisioner_state->state = EMBEDDED_PROVISIONER_STATE_RESET;
        create_network();
        return;
#else
        // If device can is SELF CONFIG or it just starts as SELF CONFIG and can turn into
        // EMBEDDED PROVISIONER later on, it should start advertising as unprovisioned device and will
        // send connectable adverts with unprovisioned service. App can also provide a name and an appearance in the scan response.
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
#endif
    }
    led_control_init(LED_CONTROL_TYPE_LEVEL);

    wiced_init_timer(&attention_timer, attention_timer_cb, 0, WICED_SECONDS_PERIODIC_TIMER);

#if defined(REMOTE_PROVISION_SERVER_SUPPORTED)
    wiced_bt_mesh_remote_provisioning_server_init();
#endif

#if defined(MESH_DFU_SUPPORTED)
    wiced_bt_mesh_model_fw_update_server_init(0, is_provisioned);
    wiced_bt_mesh_model_fw_distribution_server_init();
    wiced_bt_mesh_model_blob_transfer_server_init(0);
#endif

#if (defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION))
    wiced_bt_mesh_config_client_init(mesh_config_client_message_handler, is_provisioned);
    wiced_bt_mesh_model_default_transition_time_client_init(0, mesh_default_transition_time_client_message_handler, is_provisioned);
#endif
#if defined(EMBEDDED_PROVISION)
    if (wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) == sizeof(node))
    {
        // Tell core that there is a callback to obtain the device key.
        wiced_bt_mesh_core_set_dev_key_callback(embedded_prov_get_dev_key_callback);

        wiced_bt_mesh_scanner_client_init(mesh_config_client_message_handler, is_provisioned);
        wiced_bt_mesh_provision_client_init(mesh_config_client_message_handler, is_provisioned);
        wiced_bt_mesh_client_init(mesh_config_client_message_handler, is_provisioned);
        wiced_bt_mesh_health_client_init(mesh_config_client_message_handler, is_provisioned);

        if ((p_provisioner_state = (mesh_embedded_provisioner_state_t*)wiced_memory_permanent_allocate(sizeof(mesh_embedded_provisioner_state_t))) == NULL)
            return;

        memset(p_provisioner_state, 0, sizeof(mesh_embedded_provisioner_state_t));
        for (i = EMBEDDED_PROV_NODE_ADDR_FIRST; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
        {
            if (wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t))
            {
                // If new node supports remote provisioning, add it to the list of scanners.
                // The first device (provisioner) shall support RPR
                if ((i == EMBEDDED_PROV_NODE_ADDR_FIRST) ||
                    ((node.uuid[4] & CY_MAGIC_RPR_SUPPORTED) != 0))
                    mesh_scanner_add_provisioner(node.addr);

                // Need to remember first available address for new device
                if (node.addr + node.num_elements > p_provisioner_state->first_free_node_addr)
                    p_provisioner_state->first_free_node_addr = node.addr + node.num_elements;
            }
        }
        mesh_scanner_scan_start(0);

        wiced_init_timer(&p_provisioner_state->scan_restart_timer, mesh_embedded_provisioner_scan_restart, 0, WICED_SECONDS_PERIODIC_TIMER);
        wiced_start_timer(&p_provisioner_state->scan_restart_timer, 260);
    }
#endif
    // Initialize Light Lightness Server and register a callback which will be executed when it is time to change the brightness of the bulb
    wiced_bt_mesh_model_light_lightness_server_init(MESH_LIGHT_LIGHTNESS_SERVER_ELEMENT_INDEX, mesh_app_message_handler, is_provisioned);
    mesh_config_client = WICED_FALSE;

#if defined(RSSI_TEST)
    if (is_provisioned)
        rssi_test_init();
#endif
}

/*
 * Mesh device is being factory reset. Clean up NVRAM used by the app.
 */
void mesh_app_factory_reset(void)
{
#if defined(EMBEDDED_PROVISION)
    int i;
    wiced_result_t result;

    // if it was a factory reset, make sure to clean up NVRAM
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        wiced_hal_delete_nvram(i, &result);
    }
#endif
}

/*
 * Mesh library requests to alert user for "time" seconds.
 */
void mesh_app_attention(uint8_t element_idx, uint8_t time)
{
    WICED_BT_TRACE("dimmable light attention:%d sec\n", time);

    // If time is zero, stop alerting and restore the last known brightness
    if (time == 0)
    {
        WICED_BT_TRACE("dimmable light attention stop\n");
        wiced_stop_timer(&attention_timer);
        led_control_set_brighness_level(last_known_brightness);
        return;
    }
    wiced_start_timer(&attention_timer, 1);
    attention_time = time;
    attention_brightness = (last_known_brightness != 0) ? 0 : 100;
    led_control_set_brighness_level(attention_brightness);
}

/*
 * Attention timer callback is executed every second while user needs to be alerted.
 * Just switch brightness between 0 and 100%
 */
void attention_timer_cb(TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("dimmable light attention timeout:%d\n", attention_time);
    if (attention_time == 0)
    {
        wiced_stop_timer(&attention_timer);
        return;
    }
    if (--attention_time == 0)
    {
        WICED_BT_TRACE("timer callback attention stop\n");
        wiced_stop_timer(&attention_timer);
        led_control_set_brighness_level(last_known_brightness);
        return;
    }
    attention_brightness = (attention_brightness == 0) ? 100 : 0;
    led_control_set_brighness_level(attention_brightness);
}

/*
 * Process event received from the models library.
 */
void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
    switch (event)
    {
    case WICED_BT_MESH_LIGHT_LIGHTNESS_SET:
        mesh_app_process_set_level(element_idx, (wiced_bt_mesh_light_lightness_status_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("dimmable light unknown msg:%d\n", event);
        break;
    }
}

/*
 * Command from the level client is received to set the new level
 */
void mesh_app_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_t *p_status)
{
    WICED_BT_TRACE("mesh light srv set level element:%d present actual:%d linear:%d remaining_time:%d\n",
        element_idx, p_status->lightness_actual_present, p_status->lightness_linear_present, p_status->remaining_time);

    last_known_brightness = (uint8_t)((uint32_t)p_status->lightness_actual_present * 100 / 65535);
    led_control_set_brighness_level(last_known_brightness);

    // If we were alerting user, stop it.
    wiced_stop_timer(&attention_timer);
}

#if defined(SELF_CONFIG)
void self_configuration_execute(uint16_t addr)
{
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[30];
    uint8_t *p = buffer;

    self_configure(wiced_bt_mesh_core_get_local_addr());

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, addr, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event)
    {
        p_event->retrans_cnt = 1;       // Try 1 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
    }
    UINT8_TO_STREAM(p, MESH_VENDOR_OPCODE_CONFIGURE_COMPLETE);

    wiced_bt_mesh_models_utils_send(p_event, NULL, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
}
#endif

#if defined(EMBEDDED_PROVISION)
void mesh_embedded_provisioner_scan_restart(TIMER_PARAM_TYPE arg)
{
    int i;

    WICED_BT_TRACE("scan restart\n");

    if (p_provisioner_state->state == EMBEDDED_PROVISIONER_STATE_RESET)
        return;

    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if ((p_provisioner_state->rpr_node[i].addr != 0) /* && (p_provisioner_state->rpr_node[i].rpr_scan_state != RPR_STATE_FAILED) */)
            p_provisioner_state->rpr_node[i].rpr_scan_state = RPR_STATE_IDLE;
    }
    mesh_scanner_scan_start(0);
    wiced_start_timer(&p_provisioner_state->scan_restart_timer, 260);
}
#endif

#if defined(EMBEDDED_PROVISION) || defined(SELF_CONFIG)
/*
 * Process event received from the Configuration Server.
 */
void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t* p_event, void* p_data)
{
#if defined(EMBEDDED_PROVISION)
    if (p_provisioner_state != NULL)
    {
        switch (event)
        {
        case WICED_BT_MESH_PROVISION_SCAN_STATUS:
            mesh_scanner_process_scan_status(p_event, (wiced_bt_mesh_provision_scan_status_data_t*)p_data);
            return;

        case WICED_BT_MESH_PROVISION_SCAN_REPORT:
            mesh_scanner_process_scan_report(p_event, (wiced_bt_mesh_provision_scan_report_data_t*)p_data);
            return;

        case WICED_BT_MESH_PROVISION_GET_OOB_DATA:
            mesh_provisioner_process_device_get_oob_data(p_event, (wiced_bt_mesh_provision_device_oob_request_data_t*)p_data);
            return;

        case WICED_BT_MESH_PROVISION_DEVICE_CAPABILITIES:
            mesh_provisioner_process_device_capabilities(p_event, (wiced_bt_mesh_provision_device_capabilities_data_t*)p_data);
            return;

        case WICED_BT_MESH_PROVISION_END:
            mesh_provisioner_process_provision_end(p_event, (wiced_bt_mesh_provision_status_data_t*)p_data);
            return;

        case WICED_BT_MESH_PROVISION_LINK_REPORT:
            mesh_provisioner_process_link_report(p_event, (wiced_bt_mesh_provision_link_report_data_t*)p_data);
            return;

        case WICED_BT_MESH_CONFIG_APPKEY_STATUS:
            mesh_configure_app_key_status(p_event, (wiced_bt_mesh_config_appkey_status_data_t*)p_data);
            return;

        case WICED_BT_MESH_CONFIG_MODEL_APP_BIND_STATUS:
            mesh_configure_model_app_bind_status(p_event, (wiced_bt_mesh_config_model_app_bind_status_data_t *)p_data);
            return;

        case WICED_BT_MESH_CONFIG_NODE_RESET_STATUS:
            mesh_node_reset_status(p_event);
            return;

        case WICED_BT_MESH_TX_COMPLETE:
            if ((p_event->status.tx_flag == TX_STATUS_FAILED) && (p_event->opcode == WICED_BT_MESH_CORE_CMD_NODE_RESET))
                mesh_node_reset_failed(p_event);
            break;
        }
    }
#endif
    WICED_BT_TRACE("config client event:%d ignored\n", event);
    wiced_bt_mesh_release_event(p_event);
}

#if defined(EMBEDDED_PROVISION)
/*
 * Process device capabilities received during provisioning
 */
void mesh_provisioner_process_device_capabilities(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_device_capabilities_data_t* p_data)
{
    uint16_t rpr_addr = p_event->src;
    wiced_bt_mesh_provision_start_data_t start;
    int i, j;
    wiced_bool_t new_node_idx = 0;

    wiced_bt_mesh_release_event(p_event);

    // ToDo verify caps

    // Find RPR which perform provisioning
    if ((i = find_rpr_idx(rpr_addr)) == -1)
    {
        WICED_BT_TRACE("!Error dev caps, no RRP\n");
        return;
    }

    if ((p_event = mesh_configure_create_event(rpr_addr, rpr_addr != EMBEDDED_PROV_LOCAL_ADDR)) == NULL)
    {
        WICED_BT_TRACE("device caps no mem\n");
        return;
    }
    start.addr = p_provisioner_state->first_free_node_addr;
    start.net_key_idx = 0;
    start.algorithm = 0;
    start.public_key_type = p_data->pub_key_type;    // use value passed in the capabilities
    start.auth_method = 0;
    start.auth_action = 0;
    start.auth_size = 0;

    p_provisioner_state->rpr_node[i].new_node_addr = p_provisioner_state->first_free_node_addr;
    p_provisioner_state->rpr_node[i].num_elements = p_data->elements_num;

    WICED_BT_TRACE("Provision Start Server:%04x addr:%04x pub_key_type:%x auth_method:%x auth_action:%d auth_size:%d\n", rpr_addr, p_provisioner_state->first_free_node_addr, start.public_key_type, start.algorithm, start.auth_method, start.auth_action, start.auth_size);

    // The node takes num_elements addresses
    p_provisioner_state->first_free_node_addr += p_data->elements_num;

    wiced_bt_mesh_provision_start(p_event, &start);
}

/*
 * Process device capabilities received during provisioning
 */
void mesh_provisioner_process_device_get_oob_data(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_provision_device_oob_request_data_t *p_data)
{
    WICED_BT_TRACE("\n\n#####OOB get: type:%d size:%d action:%d\n", p_data->type, p_data->size, p_data->action);
    if (p_data->type == WICED_BT_MESH_PROVISION_GET_OOB_TYPE_GET_STATIC)
    {
        wiced_bt_mesh_provision_set_oob(static_oob, sizeof(static_oob));
    }
    else
    {
        WICED_BT_TRACE("OOB type:%d not supported\n", p_data->type);
    }
}

/*
 * Processing of the Provision End. Save info in the NVRAM.
 * Note that we cannot do configuration until Link Report is received.
 */
void mesh_provisioner_process_provision_end(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_status_data_t* p_data)
{
    uint16_t rpr_addr = p_event->src;
    mesh_node_t node, temp_node;
    wiced_result_t result;
    mesh_unprov_node_t* p_unprov_node = NULL;
    int i;

    WICED_BT_TRACE("provision end from:%04x addr:%04x result:%d\n", rpr_addr, p_data->addr, p_data->result);
    wiced_bt_mesh_release_event(p_event);

    memset(&node, 0, sizeof(node));

    node.addr = p_data->addr;

    // Find RPR which perform provisioning, it will have number of elements reported with caps
    if ((i = find_rpr_idx(rpr_addr)) == -1)
    {
        WICED_BT_TRACE("ignored\n");
        return;
    }

    p_provisioner_state->rpr_node[i].rpr_link_state = (p_data->result == 0) ? RPR_STATE_DISCONNECTING : RPR_STATE_IDLE;

    // Each RPR can provision only 1 node at a time. Find the unprovision node object based on RPR address
    for (p_unprov_node = p_provisioner_state->p_unprov_node_first; p_unprov_node != NULL; p_unprov_node = p_unprov_node->p_next)
    {
        if (p_unprov_node->rpr_addr == rpr_addr)
        {
            break;
        }
    }
    if (p_unprov_node == NULL)
    {
        WICED_BT_TRACE("Error no unprov node\n");
        return;
    }
    if (p_data->result == 0)
    {
        node.num_elements = p_provisioner_state->rpr_node[i].num_elements;
        memcpy(node.dev_key, p_data->dev_key, sizeof(node.dev_key));

        // remember the uuid and rssi level at which each provisioner saw this node
        memcpy(node.uuid, p_unprov_node->uuid, sizeof(node.uuid));
        memcpy(node.rssi_table, p_unprov_node->rssi_table, sizeof(node.rssi_table));

        // in self_provision, by default if node supports relay it is set as a relay
        node.is_relay = ((node.uuid[4] & CY_MAGIC_RELAY_SUPPORTED) != 0);

        // Find location in the NVRAM and write new node info
        for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
        {
            if (wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&temp_node, &result) != sizeof(mesh_node_t))
            {
                if (wiced_hal_write_nvram(i, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
                {
                    WICED_BT_TRACE("write node failed id:%d result:%d\n", i, result);
                    return;
                }
                break;
            }
        }
        WICED_BT_TRACE("Added node:%04x by:%04x num_elements:%d relay:%d\n", node.addr, rpr_addr, node.num_elements, node.is_relay);
        WICED_BT_TRACE_ARRAY(node.uuid, sizeof(node.uuid), "uuid       ");
        WICED_BT_TRACE_ARRAY(node.dev_key, sizeof(node.dev_key), "dev key    ");
        WICED_BT_TRACE_ARRAY((uint8_t *)node.rssi_table, sizeof(node.rssi_table), "rssi table ");

//#if defined(RSSI_TEST)
//        // Set delay for 20sec just in case we find and provision more devices
//        rssi_test_start(0, 0, 0, 20000);
//#endif
    }
    mesh_scanner_provision_end(p_unprov_node);
}

/*
 * Processing Provisioning Link Report. Start configuration by adding application key.
 */
void mesh_provisioner_process_link_report(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_link_report_data_t* p_data)
{
    uint16_t rpr_addr = p_event->src;
    int i;

    WICED_BT_TRACE("provision link report from %04x status:%d state:%d reason:%d over_gatt:%d\n", rpr_addr, p_data->link_status, p_data->rpr_state, p_data->reason, p_data->over_gatt);
    wiced_bt_mesh_release_event(p_event);

    // Find RPR which perform provisioning
    // Looking for the link report after provision is completed
    if ((i = find_rpr_idx(rpr_addr)) == -1)
    {
        WICED_BT_TRACE("ignored\n");
        return;
    }
    if (p_provisioner_state->rpr_node[i].rpr_link_state != RPR_STATE_DISCONNECTING)
    {
        WICED_BT_TRACE("ignored state:%d\n", p_provisioner_state->rpr_node[i].rpr_link_state);
        return;
    }

    p_provisioner_state->rpr_node[i].rpr_link_state = RPR_STATE_IDLE;

    WICED_BT_TRACE("rpr_addr:%04x idx:%d new_node_addr:%04x\n", rpr_addr, i, p_provisioner_state->rpr_node[i].new_node_addr);

    configure_app_key_add(p_provisioner_state->rpr_node[i].new_node_addr, EMBEDDED_PROV_NET_KEY_IDX, (uint8_t*)wiced_bt_mesh_core_get_app_key(EMBEDDED_PROV_APP_KEY_IDX, WICED_FALSE), EMBEDDED_PROV_APP_KEY_IDX);
}

/*
 * Processing App Key Status message.
 */
void mesh_configure_app_key_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_config_appkey_status_data_t *p_data)
{
    int i;
    uint16_t src = p_event->src;
    mesh_node_t node;
    wiced_result_t result;

    wiced_bt_mesh_release_event(p_event);

    WICED_BT_TRACE("AppKey Status from:%04x status:%d NetKeyIdx:%x ApptKeyIdx:%x\n", src,
        p_data->status, p_data->net_key_idx, p_data->app_key_idx);

#if 0
    // This may be initial creation of the network
    if (p_provisioner_state->state == EMBEDDED_PROVISIONER_STATE_RESET)
    {
        self_configure(EMBEDDED_PROV_LOCAL_ADDR);
        return;
    }
#endif
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if ((wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t)) &&
            (node.addr == src))
        {
            wiced_bt_mesh_core_config_model_t model =
            {
                .company_id = MESH_COMPANY_ID_CYPRESS,
                .model_id = MESH_VENDOR_SELF_CONFIG_MODEL_ID,
            };

            configure_model_app_bind(node.addr, &model, 0, EMBEDDED_PROV_APP_KEY_IDX);
            break;
        }
    }
}

/*
 * Processing Model App Key Bind Status message.
 */
void mesh_configure_model_app_bind_status(wiced_bt_mesh_event_t *p_event, wiced_bt_mesh_config_model_app_bind_status_data_t *p_data)
{
    int i;
    uint16_t src = p_event->src;
    mesh_node_t node;
    wiced_result_t result;

    wiced_bt_mesh_release_event(p_event);

    WICED_BT_TRACE("Model App Bind Status from:%x status:%d Element:%d Model ID:%x AppKeyIdx:%x\n", src,
        p_data->status, p_data->element_addr, p_data->model_id, p_data->app_key_idx);

    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if ((wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t)) &&
            (node.addr == src))
        {
            configure_vs_self_config(node.addr);
            break;
        }
    }
}

/*
 * Processing Node Reset Status message.
 */
void mesh_node_reset_complete(uint16_t addr)
{
    int i;
    mesh_node_t node;
    wiced_result_t result;

    // Reset next node
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if ((wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t)) &&
            (node.addr == addr))
        {
            wiced_hal_delete_nvram(i, &result);
            break;
        }
    }
    // may need to reset another node
    if ((addr = select_node_to_reset()) != 0)
    {
        node_reset(addr);
        return;
    }
    // if here, this device is not a provisioner, or a provisioner with no other devices.
    mesh_application_factory_reset();
}

/*
 * Processing Node Reset Status message.
 */
void mesh_node_reset_status(wiced_bt_mesh_event_t* p_event)
{
    uint16_t src = p_event->src;

    wiced_bt_mesh_release_event(p_event);

    WICED_BT_TRACE("Node Reset Status from:%04x\n", src);
    mesh_node_reset_complete(src);
}

/*
 * Processing Node Reset Status message.
 */
void mesh_node_reset_failed(wiced_bt_mesh_event_t* p_event)
{
    uint16_t dst = p_event->dst;

    wiced_bt_mesh_release_event(p_event);

    WICED_BT_TRACE("Node Reset Failed from:%04x\n", dst);
    mesh_node_reset_complete(dst);
}

#endif
#endif
#if defined(EMBEDDED_PROVISION) || defined(SELF_CONFIG) || defined(RSSI_TEST)
/*
 * This function is called when core receives a valid message for the defined Vendor
 * Model (MESH_VENDOR_CYPRESS_COMPANY_ID/MESH_VENDOR_SELF_CONFIG_MODEL_ID) combination.  The function shall return TRUE if it
 * was able to process the message, and FALSE if the message is unknown.  In the latter case the core
 * will call other registered models.
 */
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    uint8_t buffer[8];
    uint8_t *p = buffer;
    uint16_t src;
    mesh_node_t node;
    int rpr_idx, i;
    wiced_result_t result;

    // 0xffff model_id means request to check if that opcode belongs to that model
    if (p_event->model_id == 0xffff)
        return ((p_event->company_id == MESH_VENDOR_CYPRESS_COMPANY_ID) && (p_event->opcode == MESH_VENDOR_CYPRSESS_OPCODE_CONFIG));

    WICED_BT_TRACE("vs process data from:%04x reply:%04x len:%d subcode:%d prov_state:%04x\n", p_event->src, p_event->reply, data_len, p_data[0], p_provisioner_state);
    src = p_event->src;

    // Because the same app publishes and subscribes the same model, the app receives messages that it
    // sent out.
    if (src == wiced_bt_mesh_core_get_local_addr())
    {
        wiced_bt_mesh_release_event(p_event);
        return WICED_TRUE;
    }

    // If provisioner, only process MESH_VENDOR_OPCODE_CONFIGURE_STATUS, otherwise only _SET
    switch (p_data[0])
    {
#if defined(EMBEDDED_PROVISION)
    case MESH_VENDOR_OPCODE_CONFIGURE_STATUS:
        if (p_provisioner_state == NULL)
            break;

        // If we are still retransmitting cancel transmit and release mesh event
        wiced_bt_mesh_models_utils_ack_received(&p_provisioner_state->p_out_event, p_event->src);
        break;

    case MESH_VENDOR_OPCODE_CONFIGURE_COMPLETE:
        if (p_provisioner_state == NULL)
            break;

        // If we are still retransmitting cancel transmit and release mesh event
        wiced_bt_mesh_models_utils_ack_received(&p_provisioner_state->p_out_event, p_event->src);

        // If new node supports remote provisioning, add it to the list of scanners
        // Find location in the NVRAM and write new node info
        for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
        {
            if ((wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t)) &&
                (node.addr == p_event->src))
            {
                if ((node.uuid[4] & CY_MAGIC_RPR_SUPPORTED) != 0)
                {
                    if ((rpr_idx = mesh_scanner_add_provisioner(node.addr)) != -1)
                    {
                        mesh_scanner_scan_start(rpr_idx);
                    }
                }
                break;
            }
        }
        break;
#endif
#if defined(SELF_CONFIG)
    case MESH_VENDOR_OPCODE_CONFIGURE_SET:
        if (p_provisioner_state != NULL)
            break;

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
#endif
#if defined(RSSI_TEST)
    case MESH_CY_VENDOR_SUBCODE_RSSI_TEST_TX_START:
        mesh_vendor_rssi_test_process_tx_start(p_event, p_data + 1, data_len - 1);
        return WICED_TRUE;

#if defined(EMBEDDED_PROVISION)
    case MESH_CY_VENDOR_SUBCODE_RSSI_TEST_TX_START_STATUS:
        mesh_vendor_rssi_test_process_tx_start_status(p_event, p_data + 1, data_len - 1);
        return WICED_TRUE;
#endif

    case MESH_CY_VENDOR_SUBCODE_RSSI_TEST_DATA:
        mesh_vendor_rssi_test_process_data(p_event, p_data + 1, data_len - 1);
        return WICED_TRUE;

    case MESH_CY_VENDOR_SUBCODE_RSSI_TEST_RX_VALUES_GET:
        mesh_vendor_rssi_test_process_rx_values_get(p_event, p_data + 1, data_len - 1);
        return WICED_TRUE;

#if defined(EMBEDDED_PROVISION)
    case MESH_CY_VENDOR_SUBCODE_RSSI_TEST_RX_VALUES_STATUS:
        mesh_vendor_rssi_test_process_rx_values_status(p_event, p_data + 1, data_len - 1);
        return WICED_TRUE;
#endif

#endif
    default:
        WICED_BT_TRACE("unknown subcode:%d\n", p_data[0]);
        break;
    }
    wiced_bt_mesh_release_event(p_event);
    return WICED_TRUE;
}
#endif

#if defined(EMBEDDED_PROVISION)
/*
 * Create network and self provision to this network.
 */
void create_network(void)
{
    wiced_bt_mesh_local_device_set_data_t set;
    mesh_node_t node;
    wiced_result_t result;
    // uint8_t app_key[16];

    memset(&set, 0, sizeof(set));

    set.addr = EMBEDDED_PROV_LOCAL_ADDR;

    *(uint32_t*)&set.dev_key[0] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.dev_key[4] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.dev_key[8] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.dev_key[12] = wiced_hal_rand_gen_num();

    *(uint32_t*)&set.network_key[0] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.network_key[4] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.network_key[8] = wiced_hal_rand_gen_num();
    *(uint32_t*)&set.network_key[12] = wiced_hal_rand_gen_num();

    set.net_key_idx = EMBEDDED_PROV_NET_KEY_IDX;

    *(uint32_t*)&app_key[0] = wiced_hal_rand_gen_num();
    *(uint32_t*)&app_key[4] = wiced_hal_rand_gen_num();
    *(uint32_t*)&app_key[8] = wiced_hal_rand_gen_num();
    *(uint32_t*)&app_key[12] = wiced_hal_rand_gen_num();

    WICED_BT_TRACE("create net addr:%x net_key_idx:%x iv_idx:%x key_refresh:%d iv_upd:%d model_access:%d\n", set.addr, set.net_key_idx, set.iv_idx, set.key_refresh, set.iv_update);

    wiced_bt_mesh_provision_local_device_set(&set);

    node.addr = EMBEDDED_PROV_LOCAL_ADDR;
    node.num_elements = mesh_config.elements_num;

    memcpy(node.dev_key, set.dev_key, sizeof(node.dev_key));

    if (wiced_hal_write_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
    {
        WICED_BT_TRACE("write node failed id:%d result:%d\n", EMBEDDED_PROV_NODE_ADDR_FIRST, result);
    }

    configure_app_key_add(node.addr, EMBEDDED_PROV_NET_KEY_IDX, app_key, EMBEDDED_PROV_APP_KEY_IDX);
#if 1
    self_configure(EMBEDDED_PROV_LOCAL_ADDR);
#endif
}

/*
 * Send Config AppKey Add message to local or remote device.
 */
void configure_app_key_add(uint16_t node_addr, uint16_t net_key_idx, uint8_t* p_app_key, uint16_t app_key_idx)
{
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_config_appkey_change_data_t data;

    WICED_BT_TRACE("app key add addr:%04x\n", node_addr);

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != EMBEDDED_PROV_LOCAL_ADDR)) == NULL)
    {
        WICED_BT_TRACE("add key no mem\n");
        return;
    }
    memset(&data, 0, sizeof(data));

    data.operation = OPERATION_ADD;
    data.app_key_idx = app_key_idx;
    data.net_key_idx = app_key_idx;
    memcpy(data.app_key, p_app_key, sizeof(data.app_key));

    wiced_bt_mesh_config_appkey_change(p_event, &data);
}

/*
 * Send vendor specific command to perform self configuration
 */
void configure_vs_self_config(uint16_t node_addr)
{
    uint8_t buffer[30];
    uint8_t *p = buffer;
    uint16_t u16;
    wiced_bt_mesh_event_t *p_event;

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, node_addr, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event != NULL)
    {
        p_event->reply = WICED_TRUE;
        p_event->retrans_cnt = 4;       // Try 5 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
        p_event->reply_timeout = 80;    // wait for the reply 4 seconds

        UINT8_TO_STREAM(p, MESH_VENDOR_OPCODE_CONFIGURE_SET);

        wiced_bt_mesh_models_utils_send(p_event, &p_provisioner_state->p_out_event, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
    }
}

/*
 * returns pointer to the dev_key of the node with address addr
 */
const uint8_t* embedded_prov_get_dev_key_callback(uint16_t addr, uint16_t* p_net_key_idx)
{
    int i;
    wiced_result_t result;
    mesh_node_t node;

    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if ((wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t)) &&
            (node.addr == addr))
        {
            if (p_net_key_idx)
                *p_net_key_idx = 0;

            WICED_BT_TRACE("dev key callback addr:%04x\n", addr);

            memcpy(p_provisioner_state->dev_key, node.dev_key, sizeof(node.dev_key));
            return p_provisioner_state->dev_key;
        }
    }
    WICED_BT_TRACE(" !dev key callback addr:%04x not found\n", addr);
    return NULL;
}
#endif
#if defined(SELF_CONFIG) || defined(EMBEDDED_PROVISION)

uint8_t cur_element_idx = 0;
uint8_t cur_model_idx = 0;

void self_configure_next_op(TIMER_PARAM_TYPE arg)
{
    int i, element_idx, model_idx;
    uint16_t node_addr = (uint16_t)arg;

    WICED_BT_TRACE("self_configure_next_op element:%d model:%d\n", cur_element_idx, cur_model_idx);

    configure_net_beacon_set(node_addr, 0);

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
 * Send Config Net Beacon Set message to local or remote device.
 */
void configure_net_beacon_set(uint16_t node_addr, uint8_t state)
{
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_config_beacon_set_data_t data;

    WICED_BT_TRACE("net beacon set addr:%04x state:%d\n", node_addr, state);

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != EMBEDDED_PROV_LOCAL_ADDR)) == NULL)
    {
        WICED_BT_TRACE("net beacon set no mem\n");
        return;
    }
    data.state = state;
    wiced_bt_mesh_config_beacon_set(p_event, &data);
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
        p_event->reply = WICED_TRUE;
        p_event->retrans_cnt = 4;       // Try 5 times (this is in addition to network layer retransmit)
        p_event->retrans_time = 10;     // Every 500 msec
        p_event->reply_timeout = 80;    // wait for the reply 4 seconds
    }
    wiced_bt_mesh_model_default_transition_time_client_send_set(p_event, &data);
}

/*
 * Send Node Reset to remote device.
 */
void node_reset(uint16_t node_addr)
{
    wiced_bt_mesh_event_t* p_event;

    WICED_BT_TRACE("node_reset:%04x\n", node_addr);

    if ((p_event = mesh_configure_create_event(node_addr, node_addr != wiced_bt_mesh_core_get_local_addr())) == NULL)
    {
        WICED_BT_TRACE("node reset no mem\n");
        return;
    }
    wiced_bt_mesh_config_node_reset(p_event);
}

int get_num_hops(uint16_t first_idx, uint16_t second_idx)
{
    int first_hop_idx;
    int num_hops = -1;
    int result = -1;
    mesh_node_t node;
    wiced_result_t res;

    if (wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST + second_idx, sizeof(mesh_node_t), (uint8_t*)&node, &res) == sizeof(mesh_node_t))
    {
        if (node.rssi_table[first_idx] != 0)
            result = 1;
        else
        {
            // node first_idx does not see directly second_idx.
            // Go through all nodes that first_idx see. One hop will be added.
            for (first_hop_idx = first_idx + 1; first_hop_idx < EMBEDDED_PROV_MAX_NODES; first_hop_idx++)
            {
                if (wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST + first_hop_idx, sizeof(mesh_node_t), (uint8_t*)&node, &res) != sizeof(mesh_node_t))
                    continue;

                if (node.rssi_table[first_idx] == 0)
                    continue;

                // first_idx has a path to i, check if i has a path to second_idx
                if ((num_hops = get_num_hops(first_hop_idx, second_idx)) == -1)
                    continue;

                // if i is num_hops away from the second_idx, that means that first_idx is num_hops+1 from second_idx
                num_hops++;

                // find the minimum num_hops
                if (result == -1)
                    result = num_hops;
                else if (result < num_hops)
                    result = num_hops;
            }
        }
    }
    WICED_BT_TRACE("get num hops from:%d to:%d result:%d\n", first_idx, second_idx, result);
    return result;
}

uint16_t select_node_to_reset(void)
{
    mesh_node_t node;
    wiced_result_t result;
    int i, num_hops, idx_max_hops = 0, max_hops = 0;

    // first check if there are any nodes that are not relays. Can safely delete them.
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if (wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t))
        {
            if (!node.is_relay)
                return node.addr;
        }
    }
    // select a node with max number of hops
    for (i = 1; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        num_hops = get_num_hops(0, i);
        if (num_hops > max_hops)
        {
            max_hops = num_hops;
            idx_max_hops = i;
        }
    }
    if (idx_max_hops != 0)
    {
        if (!wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST + idx_max_hops, sizeof(mesh_node_t), (uint8_t*)&node, &result))
        {
            WICED_BT_TRACE("wiced_hal_read_nvram failed in select_node_to_reset\n");
        }
        WICED_BT_TRACE("reset node selected idx:%d addr:%04x\n", idx_max_hops, node.addr);
        return node.addr;
    }
    return 0;
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
            p_event->reply = WICED_TRUE;
            p_event->retrans_cnt = 4;       // Try 5 times (this is in addition to network layer retransmit)
            p_event->retrans_time = 20;     // Every 1 sec
            p_event->reply_timeout = 200;   // wait for the reply 10 seconds
        }
    }
    return p_event;
}
#endif

#if (defined(SELF_CONFIG) && defined(EMBEDDED_PROVISION))
#if !defined(CYW43012C0) && !defined(CYW20706A2) && !defined(CYW20719B0)
extern wiced_platform_button_config_t platform_button[];
#endif

void button_hardware_init(void)
{
    /* Configure buttons available on the platform */
#if defined(CYW20706A2)
    wiced_hal_gpio_configure_pin(WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_BOTH_EDGE), WICED_GPIO_BUTTON_DEFAULT_STATE);
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, button_interrupt_handler, NULL);
#elif (defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW20721B0))
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON, button_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON, WICED_GPIO_BUTTON_SETTINGS, GPIO_PIN_OUTPUT_LOW);
#else
    wiced_platform_register_button_callback(WICED_PLATFORM_BUTTON_1, button_interrupt_handler, NULL, GPIO_EN_INT_BOTH_EDGE);
#endif
}

/*
 * Process interrupts from the button.
 */
void button_interrupt_handler(void* user_data, uint8_t pin)
{
    int i, j;
    wiced_result_t result;
    static uint64_t push_time;  //time of the push interrupt
    uint64_t curr_time;
    uint32_t value = wiced_hal_gpio_get_pin_input_status(pin);
    uint16_t addr;

    curr_time = wiced_bt_mesh_core_get_tick_count();
    WICED_BT_TRACE("interrupt_handler: pin:%d value:%d time:%d\n", pin, value, curr_time);

    // On push just remember current time.
#if !defined(CYW43012C0) && !defined(CYW20706A2) && !defined(CYW20719B0)
    if (value == platform_button[WICED_PLATFORM_BUTTON_1].button_pressed_value)
#else
    if (value == 0)
#endif
    {
        push_time = curr_time;
    }
    else if (curr_time >= (push_time + 3000))
    {
        // On release button with delay >= 3 seconds do factory reset.
        // If there are other nodes in the networks, reset them first.
        if (wiced_bt_mesh_core_get_local_addr() == EMBEDDED_PROV_LOCAL_ADDR)
        {
            if ((addr = select_node_to_reset()) != 0)
            {
                p_provisioner_state->state = EMBEDDED_PROVISIONER_STATE_RESET;
                node_reset(addr);
                return;
            }
        }
        // if here, this device is not a provisioner, or a provisioner with no other devices.
        mesh_application_factory_reset();
    }
    else
    {
        if (wiced_bt_mesh_core_get_local_addr() != 0)
        {
            WICED_BT_TRACE("Device already provisioned addr:%04x. To reset press and hold for 3 secs\n", wiced_bt_mesh_core_get_local_addr());
            return;
        }
        // When a button is pushed on the unprovisioned device, the device becomes Embedded Provisioner and creates network.
        p_provisioner_state->state = EMBEDDED_PROVISIONER_STATE_ACTIVE;
        create_network();
    }
}

#endif

extern wiced_transport_buffer_pool_t* host_trans_pool;

#if defined(MESH_DFU_SUPPORTED)
uint8_t fw_distribution_server_process_upload_start(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len);
uint8_t fw_distribution_server_process_nodes_add(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len);
uint8_t fw_distribution_server_process_distribution_start(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len);
void fw_distribution_server_blob_transfer_callback(uint16_t event, void* p_data);
uint8_t fw_distribution_server_get_upload_phase(void);
uint8_t mesh_fw_distribution_get_distribution_state(void);
wiced_bool_t fw_distribution_server_get_upload_fw_id(mesh_dfu_fw_id_t *p_fw_id);
void fw_update_client_send_status_complete_callback(wiced_bt_mesh_event_t *p_event);

/*
 * start distribution of the FW with FW_ID
 */
wiced_bool_t embedded_provisioner_start_dfu(mesh_dfu_fw_id_t *p_fw_id)
{
    mesh_node_t node;
    wiced_result_t result;
    uint8_t buffer[8 + sizeof(mesh_dfu_fw_id_t)];
    uint8_t* p = buffer;
    int i;
    wiced_bool_t node_found = WICED_FALSE;

    // delete all addresses from the distribution list to start clean
    // fw_distribution_server_process_nodes_delete_all(NULL, NULL, 0);

    // add all devices in the list (for the future, we may check the fw_id)
    for (i = EMBEDDED_PROV_NODE_ADDR_FIRST + 1; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
    {
        if (wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t))
        {
            buffer[0] = node.addr & 0xff;
            buffer[1] = (node.addr >> 8) & 0xff;
            fw_distribution_server_process_nodes_add(NULL, buffer, 2);
            node_found = WICED_TRUE;
        }
    }
    if (!node_found)
        return WICED_FALSE;

    p = buffer;
    UINT16_TO_STREAM(p, EMBEDDED_PROV_APP_KEY_IDX);
    UINT8_TO_STREAM(p, 0);
    UINT16_TO_STREAM(p, EMBEDDED_PROV_DISTR_NODE_TIMEOUT);
    UINT8_TO_STREAM(p, (WICED_BT_MESH_FW_TRANSFER_MODE_PUSH | (WICED_BT_MESH_FW_UPDATE_POLICY_VERIFY_AND_APPLY << 2) | (WICED_BT_MESH_FW_DISTRIBUTION_ADDRESS_MULTICAST << 3)));
    UINT16_TO_STREAM(p, 0xFFFF);    // group address is broadcast
    mesh_dfu_fw_id_to_data(p_fw_id, &p);

    if (fw_distribution_server_process_distribution_start(NULL, buffer, (uint16_t)(p - buffer)) == WICED_BT_MESH_FW_DISTR_STATUS_SUCCESS)
    {
        fw_update_client_send_status_complete_callback(NULL);
    }
    return WICED_TRUE;
}

void mesh_provisioner_hci_send_fw_distr_status(uint8_t status)
{
    uint8_t* p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    uint8_t* p = p_buffer;
    mesh_node_t node;
    wiced_result_t result;
    int num_nodes = 0;
    int i;

    if (p_buffer == NULL)
    {
        WICED_BT_TRACE("no buffer to send FW distr status\n");
    }
    else
    {
        UINT8_TO_STREAM(p, status);
        UINT8_TO_STREAM(p, fw_distribution_server_get_upload_phase());
        UINT8_TO_STREAM(p, mesh_fw_distribution_get_distribution_state());

        for (i = EMBEDDED_PROV_NODE_ADDR_FIRST; i < EMBEDDED_PROV_NODE_ADDR_LAST; i++)
        {
            if (wiced_hal_read_nvram(i, sizeof(mesh_node_t), (uint8_t*)&node, &result) == sizeof(mesh_node_t))
            {
                UINT16_TO_STREAM(p, node.addr);
                num_nodes++;
            }
        }
        result = mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_FW_DISTRIBUTION_UPLOAD_STATUS, p_buffer, (uint16_t)(p - p_buffer));
        WICED_BT_TRACE("sending status:%d phase:%d state:%d result:%d num_nodes:%d\n", p_buffer[0], p_buffer[1], p_buffer[2], result, num_nodes);
    }
}
#endif

void mesh_provisioner_hci_send_status(uint8_t status)
{
    uint8_t* p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    uint8_t* p = p_buffer;
    wiced_result_t result;

    if (p_buffer == NULL)
    {
        WICED_BT_TRACE("no buffer to send HCI status\n");
    }
    else
    {
        UINT8_TO_STREAM(p, status);
        result = mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_COMMAND_STATUS, p_buffer, (uint16_t)(p - p_buffer));
        WICED_BT_TRACE("sent HCI status:%d result:%d\n", p_buffer[0], result);
    }
}

#if defined(RSSI_TEST)
wiced_bool_t rssi_test_started_by_host = WICED_FALSE;

void rssi_test_send_hci_result(uint16_t src, uint16_t report_addr, uint16_t rx_count, int8_t rx_rssi)
{
    uint8_t* p_buffer;
    uint8_t* p;
    wiced_result_t result;

    if (!rssi_test_started_by_host)
        return;

    p_buffer = wiced_transport_allocate_buffer(host_trans_pool);
    p = p_buffer;

    if (p_buffer == NULL)
    {
        WICED_BT_TRACE("no buffer to send HCI status\n");
    }
    else
    {
        UINT16_TO_STREAM(p, src);
        UINT16_TO_STREAM(p, report_addr);
        UINT16_TO_STREAM(p, rx_count);
        UINT8_TO_STREAM(p, rx_rssi);
        result = mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_RSSI_TEST_RESULT, p_buffer, (uint16_t)(p - p_buffer));
        WICED_BT_TRACE("Sent RSSI Test result:%d\n", result);
    }
}
#endif

/*
 * In 2 chip solutions MCU can send commands to change provisioner state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t* p_data, uint32_t length)
{
    uint8_t status = 0;
#if defined(MESH_DFU_SUPPORTED)
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_blob_transfer_block_data_t data;
    wiced_bt_mesh_blob_transfer_finish_t finish;
    mesh_dfu_fw_id_t fw_id;
#endif
#ifdef RSSI_TEST
    uint16_t rssi_test_dst;
    uint16_t rssi_test_count;
    uint8_t  rssi_test_interval;
#endif

    WICED_BT_TRACE("HCI opcode:%x\n", opcode);
    switch (opcode)
    {
#if defined(MESH_DFU_SUPPORTED)
    case HCI_CONTROL_MESH_COMMAND_FW_DISTRIBUTION_UPLOAD_START:
        if (!wiced_firmware_upgrade_init_nv_locations())
        {
            WICED_BT_TRACE("failed init nv locations\n");
            status = WICED_BT_MESH_FW_DISTR_STATUS_NOT_SUPPORTED;
        }
        else
        {
            status = fw_distribution_server_process_upload_start(NULL, p_data, length);
        }
        mesh_provisioner_hci_send_fw_distr_status(status);

        if (status == WICED_BT_MESH_FW_DISTR_STATUS_SUCCESS)
            fw_distribution_server_blob_transfer_callback(WICED_BT_MESH_BLOB_TRANSFER_START, NULL);
        break;

    case HCI_CONTROL_MESH_COMMAND_FW_DISTRIBUTION_UPLOAD_DATA:
        STREAM_TO_UINT32(data.offset, p_data);
        data.data_len = length - 4;
        data.p_data = p_data;
        WICED_BT_TRACE("Upload data offset:%d len:%d\n", data.offset, data.data_len);
        fw_distribution_server_blob_transfer_callback(WICED_BT_MESH_BLOB_TRANSFER_DATA, &data);
        mesh_provisioner_hci_send_fw_distr_status(status);
        break;

    case HCI_CONTROL_MESH_COMMAND_FW_DISTRIBUTION_UPLOAD_FINISH:
        STREAM_TO_UINT8(finish.blob_transfer_result, p_data);
        WICED_BT_TRACE("Upload finish\n");
        fw_distribution_server_blob_transfer_callback(WICED_BT_MESH_BLOB_TRANSFER_FINISH, &finish);
        mesh_provisioner_hci_send_fw_distr_status(status);

        // Check if FW ID is for any of the devices on the network
        fw_distribution_server_get_upload_fw_id(&fw_id);

        if (!embedded_provisioner_start_dfu(&fw_id))
            wiced_firmware_upgrade_finish();
        break;

    case HCI_CONTROL_MESH_COMMAND_FW_DISTRIBUTION_UPLOAD_GET_STATUS:
        mesh_provisioner_hci_send_fw_distr_status(0);
        break;
#endif

#if defined(RSSI_TEST)
    case HCI_CONTROL_MESH_COMMAND_RSSI_TEST_START:
        rssi_test_started_by_host = WICED_TRUE;

        STREAM_TO_UINT16(rssi_test_dst, p_data);
        STREAM_TO_UINT16(rssi_test_count, p_data);
        STREAM_TO_UINT8(rssi_test_interval, p_data);
        rssi_test_start(rssi_test_dst, rssi_test_count, rssi_test_interval, 0);
        break;
#endif

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
    mesh_provisioner_hci_send_status(status);
    return WICED_TRUE;
}

// To catch lost buffers. Functions wiced_bt_get_buffer_deb and wiced_bt_free_buffer_deb are implemented in the mesh_application.c which doesn't include that mesh_trace.h file
#ifdef _DEB_TRACE_BUF
void* wiced_bt_get_buffer_deb(uint32_t buffer_size)
{
    void* p_buf = wiced_bt_get_buffer(buffer_size);
    WICED_BT_TRACE("get_buffer: p_mem:%x %d\n", (uint32_t)p_buf, buffer_size);
    return p_buf;
}

void wiced_bt_free_buffer_deb(void* p_buf)
{
    WICED_BT_TRACE("free_buffer: p_mem:%x\n", (uint32_t)p_buf);
    wiced_bt_free_buffer(p_buf);
}
#endif
