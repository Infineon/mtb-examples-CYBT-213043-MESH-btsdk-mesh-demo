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
 * Embedded provisioner RSSI Test.
 *
 * This module contains methods to send RSSI Test data pattern, receiving
 * data patterns from the broadcaster, and reporting results to the device
 * that organizes the test.
 *
 * If EMBEDDED_PROVISION is defined, this module can also run the RSSI
 * test state machine. I.e. it schedules all nodes one by one to broadcast
 * data pattern and then collects RSSI measurements from all other nodes.
 */
#define RSSI_TEST           1
#define EMBEDDED_PROVISION  1
#define SELF_CONFIG         1

#include "wiced_bt_ble.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_nvram.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "wiced_bt_mesh_model_utils.h"
#include "mesh_application.h"
#include "embedded_provisioner.h"
#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

 /******************************************************
  *          Constants
  ******************************************************/
#define RSSI_TEST_TX_DST                        0xffff
#define RSSI_TEST_TX_INITIAL_START_DELAY        20000   // Wait for 20 seconds before starting the test, in case new devices are provisioned
#define RSSI_TEST_TX_RESTART_DELAY              2000    // Wait for 2 after each test
#define RSSI_TEST_TX_COUNT                      100 // 5       // Ask each node to send 5 packets
#define RSSI_TEST_TX_INTERVAL                   10 // 50      //  with 500 ms interval
#define RSSI_TEST_INTERVAL_MULTIPLIER           10
#define RSSI_TEST_TX_START_DELAY                2000
#define RSSI_TEST_RX_INACT_TIMEOUT              2000

#define RSSI_TEST_USE_LED_FOR_DEBUG             0

#define RSSI_TEST_MASTER_STATE_IDLE             0
#define RSSI_TEST_MASTER_STATE_WAIT_START_RSP   1
#define RSSI_TEST_MASTER_STATE_RUNNING          2
#define RSSI_TEST_MASTER_STATE_WAIT_RESULT      3

#define RSSI_TEST_MASTER_EVENT_START            0
#define RSSI_TEST_MASTER_EVENT_START_RSP        1
#define RSSI_TEST_MASTER_EVENT_TX_FAILED        2
#define RSSI_TEST_MASTER_EVENT_RESULT           3
#define RSSI_TEST_MASTER_EVENT_TIMEOUT          4

/******************************************************
 *          Structures
 ******************************************************/

typedef struct
{
    // TX test parameters
    uint16_t        tx_total;
    uint16_t        tx_count;
    uint16_t        tx_interval;
    uint16_t        tx_dst;
    uint16_t        tx_app_key_idx;

    // RX test data
    wiced_bool_t    rx_active;
    uint16_t        rx_src;
    uint16_t        rx_num;
    int8_t          rx_rssi;

    wiced_timer_t   timer;
} rssi_test_state_t;

rssi_test_state_t rssi_test_state;

/******************************************************
 *          Function Prototypes
 ******************************************************/

static void rssi_test_timer_cb(TIMER_PARAM_TYPE arg);
static void rssi_test_master_set_state(uint8_t state);
static void rssi_test_master_execute(uint8_t event);
static void rssi_test_master_idle_state(uint8_t event);
static void rssi_test_master_wait_start_rsp_state(uint8_t event);
static void rssi_test_master_running_state(uint8_t event);
static void rssi_test_master_wait_result_state(uint8_t event);
static wiced_bool_t rssi_test_send_tx_start(uint16_t addr);
static wiced_bool_t rssi_test_send_rx_values_get(uint16_t addr);
static void rssi_test_start_timer_cb(TIMER_PARAM_TYPE arg);
static void rssi_test_master_send_complete_callback(wiced_bt_mesh_event_t* p_event);
static void rssi_test_timer_cb(TIMER_PARAM_TYPE arg);
static void rssi_test_send_data(void);

extern mesh_embedded_provisioner_state_t *p_provisioner_state;
extern wiced_bool_t mesh_config_client;

/******************************************************
 *               Function Definitions
 ******************************************************/
void rssi_test_init(void)
{
    memset(&rssi_test_state, 0, sizeof(rssi_test_state));
    wiced_init_timer(&rssi_test_state.timer, rssi_test_timer_cb, 0, WICED_MILLI_SECONDS_TIMER);
}

void rssi_test_start(uint16_t dst, uint16_t count, uint8_t interval, uint16_t start_delay)
{
    if (p_provisioner_state == NULL)
        return;

    p_provisioner_state->rssi_test.cur_tx_start_node = 0;
    p_provisioner_state->rssi_test.cur_rx_values_get_node = 0;
    p_provisioner_state->rssi_test.dst = (dst == 0) ? RSSI_TEST_TX_DST : dst;
    p_provisioner_state->rssi_test.count = (count == 0) ? RSSI_TEST_TX_COUNT : count;
    p_provisioner_state->rssi_test.interval = ((interval == 0) ? RSSI_TEST_TX_INTERVAL : interval) * RSSI_TEST_INTERVAL_MULTIPLIER;

    // if not idle, need to wait for the previous test to complete. Otherwise just a guard timer in case provisioner finds and provisions another device.
    if (p_provisioner_state->rssi_test.state != RSSI_TEST_MASTER_STATE_IDLE)
    {
        wiced_start_timer(&rssi_test_state.timer, start_delay + (RSSI_TEST_TX_START_DELAY * 2) + (p_provisioner_state->rssi_test.count * p_provisioner_state->rssi_test.interval));
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_IDLE);
    }
    else
    {
        wiced_start_timer(&rssi_test_state.timer, start_delay == 0 ? 1 : start_delay);
    }
}

void rssi_test_master_set_state(uint8_t state)
{
    WICED_BT_TRACE("RSSI Test Master state:%d\n", state);
    p_provisioner_state->rssi_test.state = state;
}

void rssi_test_master_execute(uint8_t event)
{
    if (p_provisioner_state == NULL)
        return;

    switch (p_provisioner_state->rssi_test.state)
    {
    case RSSI_TEST_MASTER_STATE_IDLE:
        rssi_test_master_idle_state(event);
        break;

    case RSSI_TEST_MASTER_STATE_WAIT_START_RSP:
        rssi_test_master_wait_start_rsp_state(event);
        break;

    case RSSI_TEST_MASTER_STATE_RUNNING:
        rssi_test_master_running_state(event);
        break;

    case RSSI_TEST_MASTER_STATE_WAIT_RESULT:
        rssi_test_master_wait_result_state(event);
        break;
    }
}

/*
 * RSSI Test Master is in Idle state waiting for Pre Start timeout
 */
void rssi_test_master_idle_state(uint8_t event)
{
    wiced_result_t result;
    mesh_node_t node;

    WICED_BT_TRACE("rssi test idle state:event:%d cur_node:%d\n", event, p_provisioner_state->rssi_test.cur_tx_start_node);

    switch (event)
    {
    case RSSI_TEST_MASTER_EVENT_TIMEOUT:
        if (wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST + p_provisioner_state->rssi_test.cur_tx_start_node, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
        {
            WICED_BT_TRACE("rssi test_start done cur_node:%d\n", p_provisioner_state->rssi_test.cur_tx_start_node);
            return;
        }
        p_provisioner_state->rssi_test.cur_tx_start_node_addr = node.addr;
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_WAIT_START_RSP);
        if (!rssi_test_send_tx_start(node.addr))
            rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_IDLE);
        break;

    default:
        WICED_BT_TRACE("rssi event ignored\n");
        break;
    }
}

/*
 * RSSI Test Master is in Wait Start Rsp state waiting for Start Rsp from rssi_test.cur_tx_start_node
 */
void rssi_test_master_wait_start_rsp_state(uint8_t event)
{
    WICED_BT_TRACE("rssi test wait start rsp state:event:%d cur_node:%d\n", event, p_provisioner_state->rssi_test.cur_tx_start_node);

    switch (event)
    {
    case RSSI_TEST_MASTER_EVENT_TX_FAILED:
        p_provisioner_state->rssi_test.cur_tx_start_node++;
        wiced_start_timer(&rssi_test_state.timer, RSSI_TEST_TX_RESTART_DELAY);
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_IDLE);
        break;

    case RSSI_TEST_MASTER_EVENT_START_RSP:
        // Start response is received, stay in the RUNNING state until all data is received.
        wiced_start_timer(&rssi_test_state.timer, (RSSI_TEST_TX_START_DELAY * 2) + (p_provisioner_state->rssi_test.count * p_provisioner_state->rssi_test.interval));
        p_provisioner_state->rssi_test.cur_rx_values_get_node = 0;
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_RUNNING);
        break;

    default:
        WICED_BT_TRACE("rssi event ignored\n");
        break;
    }
}

/*
 * RSSI Test Master is in Running state waiting the timeout which would indicate that the test on rssi_test.cur_tx_start_node has been completed
 */
void rssi_test_master_running_state(uint8_t event)
{
    wiced_result_t result;
    mesh_node_t node;
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[8];
    uint8_t* p = buffer;

    WICED_BT_TRACE("rssi test running state:event:%d tx_node:%d get_node:%d\n", event, p_provisioner_state->rssi_test.cur_tx_start_node, p_provisioner_state->rssi_test.cur_rx_values_get_node);

    switch (event)
    {
    case RSSI_TEST_MASTER_EVENT_TIMEOUT:
        if (p_provisioner_state->rssi_test.cur_rx_values_get_node == p_provisioner_state->rssi_test.cur_tx_start_node)
            p_provisioner_state->rssi_test.cur_rx_values_get_node++;

        if (wiced_hal_read_nvram(EMBEDDED_PROV_NODE_ADDR_FIRST + p_provisioner_state->rssi_test.cur_rx_values_get_node, sizeof(node), (uint8_t*)&node, &result) != sizeof(node))
        {
            WICED_BT_TRACE("rssi last report received get_node:%d\n", p_provisioner_state->rssi_test.cur_rx_values_get_node);
            p_provisioner_state->rssi_test.cur_tx_start_node++;
            rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_IDLE);
            rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
            return;
        }
        if (rssi_test_send_rx_values_get(node.addr))
            rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_WAIT_RESULT);
        break;

    default:
        WICED_BT_TRACE("rssi event ignored\n");
        break;
    }
}

/*
 * RSSI Test Master is in Wait Responce Rx state waiting for Respond Rx from rssi_test.cur_rx_values_get_node
 */
void rssi_test_master_wait_result_state(uint8_t event)
{
    WICED_BT_TRACE("rssi test_master wait report:event:%d cur_node:%d\n", event, p_provisioner_state->rssi_test.cur_rx_values_get_node);

    switch (event)
    {
    case RSSI_TEST_MASTER_EVENT_TX_FAILED:
        p_provisioner_state->rssi_test.cur_rx_values_get_node++;
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_RUNNING);
        rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
        break;

    case RSSI_TEST_MASTER_EVENT_RESULT:
        // ToDo Save result
        p_provisioner_state->rssi_test.cur_rx_values_get_node++;
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_RUNNING);
        rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
        break;

    default:
        WICED_BT_TRACE("rssi event ignored\n");
        return;
    }
}

/*
 * Send TX Start to the next device
 */
void rssi_test_start_timer_cb(TIMER_PARAM_TYPE arg)
{
    rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
}

/*
 * Send command to a node to start TX test.
 */
wiced_bool_t rssi_test_send_tx_start(uint16_t addr)
{
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[8];
    uint8_t* p = buffer;

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, addr, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("rssi send_tx_start: no mem\n");
        return WICED_FALSE;
    }
    WICED_BT_TRACE("rssi send tx start to:%04x target:%04x count:%d interval:%d\n", p_event->dst, p_provisioner_state->rssi_test.dst, p_provisioner_state->rssi_test.count, p_provisioner_state->rssi_test.interval);

    p_event->opcode = MESH_VENDOR_CYPRSESS_OPCODE_CONFIG;
    p_event->reply = WICED_TRUE;
    p_event->retrans_cnt = 2;       // Try 3 times (this is in addition to network layer retransmit)
    p_event->retrans_time = 4;      // Every 200 msec
    p_event->reply_timeout = 20;    // wait for the reply 1 seconds

    UINT8_TO_STREAM(p, MESH_CY_VENDOR_SUBCODE_RSSI_TEST_TX_START);
    UINT16_TO_STREAM(p, p_provisioner_state->rssi_test.dst);
    UINT16_TO_STREAM(p, p_provisioner_state->rssi_test.count);
    UINT8_TO_STREAM(p, p_provisioner_state->rssi_test.interval / RSSI_TEST_INTERVAL_MULTIPLIER);

    // If request to send to this device, process TX Start and set state to Running because there will be no Start Status.
    if (addr == wiced_bt_mesh_core_get_local_addr())
    {
        mesh_vendor_rssi_test_process_tx_start(p_event, &buffer[1], (uint8_t)(p - buffer) - 1);
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_RUNNING);
        return WICED_TRUE;
    }

    if (wiced_bt_mesh_models_utils_send(p_event, &p_provisioner_state->p_out_event,
        WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), rssi_test_master_send_complete_callback) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("rssi test_start: failed to send\n");
        wiced_bt_mesh_release_event(p_event);
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*
 * Command from Mesh Client to start TX test.
 */
void mesh_vendor_rssi_test_process_tx_start(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len)
{
    uint8_t buffer[8];
    uint8_t* p = buffer;

    STREAM_TO_UINT16(rssi_test_state.tx_dst, p_data);
    STREAM_TO_UINT16(rssi_test_state.tx_total, p_data);
    STREAM_TO_UINT8(rssi_test_state.tx_interval, p_data);
    rssi_test_state.tx_interval *= RSSI_TEST_INTERVAL_MULTIPLIER;
    rssi_test_state.tx_count = rssi_test_state.tx_total;
    WICED_BT_TRACE("rssi test_start: dst:%04x count:%d interval:%d\n", rssi_test_state.tx_dst, rssi_test_state.tx_total, rssi_test_state.tx_interval);

#if RSSI_TEST_USE_LED_FOR_DEBUG
    led_control_set_onoff(1);
#endif

    // We will be sending packets using the same application key as used to set up the test
    rssi_test_state.tx_app_key_idx = p_event->app_key_idx;

    wiced_start_timer(&rssi_test_state.timer, RSSI_TEST_TX_START_DELAY);

#if defined(EMBEDDED_PROVISION)
    // if this is embedded provisioner, no need to reply
    if (p_provisioner_state != NULL)
    {
        wiced_bt_mesh_release_event(p_event);
        return;
    }
#endif
    UINT8_TO_STREAM(p, MESH_CY_VENDOR_SUBCODE_RSSI_TEST_TX_START_STATUS);
    wiced_bt_mesh_models_utils_send(wiced_bt_mesh_create_reply_event(p_event), NULL, WICED_FALSE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
}

/*
 * transmit complete callback function. Kick off state machine if TX failed.
 */
void rssi_test_master_send_complete_callback(wiced_bt_mesh_event_t* p_event)
{
   WICED_BT_TRACE(TRACE_INFO, "rssi send complete: dst:%04x status:%d\n", p_event->dst, p_event->status.tx_flag);
    if (p_event->status.tx_flag == TX_STATUS_COMPLETED)
        return;

    if (wiced_bt_mesh_models_utils_send_complete(&p_provisioner_state->p_out_event, p_event->dst))
    {
        wiced_bt_mesh_release_event(p_event);
        if (p_event->status.tx_flag == TX_STATUS_FAILED)
        {
            WICED_BT_TRACE("rssi test send complete: failed\n");
            rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TX_FAILED);
        }
    }
}

/*
 * Process TX start response.
 */
void mesh_vendor_rssi_test_process_tx_start_status(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len)
{
    // If we are still retransmitting cancel transmit and release mesh event
    wiced_bt_mesh_models_utils_ack_received(&p_provisioner_state->p_out_event, p_event->src);

    WICED_BT_TRACE("rssi received tx start status\n");

    wiced_bt_mesh_release_event(p_event);

    rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_START_RSP);
}

/*
 * Receive data packet during RSSI test
 */
void mesh_vendor_rssi_test_process_data(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len)
{
    static uint64_t data_rx_timestamp = 0;
    uint64_t curr_time = wiced_bt_mesh_core_get_tick_count();

#if RSSI_TEST_USE_LED_FOR_DEBUG
    led_control_set_onoff(1);
#endif

    // If data is received from the different source, or from the same source but more than 10 seconds ago, it is a new test
    if ((rssi_test_state.rx_src != p_event->src) || ((curr_time - data_rx_timestamp) > RSSI_TEST_TX_INITIAL_START_DELAY))
    {
        // starting rx test
        rssi_test_state.rx_src = p_event->src;
        rssi_test_state.rx_rssi = p_event->rssi;
        rssi_test_state.rx_num = 1;
    }
    else
    {
        // calculate new average
        // WARN: mean Power in NOT mean RSSI --> USE MEDIAN from KRAV-1?? or MEDIAN WINDOW 5
        rssi_test_state.rx_rssi = ((rssi_test_state.rx_rssi * rssi_test_state.rx_num) + p_event->rssi) / (rssi_test_state.rx_num + 1);
        //rssi_test_state.rx_rssi = p_event->rssi;
        rssi_test_state.rx_num++;
    }
    data_rx_timestamp = curr_time;
    wiced_bt_mesh_release_event(p_event);

#if RSSI_TEST_USE_LED_FOR_DEBUG
    led_control_set_onoff(0);
#endif
    WICED_BT_TRACE("rssi test_data: from:%04x count:%d received:%d rssi:%d\n", p_event->src, p_data[0], rssi_test_state.rx_num, p_event->rssi);
}

/*
 * Send command to a node to get RX Status.
 */
wiced_bool_t rssi_test_send_rx_values_get(uint16_t addr)
{
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[8];
    uint8_t* p = buffer;

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, addr, EMBEDDED_PROV_APP_KEY_IDX);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("rssi rx values get: no mem\n");
        return WICED_FALSE;
    }
    WICED_BT_TRACE("rssi send rx values get to:%04x for node:%04x\n", addr, p_provisioner_state->rssi_test.cur_tx_start_node_addr);

    p_event->reply = WICED_TRUE;
    p_event->retrans_cnt = 2;       // Try 3 times (this is in addition to network layer retransmit)
    p_event->retrans_time = 4;      // Every 200 msec
    p_event->reply_timeout = 20;    // wait for the reply 1 seconds

    UINT8_TO_STREAM(p, MESH_CY_VENDOR_SUBCODE_RSSI_TEST_RX_VALUES_GET);
    UINT16_TO_STREAM(p, p_provisioner_state->rssi_test.cur_tx_start_node_addr);

    // If request to send to this device, process prepare and send report RX status
    if (addr == wiced_bt_mesh_core_get_local_addr())
    {
        rssi_test_master_set_state(RSSI_TEST_MASTER_STATE_WAIT_RESULT);
        mesh_vendor_rssi_test_process_rx_values_get(p_event, &buffer[1], (uint8_t)(p - buffer) - 1);
        return WICED_TRUE;
    }

    if (wiced_bt_mesh_models_utils_send(p_event, &p_provisioner_state->p_out_event,
        WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), rssi_test_master_send_complete_callback) != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("rssi test report rx: failed to send\n");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}


/*
 * Process command from Mesh Client to report results of the previous test
 */
void mesh_vendor_rssi_test_process_rx_values_get(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len)
{
    uint8_t buffer[8];
    uint8_t* p = buffer;
    uint16_t report_addr;
    uint16_t rx_count;

    STREAM_TO_UINT16(report_addr, p_data);

    WICED_BT_TRACE("rssi process report rx for:%04x last received:%04x\n", report_addr, rssi_test_state.rx_src);

    UINT8_TO_STREAM(p, MESH_CY_VENDOR_SUBCODE_RSSI_TEST_RX_VALUES_STATUS);
    UINT16_TO_STREAM(p, report_addr);
    if (report_addr == rssi_test_state.rx_src)
    {
        UINT16_TO_STREAM(p, rssi_test_state.rx_num);
        UINT8_TO_STREAM(p, rssi_test_state.rx_rssi);
    }
    else
    {
        UINT16_TO_STREAM(p, 0);
        UINT8_TO_STREAM(p, 0);
    }
#if defined(EMBEDDED_PROVISION)
    // if this is embedded provisioner, no need to reply
    if (p_provisioner_state != NULL)
    {
        mesh_vendor_rssi_test_process_rx_values_status(p_event, &buffer[1], (uint16_t)(p - buffer) - 1);
        return;
    }
#endif
    wiced_bt_mesh_models_utils_send(wiced_bt_mesh_create_reply_event(p_event), NULL, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
}

/*
 * Process Report RX Status
 */
void mesh_vendor_rssi_test_process_rx_values_status(wiced_bt_mesh_event_t* p_event, uint8_t* p_data, uint16_t data_len)
{
    uint16_t report_addr;
    uint16_t rx_count;
    int8_t rx_rssi;

    // If we are still retransmitting cancel transmit and release mesh event
    wiced_bt_mesh_models_utils_ack_received(&p_provisioner_state->p_out_event, p_event->src);

    STREAM_TO_UINT16(report_addr, p_data);
    STREAM_TO_UINT16(rx_count, p_data);
    STREAM_TO_UINT8(rx_rssi, p_data);

    WICED_BT_TRACE("rssi report rx status from:%04x for:%04x received:%d average rssi:%d\n", p_event->src, report_addr, rx_count, rx_rssi);

    rssi_test_send_hci_result(p_event->src, report_addr, rx_count, rx_rssi);

    wiced_bt_mesh_release_event(p_event);

    rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_RESULT);
}

/*
 * Process timer callback
 */
void rssi_test_timer_cb(TIMER_PARAM_TYPE arg)
{
#if defined(EMBEDDED_PROVISION)
    if (p_provisioner_state != NULL)
    {
        WICED_BT_TRACE("rssi test_timeout state:%d\n", p_provisioner_state->rssi_test.state);
        if (p_provisioner_state->rssi_test.state != RSSI_TEST_MASTER_STATE_RUNNING)
        {
            rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
            return;
        }
    }
#endif
    WICED_BT_TRACE("rssi test_timeout count:%d\n", rssi_test_state.tx_total - rssi_test_state.tx_count + 1);
    if (rssi_test_state.tx_count != 0)
    {
        // Tx test. Send next frame and restart timer
        wiced_start_timer(&rssi_test_state.timer, rssi_test_state.tx_interval);
        rssi_test_send_data();
        rssi_test_state.tx_count--;
#if RSSI_TEST_USE_LED_FOR_DEBUG
        if (rssi_test_state.tx_count == 0)
            led_control_set_onoff(0);
#endif
        return;
    }
    rssi_test_master_execute(RSSI_TEST_MASTER_EVENT_TIMEOUT);
}

/*
 * Send packet to configured destination
 */
void rssi_test_send_data(void)
{
    wiced_bt_mesh_event_t* p_event;
    uint8_t buffer[8];
    uint8_t* p = buffer;

    p_event = wiced_bt_mesh_create_event(0, MESH_VENDOR_CYPRESS_COMPANY_ID, MESH_VENDOR_SELF_CONFIG_MODEL_ID, rssi_test_state.tx_dst, rssi_test_state.tx_app_key_idx);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("rssi send_data: no mem\n");
        return;
    }
    WICED_BT_TRACE("rssi send_data:%d\n", rssi_test_state.tx_total - rssi_test_state.tx_count + 1);

    p_event->ttl = 0;
    p_event->reply = WICED_FALSE;
    p_event->retrans_cnt = 0x80;        // 0x80 means no retransmissions on network layer or access layer

    UINT8_TO_STREAM(p, MESH_CY_VENDOR_SUBCODE_RSSI_TEST_DATA);
    UINT8_TO_STREAM(p, rssi_test_state.tx_count);

    wiced_bt_mesh_models_utils_send(p_event, NULL, WICED_TRUE, MESH_VENDOR_CYPRSESS_OPCODE_CONFIG, buffer, (uint16_t)(p - buffer), NULL);
}
