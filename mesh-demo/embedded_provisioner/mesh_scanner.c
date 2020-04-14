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
 *
 * This file shows how to create a device supporting a vendor specific model.
 */
#include "wiced_bt_ble.h"
#include "wiced_memory.h"
#include "wiced_bt_mesh_model_defs.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_provision.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_model_utils.h"
#include "embedded_provisioner.h"
#include "mesh_scanner.h"

#define RSSI_TEST           1
#define EMBEDDED_PROVISION  1
#define SELF_CONFIG         1

#if defined(EMBEDDED_PROVISION)

#define MEASURE_RSSI    1

/******************************************************
 *          Structures
 ******************************************************/

extern mesh_embedded_provisioner_state_t *p_provisioner_state;

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_scanner_before_provision_delay_timeout(TIMER_PARAM_TYPE arg);
wiced_bt_mesh_event_t* mesh_scanner_create_event(uint16_t dst);
int mesh_scanner_select_provisioner(mesh_unprov_node_t* p_unprov_node);
void mesh_scanner_scan_status_timeout(TIMER_PARAM_TYPE arg);
static wiced_bool_t mesh_provsioner_validate_unprovision_device(wiced_bt_mesh_provision_scan_report_data_t* p_data);
wiced_bool_t mesh_scanner_scan_start(uint8_t rpr_idx);
mesh_unprov_node_t* mesh_scanner_alloc_unprov_node(void);
void mesh_scanner_free_unprov_node(mesh_unprov_node_t* p_unprov_node);
uint32_t mesh_scanner_get_num_unprov_nodes(void);
wiced_bool_t mesh_scanner_check_unprov_node(mesh_unprov_node_t* p_unprov_node);
int find_rpr_idx(uint16_t rpr_addr);

/*
 * Initialize embedded provisioner's scanner module
 */
void wiced_bt_mesh_scanner_client_init(wiced_bt_mesh_provision_client_callback_t* p_callback, wiced_bool_t is_provisioned)
{
    WICED_BT_TRACE("scanner init\n");
}

/*
 * During startup application should call this function with every device that supports RPR
 */
int mesh_scanner_add_provisioner(uint16_t addr)
{
    int i;

    WICED_BT_TRACE("scanner add provisioner:%04x\n", addr);

    if ((i = find_rpr_idx(addr)) != -1)
    {
        return -1;
    }
    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if (p_provisioner_state->rpr_node[i].addr == 0)
        {
            wiced_init_timer(&p_provisioner_state->rpr_node[i].scan_status_timer, mesh_scanner_scan_status_timeout, (TIMER_PARAM_TYPE)i, WICED_SECONDS_TIMER);
            p_provisioner_state->rpr_node[i].addr = addr;
            p_provisioner_state->rpr_node[i].rpr_scan_state = RPR_STATE_IDLE;
            p_provisioner_state->rpr_node[i].rpr_link_state = RPR_STATE_IDLE;
            return i;
        }
    }
    return -1;
}

/*
 * Start scan on the node
 */
wiced_bool_t mesh_scanner_scan_start(uint8_t rpr_idx)
{
    int i;
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_provision_scan_start_data_t data;
    mesh_unprov_node_t* p_unprov_node;

    // we will only do 1 start at a time
    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if ((p_provisioner_state->rpr_node[i].addr != 0) &&
            (p_provisioner_state->rpr_node[i].rpr_scan_state == RPR_STATE_STARTING_SCAN))
        {
            WICED_BT_TRACE("scan start already starting idx:%d addr:%04x\n", rpr_idx, p_provisioner_state->rpr_node[i].addr);
            return WICED_FALSE;
        }
    }
    WICED_BT_TRACE("scan start idx:%d\n", rpr_idx);

    // start timer waiting for the Scan Status
    wiced_start_timer(&p_provisioner_state->rpr_node[rpr_idx].scan_status_timer, 3);

    // start scanning with local device
    p_provisioner_state->rpr_node[rpr_idx].rpr_scan_state = RPR_STATE_STARTING_SCAN;

    if ((p_event = mesh_scanner_create_event(p_provisioner_state->rpr_node[rpr_idx].addr)) == NULL)
    {
        WICED_BT_TRACE("scan start no mem\n");
        return WICED_FALSE;
    }
    memset(&data, 0, sizeof(data));
    data.timeout = 255; // longest possible, which is about 4 min

    WICED_BT_TRACE("scan started:%04x\n", p_provisioner_state->rpr_node[rpr_idx].addr);

    if (!wiced_bt_mesh_provision_scan_start(p_event, &data))
    {
        WICED_BT_TRACE("failed to scan start\n");
        return WICED_FALSE;
    }

    // New provisioner may be a better choice of provisioning devices that have been already reported.
    // Give new provisioner some time to measure and report RSSI of unprovisioned nodes.
    // This may increase total system provisioning time.
    for (p_unprov_node = p_provisioner_state->p_unprov_node_first; p_unprov_node != NULL; p_unprov_node = p_unprov_node->p_next)
    {
        if (p_unprov_node->rpr_addr == 0)
            wiced_start_timer(&p_unprov_node->timer, BEFORE_PROVISION_DELAY);
    }
    return WICED_TRUE;
}

/*
 * Start scan on the next RPR
 */
void start_scan_next(void)
{
    int i;

    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if (p_provisioner_state->rpr_node[i].addr != 0)
        {
            WICED_BT_TRACE("start next %04x state:%d\n", p_provisioner_state->rpr_node[i].addr, p_provisioner_state->rpr_node[i].rpr_scan_state);
        }
        if ((p_provisioner_state->rpr_node[i].addr != 0) &&
            ((p_provisioner_state->rpr_node[i].rpr_scan_state == RPR_STATE_IDLE) || (p_provisioner_state->rpr_node[i].rpr_scan_state == RPR_STATE_RESTART_REQUIRED)))
        {
            mesh_scanner_scan_start(i);
            return;
        }
    }
}

/*
 * Process scan status event
 */
void mesh_scanner_process_scan_status(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_scan_status_data_t* p_data)
{
    int i;
    uint16_t src = p_event->src;

    WICED_BT_TRACE("scan status from:%04x status:%d state:%d limit:%d timeout:%d\n", src, p_data->status, p_data->state, p_data->scanned_items_limit, p_data->timeout);
    wiced_bt_mesh_release_event(p_event);

    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if (src == p_provisioner_state->rpr_node[i].addr)
        {
            if (p_provisioner_state->rpr_node[i].rpr_scan_state == RPR_STATE_STARTING_SCAN)
            {
                p_provisioner_state->rpr_node[i].rpr_scan_state = RPR_STATE_SCANNING;
                wiced_stop_timer(&p_provisioner_state->rpr_node[i].scan_status_timer);

                start_scan_next();
            }
            else
            {
                WICED_BT_TRACE("invalid state\n");
            }
            break;
        }
    }
}

/*
 * We asked RPR to start scan, but it did not reply, just start the next one
 */
void mesh_scanner_scan_status_timeout(TIMER_PARAM_TYPE arg)
{
    int rpr_idx = (int)arg;

    WICED_BT_TRACE("scan status timeout idx:%d state:%d\n", rpr_idx, p_provisioner_state->rpr_node[rpr_idx].rpr_scan_state);
    if (p_provisioner_state->rpr_node[rpr_idx].rpr_scan_state == RPR_STATE_STARTING_SCAN)
    {
        p_provisioner_state->rpr_node[rpr_idx].rpr_scan_state = RPR_STATE_FAILED;
        start_scan_next();
    }
}

/*
 * Process provisioner scan report event
 */
void mesh_scanner_process_scan_report(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_scan_report_data_t* p_data)
{
    uint16_t src = p_event->src;
    mesh_unprov_node_t* p_unprov_node, * p_temp;
    wiced_bool_t reported_node = WICED_FALSE;
    int rpr_idx;
    int i;

    wiced_bt_mesh_release_event(p_event);

    if (p_provisioner_state->state == EMBEDDED_PROVISIONER_STATE_RESET)
        return;

    if (p_data->rssi < EMBEDDED_PROV_MIN_RSSI)
    {
        WICED_BT_TRACE("unprovisioned from:%04x device %02x-%02x oob:%04x rssi:%d too far\n", src, p_data->uuid[0], p_data->uuid[15], p_data->oob, p_data->rssi);
        return;
    }
    // check if this device is of interest
    if (!mesh_provsioner_validate_unprovision_device(p_data))
        return;

    // find the provisioner which found the new node
    if ((rpr_idx = find_rpr_idx(src)) == -1)
    {
        WICED_BT_TRACE("cannot process scan report from %04x\n", src);
        return;
    }

    // Check if we already have report about this node and update rssi table
    for (p_unprov_node = p_provisioner_state->p_unprov_node_first; p_unprov_node != NULL; p_unprov_node = p_unprov_node->p_next)
    {
        if (memcmp(p_unprov_node->uuid, p_data->uuid, 16) == 0)
        {
            p_unprov_node->rssi_table[rpr_idx] = p_data->rssi;
            reported_node = WICED_TRUE;
            break;
        }
    }

    WICED_BT_TRACE("unprovisioned from:%04x device %02x-%02x oob:%04x rssi:%d reported:%d\n", src, p_data->uuid[0], p_data->uuid[15], p_data->oob, p_data->rssi, reported_node);

    if (!reported_node)
    {
        // if we got here, none of the RPRs have seen this device before. Check number of outstanding unprovisioned nodes
        if (mesh_scanner_get_num_unprov_nodes() >= EMBEDDED_PROV_UNPROV_HI_THRESH)
        {
            WICED_BT_TRACE("too many outstanding unprovisioned nodes\n");

            // We do not stop the scan, but we need to restart the scan because this device will not be reported anymore.
            for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
            {
                if (p_provisioner_state->rpr_node[i].addr == src)
                {
                    p_provisioner_state->rpr_node[i].rpr_scan_state = RPR_STATE_RESTART_REQUIRED;
                    break;
                }
            }
            return;
        }

        if ((p_unprov_node = mesh_scanner_alloc_unprov_node()) == NULL)
        {
            WICED_BT_TRACE("new device from %04x no mem\n", src);
            return;
        }

        memcpy(p_unprov_node->uuid, p_data->uuid, sizeof(p_unprov_node->uuid));

        // Let other provisioners report the same device if they see it.
        wiced_start_timer(&p_unprov_node->timer, BEFORE_PROVISION_DELAY);
    }
    p_unprov_node->rssi_table[rpr_idx] = p_data->rssi;

    // If there are no other provisioners in the network that can report this device, we can start provisioning, otherwise wait.
    for (rpr_idx = 0; rpr_idx < EMBEDDED_PROV_MAX_NODES; rpr_idx++)
    {
        if ((p_provisioner_state->rpr_node[rpr_idx].addr != 0) && (p_provisioner_state->rpr_node[rpr_idx].rpr_scan_state == RPR_STATE_SCANNING) && (p_unprov_node->rssi_table[rpr_idx] == 0))
        {
            WICED_BT_TRACE("keep waiting for %04x\n", p_provisioner_state->rpr_node[rpr_idx].addr);
            return;
        }
    }

    // if we got here, all existing RPRs reported this device, so we can start provisioning
    // This is the same action as when scan delay timer expires.
    wiced_stop_timer(&p_unprov_node->timer);
    mesh_scanner_before_provision_delay_timeout((TIMER_PARAM_TYPE)p_unprov_node);
}

/*
 * When we received first report from RPR we started delay timeout, to allow other RPRs to report so that we can collect
 * RSSI table. Timeout expired. Start provisioning.
 */
void mesh_scanner_before_provision_delay_timeout(TIMER_PARAM_TYPE arg)
{
    mesh_unprov_node_t* p_unprov_node = (mesh_unprov_node_t*)arg;
    wiced_bt_mesh_event_t* p_event;
    wiced_bt_mesh_provision_connect_data_t connect;
    int rpr_idx;

    if (p_provisioner_state->state == EMBEDDED_PROVISIONER_STATE_RESET)
        return;

    // There can be a race condition when this function is called on timeout and when the last RPR reported the unprovisioned device
    if (!mesh_scanner_check_unprov_node(p_unprov_node))
    {
        WICED_BT_TRACE("Node deleted\n");
        return;
    }
    if (p_unprov_node->rpr_addr != 0)
    {
        WICED_BT_TRACE("Provisioner already assigned addr:%04x\n", p_unprov_node->rpr_addr);
        return;
    }
    rpr_idx = mesh_scanner_select_provisioner(p_unprov_node);
    if (rpr_idx == -1)
    {
        WICED_BT_TRACE("all provisioners are busy\n");
        wiced_start_timer(&p_unprov_node->timer, 2); // PROVISION_RETRY_DELAY);
        return;
    }
    WICED_BT_TRACE("Provisioner selected:%d addr:%04x for %02x-%02x\n", rpr_idx, p_provisioner_state->rpr_node[rpr_idx].addr, p_unprov_node->uuid[0], p_unprov_node->uuid[15]);

    if ((p_event = mesh_scanner_create_event(p_provisioner_state->rpr_node[rpr_idx].addr)) == NULL)
        return;

    p_unprov_node->rpr_addr = p_provisioner_state->rpr_node[rpr_idx].addr;

    memcpy(connect.uuid, p_unprov_node->uuid, 16);
    connect.identify_duration = 1;
    connect.procedure = WICED_BT_MESH_PROVISION_PROCEDURE_PROVISION;

    if (wiced_bt_mesh_provision_connect(p_event, &connect, WICED_FALSE))
    {
        p_provisioner_state->rpr_node[rpr_idx].rpr_link_state = RPR_STATE_PROVISIONING;
    }
    else
    {
        WICED_BT_TRACE("Failed to connect:%d addr:%04x\n", rpr_idx, p_provisioner_state->rpr_node[rpr_idx].addr);
        wiced_bt_mesh_release_event(p_event);
        p_unprov_node->rpr_addr = 0;
        wiced_start_timer(&p_unprov_node->timer, 1); // retry to connect in 1 sec
    }
}

/*
 * Check if this device has to be provisioned by the embedded provisioner, return FALSE to skip
 */
wiced_bool_t mesh_provsioner_validate_unprovision_device(wiced_bt_mesh_provision_scan_report_data_t* p_data)
{
    int i;
    uint64_t magic_number = EMBEDDED_PROV_UUID_MAGIC_NUBMER;

#ifdef EMBEDDED_PROV_RSSI_THRESHOLD
    if (p_data->rssi > EMBEDDED_PROV_RSSI_THRESHOLD))
        return WICED_TRUE;
#endif
#ifdef EMBEDDED_PROV_UUID_MAGIC_NUBMER
    if (memcmp(&p_data->uuid[5], (uint8_t*)&magic_number + 1, 7) == 0)
    {
        WICED_BT_TRACE("Validate passed\n");
        return WICED_TRUE;
    }
    else
    {
        WICED_BT_TRACE("Validate failed\n");
        WICED_BT_TRACE_ARRAY((uint8_t *)&p_data->uuid[4], 8, "received ");
        WICED_BT_TRACE_ARRAY((uint8_t *)&magic_number, 8, "expected ");
    }
#endif
    return WICED_FALSE;
}

/*
 * Select provisioner to provision the unprovisioned device
 */
int mesh_scanner_select_provisioner(mesh_unprov_node_t* p_unprov_node)
{
    int i;
    int rpr_idx = -1;
    int selected_rssi = -127;

    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        // if this provisioner reported this node at reasonable RSSI
        if ((p_unprov_node->rssi_table[i] == 0) || (p_unprov_node->rssi_table[i] < EMBEDDED_PROV_MIN_RSSI))
            continue;

        // RPR can only provision one device at a time
        if ((p_provisioner_state->rpr_node[i].rpr_link_state == RPR_STATE_PROVISIONING) ||
            (p_provisioner_state->rpr_node[i].rpr_link_state == RPR_STATE_DISCONNECTING))
            continue;

        // for now just select device with highest RSSI
        if (p_unprov_node->rssi_table[i] < selected_rssi)
            continue;

        selected_rssi = p_unprov_node->rssi_table[i];
        rpr_idx = i;
    }
    return rpr_idx;
}

/*
 * Free up unprovisioned node structure and restart scan if needed
 */
void mesh_scanner_provision_end(mesh_unprov_node_t* p_unprov_node)
{
    int i;

    mesh_scanner_free_unprov_node(p_unprov_node);

    // Check if scanning need to be restarted for any of the RPRs
    if (mesh_scanner_get_num_unprov_nodes() < EMBEDDED_PROV_UNPROV_LO_THRESH)
    {
        // We do not stop the scan, but we need to restart the scan because this device will not be reported anymore.
        for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
        {
            if (p_provisioner_state->rpr_node[i].rpr_scan_state == RPR_STATE_RESTART_REQUIRED)
            {
                start_scan_next();
                break;
            }
        }

    }
}

wiced_bt_mesh_event_t* mesh_scanner_create_event(uint16_t dst)
{
    wiced_bt_mesh_event_t* p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_REMOTE_PROVISION_CLNT, dst, 0xFFFF);
    if (p_event != NULL)
    {
        p_event->reply = WICED_TRUE;
        p_event->send_segmented = WICED_TRUE;
        p_event->retrans_cnt = 0;       // transmit once
    }
    return p_event;
}

/*
 * Allocate unprovisioned node structure to keep RSSI levels reported by different provisioners.
 */
mesh_unprov_node_t* mesh_scanner_alloc_unprov_node(void)
{
    mesh_unprov_node_t* p_unprov_node;
    mesh_unprov_node_t* p_temp;

    if ((p_unprov_node = (mesh_unprov_node_t*)wiced_bt_get_buffer(sizeof(mesh_unprov_node_t))) == NULL)
    {
        WICED_BT_TRACE("unprov alloc no mem\n");
        return NULL;
    }

    // fill data and link it to the list of unprovisioned nodes
    memset(p_unprov_node, 0, sizeof(mesh_unprov_node_t));
    wiced_init_timer(&p_unprov_node->timer, mesh_scanner_before_provision_delay_timeout, (TIMER_PARAM_TYPE)p_unprov_node, WICED_SECONDS_TIMER);

    // keep the unprovisioned node in the linked list
    if (p_provisioner_state->p_unprov_node_first == NULL)
    {
        p_provisioner_state->p_unprov_node_first = p_unprov_node;
    }
    else
    {
        for (p_temp = p_provisioner_state->p_unprov_node_first; p_temp->p_next != NULL; p_temp = p_temp->p_next)
        {
        }
        p_temp->p_next = p_unprov_node;
    }
    return p_unprov_node;
}

/*
 * Free unprovisioned node structure
 */
void mesh_scanner_free_unprov_node(mesh_unprov_node_t* p_unprov_node)
{
    mesh_unprov_node_t* p_temp;

    if (p_provisioner_state->p_unprov_node_first == p_unprov_node)
    {
        p_provisioner_state->p_unprov_node_first = p_unprov_node->p_next;
    }
    else
    {
        for (p_temp = p_provisioner_state->p_unprov_node_first; p_temp->p_next != p_unprov_node; p_temp = p_temp->p_next)
        {
            ;
        }
        p_temp->p_next = p_unprov_node->p_next;
    }
    wiced_stop_timer(&p_unprov_node->timer);
    wiced_deinit_timer(&p_unprov_node->timer);

    wiced_bt_free_buffer(p_unprov_node);
}

/*
 * Get number of nodes waiting to be provisioned.
 */
uint32_t mesh_scanner_get_num_unprov_nodes(void)
{
    mesh_unprov_node_t* p_temp;
    uint32_t num_unprov_nodes = 0;

    for (p_temp = p_provisioner_state->p_unprov_node_first; p_temp != NULL; p_temp = p_temp->p_next)
        num_unprov_nodes++;

    return num_unprov_nodes;
}

/*
 * Check if unprov node has not been removed from the list
 */
wiced_bool_t mesh_scanner_check_unprov_node(mesh_unprov_node_t* p_unprov_node)
{
    mesh_unprov_node_t* p_temp;

    for (p_temp = p_provisioner_state->p_unprov_node_first; p_temp != NULL; p_temp = p_temp->p_next)
    {
        if (p_temp == p_unprov_node)
            return WICED_TRUE;
    }
    return WICED_FALSE;
}

/*
 * Find RPR node with specified address
 */
int find_rpr_idx(uint16_t rpr_addr)
{
    int i;

    for (i = 0; i < EMBEDDED_PROV_MAX_NODES; i++)
    {
        if (p_provisioner_state->rpr_node[i].addr == rpr_addr)
            return i;
    }
    return -1;
}

#endif
