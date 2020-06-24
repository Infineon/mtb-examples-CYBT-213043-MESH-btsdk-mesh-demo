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
 *
 * This file shows how to create a device which implements mesh on/off client model.
 */

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_default_transition_time_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_default_transition_time_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_default_transition_time_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_default_transition_time_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Process event received from the Default Transition Time Server.
 */
void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->status.tx_flag);
        break;

    case WICED_BT_MESH_DEFAULT_TRANSITION_TIME_STATUS:
        WICED_BT_TRACE("deftt time:%d\n", ((wiced_bt_mesh_default_transition_time_data_t *)p_data)->time);
        break;

    default:
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * Send default_transition_time get command
 */
void mesh_default_transition_time_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_default_transition_time_client_send_get(p_event);
}

/*
 * Send default_transition_time set command
 */
void mesh_default_transition_time_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_default_transition_time_data_t set_data;

    STREAM_TO_UINT32(set_data.time, p_data);

    wiced_bt_mesh_model_default_transition_time_client_send_set(p_event, &set_data);
}
