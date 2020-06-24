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
 * This file provides definitions and function declaration for mesh scanner functionality.
 */

/*
 * Initialize embedded provisioner's scanner module
 */
extern void wiced_bt_mesh_scanner_client_init(wiced_bt_mesh_provision_client_callback_t* p_callback, wiced_bool_t is_provisioned);

/*
 * Add Remote Provision Server
 * During startup application should call this function with every device that supports RPR
 * Returns index of the new provisioner. -1 if cannot allocate index or RPR has been added already.
 */
extern int mesh_scanner_add_provisioner(uint16_t addr);

/*
 * Process scan status event
 */
extern void mesh_scanner_process_scan_status(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_scan_status_data_t* p_data);

/*
 * Process scan report event
 */
extern void mesh_scanner_process_scan_report(wiced_bt_mesh_event_t* p_event, wiced_bt_mesh_provision_scan_report_data_t* p_data);

/*
 * Start scan on the node
 */
extern wiced_bool_t mesh_scanner_scan_start(uint8_t node_idx);

/*
 * Notify Scanner that provisioning for one of the nodes has been completed
 */
extern void mesh_scanner_provision_end(mesh_unprov_node_t* p_unprov_node);
