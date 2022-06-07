/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
* BLE Hello Client Gatt functions for btstack v3
*
*/
#include <string.h>
#include "sparcommon.h"
#include "bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "hello_client.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_bt_l2c.h"

/******************************************************************************
 *  Type  Definitions
 ******************************************************************************/
typedef void (*pfn_free_buffer_t)(uint8_t *);

/******************************************************************************
 *  Imported Data Declartions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_ble_t wiced_bt_cfg_ble;
extern hello_client_app_t g_hello_client;
extern const uint8_t hello_client_gatt_database[];
extern size_t hello_client_gatt_database_size;
extern uint8_t hello_client_notify_value[];
extern size_t hello_client_notify_value_size;
extern uint8_t start_scan;

/******************************************************************************
 *  Imported Function Declartions
 ******************************************************************************/
extern hello_client_peer_info_t *      hello_client_get_peer_information( uint16_t conn_id );
extern wiced_bool_t                    hello_client_is_device_bonded( wiced_bt_device_address_t bd_address );
extern const gatt_attribute_t*         hello_client_get_attribute(uint16_t handle);
extern wiced_bt_gatt_status_t          hello_client_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );
extern void                            hello_client_add_peer_info( uint16_t conn_id, uint8_t* p_bd_addr, uint8_t role , uint8_t transport, uint8_t address_type );
extern int                             hello_client_get_num_peripherals(void);
extern void                            hello_client_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );


/******************************************************************************
 *  Global Data Declartions
 ******************************************************************************/
wiced_bt_db_hash_t headset_db_hash;

/******************************************************************************
 *  Function Declartions
 ******************************************************************************/
static uint8_t *              hello_client_alloc_buffer(int len);
static void                   hello_client_free_buffer(uint8_t *p_data);
static wiced_bt_gatt_status_t hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status);
static wiced_bt_gatt_status_t hello_client_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data );
static wiced_bt_gatt_status_t hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static void                   hello_client_process_data_from_peripheral(uint16_t conn_id, int len, uint8_t *data );
static wiced_bt_gatt_status_t hello_client_gatt_write_handler(uint16_t conn_id,
                                                              wiced_bt_gatt_opcode_t opcode,
                                                              wiced_bt_gatt_write_req_t* p_data);
static wiced_bt_gatt_status_t hello_client_gatt_read_handler(uint16_t conn_id,
                                                             wiced_bt_gatt_opcode_t opcode,
                                                             wiced_bt_gatt_read_t *p_read_req,
                                                             uint16_t len_requested);
static wiced_bt_gatt_status_t hello_client_gatts_req_read_by_type_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested);
/******************************************************************************
 *  Function Definitions
 ******************************************************************************/
/*
 * helper function to init GATT
 *
 */
wiced_bt_gatt_status_t hello_client_gatt_init(void)
{
    return wiced_bt_gatt_db_init( hello_client_gatt_database, hello_client_gatt_database_size, headset_db_hash );
}

/*
 * This function writes into peer's client configuration descriptor to enable notifications
 */
void hello_client_gatt_enable_notification ( void )
{
    wiced_bt_gatt_status_t status;
    uint16_t               u16 = GATT_CLIENT_CONFIG_NOTIFICATION;
    wiced_bt_gatt_write_hdr_t enable_notif = { 0 };

    // Allocating a buffer to send the write request
    uint8_t* val = hello_client_alloc_buffer(sizeof(uint16_t));

    if ( val )
    {
        WICED_MEMCPY(val, &u16, sizeof(uint16_t));
        enable_notif.auth_req = GATT_AUTH_REQ_NONE;
        enable_notif.handle = HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC; /* hard coded server ccd */
        enable_notif.offset = 0;
        enable_notif.len = 2;

        // Register with the server to receive notification
        status = wiced_bt_gatt_client_send_write(g_hello_client.conn_id, GATT_REQ_WRITE, &enable_notif, val, (wiced_bt_gatt_app_context_t)hello_client_free_buffer);

        WICED_BT_TRACE("wiced_bt_gatt_send_write \n", status);
    }
    UNUSED_VARIABLE(status);
}

/*
 * Callback function is executed to process various GATT events
 */
wiced_bt_gatt_status_t hello_client_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "==>hello_client_gatt_callback event %d :\n", event );

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            WICED_BT_TRACE("GATT_CONNECTION_STATUS_EVT\n");
            if ( p_data->connection_status.connected )
            {
                result = hello_client_gatt_connection_up( &p_data->connection_status );
            }
            else
            {
                result = hello_client_gatt_connection_down( &p_data->connection_status );
            }
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            WICED_BT_TRACE("GATT_DISCOVERY_RESULT_EVT %d\n", p_data->discovery_result.discovery_type);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            WICED_BT_TRACE("GATT_DISCOVERY_CPLT_EVT\n");

            /* Configure to receive notification from server */
            hello_client_gatt_enable_notification();
            break;

        case GATT_OPERATION_CPLT_EVT:
            WICED_BT_TRACE("GATT_OPERATION_CPLT_EVT\n");
            result = hello_client_gatt_op_comp_cb( &p_data->operation_complete );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            WICED_BT_TRACE("GATT_ATTRIBUTE_REQUEST_EVT\n");
            result = hello_client_gatt_req_cb( &p_data->attribute_request );
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            WICED_BT_TRACE("GATT_GET_RESPONSE_BUFFER_EVT\n");
            p_data->buffer_request.buffer.p_app_rsp_buffer = hello_client_alloc_buffer (p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt = hello_client_free_buffer;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
            WICED_BT_TRACE("GATT_APP_BUFFER_TRANSMITTED_EVT\n");
            {
                pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;
                if (pfn_free)
                {
                    pfn_free(p_data->buffer_xmitted.p_app_data);
                }
            }
            break;

        default:
            WICED_BT_TRACE("default\n");
            break;
    }

    return result;
}

/* This function is invoked on button interrupt events */
void hello_client_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;
    int             num_peripherals = 0;
    static uint32_t button_pushed_time = 0;
    static uint32_t previous_timer_count = 0;

    //WICED_BT_TRACE( "But1 %d, But2 %d, But3 %d \n", value & 0x01, ( value & 0x02 ) >> 1, ( value & 0x04 ) >> 2 );
    WICED_BT_TRACE( "==>hello_client_interrupt_handler, app timer :%d, previou_timer_count :%d\n", g_hello_client.app_timer_count, previous_timer_count );

    if ((button_pushed_time == 0) && (previous_timer_count != g_hello_client.app_timer_count))
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = g_hello_client.app_timer_count;
        previous_timer_count = g_hello_client.app_timer_count;
    }
    else
    {
        button_pushed_time = 0;
        WICED_BT_TRACE( " Button released\n" );
        //Start the scan if the button is pressed for more than 5 seconds
        if ( g_hello_client.app_timer_count - previous_timer_count > 5 )
        {
            num_peripherals = hello_client_get_num_peripherals();
            WICED_BT_TRACE( " after more than 5s, connecting to next peripheral number %d\n", num_peripherals+1 );

            if ( num_peripherals < HELLO_CLIENT_MAX_PERIPHERALS )
            {
                start_scan = 1;
                if( wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE )
                {
                    result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, hello_client_scan_result_cback );
                    WICED_BT_TRACE( "\nhello_client_interrupt_handler wiced_bt_ble_scan: %d\n", result );
                }
            }
            else
            {
                WICED_BT_TRACE(" Scan Not Started. Connected to HELLO_CLIENT_MAX_PERIPHERALS!! \n" );
            }
        }
        else
        {
            num_peripherals = hello_client_get_num_peripherals();
            WICED_BT_TRACE( " before less than 5s, num_peripherals %d\n", num_peripherals);

            for(int i = 0; i < num_peripherals; i++)
            {
                uint8_t *notif_data = hello_client_alloc_buffer(hello_client_notify_value_size);

                if(notif_data)
                {
                    WICED_BT_TRACE( " sending notifications to conn_id %d\n", g_hello_client.peer_info[i].conn_id);
                    WICED_MEMCPY(notif_data, hello_client_notify_value, hello_client_notify_value_size);
                    wiced_bt_gatt_server_send_notification( g_hello_client.peer_info[i].conn_id,
                                                            HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,
                                                            hello_client_notify_value_size,
                                                            notif_data,
                                                            hello_client_free_buffer);
                }
            }
        }
    }
}

/******************************************************************************
 *  Private Function Definitions
 ******************************************************************************/

/* This function will be called on every connection establishment */
/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_bt_dev_status_t status = 0;
    wiced_bt_gatt_discovery_param_t discovery_param = { 0 };

    if ( g_hello_client.num_connections > HELLO_CLIENT_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE("g_hello_client max connect limit!\n");
        wiced_bt_gatt_disconnect( p_conn_status->conn_id );
        return WICED_BT_GATT_SUCCESS;
    }

    WICED_BT_TRACE( "==>hclient_connection_up Conn Id:%d Num conn:%d,Addr:<%B> role:%d\n ",
                     p_conn_status->conn_id,
                     g_hello_client.num_connections,
                     p_conn_status->bd_addr,
                     p_conn_status->link_role );

    // Keep number of active connections
    g_hello_client.num_connections++;

    // Adding the peer info
    hello_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, p_conn_status->link_role, p_conn_status->transport, p_conn_status->addr_type );

    // This application supports single connection to central (phone) and multiple connections to peripherals (hello_sensors)
    if ( p_conn_status->link_role == HCI_ROLE_CENTRAL )
    {
        g_hello_client.conn_id = p_conn_status->conn_id;

        /* Start with a service discovery */
        discovery_param.s_handle = 1;
        discovery_param.e_handle = 0xFFFF;
        wiced_bt_gatt_client_send_discover (p_conn_status->conn_id, GATT_DISCOVER_SERVICES_ALL, &discovery_param);
    }
    else // Connected as peripheral
    {
        // Update connection params
        wiced_bt_l2cap_update_ble_conn_params( p_conn_status->bd_addr, 112, 128, 0, 200 );

        // Update the connection handle to the central
        g_hello_client.central_conn_id = p_conn_status->conn_id;

        // Stop the advertisement
        status =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        WICED_BT_TRACE(" [%s] start adv status %d \n", __FUNCTION__, status);
    }
    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t hello_client_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data )
{
    wiced_result_t              status;
    hello_client_peer_info_t    *p_peer_info = NULL;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    WICED_BT_TRACE("==>hello_client_gatt_op_comp_cb conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
    case GATTC_OPTYPE_READ_HANDLE:
    case GATTC_OPTYPE_READ_BY_TYPE:
    case GATTC_OPTYPE_READ_MULTIPLE:
        WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
        break;

    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
    case GATTC_OPTYPE_PREPARE_WRITE:
        WICED_BT_TRACE( "write_rsp status:%d\n", p_data->status );

        /* server puts authentication requirement. Encrypt the link */
        if( ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION ) && ( p_data->response_data.handle == HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC ) )
        {
            if ( ( p_peer_info = hello_client_get_peer_information( p_data->conn_id ) ) != NULL )
            {
                if ( hello_client_is_device_bonded(p_peer_info->peer_addr) )
                {
                    status = wiced_bt_dev_set_encryption( p_peer_info->peer_addr, p_peer_info->transport, &encryption_type );
                    WICED_BT_TRACE( "wiced_bt_dev_set_encryption %d \n", status );
                }
                else
                {
                    status = wiced_bt_dev_sec_bond( p_peer_info->peer_addr, p_peer_info->addr_type,
                                                        p_peer_info->transport,0, NULL );
                    WICED_BT_TRACE( "wiced_bt_dev_sec_bond %d \n", status );
                }
            }
        }
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
        WICED_BT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        hello_client_process_data_from_peripheral( p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        break;

    case GATTC_OPTYPE_INDICATION:
        hello_client_process_data_from_peripheral( p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        wiced_bt_gatt_client_send_indication_confirm( p_data->conn_id, p_data->response_data.handle );
        break;
    }

    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}


/*
 * Process various GATT requests received from the central
 */
wiced_bt_gatt_status_t hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE( "==>hello_sensor_gatts_req_cb. conn %d, type %d\n", p_req->conn_id, p_req->opcode );

    switch (p_req->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            result = hello_client_gatt_read_handler(p_req->conn_id,
                                                    p_req->opcode,
                                                    &p_req->data.read_req,
                                                    p_req->len_requested);
            break;
        case GATT_REQ_READ_BY_TYPE:
            result = hello_client_gatts_req_read_by_type_handler (p_req->conn_id, p_req->opcode, &p_req->data.read_by_type, p_req->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            result = hello_client_gatt_write_handler(p_req->conn_id,
                                                     p_req->opcode,
                                                     &(p_req->data.write_req));
            if ((p_req->opcode == GATT_REQ_WRITE) && (result == WICED_BT_GATT_SUCCESS))
            {
                wiced_bt_gatt_write_req_t* p_write_request = &p_req->data.write_req;
                wiced_bt_gatt_server_send_write_rsp(p_req->conn_id,
                                                    p_req->opcode,
                                                    p_write_request->handle);
            }
            break;

        case GATT_REQ_MTU:
            WICED_BT_TRACE("peer mtu:%d\n", p_req->data.remote_mtu);
            wiced_bt_gatt_server_send_mtu_rsp(p_req->conn_id,
                                              p_req->data.remote_mtu,
                                              wiced_bt_cfg_ble.ble_max_rx_pdu_size);
            break;

       default:
            WICED_BT_TRACE("Invalid GATT request conn_id:%d opcode:%d\n",
                           p_req->conn_id, p_req->opcode);
            break;
    }

    return result;
}

/*
 * This function handles notification/indication data received fromt the peripheral device
 */
void hello_client_process_data_from_peripheral( uint16_t conn_id, int len, uint8_t *data )
{
    WICED_BT_TRACE("==>hello_client_process_data_from_peripheral len:%d peripheral conn_id:%d ccc:%d\n",
            len, conn_id, g_hello_client.host_info.characteristic_client_configuration );

    WICED_BT_TRACE("data = %s\n", data);

    uint8_t *notif_data = hello_client_alloc_buffer(len);
    if(notif_data)
    {
        WICED_MEMCPY(notif_data, data, len);

        // if central allows notifications, forward received data from the peripheral
        if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION ) != 0 )
        {
            WICED_BT_TRACE("send notification data[%d] = %s to connection id %d\n", len, notif_data, g_hello_client.central_conn_id);
            wiced_bt_gatt_server_send_notification( g_hello_client.central_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, notif_data, hello_client_free_buffer);
        }
        else if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_INDICATION ) != 0 )
        {
            WICED_BT_TRACE("send indication data[%d] = %s to connection id %d\n", len, notif_data, g_hello_client.central_conn_id);
            wiced_bt_gatt_server_send_indication( g_hello_client.central_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, notif_data, hello_client_free_buffer);
        }
        else
        {
            hello_client_free_buffer(notif_data);
        }
    }
}

/*
 * Process Read request from peer device
 */
wiced_bt_gatt_status_t hello_client_gatt_read_handler(uint16_t conn_id,
                                                      wiced_bt_gatt_opcode_t opcode,
                                                      wiced_bt_gatt_read_t *p_read_req,
                                                      uint16_t len_requested)
{
    const gatt_attribute_t  *puAttribute;
    uint8_t                 *from;
    int                     len_to_send;

    if ((puAttribute = hello_client_get_attribute(p_read_req->handle)) == NULL)
    {
        WICED_BT_TRACE("[%s] read_hndlr attr not found hdl:%x\n",
                        __FUNCTION__,
                        p_read_req->handle );

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                                            p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    WICED_BT_TRACE("[%s] read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n",
                    __FUNCTION__, conn_id,
                    p_read_req->handle,
                    p_read_req->offset,
                    puAttribute->attr_len );

    if (p_read_req->offset >= puAttribute->attr_len )
    {
        WICED_BT_TRACE("[%s] offset:%d larger than attribute length:%d\n",
                        __FUNCTION__,
                        p_read_req->offset,
                        puAttribute->attr_len);

        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);

        return WICED_BT_GATT_INVALID_OFFSET;
    }

    len_to_send = MIN(len_requested, puAttribute->attr_len - p_read_req->offset);

    if (len_to_send < 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    from = ((uint8_t *)puAttribute->p_attr) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, len_to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write read-by-type request from peer device
 */
static wiced_bt_gatt_status_t hello_client_gatts_req_read_by_type_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    const gatt_attribute_t *puAttribute;
    uint16_t    attr_handle = p_read_req->s_handle;
    uint8_t     *p_rsp = hello_client_alloc_buffer (len_requested);
    uint8_t    pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE ("[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        /// Add your code here
        attr_handle = wiced_bt_gatt_find_handle_by_type (attr_handle, p_read_req->e_handle, &p_read_req->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = hello_client_get_attribute (attr_handle)) == NULL)
        {
            WICED_BT_TRACE ("[%s]  found type but no attribute ??\n", __FUNCTION__);
            wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            hello_client_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used, len_requested - used, &pair_len,
                attr_handle, puAttribute->attr_len, puAttribute->p_attr);
            if (filled == 0) {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        WICED_BT_TRACE ("[%s]  attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\n",
            __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE);
        hello_client_free_buffer (p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp (conn_id, opcode, pair_len, used, p_rsp, hello_client_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t hello_client_gatt_write_handler(uint16_t conn_id,
                                                       wiced_bt_gatt_opcode_t opcode,
                                                       wiced_bt_gatt_write_req_t* p_data)
{
    uint8_t* p_attr = p_data->p_val;

    WICED_BT_TRACE("write_handler: conn %d hdl %d opcode %d off %d len %d \n ", conn_id, p_data->handle, opcode, p_data->offset, p_data->val_len);

    if (p_data->handle == HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC)
    {
        g_hello_client.host_info.characteristic_client_configuration = p_attr[0] | (p_attr[1] << 8);
    }

    return WICED_BT_GATT_SUCCESS;
}

uint8_t * hello_client_alloc_buffer(int len)
{
    uint8_t *p = (uint8_t *)wiced_bt_get_buffer(len);
    WICED_BT_TRACE("==>%s len %d alloc 0x%x\n", __FUNCTION__, len, p);

    return p;
}

void hello_client_free_buffer(uint8_t *p_data)
{
    wiced_bt_free_buffer(p_data);

    WICED_BT_TRACE("==>%s 0x%x\n", __FUNCTION__, p_data);
}

