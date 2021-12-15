/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* BLE Hello Client Gatt functions for btstack v1
*
*/
#include <string.h>
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "hello_client.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_transport.h"
#include "wiced_bt_l2c.h"

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#if defined(CYW55572)
 #include "bt_types.h"
#endif
#include "wiced_platform.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_hal_puart.h"
#include "wiced_timer.h"
#if !defined(CYW20706A2) && !defined(CYW43012C0) && !defined(CYW55572)
#include "cycfg_pins.h"
#endif

/******************************************************************************
 *  Type Definitions
 ******************************************************************************/
#define wiced_bt_gatt_write_hdr_t wiced_bt_gatt_value_t

/******************************************************************************
 *  Imported Data Declartions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern hello_client_app_t g_hello_client;
extern const uint8_t hello_client_gatt_database[];
extern size_t hello_client_gatt_database_size;
extern size_t hello_client_notify_value_size;
extern uint8_t hello_client_notify_value[];
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
 *  Function Declartions
 ******************************************************************************/
static wiced_bt_gatt_status_t hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t hello_client_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data );
static wiced_bt_gatt_status_t hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static void                   hello_client_process_data_from_peripheral( uint16_t conn_id, int len, uint8_t *data );
static wiced_bt_gatt_status_t hello_client_gatt_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_read_data );
static wiced_bt_gatt_status_t hello_client_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );

/******************************************************************************
 *  Public Function Definitions
 ******************************************************************************/
/*
 * helper function to init GATT
 *
 */
wiced_bt_gatt_status_t hello_client_gatt_init(void)
{
    return wiced_bt_gatt_db_init( hello_client_gatt_database, hello_client_gatt_database_size );
}

/*
 * This function writes into peer's client configuration descriptor to enable notifications
 */
void hello_client_gatt_enable_notification ( void )
{
    wiced_bt_gatt_status_t status;
    uint16_t               u16 = GATT_CLIENT_CONFIG_NOTIFICATION;

    // Allocating a buffer to send the write request
#ifdef CYW43012C0
    wiced_bt_gatt_write_hdr_t *p_write = ( wiced_bt_gatt_write_hdr_t* )wiced_bt_get_buffer_from_pool( p_hello_client_buffer_pool );
#else
    wiced_bt_gatt_write_hdr_t *p_write = ( wiced_bt_gatt_write_hdr_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_write_hdr_t ) + 2 );
#endif

    if ( p_write )
    {
        uint8_t * value_p = &p_write->value[0];
        p_write->handle   = HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC; /* hard coded server ccd */
        p_write->offset   = 0;
        p_write->len      = 2;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        value_p[0] = u16 & 0xff;
        value_p[1] = (u16 >> 8) & 0xff;

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( g_hello_client.conn_id, GATT_WRITE, p_write );
        wiced_bt_free_buffer( p_write );
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

    WICED_BT_TRACE( "==>hello_client_gatt_callback event %d \n", event );

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            if ( p_data->connection_status.connected )
            {
                result = hello_client_gatt_connection_up( &p_data->connection_status );
            }
            else
            {
                result = hello_client_gatt_connection_down( &p_data->connection_status );
            }
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = hello_client_gatt_op_comp_cb( &p_data->operation_complete );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = hello_client_gatt_req_cb( &p_data->attribute_request );
            break;

        default:
            break;
    }

    return result;
}

#ifndef CYW43012C0
/* This function is invoked on button interrupt events */
void hello_client_interrupt_handler(void* user_data, uint8_t value )
{
    wiced_result_t  result;
    int             num_peripherals = 0;
    static uint32_t button_pushed_time = 0;

    //WICED_BT_TRACE( "But1 %d, But2 %d, But3 %d \n", value & 0x01, ( value & 0x02 ) >> 1, ( value & 0x04 ) >> 2 );
    WICED_BT_TRACE( "hello_client_interrupt_handler, app timer :%d\n", g_hello_client.app_timer_count );

#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20819A1) || defined(CYW20721B2) || defined(CYW20719B2) )
    if ( wiced_hal_gpio_get_pin_input_status(WICED_GET_PIN_FOR_BUTTON(WICED_PLATFORM_BUTTON_1)) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1) )
#else
    if ( wiced_hal_gpio_get_pin_input_status(HELLO_CLIENT_GPIO_BUTTON) == HELLO_CLIENT_BUTTON_PRESSED_VALUE )
#endif
    {
        WICED_BT_TRACE( " Button pressed\n" );
        button_pushed_time = g_hello_client.app_timer_count;
    }
    else if ( button_pushed_time != 0 )
    {
        WICED_BT_TRACE( " Button released " );
        //Start the scan if the button is pressed for more than 5 seconds
        if ( g_hello_client.app_timer_count - button_pushed_time > 5 )
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
                WICED_BT_TRACE( " sending notifications to conn id %d\n", g_hello_client.peer_info[i].conn_id);
                wiced_bt_gatt_send_notification( g_hello_client.peer_info[i].conn_id,
                                                 HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL,
                                                 hello_client_notify_value_size,
                                                 hello_client_notify_value );
            }
        }
    }
}
#endif

/******************************************************************************
 *  Private Function Definitions
 ******************************************************************************/

/* This function will be called on every connection establishment */
/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_client_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    uint8_t dev_role;
    wiced_bt_dev_status_t status ;

    if ( g_hello_client.num_connections > HELLO_CLIENT_MAX_CONNECTIONS )
    {
        WICED_BT_TRACE("g_hello_client max connect limit!\n");
        wiced_bt_gatt_disconnect( p_conn_status->conn_id );
        return WICED_BT_GATT_SUCCESS;
    }

    // Keep number of active connections
    g_hello_client.num_connections++;

    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Adding the peer info
    hello_client_add_peer_info( p_conn_status->conn_id, p_conn_status->bd_addr, dev_role , p_conn_status->transport, p_conn_status->addr_type );

    WICED_BT_TRACE( "hclient_connection_up Conn Id:%d Num conn:%d,Addr:<%B> role:%d\n ",
       p_conn_status->conn_id, g_hello_client.num_connections, p_conn_status->bd_addr, dev_role );

    // This application supports single connection to central (phone) and multiple connections to peripherals (hello_sensors)
    if ( dev_role == HCI_ROLE_CENTRAL )
    {
        g_hello_client.conn_id = p_conn_status->conn_id;
        /* Configure to receive notification from server */
        hello_client_gatt_enable_notification( );
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

    WICED_BT_TRACE("hello_client_gatt_op_comp_cb conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
    case GATTC_OPTYPE_READ:
        WICED_BT_TRACE( "read_rsp status:%d\n", p_data->status );
        break;

    case GATTC_OPTYPE_WRITE:
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

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        hello_client_process_data_from_peripheral( p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        break;

    case GATTC_OPTYPE_INDICATION:
        hello_client_process_data_from_peripheral( p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data );
        wiced_bt_gatt_send_indication_confirm( p_data->conn_id, p_data->response_data.handle );
        break;
    }

    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function handles notification/indication data received fromt the peripheral device
 */
void hello_client_process_data_from_peripheral( uint16_t conn_id, int len, uint8_t *data )
{
    WICED_BT_TRACE("hello_client_process_data_from_peripheral len:%d central conn_id:%d ccc:%d\n",
                    len, conn_id, g_hello_client.host_info.characteristic_client_configuration );

    // if central allows notifications, forward received data from the peripheral
    if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION ) != 0 )
    {
        wiced_bt_gatt_send_notification( g_hello_client.central_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, data );
    }
    else if ( ( g_hello_client.host_info.characteristic_client_configuration & GATT_CLIENT_CONFIG_INDICATION ) != 0 )
    {
        wiced_bt_gatt_send_indication( g_hello_client.central_conn_id, HANDLE_HELLO_CLIENT_SERVICE_CHAR_NOTIFY_VAL, len, data );
    }
}

/*
 * Process Read Request from central
 */
wiced_bt_gatt_status_t hello_client_gatt_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_read_data )
{
    const gatt_attribute_t * puAttribute;
    const uint8_t *          from;
    int                      to_copy;

    WICED_BT_TRACE("read_hndlr conn %d hdl %x\n", conn_id, p_read_data->handle );

    puAttribute = hello_client_get_attribute( p_read_data->handle );
    if ( puAttribute == NULL )
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_copy = puAttribute->attr_len - p_read_data->offset;

    if ( to_copy >= *p_read_data->p_val_len )
    {
        to_copy = *p_read_data->p_val_len - 1;
    }

    from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
    *p_read_data->p_val_len = to_copy;

    memcpy( p_read_data->p_val, from, to_copy );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or command from peer device
 */
wiced_bt_gatt_status_t hello_client_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    uint8_t *p_attr = p_data->p_val;

    WICED_BT_TRACE("write_handler: conn %d hdl %d prep %d off %d len %d \n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    if ( p_data->handle == HANDLE_HELLO_CLIENT_SERVICE_CHAR_CFG_DESC  )
    {
        g_hello_client.host_info.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process various GATT requests received from the central
 */
wiced_bt_gatt_status_t hello_client_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE( "hello_sensor_gatts_req_cb. conn %d, type %d\n", p_req->conn_id, p_req->request_type );

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hello_client_gatt_read_handler( p_req->conn_id, &(p_req->data.read_req ) );
            break;

        case GATTS_REQ_TYPE_WRITE:
            result = hello_client_gatt_write_handler( p_req->conn_id, &(p_req->data.write_req ) );
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE( "peer mtu:%d\n", p_req->data.mtu );
            break;

        default:
            break;
    }

    return result;
}

