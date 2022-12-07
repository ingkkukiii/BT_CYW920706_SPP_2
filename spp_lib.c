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
 * This file implements interface and main unit of the SPP library
 *
 */
#include "wiced_bt_dev.h"
#include "string.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_trace.h"
#include "wiced_hci.h"
#include "wiced_bt_sdp.h"

#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#include "hci_control_api.h"
#include "spp_int.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_bt_utils.h"

/*****************************************************************************
 *  Constants
 *****************************************************************************/

/*****************************************************************************
 *  Data types
 *****************************************************************************/
typedef enum
{
    INDEX_TYPE_PORT_HANDLE,
    INDEX_TYPE_STATE_OPENING,
    INDEX_TYPE_STATE_IDLE,
} spp_scb_index_type_t;

/******************************************************
 *  External Functions
 ******************************************************/
extern void spp_rfcomm_opened(spp_scb_t *p_scb);
extern void spp_rfcomm_control_callback(uint32_t port_status, uint16_t port_handle);

/******************************************************
 *  Global Variables
 ******************************************************/

// Session control block.  Currently support 1.
spp_scb_t   spp_scb[SPP_MAX_CONNECTIONS];
wiced_bool_t spp_initialized = WICED_FALSE;

wiced_port_status_t spp_port_status;

BD_ADDR     bd_addr_connected;

static wiced_bt_uuid_t  spp_uuid =
{
    .len = 2,
    .uu =
    {
        .uuid16 = UUID_SERVCLASS_SERIAL_PORT
    },
};

uint32_t    spp_bytes_rxed = 0;

/******************************************************
 *  Function Declartions
 ******************************************************/
void                spp_rfcomm_closed(spp_scb_t *p_scb);
void                spp_rfcomm_acceptor_opened(spp_scb_t *p_scb);
spp_scb_t*          spp_lib_get_scb_pointer( spp_scb_index_type_t index, uint16_t port_handle );

static void         spp_rfcomm_start_server(spp_scb_t *p_scb);
static void         spp_port_event_cback(wiced_bt_rfcomm_port_event_t event, uint16_t port_handle);
static void         spp_sdp_start_discovery(spp_scb_t *p_scb);
static wiced_bool_t spp_sdp_find_attr(spp_scb_t *p_scb);
static void         spp_sdp_cback(uint16_t status);
static void         spp_sdp_free_db(spp_scb_t *p_scb);
static void         spp_rfcomm_do_close(spp_scb_t *p_scb);
static spp_scb_t*   spp_lib_find_scb_by_bdaddr(BD_ADDR bd_addr);

/******************************************************
 *  Function Definitions
 ******************************************************/
/*
 * Start up the SPP service.
 */
wiced_result_t wiced_bt_spp_startup(wiced_bt_spp_reg_t* p)
{
    uint8_t i = 0;

    if (!spp_initialized)
    {
        memset(spp_scb, 0, sizeof(spp_scb));
        spp_initialized = WICED_TRUE;
    }

    while( i < SPP_MAX_CONNECTIONS )
    {
        if( spp_scb[i].in_use == WICED_FALSE )
            break;
        i++;
    }

    if( i == SPP_MAX_CONNECTIONS )
        return WICED_BT_NO_RESOURCES;

    memset( &spp_scb[i], 0, sizeof(spp_scb_t) );

    spp_scb[i].in_use = WICED_TRUE;

    // Application passes the scn that it used to create SDP record.  We need to use the
    // same when creating service connection.
    spp_scb[i].p_spp_reg = p;

    /* start RFCOMM server */
    spp_rfcomm_start_server(&spp_scb[i]);

    return WICED_BT_SUCCESS;
}

/*
 * Opens a connection to SPP device.  When connection is opened callback
 * function is executed.
 */
wiced_result_t wiced_bt_spp_connect(BD_ADDR bd_addr)
{
    int                          i;
    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_STATE_IDLE , 0 );

    if( p_scb == NULL )
        return WICED_BT_NO_RESOURCES;

    if(p_scb->in_use == WICED_FALSE)
    {
	/* Initialize SCB and link callbacks to spp_scb[0] */
	memset( p_scb, 0, sizeof(spp_scb_t) );
	p_scb->p_spp_reg = spp_scb[0].p_spp_reg;
	p_scb->in_use = WICED_TRUE;
    }

    if (p_scb->state != SPP_SESSION_STATE_IDLE)
    {
        SPP_TRACE ("spp_connect - no free control block for connection, State: %u\n", p_scb->state);
        return WICED_BT_ERROR;
    }

    p_scb->state = SPP_SESSION_STATE_OPENING;

    /* store parameters */
    for (i = 0; i < BD_ADDR_LEN; i++)
        p_scb->server_addr[i] = bd_addr[BD_ADDR_LEN - 1 - i];

    /* close RFCOMM server, if listening on this SCB */
    if (p_scb->rfc_serv_handle)
    {
        wiced_bt_rfcomm_remove_connection ( p_scb->rfc_serv_handle, WICED_TRUE );
        p_scb->rfc_serv_handle = 0;
    }
    else
    {
	/* Setting server handle to invalid to prevent incorrectly starting server upon disconnection */
	p_scb->rfc_serv_handle = RFCOMM_INVALID_HANDLE;
    }

    /* set role */
    p_scb->b_is_initiator = WICED_TRUE;

    /* do service search */
    spp_sdp_start_discovery(p_scb);

    return WICED_BT_SUCCESS;
}

/*
 * Close the current connection to an iOS device.
 */
wiced_result_t wiced_bt_spp_disconnect(uint16_t handle)
{
    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_PORT_HANDLE , handle );

    if( p_scb == NULL )
        return WICED_BT_ERROR;

    SPP_TRACE("spp_disconnect   State: %u\n", p_scb->state);

    if (p_scb->state == SPP_SESSION_STATE_OPENING)
    {
        p_scb->state = SPP_SESSION_STATE_CLOSING;
        spp_rfcomm_do_close(p_scb);
    }
    else if (p_scb->state == SPP_SESSION_STATE_OPEN)
    {
        p_scb->state = SPP_SESSION_STATE_CLOSING;
        spp_rfcomm_do_close(p_scb);
    }

    return WICED_BT_SUCCESS;
}

/*
 * Disable or enable flow from RFCOMM
 */
void wiced_bt_spp_rx_flow_enable(uint16_t handle, wiced_bool_t enable)
{

    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_PORT_HANDLE , handle );

    if( p_scb == NULL )
    {
        SPP_TRACE( "ERROR spp_rx_flow_enable - cannot find scb based on handle %d\n", handle );
        return;
    }

    wiced_bt_rfcomm_flow_control (p_scb->rfc_conn_handle, enable);
}

/*
 * Return the connection state
 */
uint8_t wiced_bt_spp_get_connection_state(BD_ADDR bd_addr)
{
    spp_scb_t *p_scb = spp_lib_find_scb_by_bdaddr(bd_addr);
    if (p_scb)
    {
        return p_scb->state;
    }
    else
    {
        return SPP_SESSION_STATE_IDLE;
    }
}

/*
 * Setup RFCOMM server for use by HS.
 */
void spp_rfcomm_start_server(spp_scb_t *p_scb)
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    p_scb->state = SPP_SESSION_STATE_IDLE;

    if(p_scb->rfc_serv_handle == RFCOMM_INVALID_HANDLE)
    {
        p_scb->rfc_serv_handle = 0;
    }
    else if (!p_scb->rfc_serv_handle)
    {
        rfcomm_result = wiced_bt_rfcomm_create_connection(UUID_SERVCLASS_SERIAL_PORT,
                                                           p_scb->p_spp_reg->rfcomm_scn, WICED_TRUE, p_scb->p_spp_reg->rfcomm_mtu,
                                                           bd_addr_any, &p_scb->rfc_serv_handle,
                                                           (wiced_bt_port_mgmt_cback_t *)spp_rfcomm_control_callback);

        if (rfcomm_result != WICED_BT_RFCOMM_SUCCESS)
        {
            SPP_TRACE("ERR: spp_rfcomm_start_server: wiced_bt_rfcomm_create_connection failed %d\n",
                    rfcomm_result);
        }
        else
        {
            SPP_TRACE("spp_rfcomm_start_server: rfcomm_create Port: 0x%04x\n",
                    p_scb->rfc_serv_handle);
        }
    }
}

/*
 * Open an RFCOMM connection to the peer device.
 */
void spp_rfcomm_do_open(spp_scb_t *p_scb)
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    rfcomm_result = wiced_bt_rfcomm_create_connection(UUID_SERVCLASS_SERIAL_PORT,
                                                      p_scb->server_scn, WICED_FALSE,
                                                      p_scb->p_spp_reg->rfcomm_mtu,
                                                      p_scb->server_addr,
                                                      &p_scb->rfc_conn_handle,
                                                      (wiced_bt_port_mgmt_cback_t *)spp_rfcomm_control_callback);

    SPP_TRACE("spp_rfcomm_do_open - rfcomm_create Res: 0x%x   Port: 0x%04x\n", rfcomm_result, p_scb->rfc_conn_handle);

    if (rfcomm_result != WICED_BT_RFCOMM_SUCCESS)
    {
        /* TBD Pass back that the connection attempt failed */
        if(p_scb->p_spp_reg->p_connection_failed_callback)
        {
            p_scb->p_spp_reg->p_connection_failed_callback();
        }

        spp_rfcomm_start_server(p_scb);
    }

}

/*
 * Close RFCOMM connection.
 */
void spp_rfcomm_do_close(spp_scb_t *p_scb)
{
    wiced_bt_rfcomm_result_t rfcomm_result;

    if (p_scb->rfc_conn_handle)
    {
        // Close connection keeping server listening
        p_scb->state = SPP_SESSION_STATE_CLOSING;
        rfcomm_result = wiced_bt_rfcomm_remove_connection(p_scb->rfc_conn_handle, WICED_FALSE);

        UNUSED_VARIABLE(rfcomm_result);
        SPP_TRACE("wiced_bt_rfcomm_remove_connection (0x%04x) result 0x%x\n", p_scb->rfc_conn_handle, rfcomm_result);
    }
    else
    {
        spp_rfcomm_start_server (p_scb);
    }
}


/*
 * RFCOMM connection closed.
 */
void spp_rfcomm_closed(spp_scb_t *p_scb)
{
    uint16_t handle = p_scb->rfc_conn_handle;

    /* call appropriate close cback */
    if ((p_scb->state == SPP_SESSION_STATE_OPENING) || (!p_scb->b_is_initiator && (p_scb->state == SPP_SESSION_STATE_IDLE)))
    {
        if(p_scb->p_spp_reg->p_connection_failed_callback)
        {
            p_scb->p_spp_reg->p_connection_failed_callback();
        }
    }

    /* Clear peer bd_addr */
    memset(p_scb->server_addr, 0, BD_ADDR_LEN);

    p_scb->state = SPP_SESSION_STATE_IDLE;
    p_scb->rfc_conn_handle = 0;

    /* Reopen server if needed */
    spp_rfcomm_start_server(p_scb);

    p_scb->p_spp_reg->p_connection_down_callback(handle);
}


/*
 * Handle RFCOMM channel opened when accepting connection.
 */
void spp_rfcomm_acceptor_opened(spp_scb_t *p_scb)
{
    uint16_t lcid;
    int      status;

    /* set role and connection handle */
    p_scb->b_is_initiator  = WICED_FALSE;
    p_scb->rfc_conn_handle = p_scb->rfc_serv_handle;

    /* get bd addr of peer */
    if (WICED_BT_RFCOMM_SUCCESS != (status = wiced_bt_rfcomm_check_connection(p_scb->rfc_conn_handle, p_scb->server_addr, &lcid)))
    {
        SPP_TRACE("spp_rfcomm_acceptor_opened error PORT_CheckConnection returned status %d\n", status);
        spp_rfcomm_closed(p_scb);
    }
    else
    {
        /* continue with common open processing */
        spp_rfcomm_opened(p_scb);
    }
}


/*
 * SDP callback function.
 */
static void spp_sdp_cback(uint16_t sdp_status)
{
    spp_scb_t *p_scb = spp_lib_get_scb_pointer( INDEX_TYPE_STATE_OPENING , 0 );

    if( p_scb == NULL )
    {
        SPP_TRACE( "ERROR spp_sdp_cback - cannot find scb in opening state\n" );
        return;
    }

    SPP_TRACE("spp_sdp_cback status:0x%x\n", sdp_status);

    if ((sdp_status == WICED_BT_SDP_SUCCESS) || (sdp_status == WICED_BT_SDP_DB_FULL))
    {
        if (spp_sdp_find_attr(p_scb))
        {
            spp_rfcomm_do_open(p_scb);
        }
        else
        {
            /* reopen server and notify app of the failure */
            p_scb->state = SPP_SESSION_STATE_IDLE;
            spp_rfcomm_start_server(p_scb);
            p_scb->p_spp_reg->p_service_not_found_callback();
        }
    }
    else
    {
        p_scb->state = SPP_SESSION_STATE_IDLE;
        spp_rfcomm_start_server(p_scb);
        if(p_scb->p_spp_reg->p_connection_failed_callback)
        {
            p_scb->p_spp_reg->p_connection_failed_callback();
        }
    }
    spp_sdp_free_db(p_scb);
}

/*
 * Process SDP discovery results to find requested attributes for requested service.
 * Returns WICED_TRUE if results found, WICED_FALSE otherwise.
 */
wiced_bool_t spp_sdp_find_attr(spp_scb_t *p_scb)
{
    wiced_bt_sdp_discovery_record_t     *p_rec = (wiced_bt_sdp_discovery_record_t *) NULL;
    wiced_bt_sdp_protocol_elem_t        pe;
    wiced_bool_t                        result = WICED_TRUE;

    SPP_TRACE("Looking for SPP service\n");

    p_rec = wiced_bt_sdp_find_service_uuid_in_db((wiced_bt_sdp_discovery_db_t *)p_scb->p_sdp_discovery_db, &spp_uuid, p_rec);
    if (p_rec == NULL)
    {
        SPP_TRACE("spp_sdp_find_attr() - could not find SPP service\n");
        return (WICED_FALSE);
    }

    /*** Look up the server channel number in the protocol list element ***/
    if (wiced_bt_sdp_find_protocol_list_elem_in_rec(p_rec, UUID_PROTOCOL_RFCOMM, &pe))
    {
        SPP_TRACE("spp_sdp_find_attr - num of proto elements -RFCOMM =0x%x\n",  pe.num_params);
        if (pe.num_params > 0)
        {
            p_scb->server_scn = (uint8_t)pe.params[0];
            SPP_TRACE("spp_sdp_find_attr - found SCN in SDP record. SCN=0x%x\n", p_scb->server_scn);
        }
        else
            result = WICED_FALSE;
    }
    else
    {
        result = WICED_FALSE;
    }
    return result;
}


/*
 * Do service discovery.
 */
void spp_sdp_start_discovery(spp_scb_t *p_scb)
{
    uint16_t        attr_list[4];
    uint8_t         num_attr;
    wiced_bool_t    result;

    /* We need to get Service Class (to compare UUID and Protocol Description to get SCN to connect */
    attr_list[0] = ATTR_ID_SERVICE_CLASS_ID_LIST;
    attr_list[1] = ATTR_ID_PROTOCOL_DESC_LIST;
    num_attr = 2;

    /* allocate buffer for sdp database */
    p_scb->p_sdp_discovery_db = wiced_bt_get_buffer(1024);

    /* set up service discovery database; attr happens to be attr_list len */
    result = wiced_bt_sdp_init_discovery_db((wiced_bt_sdp_discovery_db_t *)p_scb->p_sdp_discovery_db, 1024, 1, &spp_uuid, num_attr, attr_list);

    /* initiate service discovery */
    if ((result == WICED_FALSE) || (!wiced_bt_sdp_service_search_attribute_request(p_scb->server_addr, (wiced_bt_sdp_discovery_db_t *)p_scb->p_sdp_discovery_db, &spp_sdp_cback)))
    {
        WICED_BT_TRACE("spp_sdp_start_discovery - Not started. prior db_init result=%d\n", result);

        /* Service discovery not initiated - free discover db, reopen server, tell app  */
        spp_sdp_free_db(p_scb);

        spp_rfcomm_start_server(p_scb);
        if(p_scb->p_spp_reg->p_connection_failed_callback)
        {
            p_scb->p_spp_reg->p_connection_failed_callback();
        }
    }
}

/*
 * Free discovery database.
 */
void spp_sdp_free_db(spp_scb_t *p_scb)
{
    if (p_scb->p_sdp_discovery_db != NULL)
    {
        wiced_bt_free_buffer(p_scb->p_sdp_discovery_db);
        p_scb->p_sdp_discovery_db = NULL;
    }
}

spp_scb_t* spp_lib_get_scb_pointer( spp_scb_index_type_t index, uint16_t port_handle )
{
    uint8_t i;
    wiced_bool_t spp_scb_found = WICED_FALSE;

    for( i = 0; i < SPP_MAX_CONNECTIONS; i++ )
    {
        if (((index == INDEX_TYPE_PORT_HANDLE) && (spp_scb[i].rfc_serv_handle == port_handle      )) ||
            ((index == INDEX_TYPE_PORT_HANDLE) && (spp_scb[i].rfc_conn_handle == port_handle      )) ||
            ((index == INDEX_TYPE_STATE_OPENING) && (spp_scb[i].state == SPP_SESSION_STATE_OPENING)) ||
            ((index == INDEX_TYPE_STATE_IDLE) && (spp_scb[i].state == SPP_SESSION_STATE_IDLE      )) )
            spp_scb_found = WICED_TRUE;

        if( spp_scb_found )
            return &spp_scb[i];
    }
    return NULL;
}

/*
 * Find scb by bdaddr
 */
static spp_scb_t* spp_lib_find_scb_by_bdaddr(BD_ADDR bd_addr)
{
    uint8_t i;
    spp_scb_t *p_scb;

    for( i = 0; i < SPP_MAX_CONNECTIONS; i++ )
    {
        p_scb = &spp_scb[i];
        if ((p_scb->in_use) && (utl_bdcmp(p_scb->server_addr, bd_addr) == 0))
        {
            SPP_TRACE ("spp_lib_find_scb_by_bdaddr(), index:%d", i);
            return p_scb;
        }
    }

    SPP_TRACE("spp_lib_find_scb_by_bdaddr(), not found");
    return NULL;
}

uint8_t wiced_bt_spp_port_purge(uint16_t handle, uint8_t purge_flags)
{
    return PORT_Purge(handle, purge_flags);
}

/*
 * get the rfcomm peer_mtu
 */
uint16_t wiced_bt_spp_rfcomm_get_peer_mtu(uint16_t handle)
{
    memset(&spp_port_status, 0, sizeof(spp_port_status));

    if (wiced_bt_rfcomm_port_get_queue_status (handle, &spp_port_status) == WICED_BT_RFCOMM_SUCCESS)
    {
        SPP_TRACE("handle:%d, flags:0x%x, in_queue_size:%d, out_queue_size:%d, mtu_size:%d\n",
            handle, spp_port_status.flags, spp_port_status.in_queue_size, spp_port_status.out_queue_size, spp_port_status.mtu_size);

        return spp_port_status.mtu_size;
    }

    return 0;
}
