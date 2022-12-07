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
* Prototypes and definitions for WICED SPP implementation
*
*/

#ifndef __SPP_INT_H
#define __SPP_INT_H

#include "wiced.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_spp.h"

#define     SPP_BUFFER_POOL                    2
#define     SPP_MAX_CONNECTIONS                2

//#define SPP_TRACE_ENABLE
#if defined(WICED_BT_TRACE_ENABLE) && defined(SPP_TRACE_ENABLE)
#define     SPP_TRACE                          WICED_BT_TRACE
#else
#define     SPP_TRACE(...)
#endif

#define L2CAP_MIN_OFFSET                        13      /* from l2c_api.h */
#define RFCOMM_MIN_OFFSET                       5       /* from rfc_defs.h */
#define PORT_SUCCESS                            0       /* from port_api.h */
#define RFCOMM_INVALID_HANDLE                   0xFFFF

/* SPP control block */
typedef struct
{
#define     SPP_SESSION_STATE_IDLE       0
#define     SPP_SESSION_STATE_OPENING    1
#define     SPP_SESSION_STATE_OPEN       2
#define     SPP_SESSION_STATE_CLOSING    3

    uint8_t     state;                  /* state machine state */
    wiced_bool_t       in_use;                 /* indicates if control block is in use */

    uint8_t     b_is_initiator;         /* initiator of the connection ( true ) or acceptor ( false ) */

    uint16_t    rfc_serv_handle;        /* RFCOMM server handle */
    uint16_t    rfc_conn_handle;        /* RFCOMM handle of connected service */
    uint8_t     server_scn;             /* server's scn */
    BD_ADDR     server_addr;            /* server's bd address */

    void        *p_sdp_discovery_db;                /* pointer to discovery database */
    uint32_t    flow_control_on;
    wiced_bt_spp_reg_t *p_spp_reg;             /* pointer to application call backs */
    wiced_bt_rfcomm_port_event_t event_error;  /* reflect PORT_EV_ERR */
} spp_scb_t;

extern BD_ADDR              bd_addr_connected;

extern int PORT_Write (uint16_t handle, BT_HDR *p_buf);
extern int PORT_Read (uint16_t handle, BT_HDR **pp_buf);
extern int PORT_Purge (UINT16 handle, UINT8 purge_flags);
extern void *GKI_getpoolbuf (uint8_t pool_id);
extern void *GKI_getbuf (uint16_t);
extern void GKI_freebuf (void *memPtr);
extern uint16_t GKI_get_pool_bufsize (uint8_t pool_id);
extern uint16_t GKI_poolcount (uint8_t pool_id);
extern uint16_t GKI_poolfreecount (uint16_t pool_id);


#endif
