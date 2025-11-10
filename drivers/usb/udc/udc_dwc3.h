/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2025 Alif Semiconductor.
 */

#ifndef UDC_DWC3_H
#define UDC_DWC3_H

#include <stdint.h>
#include <zephyr/device.h>
#include "usb_dwc3_hw.h"

#ifdef __cplusplus
/* Yes, C++ compiler is present.  Use standard C.  */
extern   "C" {
#endif

/* Define the macros    */
#define USB_SUCCESS                                    0
#define USB_INIT_ERROR                                -1
#define USB_CORE_INVALID                              -2
#define USB_MODE_MISMATCH                             -3
#define USB_CORE_SFTRST_TIMEOUT_ERROR                 -4
#define USB_CONTROLLER_INIT_FAILED                    -5
#define USB_EP_DIRECTION_WRONG                        -6
#define USB_EP_BUSY_ERROR                             -7
#define USB_EP_BUFF_LENGTH_INVALID                    -8
#define USB_EP_CMD_CMPLT_ERROR                        -9
#define USB_EP_CMD_CMPLT_BUS_EXPIRY_ERROR             -10
#define USB_EP_CMD_CMPLT_NO_RESOURCE_ERROR            -11
#define USB_EP_CMD_CMPLT_STATUS_UNKNOWN               -12
#define USB_EP_CMD_CMPLT_TIMEOUT_ERROR                -13
#define USB_EP_INVALID                                -14
#define USB_LINKSTATE_INVALID                         -15
#define USB_LINKSTATE_RECOVERY_FAILED                 -16
#define USB_REMOTE_WAKEUP_FAILED                      -17
#define USB_LINKSTATE_TIMEOUT_ERROR                   -18
#define USB_LINKSTATE_SET_FAILED                      -19
#define USB_MODE_UNSUPPORTED                          -20
#define USB_EP_UNSUPPORTED                            -30

#define USB_DEVICE_DEFAULT_ADDRESS                     0
#define USB_EVNT_BUFF_ALLOC_ERROR                      100
#define USB_DGCMD_CMPLT_ERROR                          101
#define USB_DGCMD_TIMEOUT_ERROR                        102
#define USB_EP_ENABLE_ERROR                            103
#define USB_EP_RESOURCE_INDEX_INVALID                  105
#define USB_DEVICE_SET_ADDRESS_INVALID                 106
#define USB_DEVICE_ALREADY_CONFIGURED                  107

#define ATTR                                           __attribute__
#define ATTR_ALIGN(CACHELINE)                          ATTR((aligned(CACHELINE)))
#define ATTR_SECTION(sec)                              ATTR((section(sec)))
#define UPPER_32_BITS(n)                               ((uint32_t)(((uint32_t)(n) >> 16) \
								>> 16))
#define LOWER_32_BITS(n)                               ((uint32_t)(n))
#define SET_BIT(REG, BIT_MSK)                          ((REG) |= (BIT_MSK))
#define CLEAR_BIT(REG, BIT_MSK)                        ((REG) &= ~(BIT_MSK))

#define USB_NUM_TRBS                                  8U
#define NO_OF_TRB_PER_EP                              8U
#define USB_NUM_OF_EPS                                16
#define USB_REQUEST_IN                                0x80U
#define USB_SET_ADDRESS_REQ                           0x05
#define USB_DIR_IN                                    1U
#define USB_DIR_OUT                                   0U
#define USB_EVENT_BUFFER_SIZE                         4096
#define USB_EVENT_CNT_SIZE                            4
#define USB_SCRATCHPAD_BUF_SIZE                       4096
#define USB_SETUP_PKT_SIZE                            8

#define USB_CTRL_PHY_EP0                              0
#define USB_CTRL_PHY_EP1                              1

#define USB_DCTL_START_TIMEOUT                        500
#define USB_GENERIC_CMD_TIMEOUT                       500
#define USB_DEPCMD_TIMEOUT                            1000
#define USB_DCTL_CSFTRST_TIMEOUT                      1000
#define USB_TRANSFER_WAKEUP_RETRY                     20000
#define USB_LINK_STATE_RETRY                          10000
#define USB_BULK_EP_MAX_PKT                           512
#define USB_CONTROL_EP_MAX_PKT                        64
#define USB_ISOC_EP_MAX_PKT                           1024

#define USB_CONTROL_EP                                0
#define USB_ISOCRONOUS_EP                             1
#define USB_BULK_EP                                   2
#define USB_INTERRUPT_EP                              3

#define USB_TRB_STRUCTURE_SIZE                        16
#define USB_TRBS_PER_EP                               9
/* USB devices address can be assigned addresses from 1 to 127 */
#define USB_DEVICE_MAX_ADDRESS                        127

#define USB_EP_ENABLED                                (0x00000001U << 0U)
#define USB_EP_STALL                                  (0x00000001U << 1U)
#define USB_EP_WEDGE                                  (0x00000001U << 2U)
#define USB_EP_BUSY                                   (0x00000001U << 4U)
#define USB_EP_PENDING_REQUEST                        (0x00000001U << 5U)
#define USB_EP_MISSED_ISOC                            (0x00000001U << 6U)

/* DEPXFERCFG parameter 0 */
#define USB_DEPXFERCFG_MSK                            0xFFFFU
#define USB_DEPXFERCFG_NUM_XFER_RES(n)                (n & USB_DEPXFERCFG_MSK)
/* The EP number goes 0..31 so ep0 is always out and ep1 is always in */
#define USB_DALEPENA_EP(n)                            (0x00000001U << (n))

/* Define USB endpoint transfer status definition.  */
#define USB_EP_TRANSFER_IDLE                             0
#define USB_EP_TRANSFER_SETUP                            1
#define USB_EP_TRANSFER_DATA_COMPLETION                  2
#define USB_EP_TRANSFER_STATUS_COMPLETION                3

typedef enum udc_dwc3_device_state {
	USB_DWC3_STATE_NOTATTACHED,
	USB_DWC3_STATE_ATTACHED,
	USB_DWC3_STATE_POWERED,
	USB_DWC3_STATE_RECONNECTING,
	USB_DWC3_STATE_UNAUTHENTICATED,
	USB_DWC3_STATE_DEFAULT,
	USB_DWC3_STATE_ADDRESS,
	USBD_DWC3_STATE_CONFIGURED,
	USB_DWC3_STATE_SUSPENDED
} udc_dwc3_device_state_t;

typedef enum udc_dwc3_ep0_state {
	EP0_UNCONNECTED,
	EP0_SETUP_PHASE,
	EP0_DATA_PHASE,
	EP0_STATUS_PHASE,
} udc_dwc3_ep0_state_t;

typedef enum udc_dwc3_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
} udc_dwc3_mode_t;

typedef enum udc_dwc3_phy_interface {
	USB_PHY_INTERFACE_MODE_UNKNOWN,
	USB_PHY_INTERFACE_MODE_UTMI,
	USB_PHY_INTERFACE_MODE_UTMIW,
	USB_PHY_INTERFACE_MODE_ULPI,
	USB_PHY_INTERFACE_MODE_SERIAL,
	USB_PHY_INTERFACE_MODE_HSIC,
} udc_dwc3_phy_interface_t;
/*
 * Endpoint Parameters
 */
typedef struct udc_dwc3_ep_params {
	uint32_t param2;
	uint32_t param1;
	uint32_t param0;
} udc_dwc3_ep_params_t;

/* TRB descriptor structure */
typedef struct udc_dwc3_trb {
	uint32_t buf_ptr_low;
	uint32_t buf_ptr_high;
	uint32_t size;
	uint32_t ctrl;
} udc_dwc3_trb_t;

/* Define USB DWC3 physical endpoint structure.  */
typedef struct udc_dwc3_ep {
	udc_dwc3_trb_t  ep_trb[NO_OF_TRB_PER_EP + 1] ATTR_ALIGN(32);
	uint32_t    ep_status;
	uint8_t     ep_index;
	uint32_t    ep_transfer_status;
	uint8_t     ep_dir;
	uint32_t    ep_maxpacket;
	uint32_t    ep_resource_index;
	uint32_t    ep_requested_bytes;
	uint32_t    trb_enqueue;
	uint32_t    trb_dequeue;
	uint8_t     phy_ep;
	uint32_t    bytes_txed;
	uint32_t    unaligned_txed;
} udc_dwc3_ep_t;

/* USB setup packet structure */
typedef struct udc_ctrl_request {
	uint8_t  bRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} udc_ctrl_request_t;

/* Define USB Event structure definition. */
typedef struct udc_dwc3_event_buffer {
	void       *buf;
	uint32_t   length;
	uint32_t   lpos;
	uint32_t   count;
} udc_dwc3_event_buffer_t;

/**
 * hwparams - copy of HWPARAMS registers
 */
typedef struct udc_dwc3_hwparams {
	uint32_t hwparams0;
	uint32_t hwparams1;
	uint32_t hwparams2;
	uint32_t hwparams3;
	uint32_t hwparams4;
	uint32_t hwparams5;
	uint32_t hwparams6;
	uint32_t hwparams7;
	uint32_t hwparams8;
} udc_dwc3_hwparams_t;

typedef struct udc_dwc3_driver {
	udc_dwc3_reg_t       *regs;
	void                 (*udc_dwc3_device_reset_cb)(struct udc_dwc3_driver *drv);
	void                 (*udc_dwc3_connect_cb)(struct udc_dwc3_driver *drv);
	void                 (*udc_dwc3_disconnect_cb)(struct udc_dwc3_driver *drv);
	void                 (*udc_dwc3_setupstage_cb)(struct udc_dwc3_driver *drv);
	void                 (*udc_dwc3_data_in_cb)(struct udc_dwc3_driver *drv, uint8_t ep_num);
	void                 (*udc_dwc3_data_out_cb)(struct udc_dwc3_driver *drv, uint8_t ep_num);

	udc_ctrl_request_t   setup_data ATTR_ALIGN(32);
	udc_dwc3_trb_t           ep0_trb ATTR_ALIGN(32);
	udc_dwc3_ep_t            eps[USB_NUM_OF_EPS];
	udc_dwc3_event_buffer_t  *event_buf;
	udc_dwc3_ep0_state_t     ep0_state;
	udc_dwc3_device_state_t  config_state;
	udc_dwc3_hwparams_t  hwparams;
	uint32_t             in_eps;
	uint32_t             num_bytes;
	uint32_t             out_eps;
	uint32_t             usb2_phy_config;
	bool                 setup_packet_pending:1;
	bool                 three_stage_setup:1;
	bool                 ep0_expect_in:1;
	uint32_t             revision;
	uint8_t              current_dr_role;
	uint8_t              endp_number;
	uint8_t              dr_mode;
	uint8_t              hsphy_mode;
	uint8_t              fladj;
	uint32_t             event_type;
	uint32_t             actual_length;
} udc_dwc3_driver_t;

void dwc3_reset_cb(udc_dwc3_driver_t *drv);
void dwc3_setupstage_cb(udc_dwc3_driver_t *drv);
void dwc3_disconnect_cb(udc_dwc3_driver_t *drv);
void dwc3_connect_cb(udc_dwc3_driver_t *drv);
void dwc3_data_in_cb(udc_dwc3_driver_t *drv, uint8_t ep_num);
void dwc3_data_out_cb(udc_dwc3_driver_t *drv, uint8_t ep_num);

#ifdef __cplusplus
}
#endif
#endif
