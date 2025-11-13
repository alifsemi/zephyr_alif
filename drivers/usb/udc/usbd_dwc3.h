/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2025 Alif Semiconductor.
 */

#ifndef USBD_DWC3_H
#define USBD_DWC3_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include "soc_memory_map.h"

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
#define USB_NUM_OF_EPS                                (CONFIG_UDC_DWC3_NUM_OF_OUT_EPS + \
					CONFIG_UDC_DWC3_NUM_OF_IN_EPS)

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

#define USB_DEPEVT_XFERCOMPLETE                          0x01
#define USB_DEPEVT_XFERINPROGRESS                        0x02
#define USB_DEPEVT_XFERNOTREADY                          0x03
#define USB_DEPEVT_RXTXFIFOEVT                           0x04
#define USB_DEPEVT_STREAMEVT                             0x06
#define USB_DEPEVT_EPCMDCMPLT                            0x07

/* Control-only Status */
#define USB_DEPEVT_STATUS_CONTROL_DATA                   1U
#define USB_DEPEVT_STATUS_CONTROL_STATUS                 2U
#define USB_DEPEVT_STATUS_CONTROL_DATA_INVALTRB          9U
#define USB_DEPEVT_STATUS_CONTROL_STATUS_INVALTRB        0xAU

/*
 * return Physical EP number as dwc3 mapping
 */
#define USB_GET_PHYSICAL_EP(epnum, direction)            (((epnum) << 1U) | (direction))
#define USB_DEPCMD_MSK                                   0xF
#define USB_DEPCMD_CMD(x)                                ((x) & USB_DEPCMD_MSK)

/* Device endpoint specific events */
#define USB_DEPEVT_CMD_CMPLT_MSK                         0x3C0
#define USB_DEPEVT_CMD_CMPLT_POS                         6
#define USB_GET_DEPEVT_TYPE(reg)                         ((reg & USB_DEPEVT_CMD_CMPLT_MSK) \
								>> USB_DEPEVT_CMD_CMPLT_POS)
/* Device endpoint num  */
#define USB_DEPEVT_EP_NUM_MSK                            0x3E
#define USB_DEPEVT_EP_NUM_POS                            1
#define USB_GET_DEPEVT_EP_NUM(reg)                       ((reg & USB_DEPEVT_EP_NUM_MSK) \
								>> USB_DEPEVT_EP_NUM_POS)
/* Device specific events */
#define USB_DEVT_MSK                                     0x1F00
#define USB_DEVT_POS                                     8
#define USB_DEVT_TYPE(reg)                               ((reg & USB_DEVT_MSK) >> USB_DEVT_POS)
/* device Link state change event info  */
#define USB_DEVT_LINK_STATE_MSK                          0xF0000
#define USB_DEVT_LINK_STATE_POS                          16
#define USB_DEVT_LINK_STATE_INFO(reg)                    ((reg & USB_DEVT_LINK_STATE_MSK) \
								>> USB_DEVT_LINK_STATE_POS)
/* Device EP event status   */
#define USB_EVT_EPSTATUS_POS                             12U
#define USB_EVT_EPSTATUS_MSK                             0xF000
#define USB_GET_EP_EVENT_STATUS(reg)                     ((reg & USB_EVT_EPSTATUS_MSK) \
								>> USB_EVT_EPSTATUS_POS)

/* In response to Start Transfer */
#define USB_DEPEVT_TRANSFER_NO_RESOURCE                  1
#define USB_DEPEVT_TRANSFER_BUS_EXPIRY                   2
#define USB_DEPEVT_CMD_SUCCESS                           0

/* Device Configuration Register */
#define USB_DCFG_DEVADDR_POS                             3
#define USB_DCFG_DEVADDR(addr)                           ((addr) << USB_DCFG_DEVADDR_POS)
#define USB_DCFG_SET_ADDR_MSK                            0x7F
#define USB_DCFG_DEVADDR_MASK                            USB_DCFG_DEVADDR(USB_DCFG_SET_ADDR_MSK)
#define USB_DCFG_SPEED_MASK                              7
#define USB_DCFG_HIGHSPEED                               0
#define USB_DCFG_LOWSPEED                                1U
#define USB_DCFG_NUMP_POS                                17
#define USB_DCFG_NUMP_MSK                                0x3E0000
#define USB_DCFG_NUMP(n)                                 (((n) & USB_DCFG_NUMP_MSK) \
								>> USB_DCFG_NUMP_POS)
#define USB_DCFG_LPM_CAP                                 BIT(22)

/* Device Generic Command Register */
#define USB_DGCMD_SET_LMP                                0x01
#define USB_DGCMD_SET_PERIODIC_PAR                       0x02
#define USB_DGCMD_XMIT_FUNCTION                          0x03
#define USB_DGCMD_CMDACT                                 BIT(10)
#define USB_DGCMD_STATUS_MSK                             0xF000
#define USB_DGCMD_STATUS_POS                             12
#define USB_DGCMD_STATUS(n)                              (((n) & USB_DGCMD_STATUS_MSK) \
								>> USB_DGCMD_STATUS_POS)
#define USB_DGCMD_CMDIOC                                 BIT(8)
#define USB_DGCMD_SET_SCRATCHPAD_ADDR_LO                 0x04
#define USB_DGCMD_SET_SCRATCHPAD_ADDR_HI                 0x05

/* Global HWPARAMS0 Register */
#define USB_GHWPARAMS0_MODE_MSK                          0x3
#define USB_GHWPARAMS0_MODE(n)                           ((n) & USB_GHWPARAMS0_MODE_MSK)
#define USB_GHWPARAMS0_MODE_DEVICE                       0
#define USB_GHWPARAMS0_MODE_HOST                         1
#define USB_GHWPARAMS0_MODE_DRD                          2
#define USB_GHWPARAMS0_MBUS_TYPE_MSK                     0x38
#define USB_GHWPARAMS0_MBUS_TYPE_POS                     3
#define USB_GHWPARAMS0_MBUS_TYPE(n)                      (((n) & USB_GHWPARAMS0_MBUS_TYPE_MSK) \
								>> USB_GHWPARAMS0_MBUS_TYPE_POS)
#define USB_GHWPARAMS0_SBUS_TYPE_MSK                     0xC0
#define USB_GHWPARAMS0_SBUS_TYPE_POS                     0x6
#define USB_GHWPARAMS0_SBUS_TYPE(n)                      (((n) & USB_GHWPARAMS0_SBUS_TYPE_MSK) \
								>> USB_GHWPARAMS0_SBUS_TYPE_POS)
#define USB_GHWPARAMS0_MDWIDTH_MSK                       0xFF00
#define USB_GHWPARAMS0_MDWIDTH_POS                       8
#define USB_GHWPARAMS0_MDWIDTH(n)                        (((n) & USB_GHWPARAMS0_MDWIDTH_MSK) \
								>> USB_GHWPARAMS0_MDWIDTH_POS)
#define USB_GHWPARAMS0_SDWIDTH_MSK                       0xFF0000
#define USB_GHWPARAMS0_SDWIDTH_POS                       16
#define USB_GHWPARAMS0_SDWIDTH(n)                        (((n) & USB_GHWPARAMS0_SDWIDTH_MSK) \
								>> USB_GHWPARAMS0_SDWIDTH_POS)
#define USB_GHWPARAMS0_AWIDTH_MSK                        0xFF000000
#define USB_GHWPARAMS0_AWIDTH_POS                        24
#define USB_GHWPARAMS0_AWIDTH(n)                         (((n) & USB_GHWPARAMS0_AWIDTH_MSK) \
								>> USB_GHWPARAMS0_AWIDTH_POS)

/* Global USB2 PHY Configuration Register */
#define USB_GUSB2PHYCFG_PHYSOFTRST                       BIT(31)
#define USB_GUSB2PHYCFG_U2_FREECLK_EXISTS                BIT(30)
#define USB_GUSB2PHYCFG_SUSPHY                           BIT(6)
#define USB_GUSB2PHYCFG_ULPI_UTMI                        BIT(4)
#define USB_GUSB2PHYCFG_ENBLSLPM                         BIT(8)
#define USB_GUSB2PHYCFG_PHYIF_POS                        3
#define USB_GUSB2PHYCFG_PHYIF(n)                         ((n) << USB_GUSB2PHYCFG_PHYIF_POS)
#define USB_GUSB2PHYCFG_PHYIF_MASK                       USB_GUSB2PHYCFG_PHYIF(1)
#define USB_GUSB2PHYCFG_USBTRDTIM_POS                    10
#define USB_GUSB2PHYCFG_USBTRDTIM(n)                     ((n) << USB_GUSB2PHYCFG_USBTRDTIM_POS)
#define USB_GUSB2PHYCFG_USBTRDTIM_MASK                   USB_GUSB2PHYCFG_USBTRDTIM(0xF)
#define USBTRDTIM_UTMI_8_BIT                             9
#define USBTRDTIM_UTMI_16_BIT                            5
#define UTMI_PHYIF_16_BIT                                1
#define UTMI_PHYIF_8_BIT                                 0
#define USB_GUSB2PHYCFG_ULPIAUTORES_POS                  15
#define USB_GUSB2PHYCFG_ULPIAUTORES                      1U << USB_GUSB2PHYCFG_ULPIAUTORES_POS

/* Global HWPARAMS3 Register */
#define USB_GHWPARAMS3_HSPHY_IFC_POS                     2
#define USB_GHWPARAMS3_HSPHY_IFC_MSK                     (0x3 << USB_GHWPARAMS3_HSPHY_IFC_POS)
#define USB_GHWPARAMS3_HSPHY_IFC(n)                      (((n) & USB_GHWPARAMS3_HSPHY_IFC_MSK) \
								>> USB_GHWPARAMS3_HSPHY_IFC_POS)
#define USB_GHWPARAMS3_HSPHY_IFC_DIS                     0
#define USB_GHWPARAMS3_HSPHY_IFC_UTMI                    1
#define USB_GHWPARAMS3_HSPHY_IFC_ULPI                    2
#define USB_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI               3
#define USB_GHWPARAMS3_FSPHY_IFC_MSK                     0x30
#define USB_GHWPARAMS3_FSPHY_IFC_POS                     4
#define USB_GHWPARAMS3_FSPHY_IFC(n)                      (((n) & USB_GHWPARAMS3_FSPHY_IFC_MSK) \
								>> USB_GHWPARAMS3_FSPHY_IFC_POS)
#define USB_GHWPARAMS3_FSPHY_IFC_DIS                     0
#define USB_GHWPARAMS3_FSPHY_IFC_ENA                     1
#define USB_GHWPARAMS3_NUM_IN_EPS_MSK                    0x7C0000
#define USB_GHWPARAMS3_NUM_IN_EPS_POS                    18
#define USB_GHWPARAMS3_NUM_EPS_MSK                       0x3F000
#define USB_GHWPARAMS3_NUM_EPS_POS                       12

#define USB_NUM_EPS(ep_num)                              ((ep_num & USB_GHWPARAMS3_NUM_EPS_MSK) \
								>> USB_GHWPARAMS3_NUM_EPS_POS)
#define USB_IN_EPS(p)                                    ((p & USB_GHWPARAMS3_NUM_IN_EPS_MSK) \
								>> USB_GHWPARAMS3_NUM_IN_EPS_POS)

/* TRB Control */
#define USB_TRB_CTRL_HWO                                 BIT(0)
#define USB_TRB_CTRL_LST                                 BIT(1)
#define USB_TRB_CTRL_CHN                                 BIT(2)
#define USB_TRB_CTRL_CSP                                 BIT(3)
#define USB_TRB_CTRL_TRBCTL_MSK                          0x3F
#define USB_TRB_CTRL_TRBCTL_POS                          4
#define USB_TRB_CTRL_TRBCTL(n)                           (((n) & USB_TRB_CTRL_TRBCTL_MSK) \
								<< USB_TRB_CTRL_TRBCTL_POS)
#define USB_TRB_CTRL_ISP_IMI                             BIT(10)
#define USB_TRB_CTRL_IOC                                 BIT(11)
#define USB_TRB_CTRL_SID_SOFN_MSK                        0xFFFFU
#define USB_TRB_CTRL_SID_SOFN_POS                        14
#define USB_TRB_CTRL_SID_SOFN(n)                         (((n) & USB_TRB_CTRL_SID_SOFN_MSK) \
								<< USB_TRB_CTRL_SID_SOFN_POS)

#define USB_TRBCTL_TYPE_MSK                              0x3F0
#define USB_TRBCTL_TYPE(n)                               ((n) & USB_TRBCTL_TYPE_MSK)
#define USB_TRBCTL_NORMAL                                USB_TRB_CTRL_TRBCTL(1)
#define USB_TRBCTL_CONTROL_SETUP                         USB_TRB_CTRL_TRBCTL(2)
#define USB_TRBCTL_CONTROL_STATUS2                       USB_TRB_CTRL_TRBCTL(3)
#define USB_TRBCTL_CONTROL_STATUS3                       USB_TRB_CTRL_TRBCTL(4)
#define USB_TRBCTL_CONTROL_DATA                          USB_TRB_CTRL_TRBCTL(5)
#define USB_TRBCTL_ISOCHRONOUS_FIRST                     USB_TRB_CTRL_TRBCTL(6)
#define USB_TRBCTL_ISOCHRONOUS                           USB_TRB_CTRL_TRBCTL(7)
#define USB_TRBCTL_LINK_TRB                              USB_TRB_CTRL_TRBCTL(8)
#define USB_TRBCTL_NORMAL_ZLP                            USB_TRB_CTRL_TRBCTL(9)

/* TRB Length, PCM and Status */

#define USB_TRB_SIZE_MASK                                (0x00FFFFFF)
#define USB_TRB_SIZE_LENGTH(n)                           ((n) & USB_TRB_SIZE_MASK)
#define USB_TRB_PCM1_MSK                                 0x3
#define USB_TRB_PCM1_POS                                 24
#define USB_TRB_PCM1(n)                                  (((n) & USB_TRB_PCM1_MSK) \
								<< USB_TRB_PCM1_POS)
#define USB_TRB_SIZE_TRBSTS_MSK                          0xF0000000
#define USB_TRB_SIZE_TRBSTS_POS                          28
#define USB_TRB_SIZE_TRBSTS(n)                           (((n) & USB_TRB_SIZE_TRBSTS_MSK) \
								>> USB_TRB_SIZE_TRBSTS_POS)

#define USB_TRBSTS_OK                                    0
#define USB_TRBSTS_MISSED_ISOC                           1
#define USB_TRBSTS_SETUP_PENDING                         2
#define USB_TRB_STS_XFER_IN_PROG                         4


/* Device specific commmands, DEPCMD register is used */
#define USB_DEPCMD_DEPSTARTCFG                           (0x09 << 0)
#define USB_DEPCMD_ENDTRANSFER                           (0x08 << 0)
#define USB_DEPCMD_UPDATETRANSFER                        (0x07 << 0)
#define USB_DEPCMD_STARTTRANSFER                         (0x06 << 0)
#define USB_DEPCMD_CLEARSTALL                            (0x05 << 0)
#define USB_DEPCMD_SETSTALL                              (0x04 << 0)
#define USB_DEPCMD_GETEPSTATE                            (0x03 << 0)
#define USB_DEPCMD_SETTRANSFRESOURCE                     (0x02 << 0)
#define USB_DEPCMD_SETEPCONFIG                           (0x01 << 0)

#define USB_DEPCMD_CLEARPENDIN                           BIT(11)
#define USB_DEPCMD_CMDACT                                BIT(10)
#define USB_DEPCMD_CMDIOC                                BIT(8)

/* Define DWC3 USB device control endpoint ep0-out command configuration config values */
#define USB_DEPCMD_EP0_OUT_PAR1                          0x00000500
#define USB_DEPCMD_EP0_OUT_PAR0                          0x00000200

/* Define DWC3 USB device control endpoint ep1-in command configuration config values */

#define USB_DEPCMD_EP1_IN_PAR1                           0x06000500
#define USB_DEPCMD_EP1_IN_PAR0                           0x00000200

/* Define DWC3 USB device endpoint command config for ep/0/1/2/3/in/out */
#define USB_DEPCMD_EP_XFERCFG_PAR0                       0x00000001
#define USB_DEPCMD_EP_XFERCFG                            0x00000402

#define USB_DEPCMD_EP0_XFERCFG_PAR1                      0x02041000
#define USB_DEPCMD_EP0_XFERCFG_PAR0                      0x00000000
#define USB_DEPCMD_XFERCFG_EP0                           0x00000506

/* Device Endpoint Command Register */
#define USB_DEPCMD_PARAM_MSK                             0x7F0000
#define USB_DEPCMD_PARAM_POS                             16
#define USB_DEPCMD_PARAM(x)                              ((x) << USB_DEPCMD_PARAM_POS)
#define USB_DEPCMD_GET_RSC_IDX(x)                        (((x) & USB_DEPCMD_PARAM_MSK) \
								>> USB_DEPCMD_PARAM_POS)
#define USB_DEPCMD_STATUS_MSK                            0xF000
#define USB_DEPCMD_STATUS_POS                            12
#define USB_DEPCMD_STATUS(x)                             (((x) & USB_DEPCMD_STATUS_MSK) \
								>> USB_DEPCMD_STATUS_POS)
#define USB_DEPCMD_HIPRI_FORCERM                         BIT(11)
#define USB_DEPCMD_FORCERM                               1

/* DEPCFG parameter 0 */
#define USB_DEPCFG_EP_TYPE_POS                           1
#define USB_DEPCFG_EP_TYPE(n)                            ((n) << USB_DEPCFG_EP_TYPE_POS)
#define USB_DEPCFG_MAX_PACKET_SIZE_POS                   3
#define USB_DEPCFG_MAX_PACKET_SIZE(n)                    ((n) << USB_DEPCFG_MAX_PACKET_SIZE_POS)
#define USB_DEPCFG_FIFO_NUMBER_MSK                       0x1F
#define USB_DEPCFG_FIFO_NUMBER_POS                       17
#define USB_DEPCFG_FIFO_NUMBER(n)                        (((n) & USB_DEPCFG_FIFO_NUMBER_MSK) \
								<< USB_DEPCFG_FIFO_NUMBER_POS)
#define USB_DEPCFG_BURST_SIZE_POS                        22
#define USB_DEPCFG_BURST_SIZE(n)                         ((n) << USB_DEPCFG_BURST_SIZE_POS)
#define USB_DEPCFG_DATA_SEQ_NUM_POS                      26
#define USB_DEPCFG_DATA_SEQ_NUM(n)                       ((n) << USB_DEPCFG_DATA_SEQ_NUM_POS)

/* bit 30 and 31 is Config action of endpoint
 * value 0 for Initialize endpoint state
 * value 1 for Restore endpoint state
 * value 2 for Modify endpoint state
 */
#define USB_DEPCFG_ACTION_POS                            30
#define USB_DEPCFG_ACTION_INIT                           (0 << USB_DEPCFG_ACTION_POS)
#define USB_DEPCFG_ACTION_RESTORE                        (1 << USB_DEPCFG_ACTION_POS)
#define USB_DEPCFG_ACTION_MODIFY                         (2 << USB_DEPCFG_ACTION_POS)

#define USB_DEPCFG_INT_NUM(n)                            ((n) << 0U)
#define USB_DEPCFG_XFER_COMPLETE_EN                      BIT(8)
#define USB_DEPCFG_XFER_IN_PROGRESS_EN                   BIT(9)
#define USB_DEPCFG_XFER_NOT_READY_EN                     BIT(10)
#define USB_DEPCFG_FIFO_ERROR_EN                         BIT(11)
#define USB_DEPCFG_STREAM_EVENT_EN                       BIT(13)
#define USB_DEPCFG_BINTERVAL_M1_MSK                      0xFF
#define USB_DEPCFG_BINTERVAL_M1_POS                      16
#define USB_DEPCFG_BINTERVAL_M1(n)                       (((n) & USB_DEPCFG_BINTERVAL_M1_MSK) \
								<< USB_DEPCFG_BINTERVAL_M1_POS)
#define USB_DEPCFG_STREAM_CAPABLE                        BIT(24)
#define USB_DEPCFG_EP_NUMBER_POS                         25
#define USB_DEPCFG_EP_NUMBER(n)                          ((n) << USB_DEPCFG_EP_NUMBER_POS)
#define USB_DEPCFG_BULK_BASED                            BIT(30)
#define USB_DEPCFG_FIFO_BASED                            BIT(31)


/* Define DWC3 USB device event configuration config values */

/* USB device Disconnect event   */
#define USB_DEV_DISSCONNEVTEN                            BIT(0)
/* USB device reset event   */
#define USB_DEV_USBRSTEVTEN                              BIT(1)
/* USB device connection done event */
#define USB_DEV_CONNECTDONEEVTEN                         BIT(2)
/* USB device link state change event  */
#define USB_DEV_EVENT_ULSTCNGEN                          BIT(3)

/* Dwc3 device status register      */
#define USB_DSTS_USBLNKST_POS                            18
#define USB_DSTS_USBLNKST_MASK                           (0x0F << USB_DSTS_USBLNKST_POS)
#define USB_DSTS_USBLNKST(n)                             ((n & USB_DSTS_USBLNKST_MASK) \
								 >> USB_DSTS_USBLNKST_POS)
#define USB_DSTS_DEVCTRLHLT                              BIT(22)
#define USB_DSTS_DCNRD                                   BIT(29)

#define USB_DSTS_CONNECTSPD                              (7 << 0)
#define USB_DSTS_HIGHSPEED                               (0 << 0)

/* Device control register */
#define USB_DCTL_CSFTRST                                 BIT(30)
#define USB_DCTL_START                                   BIT(31)
#define USB_DCTL_KEEP_CONNECT                            BIT(19)
#define USB_DCTL_INITU2ENA                               BIT(12)
#define USB_DCTL_ACCEPTU2ENA                             BIT(11)
#define USB_DCTL_INITU1ENA                               BIT(10)
#define USB_DCTL_ACCEPTU1ENA                             BIT(9)
#define USB_DCTL_TSTCTRL_POS                             1
#define USB_DCTL_TSTCTRL_MASK                            (0xF << USB_DCTL_TSTCTRL_POS)
#define USB_DCTL_ULSTCHNGREQ_POS                         5
#define USB_DCTL_ULSTCHNGREQ_MASK                        0x1E0
#define USB_DCTL_ULSTCHNGREQ(n)                          (((n) << USB_DCTL_ULSTCHNGREQ_POS) \
								& USB_DCTL_ULSTCHNGREQ_MASK)
#define USB_DCTL_L1_HIBER_EN                             BIT(18)
#define USB_DCTL_HIRD_THRES_POS                          24
#define USB_DCTL_HIRD_THRES_MASK                         (0x1F << USB_DCTL_HIRD_THRES_POS)
#define USB_DCTL_HIRD_THRES(n)                           ((n) << USB_DCTL_HIRD_THRES_POS)

/* Global Event Size Registers */
#define USB_GEVNTSIZ_INTMASK                             BIT(31)
#define USB_GEVNTSIZ_SIZE_MSK                            0xFFFF
#define USB_GEVNTSIZ_SIZE(n)                             ((n) & USB_GEVNTSIZ_SIZE_MSK)
#define USB_GEVNTCOUNT_MASK                              0xFFFC

#define USB_EP_EVENT_TYPE                                0
#define USB_DEV_EVENT_TYPE                               0x1
#define USB_EVENT_TYPE_CHECK                             0x1

/* Device specific events */
#define USB_EVENT_DISCONNECT                             0
#define USB_EVENT_RESET                                  1
#define USB_EVENT_CONNECT_DONE                           2
#define USB_EVENT_LINK_STATUS_CHANGE                     3
#define USB_EVENT_WAKEUP                                 4
#define USB_EVENT_HIBER_REQ                              5
#define USB_EVENT_EOPF                                   6
#define USB_EVENT_SOF                                    7
#define USB_EVENT_ERRATIC_ERROR                          9
#define USB_EVENT_CMD_CMPL                               10
#define USB_EVENT_OVERFLOW                               11

/* dwc3 global control register bits   */
#define USB_GCTL_CORESOFTRESET                           BIT(11)
#define USB_GCTL_SOFITPSYNC                              BIT(10)
#define USB_GCTL_SCALEDOWN_POS                           4
#define USB_GCTL_SCALEDOWN(n)                            ((n) << USB_GCTL_SCALEDOWN_POS)
#define USB_GCTL_SCALEDOWN_MASK                          USB_GCTL_SCALEDOWN(3U)
#define USB_GCTL_DISSCRAMBLE                             BIT(3)
#define USB_GCTL_U2EXIT_LFPS                             BIT(2)
#define USB_GCTL_GBLHIBERNATIONEN                        BIT(1)
#define USB_GCTL_DSBLCLKGTNG                             BIT(0)
#define USB_GCTL_PRTCAP_MSK                              0x3000
#define USB_GCTL_PRTCAP_POS                              12
#define USB_GCTL_PRTCAP(n)                               (((n) & USB_GCTL_PRTCAP_MSK) \
									>> USB_GCTL_PRTCAP_POS)
#define USB_GCTL_PRTCAPDIR(n)                            ((n) << USB_GCTL_PRTCAP_POS)
#define USB_GCTL_PRTCAP_HOST                             1
#define USB_GCTL_PRTCAP_DEVICE                           2
#define USB_GCTL_PRTCAP_OTG                              3

/* Define DWC3 USB device controller global soc bus config values */

#define USB_GSBUSCFG0_INCR256BRSTENA                     BIT(7) /* INCR256 burst */
#define USB_GSBUSCFG0_INCR128BRSTENA                     BIT(6) /* INCR128 burst */
#define USB_GSBUSCFG0_INCR64BRSTENA                      BIT(5) /* INCR64 burst */
#define USB_GSBUSCFG0_INCR32BRSTENA                      BIT(4) /* INCR32 burst */
#define USB_GSBUSCFG0_INCR16BRSTENA                      BIT(3) /* INCR16 burst */
#define USB_GSBUSCFG0_INCR8BRSTENA                       BIT(2) /* INCR8 burst */
#define USB_GSBUSCFG0_INCR4BRSTENA                       BIT(1) /* INCR4 burst */
#define USB_GSBUSCFG0_INCRBRSTENA                        BIT(0) /* undefined length enable */
#define USB_GSBUSCFG0_INCRBRST_MASK                      0xFF

/* Global Frame Length Adjustment Register */
#define USB_GFLADJ_30MHZ_SDBND_SEL                       BIT(7)
#define USB_GFLADJ_30MHZ_MASK                            0x3F
#define USB_GFLADJ_DEFAULT_VALUE                         0x20
/* Global User Control Register */
#define USB_GUCTL_HSTINAUTORETRY                         BIT(14)

/* Global Synopsys ID Register  */
/* bit [31:16] for Core Identification Number */
#define USB_GSNPSID_MASK                                 0xFFFF0000
/* bit [15:0] for Core release number */
#define USB_GSNPSREV_MASK                                0xFFFF

typedef enum _USB_DEVICE_STATE {
	USB_DWC3_STATE_NOTATTACHED,
	USB_DWC3_STATE_ATTACHED,
	USB_DWC3_STATE_POWERED,
	USB_DWC3_STATE_RECONNECTING,
	USB_DWC3_STATE_UNAUTHENTICATED,
	USB_DWC3_STATE_DEFAULT,
	USB_DWC3_STATE_ADDRESS,
	USBD_DWC3_STATE_CONFIGURED,
	USB_DWC3_STATE_SUSPENDED
} USB_DEVICE_STATE;

typedef enum _USBD_EP0_STATE {
	EP0_UNCONNECTED,
	EP0_SETUP_PHASE,
	EP0_DATA_PHASE,
	EP0_STATUS_PHASE,
} USBD_EP0_STATE;

typedef enum _USB_DR_MODE {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
} USB_DR_MODE;

typedef enum _USB_PHY_INTERFACE {
	USB_PHY_INTERFACE_MODE_UNKNOWN,
	USB_PHY_INTERFACE_MODE_UTMI,
	USB_PHY_INTERFACE_MODE_UTMIW,
	USB_PHY_INTERFACE_MODE_ULPI,
	USB_PHY_INTERFACE_MODE_SERIAL,
	USB_PHY_INTERFACE_MODE_HSIC,
} USB_PHY_INTERFACE;
/*
 * Endpoint Parameters
 */
typedef struct _USBD_EP_PARAMS {
	uint32_t param2;
	uint32_t param1;
	uint32_t param0;
} USBD_EP_PARAMS;

/* TRB descriptor structure */
typedef struct _USBD_TRB {
	uint32_t buf_ptr_low;
	uint32_t buf_ptr_high;
	uint32_t size;
	uint32_t ctrl;
} USBD_TRB;

/* Define USB DWC3 physical endpoint structure.  */
typedef struct _USBD_EP {
	USBD_TRB  ep_trb[NO_OF_TRB_PER_EP + 1] ATTR_ALIGN(32);
	uint32_t  ep_status;
	uint8_t   ep_index;
	uint32_t  ep_transfer_status;
	uint8_t   ep_dir;
	uint32_t  ep_maxpacket;
	uint32_t  ep_resource_index;
	uint32_t  ep_requested_bytes;
	uint32_t  trb_enqueue;
	uint32_t  trb_dequeue;
	uint8_t   phy_ep;
	uint32_t  bytes_txed;
	uint32_t  unaligned_txed;
} USBD_EP;
/* USB setup packet structure */
typedef struct _USB_CTRL_REQUEST {
	uint8_t  bRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} USBD_CTRL_REQUEST;

/* Define USB Event structure definition. */
typedef struct _USBD_EVENT_BUFFER {
	void       *buf;
	uint32_t   length;
	uint32_t   lpos;
	uint32_t   count;
} USBD_EVENT_BUFFER;

/**
 * @brief USB_USB_ENDPNT_CMD [USB_ENDPNT_CMD] ([0..7])
 */
typedef struct {
	volatile uint32_t  DEPCMDPAR2;
	volatile uint32_t  DEPCMDPAR1;
	volatile uint32_t  DEPCMDPAR0;
	volatile uint32_t  DEPCMD;
} USB_USB_ENDPNT_CMD_Type;

/**
 * @brief USB (USB)
 */

typedef struct {     /*!< (@ 0x48200000) USB Structure */
	volatile const  uint32_t         CAPLENGTH;
	volatile const  uint32_t         HCSPARAMS1;
	volatile const  uint32_t         HCSPARAMS2;
	volatile const  uint32_t         HCSPARAMS3;
	volatile const  uint32_t         HCCPARAMS1;
	volatile const  uint32_t         DBOFF;
	volatile const  uint32_t         RTSOFF;
	volatile const  uint32_t         HCCPARAMS2;
	volatile const  uint32_t         RESERVED[12344];
	volatile uint32_t                GSBUSCFG0;
	volatile uint32_t                GSBUSCFG1;
	volatile const  uint32_t         RESERVED1[2];
	volatile uint32_t                GCTL;
	volatile const  uint32_t         RESERVED2;
	volatile uint32_t                GSTS;
	volatile uint32_t                GUCTL1;
	volatile const  uint32_t         GSNPSID;
	volatile const  uint32_t         RESERVED3;
	volatile uint32_t                GUID;
	volatile uint32_t                GUCTL;
	volatile const  uint32_t         GBUSERRADDRLO;
	volatile const  uint32_t         GBUSERRADDRHI;
	volatile const  uint32_t         RESERVED4[2];
	volatile const  uint32_t         GHWPARAMS0;
	volatile const  uint32_t         GHWPARAMS1;
	volatile const  uint32_t         GHWPARAMS2;
	volatile const  uint32_t         GHWPARAMS3;
	volatile const  uint32_t         GHWPARAMS4;
	volatile const  uint32_t         GHWPARAMS5;
	volatile const  uint32_t         GHWPARAMS6;
	volatile const  uint32_t         GHWPARAMS7;
	volatile const  uint32_t         RESERVED5[8];
	volatile uint32_t                GPRTBIMAP_HSLO;
	volatile uint32_t                GPRTBIMAP_HSHI;
	volatile uint32_t                GPRTBIMAP_FSLO;
	volatile uint32_t                GPRTBIMAP_FSHI;
	volatile const  uint32_t         RESERVED6[3];
	volatile uint32_t                GUCTL2;
	volatile const  uint32_t         RESERVED7[24];
	volatile uint32_t                GUSB2PHYCFG0;
	volatile const  uint32_t         RESERVED8[63];
	volatile uint32_t                GTXFIFOSIZ[4];
	volatile const  uint32_t         RESERVED9[28];
	volatile uint32_t                GRXFIFOSIZ[4];
	volatile const  uint32_t         RESERVED10[28];
	volatile uint32_t                GEVNTADRLO0;
	volatile uint32_t                GEVNTADRHI0;
	volatile uint32_t                GEVNTSIZ0;
	volatile uint32_t                GEVNTCOUNT0;
	volatile const  uint32_t         RESERVED11[124];
	volatile const  uint32_t         GHWPARAMS8;
	volatile const  uint32_t         RESERVED12[3];
	volatile uint32_t                GTXFIFOPRIDEV;
	volatile const  uint32_t         RESERVED13;
	volatile uint32_t                GTXFIFOPRIHST;
	volatile uint32_t                GRXFIFOPRIHST;
	volatile const  uint32_t         RESERVED14[4];
	volatile uint32_t                GFLADJ;
	volatile const  uint32_t         RESERVED15[3];
	volatile uint32_t                GUSB2RHBCTL0;
	volatile const  uint32_t         RESERVED16[47];
	volatile uint32_t                DCFG;
	volatile uint32_t                DCTL;
	volatile uint32_t                DEVTEN;
	volatile uint32_t                DSTS;
	volatile uint32_t                DGCMDPAR;
	volatile uint32_t                DGCMD;
	volatile const  uint32_t         RESERVED17[2];
	volatile uint32_t                DALEPENA;
	volatile const  uint32_t         RESERVED18[55];
	volatile USB_USB_ENDPNT_CMD_Type USB_ENDPNT_CMD[8];
	volatile const  uint32_t         RESERVED19[96];
	volatile uint32_t                DEV_IMOD0;
} USB_Type;  /*!< Size = 51716 (0xca04)  */

/**
 * hwparams - copy of HWPARAMS registers
 */
typedef struct _USB_HWPARAMS {
	uint32_t hwparams0;
	uint32_t hwparams1;
	uint32_t hwparams2;
	uint32_t hwparams3;
	uint32_t hwparams4;
	uint32_t hwparams5;
	uint32_t hwparams6;
	uint32_t hwparams7;
	uint32_t hwparams8;
} USB_HWPARAMS;

typedef struct _USB_DRIVER {
	USB_Type           *regs;
	void               (*usbd_dwc3_device_reset_cb)(struct _USB_DRIVER *drv);
	void               (*usbd_dwc3_connect_cb)(struct _USB_DRIVER *drv);
	void               (*usbd_dwc3_disconnect_cb)(struct _USB_DRIVER *drv);
	void               (*usbd_dwc3_setupstage_cb)(struct _USB_DRIVER *drv);
	void               (*usbd_dwc3_data_in_cb)(struct _USB_DRIVER *drv, uint8_t ep_num);
	void               (*usbd_dwc3_data_out_cb)(struct _USB_DRIVER *drv, uint8_t ep_num);

	USBD_CTRL_REQUEST   setup_data ATTR_ALIGN(32);
	USBD_TRB            ep0_trb ATTR_ALIGN(32);
	USBD_EP             eps[USB_NUM_OF_EPS];
	USBD_EVENT_BUFFER   *event_buf;
	USBD_EP0_STATE      ep0_state;
	USB_DEVICE_STATE    config_state;
	USB_HWPARAMS        hwparams;
	uint32_t            in_eps;
	uint32_t            num_bytes;
	uint32_t            out_eps;
	uint32_t            usb2_phy_config;
	bool                setup_packet_pending:1;
	bool                three_stage_setup:1;
	bool                ep0_expect_in:1;
	uint32_t            revision;
	uint8_t             current_dr_role;
	uint8_t             endp_number;
	uint8_t             dr_mode;
	uint8_t             hsphy_mode;
	uint8_t             fladj;
	uint32_t            event_type;
	uint32_t            actual_length;
} USB_DRIVER;

void dwc3_reset_cb(USB_DRIVER *Drv);
void dwc3_setupstage_cb(USB_DRIVER *Drv);
void dwc3_disconnect_cb(USB_DRIVER *Drv);
void dwc3_connect_cb(USB_DRIVER *Drv);
void dwc3_data_in_cb(USB_DRIVER *Drv, uint8_t ep_num);
void dwc3_data_out_cb(USB_DRIVER *Drv, uint8_t ep_num);
void  usbd_dwc3_usb2phy_config_reset(USB_DRIVER *drv);
void  usbd_dwc3_usb2phy_config_check(USB_DRIVER *drv);
uint32_t usbd_get_ep_transfer_resource_index(USB_DRIVER *drv, uint8_t ep_num, uint8_t dir);
void usbd_prepare_setup(USB_DRIVER *drv);

#ifdef __cplusplus
}
#endif
#endif
