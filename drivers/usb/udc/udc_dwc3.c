/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2025 Alif Semiconductor.
 */

#define DT_DRV_COMPAT snps_dwc3

#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/device.h>

#include "udc_common.h"
#if CONFIG_UDC_DWC3_ALIF
#include "soc_memory_map.h"
#endif
#include "udc_dwc3.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);

struct udc_dwc3_data {
	udc_dwc3_driver_t  drv;
	const struct device *dev;
	uint32_t irq;
	struct k_thread thread_data;
	struct k_msgq dwc3_msgq_data;
	enum udc_bus_speed speed;
};

struct udc_dwc3_config {
	size_t num_in_eps;
	size_t num_out_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	udc_dwc3_reg_t *const base;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	void (*make_thread)(const struct device *dev);
	uint8_t max_speed;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	clock_control_subsys_t clock_subsys2;
};

struct udc_dwc3_msg {
	uint8_t type;
	uint8_t ep;
	uint16_t recv_bytes;
	struct usb_setup_packet setup;
};

enum udc_dwc3_msg_type {
	UDC_DWC3_MSG_SETUP,
	UDC_DWC3_MSG_DATA_OUT,
	UDC_DWC3_MSG_DATA_IN,
};

#define USB_ENDPOINT_NUMBER_MASK      0xF
#define EP_NUM(ep_addr)               (ep_addr & USB_ENDPOINT_NUMBER_MASK)
#define DWC3DATA(drv) CONTAINER_OF(drv, struct udc_dwc3_data, drv)

static udc_dwc3_event_buffer_t udc_dwc3_evt_buf;
static uint8_t udc_dwc3_event_buff[USB_EVENT_BUFFER_SIZE];
static char udc_dwc3_msgq_buf[CONFIG_UDC_DWC3_MAX_QMESSAGES * sizeof(struct udc_dwc3_msg)];

static uint32_t udc_dwc3_get_ep_transfer_resource_index(udc_dwc3_driver_t *drv,
		uint8_t ep_num, uint8_t dir)
{
	uint8_t phy_ep;
	uint32_t resource_index;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	/* [22:16]: Transfer Resource Index (XferRscIdx). The hardware-assigned
	 * transfer resource index for the transfer
	 */
	resource_index = drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMD;

	return USB_DEPCMD_GET_RSC_IDX(resource_index);
}

/* save the current phy state and disable the lpm and suspend  */
static void udc_dwc3_disable_phy_suspend(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	drv->usb2_phy_config = 0;
	reg = drv->regs->GUSB2PHYCFG0;
	if (reg & USB_GUSB2PHYCFG_SUSPHY) {
		SET_BIT(drv->usb2_phy_config, USB_GUSB2PHYCFG_SUSPHY);
		CLEAR_BIT(reg, USB_GUSB2PHYCFG_SUSPHY);
	}
	if (reg & USB_GUSB2PHYCFG_ENBLSLPM) {
		SET_BIT(drv->usb2_phy_config, USB_GUSB2PHYCFG_ENBLSLPM);
		CLEAR_BIT(reg, USB_GUSB2PHYCFG_ENBLSLPM);
	}
	if (drv->usb2_phy_config) {
		drv->regs->GUSB2PHYCFG0 = reg;
	}
}

/* Restore the phy state */
static void udc_dwc3_restore_suspend_state(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	if (drv->usb2_phy_config) {
		reg = drv->regs->GUSB2PHYCFG0;
		reg |= drv->usb2_phy_config;
		drv->regs->GUSB2PHYCFG0 = reg;
	}
}

static int32_t udc_dwc3_send_ep_cmd(udc_dwc3_driver_t *drv, uint8_t phy_ep, uint32_t cmd,
		udc_dwc3_ep_params_t params)
{
	int32_t ret;
	int32_t      cmd_status = USB_EP_CMD_CMPLT_ERROR;
	int32_t      timeout = USB_DEPCMD_TIMEOUT;

	/* Check usb2phy config before issuing DEPCMD  */
	udc_dwc3_disable_phy_suspend(drv);
	if (USB_DEPCMD_CMD(cmd) == USB_DEPCMD_UPDATETRANSFER) {
		CLEAR_BIT(cmd, USB_DEPCMD_CMDIOC | USB_DEPCMD_CMDACT);
	} else {
		SET_BIT(cmd, USB_DEPCMD_CMDACT);
	}
	if (USB_DEPCMD_CMD(cmd) == USB_DEPCMD_STARTTRANSFER) {
#if CONFIG_UDC_DWC3_ALIF
		drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMDPAR1 =
				(uint32_t)local_to_global((void *)(params.param1));
#else
		drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMDPAR1 = params.param1;
#endif
	} else {
		drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMDPAR1 = params.param1;
	}
	/* Issuing DEPCFG Command for appropriate endpoint */
	drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMDPAR0 = params.param0;
	drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMD = cmd;
	do {
		/* Read the device endpoint Command register.  */
		ret = drv->regs->USB_ENDPNT_CMD[phy_ep].DEPCMD;
		if (!(ret & USB_DEPCMD_CMDACT)) {
			cmd_status = USB_DEPCMD_STATUS(ret);
			switch (cmd_status) {
			case USB_DEPEVT_CMD_SUCCESS:
				ret = cmd_status;
				break;
			case USB_DEPEVT_TRANSFER_NO_RESOURCE:
				ret = USB_EP_CMD_CMPLT_NO_RESOURCE_ERROR;
				break;
			case USB_DEPEVT_TRANSFER_BUS_EXPIRY:
				ret = USB_EP_CMD_CMPLT_BUS_EXPIRY_ERROR;
				break;
			default:
				LOG_ERR("Unknown cmd status");
				ret = USB_EP_CMD_CMPLT_STATUS_UNKNOWN;
				break;
			}
			break;
		}
	} while (--timeout);

	if (timeout == 0) {
		ret = USB_EP_CMD_CMPLT_TIMEOUT_ERROR;
		LOG_ERR("timeout for command completion\n");
	}
	/* Restore the USB2 phy state  */
	udc_dwc3_restore_suspend_state(drv);

	return ret;
}

static void udc_dwc3_prepare_setup(udc_dwc3_driver_t *drv)
{
	udc_dwc3_ep_params_t params = {0};
	udc_dwc3_trb_t      *trb_ptr;
	udc_dwc3_ep_t       *ept;
	uint32_t ret;

	/* Setup packet always on EP0 */
	ept = &drv->eps[USB_CONTROL_EP];
	trb_ptr = &drv->ep0_trb;
	sys_cache_data_flush_range(&drv->setup_data, USB_SETUP_PKT_SIZE);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low =  LOWER_32_BITS(local_to_global(&drv->setup_data));
#else
	trb_ptr->buf_ptr_low =  LOWER_32_BITS(&drv->setup_data);
#endif
	trb_ptr->buf_ptr_high = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(USB_SETUP_PKT_SIZE);
	trb_ptr->ctrl = USB_TRBCTL_CONTROL_SETUP;
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_IOC
			| USB_TRB_CTRL_ISP_IMI);

	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	drv->ep0_state = EP0_SETUP_PHASE;
	/* Issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, 0U, USB_DEPCMD_STARTTRANSFER, params);
	if (ret) {
		LOG_ERR("Failed in send the command for setup pkt and status: %d", ret);
	}
	SET_BIT(ept->ep_status, USB_EP_BUSY);
	ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
			ept->ep_index,
			ept->ep_dir);
}

static int32_t udc_dwc3_ep0_send(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t *bufferptr, uint32_t buf_len)
{
	udc_dwc3_ep_params_t params = {0};
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t *trb_ptr;
	int32_t ret;
	uint8_t phy_ep;

	if (buf_len == 0) {
		return USB_EP_BUFF_LENGTH_INVALID;
	}
	/* Control IN - EP1 */
	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];

	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		LOG_ERR("Endpoint 1 already busy returning");
		return USB_EP_BUSY_ERROR;
	}

	ept->ep_requested_bytes = buf_len;
	ept->bytes_txed = 0U;
	trb_ptr = &drv->ep0_trb;
	sys_cache_data_flush_range(bufferptr, buf_len);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low  =  LOWER_32_BITS(local_to_global((uint32_t *)bufferptr));
#else
	trb_ptr->buf_ptr_low  =  LOWER_32_BITS((uint32_t *)bufferptr);
#endif
	trb_ptr->buf_ptr_high  = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(buf_len);
	trb_ptr->ctrl = USB_TRBCTL_CONTROL_DATA;
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_ISP_IMI
			| USB_TRB_CTRL_IOC);

	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	drv->ep0_state = EP0_DATA_PHASE;
	/* Issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_STARTTRANSFER, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("Failed in sending data over EP1");
		return ret;
	}
	SET_BIT(ept->ep_status, USB_EP_BUSY);
	ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
			ept->ep_index,
			ept->ep_dir);

	return ret;
}

static int32_t udc_dwc3_ep0_recv(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t *bufferptr, uint32_t buf_len)
{
	udc_dwc3_ep_params_t params = {0};
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t *trb_ptr;
	int32_t ret;
	uint8_t phy_ep;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		LOG_ERR("Endpoint 0 already busy returning");
		return USB_EP_BUSY_ERROR;
	}
	if (buf_len < USB_CONTROL_EP_MAX_PKT) {
		buf_len = USB_CONTROL_EP_MAX_PKT;
	}
	ept->ep_requested_bytes = buf_len;
	ept->bytes_txed = 0U;
	trb_ptr = &drv->ep0_trb;
	sys_cache_data_flush_range(bufferptr, buf_len);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low  =  LOWER_32_BITS(local_to_global((uint32_t *)bufferptr));
#else
	trb_ptr->buf_ptr_low  =  LOWER_32_BITS((uint32_t *)bufferptr);
#endif
	trb_ptr->buf_ptr_high  = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(buf_len);
	trb_ptr->ctrl = USB_TRBCTL_CONTROL_DATA;
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_ISP_IMI
		| USB_TRB_CTRL_IOC);

	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	drv->ep0_state = EP0_DATA_PHASE;
	/* Issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_STARTTRANSFER, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("Failed to send command over EP0");
		return ret;
	}
	SET_BIT(ept->ep_status, USB_EP_BUSY);
	/* In response to the Start Transfer command, the hardware assigns
	 * transfer a resource index number (XferRscIdx) and returns index in the DEPCMDn register
	 * and in the Command Complete event.
	 * This index must be used in subsequent Update and End Transfer commands.
	 */
	ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
			ept->ep_index,
			ept->ep_dir);

	return ret;
}

static void usbd_isoc_micro_frame_update(udc_dwc3_driver_t *drv, uint32_t current_fn)
{
	uint32_t last_frame_num = drv->micro_frame_number & 0x3FFF;

	/* Detect rollover in the 14-bit micro-frame counter.
	 * Even though events provide 16 bits, DWC3 SOFFN is 14 bits in HS.
	 */
	if ((current_fn & 0x3FFF) < last_frame_num) {
		drv->micro_frame_number += BIT(14);
	}

	/* Keep the upper bits and update the lower 14 bits */
	drv->micro_frame_number = (drv->micro_frame_number & ~0x3FFF) | (current_fn & 0x3FFF);
}

static int32_t udc_dwc3_stop_transfer(udc_dwc3_driver_t *drv, uint8_t ep_num,
		uint8_t dir, uint8_t force_rm)
{
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t      *trb_ptr;
	udc_dwc3_ep_params_t params = {0};
	uint8_t phy_ep;
	uint32_t cmd;
	uint32_t ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	if (ept->trb_enqueue == 0U) {
		ept->trb_enqueue = NO_OF_TRB_PER_EP - 1;
	} else {
		ept->trb_enqueue--;
	}
	/* check the endpoint stall condition */
	if (ept->ep_status & USB_EP_STALL) {
		ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
		if (ret < 0) {
			LOG_ERR("Failed to send command CLEARSTALL");
			return ret;
		}
		CLEAR_BIT(ept->ep_status, USB_EP_STALL);
	}
	if (ept->ep_resource_index == 0U) {
		return USB_EP_RESOURCE_INDEX_INVALID;
	}
	/* Data book says for end transfer  HW needs some
	 * extra time to synchronize with the interconnect
	 * - Issue EndTransfer WITH CMDIOC bit set
	 * - Wait 100us
	 */
	cmd = USB_DEPCMD_ENDTRANSFER;
	cmd |= (force_rm == 1) ? USB_DEPCMD_HIPRI_FORCERM : 0U;
	SET_BIT(cmd, USB_DEPCMD_CMDIOC);
	cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);

	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
	if (ret < 0) {
		LOG_ERR("Failed to send command at END transfer");
		return ret;
	}
	trb_ptr = &ept->ep_trb[ept->trb_enqueue];
	if (trb_ptr->ctrl) {
		trb_ptr->ctrl = 0;
	}
	ept->trb_enqueue = 0;
	if (force_rm == 1) {
		ept->ep_resource_index = 0U;
	}
	CLEAR_BIT(ept->ep_status, USB_EP_BUSY);
	k_busy_wait(100);

	return ret;
}

static int32_t udc_dwc3_isoc_send(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t *bufferptr, uint32_t buf_len)
{
	udc_dwc3_ep_t        *ept;
	udc_dwc3_trb_t       *trb_ptr;
	udc_dwc3_ep_params_t params = {0};
	uint8_t              phy_ep;
	int32_t              ret;
	uint32_t             cmd;
	uint32_t             mult;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	if (ept->ep_dir != USB_DIR_IN) {
		LOG_ERR("Direction is wrong returning");
		return USB_EP_DIRECTION_WRONG;
	}

	ept->bytes_txed = 0U;
	ept->ep_requested_bytes = buf_len;
	trb_ptr = &ept->ep_trb[ept->trb_enqueue];
	ept->trb_enqueue++;
	if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
		ept->trb_enqueue = 0U;
	}
	sys_cache_data_flush_range(bufferptr, buf_len);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low  = LOWER_32_BITS(local_to_global((uint32_t *)bufferptr));
#else
	trb_ptr->buf_ptr_low  = LOWER_32_BITS((uint32_t *)bufferptr);
#endif
	trb_ptr->buf_ptr_high = 0;
	trb_ptr->size         = USB_TRB_SIZE_LENGTH(buf_len);
	trb_ptr->ctrl         = USB_TRBCTL_ISOCHRONOUS_FIRST;
	/* For High-Speed, High-Bandwidth IN endpoints, a maximum of three
	 * packets can be sent during an interval. The PktCntM1 field
	 * ([25:24] of the third DWORD) must be set to (number of packets
	 * in the Buffer Descriptor - 1).
	 *
	 * Example: If there is only a single transaction in the microframe,
	 * only a DATA0 data packet PID is used. If there are two transactions
	 * per microframe, DATA1 is used for the first transaction data packet
	 * and DATA0 is used for the second. If there are three transactions
	 * per microframe, DATA2 is used for the first, DATA1 for the second,
	 * and DATA0 for the third.
	 */
	/* If there are three transactions per microframe mult should be 2 */

	/* 1) length <= maxpacket
	 *  - DATA0
	 *
	 * 2) maxpacket < length <= (2 * maxpacket)
	 *  - DATA1, DATA0
	 *
	 * 3) (2 * maxpacket) < length <= (3 * maxpacket)
	 *  - DATA2, DATA1, DATA0
	 */
	mult = USB_ISOC_MAX_MULT_VALUE;
	if (buf_len <= (2 * (ept->ep_maxpacket))) {
		mult--;
	}
	if (buf_len <= ept->ep_maxpacket) {
		mult--;
	}
	trb_ptr->size |= USB_TRB_PCM1(mult);
	/* For Isochronous always enables the Interrupt on Missed ISOC  */
	/* Sync microframe number from DSTS to ensure drv->micro_frame_number is current */
	usbd_isoc_micro_frame_update(drv, USB_DSTS_SOFFN(drv->regs->DSTS));

	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		cmd  = USB_DEPCMD_UPDATETRANSFER;
		cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);

		SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI);
		sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
		params.param1 = (uint32_t) trb_ptr;

		ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
	} else {
		uint32_t future_frame;
		int retries = 3;

		trb_ptr->ctrl = USB_TRBCTL_ISOCHRONOUS_FIRST;
		/* The future microframe time must be an integral multiple of
		 * intervals after the current time and aligned to the beginning
		 * of an interval. Add a small margin (e.g. 2 microframes) to
		 * avoid Bus Expiry.
		 */
		future_frame = drv->micro_frame_number + 2;
		if (ept->interval_uframe > 1) {
			future_frame = ROUND_UP(future_frame, ept->interval_uframe);
		}

		SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI);
		sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
		params.param1 = (uint32_t) trb_ptr;

		do {
			cmd  = USB_DEPCMD_STARTTRANSFER;
			cmd |= USB_DEPCMD_PARAM(future_frame);

			ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
			if (ret != USB_EP_CMD_CMPLT_BUS_EXPIRY_ERROR) {
				break;
			}

			/* Bus Expiry: the requested frame has already passed.
			 * Push it further into the future.
			 */
			LOG_WRN("ISOC STARTTRANSFER Bus Expiry, retrying with future frame");
			future_frame += (ept->interval_uframe > 0) ? ept->interval_uframe : 1;
			retries--;
		} while (retries > 0);
	}

	if (ret < USB_SUCCESS) {
		LOG_ERR("failed to send the command for isoc send");
		/* Revert TRB enqueue since we completely failed to submit it */
		if (ept->trb_enqueue == 0U) {
			ept->trb_enqueue = NO_OF_TRB_PER_EP - 1;
		} else {
			ept->trb_enqueue--;
		}
		return ret;
	}

	if ((ept->ep_status & USB_EP_BUSY) == 0U) {
		ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
				ept->ep_index, ept->ep_dir);

		SET_BIT(ept->ep_status, USB_EP_BUSY);
	}

	return ret;
}
static int32_t udc_dwc3_bulk_int_send(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t *bufferptr, uint32_t buf_len)
{
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t      *trb_ptr;
	udc_dwc3_ep_params_t params = {0};
	uint8_t      phy_ep;
	int32_t     ret;
	uint32_t      cmd;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	if (ept->ep_dir != USB_DIR_IN) {
		LOG_ERR("Direction is wrong returning");
		return USB_EP_DIRECTION_WRONG;
	}
	ept->bytes_txed = 0U;
	ept->ep_requested_bytes = buf_len;
	trb_ptr = &ept->ep_trb[ept->trb_enqueue];
	ept->trb_enqueue++;
	if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
		ept->trb_enqueue = 0U;
	}
	sys_cache_data_flush_range(bufferptr, buf_len);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low  = LOWER_32_BITS(local_to_global((uint32_t *)bufferptr));
#else
	trb_ptr->buf_ptr_low  = LOWER_32_BITS((uint32_t *)bufferptr);
#endif
	trb_ptr->buf_ptr_high  = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(buf_len);
	if (buf_len == 0 && ept->ep_type == USB_BULK_EP) {
	/* Normal ZLP(BULK IN) - set to 9 for BULK IN TRB for zero
	 * length packet termination
	 */
		trb_ptr->ctrl = USB_TRBCTL_NORMAL_ZLP;
	} else {
		/* For Bulk TRB control(TRBCTL) as Normal  */
		trb_ptr->ctrl = USB_TRBCTL_NORMAL;
	}
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_IOC);
	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		cmd = USB_DEPCMD_UPDATETRANSFER;
		cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
	} else {
		cmd = USB_DEPCMD_STARTTRANSFER;
	}
	/* issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("failed to send the command for bulk_int send");
		return ret;
	}
	if ((ept->ep_status & USB_EP_BUSY) == 0U) {
		ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
				ept->ep_index, ept->ep_dir);
		SET_BIT(ept->ep_status, USB_EP_BUSY);
	}

	return ret;
}

static int32_t udc_dwc3_bulk_int_recv(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t *bufferptr, uint32_t buf_len)
{
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t   *trb_ptr;
	udc_dwc3_ep_params_t params = {0};
	uint8_t      phy_ep;
	uint32_t size;
	uint32_t cmd;
	int32_t     ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	if (ept->ep_dir != dir) {
		LOG_ERR("Wrong BULK/INT endpoint direction");
		return USB_EP_DIRECTION_WRONG;
	}
	ept->ep_requested_bytes = buf_len;
	ept->bytes_txed         = 0U;
	/*
	 * An OUT transfer size (Total TRB buffer allocation)
	 * must be a multiple of MaxPacketSize even if software is expecting a
	 * fixed non-multiple of MaxPacketSize transfer from the Host.
	 */
	if (!IS_ALIGNED(buf_len, ept->ep_maxpacket)) {
		size                = ROUND_UP(buf_len, ept->ep_maxpacket);
		ept->unaligned_txed = 1U;
	} else {
		size                = buf_len;
		ept->unaligned_txed = 0U;
	}
	trb_ptr = &ept->ep_trb[ept->trb_enqueue];

	ept->trb_enqueue++;
	if (ept->trb_enqueue == NO_OF_TRB_PER_EP) {
		ept->trb_enqueue = 0U;
	}
	sys_cache_data_flush_range(bufferptr, buf_len);
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low  = LOWER_32_BITS(local_to_global((uint32_t *)bufferptr));
#else
	trb_ptr->buf_ptr_low  = LOWER_32_BITS((uint32_t *)bufferptr);
#endif
	trb_ptr->buf_ptr_high  = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(size);
	trb_ptr->ctrl = USB_TRBCTL_NORMAL;
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_CSP | USB_TRB_CTRL_IOC | USB_TRB_CTRL_ISP_IMI
			| USB_TRB_CTRL_HWO);

	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		cmd = USB_DEPCMD_UPDATETRANSFER;
		cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
	} else {
		cmd = USB_DEPCMD_STARTTRANSFER;
	}
	/* Issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("Failed to send the command for bulk_int recv");
		return ret;
	}
	if ((ept->ep_status & USB_EP_BUSY) == 0U) {
		ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
				ept->ep_index,
				ept->ep_dir);
		SET_BIT(ept->ep_status, USB_EP_BUSY);
	}

	return ret;
}

static void udc_dwc3_clear_stall_all_eps(udc_dwc3_driver_t *drv)
{
	uint8_t phy_ep;
	int32_t     ret;
	udc_dwc3_ep_t *ept;
	udc_dwc3_ep_params_t params = {0};

	for (phy_ep = 1U; phy_ep < (drv->out_eps + drv->in_eps); phy_ep++) {
		ept = &drv->eps[phy_ep];
		 /* Skip if endpoint is not enabled */
		if ((ept->ep_status & USB_EP_ENABLED) == 0U) {
			continue;
		}
		/* Skip if endpoint is not stalled */
		if ((ept->ep_status & USB_EP_STALL) == 0U) {
			continue;
		}
		/* Issue the command to the hardware */
		ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
		if (ret < 0) {
			LOG_ERR("Failed to Clear STALL ep");
			return;
		}
		CLEAR_BIT(ept->ep_status, USB_EP_STALL);
	}
}

static void udc_dwc3_ep_xfer_complete(udc_dwc3_driver_t *drv, uint8_t endp_number)
{
	udc_dwc3_trb_t *trb_ptr;
	udc_dwc3_ep_t  *ept;
	uint32_t       remaining_len;
	uint8_t        dir;
	uint32_t       trb_status;

	ept = &drv->eps[endp_number];
	dir = ept->ep_dir;
	trb_ptr = &ept->ep_trb[ept->trb_dequeue];
	sys_cache_data_invd_range(trb_ptr, sizeof(*trb_ptr));
	trb_status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);

	ept->trb_dequeue++;
	if (ept->trb_dequeue == NO_OF_TRB_PER_EP) {
		ept->trb_dequeue = 0U;
	}

	if (trb_status == USB_TRBSTS_SETUP_PENDING) {
		drv->setup_packet_pending = true;
		LOG_ERR("TRB transmission was pending during the non control ep DATA");
		return;
	}
	if (trb_status == USB_TRBSTS_MISSED_ISOC) {
		LOG_ERR("Missed Isochronous transfer");
		ept->bytes_txed = 0;
	} else {
		/* remaining_len = remaining bytes not transferred */
		remaining_len = trb_ptr->size & USB_TRB_SIZE_MASK;
		if (dir == USB_DIR_IN) {
			/* For IN: bytes_txed = requested - remaining */
			ept->bytes_txed = ept->ep_requested_bytes - remaining_len;
		} else {
			uint32_t trb_size;
			/* For OUT: Calculate based on actual TRB buffer size */
			if (ept->unaligned_txed == 1U) {
				trb_size = ROUND_UP(ept->ep_requested_bytes,
						ept->ep_maxpacket);
				ept->unaligned_txed = 0U;
			} else {
				trb_size = ept->ep_requested_bytes;
			}
			ept->bytes_txed = trb_size - remaining_len;
		}
	}
	drv->num_bytes =  ept->bytes_txed;
	if ((drv->udc_dwc3_data_in_cb != NULL) && (dir == USB_DIR_IN)) {
		drv->udc_dwc3_data_in_cb(drv, ept->ep_index | USB_REQUEST_IN);
	} else {
		if (drv->udc_dwc3_data_out_cb != NULL) {
			drv->udc_dwc3_data_out_cb(drv, ept->ep_index);
		}
	}
}

static int32_t udc_dwc3_ep0_send_recv_status(udc_dwc3_driver_t *drv, uint8_t endp_number)
{
	udc_dwc3_ep_t       *ept;
	udc_dwc3_trb_t      *trb_ptr;
	udc_dwc3_ep_params_t params = {0};
	uint32_t type;
	uint32_t ret;
	uint8_t dir;

	ept = &drv->eps[endp_number];
	if ((ept->ep_status & USB_EP_BUSY) != 0U) {
		LOG_ERR("Ep is busy");
		return USB_EP_BUSY_ERROR;
	}
	type = (drv->three_stage_setup != false) ?
			USB_TRBCTL_CONTROL_STATUS3
			: USB_TRBCTL_CONTROL_STATUS2;
	trb_ptr = &drv->ep0_trb;
	sys_cache_data_flush_range(&drv->setup_data, USB_SETUP_PKT_SIZE);
	/* we use same trb_ptr for status */
#if CONFIG_UDC_DWC3_ALIF
	trb_ptr->buf_ptr_low = LOWER_32_BITS(local_to_global(&drv->setup_data));
#else
	trb_ptr->buf_ptr_low = LOWER_32_BITS(&drv->setup_data);
#endif
	trb_ptr->buf_ptr_high = 0;
	trb_ptr->size = USB_TRB_SIZE_LENGTH(0U);
	trb_ptr->ctrl = type;
	SET_BIT(trb_ptr->ctrl, USB_TRB_CTRL_HWO | USB_TRB_CTRL_LST | USB_TRB_CTRL_IOC
			| USB_TRB_CTRL_ISP_IMI);

	sys_cache_data_flush_range(trb_ptr, sizeof(*trb_ptr));
	params.param1 = (uint32_t)trb_ptr;
	drv->ep0_state = EP0_STATUS_PHASE;
	/*
	 * Control OUT transfer - Status stage happens on EP0 IN - EP1
	 * Control IN transfer - Status stage happens on EP0 OUT - EP0
	 */
	dir = !drv->ep0_expect_in;
	ret = udc_dwc3_send_ep_cmd(drv, 0U|dir, USB_DEPCMD_STARTTRANSFER, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("failed to execute the command at control status");
		return ret;
	}
	SET_BIT(ept->ep_status, USB_EP_BUSY);
	ept->ep_resource_index = udc_dwc3_get_ep_transfer_resource_index(drv,
			ept->ep_index,
			ept->ep_dir);

	return ret;
}

static int32_t udc_dwc3_ep0_stall_restart(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir)
{
	uint8_t        phy_ep;
	udc_dwc3_ep_t       *ept    = NULL;
	udc_dwc3_ep_params_t params = {0};
	uint32_t       ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept    = &drv->eps[phy_ep];
	/* Issue the command to the hardware */
	ret    = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETSTALL, params);
	if (ret < 0) {
		LOG_ERR("failed to send STALL command");
		return ret;
	}
	ept->ep_status |= USB_EP_STALL;
	/* when we issued stall on EP0 OUT after immediately
	 * software has to schedule the setup TRB for next setup packet.
	 */
	udc_dwc3_prepare_setup(drv);
	return ret;
}

static void udc_dwc3_ep0_end_control_data(udc_dwc3_driver_t *drv, udc_dwc3_ep_t *ept)
{
	udc_dwc3_ep_params_t params = {0};
	uint32_t     cmd;
	uint32_t ret;
	uint8_t phy_ep;

	if (ept->ep_resource_index == 0U) {
		return;
	}
	phy_ep = USB_GET_PHYSICAL_EP(ept->ep_index, ept->ep_dir);
	cmd = USB_DEPCMD_ENDTRANSFER;
	SET_BIT(cmd, USB_DEPCMD_CMDIOC);
	cmd |= USB_DEPCMD_PARAM(ept->ep_resource_index);
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, cmd, params);
	if (ret < 0) {
		LOG_ERR("Failed to send command at Endcontrol data");
		return;
	}
	ept->ep_resource_index = 0U;
}

static int32_t udc_dwc3_set_device_address(udc_dwc3_driver_t *drv, uint8_t  addr)
{
	uint32_t reg;

	if (addr > USB_DEVICE_MAX_ADDRESS) {
		LOG_ERR("Invalid address");
		return USB_DEVICE_SET_ADDRESS_INVALID;
	}
	if (drv->config_state == USBD_DWC3_STATE_CONFIGURED) {
		LOG_ERR("address can't set from Configured State");
		return USB_DEVICE_ALREADY_CONFIGURED;
	}
	reg = drv->regs->DCFG;
	reg &= ~(USB_DCFG_DEVADDR_MASK);
	reg |= USB_DCFG_DEVADDR(addr);
	drv->regs->DCFG = reg;

	if (addr > USB_DEVICE_DEFAULT_ADDRESS) {
		drv->config_state = USB_DWC3_STATE_ADDRESS;
	} else {
		drv->config_state = USB_DWC3_STATE_DEFAULT;
	}

	return USB_SUCCESS;
}

static void udc_dwc3_ep0_status_done(udc_dwc3_driver_t *drv)
{
	udc_dwc3_trb_t *trb_ptr;
	uint32_t trb_status;

	trb_ptr = &drv->ep0_trb;
	trb_status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);
	if (trb_status == USB_TRBSTS_SETUP_PENDING) {
		drv->setup_packet_pending = true;
		LOG_ERR("trb_status pending at status done");
	}
	drv->actual_length = 0;
	drv->ep0_state = EP0_SETUP_PHASE;
	/* prepare the setup trb to receive next setup packet */
	udc_dwc3_prepare_setup(drv);
}

static void udc_dwc3_ep0_data_done(udc_dwc3_driver_t *drv, uint8_t endp_number)
{
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t *trb_ptr;
	uint32_t trb_status;
	uint8_t length;

	ept = &drv->eps[endp_number];
	trb_ptr = &drv->ep0_trb;
	drv->actual_length = 0;
	sys_cache_data_invd_range(trb_ptr, sizeof(*trb_ptr));

	trb_status = USB_TRB_SIZE_TRBSTS(trb_ptr->size);
	if (trb_status == USB_TRBSTS_SETUP_PENDING) {
		drv->setup_packet_pending = true;
		LOG_ERR("TRB transmission pending in control DATA_PHASE");
		return;
	}
	length = trb_ptr->size & USB_TRB_SIZE_MASK;
	ept->bytes_txed = ept->ep_requested_bytes - length;
	drv->actual_length = ept->bytes_txed;
	if (endp_number != 0) {
		if (drv->udc_dwc3_data_in_cb != NULL) {
			drv->udc_dwc3_data_in_cb(drv, 0 | USB_REQUEST_IN);
		}
	} else {
		if (drv->udc_dwc3_data_out_cb != NULL) {
			drv->udc_dwc3_data_out_cb(drv, 0);
		}
	}
}

/* Endpoint specific events */
static void udc_dwc3_depevt_handler(udc_dwc3_driver_t *drv, uint32_t reg)
{
	udc_dwc3_ep_t *ept;
	uint8_t endp_number;
	uint32_t event_status;
	uint32_t event_type;

	/* Event status from bit[15:12]  */
	event_status = USB_GET_EP_EVENT_STATUS(reg);
	endp_number = USB_GET_DEPEVT_EP_NUM(reg);
	drv->endp_number = endp_number;
	ept = &drv->eps[endp_number];
	if (!(ept->ep_status & USB_EP_ENABLED)) {
		LOG_ERR("ep%u: stale DEPEVT discarded (endpoint not enabled)", endp_number);
		return;
	}
	/*  Get the event type  */
	event_type = USB_GET_DEPEVT_TYPE(reg);
	/* Event type can be used for Debugging purpose */
	drv->event_type = event_type;
	if (endp_number == USB_CTRL_PHY_EP0 || endp_number == USB_CTRL_PHY_EP1) {
		sys_cache_data_invd_range(&drv->setup_data, USB_SETUP_PKT_SIZE);
		sys_cache_data_invd_range(&drv->ep0_trb, sizeof(drv->ep0_trb));
		/* Get the physical endpoint associated with this endpoint.  */
		ept =  &drv->eps[endp_number];
		/* Reset the endpoint transfer status. */
		ept->ep_transfer_status =  USB_EP_TRANSFER_IDLE;
		/* Process the ep0 interrupts bases on event_type. */
		if (event_type == USB_DEPEVT_XFERNOTREADY) {
			if (event_status == USB_DEPEVT_STATUS_CONTROL_DATA) {
				if (endp_number != drv->ep0_expect_in) {
					LOG_ERR("unexpected direction for the data phase");
					udc_dwc3_ep0_end_control_data(drv, ept);
					udc_dwc3_ep0_stall_restart(drv, 0, 0);
				}
			} else if (event_status == USB_DEPEVT_STATUS_CONTROL_STATUS) {
				if (drv->setup_data.bRequest == USB_SET_ADDRESS_REQ) {
					udc_dwc3_set_device_address(drv, drv->setup_data.wValue);
				}
				udc_dwc3_ep0_send_recv_status(drv, endp_number);
			} else {
				/* Do nothing  */
			}
		} else if (event_type == USB_DEPEVT_XFERCOMPLETE) {
			udc_ctrl_request_t *setup;

			setup = &drv->setup_data;
			ept = &drv->eps[endp_number];
			CLEAR_BIT(ept->ep_status, USB_EP_BUSY);
			ept->ep_resource_index = 0U;
			drv->setup_packet_pending = false;
			switch (drv->ep0_state) {
			case EP0_SETUP_PHASE:
				ept->ep_transfer_status = USB_EP_TRANSFER_SETUP;
				if (setup->wLength == 0U) {
					drv->three_stage_setup = false;
					drv->ep0_expect_in = false;
				} else {
					drv->three_stage_setup = true;
					drv->ep0_expect_in = !!(setup->bRequestType
							& USB_REQUEST_IN);
				}
				if (drv->udc_dwc3_setupstage_cb != NULL) {
					drv->udc_dwc3_setupstage_cb(drv);
				}
				break;
			case EP0_DATA_PHASE:
				ept->ep_transfer_status = USB_EP_TRANSFER_DATA_COMPLETION;
				udc_dwc3_ep0_data_done(drv, endp_number);
				break;
			case EP0_STATUS_PHASE:
				ept->ep_transfer_status = USB_EP_TRANSFER_STATUS_COMPLETION;
				udc_dwc3_ep0_status_done(drv);
				break;
			default:
				break;
			}
		} else {
			LOG_ERR("some other events");
		}
	} else {
		/* non control ep events */
		uint32_t cmd_param;
		switch (event_type) {
		case USB_DEPEVT_XFERINPROGRESS:
			udc_dwc3_ep_xfer_complete(drv, endp_number);
			break;
		case USB_DEPEVT_XFERCOMPLETE:
			break;
		case USB_DEPEVT_XFERNOTREADY:
			/* for Isoc_only */
			/* Get the frame from event param BIT[31:16] */
			cmd_param = USB_GET_EVENT_CMD_PARAM(reg);
			usbd_isoc_micro_frame_update(drv, cmd_param);
			break;
		default:
			break;
		}
	}
}

static void udc_dwc3_connection_done_event(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	uint32_t reg;
	uint32_t speed;

	reg = drv->regs->DSTS;
	speed = reg & USB_DSTS_CONNECTSPD;
	switch (speed) {
	case USB_DSTS_HIGHSPEED:
		priv->speed = UDC_BUS_SPEED_HS;
		/* Enable USB2 LPM Capability — HS only feature */
		reg = drv->regs->DCFG;
		SET_BIT(reg, USB_DCFG_LPM_CAP);
		drv->regs->DCFG = reg;
		reg = drv->regs->DCTL;
		reg |= USB_DCTL_HIRD_THRES_MASK;
		drv->regs->DCTL = reg;
		break;
	case USB_DSTS_FULLSPEED:
		priv->speed = UDC_BUS_SPEED_FS;
		LOG_WRN("Full-speed connection detected");
		break;
	default:
		priv->speed = UDC_BUS_UNKNOWN;
		LOG_ERR("Unsupported connection speed: 0x%x", speed);
		break;
	}
}
/* Reset the USB device */
static void udc_dwc3_reset_event(udc_dwc3_driver_t *drv)
{
	uint32_t reg;
	uint32_t index;

	reg = drv->regs->DCTL;
	reg &= ~USB_DCTL_TSTCTRL_MASK;
	drv->regs->DCTL = reg;
	/* Clear STALL on all endpoints */
	udc_dwc3_clear_stall_all_eps(drv);
	for (index = 0U; index < (drv->out_eps + drv->in_eps); index++) {
		drv->eps[index].ep_status = 0U;
	}
	/* Reset device address to zero */
	reg = drv->regs->DCFG;
	reg &= ~(USB_DCFG_DEVADDR_MASK);
	drv->regs->DCFG = reg;
}

/* Device specific events   */
static void udc_dwc3_devt_handler(udc_dwc3_driver_t *drv, uint32_t reg)
{
	uint32_t event_type;

	/* Device specific events */
	event_type = USB_DEVT_TYPE(reg);
	switch (event_type) {
	case USB_EVENT_WAKEUP:
		if (drv->udc_dwc3_wakeup_cb != NULL) {
			drv->udc_dwc3_wakeup_cb(drv);
		}
		break;
	case USB_EVENT_DISCONNECT:
		if (drv->udc_dwc3_disconnect_cb != NULL) {
			drv->udc_dwc3_disconnect_cb(drv);
		}
		break;
	case USB_EVENT_EOPF:
		break;
	case USB_EVENT_RESET:
		udc_dwc3_reset_event(drv);
		if (drv->udc_dwc3_device_reset_cb != NULL) {
			drv->udc_dwc3_device_reset_cb(drv);
		}
		break;
	case USB_EVENT_CONNECT_DONE:
		udc_dwc3_connection_done_event(drv);
		if (drv->udc_dwc3_connect_cb != NULL) {
			drv->udc_dwc3_connect_cb(drv);
		}
		udc_dwc3_prepare_setup(drv);
		break;
	case USB_EVENT_LINK_STATUS_CHANGE: {
		uint32_t link_state = USB_DEVT_LINK_STATE_INFO(reg);

		switch (link_state) {
		case USB_LINK_STATE_ON:
			LOG_DBG("Link state: On (U0)");
			break;
		case USB_LINK_STATE_SLEEP_L1:
			LOG_DBG("Link state: Sleep (L1)");
			break;
		case USB_LINK_STATE_SUSPEND:
			LOG_DBG("Link state: Suspend (L2)");
			if (drv->udc_dwc3_suspend_cb != NULL) {
				drv->udc_dwc3_suspend_cb(drv);
			}
			break;
		case USB_LINK_STATE_DISCONNECTED:
			LOG_DBG("Link state: Disconnected");
			break;
		case USB_LINK_STATE_EARLY_SUSPEND:
			LOG_DBG("Link state: Early Suspend");
			break;
		default:
			LOG_DBG("Link state: Unknown (0x%x)", link_state);
			break;
		}
		break;
	}
	default:
		break;
	}
}

static void udc_dwc3_event_buffer_handler(udc_dwc3_driver_t *drv,
		udc_dwc3_event_buffer_t *event_buffer)
{
	uint32_t reg;

	sys_cache_data_invd_range(event_buffer->buf, USB_EVENT_BUFFER_SIZE);
	while (event_buffer->count) {
		reg = ((uint32_t)(*((uint32_t *) ((uint8_t *)event_buffer->buf
				+ event_buffer->lpos))));
		/* Check type of event */
		if ((reg & USB_EVENT_TYPE_CHECK) == USB_DEV_EVENT_TYPE) {
			/* process the device specific events  */
			udc_dwc3_devt_handler(drv, reg);
		} else if ((reg & USB_EVENT_TYPE_CHECK) == USB_EP_EVENT_TYPE) {
			/* process the endpoint specific events  */
			udc_dwc3_depevt_handler(drv, reg);
		} else {
			LOG_ERR("Unknown events");
		}
		event_buffer->lpos = (event_buffer->lpos + USB_EVENT_CNT_SIZE) %
							USB_EVENT_BUFFER_SIZE;
		event_buffer->count -= USB_EVENT_CNT_SIZE;
		drv->regs->GEVNTCOUNT0 = USB_EVENT_CNT_SIZE;
	}
	/* Unmask interrupt */
	event_buffer->count = 0;
}

static void udc_dwc3_interrupt_handler(udc_dwc3_driver_t  *drv)
{
	uint32_t pending_interrupt;
	uint32_t mask_interrupt;
	udc_dwc3_event_buffer_t  *event_buf;

	/* Get event pointer ...*/
	event_buf = drv->event_buf;
	pending_interrupt = drv->regs->GEVNTCOUNT0;
	pending_interrupt &= USB_GEVNTCOUNT_MASK;
	if (!pending_interrupt) {
		LOG_ERR("no pending irq");
		return;
	}
	event_buf->count = pending_interrupt;
	/* Set the Event Interrupt Mask */
	mask_interrupt = drv->regs->GEVNTSIZ0;
	SET_BIT(mask_interrupt, USB_GEVNTSIZ_INTMASK);
	drv->regs->GEVNTSIZ0 = mask_interrupt;
	/* Processes events in an Event Buffer */
	udc_dwc3_event_buffer_handler(drv, event_buf);
	/* Clear the Event Interrupt Mask */
	mask_interrupt = drv->regs->GEVNTSIZ0;
	CLEAR_BIT(mask_interrupt, USB_GEVNTSIZ_INTMASK);
	drv->regs->GEVNTSIZ0 = mask_interrupt;
}

static int32_t udc_dwc3_ep_disable(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir)
{
	udc_dwc3_ep_t *ept;
	uint32_t reg;
	uint8_t phy_ep;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	reg = drv->regs->DALEPENA;
	reg &= ~USB_DALEPENA_EP(phy_ep);
	drv->regs->DALEPENA = reg;
	CLEAR_BIT(ept->ep_status, USB_EP_ENABLED);

	return USB_SUCCESS;
}

static int32_t udc_dwc3_ep_clearstall(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir)
{
	uint8_t  phy_ep;
	udc_dwc3_ep_t *ept = NULL;
	udc_dwc3_ep_params_t params = {0};
	int32_t ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_CLEARSTALL, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("failed to send CLEARSTALL command");
		return ret;
	}
	CLEAR_BIT(ept->ep_status, USB_EP_STALL | USB_EP_WEDGE);

	return ret;
}

static int32_t udc_dwc3_ep_stall(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir)
{
	uint8_t phy_ep;
	int32_t ret;
	udc_dwc3_ep_params_t params = {0};

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	/* Handle control endpoint (EP0) */
	if (ep_num == 0) {
		udc_dwc3_trb_t *trb_ptr = &drv->ep0_trb;

		/* Check if hardware owns the TRB */
		if (trb_ptr->ctrl & USB_TRB_CTRL_HWO) {
			return USB_SUCCESS;
		}
		/* Issue the stall on control endpoint and restart */
		return udc_dwc3_ep0_stall_restart(drv, 0, 0);
	}
	/* Handle non-control endpoints */
	udc_dwc3_ep_t *ept = &drv->eps[phy_ep];

	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETSTALL, params);
	if (ret < USB_SUCCESS) {
		LOG_ERR("failed to send STALL command");
		return ret;
	}
	SET_BIT(ept->ep_status, USB_EP_STALL);

	return ret;
}
static int32_t udc_dwc3_configure_endpoint_parameters(udc_dwc3_driver_t *drv, uint8_t ep_num,
		uint8_t dir, uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t ep_interval)
{
	udc_dwc3_ep_params_t params = {0};
	uint8_t phy_ep;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	params.param0 = USB_DEPCFG_EP_TYPE(ep_type)
			| USB_DEPCFG_MAX_PACKET_SIZE(ep_max_packet_size);
	params.param0 |= USB_DEPCFG_ACTION_INIT;
	SET_BIT(params.param1, USB_DEPCFG_XFER_COMPLETE_EN | USB_DEPCFG_XFER_NOT_READY_EN);
	params.param1 |= USB_DEPCFG_EP_NUMBER(phy_ep);
	if (dir != USB_DIR_OUT) {
		params.param0 |= USB_DEPCFG_FIFO_NUMBER(phy_ep >> 1U);
	}
	if (ep_type != 0) {
		SET_BIT(params.param1, USB_DEPCFG_XFER_IN_PROGRESS_EN);
	}
	if (ep_interval) {
		params.param1 |= USB_DEPCFG_BINTERVAL_M1(ep_interval - 1);
	}

	return udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETEPCONFIG, params);
}

static int32_t udc_dwc3_set_xfer_resource(udc_dwc3_driver_t *drv, uint8_t phy_ep)
{
	udc_dwc3_ep_params_t params = {0};
	/* Set Endpoint Transfer Resource configuration parameter
	 * This field must be set to 1
	 */
	params.param0 = USB_DEPXFERCFG_NUM_XFER_RES(1U);
	return udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_SETTRANSFRESOURCE, params);
}

static int32_t udc_dwc3_start_endpoint_config(udc_dwc3_driver_t *drv, uint8_t ep_num,
		uint8_t dir)
{
	udc_dwc3_ep_params_t params = {0};
	uint8_t phy_ep;
	uint8_t ep_index;
	int32_t ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	if (phy_ep != 0) {
		return 0;
	}
	/* Issue the command to the hardware */
	ret = udc_dwc3_send_ep_cmd(drv, phy_ep, USB_DEPCMD_DEPSTARTCFG, params);
	if (ret) {
		LOG_ERR("USB_DEPCMD_DEPSTARTCFG cmd failed");
		return ret;
	}
	for (ep_index = 0; ep_index < (drv->in_eps + drv->out_eps); ep_index++) {
		ret = udc_dwc3_set_xfer_resource(drv, ep_index);
		if (ret) {
			LOG_ERR("Failed to set the xferresource command");
			return ret;
		}
	}

	return ret;
}

static int32_t udc_dwc3_ep_enable(udc_dwc3_driver_t *drv, uint8_t ep_num, uint8_t dir,
		uint8_t ep_type, uint16_t ep_max_packet_size, uint8_t ep_interval)
{
	udc_dwc3_ep_t *ept;
	uint32_t reg;
	uint8_t phy_ep;
	int32_t ret;

	phy_ep = USB_GET_PHYSICAL_EP(ep_num, dir);
	ept = &drv->eps[phy_ep];
	ept->ep_index = ep_num;
	ept->ep_dir = dir;
	ept->ep_maxpacket = ep_max_packet_size;
	ept->phy_ep   = phy_ep;
	ept->ep_type  = ep_type;
	ept->ep_interval = ep_interval;
	if (ep_interval) {
		ept->interval_uframe = 1 << (ep_interval - 1);
	} else {
		ept->interval_uframe = 0;
	}
	if (!(ept->ep_status & USB_EP_ENABLED)) {
		ret = udc_dwc3_start_endpoint_config(drv, ep_num, dir);
		if (ret) {
			LOG_ERR("Endpoint config failed");
			return ret;
		}
	}
	ret = udc_dwc3_configure_endpoint_parameters(drv, ep_num, dir, ep_type, ep_max_packet_size,
			ep_interval);
	if (ret) {
		LOG_ERR("configuring endpoint parameters failed");
		return ret;
	}
	if (!(ept->ep_status & USB_EP_ENABLED)) {
		SET_BIT(ept->ep_status, USB_EP_ENABLED);
		reg = drv->regs->DALEPENA;
		reg |=  USB_DALEPENA_EP(ept->phy_ep);
		drv->regs->DALEPENA = reg;
		if (phy_ep > 1) {
			udc_dwc3_trb_t *trb_ptr, *trb_link;
			/* Initialize TRB ring   */
			ept->trb_enqueue = 0;
			ept->trb_dequeue = 0;
			trb_ptr = &ept->ep_trb[0U];
			/* Link TRB. The HWO bit is never reset */
			trb_link = &ept->ep_trb[NO_OF_TRB_PER_EP];
			memset(trb_link, 0x0, sizeof(udc_dwc3_trb_t));
#if CONFIG_UDC_DWC3_ALIF
			trb_link->buf_ptr_low = LOWER_32_BITS(local_to_global(trb_ptr));
#else
			trb_link->buf_ptr_low = LOWER_32_BITS(trb_ptr);
#endif
			trb_link->buf_ptr_high = 0;
			trb_link->ctrl |= USB_TRBCTL_LINK_TRB;
			SET_BIT(trb_link->ctrl, USB_TRB_CTRL_HWO);
			return USB_SUCCESS;
		}
		return USB_SUCCESS;
	}

	return USB_SUCCESS;
}

static int32_t  udc_dwc3_endpoint_create(udc_dwc3_driver_t *drv, uint8_t ep_type,
		uint8_t ep_num, uint8_t dir, uint16_t ep_max_packet_size, uint8_t ep_interval)
{
	int32_t status;

	if (ep_type > USB_INTERRUPT_EP) {
		LOG_ERR("Invalid Endpoint");
		return USB_EP_INVALID;
	}
	status = udc_dwc3_ep_enable(drv, ep_num, dir, ep_type,
			ep_max_packet_size, ep_interval);
	if (status) {
		LOG_ERR("Failed to enable ep num %d: direction: %d", ep_num, dir);
	}

	return status;
}

static void udc_dwc3_set_speed(udc_dwc3_driver_t *drv)
{
	uint32_t  reg;

	reg = drv->regs->DCFG;
	reg &= ~(USB_DCFG_SPEED_MASK);
	reg |= USB_DCFG_HIGHSPEED;
	drv->regs->DCFG = reg;
}

static void udc_dwc3_initialize_physical_eps(udc_dwc3_driver_t *drv)
{
	uint8_t  ep_num;

	for (ep_num = 0; ep_num < (drv->out_eps + drv->in_eps); ep_num++) {
		drv->eps[ep_num].phy_ep             = 0;
		drv->eps[ep_num].ep_dir             = 0;
		drv->eps[ep_num].ep_resource_index  = 0U;
		drv->eps[ep_num].ep_status          = 0U;
		drv->eps[ep_num].ep_index           = 0U;
		drv->eps[ep_num].ep_transfer_status = 0U;
		drv->eps[ep_num].ep_maxpacket       = 0U;
		drv->eps[ep_num].ep_requested_bytes = 0U;
		drv->eps[ep_num].trb_enqueue        = 0U;
		drv->eps[ep_num].trb_dequeue        = 0U;
		drv->eps[ep_num].bytes_txed         = 0U;
		drv->eps[ep_num].unaligned_txed     = 0U;
	}
	/* Fill the TRB memory with zeros */
	for (ep_num = 0; ep_num < (drv->out_eps + drv->in_eps); ep_num++) {
		memset(&drv->eps[ep_num].ep_trb[0], 0x00, USB_TRBS_PER_EP * USB_TRB_STRUCTURE_SIZE);
	}
}

static void udc_dwc3_configure_event_buffer_registers(udc_dwc3_driver_t *drv)
{
	udc_dwc3_event_buffer_t *event_buf;

	event_buf = drv->event_buf;
	event_buf->lpos = 0;
#if CONFIG_UDC_DWC3_ALIF
	drv->regs->GEVNTADRLO0 = local_to_global((void *)LOWER_32_BITS(event_buf->buf));
#else
	drv->regs->GEVNTADRLO0 = LOWER_32_BITS(event_buf->buf);
#endif
	drv->regs->GEVNTADRHI0 = 0;
	drv->regs->GEVNTSIZ0 = event_buf->length;
	drv->regs->GEVNTCOUNT0 = 0;
}

static udc_dwc3_event_buffer_t *udc_dwc3_init_event_buffer(udc_dwc3_driver_t *drv)
{
	udc_dwc3_event_buffer_t *event_buf;

	event_buf = &udc_dwc3_evt_buf;
	event_buf->length = USB_EVENT_BUFFER_SIZE;
	event_buf->buf = udc_dwc3_event_buff;
	/* Fill the event buffer with zeros */
	memset(event_buf->buf, 0, USB_EVENT_BUFFER_SIZE);
	return event_buf;
}

static int32_t wait_for_depcmd_completion(udc_dwc3_driver_t *drv, uint8_t ep_index)
{
	uint32_t reg;
	int32_t timeout = USB_DEPCMD_TIMEOUT;

	do {
		/* Read the endpoint cmd register */
		reg = drv->regs->USB_ENDPNT_CMD[ep_index].DEPCMD;
		if ((reg & USB_DEPCMD_CMDACT) == 0) {
			break;
		}
	} while (--timeout);
	if (timeout == 0) {
		LOG_ERR("Timeout waiting for command completion EP%d during device init", ep_index);
		return USB_EP_CMD_CMPLT_TIMEOUT_ERROR;
	}

	return USB_SUCCESS;
}


static void udc_dwc3_enable_events(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	reg = drv->regs->DEVTEN;
	SET_BIT(reg, USB_DEV_DISSCONNEVTEN);
	SET_BIT(reg, USB_DEV_USBRSTEVTEN);
	SET_BIT(reg, USB_DEV_CONNECTDONEEVTEN);
	SET_BIT(reg, USB_DEV_WKUPEVTEN);
	SET_BIT(reg, USB_DEV_EVENT_ULSTCNGEN);
	drv->regs->DEVTEN = reg;
}

static uint32_t udc_dwc3_device_init(udc_dwc3_driver_t *drv)
{
	udc_dwc3_event_buffer_t *event_buf;
	uint32_t reg;
	uint32_t ret = USB_SUCCESS;

	/* Initialize the event buffer structure */
	event_buf = udc_dwc3_init_event_buffer(drv);
	if (event_buf == NULL) {
		LOG_ERR("Event buff allocation failed");
		return USB_EVNT_BUFF_ALLOC_ERROR;
	}
	drv->event_buf = event_buf;
	/* Configure the event buffer registers in the controller */
	udc_dwc3_configure_event_buffer_registers(drv);
	/* get the IN and OUT endpoints */
	reg = drv->regs->GHWPARAMS3;
	drv->in_eps = USB_IN_EPS(reg);
	drv->out_eps = (USB_NUM_EPS(reg) - drv->in_eps);
    /* Initialize the physical endpoints */
	udc_dwc3_initialize_physical_eps(drv);
	/* Set speed on Controller */
	udc_dwc3_set_speed(drv);
	/* Check controller USB2 phy config before issuing DEPCMD always */
	udc_dwc3_disable_phy_suspend(drv);
	/* Issuing DEPSTARTCFG Command to ep_resource[0] (for EP 0/CONTROL/) */
	drv->regs->USB_ENDPNT_CMD[0].DEPCMD = USB_DEPCMD_DEPSTARTCFG | USB_DEPCMD_CMDACT;
	ret = wait_for_depcmd_completion(drv, 0);
	if (ret) {
		return ret;
	}
	udc_dwc3_restore_suspend_state(drv);
	/* Check usb2phy config before issuing DEPCMD for EP0 OUT */
	udc_dwc3_disable_phy_suspend(drv);
	/* Issuing DEPCFG Command to ep_resource[0] (for EP 0 OUT/CONTROL/)
	 *bit[8] = XferComplete, bit[10] = XferNotReady *
	 *bit[25]: Endpoint direction:
	 *  0: OUT
	 *  1: IN
	 * bit[29:26] = Endpoint number
	 * Physical endpoint 0 (EP0) must be allocated for control endpoint 0 OUT.
	 * Physical endpoint 1 (EP1) must be allocated for control endpoint 0 IN.
	 */
	drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR1 = USB_DEPCMD_EP0_OUT_PAR1;
	/* bit[25:22] = Burst size and Burst size =0 for Control endpoint
	 * bit[21:17] = FIFONum, and FIFONum = 0;
	 * bit[13:3] =  Maximum Packet Size (MPS)
	 * bit[2:1] = Endpoint Type (EPType)
	 *  00: Control
	 *  01: Isochronous
	 *  10: Bulk
	 *  11: Interrupt
	 */
	drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP0_OUT_PAR0;
	drv->regs->USB_ENDPNT_CMD[0].DEPCMD = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
	ret = wait_for_depcmd_completion(drv, 0);
	if (ret) {
		return ret;
	}
	udc_dwc3_restore_suspend_state(drv);
	/* Check usb2phy config before issuing DEPCMD for EP1 IN */
	udc_dwc3_disable_phy_suspend(drv);
	/* Issuing DEPCFG Command to ep_resource[1] (for EP 1 IN/CONTROL/) */
	drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR1 = USB_DEPCMD_EP1_IN_PAR1;
	drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP1_IN_PAR0;
	drv->regs->USB_ENDPNT_CMD[1].DEPCMD = USB_DEPCMD_SETEPCONFIG | USB_DEPCMD_CMDACT;
	ret = wait_for_depcmd_completion(drv, 1);
	if (ret) {
		return ret;
	}
	/* Restore the USB2 phy state  */
	udc_dwc3_restore_suspend_state(drv);
	/* Check usb2phy config before issuing DEPCMD for EP0 OUT */
	udc_dwc3_disable_phy_suspend(drv);
	/* Issuing Transfer Resource command for each initialized endpoint */
	drv->regs->USB_ENDPNT_CMD[0].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
	drv->regs->USB_ENDPNT_CMD[0].DEPCMD = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
	ret = wait_for_depcmd_completion(drv, 0);
	if (ret) {
		return ret;
	}
	/* Restore the USB2 phy state  */
	udc_dwc3_restore_suspend_state(drv);
	/* Check usb2phy config before issuing DEPCMD for EP1 IN */
	udc_dwc3_disable_phy_suspend(drv);
	drv->regs->USB_ENDPNT_CMD[1].DEPCMDPAR0 = USB_DEPCMD_EP_XFERCFG_PAR0;
	drv->regs->USB_ENDPNT_CMD[1].DEPCMD = USB_DEPCMD_SETTRANSFRESOURCE | USB_DEPCMD_CMDACT;
	ret = wait_for_depcmd_completion(drv, 1);
	if (ret) {
		return ret;
	}
	/* Restore the USB2 phy state  */
	udc_dwc3_restore_suspend_state(drv);
	/* enable USB Reset, Connection Done, and USB/Link State Change events */
	udc_dwc3_enable_events(drv);

	return ret;
}

static void udc_dwc3_set_port_capability(udc_dwc3_driver_t *drv, uint32_t mode)
{
	uint32_t reg;

	reg = drv->regs->GCTL;
	reg &= ~(USB_GCTL_PRTCAPDIR(USB_GCTL_PRTCAP_OTG));
	/* Set the mode   */
	reg |= USB_GCTL_PRTCAPDIR(mode);
	drv->regs->GCTL = reg;
	drv->current_dr_role = mode;
}

static int32_t udc_dwc3_set_controller_mode(udc_dwc3_driver_t *drv)
{
	int32_t ret = USB_SUCCESS;

	switch (drv->dr_mode) {
	case USB_DR_MODE_HOST:
		udc_dwc3_set_port_capability(drv, USB_GCTL_PRTCAP_HOST);
		break;
	case USB_DR_MODE_PERIPHERAL:
		udc_dwc3_set_port_capability(drv, USB_GCTL_PRTCAP_DEVICE);
		ret = udc_dwc3_device_init(drv);
		if (ret) {
			LOG_ERR("USB device init failed");
		}
		break;
	case USB_DR_MODE_OTG:
		ret = USB_MODE_UNSUPPORTED;
		break;
	default:
		ret = USB_INIT_ERROR;
	}

	return ret;
}

static void udc_dwc3_configure_burst_transfer(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	/* Define incr in global soc bus configuration.  */
	reg = drv->regs->GSBUSCFG0;
	SET_BIT(reg, USB_GSBUSCFG0_INCRBRSTENA | USB_GSBUSCFG0_INCR32BRSTENA);
	drv->regs->GSBUSCFG0 = reg;
}

static void udc_dwc3_configure_fladj_register(udc_dwc3_driver_t *drv)
{
	uint32_t reg;
	uint32_t frame_length;

	reg = drv->regs->GFLADJ;
	frame_length = reg & USB_GFLADJ_30MHZ_MASK;
	if (frame_length != drv->fladj) {
		reg &= ~USB_GFLADJ_30MHZ_MASK;
		reg |= USB_GFLADJ_30MHZ_SDBND_SEL | drv->fladj;
		drv->regs->GFLADJ = reg;
	}
}

static void udc_dwc3_configure_global_control_reg(udc_dwc3_driver_t *drv)
{
	uint32_t reg;
	uint32_t pwropt = USB_GHWPARAMS1_EN_PWROPT(drv->hwparams.hwparams1);

	/* DWC_USB3_EN_PWROPT > 0: clock gating supported
	 * DWC_USB3_EN_PWROPT = 2: hibernation  supported
	 */
	switch (pwropt) {
	case USB_GHWPARAMS1_EN_PWROPT_HIB:
		LOG_INF("clock gating and hibernation supported");
		break;
	case USB_GHWPARAMS1_EN_PWROPT_CLK:
		LOG_INF("clock gating supported");
		break;
	default:
		LOG_INF("No power optimization supported");
		break;
	}

	reg = drv->regs->GCTL;
	SET_BIT(reg, USB_GCTL_DSBLCLKGTNG);
	drv->regs->GCTL = reg;
}

static void udc_dwc3_configure_usb2_phy(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	/*Configure Global USB2 PHY Configuration Register */
	reg = drv->regs->GUSB2PHYCFG0;
	/* The PHY must not be enabled for auto-resume in device mode.
	 * Therefore, the field GUSB2PHYCFG[15] (ULPIAutoRes) must be written with '0'
	 *  during the power-on initialization in case the reset value is '1'.
	 */
	if (reg & USB_GUSB2PHYCFG_ULPIAUTORES) {
		reg &= ~USB_GUSB2PHYCFG_ULPIAUTORES;
	}
	/* Enable PHYIF  */
	switch (drv->hsphy_mode) {
	case USB_PHY_INTERFACE_MODE_UTMI:
		reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
				USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USB_PHY_INTERFACE_MODE_UTMIW:
		reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK
				| USB_GUSB2PHYCFG_ULPI_UTMI);
		reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
				USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT)
				| USB_GUSB2PHYCFG_SUSPHY;
		break;
	default:
		break;
	}
	drv->regs->GUSB2PHYCFG0 = reg;
}

static void udc_dwc3_reset_phy_and_core(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	/* Before Resetting PHY, put Core in Reset */
	reg = drv->regs->GCTL;
	SET_BIT(reg, USB_GCTL_CORESOFTRESET);
	drv->regs->GCTL = reg;
	/* USB2 PHY reset */
	reg = drv->regs->GUSB2PHYCFG0;
	SET_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
	drv->regs->GUSB2PHYCFG0 = reg;
	k_busy_wait(50000);
	/* Clear USB2 PHY reset */
	reg = drv->regs->GUSB2PHYCFG0;
	CLEAR_BIT(reg, USB_GUSB2PHYCFG_PHYSOFTRST);
	drv->regs->GUSB2PHYCFG0 = reg;
	k_busy_wait(50000);
	/* Take Core out of reset state after PHYS are stable*/
	reg = drv->regs->GCTL;
	CLEAR_BIT(reg, USB_GCTL_CORESOFTRESET);
	drv->regs->GCTL = reg;
}

static int32_t udc_dwc3_device_soft_reset(udc_dwc3_driver_t *drv)
{
	uint32_t  reg;
	int32_t   timeout;

	reg = drv->regs->DCTL;
	SET_BIT(reg, USB_DCTL_CSFTRST);
	drv->regs->DCTL = reg;
	timeout = USB_DCTL_CSFTRST_TIMEOUT;
	/* Wait for Soft Reset to be completed.  */
	do {
		reg = drv->regs->DCTL;
		if (!(reg & USB_DCTL_CSFTRST)) {
			return USB_SUCCESS;
		}
	} while (--timeout);

	if (timeout == 0) {
		LOG_ERR("timeout for device soft reset");
		return USB_CORE_SFTRST_TIMEOUT_ERROR;
	}

	return USB_SUCCESS;
}

static void udc_dwc3_read_hw_params(udc_dwc3_driver_t *drv)
{
	udc_dwc3_hwparams_t *parms = &drv->hwparams;

	parms->hwparams0 = drv->regs->GHWPARAMS0;
	parms->hwparams1 = drv->regs->GHWPARAMS1;
	parms->hwparams2 = drv->regs->GHWPARAMS2;
	parms->hwparams3 = drv->regs->GHWPARAMS3;
	parms->hwparams4 = drv->regs->GHWPARAMS4;
	parms->hwparams5 = drv->regs->GHWPARAMS5;
	parms->hwparams6 = drv->regs->GHWPARAMS6;
	parms->hwparams7 = drv->regs->GHWPARAMS7;
	parms->hwparams8 = drv->regs->GHWPARAMS8;
}

static void udc_dwc3_set_default_config(udc_dwc3_driver_t *drv)
{

	drv->dr_mode = USB_DR_MODE_PERIPHERAL;
	/* default setting the phy mode */
	drv->hsphy_mode = USB_PHY_INTERFACE_MODE_UTMIW;
	/* default value for frame length  */
	drv->fladj = USB_GFLADJ_DEFAULT_VALUE;
}

static bool udc_dwc3_verify_ip_core(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	reg = drv->regs->GSNPSID;
	/* Read the USB core ID and followed by revision number */
	if ((reg & USB_GSNPSID_MASK) == 0x55330000) {
		/* Detected DWC_usb3 IP */
		drv->revision = reg;
	} else {
		LOG_ERR("Detected wrong IP");
		return false;
	}

	return true;
}

static void udc_dwc3_disable_events(udc_dwc3_driver_t *drv)
{
	uint32_t reg;

	reg = drv->regs->DEVTEN;
	CLEAR_BIT(reg, USB_DEV_DISSCONNEVTEN);
	CLEAR_BIT(reg, USB_DEV_USBRSTEVTEN);
	CLEAR_BIT(reg, USB_DEV_CONNECTDONEEVTEN);
	CLEAR_BIT(reg, USB_DEV_WKUPEVTEN);
	CLEAR_BIT(reg, USB_DEV_EVENT_ULSTCNGEN);
	drv->regs->DEVTEN = reg;
}

static int32_t udc_dwc3_disconnect(udc_dwc3_driver_t *drv)
{
	uint32_t reg;
	int32_t timeout = USB_DCTL_START_TIMEOUT;

	reg = drv->regs->DCTL;
	CLEAR_BIT(reg, USB_DCTL_START);
	drv->regs->DCTL = reg;

	/* Wait for controller to halt (DSTS.DEVCTRLHLT set) */
	do {
		reg = drv->regs->DSTS;
		if (reg & USB_DSTS_DEVCTRLHLT) {
			LOG_INF("USB controller stopped");
			return USB_SUCCESS;
		}
		k_busy_wait(1);
	} while (--timeout);

	LOG_ERR("Timeout waiting for controller to stop");
	return USB_CONTROLLER_INIT_FAILED;
}

static int32_t udc_dwc3_connect(udc_dwc3_driver_t *drv)
{
	uint32_t reg;
	int32_t timeout;

	/* Starting controller . */
	reg = drv->regs->DCTL;
	SET_BIT(reg, USB_DCTL_START);
	drv->regs->DCTL = reg;
	timeout = USB_DCTL_START_TIMEOUT;
	do {
		reg = drv->regs->DSTS;
		if (!(reg & USB_DSTS_DEVCTRLHLT)) {
			break;
		}
	} while (-- timeout);
	if (timeout == 0) {
		LOG_ERR("timeout to start the controller");
		drv->event_buf = NULL;
		return USB_CONTROLLER_INIT_FAILED;
	}
	LOG_INF("started USB controller");
	return USB_SUCCESS;
}

static void udc_dwc3_cleanup_event_buffer(udc_dwc3_driver_t *drv)
{
	drv->event_buf->lpos = 0;
	drv->regs->GEVNTADRLO0 = 0;
	drv->regs->GEVNTADRHI0 = 0;
	drv->regs->GEVNTSIZ0 = USB_GEVNTSIZ_INTMASK | USB_GEVNTSIZ_SIZE(0);
	drv->regs->GEVNTCOUNT0 = 0;
}

static int32_t udc_dwc3_initialize(udc_dwc3_driver_t *drv)
{
	int32_t ret = USB_SUCCESS;

	if (drv == NULL) {
		return USB_INIT_ERROR;
	}
	drv->udc_dwc3_device_reset_cb = dwc3_reset_cb;
	drv->udc_dwc3_connect_cb = dwc3_connect_cb;
	drv->udc_dwc3_setupstage_cb = dwc3_setupstage_cb;
	drv->udc_dwc3_disconnect_cb = dwc3_disconnect_cb;
	drv->udc_dwc3_wakeup_cb = dwc3_wakeup_cb;
	drv->udc_dwc3_suspend_cb = dwc3_suspend_cb;
	drv->udc_dwc3_data_in_cb = dwc3_data_in_cb;
	drv->udc_dwc3_data_out_cb = dwc3_data_out_cb;
	/* Verify the USB controller's IP core is valid */
	if (!udc_dwc3_verify_ip_core(drv)) {
		LOG_ERR("Invalid USB controller IP core detected");
		ret = USB_CORE_INVALID;
		return ret;
	}
	udc_dwc3_set_default_config(drv);
	udc_dwc3_read_hw_params(drv);

	/* Perform the device controller soft reset */
	ret = udc_dwc3_device_soft_reset(drv);
	if (ret != 0) {
		LOG_ERR("device soft reset failed");
		return ret;
	}
	/* Spec says wait for few cycles.  */
	k_busy_wait(5000);
	/* Reset the PHY and Core */
	udc_dwc3_reset_phy_and_core(drv);
	/* Configure USB2 PHY interface */
	udc_dwc3_configure_usb2_phy(drv);
	/* Configure Global control register  */
	udc_dwc3_configure_global_control_reg(drv);
	/* Configure GFLADJ register for 30MHz reference clock */
	udc_dwc3_configure_fladj_register(drv);
	/* Configure burst transfer settings */
	udc_dwc3_configure_burst_transfer(drv);
	/* Set the USB controller operating mode (host/device) */
	ret = udc_dwc3_set_controller_mode(drv);
	if (ret != USB_SUCCESS) {
		udc_dwc3_cleanup_event_buffer(drv);
		LOG_ERR("failed to set the USB controller mode");
		return ret;
	}
	/* Return successful completion.  */
	return ret;
}
void dwc3_reset_cb(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);

	udc_submit_event(priv->dev, UDC_EVT_RESET, 0);
}
void dwc3_connect_cb(udc_dwc3_driver_t *drv)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	uint8_t ep_type;
	uint8_t ep_interval;
	int status;
	struct udc_ep_config *ep;

	struct udc_dwc3_data *priv = DWC3DATA(drv);
	const struct device *dev = priv->dev;

	/* Re-Enable control endpoints */
	ep = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (ep && ep->stat.enabled) {
		ep_num = EP_NUM(ep->addr);
		ep_dir = (ep->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
		ep_type = ep->attributes & USB_EP_TRANSFER_TYPE_MASK;
		ep_interval = ep->interval;
		status =  udc_dwc3_endpoint_create(&priv->drv, ep_type, ep_num,
				ep_dir, ep->mps, ep_interval);
		if (status != 0) {
			return;
		}
	}
	ep = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	if (ep && ep->stat.enabled) {
		ep_num = EP_NUM(ep->addr);
		ep_dir = (ep->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
		ep_type = ep->attributes & USB_EP_TRANSFER_TYPE_MASK;
		ep_interval = ep->interval;
		status =  udc_dwc3_endpoint_create(&priv->drv, ep_type, ep_num,
				ep_dir, ep->mps, ep_interval);
		if (status != 0) {
			return;
		}
	}
	udc_submit_event(priv->dev, UDC_EVT_VBUS_READY, 0);
}

void dwc3_disconnect_cb(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_REMOVED, 0);
}

void dwc3_wakeup_cb(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	uint32_t reg;

	/* Disable clock gating — restore normal operation after suspend */
	reg = drv->regs->GCTL;
	SET_BIT(reg, USB_GCTL_DSBLCLKGTNG);
	drv->regs->GCTL = reg;
	udc_set_suspended(priv->dev, false);
	udc_submit_event(priv->dev, UDC_EVT_RESUME, 0);
}

void dwc3_suspend_cb(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	uint32_t reg;

	/* Enable clock gating — allow controller to save power during USB suspend */
	reg = drv->regs->GCTL;
	CLEAR_BIT(reg, USB_GCTL_DSBLCLKGTNG);
	drv->regs->GCTL = reg;
	if (!udc_is_suspended(priv->dev)) {
		udc_set_suspended(priv->dev, true);
	}
	udc_submit_event(priv->dev, UDC_EVT_SUSPEND, 0);
}

void dwc3_setupstage_cb(udc_dwc3_driver_t *drv)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	struct udc_dwc3_msg msg = {.type = UDC_DWC3_MSG_SETUP};
	int err;

	memcpy(&msg.setup, &drv->setup_data, sizeof(msg.setup));
	err = k_msgq_put(&priv->dwc3_msgq_data, &msg, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

void dwc3_data_in_cb(udc_dwc3_driver_t *drv, uint8_t ep_num)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	struct udc_dwc3_msg msg = {
		.type = UDC_DWC3_MSG_DATA_IN,
		.ep = ep_num,
	};
	int err;

	err = k_msgq_put(&priv->dwc3_msgq_data, &msg, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

void dwc3_data_out_cb(udc_dwc3_driver_t *drv, uint8_t ep_num)
{
	struct udc_dwc3_data *priv = DWC3DATA(drv);
	struct udc_dwc3_msg msg = {
		.type = UDC_DWC3_MSG_DATA_OUT,
		.ep = ep_num,
		.recv_bytes = ep_num ? drv->num_bytes : drv->actual_length,
	};
	int err;

	err = k_msgq_put(&priv->dwc3_msgq_data, &msg, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

static void udc_dwc3_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_dwc3_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static int udc_dwc3_init(const struct device *dev)
{
	int  ret;
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config * const udc_dwc3_cfg = dev->config;

	priv->drv.regs = udc_dwc3_cfg->base;

	if (!device_is_ready(udc_dwc3_cfg->clock_dev)) {
		return -EINVAL;
	}
	/* Enable clock (USB) */
	ret = clock_control_on(udc_dwc3_cfg->clock_dev, udc_dwc3_cfg->clock_subsys);
	if (ret != 0 && ret != -EALREADY) {
		return ret;
	}
#if DT_INST_NUM_CLOCKS(0) > 1
	ret = clock_control_on(udc_dwc3_cfg->clock_dev, udc_dwc3_cfg->clock_subsys2);
	if (ret != 0 && ret != -EALREADY) {
		return ret;
	}
#endif
	ret = udc_dwc3_initialize(&priv->drv);
	if (ret) {
		LOG_ERR("USB controller initialization failed");
	}

	return ret;
}

static int udc_dwc3_enable(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	int status;

	status = udc_dwc3_connect(&priv->drv);
	if (status != 0) {
		LOG_ERR("USB controller enable failed");
	}
	status = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
			64, 0);
	if (status) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_OUT);
		return status;
	}
	status = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
			64, 0);
	if (status) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_IN);
		return status;
	}
	cfg->irq_enable_func(dev);

	return status;
}

static int udc_dwc3_disable(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}
	if (udc_dwc3_disconnect(&priv->drv) != USB_SUCCESS) {
		LOG_ERR("Failed to stop USB controller");
		return -EIO;
	}
	udc_dwc3_disable_events(&priv->drv);
	cfg->irq_disable_func(dev);

	return 0;
}
static int udc_dwc3_shutdown(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg;
	int ret;

	irq_disable(priv->irq);

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (cfg && cfg->stat.enabled) {
		ret = udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
		if (ret) {
			LOG_DBG("Failed to disable control endpoint");
			return -EIO;
		}
	}

	cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_IN);
	if (cfg && cfg->stat.enabled) {
		ret = udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
		if (ret) {
			LOG_DBG("Failed to disable control endpoint");
			return -EIO;
		}
	}

	return 0;
}

static int udc_dwc3_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	return udc_dwc3_set_device_address(&priv->drv, addr);
}

static int udc_dwc3_host_wakeup(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	uint32_t reg;
	uint32_t timeout = USB_TRANSFER_WAKEUP_RETRY;

	/* Request link state transition to Recovery to generate Resume signaling */
	reg = priv->drv.regs->DCTL;
	reg &= ~USB_DCTL_ULSTCHNGREQ_MASK;
	reg |= USB_DCTL_ULSTCHNGREQ(USB_DCTL_ULSTCHNGREQ_REMOTE_WAKEUP);
	priv->drv.regs->DCTL = reg;

	/* Wait for link to return to active (U0/On) state */
	do {
		reg = priv->drv.regs->DSTS;
		if (USB_DSTS_USBLNKST(reg) == USB_LINK_STATE_ON) {
			return 0;
		}
		k_busy_wait(1);
	} while (--timeout);

	LOG_ERR("Remote wakeup failed: link state timeout");
	return -ETIMEDOUT;
}

static int udc_dwc3_ep_activate(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	uint8_t ep_type;
	uint8_t ep_interval;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep_cfg->addr);
	ep_dir = (ep_cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	ep_type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	ep_interval = ep_cfg->interval;

	return udc_dwc3_endpoint_create(&priv->drv, ep_type, ep_num, ep_dir,
			ep_cfg->mps, ep_interval);
}

static int udc_dwc3_ep_deactivate(const struct device *dev, struct udc_ep_config *ep)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep->addr);
	ep_dir = (ep->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	udc_dwc3_stop_transfer(&priv->drv, ep_num, ep_dir, USB_DEPCMD_FORCERM);
	return udc_dwc3_ep_disable(&priv->drv, ep_num, ep_dir);
}

static int udc_dwc3_ep_set_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(cfg->addr);
	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;

	return udc_dwc3_ep_stall(&priv->drv, ep_num, ep_dir);
}

static int udc_dwc3_ep_clear_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(cfg->addr);
	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	return udc_dwc3_ep_clearstall(&priv->drv, ep_num, ep_dir);
}

static int udc_dwc3_tx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	uint8_t *data;
	uint32_t len;
	uint8_t  ep_num, ep_dir;
	int ret;
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	uint8_t ep_type;

	if (ep_cfg == NULL) {
		LOG_ERR("No ep_cfg for ep 0x%02x", ep);
		return -ENODEV;
	}
	ep_type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;

	ep_num = EP_NUM(ep);
	ep_dir = (ep & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;

	data = buf->data;
	len = buf->len;
	buf->data += len;
	buf->len -= len;

	if ((data == NULL) && (len == 0) && (ep_dir != 0)) {
		return 0;
	}

	switch (ep_type) {
	case USB_CONTROL_EP:
		ret = udc_dwc3_ep0_send(&priv->drv, ep_num, ep_dir, data, len);
		break;
	case USB_BULK_EP:
	case USB_INTERRUPT_EP:
		ret = udc_dwc3_bulk_int_send(&priv->drv, ep_num, ep_dir, data, len);
		break;
	case USB_ISOCRONOUS_EP:
		ret = udc_dwc3_isoc_send(&priv->drv, ep_num, ep_dir, data, len);
		break;
	default:
		LOG_ERR("Invalid endpoint type");
		return -EINVAL;
	}

	if (ret == 0) {
		udc_ep_set_busy(dev, ep, true);
	}

	return ret;
}
static int udc_dwc3_rx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	uint8_t  ep_num, ep_dir;
	int ret;
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep);
	uint8_t ep_type;

	if (ep_cfg == NULL) {
		LOG_ERR("No ep_cfg for ep 0x%02x", ep);
		return -ENODEV;
	}
	ep_type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;

	ep_num = EP_NUM(ep);
	ep_dir = (ep & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;

	switch (ep_type) {
	case USB_CONTROL_EP:
		ret = udc_dwc3_ep0_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
		break;
	case USB_BULK_EP:
	case USB_INTERRUPT_EP:
		ret = udc_dwc3_bulk_int_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
		break;
	case USB_ISOCRONOUS_EP:
		LOG_WRN("Isoc Out transfer not yet implemented");
		return -ENOTSUP;
	default:
		LOG_ERR("Invalid endpoint type index %d", ep_type);
		return -EINVAL;
	}

	if (ret == 0) {
		udc_ep_set_busy(dev, ep, true);
	}

	return ret;
}
static int udc_dwc3_ep_enqueue(const struct device *dev, struct udc_ep_config *epcfg,
		struct net_buf *buf)
{
	unsigned int lock_key;
	int ret;

	udc_buf_put(epcfg, buf);
	lock_key = irq_lock();
	if (USB_EP_DIR_IS_IN(epcfg->addr)) {
		ret = udc_dwc3_tx(dev, epcfg->addr, buf);
	} else {
		ret = udc_dwc3_rx(dev, epcfg->addr, buf);
	}
	irq_unlock(lock_key);

	return ret;
}

static int udc_dwc3_ep_dequeue(const struct device *dev, struct udc_ep_config *epcfg)
{
	struct net_buf *buf;

	buf = udc_buf_get_all(dev, epcfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}
	udc_ep_set_busy(dev, epcfg->addr, false);
	return 0;
}

static enum udc_bus_speed udc_dwc3_device_speed(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	return priv->speed;
}

static void udc_dwc3_isr_handler(const struct device *dev)
{
	const struct udc_dwc3_data *priv =  udc_get_private(dev);

	udc_dwc3_interrupt_handler((udc_dwc3_driver_t *)&priv->drv);
}

static int udc_dwc3_ctrl_feed_dout(const struct device *dev, size_t length)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;
	uint8_t  ep_num, ep_dir;
	int ret;
	size_t alloc_len = length;

	/* Control OUT buffers must be multiple of bMaxPacketSize0 */
	alloc_len = ROUND_UP(length, USB_CONTROL_EP_MAX_PKT);
	ep_num = EP_NUM(cfg->addr);
	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, alloc_len);
	if (buf == NULL) {
		return -ENOMEM;
	}
	k_fifo_put(&cfg->fifo, buf);
	ret = udc_dwc3_ep0_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

static void udc_dwc3_drop_control_transfers(const struct device *dev)
{
	struct net_buf *buf;

	buf = udc_buf_get_all(dev, USB_CONTROL_EP_OUT);
	if (buf != NULL) {
		net_buf_unref(buf);
	}

	buf = udc_buf_get_all(dev, USB_CONTROL_EP_IN);
	if (buf != NULL) {
		net_buf_unref(buf);
	}
}


static void handle_setup_pkt(struct udc_dwc3_data *priv,
			    const struct udc_dwc3_msg *msg)
{
	const struct usb_setup_packet *setup = &msg->setup;
	const struct device *dev = priv->dev;
	struct net_buf *buf;
	int err;

	/* Free any stale buffers from the previous control transfer.
	 * EP0 OUT data buffers were not being properly released.
	 * For no-data control OUT transfers, the upper layer enqueues the
	 * status IN buffer on EP0-IN. The status stage was handled
	 * internally and the buffer may not be dequeued or freed afterward.
	 *
	 * This cleanup step drains any leftover buffers from both EP0-OUT
	 * and EP0-IN queues before processing the next setup packet,
	 * preventing exhaustion of the udc_ep_pool.
	 */
	udc_dwc3_drop_control_transfers(dev);

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buf for setup");
		return;
	}
	udc_ep_buf_set_setup(buf);
	memcpy(buf->data, setup, USB_SETUP_PKT_SIZE);
	net_buf_add(buf, USB_SETUP_PKT_SIZE);
	udc_ctrl_update_stage(dev, buf);
	if (!buf->len) {
		return;
	}
	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		err = udc_dwc3_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		udc_ctrl_submit_s_in_status(dev);
	} else {
		udc_ctrl_submit_s_status(dev);
	}
}

static void handle_data_in(struct udc_dwc3_data *priv, uint8_t epnum)
{
	const struct device *dev = priv->dev;
	uint8_t ep = epnum | USB_EP_DIR_IN;
	struct net_buf *buf;

	LOG_DBG("DataIn ep 0x%02x",  ep);
	udc_ep_set_busy(dev, ep, false);
	buf = udc_buf_peek(dev, ep);
	if (unlikely(buf == NULL)) {
		return;
	}

	udc_buf_get(dev, ep);
	if (ep == USB_CONTROL_EP_IN) {
		if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}
		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);
		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			net_buf_unref(buf);
		}
		return;
	}
	udc_submit_ep_event(dev, buf, 0);
	buf = udc_buf_peek(dev, ep);
	if (buf && buf->len) {
		udc_dwc3_tx(dev, ep, buf);
	}
}

static void handle_data_out(struct udc_dwc3_data *priv, uint8_t ep_num, uint16_t recv_bytes)
{
	const struct device *dev = priv->dev;
	uint8_t ep = ep_num | USB_EP_DIR_OUT;
	struct net_buf *buf;

	udc_ep_set_busy(dev, ep, false);
	buf = udc_buf_get(dev, ep);
	if (unlikely(buf == NULL)) {
		LOG_ERR("ep 0x%02x queue is empty", ep);
		return;
	}
	net_buf_add(buf, recv_bytes);
	if (ep == USB_CONTROL_EP_OUT) {
		if (udc_ctrl_stage_is_status_out(dev)) {
			udc_ctrl_update_stage(dev, buf);
			udc_ctrl_submit_status(dev, buf);
		} else {
			udc_ctrl_update_stage(dev, buf);
		}
		if (udc_ctrl_stage_is_status_in(dev)) {
			udc_ctrl_submit_s_out_status(dev, buf);
		}
	} else {
		udc_submit_ep_event(dev, buf, 0);
	}
	buf = udc_buf_peek(dev, ep);
	if (buf) {
		udc_dwc3_rx(dev, ep, buf);
	}
}

static void udc_dwc3_thread_handler(void *const arg)
{
	const struct device *dev = arg;
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_dwc3_msg msg;

	while (true) {
		k_msgq_get(&priv->dwc3_msgq_data, &msg, K_FOREVER);
		switch (msg.type) {
		case UDC_DWC3_MSG_SETUP:
			handle_setup_pkt(priv, &msg);
			break;
		case UDC_DWC3_MSG_DATA_IN:
			handle_data_in(priv, msg.ep);
			break;
		case UDC_DWC3_MSG_DATA_OUT:
			handle_data_out(priv, msg.ep, msg.recv_bytes);
			break;
		}
	}
}

static int udc_dwc3_driver_preinit(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	struct udc_data *data = dev->data;
	int err;
	/* default max packet size for high speed */
	uint16_t mps = 1023;

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;
	priv->speed = UDC_BUS_SPEED_FS;
	if (cfg->max_speed == 2) {
		data->caps.hs = true;
		mps = 1024;
		priv->speed = UDC_BUS_SPEED_HS;
	}
	for (unsigned int i = 0; i < cfg->num_out_eps; i++) {
		cfg->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			cfg->ep_cfg_out[i].caps.control = 1;
			cfg->ep_cfg_out[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_out[i].caps.bulk = 1;
			cfg->ep_cfg_out[i].caps.interrupt = 1;
			cfg->ep_cfg_out[i].caps.iso = 1;
			cfg->ep_cfg_out[i].caps.mps = mps;
		}
		cfg->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}
	for (unsigned int i = 0; i < cfg->num_in_eps; i++) {
		cfg->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			cfg->ep_cfg_in[i].caps.control = 1;
			cfg->ep_cfg_in[i].caps.mps = 64;
		} else {
			cfg->ep_cfg_in[i].caps.bulk = 1;
			cfg->ep_cfg_in[i].caps.interrupt = 1;
			cfg->ep_cfg_in[i].caps.iso = 1;
			cfg->ep_cfg_in[i].caps.mps = mps;
		}
		cfg->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &cfg->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}
	priv->dev = dev;
	k_msgq_init(&priv->dwc3_msgq_data, udc_dwc3_msgq_buf, sizeof(struct udc_dwc3_msg),
			CONFIG_UDC_DWC3_MAX_QMESSAGES);

	cfg->make_thread(dev);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static bool udc_dwc3_is_transfer_ongoing(struct udc_dwc3_data *priv)
{
	udc_dwc3_driver_t *drv = &priv->drv;
	udc_dwc3_ep_t *ept;
	udc_dwc3_trb_t *trb_ptr;
	uint8_t ep_num;

	/* Check control endpoint: if not in SETUP phase, transfer is active */
	if (drv->ep0_state != EP0_SETUP_PHASE) {
		LOG_WRN("PM suspend: Control EP has ongoing transfer (ep0_state=%d)",
			drv->ep0_state);
		return true;
	}

	/* Check non-control endpoints */
	for (ep_num = 2; ep_num < (drv->out_eps + drv->in_eps); ep_num++) {
		ept = &drv->eps[ep_num];

		/* Skip endpoints that are not enabled */
		if (!(ept->ep_status & USB_EP_ENABLED)) {
			continue;
		}

		/* Skip if no transfer has been started on this endpoint */
		if (ept->trb_enqueue == ept->trb_dequeue &&
			!(ept->ep_status & USB_EP_BUSY)) {
			continue;
		}

		/* Check HWO bit on the TRB at enqueue index.
		 * HWO=1: hardware owns TRB, device is ready/waiting for host
		 * HWO=0: transfer completed, pending software processing
		 */
		trb_ptr = &ept->ep_trb[ept->trb_enqueue];

		if (!(trb_ptr->ctrl & USB_TRB_CTRL_HWO)) {
			LOG_WRN("PM suspend: EP %d has ongoing transfer processing",
				ep_num);
			return true;
		}
	}

	return false;
}

static int udc_dwc3_suspend(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	int32_t ret;

	/* Check for any ongoing transfers before suspending */
	if (udc_dwc3_is_transfer_ongoing(priv)) {
		LOG_ERR("PM suspend: Cannot suspend with ongoing transfers");
		return -EBUSY;
	}

	/* Disable events to prevent spurious events during disconnect */
	udc_dwc3_disable_events(&priv->drv);
	/* Stop controller: RunStop=0, wait for DEVCTRLHLT */
	ret = udc_dwc3_disconnect(&priv->drv);
	if (ret != USB_SUCCESS) {
		LOG_ERR("PM suspend: controller stop failed");
		goto err_restore_events;
	}
	/* Disable the control endpoints */
	udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
	/* Disable USB IRQ at NVIC */
	cfg->irq_disable_func(dev);
	/* Cleanup event buffer */
	udc_dwc3_cleanup_event_buffer(&priv->drv);
	/* Disable clock (USB) */
	ret = clock_control_off(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("PM suspend: clock disable failed (%d)", ret);
		return ret;
	}
#if DT_INST_NUM_CLOCKS(0) > 1
	ret = clock_control_off(cfg->clock_dev, cfg->clock_subsys2);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("PM suspend: clock disable failed (%d)", ret);
		return ret;
	}
#endif

	return 0;

err_restore_events:
	udc_dwc3_enable_events(&priv->drv);
	return -EIO;
}

static int udc_dwc3_resume(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	int32_t ret;

	if (!device_is_ready(cfg->clock_dev)) {
		return -ENODEV;
	}
	/* Enable clock (USB) */
	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("PM resume: clock enable failed (%d)", ret);
		return ret;
	}
#if DT_INST_NUM_CLOCKS(0) > 1
	ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys2);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("PM resume: clock enable failed (%d)", ret);
		goto err_clock_off;
	}
#endif

	ret = udc_dwc3_device_init(&priv->drv);
	if (ret) {
		LOG_ERR("PM resume: device_init failed (%d)", ret);
		goto err_clock_off;
	}

	/* Enable the default control endpoint */
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
				     USB_CONTROL_EP_MAX_PKT, 0);
	if (ret) {
		LOG_ERR("PM resume: Failed enabling ep 0x%02x", USB_CONTROL_EP_OUT);
		goto err_clock_off;
	}
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     USB_CONTROL_EP_MAX_PKT, 0);
	if (ret) {
		LOG_ERR("PM resume: Failed enabling ep 0x%02x", USB_CONTROL_EP_IN);
		goto err_disable_ep_out;
	}
	/* Restart controller: RunStop=1 */
	ret = udc_dwc3_connect(&priv->drv);
	if (ret != USB_SUCCESS) {
		LOG_ERR("PM resume: controller start failed");
		goto err_disable_eps;
	}

	/* Re-enable USB IRQ at NVIC */
	cfg->irq_enable_func(dev);

	return 0;

err_disable_eps:
	udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
err_disable_ep_out:
	udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
err_clock_off:
	udc_dwc3_disable_events(&priv->drv);
	clock_control_off(cfg->clock_dev, cfg->clock_subsys);
#if DT_INST_NUM_CLOCKS(0) > 1
	clock_control_off(cfg->clock_dev, cfg->clock_subsys2);
#endif
	return -EIO;
}

static int udc_dwc3_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		return udc_dwc3_suspend(dev);
	case PM_DEVICE_ACTION_RESUME:
		return udc_dwc3_resume(dev);
	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		/* Power domain handling is automatic via PM framework */
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct udc_api udc_dwc3_api = {
	.lock = udc_dwc3_lock,
	.unlock = udc_dwc3_unlock,
	.device_speed = udc_dwc3_device_speed,
	.init = udc_dwc3_init,
	.enable = udc_dwc3_enable,
	.disable = udc_dwc3_disable,
	.shutdown = udc_dwc3_shutdown,
	.set_address = udc_dwc3_set_address,
	.host_wakeup = udc_dwc3_host_wakeup,
	.ep_try_config = NULL,
	.ep_enable = udc_dwc3_ep_activate,
	.ep_disable = udc_dwc3_ep_deactivate,
	.ep_set_halt = udc_dwc3_ep_set_halt,
	.ep_clear_halt = udc_dwc3_ep_clear_halt,
	.ep_enqueue = udc_dwc3_ep_enqueue,
	.ep_dequeue = udc_dwc3_ep_dequeue,
};

#if DT_INST_NUM_CLOCKS(0) > 1
#define UDC_DWC3_CLOCK_SUBSYS2(n) \
	.clock_subsys2 = (clock_control_subsys_t) DT_INST_CLOCKS_CELL_BY_IDX(n, 1, clkid),
#else
#define UDC_DWC3_CLOCK_SUBSYS2(n)
#endif

#define UDC_DWC3_DEVICE_DEFINE(n)						\
														\
	static struct udc_dwc3_data udc_dwc3_priv_##n = {	\
	};													\
														\
	static struct udc_data udc_data_##n = {				\
	.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),	\
	.priv = &udc_dwc3_priv_##n,							\
	};													\
														\
	static void udc_dwc3_thread_##n(void *dev, void *arg1, void *arg2)\
	{													\
		udc_dwc3_thread_handler(dev);					\
	}													\
														\
	K_THREAD_STACK_DEFINE(udc_dwc3_stack_##n, CONFIG_UDC_DWC3_STACK_SIZE);\
														\
	static void udc_dwc3_make_thread_##n(const struct device *dev)\
	{													\
		struct udc_dwc3_data *priv = udc_get_private(dev);\
														\
		k_thread_create(&priv->thread_data,				\
				udc_dwc3_stack_##n,						\
				K_THREAD_STACK_SIZEOF(udc_dwc3_stack_##n),\
				udc_dwc3_thread_##n,					\
				(void *)dev, NULL, NULL,				\
				K_PRIO_COOP(CONFIG_UDC_DWC3_THREAD_PRIORITY),\
				K_ESSENTIAL,							\
				K_NO_WAIT);								\
		k_thread_name_set(&priv->thread_data, dev->name);\
	}													\
														\
	static void udc_dwc3_irq_enable_func_##n(const struct device *dev)\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(n),					\
			DT_INST_IRQ(n, priority),					\
			udc_dwc3_isr_handler,						\
			DEVICE_DT_INST_GET(n),						\
			0);											\
														\
			irq_enable(DT_INST_IRQN(n));				\
	}													\
														\
	static void udc_dwc3_irq_disable_func_##n(const struct device *dev)\
	{													\
		irq_disable(DT_INST_IRQN(n));					\
	}													\
														\
	static struct udc_ep_config ep_cfg_in[DT_INST_PROP(n, num_in_eps)];\
	static struct udc_ep_config ep_cfg_out[DT_INST_PROP(n, num_out_eps)];\
														\
														\
	static const struct udc_dwc3_config udc_dwc3_cfg_##n  = {	\
	.num_out_eps = DT_INST_PROP(n, num_out_eps),			\
	.num_in_eps = DT_INST_PROP(n, num_in_eps),				\
	.ep_cfg_in = ep_cfg_in,									\
	.ep_cfg_out = ep_cfg_out,								\
	.max_speed = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),\
	.base = (udc_dwc3_reg_t *)DT_INST_REG_ADDR(n),			\
	.irq_enable_func = udc_dwc3_irq_enable_func_##n,		\
	.irq_disable_func = udc_dwc3_irq_disable_func_##n,		\
	.make_thread = udc_dwc3_make_thread_##n,				\
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),		\
	.clock_subsys = (clock_control_subsys_t) DT_INST_CLOCKS_CELL_BY_IDX(n, 0, clkid),\
	UDC_DWC3_CLOCK_SUBSYS2(n)							\
	};													\
								\
	PM_DEVICE_DT_INST_DEFINE(n, udc_dwc3_pm_action);	\
								\
	DEVICE_DT_INST_DEFINE(n, udc_dwc3_driver_preinit,	\
			PM_DEVICE_DT_INST_GET(n),		\
			&udc_data_##n, &udc_dwc3_cfg_##n,	\
			POST_KERNEL,				\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
			&udc_dwc3_api);\

DT_INST_FOREACH_STATUS_OKAY(UDC_DWC3_DEVICE_DEFINE)
