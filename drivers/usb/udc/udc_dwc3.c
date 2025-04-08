/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (C) 2025 Alif Semiconductor.
 */

#define DT_DRV_COMPAT snps_dwc3


#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/irq.h>

#include <zephyr/sys/util.h>

#include "udc_common.h"

#include <string.h>
#include <stdio.h>
#include <soc.h>
#include "usbd_dwc3.h"

LOG_MODULE_REGISTER(udc_dwc3, CONFIG_UDC_DRIVER_LOG_LEVEL);


struct udc_dwc3_data {
	USB_DRIVER  drv;
	const struct device *dev;
	uint32_t irq;
	struct k_thread thread_data;
	struct k_msgq dwc3_msgq_data;
};


struct udc_dwc3_config {
	uint8_t num_endpoints;
	uint16_t ep0_mps;
	uint16_t ep_mps;
	int speed_idx;
};

struct udc_dwc3_msg {
	uint8_t type;
	uint8_t ep;
	uint16_t recv_bytes;
};

enum udc_dwc3_msg_type {
	UDC_DWC3_MSG_SETUP,
	UDC_DWC3_MSG_DATA_OUT,
	UDC_DWC3_MSG_DATA_IN,
};

#define USB_ENDPOINT_NUMBER_MASK      0xF
#define EP_NUM(ep_addr)               (ep_addr & USB_ENDPOINT_NUMBER_MASK)

static struct udc_ep_config ep_cfg_in[4];
static struct udc_ep_config ep_cfg_out[4];


#define dwc3data(Drv) CONTAINER_OF(Drv, struct udc_dwc3_data, drv);
struct udc_dwc3_data __alif_ns_section udc0_dwc3_priv;

static char udc_dwc3_msgq_buf_0[CONFIG_UDC_DWC3_MAX_QMESSAGES * sizeof(struct udc_dwc3_msg)];

K_THREAD_STACK_DEFINE(udc_dwc3_stack_0, CONFIG_UDC_DWC3_STACK_SIZE);


void dwc3_reset_cb(USB_DRIVER *Drv)
{
	struct udc_dwc3_data *priv = dwc3data(Drv);

	udc_submit_event(priv->dev, UDC_EVT_RESET, 0);
}
void dwc3_connect_cb(USB_DRIVER *Drv)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	uint8_t ep_type;
	uint8_t ep_interval;
	int status;
	struct udc_ep_config *ep;

	struct udc_dwc3_data *priv = dwc3data(Drv);
	const struct device *dev = priv->dev;

	/* Re-Enable control endpoints */
	ep = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	if (ep && ep->stat.enabled) {
		ep_num = EP_NUM(ep->addr);
		ep_dir = (ep->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
		ep_type = ep->attributes & USB_EP_TRANSFER_TYPE_MASK;
		ep_interval = ep->interval;
		status =  usbd_dwc3_endpoint_create(&priv->drv, ep_type, ep_num,
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
		status =  usbd_dwc3_endpoint_create(&priv->drv, ep_type, ep_num,
				ep_dir, ep->mps, ep_interval);
		if (status != 0) {
			return;
		}
	}
	udc_submit_event(priv->dev, UDC_EVT_VBUS_READY, 0);
}

void dwc3_disconnect_cb(USB_DRIVER *Drv)
{
	struct udc_dwc3_data *priv = dwc3data(Drv);

	udc_submit_event(priv->dev, UDC_EVT_VBUS_REMOVED, 0);
}


void dwc3_setupstage_cb(USB_DRIVER *Drv)
{
	struct udc_dwc3_data *priv = dwc3data(Drv);
	struct udc_dwc3_msg msg = {.type = UDC_DWC3_MSG_SETUP,
								.recv_bytes = 4};
	int err;

	err = k_msgq_put(&priv->dwc3_msgq_data, &msg, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}

void dwc3_data_in_cb(USB_DRIVER *Drv, uint8_t ep_num)
{
	struct udc_dwc3_data *priv = dwc3data(Drv);
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

void dwc3_data_out_cb(USB_DRIVER *Drv, uint8_t ep_num)
{
	struct udc_dwc3_data *priv = dwc3data(Drv);
	struct udc_dwc3_msg msg = {
		.type = UDC_DWC3_MSG_DATA_OUT,
		.ep = ep_num,
		.recv_bytes = ep_num ? Drv->NumBytes : Drv->actual_length,
	};
	int err;

	err = k_msgq_put(&priv->dwc3_msgq_data, &msg, K_NO_WAIT);
	if (err != 0) {
		LOG_ERR("UDC Message queue overrun");
	}
}


static int udc_dwc3_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}


static int  udc_dwc3_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

int udc_dwc3_init(const struct device *dev)
{
	int  status;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	status = usbd_dwc3_initialize(&priv->drv);
	if (status) {
		LOG_ERR("USB controller initialization failed");
	}

	return status;
}

static int udc_dwc3_enable(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	int status;

	status = usbd_dwc3_connect(&priv->drv);
	if (status != 0) {
		LOG_ERR("USB controller enable failed");
	}
	status = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
			cfg->ep0_mps, 0);
	if (status) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_OUT);
		return status;
	}

	status = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
			cfg->ep0_mps, 0);
	if (status) {
		LOG_ERR("Failed enabling ep 0x%02x", USB_CONTROL_EP_IN);
		return status;
	}
	irq_enable(priv->irq);

	return status;
}

static int udc_dwc3_disable(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	usbd_dwc3_disconnect(&priv->drv);

	return 0;
}
static int udc_dwc3_shutdown(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);

	irq_disable(priv->irq);
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_DBG("Failed to disable control endpoint");
		return -EIO;
	}
	return 0;
}

static int udc_dwc3_set_address(const struct device *dev, const uint8_t addr)
{
	int status;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	status  = usbd_SetDeviceAddress(&priv->drv, addr);
	if (status != 0) {
		LOG_ERR("Failed to set the address");
	}
	return status;
}

static int udc_dwc3_host_wakeup(const struct device *dev)
{
	return 0;
}
static int udc_dwc3_ep_enable(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	uint8_t  ep_num;
	uint8_t ep_dir;
	uint8_t ep_type;
	uint8_t ep_interval;
	int status;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep_cfg->addr);
	ep_dir = (ep_cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	ep_type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	ep_interval = ep_cfg->interval;
	status =  usbd_dwc3_endpoint_create(&priv->drv, ep_type, ep_num, ep_dir,
			ep_cfg->mps, ep_interval);
	if (status != 0) {
		return status;
	} else {
		return status;
	}
}

static int udc_dwc3_ep_disable(const struct device *dev, struct udc_ep_config *ep)
{
	uint8_t  ep_num;
	int status;
	uint8_t ep_dir;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep->addr);
	ep_dir = (ep->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	status = usbd_dwc3_ep_disable(&priv->drv, ep_num, ep_dir);
	if (status != 0) {
		return status;
	} else {
		return status;
	}
}


static int udc_dwc3_ep_set_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	uint8_t  ep_num;
	int status;
	uint8_t ep_dir;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(cfg->addr);
	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	status = usbd_dwc3_ep_stall(&priv->drv, ep_num, ep_dir);
	if (status != 0) {
		return status;
	} else {
		return status;
	}
}

static int udc_dwc3_ep_clear_halt(const struct device *dev, struct udc_ep_config *cfg)
{
	uint8_t  ep_num;
	int status;
	uint8_t ep_dir;

	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(cfg->addr);
	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	status =  usbd_dwc3_ep_clearstall(&priv->drv, ep_num, ep_dir);
	if (status != 0) {
		return status;
	} else {
		return status;
	}
}

static int udc_dwc3_tx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	uint8_t *data; uint32_t len;
	uint8_t  ep_num, ep_dir;
	int ret;
	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep);
	ep_dir = (ep & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;

	data = buf->data;
	len = buf->len;
	buf->data += len;
	buf->len -= len;

	if ((data == NULL) && (len == 0) && (ep_dir != 0)) {
		return 0;
	}
	if (ep_num == 0) {
		ret = usbd_dwc3_ep0_send(&priv->drv, ep_num, ep_dir, data, len);
		if (ret != 0) {
			return ret;
		}
		udc_ep_set_busy(dev, ep, true);
	} else {
		ret = usbd_dwc3_bulk_send(&priv->drv, ep_num, ep_dir, data, len);
		if (ret != 0) {
			return ret;
		}
		udc_ep_set_busy(dev, ep, true);
	}

	return ret;
}
static int udc_dwc3_rx(const struct device *dev, uint8_t ep, struct net_buf *buf)
{
	uint8_t  ep_num, ep_dir;
	int ret;

	struct udc_dwc3_data *priv = udc_get_private(dev);

	ep_num = EP_NUM(ep);
	ep_dir = (ep & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;

	if (ep_num == 0) {
		ret = usbd_dwc3_ep0_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
		if (ret != 0) {
			return ret;
		}
		udc_ep_set_busy(dev, ep, true);
	} else {
		ret = usbd_dwc3_bulk_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
		if (ret != 0) {
			return ret;
		}
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

static void udc_dwc3_irq(const struct device *dev)
{
	const struct udc_dwc3_data *priv =  udc_get_private(dev);

	usbd_dwc3_interrupt_handler((USB_DRIVER *)&priv->drv);
}

static int usbd_ctrl_feed_dout(const struct device *dev, size_t length)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_ep_config *cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;
	uint8_t  ep_num, ep_dir;
	int ret;

	ep_num = EP_NUM(cfg->addr);

	ep_dir = (cfg->addr & USB_EP_DIR_MASK) ? USB_DIR_IN : USB_DIR_OUT;
	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}
	k_fifo_put(&cfg->fifo, buf);
	ret = usbd_dwc3_ep0_recv(&priv->drv, ep_num, ep_dir, buf->data, buf->size);
	if (ret != 0) {
		return ret;
	}

	return ret;
}
static void handle_setup_pkt(struct udc_dwc3_data *priv)
{
	struct usb_setup_packet *setup = (struct usb_setup_packet *)&priv->drv.SetupData;
	const struct device *dev = priv->dev;
	struct net_buf *buf;
	int err;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, sizeof(struct usb_setup_packet));
	if (buf == NULL) {
		LOG_ERR("Failed to allocate buf for setup");
		return;
	}
	udc_ep_buf_set_setup(buf);
	memcpy(buf->data, setup, 8);
	net_buf_add(buf, 8);

	udc_ctrl_update_stage(dev, buf);

	if (!buf->len) {
		return;
	}

	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		err = usbd_ctrl_feed_dout(dev, udc_data_stage_length(buf));
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
	if (buf) {
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
static void udc_dwc3_thread_handler(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct udc_dwc3_data *priv = udc_get_private(dev);
	struct udc_dwc3_msg msg;

	while (true) {
		k_msgq_get(&priv->dwc3_msgq_data, &msg, K_FOREVER);
		switch (msg.type) {
		case UDC_DWC3_MSG_SETUP:
			handle_setup_pkt(priv);
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

static int udc_dwc3_driver_init0(const struct device *dev)
{
	struct udc_dwc3_data *priv = udc_get_private(dev);
	const struct udc_dwc3_config *cfg = dev->config;
	struct udc_data *data = dev->data;
	int err;

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;
	if (cfg->speed_idx == 2) {
		data->caps.hs = true;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(ep_cfg_out); i++) {
		ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			ep_cfg_out[i].caps.control = 1;
			ep_cfg_out[i].caps.mps = cfg->ep0_mps;
		} else {
			ep_cfg_out[i].caps.bulk = 1;
			ep_cfg_out[i].caps.interrupt = 1;
			ep_cfg_out[i].caps.iso = 1;
			ep_cfg_out[i].caps.mps = cfg->ep_mps;
		}

		ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(ep_cfg_in); i++) {
		ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			ep_cfg_in[i].caps.control = 1;
			ep_cfg_in[i].caps.mps = cfg->ep0_mps;
		} else {
			ep_cfg_in[i].caps.bulk = 1;
			ep_cfg_in[i].caps.interrupt = 1;
			ep_cfg_in[i].caps.iso = 1;
			ep_cfg_in[i].caps.mps = 1023;
		}

		ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	priv->dev = dev;
	priv->irq = USB0_IRQ;
	k_msgq_init(&priv->dwc3_msgq_data, udc_dwc3_msgq_buf_0, sizeof(struct udc_dwc3_msg),
			CONFIG_UDC_DWC3_MAX_QMESSAGES);

	k_thread_create(&priv->thread_data, udc_dwc3_stack_0,
			K_THREAD_STACK_SIZEOF(udc_dwc3_stack_0), udc_dwc3_thread_handler,
			(void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_UDC_DWC3_THREAD_PRIORITY),
			K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(&priv->thread_data, dev->name);
	IRQ_CONNECT(USB0_IRQ, 3, udc_dwc3_irq, DEVICE_DT_INST_GET(0), 0);

	return 0;

}

static const struct udc_api udc_dwc3_api = {
	.lock = udc_dwc3_lock,
	.unlock = udc_dwc3_unlock,
	.init = udc_dwc3_init,
	.enable = udc_dwc3_enable,
	.disable = udc_dwc3_disable,
	.shutdown = udc_dwc3_shutdown,
	.set_address = udc_dwc3_set_address,
	.host_wakeup = udc_dwc3_host_wakeup,
	.ep_try_config = NULL,
	.ep_enable = udc_dwc3_ep_enable,
	.ep_disable = udc_dwc3_ep_disable,
	.ep_set_halt = udc_dwc3_ep_set_halt,
	.ep_clear_halt = udc_dwc3_ep_clear_halt,
	.ep_enqueue = udc_dwc3_ep_enqueue,
	.ep_dequeue = udc_dwc3_ep_dequeue,
};


static struct udc_data udc0_dwc3_data = {
		.mutex = Z_MUTEX_INITIALIZER(udc0_dwc3_data.mutex),
		.priv = &udc0_dwc3_priv,
	};


static const struct udc_dwc3_config udc0_dwc3_cfg  = {
		.num_endpoints = 8,
		.ep0_mps = 64,
		.ep_mps = 512,
		.speed_idx = 2,

};

DEVICE_DT_INST_DEFINE(0, udc_dwc3_driver_init0, NULL, &udc0_dwc3_data, &udc0_dwc3_cfg,
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_dwc3_api);
