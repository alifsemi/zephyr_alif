/* h4.c - H:4 UART based Bluetooth driver */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(alif_bt_driver);

#include "common/bt_str.h"

#include "../util.h"

#include "es0_power_manager.h"

#define DT_DRV_COMPAT alif_bt_hci_uart

/* Stop flag to signal rx thread */
static atomic_t stop_flag = ATOMIC_INIT(0);

/* hci_alif_rx_thread id */
static k_tid_t tid;

struct hci_alif_data {
	struct {
		struct net_buf *buf;
		struct k_fifo   fifo;

		uint16_t        remaining;
		uint16_t        discard;

		bool            have_hdr;
		bool            discardable;

		uint8_t         hdr_len;

		uint8_t         type;
		union {
			struct bt_hci_evt_hdr evt;
			struct bt_hci_acl_hdr acl;
			struct bt_hci_iso_hdr iso;
			uint8_t hdr[4];
		};
	} rx;

	struct {
		uint8_t         type;
		struct net_buf *buf;
		struct k_fifo   fifo;
	} tx;

	bt_hci_recv_t recv;
};

struct hci_alif_config {
	const struct device *uart;
	k_thread_stack_t *rx_thread_stack;
	size_t rx_thread_stack_size;
	struct k_thread *rx_thread;
};

static inline void hci_alif_get_type(const struct device *dev)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;

	/* Get packet type */
	if (uart_fifo_read(cfg->uart, &h4->rx.type, 1) != 1) {
		LOG_WRN("Unable to read H:4 packet type");
		h4->rx.type = BT_HCI_H4_NONE;
		return;
	}

	switch (h4->rx.type) {
	case BT_HCI_H4_EVT:
		h4->rx.remaining = sizeof(h4->rx.evt);
		h4->rx.hdr_len = h4->rx.remaining;
		break;
	case BT_HCI_H4_ACL:
		h4->rx.remaining = sizeof(h4->rx.acl);
		h4->rx.hdr_len = h4->rx.remaining;
		break;
	case BT_HCI_H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			h4->rx.remaining = sizeof(h4->rx.iso);
			h4->rx.hdr_len = h4->rx.remaining;
			break;
		}
		__fallthrough;
	default:
		LOG_ERR("Unknown H:4 type 0x%02x", h4->rx.type);
		h4->rx.type = BT_HCI_H4_NONE;
	}
}

static void hci_alif_read_hdr(const struct device *dev)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;
	int bytes_read = h4->rx.hdr_len - h4->rx.remaining;
	int ret;

	ret = uart_fifo_read(cfg->uart, h4->rx.hdr + bytes_read, h4->rx.remaining);
	if (unlikely(ret < 0)) {
		LOG_ERR("Unable to read from UART (ret %d)", ret);
	} else {
		h4->rx.remaining -= ret;
	}
}

static inline void get_acl_hdr(const struct device *dev)
{
	struct hci_alif_data *h4 = dev->data;

	hci_alif_read_hdr(dev);

	if (!h4->rx.remaining) {
		struct bt_hci_acl_hdr *hdr = &h4->rx.acl;

		h4->rx.remaining = sys_le16_to_cpu(hdr->len);
		LOG_DBG("Got ACL header. Payload %u bytes", h4->rx.remaining);
		h4->rx.have_hdr = true;
	}
}

static inline void get_iso_hdr(const struct device *dev)
{
	struct hci_alif_data *h4 = dev->data;

	hci_alif_read_hdr(dev);

	if (!h4->rx.remaining) {
		struct bt_hci_iso_hdr *hdr = &h4->rx.iso;

		h4->rx.remaining = bt_iso_hdr_len(sys_le16_to_cpu(hdr->len));
		LOG_DBG("Got ISO header. Payload %u bytes", h4->rx.remaining);
		h4->rx.have_hdr = true;
	}
}

static inline void get_evt_hdr(const struct device *dev)
{
	struct hci_alif_data *h4 = dev->data;

	struct bt_hci_evt_hdr *hdr = &h4->rx.evt;

	hci_alif_read_hdr(dev);

	if (h4->rx.hdr_len == sizeof(*hdr) && h4->rx.remaining < sizeof(*hdr)) {
		switch (h4->rx.evt.evt) {
		case BT_HCI_EVT_LE_META_EVENT:
			h4->rx.remaining++;
			h4->rx.hdr_len++;
			break;
#if defined(CONFIG_BT_CLASSIC)
		case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
		case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
			h4->rx.discardable = true;
			break;
#endif
		}
	}

	if (!h4->rx.remaining) {
		if (h4->rx.evt.evt == BT_HCI_EVT_LE_META_EVENT &&
		    ((h4->rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_ADVERTISING_REPORT) ||
		    (h4->rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT))) {
			LOG_DBG("Marking adv report as discardable");
			h4->rx.discardable = true;
		}

		h4->rx.remaining = hdr->len - (h4->rx.hdr_len - sizeof(*hdr));
		LOG_DBG("Got event header. Payload %u bytes", hdr->len);
		h4->rx.have_hdr = true;
	}
}


static inline void copy_hdr(struct hci_alif_data *h4)
{
	net_buf_add_mem(h4->rx.buf, h4->rx.hdr, h4->rx.hdr_len);
}

static void reset_rx(struct hci_alif_data *h4)
{
	h4->rx.type = BT_HCI_H4_NONE;
	h4->rx.remaining = 0U;
	h4->rx.have_hdr = false;
	h4->rx.hdr_len = 0U;
	h4->rx.discardable = false;
}

static struct net_buf *get_rx(struct hci_alif_data *h4, k_timeout_t timeout)
{
	LOG_DBG("type 0x%02x, evt 0x%02x", h4->rx.type, h4->rx.evt.evt);

	switch (h4->rx.type) {
	case BT_HCI_H4_EVT:
		return bt_buf_get_evt(h4->rx.evt.evt, h4->rx.discardable, timeout);
	case BT_HCI_H4_ACL:
		return bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
	case BT_HCI_H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			return bt_buf_get_rx(BT_BUF_ISO_IN, timeout);
		}
	}

	return NULL;
}

static void hci_alif_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;
	struct net_buf *buf;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("started");

	while (!atomic_get(&stop_flag)) {
		LOG_DBG("rx.buf %p", h4->rx.buf);

		/* We can only do the allocation if we know the initial
		 * header, since Command Complete/Status events must use the
		 * original command buffer (if available).
		 */
		if (h4->rx.have_hdr && !h4->rx.buf) {
			h4->rx.buf = get_rx(h4, K_FOREVER);
			LOG_DBG("Got rx.buf %p", h4->rx.buf);
			if (h4->rx.remaining > net_buf_tailroom(h4->rx.buf)) {
				LOG_ERR("Not enough space in buffer");
				h4->rx.discard = h4->rx.remaining;
				reset_rx(h4);
			} else {
				copy_hdr(h4);
			}
		}

		/* Let the ISR continue receiving new packets */
		uart_irq_rx_enable(cfg->uart);

		buf = k_fifo_get(&h4->rx.fifo, K_FOREVER);
		do {
			uart_irq_rx_enable(cfg->uart);

			LOG_DBG("Calling bt_recv(%p)", buf);
			h4->recv(dev, buf);

			/* Give other threads a chance to run if the ISR
			 * is receiving data so fast that rx.fifo never
			 * or very rarely goes empty.
			 */
			k_yield();

			uart_irq_rx_disable(cfg->uart);
			buf = k_fifo_get(&h4->rx.fifo, K_NO_WAIT);
		} while (buf);
	}

	LOG_DBG("Ending HCI Alif RX Thread");
}

static size_t hci_alif_discard(const struct device *uart, size_t len)
{
	uint8_t buf[33];
	int err;

	err = uart_fifo_read(uart, buf, MIN(len, sizeof(buf)));
	if (unlikely(err < 0)) {
		LOG_ERR("Unable to read from UART (err %d)", err);
		return 0;
	}

	return err;
}

static inline void read_payload(const struct device *dev)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;
	struct net_buf *buf;
	int read;

	if (!h4->rx.buf) {
		size_t buf_tailroom;

		h4->rx.buf = get_rx(h4, K_NO_WAIT);
		if (!h4->rx.buf) {
			if (h4->rx.discardable) {
				LOG_WRN("Discarding event 0x%02x", h4->rx.evt.evt);
				h4->rx.discard = h4->rx.remaining;
				reset_rx(h4);
				return;
			}

			LOG_WRN("Failed to allocate, deferring to rx_thread");
			uart_irq_rx_disable(cfg->uart);
			return;
		}

		LOG_DBG("Allocated rx.buf %p", h4->rx.buf);

		buf_tailroom = net_buf_tailroom(h4->rx.buf);
		if (buf_tailroom < h4->rx.remaining) {
			LOG_ERR("Not enough space in buffer %u/%zu", h4->rx.remaining,
				buf_tailroom);
			h4->rx.discard = h4->rx.remaining;
			reset_rx(h4);
			return;
		}

		copy_hdr(h4);
	}

	read = uart_fifo_read(cfg->uart, net_buf_tail(h4->rx.buf), h4->rx.remaining);
	if (unlikely(read < 0)) {
		LOG_ERR("Failed to read UART (err %d)", read);
		return;
	}

	net_buf_add(h4->rx.buf, read);
	h4->rx.remaining -= read;

	LOG_DBG("got %d bytes, remaining %u", read, h4->rx.remaining);
	LOG_DBG("Payload (len %u): %s", h4->rx.buf->len,
		bt_hex(h4->rx.buf->data, h4->rx.buf->len));

	if (h4->rx.remaining) {
		return;
	}

	buf = h4->rx.buf;
	h4->rx.buf = NULL;

	if (h4->rx.type == BT_HCI_H4_EVT) {
		bt_buf_set_type(buf, BT_BUF_EVT);
	} else {
		bt_buf_set_type(buf, BT_BUF_ACL_IN);
	}

	reset_rx(h4);

	LOG_DBG("Putting buf %p to rx fifo", buf);
	k_fifo_put(&h4->rx.fifo, buf);
}

static inline void read_header(const struct device *dev)
{
	struct hci_alif_data *h4 = dev->data;

	switch (h4->rx.type) {
	case BT_HCI_H4_NONE:
		hci_alif_get_type(dev);
		return;
	case BT_HCI_H4_EVT:
		get_evt_hdr(dev);
		break;
	case BT_HCI_H4_ACL:
		get_acl_hdr(dev);
		break;
	case BT_HCI_H4_ISO:
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			get_iso_hdr(dev);
			break;
		}
		__fallthrough;
	default:
		CODE_UNREACHABLE;
		return;
	}

	if (h4->rx.have_hdr && h4->rx.buf) {
		if (h4->rx.remaining > net_buf_tailroom(h4->rx.buf)) {
			LOG_ERR("Not enough space in buffer");
			h4->rx.discard = h4->rx.remaining;
			reset_rx(h4);
		} else {
			copy_hdr(h4);
		}
	}
}

static inline void process_tx(const struct device *dev)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;
	int bytes;

	if (!h4->tx.buf) {
		h4->tx.buf = k_fifo_get(&h4->tx.fifo, K_NO_WAIT);
		if (!h4->tx.buf) {
			LOG_ERR("TX interrupt but no pending buffer!");
			uart_irq_tx_disable(cfg->uart);
			return;
		}
	}

	if (!h4->tx.type) {
		switch (bt_buf_get_type(h4->tx.buf)) {
		case BT_BUF_ACL_OUT:
			h4->tx.type = BT_HCI_H4_ACL;
			break;
		case BT_BUF_CMD:
			h4->tx.type = BT_HCI_H4_CMD;
			break;
		case BT_BUF_ISO_OUT:
			if (IS_ENABLED(CONFIG_BT_ISO)) {
				h4->tx.type = BT_HCI_H4_ISO;
				break;
			}
			__fallthrough;
		default:
			LOG_ERR("Unknown buffer type");
			goto done;
		}

		bytes = uart_fifo_fill(cfg->uart, &h4->tx.type, 1);
		if (bytes != 1) {
			LOG_WRN("Unable to send H:4 type");
			h4->tx.type = BT_HCI_H4_NONE;
			return;
		}
	}

	bytes = uart_fifo_fill(cfg->uart, h4->tx.buf->data, h4->tx.buf->len);
	if (unlikely(bytes < 0)) {
		LOG_ERR("Unable to write to UART (err %d)", bytes);
	} else {
		net_buf_pull(h4->tx.buf, bytes);
	}

	if (h4->tx.buf->len) {
		return;
	}

done:
	h4->tx.type = BT_HCI_H4_NONE;
	net_buf_unref(h4->tx.buf);
	h4->tx.buf = k_fifo_get(&h4->tx.fifo, K_NO_WAIT);
	if (!h4->tx.buf) {
		uart_irq_tx_disable(cfg->uart);
	}
}

static inline void process_rx(const struct device *dev)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;

	LOG_DBG("remaining %u discard %u have_hdr %u rx.buf %p len %u",
		h4->rx.remaining, h4->rx.discard, h4->rx.have_hdr, h4->rx.buf,
		h4->rx.buf ? h4->rx.buf->len : 0);

	if (h4->rx.discard) {
		h4->rx.discard -= hci_alif_discard(cfg->uart, h4->rx.discard);
		return;
	}

	if (h4->rx.have_hdr) {
		read_payload(dev);
	} else {
		read_header(dev);
	}
}

static void bt_uart_isr(const struct device *uart, void *user_data)
{
	struct device *dev = user_data;

	while (uart_irq_update(uart) && uart_irq_is_pending(uart)) {
		if (uart_irq_tx_ready(uart)) {
			process_tx(dev);
		}

		if (uart_irq_rx_ready(uart)) {
			process_rx(dev);
		}
	}
}

static int hci_alif_send(const struct device *dev, struct net_buf *buf)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *h4 = dev->data;

	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	/* Deassert&assert rts_n, falling edge triggers wake up the RF core */

	wake_es0(cfg->uart);

	k_fifo_put(&h4->tx.fifo, buf);
	uart_irq_tx_enable(cfg->uart);

	return 0;
}

/** Setup the HCI transport, which usually means to reset the Bluetooth IC
  *
  * @param dev The device structure for the bus connecting to the IC
  *
  * @return 0 on success, negative error value on failure
  */
int __weak bt_hci_transport_setup(const struct device *uart)
{
	hci_alif_discard(uart, 32);
	return 0;
}

static int hci_alif_close(const struct device *dev)
{
	int err;
	struct hci_alif_data *hci = dev->data;
	const struct hci_alif_config *cfg = dev->config;

	/* Signal the BLE thread to exit the loop */
	atomic_set(&stop_flag, 1);
	/* Call k_thread_join() to ensure the rx thread is fully terminated */
	err = k_thread_join(cfg->rx_thread, K_MSEC(50));

	if (err) {

		/* Aborting thread */
		k_thread_abort(tid);

		err = k_thread_join(cfg->rx_thread, K_FOREVER);
		if (err) {
			LOG_ERR("Error aborting hci_alif_rx_thread");
			return err;
		}
	}


	err = stop_using_es0();

	if (-2 == err) {
		return -EIO;
	}

	hci->recv = NULL;

	return 0;
}

static int hci_alif_open(const struct device *dev, bt_hci_recv_t recv)
{
	const struct hci_alif_config *cfg = dev->config;
	struct hci_alif_data *data = dev->data;
	int ret;

	/* Clear the stop_flag to allow entering the hci_alif_rx_thread loop */
	atomic_clear(&stop_flag);

	ret = take_es0_into_use();
	if (ret < 0) {
		return -EIO;
	}

	uart_irq_rx_disable(cfg->uart);
	uart_irq_tx_disable(cfg->uart);

	ret = bt_hci_transport_setup(cfg->uart);
	if (ret < 0) {
		return -EIO;
	}

	data->recv = recv;

	uart_irq_callback_user_data_set(cfg->uart, bt_uart_isr, (void *)dev);

	tid = k_thread_create(cfg->rx_thread, cfg->rx_thread_stack,
			      cfg->rx_thread_stack_size,
			      hci_alif_rx_thread, (void *)dev, NULL, NULL,
			      K_PRIO_COOP(CONFIG_BT_RX_PRIO),
			      0, K_NO_WAIT);
	k_thread_name_set(tid, "hci_alif_rx_thread");

	return 0;
}

static DEVICE_API(bt_hci, hci_alif_driver_api) = {
	.open  	= hci_alif_open,
	.send  	= hci_alif_send,
	.close 	= hci_alif_close,
};

#define BT_ALIF_UART_DEVICE_INIT(inst) \
	static K_KERNEL_STACK_DEFINE(alif_rx_thread_stack_##inst, CONFIG_BT_DRV_RX_STACK_SIZE); \
	static struct k_thread alif_rx_thread_##inst; \
	static const struct hci_alif_config hci_alif_config_##inst = { \
		.uart = DEVICE_DT_GET(DT_INST_PARENT(inst)), \
		.rx_thread_stack = alif_rx_thread_stack_##inst, \
		.rx_thread_stack_size = K_KERNEL_STACK_SIZEOF(alif_rx_thread_stack_##inst), \
		.rx_thread = &alif_rx_thread_##inst, \
	}; \
	static struct hci_alif_data hci_alif_data_##inst = { \
		.rx = { \
			.fifo = Z_FIFO_INITIALIZER(hci_alif_data_##inst.rx.fifo), \
		}, \
		.tx = { \
			.fifo = Z_FIFO_INITIALIZER(hci_alif_data_##inst.tx.fifo), \
		}, \
	}; \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &hci_alif_data_##inst, &hci_alif_config_##inst, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &hci_alif_driver_api)

DT_INST_FOREACH_STATUS_OKAY(BT_ALIF_UART_DEVICE_INIT)
