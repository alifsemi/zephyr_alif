/* h4.c - H:4 UART based Bluetooth driver */

/*
 * Copyright (c) 2024 Alif Semiconductor
 * Copyright (c) 2015-2016 Intel Corporation
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
#include <zephyr/drivers/bluetooth/hci_driver.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

#include "common/bt_str.h"

#include "../util.h"

#include "es0_power_manager.h"
#include "dma_event_router.h"
#include <zephyr/sys/ring_buffer.h>

#define H4_NONE 0x00
#define H4_CMD  0x01
#define H4_ACL  0x02
#define H4_SCO  0x03
#define H4_EVT  0x04
#define H4_ISO  0x05

/* Define appropriate timeouts */
#define TX_TIMEOUT_US 200 /* 200us */
#define RX_TIMEOUT_US 0   /* No delay */

/* UART DMA request numbers from board-specific overlay */
#define DMA_UART_TX_GROUP 0 /* DMA group for UART TX */
#define DMA_UART_RX_GROUP 0 /* DMA group for UART RX */

static K_KERNEL_STACK_DEFINE(alif_rx_thread_stack, CONFIG_BT_RX_STACK_SIZE);
static struct k_thread alif_rx_thread_data;

/* Disable RX dma because it not work optimal way */
#define ALIF_HCI_DMA_RX_ENABLED 0

#if ALIF_HCI_DMA_RX_ENABLED
static bool pingpong;
#endif
#define BUFF_SIZE 32
static uint8_t temp_rx_buf[64];

/* Wait for specific message from HCI */
static K_SEM_DEFINE(hci_tx_sem, 0, 1);

#define MY_RING_BUF_BYTES 256
RING_BUF_DECLARE(hci_ring_buf, MY_RING_BUF_BYTES);

static struct {
	struct net_buf *buf;
	struct k_fifo fifo;

	uint16_t remaining;
	uint16_t discard;

	bool have_hdr;
	bool discardable;

	uint8_t hdr_len;

	uint8_t type;
	union {
		struct bt_hci_evt_hdr evt;
		struct bt_hci_acl_hdr acl;
		struct bt_hci_iso_hdr iso;
		uint8_t hdr[4];
	};
} rx = {
	.fifo = Z_FIFO_INITIALIZER(rx.fifo),
};

static struct {
	uint8_t type;
	struct net_buf *buf;
	struct k_fifo fifo;
} tx = {
	.fifo = Z_FIFO_INITIALIZER(tx.fifo),
};

static bool hci_uart_dma_rx_enabled;
static bool hci_uart_dma_tx_enabled;
/* Static tx buffer */
static uint8_t hci_tx_buf[256];

static const struct device *const hci_alif_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_uart));

static inline void hci_alif_get_type(void)
{
	/* Get packet type */
	int read = ring_buf_get(&hci_ring_buf, &rx.type, 1);

	if (read != 1) {
		rx.type = H4_NONE;
		return;
	}

	switch (rx.type) {
	case H4_EVT:
		rx.remaining = sizeof(rx.evt);
		rx.hdr_len = rx.remaining;
		break;
	case H4_ACL:
		rx.remaining = sizeof(rx.acl);
		rx.hdr_len = rx.remaining;
		break;
	case H4_ISO:
		/* TODO: Figure out a way to use shared memory for ISO */
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			rx.remaining = sizeof(rx.iso);
			rx.hdr_len = rx.remaining;
			break;
		}
		__fallthrough;
	default:
		LOG_ERR("Unknown H:4 type 0x%02x", rx.type);
		rx.type = H4_NONE;
	}
}

static void hci_alif_read_hdr(void)
{
	int bytes_read = rx.hdr_len - rx.remaining;
	int ret;

	ret = ring_buf_get(&hci_ring_buf, rx.hdr + bytes_read, rx.remaining);

	if (unlikely(ret < 0)) {
		LOG_ERR("Unable to read from UART (ret %d)", ret);
	} else {
		rx.remaining -= ret;
	}
}

static inline void get_acl_hdr(void)
{
	hci_alif_read_hdr();

	if (!rx.remaining) {
		struct bt_hci_acl_hdr *hdr = &rx.acl;

		rx.remaining = sys_le16_to_cpu(hdr->len);
		LOG_DBG("Got ACL header. Payload %u bytes", rx.remaining);
		rx.have_hdr = true;
	}
}

static inline void get_iso_hdr(void)
{
	hci_alif_read_hdr();

	if (!rx.remaining) {
		struct bt_hci_iso_hdr *hdr = &rx.iso;

		rx.remaining = bt_iso_hdr_len(sys_le16_to_cpu(hdr->len));
		LOG_DBG("Got ISO header. Payload %u bytes", rx.remaining);
		rx.have_hdr = true;
	}
}

static inline void get_evt_hdr(void)
{
	struct bt_hci_evt_hdr *hdr = &rx.evt;

	hci_alif_read_hdr();

	if (rx.hdr_len == sizeof(*hdr) && rx.remaining < sizeof(*hdr)) {
		switch (rx.evt.evt) {
		case BT_HCI_EVT_LE_META_EVENT:
			rx.remaining++;
			rx.hdr_len++;
			break;
#if defined(CONFIG_BT_BREDR)
		case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
		case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
			rx.discardable = true;
			break;
#endif
		}
	}

	if (!rx.remaining) {
		if (rx.evt.evt == BT_HCI_EVT_LE_META_EVENT &&
		    ((rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_ADVERTISING_REPORT) ||
		     (rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT))) {
			LOG_DBG("Marking adv report as discardable");
			rx.discardable = true;
		}

		rx.remaining = hdr->len - (rx.hdr_len - sizeof(*hdr));
		LOG_DBG("Got event header. Payload %u bytes", hdr->len);
		rx.have_hdr = true;
	}
}

static inline void copy_hdr(struct net_buf *buf)
{
	net_buf_add_mem(buf, rx.hdr, rx.hdr_len);
}

static void reset_rx(void)
{
	rx.type = H4_NONE;
	rx.remaining = 0U;
	rx.have_hdr = false;
	rx.hdr_len = 0U;
	rx.discardable = false;
}

static struct net_buf *get_rx(k_timeout_t timeout)
{
	LOG_DBG("type 0x%02x, evt 0x%02x", rx.type, rx.evt.evt);

	switch (rx.type) {
	case H4_EVT:
		return bt_buf_get_evt(rx.evt.evt, rx.discardable, timeout);
	case H4_ACL:
		return bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
	case H4_ISO:
		/* TODO: Figure out a way to use shared memory for ISO */
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			return bt_buf_get_rx(BT_BUF_ISO_IN, timeout);
		}
	}

	return NULL;
}

static void hci_alif_rx_thread(void *p1, void *p2, void *p3)
{
	struct net_buf *buf;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_DBG("started");

	while (1) {
		LOG_DBG("rx.buf %p", rx.buf);

		/* We can only do the allocation if we know the initial
		 * header, since Command Complete/Status events must use the
		 * original command buffer (if available).
		 */
		if (rx.have_hdr && !rx.buf) {
			rx.buf = get_rx(K_FOREVER);
			LOG_DBG("Got rx.buf %p", rx.buf);
			if (rx.remaining > net_buf_tailroom(rx.buf)) {
				LOG_ERR("Not enough space in buffer");
				rx.discard = rx.remaining;
				reset_rx();
			} else {
				copy_hdr(rx.buf);
			}
		}
		if (!hci_uart_dma_rx_enabled) {
			/* Let the ISR continue receiving new packets */
			uart_irq_rx_enable(hci_alif_dev);
		}

		buf = net_buf_get(&rx.fifo, K_FOREVER);
		do {
			if (!hci_uart_dma_rx_enabled) {
				uart_irq_rx_enable(hci_alif_dev);
			}

			LOG_DBG("Calling bt_recv(%p)", buf);
			bt_recv(buf);

			if (!hci_uart_dma_rx_enabled) {
				/* Give other threads a chance to run if the ISR
				 * is receiving data so fast that rx.fifo never
				 * or very rarely goes empty.
				 */
				k_yield();
				uart_irq_rx_disable(hci_alif_dev);
			}
			buf = net_buf_get(&rx.fifo, K_NO_WAIT);
		} while (buf);
	}
}

static size_t hci_alif_discard(const struct device *uart, size_t len)
{
	uint8_t buf[33];
	int err;

	err = ring_buf_get(&hci_ring_buf, buf, MIN(len, sizeof(buf)));

	if (unlikely(err < 0)) {
		LOG_ERR("Unable to read from UART (err %d)", err);
		return 0;
	}

	return err;
}

static inline void read_payload(void)
{
	struct net_buf *buf;
	uint8_t evt_flags;
	int read;

	if (!rx.buf) {
		size_t buf_tailroom;

		rx.buf = get_rx(K_NO_WAIT);
		if (!rx.buf) {
			if (rx.discardable) {
				LOG_DBG("Discarding event 0x%02x", rx.evt.evt);
				rx.discard = rx.remaining;
				reset_rx();
				return;
			}

			LOG_WRN("Failed to allocate, deferring to hci_alif_rx_thread");
			uart_irq_rx_disable(hci_alif_dev);
			return;
		}

		LOG_DBG("Allocated rx.buf %p", rx.buf);

		buf_tailroom = net_buf_tailroom(rx.buf);
		if (buf_tailroom < rx.remaining) {
			LOG_ERR("Not enough space in buffer %u/%zu", rx.remaining, buf_tailroom);
			rx.discard = rx.remaining;
			reset_rx();
			return;
		}

		copy_hdr(rx.buf);
	}

	read = ring_buf_get(&hci_ring_buf, net_buf_tail(rx.buf), rx.remaining);

	if (unlikely(read < 0)) {
		LOG_ERR("Failed to read UART (err %d)", read);
		return;
	}

	net_buf_add(rx.buf, read);
	rx.remaining -= read;

	LOG_DBG("got %d bytes, remaining %u", read, rx.remaining);
	LOG_DBG("Payload (len %u): %s", rx.buf->len, bt_hex(rx.buf->data, rx.buf->len));

	if (rx.remaining) {
		return;
	}

	buf = rx.buf;
	rx.buf = NULL;

	if (rx.type == H4_EVT) {
		evt_flags = bt_hci_evt_get_flags(rx.evt.evt);
		bt_buf_set_type(buf, BT_BUF_EVT);
	} else {
		evt_flags = BT_HCI_EVT_FLAG_RECV;
		bt_buf_set_type(buf, BT_BUF_ACL_IN);
	}

	reset_rx();

	if (IS_ENABLED(CONFIG_BT_RECV_BLOCKING) && (evt_flags & BT_HCI_EVT_FLAG_RECV_PRIO)) {
		LOG_DBG("Calling bt_recv_prio(%p)", buf);
		bt_recv_prio(buf);
	}

	if (evt_flags & BT_HCI_EVT_FLAG_RECV) {
		LOG_DBG("Putting buf %p to rx fifo", buf);
		net_buf_put(&rx.fifo, buf);
	}
}

static inline void read_header(void)
{
	switch (rx.type) {
	case H4_NONE:
		hci_alif_get_type();
		return;
	case H4_EVT:
		get_evt_hdr();
		break;
	case H4_ACL:
		get_acl_hdr();
		break;
	case H4_ISO:
		/* TODO: Figure out a way to use shared memory for ISO */
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			get_iso_hdr();
			break;
		}
		__fallthrough;
	default:
		CODE_UNREACHABLE;
		return;
	}

	if (rx.have_hdr && rx.buf) {
		if (rx.remaining > net_buf_tailroom(rx.buf)) {
			LOG_ERR("Not enough space in buffer");
			rx.discard = rx.remaining;
			reset_rx();
		} else {
			copy_hdr(rx.buf);
		}
	}
}

static inline void process_tx(void)
{
	int bytes;

	if (!tx.buf) {
		tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
		if (!tx.buf) {
			LOG_ERR("TX interrupt but no pending buffer!");
			uart_irq_tx_disable(hci_alif_dev);
			return;
		}
	}

	if (!tx.type) {
		switch (bt_buf_get_type(tx.buf)) {
		case BT_BUF_ACL_OUT:
			tx.type = H4_ACL;
			break;
		case BT_BUF_CMD:
			tx.type = H4_CMD;
			break;
		case BT_BUF_ISO_OUT:
			/* TODO: Figure out a way to use shared memory for ISO */
			if (IS_ENABLED(CONFIG_BT_ISO)) {
				tx.type = H4_ISO;
				break;
			}
			__fallthrough;
		default:
			LOG_ERR("Unknown buffer type");
			goto done;
		}

		bytes = uart_fifo_fill(hci_alif_dev, &tx.type, 1);
		if (bytes != 1) {
			LOG_WRN("Unable to send H:4 type");
			tx.type = H4_NONE;
			return;
		}
	}

	bytes = uart_fifo_fill(hci_alif_dev, tx.buf->data, tx.buf->len);
	if (unlikely(bytes < 0)) {
		LOG_ERR("Unable to write to UART (err %d)", bytes);
	} else {
		net_buf_pull(tx.buf, bytes);
	}

	if (tx.buf->len) {
		return;
	}

done:
	tx.type = H4_NONE;
	net_buf_unref(tx.buf);
	tx.buf = net_buf_get(&tx.fifo, K_NO_WAIT);
	if (!tx.buf) {
		uart_irq_tx_disable(hci_alif_dev);
	}
}

static inline void process_dma_tx(struct net_buf *buf)
{
	int length, copy_length;
	bool first_part = true;
	uint8_t tx_type;

	switch (bt_buf_get_type(buf)) {
	case BT_BUF_ACL_OUT:
		tx_type = H4_ACL;
		break;
	case BT_BUF_CMD:
		tx_type = H4_CMD;
		break;
	case BT_BUF_ISO_OUT:
		/* TODO: Figure out a way to use shared memory for ISO */
		if (IS_ENABLED(CONFIG_BT_ISO)) {
			tx_type = H4_ISO;
			break;
		}
		__fallthrough;
	default:
		LOG_ERR("Unknown buffer type");
		goto done;
	}

	length = buf->len;
	while (length) {
		if (first_part) {
			copy_length = length;
			if (copy_length > 255) {
				copy_length = 255;
			}
			hci_tx_buf[0] = tx_type;
			memcpy(&hci_tx_buf[1], buf->data, copy_length);
			net_buf_pull(buf, copy_length);
			length -= copy_length;
			first_part = false;
			copy_length += 1;
		} else {
			copy_length = length;
			if (copy_length > 256) {
				copy_length = 256;
			}
			memcpy(hci_tx_buf, buf->data, copy_length);
			net_buf_pull(buf, copy_length);
			length -= copy_length;
		}

		int ret = uart_tx(hci_alif_dev, hci_tx_buf, copy_length, TX_TIMEOUT_US);

		if (ret < 0) {
			LOG_ERR("TX err %d", ret);
			goto done;
		}
		if (k_sem_take(&hci_tx_sem, K_MSEC(5)) != 0) {

			LOG_ERR("TX SEM TO");
			goto done;
		}
	}
done:
	net_buf_unref(buf);
}

static inline void process_rx(void)
{
	LOG_DBG("remaining %u discard %u have_hdr %u rx.buf %p len %u", rx.remaining, rx.discard,
		rx.have_hdr, rx.buf, rx.buf ? rx.buf->len : 0);

	if (rx.discard) {
		rx.discard -= hci_alif_discard(hci_alif_dev, rx.discard);
		return;
	}

	if (rx.have_hdr) {
		read_payload();
	} else {
		read_header();
	}
}

static void hci_data_parse(void)
{
	/* Read A ring Buffer allways to empty */
	while (1) {
		if (ring_buf_is_empty(&hci_ring_buf)) {
			return;
		}

		process_rx();
	}
}

/**
 * @brief UART async event callback
 *
 * This function handles UART async events for both TX and RX operations
 * when using DMA-based transfers with the async UART API
 *
 * @param dev UART device
 * @param evt UART event
 * @param user_data User data pointer
 */
static void hci_uart_async_callback(const struct device *dev, struct uart_event *evt,
				    void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		/* TX completed successfully */
		LOG_DBG("UART TX completed successfully");
		k_sem_give(&hci_tx_sem);
		break;

	case UART_TX_ABORTED:
		/* TX was aborted */
		LOG_ERR("UART TX was aborted, sent %d bytes", evt->data.tx.len);
		k_sem_give(&hci_tx_sem);
		break;
#if ALIF_HCI_DMA_RX_ENABLED
	case UART_RX_RDY:
		/* Data received and ready for processing */
		char *p_start = evt->data.rx.buf + evt->data.rx.offset;
		/* Push To ring buffer */
		ring_buf_put(&hci_ring_buf, p_start, evt->data.rx.len);
		LOG_DBG("Rxd %d , offset %d", evt->data.rx.len, evt->data.rx.offset);
		hci_data_parse();
		break;

	case UART_RX_BUF_REQUEST:
		/* UART driver is requesting a new buffer for continuous reception */
		pingpong ^= true;
		uart_rx_buf_rsp(dev, temp_rx_buf + (32 * pingpong), BUFF_SIZE);
		break;

	case UART_RX_BUF_RELEASED:
		/* Buffer has been released */
		break;

	case UART_RX_DISABLED:
		/* RX has been disabled */
		break;

	case UART_RX_STOPPED:
		/* RX has been stopped due to error */
		LOG_ERR("UART RX stopped due to error: %d", evt->data.rx_stop.reason);
		break;
#endif

	default:
		LOG_ERR("Unknown UART event: %d", evt->type);
		break;
	}
}

static void hci_alif_uart_isr(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	while (uart_irq_update(hci_alif_dev) && uart_irq_is_pending(hci_alif_dev)) {
		if (uart_irq_tx_ready(hci_alif_dev)) {
			process_tx();
		}

		if (uart_irq_rx_ready(hci_alif_dev)) {

			int read = uart_fifo_read(hci_alif_dev, temp_rx_buf, 64);

			LOG_DBG("RX %d", read);
			if (read > 0) {
				ring_buf_put(&hci_ring_buf, temp_rx_buf, read);
				hci_data_parse();
			}
		}
	}
}

static int hci_alif_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf), buf->len);

	/* Deassert&assert rts_n, falling edge triggers wake up the RF core */

	wake_es0(hci_alif_dev);

	if (hci_uart_dma_tx_enabled) {
		process_dma_tx(buf);
	} else {
		net_buf_put(&tx.fifo, buf);
		uart_irq_tx_enable(hci_alif_dev);
	}

	return 0;
}

int __weak bt_hci_transport_setup(const struct device *dev)
{
	hci_alif_discard(hci_alif_dev, 32);
	return 0;
}

static int hci_alif_close(void)
{
	int ret = stop_using_es0();

	if (-2 == ret) {
		return -EIO;
	}
	return 0;
}

static bool hci_uart_rx_dma_driver_check(void)
{
#if CONFIG_UART_ASYNC_API && ALIF_HCI_DMA_RX_ENABLED
#if DT_NODE_HAS_STATUS(DT_DMAS_CTLR_BY_NAME(DT_NODELABEL(uart_hci), rx), okay)
	/* RX DMA is disabled because it not give great value yet */
	const struct device *rxdma =
		DEVICE_DT_GET_OR_NULL(DT_DMAS_CTLR_BY_NAME(DT_NODELABEL(uart_hci), rx));

	if (rxdma && device_is_ready(rxdma)) {
		int rx_config = DT_DMAS_CELL_BY_NAME(DT_NODELABEL(uart_hci), rx, periph);

		uart_callback_set(hci_alif_dev, hci_uart_async_callback, NULL);
		LOG_DBG("DMA RX event enable %d", rx_config);
		dma_event_router_configure(DMA_UART_RX_GROUP, rx_config, false);
		return true;
	}
#endif
#endif
	uart_irq_callback_set(hci_alif_dev, hci_alif_uart_isr);
	uart_irq_rx_enable(hci_alif_dev);
	return false;
}

static bool hci_uart_tx_dma_driver_check(void)
{
#ifdef CONFIG_UART_ASYNC_API
#if (DT_NODE_HAS_STATUS(DT_DMAS_CTLR_BY_NAME(DT_NODELABEL(uart_hci), tx), okay))
	const struct device *txdma =
		DEVICE_DT_GET_OR_NULL(DT_DMAS_CTLR_BY_NAME(DT_NODELABEL(uart_hci), tx));

	if (txdma && device_is_ready(txdma)) {
		int tx_config = DT_DMAS_CELL_BY_NAME(DT_NODELABEL(uart_hci), tx, periph);

		uart_callback_set(hci_alif_dev, hci_uart_async_callback, NULL);
		LOG_DBG("DMA tX event enable %d", tx_config);
		dma_event_router_configure(DMA_UART_TX_GROUP, tx_config, false);
		return true;
	}
#endif
#endif
	uart_irq_callback_set(hci_alif_dev, hci_alif_uart_isr);
	return false;
}

static int hci_alif_open(void)
{
	int ret;
	k_tid_t tid;

	ret = take_es0_into_use();
	if (ret < 0) {
		return -EIO;
	}

	/* Disable receiver and interrupts */
	uart_irq_rx_disable(hci_alif_dev);
	uart_irq_tx_disable(hci_alif_dev);
	uart_rx_disable(hci_alif_dev);

	hci_uart_dma_rx_enabled = hci_uart_rx_dma_driver_check();
	hci_uart_dma_tx_enabled = hci_uart_tx_dma_driver_check();

	ret = bt_hci_transport_setup(hci_alif_dev);
	if (ret < 0) {
		return -EIO;
	}
#if ALIF_HCI_DMA_RX_ENABLED
	if (hci_uart_dma_rx_enabled) {
		ret = uart_rx_enable(hci_alif_dev, temp_rx_buf + (32 * pingpong), BUFF_SIZE,
				     RX_TIMEOUT_US);
		if (ret < 0) {
			LOG_ERR("Failed to enable UART: %d", ret);
			return ret;
		}
	}
#endif
	tid = k_thread_create(&alif_rx_thread_data, alif_rx_thread_stack,
			      K_KERNEL_STACK_SIZEOF(alif_rx_thread_stack), hci_alif_rx_thread, NULL,
			      NULL, NULL, K_PRIO_COOP(CONFIG_BT_RX_PRIO), 0, K_NO_WAIT);
	k_thread_name_set(tid, "hci_alif_rx_thread");

	return 0;
}

static const struct bt_hci_driver drv = {
	.name = "H:4",
	.bus = BT_HCI_DRIVER_BUS_UART,
	.open = hci_alif_open,
	.send = hci_alif_send,
	.close = hci_alif_close,
};

static int bt_uart_init(void)
{

	if (!device_is_ready(hci_alif_dev)) {
		return -ENODEV;
	}

	bt_hci_driver_register(&drv);

	return 0;
}

SYS_INIT(bt_uart_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
