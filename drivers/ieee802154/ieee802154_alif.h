/*
 * Copyright (c) 2023 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_

#include <zephyr/net/ieee802154_radio.h>

struct alif_frame {
	uint64_t time;	      /* frame RX timestamp. */
	void *fifo_reserved;  /* 1st word reserved for use by fifo. */
	uint8_t frame[127];   /* received frame. */
	uint8_t frame_length; /* length of frame 0 means free*/
	int8_t rssi;	      /* frame RSSI value. */
	int8_t status;	      /* RX status. */
	bool ack_fpb;         /* Frame pending bit value in ACK of this Frame*/
};

struct alif_csma_ca_config {
	uint8_t macMinBe;
	uint8_t macMaxBe;
	uint8_t macMaxCsmaBackoff;
	uint16_t macBackOffPeriod;
};

struct alif_802154_data {
	/* Pointer to the network interface. */
	struct net_if *iface;

	/* 802.15.4 HW address. */
	uint8_t mac[8];

	/* RX thread stack. */
	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_IEEE802154_ALIF_RX_TASK_STACK_SIZE);

	/* RX thread control block. */
	struct k_thread rx_thread;

	/* RX fifo queue. */
	struct k_fifo rx_fifo;

	/* Buffers for passing received frame pointers and data to the
	 * RX thread via rx_fifo object.
	 */
	struct alif_frame rx_frames[CONFIG_IEEE802154_ALIF_RX_BUFFERS];

	/* CSMA CA configuration. */
	struct alif_csma_ca_config csma_ca_conf;

	/* Capabilities of the network interface. */
	enum ieee802154_hw_caps capabilities;

	/*Current Extended address*/
	uint8_t extended_addr[8];

	/*Current channel*/
	uint16_t channel;

	/*Current PANID*/
	uint16_t panid;

	/*Current short address*/
	uint16_t short_addr;

	/*Current transmission power*/
	int16_t dbm;

	/* Auto ACK with Frame pending enabled */
	bool auto_ack_fpb;

	/* Pan coordinator */
	bool pan_coordinator;

	/* Promiscuous mode  */
	bool promiscuous;

	/* Pan coordinator */
	bool receiver_on;

	/*Event handler*/
	ieee802154_event_cb_t event_handler;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_ */
