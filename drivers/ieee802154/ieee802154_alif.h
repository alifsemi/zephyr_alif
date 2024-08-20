/*
 * Copyright (c) 2023 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_

#include <zephyr/net/ieee802154_radio.h>

#define MAC_IEEE_MAC_KEY_DESC_MAX_SIZE 3
/* CSL phase resolution is 10 * symbol time */
#define CSL_PHASE_TICK_US (10*IEEE802154_PHY_OQPSK_780_TO_2450MHZ_SYMBOL_PERIOD_NS) / NSEC_PER_USEC
#define CSL_PHASE_DELAY_US 15000
#define CSL_PERIOD_DELAY (CSL_PHASE_DELAY_US/CSL_PHASE_TICK_US) + 1
#define PLATFORMTIMER_RESYNCH_PERIOD_S 600

struct alif_802154_frame {
	uint64_t time;          /* frame RX timestamp. */
	void *fifo_reserved;    /* 1st word reserved for use by fifo. */
	uint32_t ack_frame_cnt; /* Enhanced ACK used Security frame counter */
	uint8_t frame[IEEE802154_MAX_PHY_PACKET_SIZE];     /* received frame. */
	uint8_t frame_length;   /* length of frame 0 means free*/
	uint8_t ack_key_idx;    /* Enhanced ACK key id */
	int8_t rssi;            /* frame RSSI value. */
	int8_t status;          /* RX status. */
	bool ack_fpb;           /* Frame pending bit value in ACK of this Frame*/
	bool ack_sec;           /* Enhanced ACK was secured */
};

struct alif_802154_csma_ca_config {
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

	/* RX fifo buffer list. */
	struct k_fifo rx_frame_buffers;

	/* Buffers for passing received frame pointers and data to the
	 * RX thread via rx_fifo object.
	 */
	struct alif_802154_frame rx_frames[CONFIG_IEEE802154_ALIF_RX_BUFFERS];
	/* Buffers for Mac TX endoding */
	uint8_t mac_encode_msg[IEEE802154_MAX_PHY_PACKET_SIZE];

	/* CSMA CA configuration. */
	struct alif_802154_csma_ca_config csma_ca_conf;

	/* Capabilities of the network interface. */
	enum ieee802154_hw_caps capabilities;

	/*Current Frame counter*/
	uint32_t frame_counter;

	/* Configured CSL perdiod in 160us ticks */
	uint16_t csl_period;

	/*Expected time when received packet should arrive*/
	net_time_t expected_rx_time;

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

	/* Transmission allowed any time during RX */
	bool tx_opt_allowed;

	/* Auto ACK with Frame pending enabled */
	bool auto_ack_fpb;

	/* Pan coordinator */
	bool pan_coordinator;

	/* Promiscuous mode  */
	bool promiscuous;

	/* RX is enabled when interface is started */
	bool rx_on_when_idle;

	/* Receiver is enabled */
	bool receiver_on;

	/* Interface is up/down */
	bool interface_up;

	/*Event handler*/
	ieee802154_event_cb_t event_handler;
};

#endif /* ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ALIF_H_ */
