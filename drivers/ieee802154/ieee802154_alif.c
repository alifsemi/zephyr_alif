/*
 * Copyright (c) 2023 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_ahi_ieee802154

#define LOG_MODULE_NAME alif_154_rf

#if defined(CONFIG_IEEE802154_DRIVER_LOG_LEVEL)
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>

#include <soc.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/debug/stack.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>

#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#include <platform-zephyr.h>
#endif

#include <zephyr/net/ieee802154_radio.h>

#include "ieee802154_alif.h"

#include <alif_mac154_api.h>
#include <se_service.h>

#if CONFIG_IEEE802154_VENDOR_OUI_ENABLE
#define IEEE802154_ALIF_OUI CONFIG_IEEE802154_VENDOR_OUI
#else
#define IEEE802154_ALIF_OUI (uint32_t)0x785994
#endif

#define ALIF_FCS_LENGTH    2
#define ALIF_USEC_PER_TICK 128

static int alif_rx_start(const struct device *dev);
static int alif_rx_stop(const struct device *dev);
static void alif_rx_status_send(enum alif_mac154_status_code status);

static struct alif_802154_data alif_data;

#define DATA(device) ((struct alif_802154_data *)device->data)

#if defined(CONFIG_NET_L2_OPENTHREAD)
static void ot_timer_synch_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	alif_rx_status_send(ALIF_MAC154_STATUS_TIMER_SYNCH);
}

K_TIMER_DEFINE(ot_platform_timer, ot_timer_synch_handler, NULL);
static bool ot_timer_active;

static void ot_platform_synch_timer_start(void)
{
	if (ot_timer_active) {
		return;
	}
	k_timer_start(&ot_platform_timer, K_SECONDS(PLATFORMTIMER_RESYNCH_PERIOD_S),
		      K_SECONDS(PLATFORMTIMER_RESYNCH_PERIOD_S));
	ot_timer_active = true;
}
#endif

static void alif_capabilities_set(void)
{
	alif_data.capabilities = IEEE802154_HW_FCS | IEEE802154_HW_FILTER |
				 IEEE802154_HW_TX_RX_ACK | IEEE802154_HW_CSMA |
				 IEEE802154_HW_ENERGY_SCAN | IEEE802154_HW_PROMISC;
	LOG_DBG("Alif cap 0x%x", alif_data.capabilities);
}

/* Radio device API */

static enum ieee802154_hw_caps alif_get_capabilities(const struct device *dev)
{
	LOG_DBG("cap: 0x%x", alif_data.capabilities);
	return alif_data.capabilities;
}

static int alif_set_channel(const struct device *dev, uint16_t channel)
{

	LOG_DBG("ch: %u", channel);

	if (channel < 11 || channel > 26) {
		return -EINVAL;
	}
	DATA(dev)->channel = channel;

	return 0;
}

static int alif_energy_scan_start(const struct device *dev, uint16_t duration,
				  energy_scan_done_cb_t done_cb)
{
	struct alif_energy_detect ed_req;
	struct alif_energy_detect_response ed_resp;

	LOG_DBG("duration: %u", duration);

	ed_req.channel = DATA(dev)->channel;
	ed_req.nb_tics = duration * 1000 / ALIF_USEC_PER_TICK;
	ed_req.threshold = 0;

	if (!alif_mac154_energy_detection(&ed_req, &ed_resp)) {
		LOG_ERR("energy scan failed");
	}
	LOG_INF("result : a:%d, max:%d, c:%d", ed_resp.average, ed_resp.max,
		ed_resp.nb_measure);
	done_cb(dev, ed_resp.max);
	return 0;
}

static int alif_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
		       const struct ieee802154_filter *filter)
{
	LOG_DBG("type %u", type);

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		LOG_DBG("set IEEE address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
			filter->ieee_addr[7], filter->ieee_addr[6], filter->ieee_addr[5],
			filter->ieee_addr[4], filter->ieee_addr[3], filter->ieee_addr[2],
			filter->ieee_addr[1], filter->ieee_addr[0]);
		memcpy(DATA(dev)->extended_addr, filter->ieee_addr, 8);
		alif_mac154_extended_address_set(filter->ieee_addr);

	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		LOG_DBG("set short addr 0x%x", filter->short_addr);
		DATA(dev)->short_addr = filter->short_addr;
		alif_mac154_short_address_set(filter->short_addr);

	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		LOG_DBG("set PAN ID: 0x%x", filter->pan_id);
		DATA(dev)->panid = filter->pan_id;
		alif_mac154_pan_id_set(filter->pan_id);

	} else if (type == IEEE802154_FILTER_TYPE_SRC_IEEE_ADDR) {
		LOG_WRN("set SRC IEEE address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
			filter->ieee_addr[7], filter->ieee_addr[6], filter->ieee_addr[5],
			filter->ieee_addr[4], filter->ieee_addr[3], filter->ieee_addr[2],
			filter->ieee_addr[1], filter->ieee_addr[0]);
		return -ENOTSUP;
	} else if (type == IEEE802154_FILTER_TYPE_SRC_SHORT_ADDR) {
		LOG_WRN("set SRC short addr 0x%x", filter->short_addr);
		return -ENOTSUP;
	}

	return 0;
}

static int alif_set_txpower(const struct device *dev, int16_t dbm)
{
	int ret = -1;

	LOG_DBG("dbm: %d", dbm);
	if (DATA(dev)->dbm != dbm) {
		ret = alif_mac154_tx_power_set(dbm);
	}

	if (ret != ALIF_MAC154_STATUS_OK) {
		return -ENOTSUP;
	}
	DATA(dev)->dbm = dbm;
	return 0;
}

static int alif_cca(const struct device *dev)
{
	struct alif_energy_detect ed_req;
	struct alif_energy_detect_response ed_resp;
	bool restore_rx = false;
	int ret = 0;

	LOG_DBG("");

	ed_req.channel = DATA(dev)->channel;
	ed_req.nb_tics = CONFIG_IEEE802154_ALIF_CCA_TICKS;
	ed_req.threshold = CONFIG_IEEE802154_ALIF_CCA_THRESHOLD;

	if (DATA(dev)->receiver_on) {
		alif_rx_stop(dev);
		restore_rx = true;
	}

	if (!alif_mac154_energy_detection(&ed_req, &ed_resp)) {
		LOG_ERR("CCA: fallback to auto CCA");
		goto exit;
	}
	if ((int8_t)ed_resp.average > CONFIG_IEEE802154_ALIF_CCA_THRESHOLD) {
		LOG_INF("CCA: busy avg:%d, max:%d", (int8_t)ed_resp.average, ed_resp.max);
		ret = -EBUSY;
	}
exit:
	if (restore_rx) {
		alif_rx_start(dev);
	}
	return ret;
}

static int handle_ack(const struct device *dev, struct alif_tx_ack_resp *param_ack)
{
	struct net_pkt *ack_pkt;

	LOG_DBG("ACK received: len:%d rssi:%d time:%" PRId64 "", param_ack->ack_msg_len,
		param_ack->ack_rssi, param_ack->ack_timestamp);
	LOG_HEXDUMP_DBG(param_ack->ack_msg, param_ack->ack_msg_len, "ACK:");

	ack_pkt = net_pkt_rx_alloc_with_buffer(DATA(dev)->iface, param_ack->ack_msg_len, AF_UNSPEC,
					       0, K_NO_WAIT);
	if (!ack_pkt) {
		LOG_ERR("allocation failed");
		return -ENOBUFS;
	}

	if (net_pkt_write(ack_pkt, param_ack->ack_msg, param_ack->ack_msg_len) < 0) {
		LOG_ERR("Failed to write to a packet.");
		net_pkt_unref(ack_pkt);
		return -ENOBUFS;
	}

	(void)net_pkt_set_ieee802154_lqi(ack_pkt, 80);
	(void)net_pkt_set_ieee802154_rssi(ack_pkt, param_ack->ack_rssi);
	net_pkt_set_timestamp_ns(ack_pkt, param_ack->ack_timestamp * NSEC_PER_USEC);

	net_pkt_cursor_init(ack_pkt);

	if (ieee802154_handle_ack(DATA(dev)->iface, ack_pkt) != NET_OK) {
		LOG_WRN("ACK not handled");
	}
	net_pkt_unref(ack_pkt);
	return 0;
}

static enum alif_mac154_status_code alif_transmit_csma(const struct device *dev,
						       struct alif_tx_req *p_tx,
						       struct alif_tx_ack_resp *p_tx_ack)
{
	enum alif_mac154_status_code ret;
	uint8_t csma_param_NB;
	uint8_t csma_param_BE;
	bool rx_stopped = false;

	LOG_DBG("ch:%d, cca:%d, ack:%d, len:%d", p_tx->channel, p_tx->cca_requested,
		p_tx->acknowledgment_asked, p_tx->length);

	csma_param_NB = 0;
	csma_param_BE = DATA(dev)->csma_ca_conf.macMinBe;
	do {
		if (csma_param_BE) {
			if (!DATA(dev)->tx_opt_allowed && rx_stopped) {
				struct alif_rx_enable rx_enable_req = {0};

				rx_enable_req.channel = DATA(dev)->channel;
				ret = alif_mac154_receive_start(&rx_enable_req);
				if (ret != ALIF_MAC154_STATUS_OK) {
					return ret;
				}
			}
			uint32_t backoff_period = DATA(dev)->csma_ca_conf.macBackOffPeriod *
						  (sys_rand32_get() % ((2 << csma_param_BE) - 1));
			LOG_DBG("wait csma: %d us", backoff_period);

			k_sleep(K_USEC(backoff_period));
		}
		if (!DATA(dev)->tx_opt_allowed && (DATA(dev)->receiver_on || rx_stopped)) {
			ret = alif_mac154_receive_stop();
			DATA(dev)->receiver_on = false;
			rx_stopped = true;
			if (ret != ALIF_MAC154_STATUS_OK) {
				return ret;
			}
		}
		/* transmit */
		ret = alif_mac154_transmit(p_tx, p_tx_ack);

		csma_param_NB++;
		csma_param_BE = MIN(csma_param_BE + 1, DATA(dev)->csma_ca_conf.macMaxBe);
	} while (ret == ALIF_MAC154_STATUS_CHANNEL_ACCESS_FAILURE &&
		 csma_param_NB <= DATA(dev)->csma_ca_conf.macMaxCsmaBackoff);

	return ret;
}

static uint32_t alif_cls_ie_element_update(uint8_t *header_ptr, uint16_t length,
					   uint16_t csl_period)
{
	struct mac_header_IE_s csl_ie;
	struct alif_mac154_csl_phase csl_phase;
	uint16_t phase, period;
	uint32_t tx_timestamp;

	csl_ie.id = MAC_HEADER_CSL_IE_ID;
	if (!alif_mac154_ie_header_element_get(header_ptr, length, &csl_ie)) {
		return 0;
	}

	period = csl_period;

	/* Update CSL phase  */
	if (alif_mac154_csl_phase_get(&csl_phase) != ALIF_MAC154_STATUS_OK) {
		return 0;
	}

	tx_timestamp = (uint32_t)csl_phase.timestamp;
	phase = csl_phase.csl_phase;
	LOG_DBG("CSL Period %" PRIu16 " phase %" PRIu16 " ts %" PRIu16 "", period, phase,
		tx_timestamp);

	/* Calculate phase from 15ms since now */
	tx_timestamp += (CSL_PHASE_DELAY_US + CSL_PHASE_TICK_US);
	if (phase > CSL_PERIOD_DELAY) {
		phase -= CSL_PERIOD_DELAY;
	} else {
		uint16_t delta_phase = CSL_PERIOD_DELAY - phase;

		if (delta_phase == 0) {
			delta_phase += 1;
			tx_timestamp += CSL_PHASE_TICK_US;
		}
		phase = (period - delta_phase);
	}
	LOG_DBG("CSL phase %u tx time %u", phase, tx_timestamp);
	sys_put_le16(phase, csl_ie.content_ptr);
	sys_put_le16(period, csl_ie.content_ptr + 2);
	return tx_timestamp;
}

static int alif_tx(const struct device *dev, enum ieee802154_tx_mode transmit_mode,
		   struct net_pkt *pkt, struct net_buf *frag)
{
	struct alif_tx_req transmit_req = {0};
	struct alif_tx_ack_resp transmit_resp = {0};
	char *tx_mode = "invalid";
	int ret = 0;

	transmit_req.length = frag->len + ALIF_FCS_LENGTH;
	if (alif_data.capabilities & IEEE802154_HW_TX_SEC) {
		memcpy(DATA(dev)->mac_encode_msg, frag->data, frag->len);
		transmit_req.p_payload = DATA(dev)->mac_encode_msg;
	} else {
		transmit_req.p_payload = frag->data;
	}

	transmit_req.channel = DATA(dev)->channel;
	transmit_req.cca_requested = true;
	transmit_req.acknowledgment_asked = ieee802154_is_ar_flag_set(frag);
	transmit_req.msg_id = 0;

	switch (transmit_mode) {
	case IEEE802154_TX_MODE_DIRECT:
		tx_mode = "TX with direct";
		transmit_req.cca_requested = false;
		break;
	case IEEE802154_TX_MODE_CCA:
		tx_mode = "TX with CCA";
		transmit_req.cca_requested = true;
		break;
	case IEEE802154_TX_MODE_CSMA_CA:
		tx_mode = "TX with CSMA-CA";
		transmit_req.cca_requested = true;
		break;
	case IEEE802154_TX_MODE_TXTIME:
		tx_mode = "TX Time";
		transmit_req.cca_requested = true;
		transmit_req.timestamp = (net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC);
		break;
	case IEEE802154_TX_MODE_TXTIME_CCA:
		tx_mode = "TX Time with CCA";
		transmit_req.cca_requested = true;
		transmit_req.timestamp = (net_pkt_timestamp_ns(pkt) / NSEC_PER_USEC);
		break;
	default:
		LOG_ERR("TX mode %d not supported", transmit_mode);
		return -ENOTSUP;
	}

	LOG_DBG("TX len: %d mode: %s, %s, %s time: %d", frag->len, tx_mode,
		transmit_req.acknowledgment_asked ? "ACK" : "no ACK",
		transmit_req.cca_requested ? "CCA" : "no CCA", transmit_req.timestamp);

	if (alif_data.capabilities & IEEE802154_HW_TX_SEC) {
		struct alif_802154_data *alif_radio = dev->data;
		struct alif_802154_frame_parser mac_frame;

		mac_frame.mac_packet = DATA(dev)->mac_encode_msg;
		mac_frame.mac_packet_length = frag->len;
		mac_frame.net_packet_ptr = frag->data;

		mac_frame.packet_ready = net_pkt_ieee802154_mac_hdr_rdy(pkt);

		if (alif_mac154_tx_packet_parse(&mac_frame) != ALIF_MAC154_STATUS_OK) {
			return -ENOTSUP;
		}
		/* Check IE Elements */

		if (mac_frame.ie_info.ie_header_len) {

			transmit_req.timestamp = alif_cls_ie_element_update(
				mac_frame.ie_info.ie_header_ptr, mac_frame.ie_info.ie_header_len,
				DATA(dev)->csl_period);
			if (transmit_req.timestamp) {
				transmit_mode = IEEE802154_TX_MODE_TXTIME;
				tx_mode = "TX Time";
				transmit_req.cca_requested = true;
			}
		}

		/* Encrypt */
		if (alif_mac154_mac_data_encode_and_authenticate(
			    &mac_frame, alif_radio->extended_addr) != ALIF_MAC154_STATUS_OK) {
			return -ENOTSUP;
		}

		net_pkt_set_ieee802154_frame_secured(pkt, mac_frame.encoded_packet);
		if (mac_frame.encoded_packet) {
			net_pkt_set_ieee802154_mac_hdr_rdy(pkt, mac_frame.packet_ready);
		}
	}

	if (transmit_mode == IEEE802154_TX_MODE_CSMA_CA) {
		ret = alif_transmit_csma(dev, &transmit_req, &transmit_resp);
	} else {
		if (!DATA(dev)->tx_opt_allowed && DATA(dev)->receiver_on) {
			alif_mac154_receive_stop();
			DATA(dev)->receiver_on = false;
		}
		ret = alif_mac154_transmit(&transmit_req, &transmit_resp);
	}

	switch (ret) {
	case ALIF_MAC154_STATUS_OK:
		if (!transmit_req.acknowledgment_asked) {
			return 0;
		}
		return handle_ack(dev, &transmit_resp);
	case ALIF_MAC154_STATUS_CHANNEL_ACCESS_FAILURE:
		LOG_DBG("TX CCA failed");
		return -EBUSY;
	case ALIF_MAC154_STATUS_NO_ACK:
		LOG_DBG("TX no ACK");
		return -ENOMSG;
	default:
		LOG_WRN("TX Failed %d", ret);
		return -EIO;
	}
	return 0;
}

static net_time_t alif_get_time(const struct device *dev)
{
	uint64_t ret;

	alif_mac154_timestamp_get(&ret);
	LOG_DBG("%" PRId64, ret);
	/* Covert us to ns */
	return (net_time_t)ret * NSEC_PER_USEC;
}

static uint8_t alif_get_accuracy(const struct device *dev)
{
	LOG_DBG("%d", CONFIG_IEEE802154_ALIF_CLOCK_ACCURACY);

	return CONFIG_IEEE802154_ALIF_CLOCK_ACCURACY;
}

static void alif_restore(const struct device *dev)
{
	LOG_WRN("restore mac configuration");
	alif_mac154_extended_address_set(DATA(dev)->extended_addr);
	alif_mac154_short_address_set(DATA(dev)->short_addr);
	alif_mac154_pan_id_set(DATA(dev)->panid);
	alif_mac154_tx_power_set(DATA(dev)->dbm);
	alif_mac154_cca_mode_set(CONFIG_IEEE802154_ALIF_MAC154_CCA_MODE);
	alif_mac154_ed_threshold_set(CONFIG_IEEE802154_ALIF_CCA_THRESHOLD);
}

static struct alif_802154_frame *alif_rx_frame_allocate(void)
{
	struct alif_802154_frame *p_frame;

	p_frame = k_fifo_get(&alif_data.rx_frame_buffers, K_NO_WAIT);

	return p_frame;

}

static void alif_rx_frame_free(struct alif_802154_frame *p_frame)
{
	if (p_frame) {
		p_frame->frame_length = 0;
		k_fifo_put(&alif_data.rx_frame_buffers, p_frame);
	}
}

static void alif_rx_frame_callback(struct alif_rx_frame_received *p_frame_recv)

{
	struct alif_802154_frame *p_frame = alif_rx_frame_allocate();

	LOG_DBG("RX frame received size:%d, rssi:%d, fpb %d", p_frame_recv->len, p_frame_recv->rssi,
		p_frame_recv->frame_pending);

	if (!p_frame) {
		LOG_ERR("RX frame dropped");
		return;
	}

	p_frame->rssi = p_frame_recv->rssi;
	p_frame->time = p_frame_recv->timestamp;
	p_frame->ack_fpb = p_frame_recv->frame_pending;
	p_frame->frame_length = p_frame_recv->len;
	p_frame->status = ALIF_MAC154_STATUS_OK;
	p_frame->ack_sec = p_frame_recv->ack_sec;
	p_frame->ack_key_idx = p_frame_recv->ack_key_idx;
	p_frame->ack_frame_cnt = p_frame_recv->ack_frame_cnt;
	memcpy(p_frame->frame, p_frame_recv->p_data, p_frame_recv->len);
	k_fifo_put(&alif_data.rx_fifo, p_frame);
}

static void alif_rx_status_send(enum alif_mac154_status_code status)
{
	struct alif_802154_frame *p_frame = alif_rx_frame_allocate();

	/* send RX failure event to RX task */

	if (!p_frame) {
		LOG_ERR("RF status callbakc lost: %u", status);
		return;
	}
	/* Set length for allocate frame */
	p_frame->status = status;
	k_fifo_put(&alif_data.rx_fifo, p_frame);
}

static void alif_rx_status_callback(enum alif_mac154_status_code status)
{
	LOG_DBG("status:: %d", status);

	if (status != ALIF_MAC154_STATUS_OK) {
		alif_data.receiver_on = false;
		alif_rx_status_send(status);
	}
}

static int alif_rx_start(const struct device *dev)
{
	struct alif_rx_enable rx_enable_req = {0};

	LOG_DBG("ch: %d, state: %d", DATA(dev)->channel, DATA(dev)->receiver_on);
	if (DATA(dev)->receiver_on) {
		return 0;
	}
	rx_enable_req.channel = DATA(dev)->channel;
	alif_mac154_receive_start(&rx_enable_req);
	DATA(dev)->receiver_on = true;

	return 0;
}

static int alif_rx_stop(const struct device *dev)
{
	LOG_DBG("state: %d", DATA(dev)->receiver_on);
	if (!DATA(dev)->receiver_on) {
		return 0;
	}

	alif_mac154_receive_stop();
	DATA(dev)->receiver_on = false;
	return 0;
}

static int alif_interface_start(const struct device *dev)
{
	LOG_DBG("");
	DATA(dev)->interface_up = true;

	if (DATA(dev)->rx_on_when_idle) {
		alif_rx_start(dev);
	}
	return 0;
}
static int alif_interface_stop(const struct device *dev)
{
	LOG_DBG("");
	DATA(dev)->interface_up = false;

	alif_rx_stop(dev);
	return 0;
}

static void alif_rx_thread(void *arg1, void *arg2, void *arg3)
{
	struct device *dev = (struct device *)arg1;
	struct net_pkt *pkt;
	struct alif_802154_frame *rx_frame;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	LOG_INF("RX task started");

	while (1) {
		LOG_DBG("Waiting for frame");

		rx_frame = k_fifo_get(&DATA(dev)->rx_fifo, K_FOREVER);

		if (rx_frame->status != ALIF_MAC154_STATUS_OK) {
			if (rx_frame->status == ALIF_MAC154_STATUS_OUT_OF_SYNC) {
				alif_mac154_reset();
				alif_restore(dev);
			} else if (rx_frame->status == ALIF_MAC154_STATUS_RX_STOPPED &&
				   DATA(dev)->receiver_on) {
				DATA(dev)->receiver_on = false;
				LOG_INF("RX Start from stopped");
				alif_rx_start(dev);
			} else if (rx_frame->status == ALIF_MAC154_STATUS_TIMER_SYNCH) {
				if (IS_ENABLED(CONFIG_NET_L2_OPENTHREAD)) {
					platformAlarmInit();
					LOG_DBG("Platform timer synch");
				}
			}
			goto process_done;
		}

		LOG_DBG("Frame received length:%d rssi: %d, time:%" PRId64 "",
			rx_frame->frame_length, rx_frame->rssi, rx_frame->time);

		pkt = net_pkt_rx_alloc_with_buffer(DATA(dev)->iface, rx_frame->frame_length,
						   AF_UNSPEC, 0, K_FOREVER);

		if (net_pkt_write(pkt, rx_frame->frame, rx_frame->frame_length)) {
			LOG_ERR("write failed");
			net_pkt_unref(pkt);
			goto process_done;
		}

		net_pkt_set_ieee802154_rssi(pkt, rx_frame->rssi);
		net_pkt_set_ieee802154_lqi(pkt, 80);
		net_pkt_set_ieee802154_ack_fpb(pkt, rx_frame->ack_fpb);
		net_pkt_set_timestamp_ns(pkt, rx_frame->time * NSEC_PER_USEC);
		net_pkt_set_ieee802154_ack_seb(pkt, rx_frame->ack_sec);
		net_pkt_set_ieee802154_ack_keyid(pkt, rx_frame->ack_key_idx);
		net_pkt_set_ieee802154_ack_fc(pkt, rx_frame->ack_frame_cnt);

		if (net_recv_data(DATA(dev)->iface, pkt) < 0) {
			LOG_ERR("Packet dropped by NET stack");
			net_pkt_unref(pkt);
		}
process_done:
		alif_rx_frame_free(rx_frame);
#if (LOG_LEVEL > LOG_LEVEL_INF)
		log_stack_usage(&DATA(dev)->rx_thread);
#endif
	}
}

static int alif_init(const struct device *dev)
{
	struct alif_mac154_api_cb api_cb = {0};

	LOG_INF("802154 rf init");

	memset(&alif_data, 0, sizeof(struct alif_802154_data));

	alif_capabilities_set();

	api_cb.rx_frame_recv_cb = alif_rx_frame_callback;
	api_cb.rx_status_cb = alif_rx_status_callback;
	alif_mac154_init(&api_cb);

	k_fifo_init(&DATA(dev)->rx_fifo);
	k_fifo_init(&DATA(dev)->rx_frame_buffers);
	/* Load frames to buffer list */
	for (int i = 0; i < ARRAY_SIZE(alif_data.rx_frames); i++) {
		alif_rx_frame_free(&alif_data.rx_frames[i]);
	}

	k_thread_create(&DATA(dev)->rx_thread, DATA(dev)->rx_stack,
			CONFIG_IEEE802154_ALIF_RX_TASK_STACK_SIZE, alif_rx_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);

	k_thread_name_set(&DATA(dev)->rx_thread, "alif_rx");

	return 0;
}

static void alif_eui64_read(uint8_t *eui64)
{
	/* We have a good EUI64 */
	if (*((uint64_t *)eui64) != 0) {
		return;
	}
#ifdef IEEE802154_ALIF_OUI
	eui64[0] = (uint8_t)(IEEE802154_ALIF_OUI >> 16);
	eui64[1] = (uint8_t)(IEEE802154_ALIF_OUI >> 8);
	eui64[2] = (uint8_t)(IEEE802154_ALIF_OUI);
#endif
	se_system_get_eui_extension(false, &eui64[3]);
	if ((*((uint64_t *)eui64) & 0xFFFFFFFFFF) != 0) {
		return;
	}
	/* Generate Random Local value (ELI) */
	se_service_get_rnd_num(&eui64[3], 5);
	eui64[0] = (eui64[0] & ~0x01) | 0x02;
}

static void alif_iface_init(struct net_if *iface)
{
	uint8_t version_major, version_minor, version_patch;
	enum alif_mac154_status_code reset_status;
	uint32_t capabilities;

	LOG_INF("");
	const struct device *dev = net_if_get_device(iface);
	struct alif_802154_data *alif_radio = dev->data;

	/* Generate unique Local id */
	alif_eui64_read(alif_radio->mac);

	LOG_INF("set MAC address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", alif_radio->mac[0],
		alif_radio->mac[1], alif_radio->mac[2], alif_radio->mac[3], alif_radio->mac[4],
		alif_radio->mac[5], alif_radio->mac[6], alif_radio->mac[7]);

	net_if_set_link_addr(iface, alif_radio->mac, sizeof(alif_radio->mac), NET_LINK_IEEE802154);

	alif_radio->dbm = CONFIG_IEEE802154_ALIF_POWER_LEVEL;

	alif_radio->iface = iface;

	/* Reset MAC state */
	reset_status = alif_mac154_reset();
	LOG_INF("802154 reset %d", reset_status);

	alif_mac154_version_get(&version_major, &version_minor, &version_patch);
	capabilities = alif_mac154_capabilities_get();

	if (IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP) && IS_ENABLED(CONFIG_NET_PKT_TXTIME) &&
	    capabilities & ALIF_IEEE802154_MAC_TXTIME) {
		alif_data.capabilities |= IEEE802154_HW_TXTIME;
	}
	if (IS_ENABLED(CONFIG_NET_PKT_TIMESTAMP) && IS_ENABLED(CONFIG_NET_PKT_TXTIME) &&
	    capabilities & ALIF_IEEE802154_MAC_RXTIME) {
		alif_data.capabilities |= IEEE802154_HW_RXTIME;
	}
	if (capabilities & ALIF_IEEE802154_MAC_TX_SEC) {
		alif_data.capabilities |= IEEE802154_HW_TX_SEC;
	}
	if (capabilities & ALIF_IEEE802154_MAC_RX_OPT) {
		alif_radio->tx_opt_allowed = true;
	}
	LOG_INF("802154 module version: %d.%d.%d. HAL cap:0x%x RF cap:0x%x", version_major,
		version_minor, version_patch, capabilities, alif_data.capabilities);

	/* Configure the CCA parameters */
	alif_mac154_cca_mode_set(CONFIG_IEEE802154_ALIF_MAC154_CCA_MODE);
	alif_mac154_ed_threshold_set(CONFIG_IEEE802154_ALIF_CCA_THRESHOLD);

	/* CSMA configuration value defaults */
	alif_radio->csma_ca_conf.macMinBe = 3;
	alif_radio->csma_ca_conf.macMaxBe = 5;
	alif_radio->csma_ca_conf.macMaxCsmaBackoff = 4;
	alif_radio->csma_ca_conf.macBackOffPeriod = 320;

	/*Receiver is started automatically if interface is started*/
	alif_radio->rx_on_when_idle = true;

	ieee802154_init(iface);
	if (IS_ENABLED(CONFIG_NET_L2_OPENTHREAD)) {
		/* Start timer for re-sych OpenThread platform timer */
		ot_platform_synch_timer_start();
	}
}

static int alif_configure(const struct device *dev, enum ieee802154_config_type type,
			  const struct ieee802154_config *config)
{
	int ret;

	switch (type) {
	case IEEE802154_CONFIG_EVENT_HANDLER:
		DATA(dev)->event_handler = config->event_handler;
		break;
	case IEEE802154_CONFIG_AUTO_ACK_FPB:
		if (config->auto_ack_fpb.enabled &&
		    config->auto_ack_fpb.mode == IEEE802154_FPB_ADDR_MATCH_THREAD) {
			DATA(dev)->auto_ack_fpb = true;
		} else if (!config->auto_ack_fpb.enabled) {
			DATA(dev)->auto_ack_fpb = false;
		} else {
			LOG_WRN("configure AUTO_ACK_FPB not supported: Enabled:%d, mode:%d",
				config->auto_ack_fpb.enabled, config->auto_ack_fpb.mode);
			return -ENOTSUP;
		}
		break;
	case IEEE802154_CONFIG_ACK_FPB:
		if (config->ack_fpb.extended) {
			if (!config->ack_fpb.addr) {
				ret = alif_mac154_pendings_long_address_insert(NULL);
			} else if (config->ack_fpb.enabled) {
				ret = alif_mac154_pendings_long_address_insert(
					config->ack_fpb.addr);
			} else {
				ret = alif_mac154_pendings_long_address_remove(
					config->ack_fpb.addr);
			}
		} else {
			if (!config->ack_fpb.addr) {
				ret = alif_mac154_pendings_short_address_insert(0xffff);
			} else if (config->ack_fpb.enabled) {
				ret = alif_mac154_pendings_short_address_insert(
					*((uint16_t *)config->ack_fpb.addr));
			} else {
				ret = alif_mac154_pendings_short_address_remove(
					*((uint16_t *)config->ack_fpb.addr));
			}
		}
		if (ret) {
			LOG_WRN("configure ACK_FPB: ret: %d Enabled:%d, extended:%d, ptr:%d", ret,
				config->ack_fpb.enabled, config->ack_fpb.extended,
				(uint32_t)config->ack_fpb.addr);
			return -EINVAL;
		}
		break;

	case IEEE802154_CONFIG_PROMISCUOUS:
		DATA(dev)->promiscuous = config->promiscuous;
		alif_mac154_promiscious_mode_set(config->promiscuous);
		break;
	case IEEE802154_CONFIG_MAC_KEYS:
		int n = 0;
		struct alif_mac154_key_description key_desc[MAC_IEEE_MAC_KEY_DESC_MAX_SIZE];

		if (!config->mac_keys[n].key_value) {
			LOG_INF("configure MAC Keys: Clear keys");
		} else {
			while (config->mac_keys[n].key_value) {
				LOG_INF("configure MAC Keys[%d]: FrameCounterPerKey:%d frame "
					"counter:%d "
					"Key id mode:%d Key index:%d",
					n, config->mac_keys[n].frame_counter_per_key,
					config->mac_keys[n].key_frame_counter,
					config->mac_keys[n].key_id_mode,
					*(config->mac_keys[n].key_id));

				LOG_HEXDUMP_INF(config->mac_keys[n].key_value, 16, "Key:");
				key_desc[n].key_value = config->mac_keys[n].key_value;
				key_desc[n].key_id = config->mac_keys[n].key_id;
				key_desc[n].key_id_mode = config->mac_keys[n].key_id_mode;
				key_desc[n].frame_counter = config->mac_keys[n].key_frame_counter;
				key_desc[n].frame_counter_per_key =
					config->mac_keys[n].frame_counter_per_key;
				n++;
			}
		}
		alif_mac154_key_value_description_set(key_desc, n);
		break;
	case IEEE802154_CONFIG_FRAME_COUNTER:
		LOG_INF("configure Frame counter:%d", config->frame_counter);
		DATA(dev)->frame_counter = config->frame_counter;
		alif_mac154_security_frame_counter_set(config->frame_counter);
		break;
	case IEEE802154_CONFIG_FRAME_COUNTER_IF_LARGER:
		LOG_INF("configure Frame counter if larger:%d", config->frame_counter);
		if (config->frame_counter > DATA(dev)->frame_counter) {
			DATA(dev)->frame_counter = config->frame_counter;
		}
		alif_mac154_security_frame_counter_set_if_larger(config->frame_counter);
		break;
	case IEEE802154_CONFIG_ENH_ACK_HEADER_IE:

		uint8_t ext_addr[8] = {0};
		const uint8_t *ext_ptr = NULL;

		if (config->ack_ie.ext_addr) {
			ext_ptr = ext_addr;
			/* Convert from big endian to little endian */
			for (int i = 0; i < 8; i++) {
				ext_addr[i] = config->ack_ie.ext_addr[7 - i];
			}

		}

		LOG_INF("configure ACK Header IE: short addr:0x%x purge:%d",
			config->ack_ie.short_addr, config->ack_ie.purge_ie);
		LOG_HEXDUMP_INF(config->ack_ie.ext_addr, 8, "ext_addr:");

		if (config->ack_ie.header_ie) {
			LOG_INF("ACK Header IE: type:%d, length:%d, high:%d, low:%d",
				config->ack_ie.header_ie->type, config->ack_ie.header_ie->length,
				config->ack_ie.header_ie->element_id_high,
				config->ack_ie.header_ie->element_id_low);
			LOG_HEXDUMP_INF(&config->ack_ie.header_ie->content,
					config->ack_ie.header_ie->length, "header:");
		}
		ret = alif_mac154_ack_header_ie_set(
			config->ack_ie.short_addr, ext_ptr, config->ack_ie.purge_ie,
			config->ack_ie.header_ie);

		if (ret != ALIF_MAC154_STATUS_OK) {
			return -EINVAL;
		}
		break;
	case IEEE802154_CONFIG_EXPECTED_RX_TIME:
		uint32_t expected_32bit_time;

		DATA(dev)->expected_rx_time = config->expected_rx_time;
		expected_32bit_time = config->expected_rx_time / NSEC_PER_USEC;
		LOG_DBG("configure CSL_RX_TIME: %" PRIu32 "", expected_32bit_time);

		alif_mac154_expected_rx_time_set(expected_32bit_time);

		break;
	case IEEE802154_CONFIG_RX_SLOT:
		struct alif_mac154_rx_slot rx_slot_config;

		rx_slot_config.start = config->rx_slot.start / NSEC_PER_USEC;
		rx_slot_config.duration = config->rx_slot.duration / NSEC_PER_USEC;
		rx_slot_config.channel = config->rx_slot.channel;
		LOG_DBG("configure RX_SLOT: start:%u duration:%u channel:%u", rx_slot_config.start,
			rx_slot_config.duration, config->rx_slot.channel);
		/* TODO Missing
		 * RX is restarted after RX slot is finalized if rx on when idle is enabled.
		 */
		if (DATA(dev)->receiver_on) {
			/*Receiver can't be on when scheduling RX slot*/
			alif_rx_stop(dev);
		}
		ret = alif_mac154_rx_slot_set(&rx_slot_config);
		if (ret != ALIF_MAC154_STATUS_OK) {
			return -EINVAL;
		}
		break;
	case IEEE802154_CONFIG_CSL_PERIOD:
		struct alif_mac154_csl_config csl_config = {0};
		LOG_INF("configure CSL_PERIOD: %d", config->csl_period);
		DATA(dev)->csl_period = config->csl_period;

		csl_config.csl_period = DATA(dev)->csl_period;

		ret = alif_mac154_csl_config_set(&csl_config);

		if (ret != ALIF_MAC154_STATUS_OK) {
			return -EINVAL;
		}

		break;
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		LOG_INF("configure pan coordinator: %d", config->pan_coordinator);
		return -ENOTSUP;
		break;
	case IEEE802154_CONFIG_RX_ON_WHEN_IDLE:
		LOG_INF("configure rx on when idle:%d", config->rx_on_when_idle);
		DATA(dev)->rx_on_when_idle = config->rx_on_when_idle;
		break;
	default:
		LOG_WRN("configure: %d", type);
		return -EINVAL;
	}

	return 0;
}

static struct ieee802154_radio_api alif_radio_api = {
	.iface_api.init = alif_iface_init,
	.get_capabilities = alif_get_capabilities,
	.cca = alif_cca,
	.set_channel = alif_set_channel,
	.filter = alif_filter,
	.set_txpower = alif_set_txpower,
	.start = alif_interface_start,
	.stop = alif_interface_stop,
	.tx = alif_tx,
	.ed_scan = alif_energy_scan_start,
	.get_time = alif_get_time,
	.get_sch_acc = alif_get_accuracy,
	.configure = alif_configure,
};

#if defined(CONFIG_NET_L2_OPENTHREAD)
#define L2          OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU         1280
#endif

NET_DEVICE_DT_INST_DEFINE(0, alif_init, NULL, &alif_data, NULL, CONFIG_IEEE802154_ALIF_INIT_PRIO,
			  &alif_radio_api, L2, L2_CTX_TYPE, MTU);
