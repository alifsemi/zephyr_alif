/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief AIROC Wi-Fi driver.
 */

#include <zephyr/logging/log.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <string.h>
#include <errno.h>
#include <airoc_wifi.h>
#include <whd_sdpcm.h>
#include <airoc_whd_hal_common.h>
#include <whd_wlioctl.h>
#if defined(CONFIG_AIROC_WIFI_P2P) || defined(CONFIG_AIROC_WIFI6)
/* Avoid #include <whd_int.h> here — conflicts with public whd_events.h via
 * other WHD headers. Declare only what softAP bring-up needs. */
whd_result_t whd_wifi_set_iovar_value(whd_interface_t ifp, const char *iovar,
				      uint32_t value);
whd_result_t whd_wifi_get_iovar_value(whd_interface_t ifp, const char *iovar,
				      uint32_t *value);
#endif
#if defined(CONFIG_AIROC_WIFI_P2P)
#include <airoc_wifi_p2p.h>
#include <airoc_wifi_wps_reg.h>
#endif

LOG_MODULE_REGISTER(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

#ifndef AIROC_WIFI_TX_PACKET_POOL_COUNT
#define AIROC_WIFI_TX_PACKET_POOL_COUNT (10)
#endif

#ifndef AIROC_WIFI_RX_PACKET_POOL_COUNT
#define AIROC_WIFI_RX_PACKET_POOL_COUNT (10)
#endif

#ifndef AIROC_WIFI_PACKET_POOL_SIZE
#define AIROC_WIFI_PACKET_POOL_SIZE     (1600)
#endif

#define AIROC_WIFI_PACKET_POOL_COUNT                                                               \
	(AIROC_WIFI_TX_PACKET_POOL_COUNT + AIROC_WIFI_RX_PACKET_POOL_COUNT)

#define AIROC_WIFI_WAIT_SEMA_MS    (30 * 1000)
#define AIROC_WIFI_SCAN_TIMEOUT_MS (12 * 1000)

/* AIROC private functions */
static whd_result_t airoc_wifi_host_buffer_get(whd_buffer_t *buffer, whd_buffer_dir_t direction,
					       uint16_t size, uint32_t timeout_ms);
static void airoc_wifi_buffer_release(whd_buffer_t buffer, whd_buffer_dir_t direction);
static uint8_t *airoc_wifi_buffer_get_current_piece_data_pointer(whd_buffer_t buffer);
static uint16_t airoc_wifi_buffer_get_current_piece_size(whd_buffer_t buffer);
static whd_result_t airoc_wifi_buffer_set_size(whd_buffer_t buffer, unsigned short size);
static whd_result_t airoc_wifi_buffer_add_remove_at_front(whd_buffer_t *buffer,
							  int32_t add_remove_amount);
static void airoc_wifi_network_process_ethernet_data(whd_interface_t interface,
						     whd_buffer_t buffer);
int airoc_wifi_init_primary(const struct device *dev, whd_interface_t *interface,
			    whd_netif_funcs_t *netif_funcs, whd_buffer_funcs_t *buffer_if);

/* Allocate network pool */
NET_BUF_POOL_FIXED_DEFINE(airoc_pool, AIROC_WIFI_PACKET_POOL_COUNT,
				AIROC_WIFI_PACKET_POOL_SIZE, 0, NULL);

/* AIROC globals */
static uint16_t ap_event_handler_index = 0xFF;
static uint16_t ap_event_handler_index_prim = 0xFF;

/* Use global iface pointer to support any Ethernet driver */
/* necessary for wifi callback functions */
static struct net_if *airoc_wifi_iface;

static whd_interface_t airoc_if;
static whd_interface_t airoc_sta_if;
static whd_interface_t airoc_ap_if;
static whd_interface_t airoc_active_ap_if;
static bool airoc_ap_on_primary;

static const whd_event_num_t sta_link_events[] = {
	WLC_E_LINK,    WLC_E_DEAUTH_IND,       WLC_E_DISASSOC_IND,
	WLC_E_PSK_SUP, WLC_E_CSA_COMPLETE_IND, WLC_E_NONE};

#ifndef WLC_EVENT_MSG_LINK
#define WLC_EVENT_MSG_LINK (0x01)
#endif
/* Not in public whd_events.h; WHD SoftAP uses this for IF add/change */
#ifndef WLC_E_IF
#define WLC_E_IF 54
#endif

static const whd_event_num_t ap_link_events[] = {
	WLC_E_DISASSOC_IND, WLC_E_DEAUTH_IND, WLC_E_ASSOC_IND, WLC_E_REASSOC_IND,
	WLC_E_AUTHORIZED,   WLC_E_AUTH,	     WLC_E_LINK,      WLC_E_IF,
	WLC_E_NONE};

struct airoc_wifi_event_t {
	uint8_t is_ap_event;
	uint32_t event_type;
	uint16_t flags;
	uint32_t status;
	uint32_t reason;
	uint8_t addr[6];
};

static uint16_t sta_event_handler_index = 0xFF;
static void airoc_event_task(void);
static struct airoc_wifi_data airoc_wifi_data = {0};

#if defined(SPI_DATA_IRQ_SHARED)
PINCTRL_DT_INST_DEFINE(0);
#endif

static struct airoc_wifi_config airoc_wifi_config = {
#if defined(CONFIG_AIROC_WIFI_BUS_SDIO)
	.bus_dev.bus_sdio = DEVICE_DT_GET(DT_INST_PARENT(0)),
#elif defined(CONFIG_AIROC_WIFI_BUS_SPI)
	.bus_dev.bus_spi = SPI_DT_SPEC_GET(DT_DRV_INST(0), AIROC_WIFI_SPI_OPERATION, 0),
	.bus_select_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), bus_select_gpios, {0}),
#if defined(SPI_DATA_IRQ_SHARED)
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
#endif
#endif
	.wifi_reg_on_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_reg_on_gpios, {0}),
	.wifi_host_wake_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_host_wake_gpios, {0}),
	.wifi_dev_wake_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(0), wifi_dev_wake_gpios, {0}),
};

static whd_buffer_funcs_t airoc_wifi_buffer_if_default = {
	.whd_host_buffer_get = airoc_wifi_host_buffer_get,
	.whd_buffer_release = airoc_wifi_buffer_release,
	.whd_buffer_get_current_piece_data_pointer =
		airoc_wifi_buffer_get_current_piece_data_pointer,
	.whd_buffer_get_current_piece_size = airoc_wifi_buffer_get_current_piece_size,
	.whd_buffer_set_size = airoc_wifi_buffer_set_size,
	.whd_buffer_add_remove_at_front = airoc_wifi_buffer_add_remove_at_front,
};

static whd_netif_funcs_t airoc_wifi_netif_if_default = {
	.whd_network_process_ethernet_data = airoc_wifi_network_process_ethernet_data,
};

K_MSGQ_DEFINE(airoc_wifi_msgq, sizeof(struct airoc_wifi_event_t), 10, 4);
K_THREAD_STACK_DEFINE(airoc_wifi_event_stack, CONFIG_AIROC_WIFI_EVENT_TASK_STACK_SIZE);
static struct k_thread airoc_wifi_event_thread;

/*
 * AIROC Wi-Fi helper functions
 */
whd_interface_t airoc_wifi_get_whd_interface(void)
{
	return airoc_if;
}

int airoc_wifi_send_raw_eth(const uint8_t *frame, size_t len)
{
	struct net_buf *buf = NULL;
	cy_rslt_t ret;

	if (frame == NULL || len == 0 || airoc_if == NULL) {
		return -EINVAL;
	}

	ret = airoc_wifi_host_buffer_get((whd_buffer_t *)&buf, WHD_NETWORK_TX,
					 (uint16_t)(len + sizeof(data_header_t)),
					 0);
	if ((ret != WHD_SUCCESS) || (buf == NULL)) {
		return -ENOBUFS;
	}

	net_buf_reserve(buf, sizeof(data_header_t));
	memcpy(buf->data, frame, len);
	buf->len = len;

	/* WHD owns the buffer from here on and releases it on error too. */
	ret = whd_network_send_ethernet_data(airoc_if, (void *)buf);
	if (ret != WHD_SUCCESS) {
		return -EIO;
	}
	return 0;
}

static void airoc_wifi_scan_cb_search(whd_scan_result_t **result_ptr, void *user_data,
				      whd_scan_status_t status)
{
	if (status == WHD_SCAN_ABORTED) {
		k_sem_give(&airoc_wifi_data.sema_scan);
		return;
	}

	if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY) {
		k_sem_give(&airoc_wifi_data.sema_scan);
	} else if ((status == WHD_SCAN_INCOMPLETE) && (user_data != NULL) &&
		   ((**result_ptr).SSID.length == ((whd_scan_result_t *)user_data)->SSID.length)) {
		if (strncmp(((whd_scan_result_t *)user_data)->SSID.value, (**result_ptr).SSID.value,
			    (**result_ptr).SSID.length) == 0) {
			memcpy(user_data, *result_ptr, sizeof(whd_scan_result_t));
		}
	}
}

static int convert_whd_security_to_zephyr(whd_security_t security)
{
	int zephyr_security = WIFI_SECURITY_TYPE_UNKNOWN;

	switch (security) {
	case WHD_SECURITY_OPEN:
		zephyr_security = WIFI_SECURITY_TYPE_NONE;
		break;

	case WHD_SECURITY_WEP_PSK:
	case WHD_SECURITY_WEP_SHARED:
		zephyr_security = WIFI_SECURITY_TYPE_WEP;
		break;

	case WHD_SECURITY_WPA2_WPA_MIXED_PSK:
	case WHD_SECURITY_WPA2_WPA_AES_PSK:
	case WHD_SECURITY_WPA3_WPA2_PSK:
		zephyr_security = WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL;
		break;

	case WHD_SECURITY_WPA2_AES_PSK:
		zephyr_security = WIFI_SECURITY_TYPE_PSK;
		break;

	case WHD_SECURITY_WPA2_AES_PSK_SHA256:
		zephyr_security = WIFI_SECURITY_TYPE_PSK_SHA256;
		break;

	case WHD_SECURITY_WPA3_SAE:
		zephyr_security = WIFI_SECURITY_TYPE_SAE;
		break;

	case WHD_SECURITY_WPA_TKIP_PSK:
	case WHD_SECURITY_WPA_MIXED_PSK:
	case WHD_SECURITY_WPA_AES_PSK:
		zephyr_security = WIFI_SECURITY_TYPE_WPA_PSK;
		break;

	default:
		if ((security & ENTERPRISE_ENABLED) != 0) {
			zephyr_security = WIFI_SECURITY_TYPE_EAP;
		}
		break;
	}
	return zephyr_security;
}

static whd_security_t convert_zephyr_security_to_whd(int security)
{
	whd_security_t whd_security = WIFI_SECURITY_TYPE_UNKNOWN;

	switch (security) {
	case WIFI_SECURITY_TYPE_NONE:
		whd_security = WHD_SECURITY_OPEN;
		break;

	case WIFI_SECURITY_TYPE_WEP:
		whd_security = WHD_SECURITY_WEP_PSK;
		break;

	case WIFI_SECURITY_TYPE_WPA_AUTO_PERSONAL:
		whd_security = WHD_SECURITY_WPA3_WPA2_PSK;
		break;

	case WIFI_SECURITY_TYPE_PSK:
		whd_security = WHD_SECURITY_WPA2_AES_PSK;
		break;

	case WIFI_SECURITY_TYPE_PSK_SHA256:
		whd_security = WIFI_SECURITY_TYPE_PSK_SHA256;
		break;

	case WIFI_SECURITY_TYPE_SAE:
		whd_security = WHD_SECURITY_WPA3_SAE;
		break;

	case WIFI_SECURITY_TYPE_WPA_PSK:
		whd_security = WHD_SECURITY_WPA_AES_PSK;
		break;

	default:
		break;
	}
	return whd_security;
}

static uint8_t convert_whd_band_to_zephyr(whd_802_11_band_t band)
{
	uint8_t zephyr_band = WIFI_FREQ_BAND_UNKNOWN;

	switch (band) {
	case WHD_802_11_BAND_2_4GHZ:
		zephyr_band = WIFI_FREQ_BAND_2_4_GHZ;
		break;

	case WHD_802_11_BAND_5GHZ:
		zephyr_band = WIFI_FREQ_BAND_5_GHZ;
		break;

	case WHD_802_11_BAND_6GHZ:
		zephyr_band = WIFI_FREQ_BAND_6_GHZ;
		break;
	}
	return zephyr_band;
}

static void parse_scan_result(whd_scan_result_t *p_whd_result, struct wifi_scan_result *p_zy_result)
{
	if (p_whd_result->SSID.length != 0) {
		p_zy_result->ssid_length = p_whd_result->SSID.length;
		strncpy(p_zy_result->ssid, p_whd_result->SSID.value, p_whd_result->SSID.length);
		p_zy_result->channel = p_whd_result->channel;
		p_zy_result->band = convert_whd_band_to_zephyr(p_whd_result->band);
		p_zy_result->security = convert_whd_security_to_zephyr(p_whd_result->security);
		p_zy_result->rssi = (int8_t)p_whd_result->signal_strength;
		p_zy_result->mac_length = 6;
		memcpy(p_zy_result->mac, &p_whd_result->BSSID, 6);
	}
}

static void scan_callback(whd_scan_result_t **result_ptr, void *user_data, whd_scan_status_t status)
{
	struct airoc_wifi_data *data = user_data;
	whd_scan_result_t whd_scan_result;
	struct wifi_scan_result zephyr_scan_result = {0};

	if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY || status == WHD_SCAN_ABORTED) {
		data->scan_rslt_cb(data->iface, 0, NULL);
		data->scan_rslt_cb = NULL;
		/* NOTE: It is complete of scan packet, do not need to clean result_ptr,
		 * WHD will release result_ptr buffer
		 */
		return;
	}

	/* We recived scan data so process it */
	if ((result_ptr != NULL) && (*result_ptr != NULL)) {
		memcpy(&whd_scan_result, *result_ptr, sizeof(whd_scan_result_t));
		parse_scan_result(&whd_scan_result, &zephyr_scan_result);
		data->scan_rslt_cb(data->iface, 0, &zephyr_scan_result);
	}
	memset(*result_ptr, 0, sizeof(whd_scan_result_t));
}

/*
 * Implement WHD network buffers functions
 */
static whd_result_t airoc_wifi_host_buffer_get(whd_buffer_t *buffer, whd_buffer_dir_t direction,
					       uint16_t size, uint32_t timeout_ms)
{
	ARG_UNUSED(direction);
	ARG_UNUSED(timeout_ms);
	struct net_buf *buf;

	buf = net_buf_alloc_len(&airoc_pool, size, K_NO_WAIT);
	if ((buf == NULL) || (buf->size < size)) {
		return WHD_BUFFER_ALLOC_FAIL;
	}
	*buffer = buf;

	/* Set buffer size */
	(void) airoc_wifi_buffer_set_size(*buffer, size);

	return WHD_SUCCESS;
}

static void airoc_wifi_buffer_release(whd_buffer_t buffer, whd_buffer_dir_t direction)
{
	CY_UNUSED_PARAMETER(direction);
	(void)net_buf_destroy((struct net_buf *)buffer);
}

static uint8_t *airoc_wifi_buffer_get_current_piece_data_pointer(whd_buffer_t buffer)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	return (uint8_t *)buf->data;
}

static uint16_t airoc_wifi_buffer_get_current_piece_size(whd_buffer_t buffer)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	return (uint16_t)buf->size;
}

static whd_result_t airoc_wifi_buffer_set_size(whd_buffer_t buffer, unsigned short size)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf *buf = (struct net_buf *)buffer;

	buf->size = size;
	return CY_RSLT_SUCCESS;
}

static whd_result_t airoc_wifi_buffer_add_remove_at_front(whd_buffer_t *buffer,
							  int32_t add_remove_amount)
{
	CY_ASSERT(buffer != NULL);
	struct net_buf **buf = (struct net_buf **)buffer;

	if (add_remove_amount > 0) {
		(*buf)->len = (*buf)->size;
		(*buf)->data = net_buf_pull(*buf, add_remove_amount);
	} else {
		(*buf)->data = net_buf_push(*buf, -add_remove_amount);
		(*buf)->len = (*buf)->size;
	}
	return WHD_SUCCESS;
}

static int airoc_mgmt_send(const struct device *dev, struct net_pkt *pkt)
{
	struct airoc_wifi_data *data = dev->data;
	cy_rslt_t ret;
	size_t pkt_len = net_pkt_get_len(pkt);
	struct net_buf *buf = NULL;

	/* Read the packet payload */
	if (net_pkt_read(pkt, data->frame_buf, pkt_len) < 0) {
		LOG_ERR("net_pkt_read failed");
		return -EIO;
	}

	/* Allocate Network Buffer from pool with Packet Length + Data Header */
	ret = airoc_wifi_host_buffer_get((whd_buffer_t *) &buf, WHD_NETWORK_TX,
					 pkt_len + sizeof(data_header_t), 0);
	if ((ret != WHD_SUCCESS) || (buf == NULL)) {
		return -EIO;
	}

	/* Reserve the buffer Headroom for WHD Data header */
	net_buf_reserve(buf, sizeof(data_header_t));

	/* Copy the buffer to network Buffer pointer */
	(void)memcpy(buf->data, data->frame_buf, pkt_len);

	/* Call WHD API to send out the Packet */
	ret = whd_network_send_ethernet_data(airoc_if, (void *)buf);
	if (ret != CY_RSLT_SUCCESS) {
		LOG_ERR("whd_network_send_ethernet_data failed");
#if defined(CONFIG_NET_STATISTICS_WIFI)
		data->stats.errors.tx++;
#endif
		return -EIO;
	}

#if defined(CONFIG_NET_STATISTICS_WIFI)
	data->stats.bytes.sent += pkt_len;
	data->stats.pkts.tx++;
#endif

	return 0;
}

static void airoc_wifi_network_process_ethernet_data(whd_interface_t interface, whd_buffer_t buffer)
{
	struct net_pkt *pkt;
	uint8_t *data;
	uint32_t len;
	bool net_pkt_unref_flag = false;

	if (interface == NULL || interface->whd_driver == NULL || buffer == NULL) {
		if (buffer != NULL) {
			airoc_wifi_buffer_release(buffer, WHD_NETWORK_RX);
		}
		return;
	}

	data = whd_buffer_get_current_piece_data_pointer(interface->whd_driver, buffer);
	len = whd_buffer_get_current_piece_size(interface->whd_driver, buffer);

#if defined(CONFIG_AIROC_WIFI_P2P)
	if (airoc_p2p_eapol_rx(data, len)) {
		airoc_wifi_buffer_release(buffer, WHD_NETWORK_RX);
		return;
	}
#endif

	if ((airoc_wifi_iface != NULL) && net_if_flag_is_set(airoc_wifi_iface, NET_IF_UP)) {

		pkt = net_pkt_rx_alloc_with_buffer(airoc_wifi_iface, len, AF_UNSPEC, 0, K_NO_WAIT);

		if (pkt != NULL) {
			if (net_pkt_write(pkt, data, len) < 0) {
				LOG_ERR("Failed to write pkt");
				net_pkt_unref_flag = true;
			}

			if ((net_pkt_unref_flag) || (net_recv_data(airoc_wifi_iface, pkt) < 0)) {
				LOG_ERR("Failed to push received data");
				net_pkt_unref_flag = true;
			}
		} else {
			LOG_ERR("Failed to get net buffer (len=%u)", len);
		}
	}

	/* Release a packet buffer */
	airoc_wifi_buffer_release(buffer, WHD_NETWORK_RX);

#if defined(CONFIG_NET_STATISTICS_WIFI)
	airoc_wifi_data.stats.bytes.received += len;
	airoc_wifi_data.stats.pkts.rx++;
#endif

	if (net_pkt_unref_flag) {
		net_pkt_unref(pkt);
#if defined(CONFIG_NET_STATISTICS_WIFI)
		airoc_wifi_data.stats.errors.rx++;
#endif
	}
}

static enum ethernet_hw_caps airoc_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return ETHERNET_HW_FILTERING;
}

static int airoc_set_config(const struct device *dev,
			    enum ethernet_config_type type,
			    const struct ethernet_config *config)
{
	ARG_UNUSED(dev);
	whd_mac_t whd_mac_addr;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_FILTER:
		for (int i = 0; i < WHD_ETHER_ADDR_LEN; i++) {
			whd_mac_addr.octet[i] = config->filter.mac_address.addr[i];
		}
		if (config->filter.set) {
			whd_wifi_register_multicast_address(airoc_if, &whd_mac_addr);
		} else {
			whd_wifi_unregister_multicast_address(airoc_if, &whd_mac_addr);
		}
		return 0;
	default:
		break;
	}
	return -ENOTSUP;
}

static void *link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header,
				 const uint8_t *event_data, void *handler_user_data)
{
	struct airoc_wifi_event_t airoc_event = {
		.is_ap_event = 0,
		.event_type = event_header->event_type,
		.flags = event_header->flags,
		.status = event_header->status,
		.reason = event_header->reason,
	};

	ARG_UNUSED(ifp);
	ARG_UNUSED(event_data);
	ARG_UNUSED(handler_user_data);

	k_msgq_put(&airoc_wifi_msgq, &airoc_event, K_FOREVER);
	return NULL;
}

static void airoc_event_task(void)
{
	struct airoc_wifi_event_t event;

	while (1) {
		k_msgq_get(&airoc_wifi_msgq, &event, K_FOREVER);

		if (event.is_ap_event) {
			switch ((whd_event_num_t)event.event_type) {
			case WLC_E_ASSOC_IND:
			case WLC_E_REASSOC_IND:
				LOG_INF("AP/GO: station associating "
					"%02x:%02x:%02x:%02x:%02x:%02x",
					event.addr[0], event.addr[1], event.addr[2],
					event.addr[3], event.addr[4], event.addr[5]);
#if defined(CONFIG_AIROC_WIFI_P2P)
				airoc_wps_reg_on_assoc(event.addr);
#endif
				break;
			case WLC_E_AUTHORIZED:
				LOG_INF("AP/GO: station authorized");
				break;
			case WLC_E_DEAUTH_IND:
			case WLC_E_DISASSOC_IND:
				LOG_INF("AP/GO: station left "
					"%02x:%02x:%02x:%02x:%02x:%02x",
					event.addr[0], event.addr[1], event.addr[2],
					event.addr[3], event.addr[4], event.addr[5]);
				break;
			default:
				break;
			}
			continue;
		}

		switch ((whd_event_num_t)event.event_type) {
		case WLC_E_LINK:
			break;

		case WLC_E_DEAUTH_IND:
		case WLC_E_DISASSOC_IND:
			net_if_dormant_on(airoc_wifi_iface);
			break;

		default:
			break;
		}
	}
}

static void airoc_mgmt_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct airoc_wifi_data *data = dev->data;
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
	data->iface = iface;
	airoc_wifi_iface = iface;

#if defined(CONFIG_AIROC_WIFI_P2P)
	airoc_p2p_set_iface(iface);
#endif

	/* Read WLAN MAC Address */
	if (whd_wifi_get_mac_address(airoc_sta_if, &airoc_sta_if->mac_addr) != WHD_SUCCESS) {
		LOG_ERR("Failed to get mac address");
	} else {
		(void)memcpy(&data->mac_addr, &airoc_sta_if->mac_addr,
			     sizeof(airoc_sta_if->mac_addr));
	}

	/* Assign link local address. */
	if (net_if_set_link_addr(iface, data->mac_addr, 6, NET_LINK_ETHERNET)) {
		LOG_ERR("Failed to set link addr");
	}

	/* Initialize Ethernet L2 stack */
	ethernet_init(iface);

	/* Not currently connected to a network */
	net_if_dormant_on(iface);

	/* L1 network layer (physical layer) is up */
	net_if_carrier_on(data->iface);
}

static int airoc_mgmt_scan(const struct device *dev, struct wifi_scan_params *params,
			   scan_result_cb_t cb)
{
	struct airoc_wifi_data *data = dev->data;

	if (data->scan_rslt_cb != NULL) {
		LOG_INF("Scan callback in progress");
		return -EINPROGRESS;
	}

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	data->scan_rslt_cb = cb;

	/* Connect to the network */
	if (whd_wifi_scan(airoc_sta_if, params->scan_type, WHD_BSS_TYPE_ANY, &(data->ssid), NULL,
			  NULL, NULL, scan_callback, &(data->scan_result), data) != WHD_SUCCESS) {
		LOG_ERR("Failed to start scan");
		k_sem_give(&data->sema_common);
		return -EAGAIN;
	}

	k_sem_give(&data->sema_common);
	return 0;
}

static bool is_invalid_security(int security, uint8_t psk_length)
{
	return ((security == WIFI_SECURITY_TYPE_NONE) && (psk_length > 0));
}

static int airoc_mgmt_connect(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;
	int ret = 0;
	whd_scan_result_t scan_result;
	whd_scan_result_t usr_result = {0};
	/* Try to scan ssid to define security */
	whd_scan_result_t tmp_result = {0};

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (data->is_sta_connected) {
		LOG_ERR("Already connected");
		ret = -EALREADY;
		goto error;
	}

	if (data->is_ap_up) {
		LOG_ERR("Network interface is busy AP. Please first disable AP.");
		ret = -EBUSY;
		goto error;
	}

	usr_result.SSID.length = params->ssid_length;
	memcpy(usr_result.SSID.value, params->ssid, params->ssid_length);
	usr_result.security = convert_zephyr_security_to_whd(params->security);

	if (is_invalid_security(params->security, params->psk_length)) {

		if (whd_wifi_scan(airoc_sta_if, WHD_SCAN_TYPE_ACTIVE, WHD_BSS_TYPE_ANY, NULL, NULL,
				  NULL, NULL, airoc_wifi_scan_cb_search, &scan_result,
				  &(tmp_result)) != WHD_SUCCESS) {
			LOG_ERR("Failed start scan");
			ret = -EAGAIN;
			goto error;
		}

		if (k_sem_take(&airoc_wifi_data.sema_scan, K_MSEC(AIROC_WIFI_SCAN_TIMEOUT_MS)) !=
		    0) {
			whd_wifi_stop_scan(airoc_sta_if);
			ret = -EAGAIN;
			goto error;
		}
	} else {
		/* Fallback to user input */
		if (tmp_result.security == WHD_SECURITY_UNKNOWN) {
			usr_result.security = tmp_result.security;
		}
	}

	if (usr_result.security == WHD_SECURITY_UNKNOWN) {
		ret = -EAGAIN;
		LOG_ERR("Could not scan device");
		goto error;
	}

	/* Connect to the network */
	if (whd_wifi_join(airoc_sta_if, &usr_result.SSID, usr_result.security, params->psk,
			  params->psk_length) != WHD_SUCCESS) {
		LOG_ERR("Failed to connect with network");

		ret = -EAGAIN;
		goto error;
	}

error:
	if (ret < 0) {
		net_if_dormant_on(data->iface);
	} else {
		net_if_dormant_off(data->iface);
		data->is_sta_connected = true;
#if defined(CONFIG_NET_DHCPV4)
		net_dhcpv4_restart(data->iface);
#endif /* defined(CONFIG_NET_DHCPV4) */
	}

	wifi_mgmt_raise_connect_result_event(data->iface, ret);
	k_sem_give(&data->sema_common);
	return ret;
}

static int airoc_mgmt_disconnect(const struct device *dev)
{
	int ret = 0;
	struct airoc_wifi_data *data = (struct airoc_wifi_data *)dev->data;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (whd_wifi_leave(airoc_sta_if) != WHD_SUCCESS) {
		k_sem_give(&data->sema_common);
		ret = -EAGAIN;
	} else {
		data->is_sta_connected = false;
		net_if_dormant_on(data->iface);
	}

	wifi_mgmt_raise_disconnect_result_event(data->iface, ret);
	k_sem_give(&data->sema_common);

	return ret;
}

static void *airoc_wifi_ap_link_events_handler(whd_interface_t ifp,
					       const whd_event_header_t *event_header,
					       const uint8_t *event_data, void *handler_user_data)
{
	struct airoc_wifi_event_t airoc_event = {
		.is_ap_event = 1,
		.event_type = event_header->event_type,
		.flags = event_header->flags,
		.status = event_header->status,
		.reason = event_header->reason,
	};

	ARG_UNUSED(ifp);
	ARG_UNUSED(event_data);
	ARG_UNUSED(handler_user_data);

	memcpy(airoc_event.addr, event_header->addr.octet, 6);
	k_msgq_put(&airoc_wifi_msgq, &airoc_event, K_FOREVER);

	return NULL;
}

static int airoc_mgmt_ap_enable(const struct device *dev, struct wifi_connect_req_params *params)
{
	struct airoc_wifi_data *data = dev->data;
	whd_security_t security;
	whd_ssid_t ssid;
	whd_interface_t ap_if;
	uint16_t chanspec;
	int ret = 0;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (data->is_sta_connected) {
		LOG_ERR("Network interface is busy in STA mode. Please first disconnect STA.");
		ret = -EBUSY;
		goto error;
	}

	if (data->is_ap_up) {
		LOG_ERR("Already AP is on - first disable");
		ret = -EAGAIN;
		goto error;
	}

#if defined(CONFIG_AIROC_WIFI6)
	/* CYW55500/55572: secondary SoftAP beacons sporadically and won't
	 * accept associations. WHD primary path disables apsta and runs
	 * classic SoftAP with stable beacon TX.
	 */
	ap_if = airoc_sta_if;
	airoc_ap_on_primary = true;
#else
	if (!data->second_interface_init) {
		/* NULL MAC: init_ap + set_mac toggles LAA for AP role.
		 * Do not pre-set LAA — that makes WHD toggle back to STA MAC.
		 */
		if (whd_add_secondary_interface(data->whd_drv, NULL, &airoc_ap_if) !=
		    CY_RSLT_SUCCESS) {
			LOG_ERR("Error Unable to bring up the whd secondary interface");
			ret = -EAGAIN;
			goto error;
		}
		data->second_interface_init = true;
	}
	ap_if = airoc_ap_if;
	airoc_ap_on_primary = false;
#endif

	(void)whd_wifi_set_iovar_value(ap_if, IOVAR_STR_MPC, 0);

	ssid.length = params->ssid_length;
	memcpy(ssid.value, params->ssid, ssid.length);

	/*
	 * WIFI6: pass a full chanspec (band|bw|channel); WHD CH20MHZ_CHSPEC uses
	 * CHSPEC_IS2G(). WIFI5 (e.g. CYW43439): pass channel number only — WHD
	 * CH20MHZ_CHSPEC uses (chspec <= 14) to pick 2G vs 5G. Passing 0x1006
	 * wrongly selects 5G (0xd006) and returns WHD_WLAN_BADCHAN.
	 */
	if ((params->channel > 0) && (params->channel < 12)) {
		chanspec = params->channel;
	} else if ((params->channel > 35) && (params->channel < 166)) {
		chanspec = params->channel;
	} else {
		chanspec = 1;
		LOG_WRN("Discard of setting unsupported channel: %u (will set 1)",
			params->channel);
	}

#if defined(CONFIG_AIROC_WIFI6)
	if ((chanspec > 0) && (chanspec < 12)) {
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BAND_2G);
	} else {
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BAND_5G);
	}

	switch (params->bandwidth) {
	case WIFI_FREQ_BANDWIDTH_20MHZ:
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BW_20);
		break;
	case WIFI_FREQ_BANDWIDTH_40MHZ:
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BW_40);
		break;
	case WIFI_FREQ_BANDWIDTH_80MHZ:
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BW_80);
		break;
	default:
		chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BW_20);
		LOG_WRN("Discard of setting unsupported bandwidth: %u (will set 20MHz)",
			params->bandwidth);
		break;
	}
#else
	ARG_UNUSED(params->bandwidth);
#endif

	switch (params->security) {
	case WIFI_SECURITY_TYPE_NONE:
		security = WHD_SECURITY_OPEN;
		break;
	case WIFI_SECURITY_TYPE_PSK:
		security = WHD_SECURITY_WPA2_AES_PSK;
		break;
	case WIFI_SECURITY_TYPE_SAE:
		security = WHD_SECURITY_WPA3_SAE;
		break;
	default:
		goto error;
	}

	if (whd_wifi_init_ap(ap_if, &ssid, security, (const uint8_t *)params->psk,
			     params->psk_length, chanspec) != 0) {
		LOG_ERR("Failed to init whd ap interface");
		ret = -EAGAIN;
		goto error;
	}

	/* Keep radio awake for SoftAP beaconing. Chip init enables PM2 for
	 * CYW55500; BSS-up can reassert sleep — set before and after start_ap.
	 */
	(void)whd_wifi_ap_set_beacon_interval(ap_if, 100);
#if defined(CONFIG_AIROC_WIFI6)
	/* set_max_assoc is WIFI6-only in this WHD tree */
	(void)whd_wifi_ap_set_max_assoc(ap_if, 8);
#endif
	(void)whd_wifi_set_iovar_value(ap_if, IOVAR_STR_MPC, 0);
	(void)whd_wifi_disable_powersave(ap_if);
	(void)whd_wifi_set_ioctl_value(ap_if, WLC_SET_PM, 0);

	if (whd_wifi_start_ap(ap_if) != 0) {
		LOG_ERR("Failed to start whd ap interface");
		ret = -EAGAIN;
		goto error;
	}

	/* Re-assert after BSS-up; FW may restore PM/mpc during start_ap */
	(void)whd_wifi_set_iovar_value(ap_if, IOVAR_STR_MPC, 0);
	(void)whd_wifi_disable_powersave(ap_if);
	(void)whd_wifi_set_ioctl_value(ap_if, WLC_SET_PM, 0);

	/* set event handler */
	if (whd_management_set_event_handler(ap_if, ap_link_events,
					     airoc_wifi_ap_link_events_handler, NULL,
					     &ap_event_handler_index) != 0) {
		whd_wifi_stop_ap(ap_if);
		ret = -EAGAIN;
		goto error;
	}

	data->is_ap_up = true;
	airoc_active_ap_if = ap_if;
	airoc_if = ap_if;
	net_if_dormant_off(data->iface);
error:

	k_sem_give(&data->sema_common);
	return ret;
}

#if defined(CONFIG_NET_STATISTICS_WIFI)
static int airoc_mgmt_wifi_stats(const struct device *dev, struct net_stats_wifi *stats)
{
	struct airoc_wifi_data *data = dev->data;

	stats->bytes.received = data->stats.bytes.received;
	stats->bytes.sent = data->stats.bytes.sent;
	stats->pkts.rx = data->stats.pkts.rx;
	stats->pkts.tx = data->stats.pkts.tx;
	stats->errors.rx = data->stats.errors.rx;
	stats->errors.tx = data->stats.errors.tx;
	stats->broadcast.rx = data->stats.broadcast.rx;
	stats->broadcast.tx = data->stats.broadcast.tx;
	stats->multicast.rx = data->stats.multicast.rx;
	stats->multicast.tx = data->stats.multicast.tx;
	stats->sta_mgmt.beacons_rx = data->stats.sta_mgmt.beacons_rx;
	stats->sta_mgmt.beacons_miss = data->stats.sta_mgmt.beacons_miss;

	return 0;
}
#endif

static int airoc_mgmt_ap_disable(const struct device *dev)
{
	cy_rslt_t whd_ret;
	struct airoc_wifi_data *data = dev->data;
	whd_interface_t ap_if = data->is_ap_up ? airoc_active_ap_if :
				 (airoc_ap_on_primary ? airoc_sta_if : airoc_ap_if);

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (ap_event_handler_index != 0xFF) {
		(void)whd_wifi_deregister_event_handler(ap_if, ap_event_handler_index);
		ap_event_handler_index = 0xFF;
	}
	if (ap_event_handler_index_prim != 0xFF) {
		(void)whd_wifi_deregister_event_handler(airoc_sta_if,
							ap_event_handler_index_prim);
		ap_event_handler_index_prim = 0xFF;
	}

	whd_ret = whd_wifi_stop_ap(ap_if);
	if (whd_ret == CY_RSLT_SUCCESS) {
		data->is_ap_up = false;
		airoc_active_ap_if = NULL;
		airoc_ap_on_primary = false;
		airoc_if = airoc_sta_if;
		net_if_dormant_on(data->iface);
	} else {
		LOG_ERR("Can't stop wifi ap: %u", whd_ret);
	}

	k_sem_give(&data->sema_common);

	if (whd_ret != CY_RSLT_SUCCESS) {
		return -ENODEV;
	}

	return 0;
}

#if defined(CONFIG_AIROC_WIFI_P2P)
static whd_interface_t airoc_go_if;
static bool airoc_go_on_primary; /* true when GO uses primary SoftAP path */
#if !defined(CONFIG_AIROC_WIFI6)
static bool airoc_go_tertiary_init; /* SoftAP on bsscfg2 / ifidx1 */
#endif

int airoc_wifi_go_softap_start(const char *ssid_str, size_t ssid_len,
			       const char *psk, size_t psk_len,
			       uint8_t channel, const whd_mac_t *p2p_if_mac,
			       whd_interface_t *out_if)
{
	struct airoc_wifi_data *data = &airoc_wifi_data;
	whd_ssid_t ssid;
	whd_security_t security;
	whd_interface_t ap_if;
	uint16_t chanspec;
	int ret = 0;
	const char *if_label;

	if ((ssid_str == NULL) || (ssid_len == 0) || (ssid_len > sizeof(ssid.value))) {
		return -EINVAL;
	}

	/* psk_len == 0 → open network */
	if ((psk_len > 0) && (psk == NULL)) {
		return -EINVAL;
	}

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (data->is_sta_connected) {
		LOG_ERR("Network interface is busy in STA mode. Please first disconnect STA.");
		ret = -EBUSY;
		goto error;
	}

	if (data->is_ap_up) {
		LOG_ERR("Already AP is on - first disable");
		ret = -EALREADY;
		goto error;
	}

	/*
	 * WIFI6: primary SoftAP.
	 * WIFI5 + p2p_ifadd: tertiary bsscfg2 (p2p_disc keeps bsscfg1).
	 * WIFI5 without p2p_ifadd: secondary SoftAP (known-good).
	 */
#if defined(CONFIG_AIROC_WIFI6)
	ARG_UNUSED(p2p_if_mac);
	ap_if = airoc_sta_if;
	airoc_go_on_primary = true;
	airoc_go_if = ap_if;
	if_label = "primary";
#else
	if (p2p_if_mac != NULL) {
		whd_mac_t go_mac = *p2p_if_mac;

		/*
		 * Host IF lives in WHD iflist[2]. Re-bind every start:
		 * add is idempotent and returns the existing entry.
		 */
		if (whd_add_interface(data->whd_drv, AIROC_P2P_BSSCFG_GO,
				      AIROC_P2P_IFIDX_GO, AIROC_P2P_GO_IFNAME,
				      &go_mac,
				      &airoc_go_if) != WHD_SUCCESS ||
		    airoc_go_if == NULL) {
			LOG_ERR("Unable to bring up tertiary IF for P2P GO");
			ret = -EAGAIN;
			goto error;
		}
		airoc_go_tertiary_init = true;
		ap_if = airoc_go_if;
		airoc_go_on_primary = false;
		if_label = "tertiary(bsscfg2/ifidx1)";
	} else {
		if (!data->second_interface_init) {
			if (whd_add_secondary_interface(data->whd_drv, NULL,
						       &airoc_ap_if) !=
			    WHD_SUCCESS) {
				LOG_ERR("Unable to bring up secondary IF for P2P GO");
				ret = -EAGAIN;
				goto error;
			}
			data->second_interface_init = true;
		}
		ap_if = airoc_ap_if;
		airoc_go_if = ap_if;
		airoc_go_on_primary = false;
		if_label = "secondary(bsscfg1)";
	}
#endif

	if (ap_if == NULL) {
		LOG_ERR("P2P GO SoftAP IF is NULL");
		ret = -ENODEV;
		goto error;
	}

	(void)whd_wifi_set_iovar_value(ap_if, IOVAR_STR_MPC, 0);

	memset(&ssid, 0, sizeof(ssid));
	ssid.length = ssid_len;
	memcpy(ssid.value, ssid_str, ssid.length);

	/* Same WIFI5 vs WIFI6 chanspec rule as airoc_mgmt_ap_enable. */
	if ((channel > 0) && (channel < 12)) {
		chanspec = channel;
	} else {
		chanspec = 6;
		LOG_WRN("Unsupported GO channel %u — using 6", channel);
	}
#if defined(CONFIG_AIROC_WIFI6)
	chanspec |= GET_C_VAR(ap_if->whd_driver, CHANSPEC_BAND_2G) |
		    GET_C_VAR(ap_if->whd_driver, CHANSPEC_BW_20);
#endif

	security = (psk_len == 0) ? WHD_SECURITY_OPEN : WHD_SECURITY_WPA2_AES_PSK;

	if (whd_wifi_init_ap(ap_if, &ssid, security,
			     (const uint8_t *)psk, psk_len, chanspec) != 0) {
		LOG_ERR("Failed to init whd ap interface for P2P GO");
		ret = -EAGAIN;
		goto error;
	}

	/* Must be set before start_ap to take effect */
	(void)whd_wifi_ap_set_beacon_interval(ap_if, 100);
#if defined(CONFIG_AIROC_WIFI6)
	(void)whd_wifi_ap_set_max_assoc(ap_if, 8);
#endif
	(void)whd_wifi_disable_powersave(ap_if);
	(void)whd_wifi_set_ioctl_value(ap_if, WLC_SET_PM, 0);

	if (whd_wifi_start_ap(ap_if) != 0) {
		LOG_ERR("Failed to start whd ap interface for P2P GO");
		ret = -EAGAIN;
		goto error;
	}

	/* FW may re-enable mpc/PM during start_ap — keep SoftAP awake for Auth */
	(void)whd_wifi_set_iovar_value(ap_if, IOVAR_STR_MPC, 0);
	(void)whd_wifi_disable_powersave(ap_if);
	(void)whd_wifi_set_ioctl_value(ap_if, WLC_SET_PM, 0);

	if (whd_management_set_event_handler(ap_if, ap_link_events,
					     airoc_wifi_ap_link_events_handler, NULL,
					     &ap_event_handler_index) != 0) {
		whd_wifi_stop_ap(ap_if);
		ret = -EAGAIN;
		goto error;
	}
	ap_event_handler_index_prim = 0xFF;

	data->is_ap_up = true;
	airoc_if = ap_if;
	net_if_dormant_off(data->iface);
	(void)net_if_up(data->iface);

	if (out_if != NULL) {
		*out_if = ap_if;
	}

	LOG_INF("P2P GO SoftAP up (%s)", if_label);
error:
	k_sem_give(&data->sema_common);
	return ret;
}

int airoc_wifi_go_softap_stop(void)
{
	cy_rslt_t whd_ret;
	struct airoc_wifi_data *data = &airoc_wifi_data;
	whd_interface_t ap_if = airoc_go_if ? airoc_go_if : airoc_sta_if;

	if (k_sem_take(&data->sema_common, K_MSEC(AIROC_WIFI_WAIT_SEMA_MS)) != 0) {
		return -EAGAIN;
	}

	if (!data->is_ap_up) {
		k_sem_give(&data->sema_common);
		return -ENOENT;
	}

	if (ap_event_handler_index != 0xFF) {
		(void)whd_wifi_deregister_event_handler(ap_if, ap_event_handler_index);
		ap_event_handler_index = 0xFF;
	}
	if (ap_event_handler_index_prim != 0xFF) {
		(void)whd_wifi_deregister_event_handler(airoc_sta_if,
							ap_event_handler_index_prim);
		ap_event_handler_index_prim = 0xFF;
	}

	whd_ret = whd_wifi_stop_ap(ap_if);
	/*
	 * Always clear host AP state. P2P GO BSS_DOWN/ifdel often fails in
	 * firmware; leaving is_ap_up set blocks the next group add.
	 */
	data->is_ap_up = false;
	airoc_if = airoc_sta_if;
	airoc_go_on_primary = false;
	if (whd_ret != CY_RSLT_SUCCESS) {
		LOG_WRN("P2P GO softAP stop returned %u — host state cleared",
			(unsigned int)whd_ret);
	}
	/*
	 * Do not net_if_dormant_on(): this iface is also STA/offload. Flapping
	 * it DOWN drops IPv4 mcast membership and races DHCPv4 server restart.
	 */

	k_sem_give(&data->sema_common);

	return 0;
}
#endif /* CONFIG_AIROC_WIFI_P2P */

static int airoc_iface_status(const struct device *dev, struct wifi_iface_status *status)
{
	struct airoc_wifi_data *data = dev->data;
	whd_result_t result;
	wl_bss_info_t bss_info;
	whd_security_t security_info = 0;
	uint32_t wpa_data_rate_value = 0;
	uint32_t join_status;

	if (airoc_if == NULL) {
		return -ENOTSUP;
	}

	status->iface_mode =
		(data->is_ap_up ? WIFI_MODE_AP
				: (data->is_sta_connected ? WIFI_MODE_INFRA : WIFI_MODE_UNKNOWN));

	join_status = whd_wifi_is_ready_to_transceive(airoc_if);

	if (join_status == WHD_SUCCESS) {
		status->state = WIFI_STATE_COMPLETED;
	} else if (join_status == WHD_JOIN_IN_PROGRESS) {
		status->state = WIFI_STATE_ASSOCIATING;
	} else if (join_status == WHD_NOT_KEYED) {
		status->state = WIFI_STATE_AUTHENTICATING;
	} else {
		status->state = WIFI_STATE_DISCONNECTED;
	}

	result = whd_wifi_get_ap_info(airoc_if, &bss_info, &security_info);

	if (result == WHD_SUCCESS) {
		memcpy(&(status->bssid[0]), &(bss_info.BSSID), sizeof(whd_mac_t));

		whd_wifi_get_channel(airoc_if, (int *)&status->channel);

		status->band = (status->channel <= CH_MAX_2G_CHANNEL) ? WIFI_FREQ_BAND_2_4_GHZ
								      : WIFI_FREQ_BAND_5_GHZ;

		status->rssi = (int)bss_info.RSSI;

		status->ssid_len = bss_info.SSID_len;
		strncpy(status->ssid, bss_info.SSID, status->ssid_len);

		status->security = convert_whd_security_to_zephyr(security_info);

		status->beacon_interval = (unsigned short)bss_info.beacon_period;
		status->dtim_period = (unsigned char)bss_info.dtim_period;

		status->twt_capable = false;
	}

	whd_wifi_get_ioctl_value(airoc_if, WLC_GET_RATE, &wpa_data_rate_value);
	status->current_phy_tx_rate = wpa_data_rate_value;

	/* Unbelievably, this appears to be the only way to determine the phy mode with
	 *  the whd SDK that we're currently using. Note that the logic below is only valid on
	 *  devices that are limited to the 2.4Ghz band. Other versions of the SDK and chip
	 * evidently allow one to obtain a phy_mode value directly from bss_info
	 */
	if (wpa_data_rate_value > 54) {
		status->link_mode = WIFI_4;
	} else if (wpa_data_rate_value == 6 || wpa_data_rate_value == 9 ||
		   wpa_data_rate_value == 12 || wpa_data_rate_value == 18 ||
		   wpa_data_rate_value == 24 || wpa_data_rate_value == 36 ||
		   wpa_data_rate_value == 48 || wpa_data_rate_value == 54) {
		status->link_mode = WIFI_3;
	} else {
		status->link_mode = WIFI_1;
	}

	return 0;
}

static int airoc_init(const struct device *dev)
{
	int ret;
	cy_rslt_t whd_ret;
	struct airoc_wifi_data *data = dev->data;

	k_tid_t tid = k_thread_create(
		&airoc_wifi_event_thread, airoc_wifi_event_stack,
		CONFIG_AIROC_WIFI_EVENT_TASK_STACK_SIZE, (k_thread_entry_t)airoc_event_task, NULL,
		NULL, NULL, CONFIG_AIROC_WIFI_EVENT_TASK_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

	if (!tid) {
		LOG_ERR("ERROR spawning tx thread");
		return -EAGAIN;
	}
	k_thread_name_set(tid, "airoc_event");

	whd_ret = airoc_wifi_init_primary(dev, &airoc_sta_if, &airoc_wifi_netif_if_default,
					  &airoc_wifi_buffer_if_default);
	if (whd_ret != CY_RSLT_SUCCESS) {
		LOG_ERR("airoc_wifi_init_primary failed ret = %d \r\n", whd_ret);
		return -EAGAIN;
	}
	airoc_if = airoc_sta_if;

	whd_ret = whd_management_set_event_handler(airoc_sta_if, sta_link_events,
				link_events_handler, NULL, &sta_event_handler_index);
	if (whd_ret != CY_RSLT_SUCCESS) {
		LOG_ERR("whd_management_set_event_handler failed ret = %d \r\n", whd_ret);
		return -EAGAIN;
	}

	ret = k_sem_init(&data->sema_common, 1, 1);
	if (ret != 0) {
		LOG_ERR("k_sem_init(sema_common) failure");
		return ret;
	}

	ret = k_sem_init(&data->sema_scan, 0, 1);
	if (ret != 0) {
		LOG_ERR("k_sem_init(sema_scan) failure");
		return ret;
	}

#if defined(CONFIG_AIROC_WIFI_P2P)
	ret = airoc_p2p_init(data->whd_drv, airoc_sta_if);
	if (ret == 0) {
		data->p2p_initialized = true;
	} else {
		LOG_WRN("P2P init failed: %d (continuing without P2P)", ret);
	}
#endif

	return 0;
}

static const struct wifi_mgmt_ops airoc_wifi_mgmt = {
	.scan = airoc_mgmt_scan,
	.connect = airoc_mgmt_connect,
	.disconnect = airoc_mgmt_disconnect,
	.ap_enable = airoc_mgmt_ap_enable,
	.ap_disable = airoc_mgmt_ap_disable,
	.iface_status = airoc_iface_status,
#if defined(CONFIG_NET_STATISTICS_WIFI)
	.get_stats = airoc_mgmt_wifi_stats,
#endif
#if defined(CONFIG_AIROC_WIFI_P2P)
	.p2p_oper = airoc_p2p_oper,
#endif
};

static const struct net_wifi_mgmt_offload airoc_api = {
	.wifi_iface.iface_api.init = airoc_mgmt_init,
	.wifi_iface.send = airoc_mgmt_send,
	.wifi_iface.get_capabilities = airoc_get_capabilities,
	.wifi_iface.set_config = airoc_set_config,
	.wifi_mgmt_api = &airoc_wifi_mgmt,
};

NET_DEVICE_DT_INST_DEFINE(0, airoc_init, NULL, &airoc_wifi_data, &airoc_wifi_config,
			  CONFIG_WIFI_INIT_PRIORITY, &airoc_api, ETHERNET_L2,
			  NET_L2_GET_CTX_TYPE(ETHERNET_L2), WHD_LINK_MTU);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
