/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AIROC_WIFI_P2P_PRIV_H
#define AIROC_WIFI_P2P_PRIV_H

#include "airoc_wifi_p2p.h"

#include <stddef.h>
#include <string.h>
#include <whd_int.h>
#include <whd_wifi_api.h>

#define AIROC_P2P_DEV_NAME          CONFIG_AIROC_WIFI_P2P_DEVICE_NAME
#define AIROC_P2P_GO_SSID           CONFIG_AIROC_WIFI_P2P_GO_SSID
#define AIROC_P2P_ADV_SSID          CONFIG_AIROC_WIFI_P2P_GO_SSID
#define AIROC_P2P_GO_PSK            CONFIG_AIROC_WIFI_P2P_GO_PSK
#define AIROC_P2P_GO_IP_ADDR        CONFIG_AIROC_WIFI_P2P_GO_IP_ADDR
#define AIROC_P2P_GO_NETMASK        CONFIG_AIROC_WIFI_P2P_GO_NETMASK
#define AIROC_P2P_GO_DHCP_POOL_BASE CONFIG_AIROC_WIFI_P2P_GO_DHCP_POOL_BASE

#define AIROC_P2P_GO_USE_OPEN        0

#define AIROC_WPS_RESP_ENROLLEE      0x01
#define AIROC_WPS_RESP_AP            0x03
#define AIROC_WPS_DEV_PWD_PBC        0x0004
#define AIROC_WPS_CONFIG_PBC         0x0080
#define AIROC_WPS_CONFIG_PBC_DISPLAY 0x0180

#define AIROC_P2P_PUB_AF_CATEGORY    0x04
#define AIROC_P2P_PUB_AF_ACTION      0x09
#define AIROC_P2P_OUI_TYPE           0x09
#define AIROC_P2P_PAF_PD_REQ         0x07
#define AIROC_P2P_PAF_PD_RSP         0x08
#define AIROC_P2P_GROUP_CAP_GO       0x01
#define AIROC_P2P_DOT11_MGMT_HDR_LEN 24
#define AIROC_P2P_AF_MAX_LEN         256

#define AIROC_P2P_WILDCARD_SSID      "DIRECT-"
#define AIROC_P2P_WILDCARD_SSID_LEN  7
#define AIROC_P2P_SOCIAL_CHAN_CNT    3
#define AIROC_P2P_HOME_TIME_MS       60
#define AIROC_P2P_SOCIAL_DWELL_MS    40
#define AIROC_P2P_NPROBES            2

#define AIROC_P2P_ESCAN_BUF_SIZE \
	(4 + offsetof(wl_escan_params_t, params) + WL_SCAN_PARAMS_FIXED_SIZE + \
	 (AIROC_P2P_SOCIAL_CHAN_CNT * sizeof(uint16_t)))

static inline void p2p_put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)(v & 0xff);
}

static inline void p2p_put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xff);
	p[1] = (uint8_t)(v >> 8);
}

/* Shared driver state (defined in airoc_wifi_p2p.c) */
extern struct airoc_p2p_ctx p2p_ctx;
extern whd_driver_t p2p_whd_drv;
extern whd_interface_t p2p_sta_if;
extern whd_interface_t p2p_disc_bss_if;
extern bool p2p_ies_installed;
extern bool p2p_ies_on_disc_bss;
extern bool p2p_go_wps_pbc_active;
extern uint8_t p2p_wps_ie_data[128];
extern uint16_t p2p_wps_ie_len;
extern uint8_t p2p_p2p_ie_data[96];
extern uint16_t p2p_p2p_ie_len;
extern uint8_t p2p_disc_wps_ie_data[128];
extern uint16_t p2p_disc_wps_ie_len;
extern uint8_t p2p_disc_dev_mac[6];
extern bool p2p_disc_dev_mac_valid;

whd_interface_t p2p_bsscfg_shadow(whd_interface_t src, uint8_t bsscfg,
				  struct whd_interface *shadow);
int p2p_bsscfg_set_int(whd_interface_t ifp, uint8_t bsscfg,
		       const char *name, uint32_t value);
int p2p_bsscfg_get_mac(whd_interface_t ifp, uint8_t bsscfg, uint8_t mac_out[6]);
void p2p_remember_disc_mac(const uint8_t mac[6]);
void p2p_derive_disc_mac_from_go(const uint8_t go_mac[6]);
int p2p_set_state(whd_interface_t ifp, uint8_t state, uint16_t chanspec,
		  uint16_t dwell);

uint16_t p2p_build_wps_attrs(uint8_t *buf, size_t buflen, const char *name,
			     bool as_ap, bool sel_reg_pbc,
			     uint16_t config_methods);
uint16_t p2p_build_p2p_attrs(uint8_t *buf, size_t buflen,
			     const uint8_t *dev_mac, const char *name,
			     uint8_t group_cap);
int p2p_ie_add(whd_interface_t ifp, const uint8_t *oui, uint8_t subtype,
	       const uint8_t *data, uint16_t len, uint16_t pktflag);
int p2p_ie_add_wps_p2p(whd_interface_t ifp, uint16_t pktflag);
void p2p_store_disc_wps_blob(void);
int p2p_build_discovery_ie_blobs(whd_mac_t *mac_out, uint8_t group_cap);
int p2p_install_discovery_ies(void);
int p2p_install_go_ies(whd_interface_t ap_ifp);
void p2p_remove_discovery_ies(void);

#endif /* AIROC_WIFI_P2P_PRIV_H */
