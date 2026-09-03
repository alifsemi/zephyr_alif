/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief AIROC Wi-Fi P2P (Wi-Fi Direct) implementation.
 *
 * All P2P logic is driven via WHD IOVARs — no hostap / wpa_supplicant.
 * Autonomous Group Owner only (no P2P client / GO negotiation).
 * State machine: IDLE → DISCOVERING / LISTENING → GROUP_FORMED.
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <string.h>
#include <stdio.h>
#include <airoc_wifi.h>
#include <airoc_wifi_p2p_priv.h>
#include <airoc_wifi_wps_reg.h>
#include <airoc_whd_hal_common.h>
#include <whd_wlioctl.h>
#include <whd_wifi_api.h>
#include <whd_wifi_p2p.h>
#include <whd_int.h>
#include <whd_endian.h>
#include <whd_types.h>

#if defined(CONFIG_NET_DHCPV4_SERVER)
#include <zephyr/net/dhcpv4_server.h>
#endif

LOG_MODULE_DECLARE(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

/* WHD headers omit pack; firmware + hosts use 1-byte pad after state
 * (same as brcmf_p2p_disc_st_le / natural ARM alignment = 6 bytes).
 * A 5-byte packed layout only breaks LISTEN (chanspec is ignored for SEARCH).
 */
struct __packed airoc_wl_p2p_disc_st {
	uint8_t state;
	uint8_t reserved;
	uint16_t chanspec;
	uint16_t dwell_time_ms;
};

/* Firmware / brcmfmac expect 10 bytes: mac(6) + type(le16) + chanspec(le16).
 * A 9-byte packed uint8 type yields WHD_WLAN_BADARG on p2p_ifadd.
 */
struct __packed airoc_wl_p2p_if {
	uint8_t mac[6];
	uint16_t interface_type;
	uint16_t chan_spec;
};

/* ------------------------------------------------------------------ */
/* Static context                                                      */
/* ------------------------------------------------------------------ */
struct airoc_p2p_ctx p2p_ctx;
whd_driver_t  p2p_whd_drv;
whd_interface_t p2p_sta_if;   /* primary STA interface */
whd_interface_t p2p_disc_bss_if; /* primary ifp used with bsscfg override */
bool p2p_ies_installed;
bool p2p_ies_on_disc_bss; /* true if IEs landed on bsscfg 1 (needed for phone) */
bool p2p_go_wps_pbc_active;
static uint32_t p2p_go_last_client_count = 0xffffffffU;
uint8_t p2p_wps_ie_data[128];
uint16_t p2p_wps_ie_len;
uint8_t p2p_p2p_ie_data[96];
uint16_t p2p_p2p_ie_len;
/* WPS blob currently installed on disc bsscfg1 (may differ from GO blob). */
uint8_t p2p_disc_wps_ie_data[128];
uint16_t p2p_disc_wps_ie_len;
/* P2P device address on discovery BSS (phone addresses PD here). */
uint8_t p2p_disc_dev_mac[6];
bool p2p_disc_dev_mac_valid;

/* Pending Provision Discovery Request (phone → GO) */
static struct {
	bool pending;
	uint8_t peer[6];
	uint8_t our_addr[6]; /* RA of the request: our device or GO address */
	uint8_t bsscfg;      /* BSSCFG the request arrived on (1=disc, 2=GO) */
	uint8_t dialog_token;
	uint16_t channel;
	uint16_t config_methods; /* echoed back in the PD Response */
} p2p_pd_req;

/* WHD event callback vs delayed work; not p2p_ctx.lock (callback must not block). */
static struct k_spinlock p2p_pd_lock;

/* Dedup WLC_E_ACTION_FRAME (59) + WLC_E_ACTION_FRAME_RX (75) for same PD */
static struct {
	uint8_t peer[6];
	uint8_t dialog;
	int64_t at_ms;
} p2p_pd_dedup;

/* Delayable so event 75 (full 802.11 hdr) can fill RA before we TX PD RSP. */
static struct k_work_delayable p2p_af_work;

/* Event registration for the group interface (ifidx 1); p2p_ctx.event_index
 * holds the primary-interface (ifidx 0) registration.
 */
static uint16_t p2p_grp_event_index = 0xFF;

/* P2P events we want to receive from firmware */
static const whd_event_num_t p2p_events[] = {
	WLC_E_P2P_DISC_LISTEN_COMPLETE,
	WLC_E_P2P_PROBREQ_MSG,
	WLC_E_PROBREQ_MSG,
	WLC_E_ACTION_FRAME,
	75, /* WLC_E_ACTION_FRAME_RX — frame + wl_event_rx_frame_data_t */
	WLC_E_ACTION_FRAME_COMPLETE,
	WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE,
	WLC_E_ESCAN_RESULT,
	WLC_E_P2PO_ADD_DEVICE,
	WLC_E_P2PO_DEL_DEVICE,
	WLC_E_NONE
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */
static void p2p_disc_timeout_handler(struct k_work *work);
static void p2p_event_work_handler(struct k_work *work);
static void p2p_af_work_handler(struct k_work *work);
static void *p2p_event_handler(whd_interface_t ifp,
			       const whd_event_header_t *event_header,
			       const uint8_t *event_data,
			       void *handler_user_data);
static int p2p_stop_find(void);
int p2p_set_state(whd_interface_t ifp, uint8_t state,
			 uint16_t chanspec, uint16_t dwell_ms);
static int p2p_set_discovery(whd_interface_t ifp, bool enable);
static int p2p_ensure_events(whd_interface_t ifp);

/* ------------------------------------------------------------------ */
/* Helper: build a 2.4 GHz chanspec for a given channel number         */
/* ------------------------------------------------------------------ */
static uint16_t p2p_make_chanspec_2g(whd_driver_t drv, uint16_t channel)
{
#if defined(CONFIG_AIROC_WIFI6)
	return (uint16_t)(channel |
			  GET_C_VAR(drv, CHANSPEC_BAND_2G) |
			  GET_C_VAR(drv, CHANSPEC_BW_20) |
			  GET_C_VAR(drv, CHANSPEC_CTL_SB_NONE));
#else
	/* WIFI5 SoftAP path: plain channel through CH20MHZ_CHSPEC.
	 * Macro expands using local name "whd_driver".
	 */
	whd_driver_t whd_driver = drv;

	return (uint16_t)CH20MHZ_CHSPEC(channel);
#endif
}


/**
 * P2P discovery BSSCFG is bsscfg=1 on the SAME ifidx as primary (0).
 * Never mutate the live STA ifp->bsscfgidx — concurrent WHD/event paths
 * use that pointer and racing overrides break p2p_state / p2p_scan.
 */
whd_interface_t p2p_bsscfg_shadow(whd_interface_t src, uint8_t bsscfg,
					 struct whd_interface *shadow)
{
	*shadow = *src;
	shadow->bsscfgidx = bsscfg;
	return shadow;
}

/** bsscfg-qualified int set (Linux brcmf_fil_bsscfg_int_set). */
int p2p_bsscfg_set_int(whd_interface_t ifp, uint8_t bsscfg,
			      const char *name, uint32_t value)
{
	char iovar_name[32];
	uint32_t data[2];
	whd_result_t ret;
	int n;

	n = snprintf(iovar_name, sizeof(iovar_name), "bsscfg:%s", name);
	if (n <= 0 || n >= (int)sizeof(iovar_name)) {
		return -EINVAL;
	}

	data[0] = htod32(bsscfg);
	data[1] = htod32(value);
	ret = whd_wifi_set_iovar_buffer(ifp, iovar_name, data, sizeof(data));
	if (ret != WHD_SUCCESS) {
		LOG_DBG("bsscfg:%s(%u)=%u failed: %d (0x%x)",
			name, bsscfg, value, ret, ret);
		return -EIO;
	}
	return 0;
}

/** Read MAC of a BSSCFG (discovery device address lives on bsscfg 1). */
int p2p_bsscfg_get_mac(whd_interface_t ifp, uint8_t bsscfg,
			      uint8_t mac_out[6])
{
	uint32_t idx;
	whd_result_t ret;

	if (ifp == NULL || mac_out == NULL) {
		return -EINVAL;
	}

	idx = htod32(bsscfg);
	ret = whd_wifi_get_iovar_buffer_with_param(ifp, "bsscfg:cur_etheraddr",
						   &idx, sizeof(idx), mac_out,
						   6);
	if (ret != WHD_SUCCESS || NULL_MAC(mac_out)) {
		return -EIO;
	}
	return 0;
}

void p2p_remember_disc_mac(const uint8_t mac[6])
{
	if (mac == NULL || NULL_MAC(mac)) {
		return;
	}
	memcpy(p2p_disc_dev_mac, mac, 6);
	p2p_disc_dev_mac_valid = true;
}

/**
 * Derive P2P device address from GO SoftAP BSSID (brcmfmac-style):
 * LAA bit set + bit7 of octet[4] flipped. Used when bsscfg:cur_etheraddr
 * on disc fails (common on CYW43439).
 */
void p2p_derive_disc_mac_from_go(const uint8_t go_mac[6])
{
	uint8_t disc[6];

	if (go_mac == NULL || NULL_MAC(go_mac) || p2p_disc_dev_mac_valid) {
		return;
	}
	memcpy(disc, go_mac, 6);
	disc[0] |= 0x02;
	disc[4] ^= 0x80;
	p2p_remember_disc_mac(disc);
}

/**
 * CYW43439 P2P LISTEN on bsscfg1 shares the single radio with SoftAP.
 * Continuous LISTEN prevents SoftAP from answering Auth after PD — park
 * discovery in SCAN while WPS-PBC join is armed.
 */
static void p2p_go_park_disc_for_assoc(void)
{
	uint8_t go_ch;
	uint16_t chanspec;

	if (!p2p_ctx.disc_enabled || p2p_sta_if == NULL) {
		return;
	}

	go_ch = p2p_ctx.go_channel ? p2p_ctx.go_channel : AIROC_P2P_SOCIAL_CH_6;
	chanspec = p2p_make_chanspec_2g(p2p_whd_drv, go_ch);
	(void)p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SCAN, chanspec, 0);
}

/**
 * Locate 802.11 Action frame in ACTION_FRAME_RX payload.
 * Firmware prefixes wl_event_rx_frame_data (commonly 12B packed; some trees
 * use 16B). Scan known offsets for mgmt Action FC=0xd0.
 */
static bool p2p_locate_dot11_action(const uint8_t *data, uint32_t datalen,
				    const uint8_t **dot11_out,
				    const uint8_t **body_out, uint32_t *blen_out,
				    uint16_t *ch_inout)
{
	static const uint8_t pref[] = { 12, 16, 8, 0 };
	size_t i;

	if (data == NULL || datalen < AIROC_P2P_DOT11_MGMT_HDR_LEN) {
		return false;
	}

	for (i = 0; i < ARRAY_SIZE(pref); i++) {
		uint8_t off = pref[i];

		if ((uint32_t)off + AIROC_P2P_DOT11_MGMT_HDR_LEN + 8u > datalen) {
			continue;
		}
		if ((data[off] & 0xfc) != 0xd0) {
			continue;
		}
		*dot11_out = data + off;
		*body_out = data + off + AIROC_P2P_DOT11_MGMT_HDR_LEN;
		*blen_out = datalen - off - AIROC_P2P_DOT11_MGMT_HDR_LEN;
		if (off >= 4 && ch_inout != NULL) {
			uint16_t rxch = (uint16_t)data[2] |
					((uint16_t)data[3] << 8);

			if ((rxch & 0xff) >= 1 && (rxch & 0xff) <= 14) {
				*ch_inout = rxch & 0xff;
			}
		}
		return true;
	}
	return false;
}


/**
 * Action-frame TX scoped to a BSSCFG (Linux brcmf_p2p_tx_action_frame).
 * The plain "actframe" iovar always uses the BSS of the addressed ifidx, which
 * is the STA BSS for anything on ifidx 0, so the discovery BSS needs the
 * bsscfg-qualified form to transmit from the P2P device address.
 */
static int p2p_bsscfg_send_af(whd_interface_t ifp, uint8_t bsscfg,
			      const whd_af_params_t *af)
{
	static uint8_t buf[4 + sizeof(whd_af_params_t)] __aligned(4);
	uint32_t idx = htod32(bsscfg);
	whd_result_t ret;

	if (ifp == NULL || af == NULL) {
		return -ENODEV;
	}

	memcpy(buf, &idx, sizeof(idx));
	memcpy(buf + 4, af, sizeof(*af));

	ret = whd_wifi_set_iovar_buffer(ifp, "bsscfg:actframe", buf,
					sizeof(buf));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("bsscfg:actframe(%u) failed: %d (0x%x)", bsscfg, ret,
			ret);
		return -EIO;
	}
	return 0;
}


/**
 * Let a WPS enrollee associate without an RSN IE. Android's Wi-Fi Direct
 * client associates open first to run EAP-WSC and collect credentials; a
 * plain WPA2-PSK BSS answers that AssocReq with status 12.
 */
static int p2p_go_allow_wps_assoc(whd_interface_t ap_ifp)
{
	uint8_t bsscfg;
	int err;

	if (ap_ifp == NULL || p2p_sta_if == NULL) {
		return -ENODEV;
	}
	if (AIROC_P2P_GO_USE_OPEN) {
		return 0;
	}

	bsscfg = ap_ifp->bsscfgidx;
	err = p2p_bsscfg_set_int(p2p_sta_if, bsscfg, "wsec",
				 AIROC_WSEC_AES_ENABLED |
				 AIROC_WSEC_SES_OW_ENABLED);
		if (err) {
		LOG_WRN("P2P GO: wsec SES_OW on bsscfg%u failed (%d)",
			bsscfg, err);
		return err;
	}

	return 0;
}

/**
 * After Provision Discovery (PBC), mark Selected Registrar in beacon/probe
 * so the phone knows it can start WPS join. Full WPS EAP registrar is still
 * required for credentials — this only advertises readiness.
 */
static int p2p_go_arm_wps_pbc(void)
{
	static const uint8_t wps_oui[3] = { 0x00, 0x50, 0xf2 };
	static const uint16_t pktflags[] = {
		VENDOR_IE_BEACON, VENDOR_IE_PROBE_RESPONSE,
	};
	whd_interface_t ap_ifp = p2p_ctx.group_if;
	uint8_t old_wps[128];
	uint16_t old_len;
	whd_mac_t mac;
	int err;
	size_t f;

	if (ap_ifp == NULL) {
		return -ENODEV;
	}

	old_len = p2p_wps_ie_len;
	if (old_len > 0 && old_len <= sizeof(old_wps)) {
		memcpy(old_wps, p2p_wps_ie_data, old_len);
	} else {
		old_len = 0;
	}

	p2p_go_wps_pbc_active = true;
	/* Free the radio from P2P LISTEN so SoftAP can answer Auth */
	p2p_go_park_disc_for_assoc();
	if (ap_ifp != NULL) {
		(void)whd_wifi_set_iovar_value(ap_ifp, IOVAR_STR_MPC, 0);
		(void)whd_wifi_disable_powersave(ap_ifp);
		(void)whd_wifi_set_ioctl_value(ap_ifp, WLC_SET_PM, 0);
	}

	err = p2p_build_discovery_ie_blobs(&mac, AIROC_P2P_GROUP_CAP_GO);
	if (err) {
		p2p_go_wps_pbc_active = false;
		return err;
	}

	for (f = 0; f < ARRAY_SIZE(pktflags); f++) {
		if (old_len) {
			(void)whd_wifi_manage_custom_ie(
				ap_ifp, WHD_REMOVE_CUSTOM_IE, wps_oui, AIROC_WPS_IE_OUI_TYPE,
				old_wps, old_len, pktflags[f]);
		}

		err = p2p_ie_add(ap_ifp, wps_oui, AIROC_WPS_IE_OUI_TYPE, p2p_wps_ie_data,
				 p2p_wps_ie_len, pktflags[f]);
		if (err) {
			LOG_WRN("GO WPS Selected-Registrar IE update failed "
				"(%d, flag=0x%x)", err, pktflags[f]);
			return err;
		}
	}

	if (p2p_ies_on_disc_bss && p2p_sta_if != NULL) {
		struct whd_interface shadow;
		whd_interface_t disc_ifp = p2p_bsscfg_shadow(p2p_sta_if, AIROC_P2P_BSSCFG_DISC,
							     &shadow);
		const uint8_t *rm = p2p_disc_wps_ie_len ? p2p_disc_wps_ie_data
							: old_wps;
		uint16_t rm_len = p2p_disc_wps_ie_len ? p2p_disc_wps_ie_len
						      : old_len;
		int disc_err;

		if (rm_len) {
			(void)whd_wifi_manage_custom_ie(
				disc_ifp, WHD_REMOVE_CUSTOM_IE, wps_oui, AIROC_WPS_IE_OUI_TYPE,
				rm, rm_len, VENDOR_IE_PROBE_RESPONSE);
		}
		disc_err = p2p_ie_add(disc_ifp, wps_oui, AIROC_WPS_IE_OUI_TYPE, p2p_wps_ie_data,
				      p2p_wps_ie_len, VENDOR_IE_PROBE_RESPONSE);
		if (disc_err) {
			LOG_WRN("disc WPS Selected-Registrar IE update failed "
				"(%d) — phone may keep probing without WPS join",
				disc_err);
		} else {
			p2p_store_disc_wps_blob();
		}
	}

	(void)p2p_go_allow_wps_assoc(ap_ifp);

	{
		whd_mac_t go_mac;

		if (whd_wifi_get_mac_address(ap_ifp, &go_mac) == WHD_SUCCESS) {
			(void)airoc_wps_reg_arm(go_mac.octet, AIROC_P2P_GO_SSID,
						AIROC_P2P_GO_PSK);
		}
	}

	LOG_INF("P2P GO WPS PBC armed");
	return 0;
}

/**
 * After WSC_Done: drop Selected Registrar advertisement and stop the
 * EAP-WSC registrar so the enrollee can rejoin with the PSK only.
 */
static int p2p_go_disarm_wps_pbc(void)
{
	static const uint8_t wps_oui[3] = { 0x00, 0x50, 0xf2 };
	static const uint16_t pktflags[] = {
		VENDOR_IE_BEACON, VENDOR_IE_PROBE_RESPONSE,
	};
	whd_interface_t ap_ifp = p2p_ctx.group_if;
	uint8_t old_wps[128];
	uint16_t old_len;
	whd_mac_t mac;
	size_t f;

	airoc_wps_reg_disarm();

	if (!p2p_go_wps_pbc_active) {
		return 0;
	}

	old_len = p2p_wps_ie_len;
	if (old_len > 0 && old_len <= sizeof(old_wps)) {
		memcpy(old_wps, p2p_wps_ie_data, old_len);
	} else {
		old_len = 0;
	}

	p2p_go_wps_pbc_active = false;
	if (p2p_build_discovery_ie_blobs(&mac, AIROC_P2P_GROUP_CAP_GO) != 0) {
		LOG_ERR("P2P GO: WPS disarm IE rebuild failed");
		return -EIO;
	}

	if (ap_ifp != NULL) {
		for (f = 0; f < ARRAY_SIZE(pktflags); f++) {
			if (old_len) {
				(void)whd_wifi_manage_custom_ie(
					ap_ifp, WHD_REMOVE_CUSTOM_IE, wps_oui,
					AIROC_WPS_IE_OUI_TYPE, old_wps, old_len,
					pktflags[f]);
			}
			(void)p2p_ie_add(ap_ifp, wps_oui, AIROC_WPS_IE_OUI_TYPE, p2p_wps_ie_data,
					 p2p_wps_ie_len, pktflags[f]);
		}
	}

	if (p2p_ies_on_disc_bss && p2p_sta_if != NULL) {
		struct whd_interface shadow;
		whd_interface_t disc_ifp = p2p_bsscfg_shadow(p2p_sta_if, AIROC_P2P_BSSCFG_DISC,
							     &shadow);
		const uint8_t *rm = p2p_disc_wps_ie_len ? p2p_disc_wps_ie_data
							: old_wps;
		uint16_t rm_len = p2p_disc_wps_ie_len ? p2p_disc_wps_ie_len
						      : old_len;

		if (rm_len) {
			(void)whd_wifi_manage_custom_ie(
				disc_ifp, WHD_REMOVE_CUSTOM_IE, wps_oui, AIROC_WPS_IE_OUI_TYPE,
				rm, rm_len, VENDOR_IE_PROBE_RESPONSE);
		}
		if (p2p_ie_add(disc_ifp, wps_oui, AIROC_WPS_IE_OUI_TYPE, p2p_wps_ie_data,
			       p2p_wps_ie_len, VENDOR_IE_PROBE_RESPONSE) == 0) {
			p2p_store_disc_wps_blob();
		}
	}

	LOG_INF("P2P GO: WPS PBC complete");
	return 0;
}

void airoc_p2p_wps_pbc_complete(void)
{
	(void)p2p_go_disarm_wps_pbc();
}

/**
 * SoftAP ignores ProbeReq SSID="DIRECT-" unless SoftAP SSID is exactly that.
 * Enable firmware p2p_disc LISTEN on bsscfg1 (must not share that index with
 * SoftAP — GO SoftAP uses tertiary bsscfg2 on WIFI5).
 */
static int p2p_go_enable_wildcard_listen(uint8_t channel)
{
	whd_ssid_t p2p_ssid;
	struct whd_interface shadow;
	whd_interface_t disc_ifp;
	uint16_t chanspec;
	int err;

	if (p2p_sta_if == NULL) {
		return -ENODEV;
	}
	if (channel == 0) {
		channel = AIROC_P2P_SOCIAL_CH_6;
	}

	memset(&p2p_ssid, 0, sizeof(p2p_ssid));
	p2p_ssid.length = strlen(AIROC_P2P_GO_SSID);
	if (p2p_ssid.length > sizeof(p2p_ssid.value)) {
		p2p_ssid.length = sizeof(p2p_ssid.value);
	}
	memcpy(p2p_ssid.value, AIROC_P2P_GO_SSID, p2p_ssid.length);
	(void)whd_wifi_set_iovar_buffer(p2p_sta_if, IOVAR_STR_P2P_SSID,
					&p2p_ssid, sizeof(p2p_ssid));

	err = p2p_set_discovery(p2p_sta_if, true);
	if (err) {
		LOG_ERR("P2P GO: p2p_disc enable failed (%d)", err);
		return err;
	}
	p2p_ctx.disc_enabled = true;

	/* Disc is the P2P Device — not a second GO (avoids dual list entries) */
	(void)p2p_build_discovery_ie_blobs(NULL, 0x00);
	(void)p2p_bsscfg_set_int(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, "wsec", AIROC_WSEC_AES_ENABLED);
	disc_ifp = p2p_bsscfg_shadow(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, &shadow);
	if (p2p_ie_add_wps_p2p(disc_ifp, VENDOR_IE_PROBE_RESPONSE) == 0) {
		p2p_ies_on_disc_bss = true;
		p2p_store_disc_wps_blob();
		if (p2p_bsscfg_get_mac(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, p2p_disc_dev_mac) == 0) {
			p2p_disc_dev_mac_valid = true;
		}
	}

	chanspec = p2p_make_chanspec_2g(p2p_whd_drv, channel);
	err = p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_LISTEN, chanspec,
			    5000);
	if (err) {
		LOG_WRN("GO P2P LISTEN failed (%d)", err);
		return err;
	}

	return 0;
}


/* ------------------------------------------------------------------ */
/* Low-level P2P IOVAR helpers                                         */
/* ------------------------------------------------------------------ */

/**
 * Enable or disable P2P discovery on the given interface.
 * Must be issued on the primary STA interface (firmware creates disc BSSCFG).
 */
static int p2p_set_discovery(whd_interface_t ifp, bool enable)
{
	uint32_t val = enable ? 1 : 0;
	whd_result_t ret;

	ret = whd_wifi_set_iovar_value(ifp, IOVAR_STR_P2P_DISC, val);
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_disc set %u failed: %d (0x%x)", val, ret, ret);
		return -EIO;
	}
	LOG_DBG("p2p_disc = %u", val);
	return 0;
}

/**
 * Set P2P discovery state (SCAN / LISTEN / SEARCH) — packed for firmware.
 */
int p2p_set_state(whd_interface_t ifp, uint8_t state,
			 uint16_t chanspec, uint16_t dwell_ms)
{
	struct airoc_wl_p2p_disc_st disc_st = {
		.state = state,
		.reserved = 0,
		.chanspec = htod16(chanspec),
		.dwell_time_ms = htod16(dwell_ms),
	};
	whd_result_t ret;

	ret = whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_P2P_STATE,
					&disc_st, sizeof(disc_st));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_state set (state=%u) failed: %d (0x%x) "
			"chanspec=0x%04x dwell=%u sizeof=%u",
			state, ret, ret, chanspec, dwell_ms,
			(unsigned)sizeof(disc_st));
		return -EIO;
	}
	LOG_DBG("p2p_state = %u  chanspec=0x%04x  dwell=%u ms",
		state, chanspec, dwell_ms);
	return 0;
}

/**
 * Issue a P2P escan on social channels with DIRECT- wildcard SSID.
 * Firmware requires full escan params after the 4-byte p2p_scan header.
 */
static int p2p_trigger_scan(whd_interface_t ifp)
{
	uint8_t buf[AIROC_P2P_ESCAN_BUF_SIZE];
	wl_escan_params_t *escan;
	whd_result_t ret;
	const uint16_t social[] = {
		AIROC_P2P_SOCIAL_CH_1,
		AIROC_P2P_SOCIAL_CH_6,
		AIROC_P2P_SOCIAL_CH_11,
	};

	memset(buf, 0, sizeof(buf));
	buf[0] = 'E'; /* escan */

	escan = (wl_escan_params_t *)&buf[4];
	escan->version = htod32(ESCAN_REQ_VERSION);
	escan->action = htod16(WL_SCAN_ACTION_START);
	escan->sync_id = htod16(0x1234);

	escan->params.bss_type = WL_BSSTYPE_ANY;
	escan->params.scan_type = 0; /* active */
	memset(escan->params.bssid.octet, 0xff, sizeof(escan->params.bssid));
	escan->params.nprobes = htod32(AIROC_P2P_NPROBES);
	escan->params.active_time = htod32(AIROC_P2P_SOCIAL_DWELL_MS);
	escan->params.passive_time = htod32(-1);
	escan->params.home_time = htod32(AIROC_P2P_HOME_TIME_MS);

	escan->params.ssid.SSID_len = htod32(AIROC_P2P_WILDCARD_SSID_LEN);
	memcpy(escan->params.ssid.SSID, AIROC_P2P_WILDCARD_SSID,
	       AIROC_P2P_WILDCARD_SSID_LEN);

	for (int i = 0; i < AIROC_P2P_SOCIAL_CHAN_CNT; i++) {
		escan->params.channel_list[i] =
			htod16(p2p_make_chanspec_2g(p2p_whd_drv, social[i]));
	}
	escan->params.channel_num = htod32(AIROC_P2P_SOCIAL_CHAN_CNT);

	ret = whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_P2P_SCAN,
					buf, sizeof(buf));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_scan failed: %d (0x%x) — WHD_WLAN_BADARG=0x%x",
			ret, ret, (unsigned)WHD_WLAN_BADARG);
		return -EIO;
	}
	LOG_DBG("p2p_scan (escan, social) triggered");
	return 0;
}

/**
 * Add a P2P group interface (GO or Client) — packed for firmware.
 * With p2p_disc already on bsscfg1, firmware places GO on another BSS
 * (typically bsscfg2) so SoftAP can coexist with DIRECT- probe answers.
 */
static int p2p_ifadd(whd_interface_t ifp, const whd_mac_t *mac,
		     uint8_t if_type, uint16_t chanspec)
{
	struct airoc_wl_p2p_if if_req;
	whd_result_t ret;

	memset(&if_req, 0, sizeof(if_req));
	memcpy(if_req.mac, mac->octet, sizeof(if_req.mac));
	if_req.interface_type = htod16(if_type);
	if_req.chan_spec = htod16(chanspec);

	ret = whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_P2P_IFADD,
					&if_req, sizeof(if_req));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_ifadd type=%u chanspec=0x%04x failed: %d",
			if_type, chanspec, ret);
		return -EIO;
	}
	LOG_INF("p2p_ifadd type=%u chanspec=0x%04x OK", if_type, chanspec);
	return 0;
}

/**
 * Delete the P2P group interface.
 */
static int p2p_ifdel(whd_interface_t ifp, const whd_mac_t *mac)
{
	whd_result_t ret;

	ret = whd_wifi_set_iovar_buffer(ifp, IOVAR_STR_P2P_IFDEL,
					(void *)mac, sizeof(whd_mac_t));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_ifdel failed: %d", ret);
		return -EIO;
	}
	LOG_DBG("p2p_ifdel done");
	return 0;
}

/**
 * Set OppPS / CTWindow for P2P power save (GO only).
 */
static int p2p_set_ops(whd_interface_t ifp, bool enable, uint8_t ctwindow)
{
	wl_p2p_ops_t ops = {
		.ops = enable ? 1 : 0,
		.ctw = ctwindow,
	};
	whd_result_t ret;

	ret = whd_wifi_set_iovar_buffer(ifp, "p2p_ops", &ops, sizeof(ops));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_ops set failed: %d", ret);
		return -EIO;
	}
	LOG_DBG("p2p_ops enable=%u ctw=%u", ops.ops, ops.ctw);
	return 0;
}

/**
 * Set a NoA (Notice of Absence) schedule (GO only).
 */
static int p2p_set_noa(whd_interface_t ifp, uint32_t count,
		       uint32_t interval_ms, uint32_t duration_ms)
{
	wl_p2p_sched_t sched;

	memset(&sched, 0, sizeof(sched));
	sched.type   = WL_P2P_SCHED_TYPE_ABS;
	sched.action = (count == 0) ? WL_P2P_SCHED_ACTION_RESET
				    : WL_P2P_SCHED_ACTION_DOZE;
	sched.option = WL_P2P_SCHED_OPTION_NORMAL;
	sched.desc[0].count    = count;
	sched.desc[0].interval = interval_ms * 1000; /* firmware expects µs */
	sched.desc[0].duration = duration_ms * 1000;
	sched.desc[0].start    = 0;

	whd_result_t ret = whd_wifi_set_iovar_buffer(ifp, "p2p_noa",
						     &sched, sizeof(sched));
	if (ret != WHD_SUCCESS) {
		LOG_ERR("p2p_noa set failed: %d", ret);
		return -EIO;
	}
	LOG_DBG("p2p_noa count=%u interval=%u ms duration=%u ms",
		count, interval_ms, duration_ms);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Peer cache helpers                                                  */
/* ------------------------------------------------------------------ */
static void p2p_peer_cache_clear(void)
{
	memset(p2p_ctx.peers, 0, sizeof(p2p_ctx.peers));
	p2p_ctx.peer_count = 0;
}

static struct airoc_p2p_peer *p2p_peer_find(const uint8_t *mac)
{
	for (int i = 0; i < AIROC_P2P_MAX_PEERS; i++) {
		if (p2p_ctx.peers[i].valid &&
		    memcmp(p2p_ctx.peers[i].mac, mac, 6) == 0) {
			return &p2p_ctx.peers[i];
		}
	}
	return NULL;
}

static void p2p_raise_device_found(const uint8_t *mac, int8_t rssi,
				   const char *name)
{
	struct wifi_p2p_device_info info;

	if (p2p_ctx.iface == NULL) {
		return;
	}

	memset(&info, 0, sizeof(info));
	memcpy(info.mac, mac, WIFI_MAC_ADDR_LEN);
	info.rssi = rssi;
	if (name != NULL && name[0] != '\0') {
		strncpy(info.device_name, name, sizeof(info.device_name) - 1);
	}
	wifi_mgmt_raise_p2p_device_found_event(p2p_ctx.iface, &info);
}

static struct airoc_p2p_peer *p2p_peer_add(const uint8_t *mac, int8_t rssi,
					  const char *name)
{
	struct airoc_p2p_peer *peer = p2p_peer_find(mac);
	bool is_new = false;

	if (peer) {
		peer->rssi = rssi;
		if (name != NULL && name[0] != '\0' && peer->name[0] == '\0') {
			strncpy(peer->name, name, sizeof(peer->name) - 1);
		}
	} else {
		for (int i = 0; i < AIROC_P2P_MAX_PEERS; i++) {
			if (!p2p_ctx.peers[i].valid) {
				memcpy(p2p_ctx.peers[i].mac, mac, 6);
				p2p_ctx.peers[i].rssi = rssi;
				p2p_ctx.peers[i].valid = true;
				if (name != NULL && name[0] != '\0') {
					strncpy(p2p_ctx.peers[i].name, name,
						sizeof(p2p_ctx.peers[i].name) - 1);
				}
				p2p_ctx.peer_count++;
				peer = &p2p_ctx.peers[i];
				is_new = true;
				break;
			}
		}
	}

	if (!peer) {
		LOG_WRN("P2P peer cache full");
		return NULL;
	}

	if (is_new) {
		LOG_DBG("P2P peer: %s %02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
			peer->name[0] ? peer->name : "(no-name)",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
		p2p_raise_device_found(mac, rssi, peer->name);
	}

	return peer;
}

static void p2p_handle_escan_result(const whd_event_header_t *event_header,
				    const uint8_t *event_data)
{
	const wl_escan_result_t *eresult;
	const wl_bss_info_t *bss;
	char name[WIFI_P2P_DEVICE_NAME_MAX_LEN + 1];
	uint16_t bss_count;
	uint8_t ssid_len;

	if (event_header->status == WLC_E_STATUS_SUCCESS) {
		/* One social-channel sweep done — enter Listen so phones can
		 * see us and we can catch their probe requests, then SEARCH again.
		 */
		if (p2p_ctx.state == AIROC_P2P_STATE_DISCOVERING) {
			k_work_submit(&p2p_ctx.event_work);
		}
		return;
	}

	if (event_header->status != WLC_E_STATUS_PARTIAL || event_data == NULL) {
		return;
	}

	eresult = (const wl_escan_result_t *)event_data;
	bss_count = dtoh16(eresult->bss_count);
	if (bss_count == 0) {
		return;
	}

	bss = &eresult->bss_info[0];
	ssid_len = bss->SSID_len;
	if (ssid_len > sizeof(bss->SSID)) {
		ssid_len = sizeof(bss->SSID);
	}

	/* Prefer DIRECT-* names; still accept other p2p_scan hits. */
	memset(name, 0, sizeof(name));
	if (ssid_len > 0) {
		size_t copy = ssid_len;

		if (copy > sizeof(name) - 1) {
			copy = sizeof(name) - 1;
		}
		memcpy(name, bss->SSID, copy);
	}

	p2p_peer_add(bss->BSSID.octet, (int8_t)dtoh16(bss->RSSI), name);
}

static bool p2p_mac_is_broadcast(const uint8_t *mac)
{
	static const uint8_t bcast[WIFI_MAC_ADDR_LEN] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};

	return memcmp(mac, bcast, WIFI_MAC_ADDR_LEN) == 0;
}

static int p2p_peer_query(struct wifi_p2p_params *params)
{
	uint16_t max_peers;
	uint16_t out = 0;
	bool all;

	if (params->peers == NULL || params->peer_count == 0) {
		return -EINVAL;
	}

	max_peers = params->peer_count;
	all = p2p_mac_is_broadcast(params->peer_addr);

	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	for (int i = 0; i < AIROC_P2P_MAX_PEERS && out < max_peers; i++) {
		if (!p2p_ctx.peers[i].valid) {
			continue;
		}
		if (!all &&
		    memcmp(p2p_ctx.peers[i].mac, params->peer_addr,
			   WIFI_MAC_ADDR_LEN) != 0) {
			continue;
		}

		memset(&params->peers[out], 0, sizeof(params->peers[out]));
		memcpy(params->peers[out].mac, p2p_ctx.peers[i].mac,
		       WIFI_MAC_ADDR_LEN);
		params->peers[out].rssi = p2p_ctx.peers[i].rssi;
		if (p2p_ctx.peers[i].name[0] != '\0') {
			strncpy(params->peers[out].device_name,
				p2p_ctx.peers[i].name,
				sizeof(params->peers[out].device_name) - 1);
		}
		out++;
	}

	k_mutex_unlock(&p2p_ctx.lock);
	params->peer_count = out;
	return 0;
}

static uint16_t p2p_freq_to_channel(int freq)
{
	if (freq <= 0) {
		return 0;
	}
	/* Allow channel numbers 1–14 as well as MHz */
	if (freq >= 1 && freq <= 14) {
		return (uint16_t)freq;
	}
	if (freq >= 2412 && freq <= 2484) {
		return (uint16_t)((freq - 2407) / 5);
	}
	return 0;
}

/* ------------------------------------------------------------------ */
/* P2P Discovery (Find / Listen / Stop)                                */
/* ------------------------------------------------------------------ */

/**
 * Start P2P discovery (Find) on the primary STA interface.
 * Firmware creates the discovery BSSCFG when p2p_disc=1.
 */
static int p2p_start_find(enum wifi_p2p_discovery_type find_type,
			  uint16_t timeout_s)
{
	int err;

	ARG_UNUSED(find_type); /* Phase-1: always social-channel escan */

	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	if (p2p_ctx.state == AIROC_P2P_STATE_GROUP_FORMED) {
		LOG_ERR("Stop P2P group before starting find");
		k_mutex_unlock(&p2p_ctx.lock);
		return -EBUSY;
	}

	if (p2p_ctx.state != AIROC_P2P_STATE_IDLE) {
		LOG_WRN("P2P not idle (state=%d), stopping first", p2p_ctx.state);
		k_mutex_unlock(&p2p_ctx.lock);
		p2p_stop_find();
		k_mutex_lock(&p2p_ctx.lock, K_FOREVER);
	}

	if (p2p_sta_if == NULL) {
		LOG_ERR("P2P STA interface not ready");
		k_mutex_unlock(&p2p_ctx.lock);
		return -ENODEV;
	}

	/* Track discovery against primary; firmware owns the disc BSSCFG */
	p2p_ctx.disc_if = p2p_sta_if;

	/* Register event handler once */
	if (p2p_ensure_events(NULL) != 0) {
		p2p_ctx.disc_if = NULL;
		k_mutex_unlock(&p2p_ctx.lock);
		return -EIO;
	}

	/* Enable P2P discovery */
	err = p2p_set_discovery(p2p_sta_if, true);
	if (err) {
		p2p_ctx.disc_if = NULL;
		k_mutex_unlock(&p2p_ctx.lock);
		return err;
	}
	p2p_ctx.disc_enabled = true;

	/* One IE install under lock before search — no concurrent retries */
	err = p2p_install_discovery_ies();
	if (err) {
		LOG_WRN("Discovery IE install failed (%d) — phone may not list us",
			err);
	}

	/* SEARCH on social channel 1 (scan covers 1/6/11) */
	uint16_t chanspec = p2p_make_chanspec_2g(p2p_whd_drv, AIROC_P2P_SOCIAL_CH_1);

	err = p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SEARCH,
			    chanspec, AIROC_P2P_SEARCH_DWELL_MS);
	if (err) {
		p2p_remove_discovery_ies();
		p2p_set_discovery(p2p_sta_if, false);
		p2p_ctx.disc_enabled = false;
		p2p_ctx.disc_if = NULL;
		k_mutex_unlock(&p2p_ctx.lock);
		return err;
	}

	err = p2p_trigger_scan(p2p_sta_if);
	if (err) {
		p2p_remove_discovery_ies();
		p2p_set_discovery(p2p_sta_if, false);
		p2p_ctx.disc_enabled = false;
		p2p_ctx.disc_if = NULL;
		k_mutex_unlock(&p2p_ctx.lock);
		return err;
	}

	p2p_ctx.state = AIROC_P2P_STATE_DISCOVERING;
	p2p_peer_cache_clear();

	if (timeout_s == 0) {
		timeout_s = AIROC_P2P_DISC_TIMEOUT_DEFAULT_S;
	}
	k_work_schedule(&p2p_ctx.disc_timeout_work, K_SECONDS(timeout_s));

	LOG_INF("P2P Find started (timeout %u s)", timeout_s);
	k_mutex_unlock(&p2p_ctx.lock);
	return 0;
}

static int p2p_stop_find(void)
{
	int err = 0;

	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	k_work_cancel_delayable(&p2p_ctx.disc_timeout_work);

	if (p2p_ctx.disc_enabled && p2p_sta_if != NULL) {
		p2p_remove_discovery_ies();
		err = p2p_set_discovery(p2p_sta_if, false);
		p2p_ctx.disc_enabled = false;
	}

	if (p2p_ctx.event_index != 0xFF && p2p_sta_if != NULL) {
		whd_wifi_deregister_event_handler(p2p_sta_if,
						  p2p_ctx.event_index);
		p2p_ctx.event_index = 0xFF;
	}
	if (p2p_grp_event_index != 0xFF && p2p_sta_if != NULL) {
		whd_wifi_deregister_event_handler(p2p_sta_if,
						  p2p_grp_event_index);
		p2p_grp_event_index = 0xFF;
	}

	p2p_ctx.disc_if = NULL;
	p2p_ctx.state = AIROC_P2P_STATE_IDLE;
	LOG_INF("P2P Find/Listen stopped");
	k_mutex_unlock(&p2p_ctx.lock);
	return err;
}

/* ------------------------------------------------------------------ */
/* P2P Group Add / Remove (Autonomous GO)                              */
/* ------------------------------------------------------------------ */

#if defined(CONFIG_NET_DHCPV4_SERVER)
static int p2p_go_enable_dhcp(struct net_if *iface)
{
	struct in_addr addr;
	struct in_addr netmask;
	struct in_addr pool;
	int ret;

	if (iface == NULL) {
		iface = net_if_get_default();
	}
	if (iface == NULL) {
		LOG_ERR("No net_if for P2P GO DHCP");
		return -ENODEV;
	}

	if (net_addr_pton(AF_INET, AIROC_P2P_GO_IP_ADDR, &addr) ||
	    net_addr_pton(AF_INET, AIROC_P2P_GO_NETMASK, &netmask) ||
	    net_addr_pton(AF_INET, AIROC_P2P_GO_DHCP_POOL_BASE, &pool)) {
		LOG_ERR("Invalid P2P GO IPv4 config");
		return -EINVAL;
	}

	(void)net_if_up(iface);

	net_if_ipv4_set_gw(iface, &addr);

	if (net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0) == NULL) {
		LOG_WRN("P2P GO IPv4 address already set or add failed");
	}

	if (!net_if_ipv4_set_netmask_by_addr(iface, &addr, &netmask)) {
		LOG_ERR("Failed to set P2P GO netmask");
		return -EIO;
	}

	ret = net_dhcpv4_server_start(iface, &pool);
	if (ret && ret != -EALREADY) {
		LOG_ERR("Failed to start DHCPv4 server for P2P GO: %d", ret);
		return ret;
	}

	LOG_INF("P2P GO: DHCPv4 server %s (pool %s+)",
		AIROC_P2P_GO_IP_ADDR, AIROC_P2P_GO_DHCP_POOL_BASE);
	return 0;
}

static void p2p_go_disable_dhcp(struct net_if *iface)
{
	if (iface == NULL) {
		iface = net_if_get_default();
	}
	if (iface != NULL) {
		(void)net_dhcpv4_server_stop(iface);
	}
}
#else
static int p2p_go_enable_dhcp(struct net_if *iface)
{
	ARG_UNUSED(iface);
	LOG_WRN("P2P GO: CONFIG_NET_DHCPV4_SERVER=n — clients will not get an IP");
	return 0;
}

static void p2p_go_disable_dhcp(struct net_if *iface)
{
	ARG_UNUSED(iface);
}
#endif /* CONFIG_NET_DHCPV4_SERVER */

/**
 * Create an autonomous Group Owner on the specified channel.
 * WIFI5: p2p_disc (bsscfg1) + p2p_ifadd GO + SoftAP on tertiary (bsscfg2).
 */
static int p2p_group_add(uint16_t channel)
{
	int err;
	whd_interface_t group_if = NULL;
	whd_mac_t go_mac;
	uint16_t chanspec;
	bool used_ifadd = false;
	bool disc_released = false;

	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	if (p2p_ctx.state == AIROC_P2P_STATE_GROUP_FORMED) {
		LOG_ERR("P2P group already active");
		k_mutex_unlock(&p2p_ctx.lock);
		return -EALREADY;
	}

	/* Stop any active discovery */
	if (p2p_ctx.state != AIROC_P2P_STATE_IDLE) {
		k_mutex_unlock(&p2p_ctx.lock);
		p2p_stop_find();
		k_mutex_lock(&p2p_ctx.lock, K_FOREVER);
	}

	if (channel == 0) {
		channel = AIROC_P2P_SOCIAL_CH_6;
	}
	chanspec = p2p_make_chanspec_2g(p2p_whd_drv, channel);

	LOG_INF("P2P GO bring-up channel=%u chanspec=0x%04x", channel, chanspec);

	/*
	 * Claim bsscfg1 for p2p_disc BEFORE SoftAP/GO IF. SoftAP uses
	 * tertiary bsscfg2 so discovery can answer ProbeReq SSID=DIRECT-.
	 */
	k_mutex_unlock(&p2p_ctx.lock);
	(void)p2p_go_enable_wildcard_listen((uint8_t)channel);

	/* Firmware P2P GO IF — creates concurrent BSS for SoftAP */
	memset(&go_mac, 0, sizeof(go_mac));
	if (p2p_sta_if != NULL &&
	    whd_wifi_get_mac_address(p2p_sta_if, &go_mac) == WHD_SUCCESS) {
		/*
		 * P2P interface address, same derivation as brcmfmac:
		 * dev_addr = STA MAC with LAA bit, int_addr additionally
		 * flips bit 7 of octet 4. Using dev_addr alone collides with
		 * the SoftAP MAC that WHD derives, which firmware rejects.
		 */
		go_mac.octet[0] |= 0x02;
		go_mac.octet[4] ^= 0x80;
		/* LISTEN can reject ifadd — park discovery in SCAN first */
		(void)p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SCAN,
				    chanspec, 0);
		if (p2p_ifadd(p2p_sta_if, &go_mac, AIROC_P2P_IF_GO,
			      chanspec) == 0) {
			used_ifadd = true;
			k_msleep(50);
		} else {
			LOG_WRN("p2p_ifadd failed — SoftAP without concurrent "
				"DIRECT- ProbeResp");
		}
	}

	/*
	 * concurrent SoftAP (bsscfg2) only when p2p_ifadd created the GO BSS.
	 * Raw SoftAP on bsscfg2 fails set_chanspec on CYW43439.
	 */
	if (!used_ifadd && p2p_ctx.disc_enabled && p2p_sta_if != NULL) {
		(void)p2p_set_discovery(p2p_sta_if, false);
		p2p_ctx.disc_enabled = false;
		p2p_remove_discovery_ies();
		disc_released = true;
		LOG_WRN("P2P GO: disabled p2p_disc so SoftAP can use bsscfg1");
	}

#if AIROC_P2P_GO_USE_OPEN
	err = airoc_wifi_go_softap_start(AIROC_P2P_GO_SSID,
					 strlen(AIROC_P2P_GO_SSID),
					 NULL, 0,
					 (uint8_t)channel,
					 used_ifadd ? &go_mac : NULL,
					 &group_if);
#else
	err = airoc_wifi_go_softap_start(AIROC_P2P_GO_SSID,
					 strlen(AIROC_P2P_GO_SSID),
					 AIROC_P2P_GO_PSK,
					 strlen(AIROC_P2P_GO_PSK),
					 (uint8_t)channel,
					 used_ifadd ? &go_mac : NULL,
					 &group_if);
#endif
	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);
	if (err) {
		LOG_ERR("softAP-style P2P GO start failed: %d", err);
		if (used_ifadd && p2p_sta_if != NULL) {
			(void)p2p_ifdel(p2p_sta_if, &go_mac);
		}
		if (p2p_ctx.disc_enabled && p2p_sta_if != NULL) {
			(void)p2p_set_discovery(p2p_sta_if, false);
			p2p_ctx.disc_enabled = false;
		}
		p2p_remove_discovery_ies();
		k_mutex_unlock(&p2p_ctx.lock);
		return err;
	}

	p2p_ctx.group_if = group_if;
	p2p_ctx.used_p2p_ifadd = used_ifadd;
	p2p_ctx.is_go = true;
	p2p_ctx.go_channel = (uint8_t)channel;
	p2p_ctx.state = AIROC_P2P_STATE_GROUP_FORMED;
	if (used_ifadd) {
		whd_wifi_p2p_set_go_is_up(p2p_whd_drv, WHD_TRUE);
	}

	/* Disc MAC before GO IEs so Device Info is not the SoftAP BSSID */
	{
		whd_mac_t go_mac_now;

		if (whd_wifi_get_mac_address(group_if, &go_mac_now) ==
		    WHD_SUCCESS) {
			p2p_derive_disc_mac_from_go(go_mac_now.octet);
		}
		if (!p2p_disc_dev_mac_valid) {
			(void)p2p_bsscfg_get_mac(p2p_sta_if, AIROC_P2P_BSSCFG_DISC,
						 p2p_disc_dev_mac);
			if (!NULL_MAC(p2p_disc_dev_mac)) {
				p2p_disc_dev_mac_valid = true;
			}
		}
	}

	/* SoftAP accepts vndr_ie — required for phone Wi-Fi Direct listing */
	(void)p2p_install_go_ies(group_if);

	/* A phone may associate for WPS before any Provision Discovery */
	(void)p2p_go_allow_wps_assoc(group_if);

	/*
	 * Register action-frame events on the SoftAP IF so Provision
	 * Discovery Requests from the phone are delivered (find path
	 * deregisters these before group add).
	 */
	err = p2p_ensure_events(group_if);
	if (err) {
		LOG_WRN("P2P AF event register failed (%d) — "
			"phone invite may hang", err);
	}

	/* SoftAP bring-up can disturb LISTEN — re-arm disc after AP is up */
	if (p2p_ctx.disc_enabled) {
		uint16_t listen_cs = p2p_make_chanspec_2g(p2p_whd_drv,
							  (uint8_t)channel);

		(void)p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_LISTEN,
				    listen_cs, 5000);
	} else if (disc_released) {
		/* SoftAP owns bsscfg1; re-enabling disc can only return BUSY */
		LOG_WRN("P2P GO: no disc BSS — DIRECT- ProbeReq unanswered");
	} else {
		(void)p2p_go_enable_wildcard_listen((uint8_t)channel);
	}

	err = p2p_go_enable_dhcp(p2p_ctx.iface);
	if (err) {
		LOG_WRN("P2P GO: DHCP setup failed (%d) — continuing", err);
	}

	k_work_reschedule(&p2p_ctx.go_client_poll_work, K_SECONDS(2));

	LOG_INF("P2P autonomous GO started on channel %u (SSID=%s)",
		channel, AIROC_P2P_GO_SSID);
	k_mutex_unlock(&p2p_ctx.lock);
	return 0;
}

/**
 * Remove the current P2P group (GO or Client).
 */
static int p2p_group_remove(void)
{
	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	if (p2p_ctx.state != AIROC_P2P_STATE_GROUP_FORMED) {
		LOG_WRN("No P2P group to remove");
		k_mutex_unlock(&p2p_ctx.lock);
		return -ENOENT;
	}

	p2p_go_disable_dhcp(p2p_ctx.iface);
	(void)k_work_cancel_delayable(&p2p_ctx.go_client_poll_work);
	if (p2p_ctx.disc_enabled && p2p_sta_if != NULL) {
		(void)p2p_set_discovery(p2p_sta_if, false);
		p2p_ctx.disc_enabled = false;
	}
	p2p_remove_discovery_ies();

	if (p2p_ctx.is_go && p2p_ctx.group_if != NULL) {
		bool did_ifadd = p2p_ctx.used_p2p_ifadd;
		whd_mac_t mac;

		memset(&mac, 0, sizeof(mac));
		if (did_ifadd) {
			(void)whd_wifi_get_mac_address(p2p_ctx.group_if, &mac);
		}

		k_mutex_unlock(&p2p_ctx.lock);
		(void)airoc_wifi_go_softap_stop();
		k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

		if (did_ifadd && p2p_sta_if != NULL) {
			whd_wifi_p2p_set_go_is_up(p2p_whd_drv, WHD_FALSE);
			if (!NULL_MAC(mac.octet)) {
				(void)p2p_ifdel(p2p_sta_if, &mac);
			}
		}
	} else if (p2p_ctx.group_if != NULL) {
		/* P2P Client — just leave */
		whd_wifi_leave(p2p_ctx.group_if);
		if (p2p_ctx.used_p2p_ifadd) {
			whd_mac_t mac;

			whd_wifi_get_mac_address(p2p_ctx.group_if, &mac);
			p2p_ifdel(p2p_sta_if, &mac);
		}
	}

	if (p2p_grp_event_index != 0xFF && p2p_sta_if != NULL) {
		whd_wifi_deregister_event_handler(p2p_sta_if,
						  p2p_grp_event_index);
		p2p_grp_event_index = 0xFF;
	}

	p2p_ctx.group_if = NULL;
	p2p_ctx.is_go = false;
	p2p_ctx.used_p2p_ifadd = false;
	p2p_ctx.go_channel = 0;
	p2p_go_wps_pbc_active = false;
	airoc_wps_reg_disarm();
	p2p_go_last_client_count = 0xffffffffU;
	p2p_ctx.state = AIROC_P2P_STATE_IDLE;

	LOG_INF("P2P group removed");
	k_mutex_unlock(&p2p_ctx.lock);
	return 0;
}

/* ------------------------------------------------------------------ */
/* P2P Power Save (OppPS / NoA — GO only)                              */
/* ------------------------------------------------------------------ */
static int p2p_power_save(bool enable)
{
	if (p2p_ctx.state != AIROC_P2P_STATE_GROUP_FORMED || !p2p_ctx.is_go) {
		LOG_ERR("P2P power save only valid when GO is active");
		return -EINVAL;
	}

	int err = p2p_set_ops(p2p_ctx.group_if, enable, enable ? 50 : 0);

	if (err) {
		return err;
	}

	if (enable) {
		err = p2p_set_noa(p2p_ctx.group_if, 255, 100, 25);
	} else {
		err = p2p_set_noa(p2p_ctx.group_if, 0, 0, 0);
	}
	return err;
}

/* ------------------------------------------------------------------ */
/* WHD event handler (ISR-safe — enqueues work)                        */
/* ------------------------------------------------------------------ */
/* ------------------------------------------------------------------ */
/* P2P action-frame helpers (Provision Discovery)                      */
/* ------------------------------------------------------------------ */

static int p2p_register_events(whd_interface_t ifp, uint16_t *slot)
{
	whd_result_t ret;

	if (ifp == NULL) {
		return -ENODEV;
	}
	if (*slot != 0xFF) {
		return 0;
	}

	ret = whd_management_set_event_handler(ifp, p2p_events,
					       p2p_event_handler, NULL, slot);
	if (ret != WHD_SUCCESS) {
		LOG_ERR("P2P event register (ifidx %u) failed: %d",
			ifp->ifidx, ret);
		*slot = 0xFF;
		return -EIO;
	}
	return 0;
}

/**
 * WHD delivers an event only to handlers registered with the same ifidx, so a
 * GO-only registration misses everything on ifidx 0. Provision Discovery for
 * our P2P device address lands on the discovery BSSCFG, which shares ifidx 0
 * with the primary interface, so both interfaces need a handler.
 */
static int p2p_ensure_events(whd_interface_t ifp)
{
	int err;

	if (ifp == NULL && p2p_sta_if == NULL) {
		return -ENODEV;
	}

	err = p2p_register_events(p2p_sta_if, &p2p_ctx.event_index);

	if (ifp != NULL && p2p_sta_if != NULL &&
	    ifp->ifidx != p2p_sta_if->ifidx) {
		int grp_err = p2p_register_events(ifp, &p2p_grp_event_index);

		if (err == 0) {
			err = grp_err;
		}
	}
	return err;
}

/**
 * Extract WSC Config Methods (0x1008) from the IEs of a received PD Request.
 * Returns AIROC_WPS_CONFIG_PBC when absent, since PBC is all we support.
 */
static uint16_t p2p_parse_req_config_methods(const uint8_t *ie, uint32_t len)
{
	static const uint8_t wps_oui[4] = { 0x00, 0x50, 0xf2,
					    AIROC_WPS_IE_OUI_TYPE };
	uint32_t o = 0;

	while ((o + 2u) <= len) {
		uint8_t eid = ie[o];
		uint8_t elen = ie[o + 1];
		const uint8_t *body = &ie[o + 2];

		if ((o + 2u + elen) > len) {
			break;
		}
		if (eid == 0xdd && elen >= 4 &&
		    memcmp(body, wps_oui, 4) == 0) {
			uint32_t a = 4;

			/* WSC attributes are big-endian id/len pairs */
			while ((a + 4u) <= elen) {
				uint16_t aid = (uint16_t)(((uint16_t)body[a] << 8) |
							  body[a + 1]);
				uint16_t alen = (uint16_t)(((uint16_t)body[a + 2] << 8) |
							   body[a + 3]);

				if ((a + 4u + alen) > elen) {
					break;
				}
				if (aid == 0x1008 && alen == 2) {
					return (uint16_t)(((uint16_t)body[a + 4] << 8) |
							  body[a + 5]);
				}
				a += 4u + alen;
			}
		}
		o += 2u + elen;
	}

	return AIROC_WPS_CONFIG_PBC;
}

/**
 * Parse RX action payload for P2P Provision Discovery Request.
 * event_data may be raw action body, or wl_event_rx_frame_data + 802.11 hdr.
 */
static void p2p_handle_action_rx(const whd_event_header_t *eh,
				 const uint8_t *data, uint32_t datalen)
{
	static const uint8_t p2p_oui[3] = { 0x50, 0x6f, 0x9a };
	const uint8_t *body = data;
	const uint8_t *dot11 = NULL;
	uint32_t blen = datalen;
	uint8_t dialog;
	uint16_t ch = p2p_ctx.go_channel ? p2p_ctx.go_channel
					 : AIROC_P2P_SOCIAL_CH_6;

	if (data == NULL || datalen < 8) {
		return;
	}

	/*
	 * WLC_E_ACTION_FRAME_RX prefixes wl_event_rx_frame_data (12B packed
	 * on most FW; some builds pad to 16). Fall back to bare 802.11.
	 */
	if (!p2p_locate_dot11_action(data, datalen, &dot11, &body, &blen,
				     &ch)) {
		dot11 = NULL;
		body = data;
		blen = datalen;
	}

	/* Public Action Vendor-Specific P2P */
	if (blen < 8 ||
	    body[0] != AIROC_P2P_PUB_AF_CATEGORY ||
	    body[1] != AIROC_P2P_PUB_AF_ACTION ||
	    memcmp(&body[2], p2p_oui, 3) != 0 ||
	    body[5] != AIROC_P2P_OUI_TYPE) {
		return;
	}

	if (body[6] != AIROC_P2P_PAF_PD_REQ) {
		return;
	}

	dialog = body[7];
	{
		uint8_t peer[6];
		uint8_t ra[6];
		bool have_ra = false;
		int64_t now;
		bool dup;

		memcpy(peer, eh->addr.octet, 6);
		if (dot11 != NULL && NULL_MAC(peer)) {
			memcpy(peer, &dot11[10], 6);
		}

		if (dot11 != NULL && !NULL_MAC(&dot11[4])) {
			memcpy(ra, &dot11[4], 6);
			have_ra = true;
			if (eh->bsscfgidx == AIROC_P2P_BSSCFG_DISC) {
				p2p_remember_disc_mac(ra);
			}
		}

		now = k_uptime_get();
		{
			uint16_t cfg_methods =
				p2p_parse_req_config_methods(&body[8], blen - 8);
			k_spinlock_key_t key = k_spin_lock(&p2p_pd_lock);

			dup = (p2p_pd_dedup.dialog == dialog &&
			       memcmp(p2p_pd_dedup.peer, peer, 6) == 0 &&
			       (now - p2p_pd_dedup.at_ms) < 400);

			if (dup) {
				/* Event 75 upgrades RA while delayed PD work is pending */
				if (have_ra) {
					memcpy(p2p_pd_req.our_addr, ra, 6);
				}
				k_spin_unlock(&p2p_pd_lock, key);
				return;
			}

			memcpy(p2p_pd_dedup.peer, peer, 6);
			p2p_pd_dedup.dialog = dialog;
			p2p_pd_dedup.at_ms = now;

			p2p_pd_req.pending = true;
			memcpy(p2p_pd_req.peer, peer, 6);
			p2p_pd_req.bsscfg = eh->bsscfgidx;
			p2p_pd_req.dialog_token = dialog;
			p2p_pd_req.channel = ch;
			p2p_pd_req.config_methods = cfg_methods;

			if (have_ra) {
				memcpy(p2p_pd_req.our_addr, ra, 6);
			} else if (eh->bsscfgidx == AIROC_P2P_BSSCFG_DISC &&
				   p2p_disc_dev_mac_valid) {
				memcpy(p2p_pd_req.our_addr, p2p_disc_dev_mac, 6);
			} else {
				memset(p2p_pd_req.our_addr, 0, 6);
			}
			k_spin_unlock(&p2p_pd_lock, key);
		}

		LOG_INF("P2P Provision Discovery REQ from "
			"%02x:%02x:%02x:%02x:%02x:%02x",
			peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
		/* Wait briefly for the paired ACTION_FRAME_RX (75) with RA */
		k_work_reschedule(&p2p_af_work, K_MSEC(30));
	}
}

static int p2p_send_pd_response(const uint8_t *peer, const uint8_t *our_addr,
				uint8_t bsscfg, uint8_t dialog,
				uint16_t channel, uint16_t config_methods)
{
	static whd_af_params_t af;
	whd_interface_t ifp = p2p_ctx.group_if ? p2p_ctx.group_if : p2p_sta_if;
	struct whd_interface shadow;
	bool on_disc_bss = (bsscfg == 1);
	whd_mac_t go_mac;
	uint8_t dev_addr[6];
	uint8_t *p;
	uint8_t *ie_len_ptr;
	size_t ssid_len = strlen(AIROC_P2P_GO_SSID);
	size_t name_len = strlen(AIROC_P2P_DEV_NAME);
	int err;
	uint8_t op_ch = (uint8_t)channel;

	if (ifp == NULL || peer == NULL) {
		return -ENODEV;
	}
	if (op_ch == 0 || op_ch > 14) {
		op_ch = p2p_ctx.go_channel ? p2p_ctx.go_channel
					  : AIROC_P2P_SOCIAL_CH_6;
	}
	if (name_len > 32) {
		name_len = 32;
	}

	if (whd_wifi_get_mac_address(ifp, &go_mac) != WHD_SUCCESS) {
		memset(go_mac.octet, 0, 6);
	}

	/* Device Info must match the address the peer talked to (disc MAC
	 * on bsscfg1). Group ID still points at the GO SoftAP BSSID.
	 */
	if (our_addr != NULL && !NULL_MAC(our_addr)) {
		memcpy(dev_addr, our_addr, 6);
	} else if (on_disc_bss && p2p_disc_dev_mac_valid) {
		memcpy(dev_addr, p2p_disc_dev_mac, 6);
	} else {
		memcpy(dev_addr, go_mac.octet, 6);
	}

	if (on_disc_bss && p2p_sta_if != NULL) {
		ifp = p2p_bsscfg_shadow(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, &shadow);
	}

	memset(&af, 0, sizeof(af));
	af.channel = op_ch;
	af.dwell_time = 0; /* on-channel SoftAP — no off-chan dwell */
	memcpy(af.BSSID.octet, dev_addr, 6);
	memcpy(af.action_frame.da.octet, peer, 6);
	af.action_frame.packetId = 1;

	p = af.action_frame.data;
	*p++ = AIROC_P2P_PUB_AF_CATEGORY;
	*p++ = AIROC_P2P_PUB_AF_ACTION;
	*p++ = 0x50;
	*p++ = 0x6f;
	*p++ = 0x9a;
	*p++ = AIROC_P2P_OUI_TYPE;
	*p++ = AIROC_P2P_PAF_PD_RSP;
	*p++ = dialog;

	/* Vendor IE: Status + Cap(GO) + Device Info + Group ID + Op Chan */
	*p++ = 0xdd;
	ie_len_ptr = p++;
	*p++ = 0x50;
	*p++ = 0x6f;
	*p++ = 0x9a;
	*p++ = AIROC_P2P_OUI_TYPE;
	/* Status = success */
	*p++ = 0x00;
	p2p_put_le16(p, 1); p += 2;
	*p++ = 0x00;
	/* Capability: device 0x25, group GO */
	*p++ = 0x02;
	p2p_put_le16(p, 2); p += 2;
	*p++ = 0x25;
	*p++ = AIROC_P2P_GROUP_CAP_GO;
	/* Device Info — address the peer used as destination */
	*p++ = 13;
	p2p_put_le16(p, (uint16_t)(6 + 2 + 8 + 1 + 4 + name_len)); p += 2;
	memcpy(p, dev_addr, 6); p += 6;
	p2p_put_be16(p, 0x0180); p += 2; /* PBC | Display */
	p2p_put_be16(p, 0x0001); p += 2;
	memcpy(p, "\x00\x50\xf2\x04", 4); p += 4;
	p2p_put_be16(p, 0x0001); p += 2;
	*p++ = 0;
	p2p_put_be16(p, 0x1011); p += 2;
	p2p_put_be16(p, (uint16_t)name_len); p += 2;
	memcpy(p, AIROC_P2P_DEV_NAME, name_len); p += name_len;
	/* Group ID — GO BSS the peer must join */
	*p++ = 15;
	p2p_put_le16(p, (uint16_t)(6 + ssid_len)); p += 2;
	memcpy(p, go_mac.octet, 6); p += 6;
	memcpy(p, AIROC_P2P_GO_SSID, ssid_len); p += ssid_len;
	/* Config Timeout (attr id 5) — id 4 is GO Intent; phones reject that */
	*p++ = 5;
	p2p_put_le16(p, 2); p += 2;
	*p++ = 100; /* GO timeout, units of 10 ms */
	*p++ = 20;  /* client timeout, units of 10 ms */
	/* Operating Channel */
	*p++ = 17;
	p2p_put_le16(p, 5); p += 2;
	*p++ = 'X';
	*p++ = 'X';
	*p++ = AIROC_P2P_OP_CLASS_2G4;
	*p++ = 81;
	*p++ = op_ch;

	*ie_len_ptr = (uint8_t)(p - ie_len_ptr - 1);

	/* WPS IE: Configured AP + Selected Registrar / PBC */
	{
		uint8_t wps_tmp[96];
		uint16_t wps_len;

		wps_len = p2p_build_wps_attrs(wps_tmp, sizeof(wps_tmp),
					      AIROC_P2P_DEV_NAME, true, true,
					      config_methods);
		if (wps_len > 0 && (p + 2 + 4 + wps_len) <
		    (af.action_frame.data + ACTION_FRAME_SIZE)) {
			*p++ = 0xdd;
			*p++ = (uint8_t)(4 + wps_len);
			*p++ = 0x00;
			*p++ = 0x50;
			*p++ = 0xf2;
			*p++ = AIROC_WPS_IE_OUI_TYPE;
			memcpy(p, wps_tmp, wps_len);
			p += wps_len;
		}
	}

	af.action_frame.len = (uint16_t)(p - af.action_frame.data);

	/* Advertise Selected Registrar before/with the PD response */
	(void)p2p_go_arm_wps_pbc();

	if (on_disc_bss) {
		err = p2p_bsscfg_send_af(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, &af);
		if (err) {
			/* Fall back to plain actframe on the disc shadow */
			whd_result_t ret = whd_wifi_send_action_frame(ifp, &af);

			err = (ret == WHD_SUCCESS) ? 0 : -EIO;
			if (err) {
				LOG_ERR("PD Response TX (disc) failed: %d (0x%x)",
					ret, ret);
			} else {
				LOG_DBG("P2P GO: PD RSP via actframe fallback");
			}
		}
	} else {
		whd_result_t ret = whd_wifi_send_action_frame(ifp, &af);

		err = (ret == WHD_SUCCESS) ? 0 : -EIO;
		if (err) {
			LOG_ERR("PD Response TX failed: %d (0x%x)", ret, ret);
		}
	}
	if (err) {
		return err;
	}

	LOG_INF("P2P Provision Discovery RSP sent");
	return 0;
}

static void p2p_af_work_handler(struct k_work *work)
{
	uint8_t peer[6];
	uint8_t our_addr[6];
	uint8_t bsscfg;
	uint8_t dialog;
	uint16_t channel;
	uint16_t config_methods;
	k_spinlock_key_t key;

	ARG_UNUSED(work);

	key = k_spin_lock(&p2p_pd_lock);
	if (!p2p_pd_req.pending) {
		k_spin_unlock(&p2p_pd_lock, key);
		return;
	}
	memcpy(peer, p2p_pd_req.peer, 6);
	memcpy(our_addr, p2p_pd_req.our_addr, 6);
	bsscfg = p2p_pd_req.bsscfg;
	dialog = p2p_pd_req.dialog_token;
	channel = p2p_pd_req.channel;
	config_methods = p2p_pd_req.config_methods;
	p2p_pd_req.pending = false;
	k_spin_unlock(&p2p_pd_lock, key);

	(void)p2p_send_pd_response(peer, our_addr, bsscfg, dialog, channel,
				   config_methods);
}

static void *p2p_event_handler(whd_interface_t ifp,
			       const whd_event_header_t *event_header,
			       const uint8_t *event_data,
			       void *handler_user_data)
{
	ARG_UNUSED(ifp);
	ARG_UNUSED(handler_user_data);

	switch ((whd_event_num_t)event_header->event_type) {
	case WLC_E_P2P_DISC_LISTEN_COMPLETE:
		LOG_DBG("P2P event: LISTEN_COMPLETE");
		k_work_submit(&p2p_ctx.event_work);
		break;

	case WLC_E_P2P_PROBREQ_MSG:
	case WLC_E_PROBREQ_MSG:
		/* While GO is up, skip peer-cache fills — ProbeReq storms
		 * only produced "peer cache full" spam and are not useful.
		 */
		if (p2p_ctx.state != AIROC_P2P_STATE_GROUP_FORMED) {
			p2p_peer_add(event_header->addr.octet, 0, NULL);
		}
		break;

	case WLC_E_ACTION_FRAME:
	case 75: /* WLC_E_ACTION_FRAME_RX */
		p2p_handle_action_rx(event_header, event_data,
				     event_header->datalen);
		break;

	case WLC_E_ACTION_FRAME_COMPLETE:
	case WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE:
		LOG_DBG("P2P event: ACTION_FRAME done (type %u status %u)",
			event_header->event_type, event_header->status);
		break;

	case WLC_E_ESCAN_RESULT:
		p2p_handle_escan_result(event_header, event_data);
		break;

	case WLC_E_P2PO_ADD_DEVICE:
		LOG_INF("P2P offload: device found");
		if (event_data) {
			p2p_peer_add(event_header->addr.octet, 0, NULL);
		}
		break;

	case WLC_E_P2PO_DEL_DEVICE:
		LOG_DBG("P2P offload: device removed");
		break;

	default:
		break;
	}

	return NULL;
}

/* Work handler: after search sweep → Listen; after listen → Search again */
static void p2p_event_work_handler(struct k_work *work)
{
	uint16_t chanspec;
	static const uint16_t social_chs[] = {
		AIROC_P2P_SOCIAL_CH_1,
		AIROC_P2P_SOCIAL_CH_6,
		AIROC_P2P_SOCIAL_CH_11,
	};
	uint16_t ch;

	ARG_UNUSED(work);

	k_mutex_lock(&p2p_ctx.lock, K_FOREVER);

	if (!p2p_ctx.disc_enabled || p2p_sta_if == NULL) {
		k_mutex_unlock(&p2p_ctx.lock);
		return;
	}

	/* GO assist: keep LISTEN alive on operating channel for DIRECT- probes.
	 * While WPS-PBC is armed, stay parked in SCAN so SoftAP can Auth.
	 */
	if (p2p_ctx.state == AIROC_P2P_STATE_GROUP_FORMED && p2p_ctx.is_go) {
		uint8_t go_ch = p2p_ctx.go_channel ? p2p_ctx.go_channel
						   : AIROC_P2P_SOCIAL_CH_6;

		chanspec = p2p_make_chanspec_2g(p2p_whd_drv, go_ch);
		if (p2p_go_wps_pbc_active) {
			(void)p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SCAN,
					    chanspec, 0);
		} else {
			(void)p2p_set_state(p2p_sta_if,
					    AIROC_P2P_DISC_ST_LISTEN,
					    chanspec, 5000);
		}
		k_mutex_unlock(&p2p_ctx.lock);
		return;
	}

	if (p2p_ctx.state == AIROC_P2P_STATE_DISCOVERING) {
		/* Search sweep finished → Listen on a social channel */
		ch = social_chs[k_cycle_get_32() % ARRAY_SIZE(social_chs)];
		chanspec = p2p_make_chanspec_2g(p2p_whd_drv, ch);
		if (p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_LISTEN,
				  chanspec, AIROC_P2P_LISTEN_DWELL_MS) == 0) {
			p2p_ctx.state = AIROC_P2P_STATE_LISTENING;
			LOG_DBG("Search done → Listen ch=%u chanspec=0x%04x",
				ch, chanspec);
		} else {
			/* Keep searching if listen rejected (e.g. BADCHAN) */
			chanspec = p2p_make_chanspec_2g(p2p_whd_drv,
							AIROC_P2P_SOCIAL_CH_1);
			p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SEARCH,
				      chanspec, AIROC_P2P_SEARCH_DWELL_MS);
			p2p_trigger_scan(p2p_sta_if);
			LOG_WRN("Listen failed — retrying search");
		}
	} else if (p2p_ctx.state == AIROC_P2P_STATE_LISTENING) {
		/* Listen expired — SEARCH + another social escan */
		chanspec = p2p_make_chanspec_2g(p2p_whd_drv,
						AIROC_P2P_SOCIAL_CH_1);
		p2p_set_state(p2p_sta_if, AIROC_P2P_DISC_ST_SEARCH,
			      chanspec, AIROC_P2P_SEARCH_DWELL_MS);
		p2p_trigger_scan(p2p_sta_if);
		p2p_ctx.state = AIROC_P2P_STATE_DISCOVERING;
		LOG_DBG("Listen complete → resuming search");
	}

	k_mutex_unlock(&p2p_ctx.lock);
}

/* Discovery timeout handler */
static void p2p_disc_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("P2P discovery timeout");
	p2p_stop_find();
}

/* Poll SoftAP associated-client list (events may be missing on some FW) */
static void p2p_go_client_poll_handler(struct k_work *work)
{
	uint8_t buf[sizeof(whd_maclist_t) + 4 * sizeof(whd_mac_t)];
	whd_maclist_t *list = (whd_maclist_t *)buf;
	whd_result_t ret;

	ARG_UNUSED(work);

	if (p2p_ctx.state != AIROC_P2P_STATE_GROUP_FORMED ||
	    p2p_ctx.group_if == NULL) {
		return;
	}

	memset(buf, 0, sizeof(buf));
	list->count = 4;
	ret = whd_wifi_get_associated_client_list(p2p_ctx.group_if, list,
						  sizeof(buf));
	if (ret == WHD_SUCCESS) {
		if (list->count != p2p_go_last_client_count) {
			LOG_INF("P2P GO clients: %u", list->count);
			p2p_go_last_client_count = list->count;
		}
	} else {
		LOG_DBG("P2P GO client poll failed (%d)", ret);
	}

	k_work_reschedule(&p2p_ctx.go_client_poll_work, K_SECONDS(2));
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

bool airoc_p2p_eapol_rx(const uint8_t *frame, uint32_t len)
{
	if (p2p_ctx.state != AIROC_P2P_STATE_GROUP_FORMED ||
	    frame == NULL || len < 14) {
		return false;
	}

	return airoc_wps_reg_eapol_rx(frame, len);
}

int airoc_p2p_init(whd_driver_t whd_drv, whd_interface_t sta_if)
{
	memset(&p2p_ctx, 0, sizeof(p2p_ctx));

	p2p_whd_drv = whd_drv;
	p2p_sta_if  = sta_if;

	p2p_ctx.state       = AIROC_P2P_STATE_IDLE;
	p2p_ctx.event_index = 0xFF;
	p2p_grp_event_index = 0xFF;

	k_mutex_init(&p2p_ctx.lock);
	k_work_init_delayable(&p2p_ctx.disc_timeout_work,
			      p2p_disc_timeout_handler);
	k_work_init_delayable(&p2p_ctx.go_client_poll_work,
			      p2p_go_client_poll_handler);
	k_work_init(&p2p_ctx.event_work, p2p_event_work_handler);
	k_work_init_delayable(&p2p_af_work, p2p_af_work_handler);

	LOG_INF("AIROC P2P subsystem initialised");
	return 0;
}

void airoc_p2p_set_iface(struct net_if *iface)
{
	p2p_ctx.iface = iface;
}

int airoc_p2p_oper(const struct device *dev, struct net_if *iface,
		   struct wifi_p2p_params *params)
{
	ARG_UNUSED(dev);

	if (!params) {
		return -EINVAL;
	}

	if (iface != NULL) {
		p2p_ctx.iface = iface;
	}

	switch (params->oper) {
	case WIFI_P2P_FIND:
		return p2p_start_find(params->discovery_type, params->timeout);

	case WIFI_P2P_STOP_FIND:
		return p2p_stop_find();

	case WIFI_P2P_PEER:
		return p2p_peer_query(params);

	case WIFI_P2P_GROUP_ADD:
		return p2p_group_add(p2p_freq_to_channel(params->group_add.freq));

	case WIFI_P2P_GROUP_REMOVE:
		return p2p_group_remove();

	case WIFI_P2P_POWER_SAVE:
		return p2p_power_save(params->power_save);

	case WIFI_P2P_CONNECT:
	case WIFI_P2P_INVITE:
	case WIFI_P2P_LIST_NETWORKS:
	case WIFI_P2P_PERSISTENT_REMOVE:
		LOG_WRN("P2P oper %d not supported (connect/invite not implemented)", params->oper);
		return -ENOTSUP;

	default:
		LOG_ERR("Unknown P2P oper %d", params->oper);
		return -ENOTSUP;
	}
}

void airoc_p2p_deinit(void)
{
	if (p2p_ctx.state == AIROC_P2P_STATE_GROUP_FORMED) {
		p2p_group_remove();
	}
	if (p2p_ctx.state != AIROC_P2P_STATE_IDLE) {
		p2p_stop_find();
	}
	LOG_INF("AIROC P2P subsystem de-initialised");
}

struct airoc_p2p_ctx *airoc_p2p_get_ctx(void)
{
	return &p2p_ctx;
}
