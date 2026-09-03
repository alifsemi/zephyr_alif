/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "airoc_wifi_p2p_priv.h"
#include <whd_wlioctl.h>
#include <whd_wifi_api.h>

LOG_MODULE_DECLARE(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

/* ---- extracted from airoc_wifi_p2p.c L232-371 ---- */
uint16_t p2p_build_wps_attrs(uint8_t *buf, size_t buflen, const char *name,
				    bool as_ap, bool sel_reg_pbc,
				    uint16_t config_methods)
{
	size_t nlen = strlen(name);
	size_t need;
	uint8_t *p = buf;
	uint8_t uuid[16];
	whd_mac_t mac;
	whd_interface_t ifp;

	if (nlen > 32) {
		nlen = 32;
	}

	/* Version + State + RespType + UUID-E + ConfigMethods + PrimDevType +
	 * Device Name + RF Bands (+ optional Selected Registrar set)
	 */
	need = 5 + 5 + 5 + 20 + 6 + 12 + (4 + nlen) + 5;
	if (sel_reg_pbc) {
		need += 5 + 6 + 6;
	}
	if (need > buflen) {
		return 0;
	}

	/* Stable UUID-E from interface MAC */
	memset(uuid, 0, sizeof(uuid));
	ifp = p2p_ctx.group_if ? p2p_ctx.group_if : p2p_sta_if;
	memcpy(uuid, "AIROCP2P", 8);
	if (ifp != NULL &&
	    whd_wifi_get_mac_address(ifp, &mac) == WHD_SUCCESS) {
		memcpy(&uuid[10], mac.octet, 6);
	} else {
		memset(&uuid[10], 0x11, 6);
	}
	/* RFC4122 version 4 / variant */
	uuid[6] = (uint8_t)((uuid[6] & 0x0f) | 0x40);
	uuid[8] = (uint8_t)((uuid[8] & 0x3f) | 0x80);

	/* Version 0x104A */
	p2p_put_be16(p, 0x104A); p += 2;
	p2p_put_be16(p, 1); p += 2;
	*p++ = 0x10;
	/* WPS State 0x1044: Configured for GO/AP */
	p2p_put_be16(p, 0x1044); p += 2;
	p2p_put_be16(p, 1); p += 2;
	*p++ = as_ap ? 0x02 : 0x01;
	/* Response Type 0x103B */
	p2p_put_be16(p, 0x103B); p += 2;
	p2p_put_be16(p, 1); p += 2;
	*p++ = as_ap ? AIROC_WPS_RESP_AP : AIROC_WPS_RESP_ENROLLEE;
	/* UUID-E 0x1047 */
	p2p_put_be16(p, 0x1047); p += 2;
	p2p_put_be16(p, 16); p += 2;
	memcpy(p, uuid, 16); p += 16;
	/* Config Methods 0x1008 */
	p2p_put_be16(p, 0x1008); p += 2;
	p2p_put_be16(p, 2); p += 2;
	p2p_put_be16(p, config_methods); p += 2;
	/* Primary Device Type 0x1054 */
	p2p_put_be16(p, 0x1054); p += 2;
	p2p_put_be16(p, 8); p += 2;
	p2p_put_be16(p, 0x0001); p += 2;
	memcpy(p, "\x00\x50\xf2\x04", 4); p += 4;
	p2p_put_be16(p, 0x0001); p += 2;
	/* Device Name 0x1011 */
	p2p_put_be16(p, 0x1011); p += 2;
	p2p_put_be16(p, (uint16_t)nlen); p += 2;
	memcpy(p, name, nlen); p += nlen;
	/* RF Bands 0x103C = 2.4 GHz */
	p2p_put_be16(p, 0x103C); p += 2;
	p2p_put_be16(p, 1); p += 2;
	*p++ = 0x01;

	if (sel_reg_pbc) {
		/* Selected Registrar 0x1041 = true */
		p2p_put_be16(p, 0x1041); p += 2;
		p2p_put_be16(p, 1); p += 2;
		*p++ = 0x01;
		/* Device Password ID 0x1012 = PushButton */
		p2p_put_be16(p, 0x1012); p += 2;
		p2p_put_be16(p, 2); p += 2;
		p2p_put_be16(p, AIROC_WPS_DEV_PWD_PBC); p += 2;
		/* Selected Registrar Config Methods 0x1053 = PBC */
		p2p_put_be16(p, 0x1053); p += 2;
		p2p_put_be16(p, 2); p += 2;
		p2p_put_be16(p, AIROC_WPS_CONFIG_PBC); p += 2;
	}

	return (uint16_t)(p - buf);
}

/**
 * Build P2P IE attributes (Capability + Device Info).
 * @group_cap: P2P Group Capability bitmap (set bit0=GO for SoftAP GO).
 */
uint16_t p2p_build_p2p_attrs(uint8_t *buf, size_t buflen,
				    const uint8_t *dev_mac, const char *name,
				    uint8_t group_cap)
{
	size_t nlen = strlen(name);
	size_t di_len;
	size_t need;
	uint8_t *p = buf;
	uint8_t *di;

	if (nlen > 32) {
		nlen = 32;
	}
	/* Device Info: mac(6)+cfg(2)+prim(8)+num_sec(1)+WPS name(4+nlen) */
	di_len = 6 + 2 + 8 + 1 + 4 + nlen;
	need = 1 + 2 + 2 + 1 + 2 + di_len;
	if (need > buflen) {
		return 0;
	}

	/* Capability attr id=2 */
	*p++ = 2;
	p2p_put_le16(p, 2); p += 2;
	*p++ = 0x25; /* Device Capability: Service Discovery | P2P Concurrent | P2P Device */
	*p++ = group_cap;

	/* Device Info attr id=13 */
	*p++ = 13;
	p2p_put_le16(p, (uint16_t)di_len); p += 2;
	di = p;
	memcpy(di, dev_mac, 6); di += 6;
	p2p_put_be16(di, 0x0180); di += 2; /* config methods: PBC | Display */
	p2p_put_be16(di, 0x0001); di += 2;
	memcpy(di, "\x00\x50\xf2\x04", 4); di += 4;
	p2p_put_be16(di, 0x0001); di += 2;
	*di++ = 0; /* no secondary types */
	p2p_put_be16(di, 0x1011); di += 2;
	p2p_put_be16(di, (uint16_t)nlen); di += 2;
	memcpy(di, name, nlen); di += nlen;
	p = di;

	return (uint16_t)(p - buf);
}

/* ---- extracted from airoc_wifi_p2p.c L520-526 ---- */
void p2p_store_disc_wps_blob(void)
{
	if (p2p_wps_ie_len > 0 && p2p_wps_ie_len <= sizeof(p2p_disc_wps_ie_data)) {
		memcpy(p2p_disc_wps_ie_data, p2p_wps_ie_data, p2p_wps_ie_len);
		p2p_disc_wps_ie_len = p2p_wps_ie_len;
	}
}

/* ---- extracted from airoc_wifi_p2p.c L558-754 ---- */
int p2p_ie_add(whd_interface_t ifp, const uint8_t *oui, uint8_t subtype,
		      const uint8_t *data, uint16_t len, uint16_t pktflag)
{
	whd_result_t ret;

	ret = whd_wifi_manage_custom_ie(ifp, WHD_ADD_CUSTOM_IE, oui, subtype,
					data, len, pktflag);
	if (ret != WHD_SUCCESS) {
		LOG_WRN("vndr_ie add (bsscfg=%u flag=0x%x oui=%02x%02x%02x "
			"type=%u len=%u) failed: %d (0x%x)",
			ifp->bsscfgidx, pktflag, oui[0], oui[1], oui[2],
			subtype, len, ret, ret);
		return -EIO;
	}
	return 0;
}

int p2p_ie_add_wps_p2p(whd_interface_t ifp, uint16_t pktflag)
{
	static const uint8_t wps_oui[3] = { 0x00, 0x50, 0xf2 };
	static const uint8_t p2p_oui[3] = { 0x50, 0x6f, 0x9a };
	int err;

	err = p2p_ie_add(ifp, wps_oui, AIROC_WPS_IE_OUI_TYPE, p2p_wps_ie_data,
			 p2p_wps_ie_len, pktflag);
	if (err) {
		return err;
	}
	err = p2p_ie_add(ifp, p2p_oui, AIROC_P2P_IE_OUI_TYPE, p2p_p2p_ie_data,
			 p2p_p2p_ie_len, pktflag);
	if (err) {
		(void)whd_wifi_manage_custom_ie(
			ifp, WHD_REMOVE_CUSTOM_IE, wps_oui, AIROC_WPS_IE_OUI_TYPE,
			p2p_wps_ie_data, p2p_wps_ie_len, pktflag);
		return err;
	}
	return 0;
}

int p2p_build_discovery_ie_blobs(whd_mac_t *mac_out, uint8_t group_cap)
{
	whd_interface_t ifp = p2p_sta_if;
	whd_mac_t mac;
	bool as_ap = (group_cap & AIROC_P2P_GROUP_CAP_GO) != 0;

	if (ifp == NULL) {
		return -ENODEV;
	}

	if (whd_wifi_get_mac_address(ifp, &mac) != WHD_SUCCESS) {
		LOG_WRN("Primary MAC read failed");
		memset(mac.octet, 0, sizeof(mac.octet));
	}
	/*
	 * P2P Device Address must be the discovery MAC on both disc and GO
	 * IEs. Using the SoftAP BSSID here made phones list two AIROC-P2P
	 * entries (disc MAC + GO MAC).
	 */
	if (p2p_disc_dev_mac_valid) {
		memcpy(mac.octet, p2p_disc_dev_mac, 6);
	} else if (as_ap && p2p_ctx.group_if != NULL) {
		whd_mac_t ap_mac;

		if (whd_wifi_get_mac_address(p2p_ctx.group_if, &ap_mac) ==
		    WHD_SUCCESS) {
			p2p_derive_disc_mac_from_go(ap_mac.octet);
			if (p2p_disc_dev_mac_valid) {
				memcpy(mac.octet, p2p_disc_dev_mac, 6);
			} else {
				mac = ap_mac;
			}
		}
	} else {
		mac.octet[0] |= 0x02;
	}
	if (mac_out) {
		*mac_out = mac;
	}

	p2p_wps_ie_len = p2p_build_wps_attrs(p2p_wps_ie_data,
					     sizeof(p2p_wps_ie_data),
					     AIROC_P2P_DEV_NAME, as_ap,
					     as_ap && p2p_go_wps_pbc_active,
					     AIROC_WPS_CONFIG_PBC_DISPLAY);
	p2p_p2p_ie_len = p2p_build_p2p_attrs(p2p_p2p_ie_data,
					     sizeof(p2p_p2p_ie_data),
					     mac.octet, AIROC_P2P_DEV_NAME,
					     group_cap);
	if (p2p_wps_ie_len == 0 || p2p_p2p_ie_len == 0) {
		LOG_ERR("Failed to build discovery IEs");
		return -ENOMEM;
	}
	return 0;
}

/**
 * Install WPS + P2P IEs for discovery (probe resp/req).
 * Follow Linux brcmf_p2p_enable_discovery order: SCAN → wsec → vndr_ie.
 */
int p2p_install_discovery_ies(void)
{
	whd_interface_t ifp = p2p_sta_if;
	struct whd_interface shadow;
	whd_interface_t disc_ifp;
	whd_mac_t mac;
	int err;

	if (ifp == NULL) {
		return -ENODEV;
	}

	if (p2p_ies_installed) {
		return 0;
	}

	err = p2p_build_discovery_ie_blobs(&mac, 0x00);
	if (err) {
		return err;
	}

	p2p_disc_bss_if = ifp;
	disc_ifp = p2p_bsscfg_shadow(ifp, AIROC_P2P_BSSCFG_DISC, &shadow);

	/*
	 * Linux: p2p_disc=1 (already done), then WL_P2P_DISC_ST_SCAN, then
	 * bsscfg:wsec=AES so probe responses carry privacy + accept vndr_ie.
	 */
	(void)p2p_set_state(ifp, AIROC_P2P_DISC_ST_SCAN, 0, 0);
	(void)p2p_bsscfg_set_int(ifp, AIROC_P2P_BSSCFG_DISC, "wsec", AIROC_WSEC_AES_ENABLED);

	/* Probe-response IEs first (what the phone needs), then probe-req */
	err = p2p_ie_add_wps_p2p(disc_ifp, VENDOR_IE_PROBE_RESPONSE);
	if (!err) {
		(void)p2p_ie_add_wps_p2p(disc_ifp, VENDOR_IE_PROBE_REQUEST);
		p2p_ies_installed = true;
		p2p_ies_on_disc_bss = true;
		p2p_store_disc_wps_blob();
		if (p2p_bsscfg_get_mac(ifp, AIROC_P2P_BSSCFG_DISC, p2p_disc_dev_mac) == 0) {
			p2p_disc_dev_mac_valid = true;
		} else {
			p2p_remember_disc_mac(mac.octet);
		}
		return 0;
	}

	LOG_WRN("bsscfg1 PRBRSP IE failed — FW may not allow vndr_ie on disc; "
		"phone Wi-Fi Direct will not list us in find mode. "
		"Use: wifi p2p group add (autonomous GO with P2P IEs)");
	/*
	 * bsscfg0 fallback does not appear in P2P disc probe responses
	 * (confirmed by pcap) — skip it to avoid false hope.
	 */
	return err;
}

/**
 * Install WPS + P2P IEs on SoftAP GO so phones list us under Wi-Fi Direct.
 * SoftAP bsscfg accepts vndr_ie (disc bsscfg on 43439 often does not).
 */
int p2p_install_go_ies(whd_interface_t ap_ifp)
{
	whd_mac_t mac;
	int err;
	int prb_err;

	if (ap_ifp == NULL) {
		return -ENODEV;
	}

	p2p_go_wps_pbc_active = false;
	err = p2p_build_discovery_ie_blobs(&mac, AIROC_P2P_GROUP_CAP_GO);
	if (err) {
		return err;
	}

	/*
	 * Register each packet type separately, as brcmfmac does. A combined
	 * BEACON|PROBE_RESPONSE flag was accepted by firmware but only ever
	 * put the P2P IE in beacons, leaving Probe Responses without it.
	 */
	err = p2p_ie_add_wps_p2p(ap_ifp, VENDOR_IE_BEACON);
	if (err) {
		LOG_WRN("GO beacon IE add failed (%d)", err);
		return err;
	}
	prb_err = p2p_ie_add_wps_p2p(ap_ifp, VENDOR_IE_PROBE_RESPONSE);
	if (prb_err) {
		LOG_WRN("GO ProbeResp IE add failed (%d) — phone may not "
			"recognise the group in Wi-Fi Direct", prb_err);
	}

	p2p_ies_installed = true;
	/* Do not clear p2p_ies_on_disc_bss — SoftAP IEs are additive; disc
	 * ProbeResp still needs Selected Registrar after Provision Discovery.
	 */
	return 0;
}

/* ---- extracted from airoc_wifi_p2p.c L1022-1064 ---- */
void p2p_remove_discovery_ies(void)
{
	static const uint8_t wps_oui[3] = { 0x00, 0x50, 0xf2 };
	static const uint8_t p2p_oui[3] = { 0x50, 0x6f, 0x9a };
	const uint16_t flags[] = {
		VENDOR_IE_PROBE_RESPONSE,
		VENDOR_IE_PROBE_REQUEST,
		VENDOR_IE_BEACON,
	};
	whd_interface_t ifaces[3];
	struct whd_interface shadow;
	unsigned n = 0;
	unsigned i, f;

	if (!p2p_ies_installed) {
		return;
	}

	if (p2p_sta_if) {
		ifaces[n++] = p2p_sta_if;
		ifaces[n++] = p2p_bsscfg_shadow(p2p_sta_if, AIROC_P2P_BSSCFG_DISC, &shadow);
	}
	if (p2p_ctx.group_if) {
		ifaces[n++] = p2p_ctx.group_if;
	}

	for (i = 0; i < n; i++) {
		for (f = 0; f < ARRAY_SIZE(flags); f++) {
			(void)whd_wifi_manage_custom_ie(
				ifaces[i], WHD_REMOVE_CUSTOM_IE, p2p_oui, AIROC_P2P_IE_OUI_TYPE,
				p2p_p2p_ie_data, p2p_p2p_ie_len, flags[f]);
			(void)whd_wifi_manage_custom_ie(
				ifaces[i], WHD_REMOVE_CUSTOM_IE, wps_oui, AIROC_WPS_IE_OUI_TYPE,
				p2p_wps_ie_data, p2p_wps_ie_len, flags[f]);
		}
	}

	p2p_ies_installed = false;
	p2p_ies_on_disc_bss = false;
	p2p_disc_wps_ie_len = 0;
	p2p_disc_dev_mac_valid = false;
	memset(p2p_disc_dev_mac, 0, sizeof(p2p_disc_dev_mac));
}

