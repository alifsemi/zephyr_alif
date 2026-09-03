/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef AIROC_WIFI_P2P_H
#define AIROC_WIFI_P2P_H

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <whd.h>
#include <whd_wlioctl.h>
#include <whd_wifi_p2p.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware BSSCFG / ifidx layout (WIFI5 + p2p_disc + p2p_ifadd GO) */
#define AIROC_P2P_BSSCFG_STA   0
#define AIROC_P2P_BSSCFG_DISC  1
#define AIROC_P2P_BSSCFG_GO    2
#define AIROC_P2P_IFIDX_STA    0
#define AIROC_P2P_IFIDX_GO     1
#define AIROC_P2P_GO_IFNAME    "wlan2"

#define AIROC_WPS_IE_OUI_TYPE      0x04
#define AIROC_P2P_IE_OUI_TYPE      0x09
#define AIROC_WSEC_AES_ENABLED     0x0004
#define AIROC_WSEC_SES_OW_ENABLED  0x0040
#define AIROC_P2P_OP_CLASS_2G4     0x04

/* P2P discovery states (firmware-level, for wl_p2p_disc_st_t.state) */
#define AIROC_P2P_DISC_ST_SCAN    0
#define AIROC_P2P_DISC_ST_LISTEN  1
#define AIROC_P2P_DISC_ST_SEARCH  2

/* P2P interface types for wl_p2p_if_t.interface_type */
#define AIROC_P2P_IF_CLIENT       0
#define AIROC_P2P_IF_GO           1

/* P2P default timing parameters */
#define AIROC_P2P_LISTEN_DWELL_MS        500
#define AIROC_P2P_SEARCH_DWELL_MS        200
#define AIROC_P2P_DISC_TIMEOUT_DEFAULT_S 30
#define AIROC_P2P_MAX_PEERS              CONFIG_WIFI_P2P_MAX_PEERS

/* Social channels for P2P discovery */
#define AIROC_P2P_SOCIAL_CH_1   1
#define AIROC_P2P_SOCIAL_CH_6   6
#define AIROC_P2P_SOCIAL_CH_11  11

/* P2P driver state machine */
enum airoc_p2p_state {
	AIROC_P2P_STATE_IDLE = 0,
	AIROC_P2P_STATE_DISCOVERING,
	AIROC_P2P_STATE_LISTENING,
	AIROC_P2P_STATE_GO_NEGOTIATING,
	AIROC_P2P_STATE_GROUP_FORMED,
};

/* Cached P2P peer information */
struct airoc_p2p_peer {
	uint8_t  mac[6];
	int8_t   rssi;
	bool     valid;
	char     name[WIFI_P2P_DEVICE_NAME_MAX_LEN + 1];
};

/* P2P context — all mutable state for one P2P session */
struct airoc_p2p_ctx {
	enum airoc_p2p_state state;
	whd_interface_t      disc_if;       /* P2P discovery interface */
	whd_interface_t      group_if;      /* P2P group interface (GO or Client) */
	struct net_if       *iface;         /* Zephyr net_if for mgmt events */
	uint16_t             event_index;   /* WHD event handler registration index */
	bool                 disc_enabled;  /* p2p_disc IOVAR is active */
	bool                 is_go;         /* acting as Group Owner */
	bool                 used_p2p_ifadd; /* firmware p2p_ifadd succeeded */
	uint8_t              go_channel;    /* SoftAP/GO operating channel */
	struct k_work_delayable disc_timeout_work;
	struct k_work_delayable go_client_poll_work;
	struct k_work        event_work;
	struct k_mutex       lock;

	/* peer cache */
	struct airoc_p2p_peer peers[AIROC_P2P_MAX_PEERS];
	uint8_t              peer_count;
};

/**
 * Initialise P2P context (called once from airoc_init).
 */
int airoc_p2p_init(whd_driver_t whd_drv, whd_interface_t sta_if);

/**
 * Bind the Zephyr net_if used for P2P mgmt events (called from iface init).
 */
void airoc_p2p_set_iface(struct net_if *iface);

/**
 * Dispatch a P2P operation from wifi_mgmt_ops.p2p_oper.
 */
int airoc_p2p_oper(const struct device *dev, struct net_if *iface,
		   struct wifi_p2p_params *params);

/**
 * De-initialise P2P, tear down any running group/discovery.
 */
void airoc_p2p_deinit(void);

/**
 * Return pointer to P2P context (for status queries).
 */
struct airoc_p2p_ctx *airoc_p2p_get_ctx(void);

/**
 * Inspect an inbound Ethernet frame for EAPOL (WPS enrollee traffic).
 * Returns true when the frame was consumed and must not reach the IP stack.
 */
bool airoc_p2p_eapol_rx(const uint8_t *frame, uint32_t len);

/**
 * WPS-PBC finished (credentials delivered). Clear Selected Registrar IEs and
 * disarm the registrar so the phone's PSK rejoin is not interrupted by EAP.
 */
void airoc_p2p_wps_pbc_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* AIROC_WIFI_P2P_H */
