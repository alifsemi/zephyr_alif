/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal WPS registrar (EAP-WSC, PBC) for AIROC P2P GO — no hostap.
 */

#ifndef AIROC_WIFI_WPS_REG_H
#define AIROC_WIFI_WPS_REG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Arm registrar with credentials to hand out (SSID + PSK) and our GO MAC.
 * Call when PBC Selected Registrar is advertised.
 */
int airoc_wps_reg_arm(const uint8_t go_mac[6], const char *ssid, const char *psk);

/** Cancel an in-progress or armed session. */
void airoc_wps_reg_disarm(void);

/** SoftAP saw an association — start EAP Identity Request toward peer. */
void airoc_wps_reg_on_assoc(const uint8_t peer_mac[6]);

/**
 * Handle inbound Ethernet frame if it is EAPOL for this registrar.
 * @return true if consumed (do not pass to IP stack).
 */
bool airoc_wps_reg_eapol_rx(const uint8_t *frame, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* AIROC_WIFI_WPS_REG_H */
