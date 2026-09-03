/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal Wi-Fi Simple Config registrar for P2P GO (PBC only).
 * Implements EAP-WSC M1–M8 + WSC_Done so a phone can obtain DIRECT-* PSK.
 */

#include "airoc_wifi_wps_reg.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>
#include <string.h>
#include <errno.h>

#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <mbedtls/aes.h>
#include <mbedtls/dhm.h>
#include <mbedtls/bignum.h>

#include <whd_network_types.h>
#include <whd_buffer_api.h>
#include <airoc_wifi.h>
#include <airoc_wifi_p2p.h>

LOG_MODULE_DECLARE(infineon_airoc_wifi, CONFIG_WIFI_LOG_LEVEL);

/* WPS attribute types */
#define WPS_ATTR_AP_CHANNEL           0x1001
#define WPS_ATTR_ASSOC_STATE          0x1002
#define WPS_ATTR_AUTH_TYPE            0x1003
#define WPS_ATTR_AUTH_TYPE_FLAGS      0x1004
#define WPS_ATTR_AUTHENTICATOR        0x1005
#define WPS_ATTR_CONFIG_METHODS       0x1008
#define WPS_ATTR_CONFIG_ERROR         0x1009
#define WPS_ATTR_CONFIRM_URL4         0x100A
#define WPS_ATTR_CONFIRM_URL6         0x100B
#define WPS_ATTR_CONN_TYPE            0x100C
#define WPS_ATTR_CONN_TYPE_FLAGS      0x100D
#define WPS_ATTR_CREDENTIAL           0x100E
#define WPS_ATTR_DEVICE_NAME          0x1011
#define WPS_ATTR_DEVICE_PASSWORD_ID   0x1012
#define WPS_ATTR_E_HASH1              0x1014
#define WPS_ATTR_E_HASH2              0x1015
#define WPS_ATTR_E_SNONCE1            0x1016
#define WPS_ATTR_E_SNONCE2            0x1017
#define WPS_ATTR_ENCR_SETTINGS        0x1018
#define WPS_ATTR_ENCR_TYPE            0x100F
#define WPS_ATTR_ENCR_TYPE_FLAGS      0x1010
#define WPS_ATTR_ENROLLEE_NONCE       0x101A
#define WPS_ATTR_KEY_WRAP_AUTH        0x101E
#define WPS_ATTR_MAC_ADDR             0x1020
#define WPS_ATTR_MANUFACTURER         0x1021
#define WPS_ATTR_MSG_TYPE             0x1022
#define WPS_ATTR_MODEL_NAME           0x1023
#define WPS_ATTR_MODEL_NUMBER         0x1024
#define WPS_ATTR_NETWORK_INDEX        0x1026
#define WPS_ATTR_NETWORK_KEY          0x1027
#define WPS_ATTR_OS_VERSION           0x102D
#define WPS_ATTR_PRIMARY_DEV_TYPE     0x1054
#define WPS_ATTR_PSK_CURRENT          0x102C
#define WPS_ATTR_PUBLIC_KEY           0x1032
#define WPS_ATTR_RADIO_BANDS          0x103C
#define WPS_ATTR_REGISTRAR_NONCE      0x1039
#define WPS_ATTR_R_HASH1              0x103D
#define WPS_ATTR_R_HASH2              0x103E
#define WPS_ATTR_R_SNONCE1            0x103F
#define WPS_ATTR_R_SNONCE2            0x1040
#define WPS_ATTR_SELECTED_REGISTRAR   0x1041
#define WPS_ATTR_SERIAL_NUMBER        0x1042
#define WPS_ATTR_SSID                 0x1045
#define WPS_ATTR_UUID_E               0x1047
#define WPS_ATTR_UUID_R               0x1048
#define WPS_ATTR_VERSION              0x104A
#define WPS_ATTR_WFA_EXT              0x1049
#define WPS_ATTR_VERSION2             0x00 /* inside WFA ext */

#define WPS_MSG_M1  0x04
#define WPS_MSG_M2  0x05
#define WPS_MSG_M3  0x07
#define WPS_MSG_M4  0x08
#define WPS_MSG_M5  0x09
#define WPS_MSG_M6  0x0A
#define WPS_MSG_M7  0x0B
#define WPS_MSG_M8  0x0C

#define WPS_AUTH_WPA2PSK  0x0020
#define WPS_ENCR_AES      0x0008
#define WPS_DEV_PWD_PBC   0x0004

#define WSC_OP_START  0x01
#define WSC_OP_ACK    0x02
#define WSC_OP_NACK   0x03
#define WSC_OP_MSG    0x04
#define WSC_OP_DONE   0x05

#define EAP_CODE_REQUEST  1
#define EAP_CODE_RESPONSE 2
#define EAP_CODE_SUCCESS  3
#define EAP_CODE_FAILURE  4
#define EAP_TYPE_IDENTITY 1
#define EAP_TYPE_EXPANDED 254

#define WPS_DH_LEN 192
#define WPS_NONCE_LEN 16
#define WPS_HASH_LEN 32
#define WPS_AUTHKEY_LEN 32
#define WPS_KEYWRAP_LEN 16
#define WPS_AUTHENTICATOR_LEN 8
#define WPS_UUID_LEN 16
#define WPS_MSG_MAX 640
#define WPS_LAST_MAX 640

/* Wi-Fi Simple Config DH group (1536-bit MODP) */
static const char wps_dh_p_hex[] =
	"FFFFFFFFFFFFFFFFC90FDAA22168C234C4C6628B80DC1CD1"
	"29024E088A67CC74020BBEA63B139B22514A08798E3404DD"
	"EF9519B3CD3A431B302B0A6DF25F14374FE1356D6D51C245"
	"E485B576625E7EC6F44C42E9A637ED6B0BFF5CB6F406B7ED"
	"EE386BFB5A899FA5AE9F24117C4B1FE649286651ECE45B3D"
	"C2007CB8A163BF0598DA48361C55D39A69163FA8FD24CF5F"
	"83655D23DCA3AD961C62F356208552BB9ED529077096966D"
	"670C354E4ABC9804F1746C08CA237327FFFFFFFFFFFFFFFF";

enum wps_state {
	WPS_ST_IDLE = 0,
	WPS_ST_ARMED,
	WPS_ST_WAIT_ID,
	WPS_ST_WAIT_M1,
	WPS_ST_WAIT_M3,
	WPS_ST_WAIT_M5,
	WPS_ST_WAIT_M7,
	WPS_ST_WAIT_DONE,
	WPS_ST_DONE,
};

struct wps_attr_view {
	const uint8_t *enrollee_nonce;
	const uint8_t *registrar_nonce;
	const uint8_t *uuid_e;
	const uint8_t *mac_addr;
	const uint8_t *public_key;
	const uint8_t *e_hash1;
	const uint8_t *e_hash2;
	const uint8_t *authenticator;
	const uint8_t *encr_settings;
	uint16_t encr_settings_len;
	uint8_t msg_type;
};

struct wps_reg {
	enum wps_state state;
	bool armed;
	uint8_t peer[6];
	uint8_t go_mac[6];
	char ssid[33];
	char psk[65];
	uint8_t eap_id;

	uint8_t nonce_e[WPS_NONCE_LEN];
	uint8_t nonce_r[WPS_NONCE_LEN];
	uint8_t uuid_r[WPS_UUID_LEN];
	uint8_t pk_e[WPS_DH_LEN];
	uint8_t pk_r[WPS_DH_LEN];
	uint8_t e_hash1[WPS_HASH_LEN];
	uint8_t e_hash2[WPS_HASH_LEN];
	uint8_t r_s1[WPS_NONCE_LEN];
	uint8_t r_s2[WPS_NONCE_LEN];
	uint8_t psk1[16];
	uint8_t psk2[16];
	uint8_t authkey[WPS_AUTHKEY_LEN];
	uint8_t keywrapkey[WPS_KEYWRAP_LEN];

	mbedtls_dhm_context dhm;

	uint8_t last_msg[WPS_LAST_MAX];
	size_t last_msg_len;

	uint8_t rx_wsc[WPS_MSG_MAX];
	size_t rx_wsc_len;

	struct k_work work;
	uint8_t pending_op; /* what work should send */
};

static struct wps_reg g_wps;

/* Use system workqueue (stack sized in board conf) to save a dedicated stack. */

/* pending_op values */
#define WPS_OP_ID_REQ     1
#define WPS_OP_WSC_START  2
#define WPS_OP_M2         3
#define WPS_OP_M4         4
#define WPS_OP_M6         5
#define WPS_OP_M8         6
#define WPS_OP_EAP_FAIL   7

static int wps_rng(void *ctx, unsigned char *out, size_t len)
{
	ARG_UNUSED(ctx);
	if (sys_csrand_get(out, len) == 0) {
		return 0;
	}
	sys_rand_get(out, len);
	return 0;
}

static void wps_put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)(v);
}

static void wps_put_be32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)(v);
}

static uint16_t wps_get_be16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static int wps_hmac_sha256(const uint8_t *key, size_t key_len,
			   const uint8_t *data, size_t data_len,
			   uint8_t out[32])
{
	const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

	if (info == NULL) {
		return -1;
	}
	return mbedtls_md_hmac(info, key, key_len, data, data_len, out);
}

static int wps_sha256(const uint8_t *data, size_t len, uint8_t out[32])
{
	return mbedtls_sha256(data, len, out, 0);
}

/* WPS KDF (Wi-Fi Easy and Secure Key Derivation) */
static int wps_kdf(const uint8_t *kdk, const uint8_t *label, size_t label_len,
		   uint8_t *out, size_t out_bits)
{
	uint8_t i_buf[4];
	uint8_t key_bits[4];
	uint8_t hash[32];
	size_t out_len = (out_bits + 7) / 8;
	size_t offset = 0;
	uint32_t i = 1;

	wps_put_be32(key_bits, (uint32_t)out_bits);

	while (offset < out_len) {
		uint8_t msg[4 + 64 + 4];
		size_t mlen = 0;
		size_t n;

		wps_put_be32(i_buf, i);
		memcpy(msg, i_buf, 4);
		mlen = 4;
		memcpy(msg + mlen, label, label_len);
		mlen += label_len;
		memcpy(msg + mlen, key_bits, 4);
		mlen += 4;

		if (wps_hmac_sha256(kdk, 32, msg, mlen, hash) != 0) {
			return -1;
		}
		n = out_len - offset;
		if (n > 32) {
			n = 32;
		}
		memcpy(out + offset, hash, n);
		offset += n;
		i++;
	}
	return 0;
}

static uint8_t *wps_attr_put(uint8_t *p, uint8_t *end, uint16_t type,
			     const void *data, uint16_t len)
{
	if (p + 4 + len > end) {
		return NULL;
	}
	wps_put_be16(p, type);
	wps_put_be16(p + 2, len);
	if (len && data) {
		memcpy(p + 4, data, len);
	}
	return p + 4 + len;
}

static uint8_t *wps_attr_put_be16(uint8_t *p, uint8_t *end, uint16_t type,
				 uint16_t val)
{
	uint8_t b[2];

	wps_put_be16(b, val);
	return wps_attr_put(p, end, type, b, 2);
}

static uint8_t *wps_attr_put_be32(uint8_t *p, uint8_t *end, uint16_t type,
				  uint32_t val)
{
	uint8_t b[4];

	wps_put_be32(b, val);
	return wps_attr_put(p, end, type, b, 4);
}

static uint8_t *wps_attr_put_u8(uint8_t *p, uint8_t *end, uint16_t type,
				uint8_t val)
{
	return wps_attr_put(p, end, type, &val, 1);
}

/* WFA Vendor Extension: Version2 = 0x20 (required by Android WPS2) */
static uint8_t *wps_attr_put_version2(uint8_t *p, uint8_t *end)
{
	static const uint8_t ext[] = {
		0x00, 0x37, 0x2a, /* WFA OUI */
		0x00, 0x01, 0x20, /* Version2 id=0 len=1 val=0x20 */
	};

	return wps_attr_put(p, end, WPS_ATTR_WFA_EXT, ext, sizeof(ext));
}

static const uint8_t *wps_attr_get(const uint8_t *buf, size_t len,
				   uint16_t type, uint16_t *out_len)
{
	size_t i = 0;

	while (i + 4 <= len) {
		uint16_t t = wps_get_be16(buf + i);
		uint16_t l = wps_get_be16(buf + i + 2);

		if (i + 4 + l > len) {
			return NULL;
		}
		if (t == type) {
			if (out_len) {
				*out_len = l;
			}
			return buf + i + 4;
		}
		i += 4 + l;
	}
	return NULL;
}

static int wps_parse_msg(const uint8_t *buf, size_t len, struct wps_attr_view *v)
{
	uint16_t l;
	const uint8_t *a;

	memset(v, 0, sizeof(*v));
	a = wps_attr_get(buf, len, WPS_ATTR_MSG_TYPE, &l);
	if (a && l == 1) {
		v->msg_type = a[0];
	}
	a = wps_attr_get(buf, len, WPS_ATTR_ENROLLEE_NONCE, &l);
	if (a && l == WPS_NONCE_LEN) {
		v->enrollee_nonce = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_REGISTRAR_NONCE, &l);
	if (a && l == WPS_NONCE_LEN) {
		v->registrar_nonce = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_UUID_E, &l);
	if (a && l == WPS_UUID_LEN) {
		v->uuid_e = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_MAC_ADDR, &l);
	if (a && l == 6) {
		v->mac_addr = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_PUBLIC_KEY, &l);
	if (a && l == WPS_DH_LEN) {
		v->public_key = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_E_HASH1, &l);
	if (a && l == WPS_HASH_LEN) {
		v->e_hash1 = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_E_HASH2, &l);
	if (a && l == WPS_HASH_LEN) {
		v->e_hash2 = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_AUTHENTICATOR, &l);
	if (a && l == WPS_AUTHENTICATOR_LEN) {
		v->authenticator = a;
	}
	a = wps_attr_get(buf, len, WPS_ATTR_ENCR_SETTINGS, &l);
	if (a && l > 16) {
		v->encr_settings = a;
		v->encr_settings_len = l;
	}
	return 0;
}

static int wps_derive_keys(struct wps_reg *w)
{
	uint8_t shared[WPS_DH_LEN];
	size_t shared_len = sizeof(shared);
	uint8_t dhkey[32];
	uint8_t kdk[32];
	uint8_t keys[32 + 16 + 32];
	uint8_t mac_nonce[WPS_NONCE_LEN + WPS_NONCE_LEN + 6];
	uint8_t pwd_hash[32];
	static const uint8_t label[] = "Wi-Fi Easy and Secure Key Derivation";
	int ret;

	ret = mbedtls_dhm_calc_secret(&w->dhm, shared, sizeof(shared),
				      &shared_len, wps_rng, NULL);
	if (ret != 0) {
		LOG_ERR("WPS: DH shared failed (%d)", ret);
		return -1;
	}
	/* Left-pad to 192 bytes */
	if (shared_len < WPS_DH_LEN) {
		memmove(shared + (WPS_DH_LEN - shared_len), shared, shared_len);
		memset(shared, 0, WPS_DH_LEN - shared_len);
		shared_len = WPS_DH_LEN;
	}

	if (wps_sha256(shared, WPS_DH_LEN, dhkey) != 0) {
		return -1;
	}

	/*
	 * KDK = HMAC-SHA-256_DHKey(N1 || EnrolleeMAC || N2)
	 * Match hostapd/wpa_supplicant (Android). Note: some WPS text
	 * drafts list N1||N2||MAC — that order fails phone AuthKey checks.
	 */
	memcpy(mac_nonce, w->nonce_e, WPS_NONCE_LEN);
	memcpy(mac_nonce + WPS_NONCE_LEN, w->peer, 6);
	memcpy(mac_nonce + WPS_NONCE_LEN + 6, w->nonce_r, WPS_NONCE_LEN);

	if (wps_hmac_sha256(dhkey, 32, mac_nonce, sizeof(mac_nonce), kdk) != 0) {
		return -1;
	}
	if (wps_kdf(kdk, label, sizeof(label) - 1, keys, 640) != 0) {
		return -1;
	}
	memcpy(w->authkey, keys, 32);
	memcpy(w->keywrapkey, keys + 32, 16);

	/*
	 * PSK1/PSK2 (hostapd wps_derive_psk): HMAC each half of the device
	 * password separately — NOT one HMAC of the full password.
	 * PBC password = "00000000" → PSK1=HMAC("0000"), PSK2=HMAC("0000").
	 */
	{
		static const uint8_t pwd[] = "00000000";
		const size_t pwd_len = 8;
		size_t half = (pwd_len + 1) / 2;

		if (wps_hmac_sha256(w->authkey, 32, pwd, half, pwd_hash) != 0) {
			return -1;
		}
		memcpy(w->psk1, pwd_hash, 16);
		if (wps_hmac_sha256(w->authkey, 32, pwd + half, pwd_len / 2,
				    pwd_hash) != 0) {
			return -1;
		}
		memcpy(w->psk2, pwd_hash, 16);
	}
	return 0;
}

static int wps_verify_authenticator(struct wps_reg *w, const uint8_t *msg,
				    size_t msg_len, const uint8_t *auth)
{
	/* Authenticator covers last_msg || msg without Authenticator attr */
	uint8_t hash[32];
	uint8_t tmp[WPS_LAST_MAX + WPS_MSG_MAX];
	size_t auth_off;
	size_t body_len;
	const uint8_t *a;
	uint16_t alen;

	a = wps_attr_get(msg, msg_len, WPS_ATTR_AUTHENTICATOR, &alen);
	if (a == NULL || alen != WPS_AUTHENTICATOR_LEN) {
		return -1;
	}
	auth_off = (size_t)(a - msg) - 4; /* include type/len of attr */
	if (auth_off + 4 + WPS_AUTHENTICATOR_LEN != msg_len) {
		/* Authenticator must be last attribute */
		body_len = auth_off;
	} else {
		body_len = auth_off;
	}
	if (w->last_msg_len + body_len > sizeof(tmp)) {
		return -1;
	}
	memcpy(tmp, w->last_msg, w->last_msg_len);
	memcpy(tmp + w->last_msg_len, msg, body_len);
	if (wps_hmac_sha256(w->authkey, 32, tmp, w->last_msg_len + body_len,
			    hash) != 0) {
		return -1;
	}
	if (memcmp(hash, auth, WPS_AUTHENTICATOR_LEN) != 0) {
		LOG_ERR("WPS: Authenticator mismatch");
		return -1;
	}
	return 0;
}

static int wps_add_authenticator(struct wps_reg *w, uint8_t *msg, size_t *msg_len,
				 size_t max_len)
{
	uint8_t hash[32];
	uint8_t tmp[WPS_LAST_MAX + WPS_MSG_MAX];
	uint8_t *p;

	if (w->last_msg_len + *msg_len > sizeof(tmp)) {
		return -1;
	}
	if (*msg_len + 4 + WPS_AUTHENTICATOR_LEN > max_len) {
		return -1;
	}
	memcpy(tmp, w->last_msg, w->last_msg_len);
	memcpy(tmp + w->last_msg_len, msg, *msg_len);
	if (wps_hmac_sha256(w->authkey, 32, tmp, w->last_msg_len + *msg_len,
			    hash) != 0) {
		return -1;
	}
	p = msg + *msg_len;
	wps_put_be16(p, WPS_ATTR_AUTHENTICATOR);
	wps_put_be16(p + 2, WPS_AUTHENTICATOR_LEN);
	memcpy(p + 4, hash, WPS_AUTHENTICATOR_LEN);
	*msg_len += 4 + WPS_AUTHENTICATOR_LEN;
	return 0;
}

static int wps_aes_128_cbc_encrypt(const uint8_t *key, uint8_t *iv_plain,
				   size_t plain_len, uint8_t *out,
				   size_t *out_len)
{
	mbedtls_aes_context aes;
	uint8_t iv[16];
	size_t pad;
	size_t i;

	/* PKCS#5 pad */
	pad = 16 - (plain_len % 16);
	if (plain_len + pad + 16 > *out_len) {
		return -1;
	}
	if (wps_rng(NULL, iv, 16) != 0) {
		return -1;
	}
	memcpy(out, iv, 16);
	memcpy(out + 16, iv_plain, plain_len);
	for (i = 0; i < pad; i++) {
		out[16 + plain_len + i] = (uint8_t)pad;
	}

	mbedtls_aes_init(&aes);
	if (mbedtls_aes_setkey_enc(&aes, key, 128) != 0) {
		mbedtls_aes_free(&aes);
		return -1;
	}
	if (mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, plain_len + pad,
				  iv, out + 16, out + 16) != 0) {
		mbedtls_aes_free(&aes);
		return -1;
	}
	mbedtls_aes_free(&aes);
	*out_len = 16 + plain_len + pad;
	return 0;
}

static int wps_aes_128_cbc_decrypt(const uint8_t *key, const uint8_t *in,
				   size_t in_len, uint8_t *out, size_t *out_len)
{
	mbedtls_aes_context aes;
	uint8_t iv[16];
	uint8_t pad;

	if (in_len < 32 || (in_len % 16) != 0) {
		return -1;
	}
	memcpy(iv, in, 16);
	mbedtls_aes_init(&aes);
	if (mbedtls_aes_setkey_dec(&aes, key, 128) != 0) {
		mbedtls_aes_free(&aes);
		return -1;
	}
	if (*out_len < in_len - 16) {
		mbedtls_aes_free(&aes);
		return -1;
	}
	if (mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, in_len - 16, iv,
				  in + 16, out) != 0) {
		mbedtls_aes_free(&aes);
		return -1;
	}
	mbedtls_aes_free(&aes);

	{
		size_t plain_len = in_len - 16;
		size_t i;

		pad = out[plain_len - 1];
		if (pad < 1 || pad > 16 || pad > plain_len) {
			return -1;
		}
		/* PKCS#7: every padding byte must equal the pad length. */
		for (i = 0; i < pad; i++) {
			if (out[plain_len - 1 - i] != pad) {
				return -1;
			}
		}
		*out_len = plain_len - pad;
	}
	return 0;
}

static int wps_build_key_wrap_auth(struct wps_reg *w, const uint8_t *plain,
				   size_t plain_len, uint8_t kwa[8])
{
	uint8_t hash[32];

	if (wps_hmac_sha256(w->authkey, 32, plain, plain_len, hash) != 0) {
		return -1;
	}
	memcpy(kwa, hash, 8);
	return 0;
}

static int wps_encrypt_settings(struct wps_reg *w, const uint8_t *plain,
				size_t plain_len, uint8_t *out, size_t *out_len)
{
	uint8_t buf[512];
	uint8_t kwa[8];
	size_t plen;

	if (plain_len + 4 + 8 > sizeof(buf)) {
		return -1;
	}
	memcpy(buf, plain, plain_len);
	plen = plain_len;
	if (wps_build_key_wrap_auth(w, buf, plen, kwa) != 0) {
		return -1;
	}
	wps_put_be16(buf + plen, WPS_ATTR_KEY_WRAP_AUTH);
	wps_put_be16(buf + plen + 2, 8);
	memcpy(buf + plen + 4, kwa, 8);
	plen += 12;
	return wps_aes_128_cbc_encrypt(w->keywrapkey, buf, plen, out, out_len);
}

static void wps_save_last(struct wps_reg *w, const uint8_t *msg, size_t len)
{
	if (len > sizeof(w->last_msg)) {
		len = sizeof(w->last_msg);
	}
	memcpy(w->last_msg, msg, len);
	w->last_msg_len = len;
}

static int wps_send_eth(struct wps_reg *w, const uint8_t *payload, size_t plen)
{
	uint8_t frame[14 + 4 + 600];
	size_t flen;

	if (14 + plen > sizeof(frame)) {
		return -ENOMEM;
	}
	memcpy(frame, w->peer, 6);
	memcpy(frame + 6, w->go_mac, 6);
	frame[12] = 0x88;
	frame[13] = 0x8e;
	memcpy(frame + 14, payload, plen);
	flen = 14 + plen;

	return airoc_wifi_send_raw_eth(frame, flen);
}

static int wps_send_eap(struct wps_reg *w, uint8_t code, const uint8_t *eap_data,
			size_t eap_data_len)
{
	/* EAPOL(4) + EAP hdr(4) + type/data */
	uint8_t eapol[4 + 4 + 600];
	uint16_t eap_len = (uint16_t)(4 + eap_data_len);

	if (eap_data_len + 4 > sizeof(eapol) - 4) {
		return -ENOMEM;
	}
	eapol[0] = 1; /* EAPOL version */
	eapol[1] = 0; /* EAP packet */
	eapol[2] = (uint8_t)(eap_len >> 8);
	eapol[3] = (uint8_t)(eap_len);
	eapol[4] = code;
	eapol[5] = w->eap_id;
	eapol[6] = (uint8_t)(eap_len >> 8);
	eapol[7] = (uint8_t)(eap_len);
	if (eap_data_len) {
		memcpy(eapol + 8, eap_data, eap_data_len);
	}
	return wps_send_eth(w, eapol, 4 + eap_len);
}

static int wps_send_eap_typed(struct wps_reg *w, uint8_t code, uint8_t type,
			      const uint8_t *data, size_t data_len)
{
	uint8_t body[1 + 600];

	if (data_len + 1 > sizeof(body)) {
		return -ENOMEM;
	}
	body[0] = type;
	if (data_len) {
		memcpy(body + 1, data, data_len);
	}
	return wps_send_eap(w, code, body, 1 + data_len);
}

static int wps_send_wsc(struct wps_reg *w, uint8_t opcode, const uint8_t *msg,
			size_t msg_len)
{
	uint8_t body[8 + WPS_MSG_MAX];
	size_t blen = 0;

	/* Expanded type header: type, vendor(3), vendor-type(4), op, flags */
	body[blen++] = EAP_TYPE_EXPANDED;
	body[blen++] = 0x00;
	body[blen++] = 0x37;
	body[blen++] = 0x2a; /* WFA OUI */
	body[blen++] = 0x00;
	body[blen++] = 0x00;
	body[blen++] = 0x00;
	body[blen++] = 0x01; /* SimpleConfig */
	body[blen++] = opcode;
	body[blen++] = 0x00; /* flags */
	if (msg_len) {
		if (blen + msg_len > sizeof(body)) {
			return -ENOMEM;
		}
		memcpy(body + blen, msg, msg_len);
		blen += msg_len;
	}
	/* send_eap expects data starting at type — body already has type */
	return wps_send_eap(w, EAP_CODE_REQUEST, body, blen);
}

static int wps_send_identity_req(struct wps_reg *w)
{
	w->eap_id++;
	return wps_send_eap_typed(w, EAP_CODE_REQUEST, EAP_TYPE_IDENTITY,
				  NULL, 0);
}

static int wps_send_wsc_start(struct wps_reg *w)
{
	w->eap_id++;
	return wps_send_wsc(w, WSC_OP_START, NULL, 0);
}

static int wps_build_m2(struct wps_reg *w, uint8_t *msg, size_t *msg_len)
{
	uint8_t *p = msg;
	uint8_t *end = msg + WPS_MSG_MAX;
	uint8_t version = 0x10;
	uint8_t msg_type = WPS_MSG_M2;
	uint8_t prim_dev[8] = {
		0x00, 0x01, 0x00, 0x50, 0xf2, AIROC_WPS_IE_OUI_TYPE, 0x00, 0x01
	}; /* Computer / PC */
	uint8_t rf = 0x01; /* 2.4 GHz */
	static const char mfg[] = "Alif";
	static const char model[] = CONFIG_AIROC_WIFI_P2P_DEVICE_NAME;
	static const char model_num[] = "1";
	static const char serial[] = "1";
	static const char devname[] = CONFIG_AIROC_WIFI_P2P_DEVICE_NAME;
	size_t len;

	p = wps_attr_put_u8(p, end, WPS_ATTR_VERSION, version);
	p = wps_attr_put_u8(p, end, WPS_ATTR_MSG_TYPE, msg_type);
	p = wps_attr_put(p, end, WPS_ATTR_ENROLLEE_NONCE, w->nonce_e,
			 WPS_NONCE_LEN);
	p = wps_attr_put(p, end, WPS_ATTR_REGISTRAR_NONCE, w->nonce_r,
			 WPS_NONCE_LEN);
	p = wps_attr_put(p, end, WPS_ATTR_UUID_R, w->uuid_r, WPS_UUID_LEN);
	p = wps_attr_put(p, end, WPS_ATTR_PUBLIC_KEY, w->pk_r, WPS_DH_LEN);
	p = wps_attr_put_be16(p, end, WPS_ATTR_AUTH_TYPE_FLAGS, 0x0022);
	p = wps_attr_put_be16(p, end, WPS_ATTR_ENCR_TYPE_FLAGS, 0x000c);
	p = wps_attr_put_u8(p, end, WPS_ATTR_CONN_TYPE_FLAGS, 0x01);
	p = wps_attr_put_be16(p, end, WPS_ATTR_CONFIG_METHODS, 0x0080);
	p = wps_attr_put(p, end, WPS_ATTR_MANUFACTURER, mfg, sizeof(mfg) - 1);
	p = wps_attr_put(p, end, WPS_ATTR_MODEL_NAME, model, sizeof(model) - 1);
	p = wps_attr_put(p, end, WPS_ATTR_MODEL_NUMBER, model_num,
			 sizeof(model_num) - 1);
	p = wps_attr_put(p, end, WPS_ATTR_SERIAL_NUMBER, serial,
			 sizeof(serial) - 1);
	p = wps_attr_put(p, end, WPS_ATTR_PRIMARY_DEV_TYPE, prim_dev, 8);
	p = wps_attr_put(p, end, WPS_ATTR_DEVICE_NAME, devname,
			 sizeof(devname) - 1);
	p = wps_attr_put_u8(p, end, WPS_ATTR_RADIO_BANDS, rf);
	p = wps_attr_put_be16(p, end, WPS_ATTR_ASSOC_STATE, 0);
	p = wps_attr_put_be16(p, end, WPS_ATTR_CONFIG_ERROR, 0);
	p = wps_attr_put_be16(p, end, WPS_ATTR_DEVICE_PASSWORD_ID,
			      WPS_DEV_PWD_PBC);
	p = wps_attr_put_be32(p, end, WPS_ATTR_OS_VERSION, 0x80000000);
	p = wps_attr_put_version2(p, end);
	if (p == NULL) {
		return -ENOMEM;
	}
	len = (size_t)(p - msg);
	if (wps_add_authenticator(w, msg, &len, WPS_MSG_MAX) != 0) {
		return -1;
	}
	*msg_len = len;
	return 0;
}

static int wps_build_m4(struct wps_reg *w, uint8_t *msg, size_t *msg_len)
{
	uint8_t *p = msg;
	uint8_t *end = msg + WPS_MSG_MAX;
	uint8_t plain[64];
	uint8_t *pp = plain;
	uint8_t *pend = plain + sizeof(plain);
	uint8_t enc[128];
	size_t enc_len = sizeof(enc);
	uint8_t r_hash1[32], r_hash2[32];
	uint8_t hash_in[16 + 16 + WPS_DH_LEN + WPS_DH_LEN];
	size_t len;

	/* R-Hash1 = HMAC(R-S1 || PSK1 || PK_E || PK_R) */
	memcpy(hash_in, w->r_s1, 16);
	memcpy(hash_in + 16, w->psk1, 16);
	memcpy(hash_in + 32, w->pk_e, WPS_DH_LEN);
	memcpy(hash_in + 32 + WPS_DH_LEN, w->pk_r, WPS_DH_LEN);
	if (wps_hmac_sha256(w->authkey, 32, hash_in, sizeof(hash_in),
			    r_hash1) != 0) {
		return -1;
	}
	memcpy(hash_in, w->r_s2, 16);
	memcpy(hash_in + 16, w->psk2, 16);
	if (wps_hmac_sha256(w->authkey, 32, hash_in, sizeof(hash_in),
			    r_hash2) != 0) {
		return -1;
	}

	pp = wps_attr_put(pp, pend, WPS_ATTR_R_SNONCE1, w->r_s1, 16);
	if (pp == NULL) {
		return -1;
	}
	if (wps_encrypt_settings(w, plain, (size_t)(pp - plain), enc,
				 &enc_len) != 0) {
		return -1;
	}

	p = wps_attr_put_u8(p, end, WPS_ATTR_VERSION, 0x10);
	p = wps_attr_put_u8(p, end, WPS_ATTR_MSG_TYPE, WPS_MSG_M4);
	p = wps_attr_put(p, end, WPS_ATTR_ENROLLEE_NONCE, w->nonce_e, 16);
	p = wps_attr_put(p, end, WPS_ATTR_REGISTRAR_NONCE, w->nonce_r, 16);
	p = wps_attr_put(p, end, WPS_ATTR_R_HASH1, r_hash1, 32);
	p = wps_attr_put(p, end, WPS_ATTR_R_HASH2, r_hash2, 32);
	p = wps_attr_put(p, end, WPS_ATTR_ENCR_SETTINGS, enc, (uint16_t)enc_len);
	p = wps_attr_put_version2(p, end);
	if (p == NULL) {
		return -1;
	}
	len = (size_t)(p - msg);
	if (wps_add_authenticator(w, msg, &len, WPS_MSG_MAX) != 0) {
		return -1;
	}
	*msg_len = len;
	return 0;
}

static int wps_build_m6(struct wps_reg *w, uint8_t *msg, size_t *msg_len)
{
	uint8_t *p = msg;
	uint8_t *end = msg + WPS_MSG_MAX;
	uint8_t plain[64];
	uint8_t *pp = plain;
	uint8_t enc[128];
	size_t enc_len = sizeof(enc);
	size_t len;

	pp = wps_attr_put(pp, plain + sizeof(plain), WPS_ATTR_R_SNONCE2,
			  w->r_s2, 16);
	if (pp == NULL) {
		return -1;
	}
	if (wps_encrypt_settings(w, plain, (size_t)(pp - plain), enc,
				 &enc_len) != 0) {
		return -1;
	}

	p = wps_attr_put_u8(p, end, WPS_ATTR_VERSION, 0x10);
	p = wps_attr_put_u8(p, end, WPS_ATTR_MSG_TYPE, WPS_MSG_M6);
	p = wps_attr_put(p, end, WPS_ATTR_ENROLLEE_NONCE, w->nonce_e, 16);
	p = wps_attr_put(p, end, WPS_ATTR_REGISTRAR_NONCE, w->nonce_r, 16);
	p = wps_attr_put(p, end, WPS_ATTR_ENCR_SETTINGS, enc, (uint16_t)enc_len);
	p = wps_attr_put_version2(p, end);
	if (p == NULL) {
		return -1;
	}
	len = (size_t)(p - msg);
	if (wps_add_authenticator(w, msg, &len, WPS_MSG_MAX) != 0) {
		return -1;
	}
	*msg_len = len;
	return 0;
}

static int wps_build_m8(struct wps_reg *w, uint8_t *msg, size_t *msg_len)
{
	uint8_t *p = msg;
	uint8_t *end = msg + WPS_MSG_MAX;
	uint8_t cred[256];
	uint8_t *c = cred;
	uint8_t *cend = cred + sizeof(cred);
	uint8_t plain[320];
	uint8_t *pp;
	uint8_t enc[400];
	size_t enc_len = sizeof(enc);
	size_t ssid_len = strlen(w->ssid);
	size_t psk_len = strlen(w->psk);
	size_t len;

	c = wps_attr_put_u8(c, cend, WPS_ATTR_NETWORK_INDEX, 1);
	c = wps_attr_put(c, cend, WPS_ATTR_SSID, w->ssid, (uint16_t)ssid_len);
	c = wps_attr_put_be16(c, cend, WPS_ATTR_AUTH_TYPE, WPS_AUTH_WPA2PSK);
	c = wps_attr_put_be16(c, cend, WPS_ATTR_ENCR_TYPE, WPS_ENCR_AES);
	c = wps_attr_put(c, cend, WPS_ATTR_NETWORK_KEY, w->psk, (uint16_t)psk_len);
	c = wps_attr_put(c, cend, WPS_ATTR_MAC_ADDR, w->peer, 6);
	if (c == NULL) {
		return -1;
	}

	pp = plain;
	pp = wps_attr_put(pp, plain + sizeof(plain), WPS_ATTR_CREDENTIAL, cred,
			  (uint16_t)(c - cred));
	if (pp == NULL) {
		return -1;
	}
	if (wps_encrypt_settings(w, plain, (size_t)(pp - plain), enc,
				 &enc_len) != 0) {
		return -1;
	}

	p = wps_attr_put_u8(p, end, WPS_ATTR_VERSION, 0x10);
	p = wps_attr_put_u8(p, end, WPS_ATTR_MSG_TYPE, WPS_MSG_M8);
	p = wps_attr_put(p, end, WPS_ATTR_ENROLLEE_NONCE, w->nonce_e, 16);
	p = wps_attr_put(p, end, WPS_ATTR_REGISTRAR_NONCE, w->nonce_r, 16);
	p = wps_attr_put(p, end, WPS_ATTR_ENCR_SETTINGS, enc, (uint16_t)enc_len);
	p = wps_attr_put_version2(p, end);
	if (p == NULL) {
		return -1;
	}
	len = (size_t)(p - msg);
	if (wps_add_authenticator(w, msg, &len, WPS_MSG_MAX) != 0) {
		return -1;
	}
	*msg_len = len;
	return 0;
}

static int wps_setup_dh(struct wps_reg *w)
{
	mbedtls_mpi P, G;
	int ret;

	mbedtls_mpi_init(&P);
	mbedtls_mpi_init(&G);
	mbedtls_dhm_free(&w->dhm);
	mbedtls_dhm_init(&w->dhm);

	ret = mbedtls_mpi_read_string(&P, 16, wps_dh_p_hex);
	if (ret == 0) {
		ret = mbedtls_mpi_lset(&G, 2);
	}
	if (ret == 0) {
		ret = mbedtls_dhm_set_group(&w->dhm, &P, &G);
	}
	if (ret == 0) {
		ret = mbedtls_dhm_make_public(&w->dhm, WPS_DH_LEN, w->pk_r,
					      WPS_DH_LEN, wps_rng, NULL);
	}
	mbedtls_mpi_free(&P);
	mbedtls_mpi_free(&G);
	if (ret != 0) {
		LOG_ERR("WPS: DH setup failed (%d)", ret);
	}
	return ret;
}

static int wps_process_m1(struct wps_reg *w, const uint8_t *msg, size_t len)
{
	struct wps_attr_view v;
	size_t olen;
	int ret;

	wps_parse_msg(msg, len, &v);
	if (v.msg_type != WPS_MSG_M1 || v.enrollee_nonce == NULL ||
	    v.public_key == NULL) {
		LOG_ERR("WPS: bad M1");
		return -1;
	}
	memcpy(w->nonce_e, v.enrollee_nonce, WPS_NONCE_LEN);
	memcpy(w->pk_e, v.public_key, WPS_DH_LEN);
	/* KDK uses Enrollee MAC from M1 (not just Ethernet SA) */
	if (v.mac_addr != NULL) {
		memcpy(w->peer, v.mac_addr, 6);
	}
	wps_save_last(w, msg, len);

	if (wps_rng(NULL, w->nonce_r, WPS_NONCE_LEN) != 0 ||
	    wps_rng(NULL, w->r_s1, WPS_NONCE_LEN) != 0 ||
	    wps_rng(NULL, w->r_s2, WPS_NONCE_LEN) != 0 ||
	    wps_rng(NULL, w->uuid_r, WPS_UUID_LEN) != 0) {
		return -1;
	}
	/* UUID version/variant bits */
	w->uuid_r[6] = (w->uuid_r[6] & 0x0f) | 0x40;
	w->uuid_r[8] = (w->uuid_r[8] & 0x3f) | 0x80;

	if (wps_setup_dh(w) != 0) {
		return -1;
	}

	olen = WPS_DH_LEN;
	ret = mbedtls_dhm_read_public(&w->dhm, w->pk_e, WPS_DH_LEN);
	if (ret != 0) {
		LOG_ERR("WPS: read peer PK failed (%d)", ret);
		return -1;
	}
	if (wps_derive_keys(w) != 0) {
		return -1;
	}

	w->pending_op = WPS_OP_M2;
	k_work_submit(&w->work);
	return 0;
}

static int wps_check_e_hash(struct wps_reg *w, const uint8_t *snonce,
			    const uint8_t *psk, const uint8_t *expected)
{
	uint8_t hash_in[16 + 16 + WPS_DH_LEN + WPS_DH_LEN];
	uint8_t hash[32];

	memcpy(hash_in, snonce, 16);
	memcpy(hash_in + 16, psk, 16);
	memcpy(hash_in + 32, w->pk_e, WPS_DH_LEN);
	memcpy(hash_in + 32 + WPS_DH_LEN, w->pk_r, WPS_DH_LEN);
	if (wps_hmac_sha256(w->authkey, 32, hash_in, sizeof(hash_in), hash) !=
	    0) {
		return -1;
	}
	return memcmp(hash, expected, 32) == 0 ? 0 : -1;
}

static int wps_process_m3(struct wps_reg *w, const uint8_t *msg, size_t len)
{
	struct wps_attr_view v;

	wps_parse_msg(msg, len, &v);
	if (v.msg_type != WPS_MSG_M3 || v.e_hash1 == NULL || v.e_hash2 == NULL ||
	    v.authenticator == NULL) {
		return -1;
	}
	if (wps_verify_authenticator(w, msg, len, v.authenticator) != 0) {
		return -1;
	}
	memcpy(w->e_hash1, v.e_hash1, 32);
	memcpy(w->e_hash2, v.e_hash2, 32);
	wps_save_last(w, msg, len);
	w->pending_op = WPS_OP_M4;
	k_work_submit(&w->work);
	return 0;
}

static int wps_process_m5(struct wps_reg *w, const uint8_t *msg, size_t len)
{
	struct wps_attr_view v;
	uint8_t plain[128];
	size_t plain_len = sizeof(plain);
	const uint8_t *s1;
	uint16_t l;

	wps_parse_msg(msg, len, &v);
	if (v.msg_type != WPS_MSG_M5 || v.encr_settings == NULL ||
	    v.authenticator == NULL) {
		return -1;
	}
	if (wps_verify_authenticator(w, msg, len, v.authenticator) != 0) {
		return -1;
	}
	if (wps_aes_128_cbc_decrypt(w->keywrapkey, v.encr_settings,
				    v.encr_settings_len, plain, &plain_len) !=
	    0) {
		LOG_ERR("WPS: M5 decrypt failed");
		return -1;
	}
	s1 = wps_attr_get(plain, plain_len, WPS_ATTR_E_SNONCE1, &l);
	if (s1 == NULL || l != 16) {
		return -1;
	}
	if (wps_check_e_hash(w, s1, w->psk1, w->e_hash1) != 0) {
		LOG_ERR("WPS: E-Hash1 mismatch");
		return -1;
	}
	wps_save_last(w, msg, len);
	w->pending_op = WPS_OP_M6;
	k_work_submit(&w->work);
	return 0;
}

static int wps_process_m7(struct wps_reg *w, const uint8_t *msg, size_t len)
{
	struct wps_attr_view v;
	uint8_t plain[256];
	size_t plain_len = sizeof(plain);
	const uint8_t *s2;
	uint16_t l;

	wps_parse_msg(msg, len, &v);
	if (v.msg_type != WPS_MSG_M7 || v.encr_settings == NULL ||
	    v.authenticator == NULL) {
		return -1;
	}
	if (wps_verify_authenticator(w, msg, len, v.authenticator) != 0) {
		return -1;
	}
	if (wps_aes_128_cbc_decrypt(w->keywrapkey, v.encr_settings,
				    v.encr_settings_len, plain, &plain_len) !=
	    0) {
		LOG_ERR("WPS: M7 decrypt failed");
		return -1;
	}
	s2 = wps_attr_get(plain, plain_len, WPS_ATTR_E_SNONCE2, &l);
	if (s2 == NULL || l != 16) {
		return -1;
	}
	if (wps_check_e_hash(w, s2, w->psk2, w->e_hash2) != 0) {
		LOG_ERR("WPS: E-Hash2 mismatch");
		return -1;
	}
	wps_save_last(w, msg, len);
	w->pending_op = WPS_OP_M8;
	k_work_submit(&w->work);
	return 0;
}

static void wps_work_handler(struct k_work *work)
{
	struct wps_reg *w = CONTAINER_OF(work, struct wps_reg, work);
	uint8_t msg[WPS_MSG_MAX];
	size_t msg_len = 0;
	int err = 0;

	switch (w->pending_op) {
	case WPS_OP_ID_REQ:
		err = wps_send_identity_req(w);
		if (!err) {
			w->state = WPS_ST_WAIT_ID;
		}
		break;
	case WPS_OP_WSC_START:
		err = wps_send_wsc_start(w);
		if (!err) {
			w->state = WPS_ST_WAIT_M1;
			w->last_msg_len = 0;
		}
		break;
	case WPS_OP_M2:
		err = wps_build_m2(w, msg, &msg_len);
		if (!err) {
			w->eap_id++;
			err = wps_send_wsc(w, WSC_OP_MSG, msg, msg_len);
			if (!err) {
				wps_save_last(w, msg, msg_len);
				w->state = WPS_ST_WAIT_M3;
			}
		}
		break;
	case WPS_OP_M4:
		err = wps_build_m4(w, msg, &msg_len);
		if (!err) {
			w->eap_id++;
			err = wps_send_wsc(w, WSC_OP_MSG, msg, msg_len);
			if (!err) {
				wps_save_last(w, msg, msg_len);
				w->state = WPS_ST_WAIT_M5;
			}
		}
		break;
	case WPS_OP_M6:
		err = wps_build_m6(w, msg, &msg_len);
		if (!err) {
			w->eap_id++;
			err = wps_send_wsc(w, WSC_OP_MSG, msg, msg_len);
			if (!err) {
				wps_save_last(w, msg, msg_len);
				w->state = WPS_ST_WAIT_M7;
			}
		}
		break;
	case WPS_OP_M8:
		err = wps_build_m8(w, msg, &msg_len);
		if (!err) {
			w->eap_id++;
			err = wps_send_wsc(w, WSC_OP_MSG, msg, msg_len);
			if (!err) {
				wps_save_last(w, msg, msg_len);
				w->state = WPS_ST_WAIT_DONE;
			}
		}
		break;
	case WPS_OP_EAP_FAIL:
		w->eap_id++;
		{
			uint8_t eapol[8];

			eapol[0] = 1;
			eapol[1] = 0;
			eapol[2] = 0;
			eapol[3] = 4;
			eapol[4] = EAP_CODE_FAILURE;
			eapol[5] = w->eap_id;
			eapol[6] = 0;
			eapol[7] = 4;
			err = wps_send_eth(w, eapol, 8);
		}
		w->state = WPS_ST_DONE;
		/* Stop registrar + clear Selected Registrar before PSK rejoin */
		airoc_p2p_wps_pbc_complete();
		break;
	default:
		break;
	}

	if (err) {
		LOG_ERR("WPS: TX op %u failed (%d)", w->pending_op, err);
	}
	w->pending_op = 0;
}

int airoc_wps_reg_arm(const uint8_t go_mac[6], const char *ssid, const char *psk)
{
	if (go_mac == NULL || ssid == NULL || psk == NULL) {
		return -EINVAL;
	}

	/* Re-arming on a repeated PD request must not orphan the previous DH
	 * context; the memset below would drop its heap pointers.
	 */
	mbedtls_dhm_free(&g_wps.dhm);

	memset(&g_wps, 0, sizeof(g_wps));
	mbedtls_dhm_init(&g_wps.dhm);
	k_work_init(&g_wps.work, wps_work_handler);

	memcpy(g_wps.go_mac, go_mac, 6);
	strncpy(g_wps.ssid, ssid, sizeof(g_wps.ssid) - 1);
	strncpy(g_wps.psk, psk, sizeof(g_wps.psk) - 1);
	g_wps.armed = true;
	g_wps.state = WPS_ST_ARMED;
	g_wps.eap_id = 0;

	LOG_INF("WPS registrar armed (PBC)");
	return 0;
}

void airoc_wps_reg_disarm(void)
{
	g_wps.armed = false;
	g_wps.state = WPS_ST_IDLE;
	mbedtls_dhm_free(&g_wps.dhm);
}

void airoc_wps_reg_on_assoc(const uint8_t peer_mac[6])
{
	if (!g_wps.armed || peer_mac == NULL) {
		return;
	}
	/* Only the PBC join path starts EAP; PSK rejoin must not. */
	if (g_wps.state != WPS_ST_ARMED) {
		return;
	}

	memcpy(g_wps.peer, peer_mac, 6);
	g_wps.pending_op = WPS_OP_ID_REQ;
	k_work_submit(&g_wps.work);
}

bool airoc_wps_reg_eapol_rx(const uint8_t *frame, size_t len)
{
	uint16_t ethertype;
	uint8_t eapol_type;
	const uint8_t *eap;
	uint16_t eap_len;
	uint8_t code, id, type;

	if (!g_wps.armed || frame == NULL || len < 18) {
		return false;
	}

	ethertype = (uint16_t)(((uint16_t)frame[12] << 8) | frame[13]);
	if (ethertype != 0x888e) {
		return false;
	}

	/* Learn peer from SA if needed */
	if (g_wps.state == WPS_ST_ARMED) {
		memcpy(g_wps.peer, frame + 6, 6);
	}

	eapol_type = frame[15];

	/* EAPOL-Start — only when waiting for a WPS enrollee */
	if (eapol_type == 1) {
		if (g_wps.state == WPS_ST_ARMED || g_wps.state == WPS_ST_WAIT_ID) {
			memcpy(g_wps.peer, frame + 6, 6);
			g_wps.pending_op = WPS_OP_ID_REQ;
			k_work_submit(&g_wps.work);
			return true;
		}
		return false;
	}

	/* Do not swallow EAPOL-Key (4-way HS) — only EAP packets for WSC */
	if (eapol_type != 0 || len < 22) {
		return false;
	}

	eap = frame + 18;
	eap_len = (uint16_t)(((uint16_t)frame[16] << 8) | frame[17]);
	if (eap_len + 18 > len) {
		eap_len = (uint16_t)(len - 18);
	}
	code = eap[0];
	id = eap[1];
	ARG_UNUSED(id);
	if (eap_len < 5) {
		return true;
	}
	type = eap[4];

	if (code == EAP_CODE_RESPONSE && type == EAP_TYPE_IDENTITY) {
		g_wps.pending_op = WPS_OP_WSC_START;
		k_work_submit(&g_wps.work);
		return true;
	}

	if (code == EAP_CODE_RESPONSE && type == EAP_TYPE_EXPANDED &&
	    eap_len >= 12) {
		const uint8_t *wfa = eap + 5;
		uint8_t opcode;
		const uint8_t *wsc;
		size_t wsc_len;

		/* vendor OUI 00:37:2a, type 00000001 */
		if (wfa[0] != 0x00 || wfa[1] != 0x37 || wfa[2] != 0x2a) {
			return true;
		}
		opcode = wfa[7];
		wsc = wfa + 9;
		wsc_len = eap_len - 5 - 9;
		if (eap_len < 5 + 9) {
			wsc_len = 0;
		}

		if (opcode == WSC_OP_MSG && wsc_len > 0) {
			struct wps_attr_view v;

			if (wsc_len > sizeof(g_wps.rx_wsc)) {
				LOG_ERR("WPS: MSG too long (%u)",
					(unsigned)wsc_len);
				return true;
			}
			memcpy(g_wps.rx_wsc, wsc, wsc_len);
			g_wps.rx_wsc_len = wsc_len;
			wps_parse_msg(wsc, wsc_len, &v);

			if (v.msg_type == WPS_MSG_M1) {
				(void)wps_process_m1(&g_wps, wsc, wsc_len);
			} else if (v.msg_type == WPS_MSG_M3) {
				(void)wps_process_m3(&g_wps, wsc, wsc_len);
			} else if (v.msg_type == WPS_MSG_M5) {
				(void)wps_process_m5(&g_wps, wsc, wsc_len);
			} else if (v.msg_type == WPS_MSG_M7) {
				(void)wps_process_m7(&g_wps, wsc, wsc_len);
			}
		} else if (opcode == WSC_OP_DONE) {
			LOG_INF("WPS: WSC_Done");
			g_wps.pending_op = WPS_OP_EAP_FAIL;
			k_work_submit(&g_wps.work);
		} else if (opcode == WSC_OP_NACK) {
			struct wps_attr_view v;
			uint16_t cfg_err = 0xffff;
			const uint8_t *a;
			uint16_t alen;

			wps_parse_msg(wsc, wsc_len, &v);
			a = wps_attr_get(wsc, wsc_len, WPS_ATTR_CONFIG_ERROR,
					 &alen);
			if (a && alen == 2) {
				cfg_err = wps_get_be16(a);
			}
			LOG_WRN("WPS: WSC_NACK msg=0x%02x config_error=%u",
				v.msg_type, cfg_err);
			g_wps.state = WPS_ST_ARMED;
		}
		return true;
	}

	return true;
}
