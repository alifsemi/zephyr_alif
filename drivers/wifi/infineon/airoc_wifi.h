/*
 * Copyright (c) 2023 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <whd_buffer_api.h>
#include <whd_chip_constants.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/wifi_mgmt.h>

#ifdef CONFIG_AIROC_WIFI_BUS_SDIO
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sdio.h>
#endif
#ifdef CONFIG_AIROC_WIFI_BUS_SPI
#include <zephyr/drivers/spi.h>
#endif

#include <cy_utils.h>

#define DT_DRV_COMPAT infineon_airoc_wifi

#if DT_PROP(DT_DRV_INST(0), spi_data_irq_shared)
#define SPI_DATA_IRQ_SHARED
#include <zephyr/drivers/pinctrl.h>

#define PINCTRL_STATE_HOST_WAKE PINCTRL_STATE_PRIV_START
#endif

#if defined(CONFIG_AIROC_WIFI_BUS_SPI)
#define AIROC_WIFI_SPI_OPERATION (SPI_WORD_SET(DT_PROP_OR(DT_DRV_INST(0), spi_word_size, 8)) \
			| (DT_PROP(DT_DRV_INST(0), spi_half_duplex) \
				? SPI_HALF_DUPLEX : SPI_FULL_DUPLEX) \
			| SPI_TRANSFER_MSB)
#endif

struct airoc_wifi_data {
#if defined(CONFIG_AIROC_WIFI_BUS_SDIO)
	struct sd_card card;
	struct sdio_func sdio_func1;
	struct sdio_func sdio_func2;
#endif
#if defined(SPI_DATA_IRQ_SHARED)
	uint8_t prev_irq_state;
#endif
	struct net_if *iface;
	bool second_interface_init;
	bool is_ap_up;
	bool is_sta_connected;
#if defined(CONFIG_AIROC_WIFI_P2P)
	bool p2p_initialized;
#endif
	uint8_t mac_addr[6];
	scan_result_cb_t scan_rslt_cb;
	whd_ssid_t ssid;
	whd_scan_result_t scan_result;
	struct k_sem sema_common;
	struct k_sem sema_scan;
#if defined(CONFIG_NET_STATISTICS_WIFI)
	struct net_stats_wifi stats;
#endif
	whd_driver_t whd_drv;
	struct gpio_callback host_oob_pin_cb;
	uint8_t frame_buf[NET_ETH_MAX_FRAME_SIZE];
};

union airoc_wifi_bus {
#if defined(CONFIG_AIROC_WIFI_BUS_SDIO)
	const struct device *bus_sdio;
#endif
#if defined(CONFIG_AIROC_WIFI_BUS_SPI)
	const struct spi_dt_spec bus_spi;
#endif
};

struct airoc_wifi_config {
	const union airoc_wifi_bus bus_dev;
	struct gpio_dt_spec wifi_reg_on_gpio;
	struct gpio_dt_spec wifi_host_wake_gpio;
	struct gpio_dt_spec wifi_dev_wake_gpio;
#if defined(CONFIG_AIROC_WIFI_BUS_SPI)
	struct gpio_dt_spec bus_select_gpio;
#if defined(SPI_DATA_IRQ_SHARED)
	const struct pinctrl_dev_config *pcfg;
#endif
#endif
};

/**
 * \brief This function returns pointer type to handle instance
 *        of whd interface (whd_interface_t) which allocated in
 *        Zephyr AIROC driver (drivers/wifi/infineon/airoc_wifi.c)
 */

whd_interface_t airoc_wifi_get_whd_interface(void);

#if defined(CONFIG_AIROC_WIFI_P2P) && !defined(CONFIG_AIROC_WIFI6)
/*
 * Declared in WHD whd_int.h. Prototype lives here so airoc_wifi.c need not
 * include that private header (it conflicts with public whd_events.h).
 */
whd_result_t whd_add_interface(whd_driver_t whd_driver, uint8_t bsscfgidx,
			       uint8_t ifidx, const char *name,
			       whd_mac_t *mac_addr, whd_interface_t *ifpp);
#endif

/** Transmit a raw Ethernet frame on the active WHD interface (SoftAP/GO). */
int airoc_wifi_send_raw_eth(const uint8_t *frame, size_t len);

#if defined(CONFIG_AIROC_WIFI_P2P)
/**
 * SoftAP-style P2P GO bring-up.
 * @param p2p_if_mac  address of the GO bsscfg created by p2p_ifadd → GO runs on
 *                    tertiary bsscfg2 and p2p_disc keeps bsscfg1. NULL when
 *                    p2p_ifadd was unavailable → secondary bsscfg1 SoftAP.
 */
int airoc_wifi_go_softap_start(const char *ssid_str, size_t ssid_len,
			       const char *psk, size_t psk_len,
			       uint8_t channel, const whd_mac_t *p2p_if_mac,
			       whd_interface_t *out_if);

/** Tear down softAP-style P2P GO started by airoc_wifi_go_softap_start(). */
int airoc_wifi_go_softap_stop(void);
#endif
