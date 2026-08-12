/*
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8201fr

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME phy_mc_rtl8201fr
#define LOG_LEVEL CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DEV_DATA(dev) ((struct phy_rtl8201fr_data *)((dev)->data))
#define DEV_CFG(dev)  ((const struct phy_rtl8201fr_config *)((dev)->config))

struct phy_rtl8201fr_config {
	uint8_t addr;
	const struct device *mdio_dev;
	uint8_t speed;
	uint8_t mode;
};

struct phy_rtl8201fr_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
};

#define ETH_PHY_AUTO_NEGOTIATE		0x0
#define ETH_PHY_FULL_DUPLEX		0x1
#define ETH_PHY_HALF_DUPLEX		0x2

#define ETH_PHY_SPEED_10M		0x0
#define ETH_PHY_SPEED_100M		0x1

static int phy_rtl8201fr_read(const struct device *dev,
				uint16_t reg_addr, uint32_t *data)
{
	const struct phy_rtl8201fr_config *config = DEV_CFG(dev);
	int ret;

	ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_rtl8201fr_write(const struct device *dev,
				uint16_t reg_addr, uint32_t data)
{
	const struct phy_rtl8201fr_config *config = DEV_CFG(dev);
	int ret;

	ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_rtl8201fr_get_link(const struct device *dev,
					struct phy_link_state *state)
{
	const struct phy_rtl8201fr_config *config = DEV_CFG(dev);
	struct phy_rtl8201fr_data *data = DEV_DATA(dev);
	uint32_t bmsr = 0;
	uint32_t anar = 0;
	uint32_t anlpar = 0;
	int ret;

	struct phy_link_state old_state = data->state;

	/* Read link state */
	ret = phy_rtl8201fr_read(dev, MII_BMSR, &bmsr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) link partner register", config->addr);
		return ret;
	}

	state->is_up = bmsr & MII_BMSR_LINK_STATUS;

	if (!state->is_up) {
		goto result;
	}

	/* Read currently configured advertising options */
	ret = phy_rtl8201fr_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		return ret;
	}

	/* Read link partner capability */
	ret = phy_rtl8201fr_read(dev, MII_ANLPAR, &anlpar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) link partner register", config->addr);
		return ret;
	}

	uint32_t mutual_capabilities = anar & anlpar;

	if (mutual_capabilities & MII_ADVERTISE_100_FULL) {
		state->speed = LINK_FULL_100BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_100_HALF) {
		state->speed = LINK_HALF_100BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_10_FULL) {
		state->speed = LINK_FULL_10BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_10_HALF) {
		state->speed = LINK_HALF_10BASE_T;
	} else {
		ret = -EIO;
	}

result:

	if (memcmp(&old_state, state, sizeof(struct phy_link_state)) != 0) {
		LOG_DBG("PHY %d is %s", config->addr, state->is_up ? "up" : "down");
		LOG_DBG("PHY (%d) Link speed %s Mb, %s duplex\n", config->addr,
			(PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
			PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
	}

	return ret;
}

static int phy_rtl8201fr_cfg_link(const struct device *dev,
					enum phy_link_speed speeds)
{
	const struct phy_rtl8201fr_config *config = DEV_CFG(dev);
	struct phy_rtl8201fr_data *data = DEV_DATA(dev);
	int ret;
	uint32_t anar;
	uint32_t bmcr;

	/* Read ANAR register to write back */
	ret = phy_rtl8201fr_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
	}

	/* Setup advertising register */
	if (speeds & LINK_FULL_100BASE_T) {
		anar |= MII_ADVERTISE_100_FULL;
	} else {
		anar &= ~MII_ADVERTISE_100_FULL;
	}
	if (speeds & LINK_HALF_100BASE_T) {
		anar |= MII_ADVERTISE_100_HALF;
	} else {
		anar &= ~MII_ADVERTISE_100_HALF;
	}
	if (speeds & LINK_FULL_10BASE_T) {
		anar |= MII_ADVERTISE_10_FULL;
	} else {
		anar &= ~MII_ADVERTISE_10_FULL;
	}
	if (speeds & LINK_HALF_10BASE_T) {
		anar |= MII_ADVERTISE_10_HALF;
	} else {
		anar &= ~MII_ADVERTISE_10_HALF;
	}

	/* Write capabilities to advertising register */
	ret = phy_rtl8201fr_write(dev, MII_ANAR, anar);
	if (ret) {
		LOG_ERR("Error writing phy (%d) advertising register", config->addr);
	}

	/* Read control register */
	ret = phy_rtl8201fr_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	if (config->mode == ETH_PHY_AUTO_NEGOTIATE) {
		bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
	}

	if (config->mode == ETH_PHY_FULL_DUPLEX) {
		bmcr |= MII_BMCR_DUPLEX_MODE;
	}

	if (config->mode == ETH_PHY_HALF_DUPLEX) {
		bmcr &= ~MII_BMCR_DUPLEX_MODE;
	}

	if (config->speed == ETH_PHY_SPEED_100M) {
		bmcr |= MII_BMCR_SPEED_100;
	}

	if (config->speed == ETH_PHY_SPEED_10M) {
		bmcr |= MII_BMCR_SPEED_10;
	}

	ret = phy_rtl8201fr_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	/* Get link status */
	ret = phy_rtl8201fr_get_link(dev, &data->state);

	return ret;
}

static int phy_rtl8201fr_link_cb_set(const struct device *dev,
					phy_callback_t cb, void *user_data)
{
	struct phy_rtl8201fr_data *data = DEV_DATA(dev);

	data->cb = cb;
	data->cb_data = user_data;

	phy_rtl8201fr_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

static int phy_rtl8201fr_init(const struct device *dev)
{
	const struct phy_rtl8201fr_config *config = DEV_CFG(dev);
	uint32_t ret = 0;
	uint32_t val;
	uint16_t bmcr = 0;

	ret = phy_rtl8201fr_read(dev, MII_PHYID1R, &val);
	if (ret) {
		LOG_ERR("Error reading PHY ID 1 Register");
		return -EIO;
	}

	if (val == 0x0 || val == 0xFFFF) {
		/* A valid PHY ID cannot be all zeroes or all ones */
		LOG_ERR("invalid PHY");
		return -ENODEV;
	}

	ret = phy_rtl8201fr_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return -EIO;
	}

	/* Advertise all speeds */
	phy_rtl8201fr_cfg_link(dev, LINK_HALF_10BASE_T |
				LINK_FULL_10BASE_T |
				LINK_HALF_100BASE_T |
				LINK_FULL_100BASE_T);

	return 0;
}

static const struct ethphy_driver_api rtl8201fr_phy_api = {
	.get_link = phy_rtl8201fr_get_link,
	.cfg_link = phy_rtl8201fr_cfg_link,
	.link_cb_set = phy_rtl8201fr_link_cb_set,
	.read = phy_rtl8201fr_read,
	.write = phy_rtl8201fr_write,
};

#define REALTEK_INIT(n)								\
	static const struct phy_rtl8201fr_config mc_rtl8201fr_##n##_config = {	\
		.addr = DT_INST_REG_ADDR(n),					\
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		.speed = DT_INST_ENUM_IDX(n, phy_speed),			\
		.mode = DT_INST_ENUM_IDX(n, phy_mode),				\
	};									\
										\
	static struct phy_rtl8201fr_data mc_rtl8201fr_##n##_data;		\
										\
	DEVICE_DT_INST_DEFINE(n, &phy_rtl8201fr_init, NULL,			\
			&mc_rtl8201fr_##n##_data, &mc_rtl8201fr_##n##_config,	\
			POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,			\
			&rtl8201fr_phy_api);
DT_INST_FOREACH_STATUS_OKAY(REALTEK_INIT)
