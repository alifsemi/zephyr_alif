/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <soc.h>

#define LOG_MODULE_NAME alif_mdio
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define MAC_MDIO_DATA				0x0204
#define MAC_MDIO_ADDRESS			0x0200

#define MAC_MDIO_ADDR_PA_SHIFT			21
#define MAC_MDIO_ADDR_RDA_SHIFT			16
#define MAC_MDIO_ADDR_CR_SHIFT			8
#define MAC_MDIO_ADDR_GOC_SHIFT			2
#define MAC_MDIO_ADDR_GOC_READ			3
#define MAC_MDIO_ADDR_GOC_WRITE			1
#define MAC_MDIO_ADDR_GB			BIT(0)

/* MDC clock divisors for different CSR clk values */
#define MAC_MDIO_ADDR_CR_60_100			0
#define MAC_MDIO_ADDR_CR_100_150		1
#define MAC_MDIO_ADDR_CR_20_35			2
#define MAC_MDIO_ADDR_CR_35_60			3
#define MAC_MDIO_ADDR_CR_150_250		4
#define MAC_MDIO_ADDR_CR_250_300		5
#define MAC_MDIO_ADDR_CR_300_500		6
#define MAC_MDIO_ADDR_CR_500_800		7

struct alif_mdio_config {
	DEVICE_MMIO_ROM;
	uint32_t base;
};

struct alif_mdio_data {
	DEVICE_MMIO_RAM;
};

static void alif_mdio_bus_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
}

/* MDIO Read API implementation */
static int alif_mdio_read(const struct device *dev, uint8_t phy_addr,
			uint8_t reg_addr, uint16_t *read_data)
{
	const struct alif_mdio_config *config = dev->config;
	uint32_t read;
	uint32_t val, timeout = 5;

	val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
			(reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
			(MAC_MDIO_ADDR_GOC_READ << MAC_MDIO_ADDR_GOC_SHIFT) |
			(MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) |
			MAC_MDIO_ADDR_GB;

	sys_write32(val, config->base + MAC_MDIO_ADDRESS);

	do {
		read = sys_read32(config->base + MAC_MDIO_ADDRESS);
		if (!(read &  MAC_MDIO_ADDR_GB)) {
			*read_data = sys_read32(config->base + MAC_MDIO_DATA);
			break;
		}

		k_msleep(500);
		timeout--;
	} while (timeout);

	if (!timeout) {
		LOG_ERR("Error timeout\n");
		return -1;
	}

	return 0;
}

/* MDIO Write API implementation */
static int alif_mdio_write(const struct device *dev, uint8_t phy_addr,
			uint8_t reg_addr, uint16_t write_data)
{
	const struct alif_mdio_config *config = dev->config;
	uint32_t read;

	uint32_t val, timeout = 5;

	val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
			(reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
			(MAC_MDIO_ADDR_GOC_WRITE << MAC_MDIO_ADDR_GOC_SHIFT) |
			(MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) |
			MAC_MDIO_ADDR_GB;

	sys_write32(write_data, config->base + MAC_MDIO_DATA);
	sys_write32(val, config->base + MAC_MDIO_ADDRESS);

	do {
		read = sys_read32(config->base + MAC_MDIO_ADDRESS);
		if (!(read &  MAC_MDIO_ADDR_GB))
			break;
		k_msleep(500);
		timeout--;
	} while (timeout);

	if (!timeout) {
		LOG_ERR("Error timeout\n");
		return -1;
	}

	return 0;
}

static int alif_mdio_init(const struct device *dev)
{
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	return 0;
}

static const struct mdio_driver_api alif_mdio_api = {
	.read = alif_mdio_read,
	.write = alif_mdio_write,
	.bus_enable = alif_mdio_bus_enable,
};

#define ALIF_MDIO_INIT(inst)							\
	static const struct alif_mdio_config alif_mdio_cfg_##inst = {		\
		.base = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(inst))),		\
	};									\
	static struct alif_mdio_data alif_mdio_data_##inst;			\
										\
	DEVICE_DT_INST_DEFINE(inst, &alif_mdio_init, NULL,			\
			      &alif_mdio_data_##inst, &alif_mdio_cfg_##inst,	\
			      POST_KERNEL, CONFIG_ETH_INIT_PRIORITY,		\
			      &alif_mdio_api);
DT_INST_FOREACH_STATUS_OKAY(ALIF_MDIO_INIT)
