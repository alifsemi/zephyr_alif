/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <soc.h>

#define LOG_MODULE_NAME snps_designware_mdio
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DEV_DATA(dev) ((struct mdio_dwmac_data *)((dev)->data))
#define DEV_CFG(dev)  ((const struct mdio_dwmac_config *)((dev)->config))

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

struct mdio_dwmac_config {
	DEVICE_MMIO_ROM;
};

struct mdio_dwmac_data {
	DEVICE_MMIO_RAM;
	struct k_mutex mdio_lock;
};

/* MDIO Read API implementation */
static int mdio_dwmac_read(const struct device *dev, uint8_t phy_addr,
			uint8_t reg_addr, uint16_t *read_data)
{
	struct mdio_dwmac_data *data = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val, timeout = 5;

	k_mutex_lock(&data->mdio_lock, K_FOREVER);
	val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
			(reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
			(MAC_MDIO_ADDR_GOC_READ << MAC_MDIO_ADDR_GOC_SHIFT) |
			(MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) |
			MAC_MDIO_ADDR_GB;

	sys_write32(val, reg_base + MAC_MDIO_ADDRESS);

	do {
		val = sys_read32(reg_base + MAC_MDIO_ADDRESS);
		if (!(val & MAC_MDIO_ADDR_GB)) {
			*read_data = sys_read32(reg_base + MAC_MDIO_DATA);
			break;
		}

		k_msleep(500);
		timeout--;
	} while (timeout);

	k_mutex_unlock(&data->mdio_lock);

	if (!timeout) {
		LOG_ERR("Error timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

/* MDIO Write API implementation */
static int mdio_dwmac_write(const struct device *dev, uint8_t phy_addr,
			uint8_t reg_addr, uint16_t write_data)
{
	struct mdio_dwmac_data *data = DEV_DATA(dev);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t val, timeout = 5;

	k_mutex_lock(&data->mdio_lock, K_FOREVER);
	val = (phy_addr << MAC_MDIO_ADDR_PA_SHIFT) |
			(reg_addr << MAC_MDIO_ADDR_RDA_SHIFT) |
			(MAC_MDIO_ADDR_GOC_WRITE << MAC_MDIO_ADDR_GOC_SHIFT) |
			(MAC_MDIO_ADDR_CR_150_250 << MAC_MDIO_ADDR_CR_SHIFT) |
			MAC_MDIO_ADDR_GB;

	sys_write32(write_data, reg_base + MAC_MDIO_DATA);
	sys_write32(val, reg_base + MAC_MDIO_ADDRESS);

	do {
		val = sys_read32(reg_base + MAC_MDIO_ADDRESS);
		if (!(val & MAC_MDIO_ADDR_GB)) {
			break;
		}
		k_msleep(500);
		timeout--;
	} while (timeout);

	k_mutex_unlock(&data->mdio_lock);

	if (!timeout) {
		LOG_ERR("Error timeout");
		return -ETIMEDOUT;
	}

	return 0;
}


static int mdio_dwmac_init(const struct device *dev)
{
	struct mdio_dwmac_data *data = DEV_DATA(dev);

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	k_mutex_init(&data->mdio_lock);

	return 0;
}

static const struct mdio_driver_api mdio_dwmac_api  = {
	.read = mdio_dwmac_read,
	.write = mdio_dwmac_write,
};

#define MDIO_DWMAC_INIT(inst)							  \
	static const struct mdio_dwmac_config mdio_dwmac_config_##inst = {	  \
		DEVICE_MMIO_ROM_INIT(DT_PARENT(DT_DRV_INST(inst)))		  \
	};									  \
	static struct mdio_dwmac_data mdio_dwmac_data_##inst;			  \
										  \
	DEVICE_DT_INST_DEFINE(inst, &mdio_dwmac_init, NULL,			  \
			      &mdio_dwmac_data_##inst, &mdio_dwmac_config_##inst, \
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,		  \
			      &mdio_dwmac_api);
DT_INST_FOREACH_STATUS_OKAY(MDIO_DWMAC_INIT)
