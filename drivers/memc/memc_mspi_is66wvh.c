/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ISSI IS66/67WVH HyperRAM support over MSPI.
 *
 * These devices power up with usable default configuration registers. The
 * controller generates the HyperBus command/address phase for accesses to the
 * memory-mapped window, so no serial initialization commands are required.
 */

#define DT_DRV_COMPAT issi_is66wvh

#include <zephyr/device.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(memc_mspi_is66wvh, CONFIG_MEMC_LOG_LEVEL);

struct memc_mspi_is66wvh_config {
	const struct device *bus;
	struct mspi_dev_id dev_id;
	struct mspi_dev_cfg dev_cfg;
	struct mspi_xip_cfg xip_cfg;
	uintptr_t xip_base;
	uint32_t mem_size;
};

struct memc_mspi_is66wvh_data {
	void *mem_base;
};

static int memc_mspi_is66wvh_init(const struct device *dev)
{
	const struct memc_mspi_is66wvh_config *cfg = dev->config;
	struct memc_mspi_is66wvh_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->bus)) {
		LOG_ERR("MSPI controller is not ready");
		return -ENODEV;
	}

	if (cfg->dev_cfg.io_mode != MSPI_IO_MODE_OCTAL ||
	    cfg->dev_cfg.data_rate != MSPI_DATA_RATE_DUAL ||
	    !cfg->dev_cfg.dqs_enable) {
		LOG_ERR("HyperRAM requires Octal DDR mode with RWDS enabled");
		return -EINVAL;
	}

	if (!cfg->xip_cfg.enable || cfg->xip_cfg.permission != MSPI_XIP_READ_WRITE) {
		LOG_ERR("HyperRAM requires a read/write memory-mapped window");
		return -EINVAL;
	}

	ret = mspi_dev_config(cfg->bus, &cfg->dev_id, MSPI_DEVICE_CONFIG_ALL,
			      &cfg->dev_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure the MSPI HyperRAM target: %d", ret);
		return ret;
	}

	ret = mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->xip_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to enable the HyperRAM mapped window: %d", ret);
		return ret;
	}

	data->mem_base = (void *)(cfg->xip_base + cfg->xip_cfg.address_offset);
	LOG_INF("HyperRAM mapped at %p (%u bytes)", data->mem_base, cfg->mem_size);

	return 0;
}

#define MEMC_MSPI_IS66WVH(inst)								\
	BUILD_ASSERT(IS_ENABLED(CONFIG_MSPI_XIP),					\
		     "IS66WVH requires CONFIG_MSPI_XIP");				\
	static const struct memc_mspi_is66wvh_config memc_mspi_is66wvh_config_##inst = {	\
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),					\
		.dev_id = MSPI_DEVICE_ID_DT_INST(inst),					\
		.dev_cfg = MSPI_DEVICE_CONFIG_DT_INST(inst),				\
		.xip_cfg = MSPI_XIP_CONFIG_DT_INST(inst),				\
		.xip_base = DT_REG_ADDR_BY_IDX(DT_INST_BUS(inst), 1),			\
		.mem_size = DT_INST_PROP(inst, size),					\
	};											\
	static struct memc_mspi_is66wvh_data memc_mspi_is66wvh_data_##inst;		\
	DEVICE_DT_INST_DEFINE(inst, memc_mspi_is66wvh_init, NULL,			\
			      &memc_mspi_is66wvh_data_##inst,				\
			      &memc_mspi_is66wvh_config_##inst, POST_KERNEL,		\
			      CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_MSPI_IS66WVH)
