/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Infineon S80KS HyperRAM support over MSPI. */

#define DT_DRV_COMPAT infineon_s80ks

#include <zephyr/device.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/mspi/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(memc_mspi_s80ks, CONFIG_MEMC_LOG_LEVEL);

/* HyperBus 48-bit command/address for Configuration Register 0 write. */
#define S80KS_CR0_WRITE_CMD              0x6000U
#define S80KS_CR0_WRITE_ADDR             0x01000000U

#define S80KS_CR0_BURST_LENGTH_MASK      GENMASK(1, 0)
#define S80KS_CR0_WRAPPED_BURST_SEQ      BIT(2)
#define S80KS_CR0_FIXED_LATENCY          BIT(3)
#define S80KS_CR0_INITIAL_LATENCY_MASK   GENMASK(7, 4)
#define S80KS_CR0_RESERVED_MASK          GENMASK(11, 8)
#define S80KS_CR0_DRIVE_STRENGTH_MASK    GENMASK(14, 12)
#define S80KS_CR0_NORMAL_OPERATION       BIT(15)

#define S80KS_TRANSFER_TIMEOUT_MS        10U

struct memc_mspi_s80ks_config {
	const struct device *bus;
	struct mspi_dev_id dev_id;
	struct mspi_dev_cfg dev_cfg;
	struct mspi_xip_cfg xip_cfg;
	uintptr_t xip_base;
	uint32_t mem_size;
	uint8_t initial_latency;
	uint8_t burst_length;
	uint8_t drive_strength;
	bool fixed_latency;
	bool hybrid_burst;
};

struct memc_mspi_s80ks_data {
	void *mem_base;
};

static int s80ks_write_cr0(const struct device *dev)
{
	const struct memc_mspi_s80ks_config *cfg = dev->config;
	uint8_t burst_length_code;
	uint8_t drive_strength_code;

	/* Use a 32-byte wrap size for RTSS-HP and RTSS-HE.
	 * Reduce wait cycles to improve performance.
	 */
	/*
	 * CA bit assignment for Configuration Register 0 write:
	 * bit[47:40] = 0x60
	 * bit[39:32] = 0x00
	 * bit[31:24] = 0x01
	 * bit[23:16] = 0x00
	 * bit[15:8]  = 0x00
	 * bit[7:0]   = 0x00
	 */

	switch (cfg->burst_length) {
	case 16:
		burst_length_code = 2U;
		break;
	case 32:
		burst_length_code = 3U;
		break;
	case 64:
		burst_length_code = 1U;
		break;
	default:
		burst_length_code = 0U;
		break;
	}

	switch (cfg->drive_strength) {
	case 115:
		drive_strength_code = 1U;
		break;
	case 67:
		drive_strength_code = 2U;
		break;
	case 46:
		drive_strength_code = 3U;
		break;
	case 27:
		drive_strength_code = 5U;
		break;
	case 22:
		drive_strength_code = 6U;
		break;
	case 19:
		drive_strength_code = 7U;
		break;
	default:
		drive_strength_code = 0U;
		break;
	}

	uint16_t cr0 = S80KS_CR0_NORMAL_OPERATION |
		S80KS_CR0_RESERVED_MASK |
		FIELD_PREP(S80KS_CR0_INITIAL_LATENCY_MASK,
			   (cfg->initial_latency - 5U) & 0xFU) |
		FIELD_PREP(S80KS_CR0_DRIVE_STRENGTH_MASK, drive_strength_code) |
		FIELD_PREP(S80KS_CR0_BURST_LENGTH_MASK, burst_length_code);

	if (cfg->fixed_latency) {
		cr0 |= S80KS_CR0_FIXED_LATENCY;
	}

	if (!cfg->hybrid_burst) {
		cr0 |= S80KS_CR0_WRAPPED_BURST_SEQ;
	}

	uint16_t cr0_be = sys_cpu_to_be16(cr0);
	struct mspi_xfer_packet packet = {
		.dir = MSPI_TX,
		.cmd = S80KS_CR0_WRITE_CMD,
		.address = S80KS_CR0_WRITE_ADDR,
		.num_bytes = sizeof(cr0),
		.data_buf = (uint8_t *)&cr0_be,
	};
	const struct mspi_xfer xfer = {
		.async = false,
		.xfer_mode = MSPI_PIO,
		.tx_dummy = 0,
		.rx_dummy = 0,
		.cmd_length = 2,
		.addr_length = 4,
		.hold_ce = false,
		.packets = &packet,
		.num_packet = 1,
		.timeout = S80KS_TRANSFER_TIMEOUT_MS,
	};

	return mspi_transceive(cfg->bus, &cfg->dev_id, &xfer);
}

static int memc_mspi_s80ks_init(const struct device *dev)
{
	const struct memc_mspi_s80ks_config *cfg = dev->config;
	struct memc_mspi_s80ks_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->bus)) {
		LOG_ERR("MSPI controller is not ready");
		return -ENODEV;
	}

	if ((cfg->dev_cfg.io_mode != MSPI_IO_MODE_OCTAL &&
	     cfg->dev_cfg.io_mode != MSPI_IO_MODE_HEX_8_8_16) ||
	    cfg->dev_cfg.data_rate != MSPI_DATA_RATE_DUAL ||
	    !cfg->dev_cfg.dqs_enable) {
		LOG_ERR("HyperRAM requires Octal/HyperBus DDR mode with RWDS enabled");
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

	ret = s80ks_write_cr0(dev);
	if (ret != 0) {
		LOG_ERR("Failed to program S80KS CR0: %d", ret);
		return ret;
	}

	ret = mspi_xip_config(cfg->bus, &cfg->dev_id, &cfg->xip_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to enable the HyperRAM mapped window: %d", ret);
		return ret;
	}

	data->mem_base = (void *)(cfg->xip_base + cfg->xip_cfg.address_offset);
	LOG_INF("S80KS HyperRAM mapped at %p (%u bytes), CR0 latency %u",
		data->mem_base, cfg->mem_size, cfg->initial_latency);

	return 0;
}

#define MEMC_MSPI_S80KS(inst)                                                        \
	BUILD_ASSERT(IS_ENABLED(CONFIG_MSPI_XIP), "S80KS requires CONFIG_MSPI_XIP"); \
	static const struct memc_mspi_s80ks_config memc_mspi_s80ks_config_##inst = {   \
		.bus = DEVICE_DT_GET(DT_INST_BUS(inst)),                                \
		.dev_id = MSPI_DEVICE_ID_DT_INST(inst),                                 \
		.dev_cfg = MSPI_DEVICE_CONFIG_DT_INST(inst),                             \
		.xip_cfg = MSPI_XIP_CONFIG_DT_INST(inst),                               \
		.xip_base = DT_REG_ADDR_BY_IDX(DT_INST_BUS(inst), 1),                    \
		.mem_size = DT_INST_PROP(inst, size),                                    \
		.initial_latency = DT_INST_PROP(inst, initial_latency),                   \
		.burst_length = DT_INST_PROP(inst, burst_length),                         \
		.drive_strength = DT_INST_PROP(inst, drive_strength_ohms),                \
		.fixed_latency = DT_INST_PROP(inst, fixed_latency),                       \
		.hybrid_burst = DT_INST_PROP(inst, hybrid_burst),                         \
	};                                                                             \
	static struct memc_mspi_s80ks_data memc_mspi_s80ks_data_##inst;                \
	DEVICE_DT_INST_DEFINE(inst, memc_mspi_s80ks_init, NULL,                         \
			      &memc_mspi_s80ks_data_##inst,                              \
			      &memc_mspi_s80ks_config_##inst, POST_KERNEL,                \
			      CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_MSPI_S80KS)
