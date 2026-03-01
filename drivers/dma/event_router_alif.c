/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Alif Event Router (DMA Multiplexer) driver
 *
 * This driver implements a thin wrapper over a DMA controller,
 * managing the Alif Event Router hardware that multiplexes peripheral DMA
 * requests to the underlying DMA controller.
 *
 * Architecture:
 * - Event Router: 32 channels, each can select from 1-4 peripheral inputs
 * - Mapping: Event router channel N → DMA peripheral request N (1:1)
 *
 * Provides two APIs:
 * 1. DMA API: Standard Zephyr DMA API (delegates to DMA controller)
 * 2. Control API: alif_evtrtr_configure_channel() for non-DMA use cases
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/event_router_alif.h>
#include <zephyr/dt-bindings/dma/alif_dma.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(event_router_alif, CONFIG_EVENT_ROUTER_ALIF_LOG_LEVEL);

#define DT_DRV_COMPAT alif_event_router

/* Event Router register offsets */
#define EVTRTR_DMA_CTRL0_OFFSET		0x00
#define EVTRTR_DMA_REQ_CTRL_OFFSET	0x80
#define EVTRTR_DMA_ACK_TYPE0_OFFSET	0x90

/* Register stride/offset calculations */
#define EVTRTR_DMA_CTRL_REG_SIZE	4	/* Each DMA_CTRL register is 4 bytes */
#define EVTRTR_DMA_ACK_TYPE_REG_SIZE	4	/* Each DMA_ACK_TYPE register is 4 bytes */

/* Helper macros for register address calculation */
#define EVTRTR_DMA_CTRL_REG(base, channel) \
	((base) + EVTRTR_DMA_CTRL0_OFFSET + ((channel) * EVTRTR_DMA_CTRL_REG_SIZE))

#define EVTRTR_DMA_ACK_TYPE_REG(base, dma_group) \
	((base) + EVTRTR_DMA_ACK_TYPE0_OFFSET + ((dma_group) * EVTRTR_DMA_ACK_TYPE_REG_SIZE))

/* DMA_CTRL register bit definitions */
#define EVTRTR_DMA_CTRL_GROUP_MASK	0x3	/* Bits 1-0: dma_group (0-3) */
#define EVTRTR_DMA_CTRL_ENA		BIT(4)	/* Bit 4: Enable */

/* DMA_REQ_CTRL register bit definitions */
#define EVTRTR_DMA_REQ_CTRL_CB		BIT(12)	/* Enable peripheral Burst DMA request */
#define EVTRTR_DMA_REQ_CTRL_CS		BIT(8)	/* Enable peripheral Single DMA request */
#define EVTRTR_DMA_REQ_CTRL_CLB		BIT(4)	/* Enable peripheral Last Burst DMA request */
#define EVTRTR_DMA_REQ_CTRL_CLS		BIT(0)	/* Enable peripheral Last Single DMA request */

/**
 * @brief Event router instance configuration (read-only, in ROM)
 */
struct evtrtr_alif_config {
	DEVICE_MMIO_ROM;		/* MMIO region for register base */
	const struct device *dma_dev;	/* DMA controller device */
	uint8_t num_channels;		/* Number of channels */
	uint8_t num_groups;		/* Number of dma-groups per channel */
};

/**
 * @brief Event router runtime data (in RAM)
 */
struct evtrtr_alif_data {
	DEVICE_MMIO_RAM;		/* Mapped MMIO region */
	struct k_mutex lock;		/* Mutex for thread-safe access */
};

/**
 * @brief Configure event router channel
 *
 * @param dev Event router device
 * @param channel Event router channel
 * @param dma_group Input source selection
 * @param enable_handshake Enable hardware handshaking
 *
 * @return 0 on success, negative errno on error
 */
static int evtrtr_configure_channel(const struct device *dev, uint32_t channel,
				     uint32_t dma_group, bool enable_handshake)
{
	uint32_t ctrl, ack;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	/* Configure event router channel control register */
	ctrl = EVTRTR_DMA_CTRL_ENA |
	       (dma_group & EVTRTR_DMA_CTRL_GROUP_MASK);
	sys_write32(ctrl, EVTRTR_DMA_CTRL_REG(reg_base, channel));

	LOG_DBG("%s ch %u: DMA_CTRL = 0x%08x (group %u)",
		dev->name, channel, ctrl, dma_group);

	/* Configure DMA_ACK_TYPE register (handshaking enable/disable) */
	ack = sys_read32(EVTRTR_DMA_ACK_TYPE_REG(reg_base, dma_group));

	if (enable_handshake) {
		ack |= BIT(channel);	/* Enable handshake for this channel */
	} else {
		ack &= ~BIT(channel);	/* Disable handshake for this channel */
	}

	sys_write32(ack, EVTRTR_DMA_ACK_TYPE_REG(reg_base, dma_group));

	LOG_DBG("%s ch %u: DMA_ACK_TYPE[%u] = 0x%08x (handshake %s)",
		dev->name, channel, dma_group, ack, enable_handshake ? "on" : "off");

	return 0;
}

/**
 * @brief Configure DMA transfer through event router
 * 1. Decodes channel, dma_group, and handshake from encoded channel parameter
 * 2. Configures the event router hardware
 * 3. Delegates to DMA driver with decoded physical channel
 *
 * @param dev Event router device
 * @param encoded_channel Encoded channel parameter from device tree
 * @param config DMA configuration with periph in dma_slot
 *
 * @return 0 on success, negative errno on error
 */
static int evtrtr_alif_configure(const struct device *dev, uint32_t encoded_channel,
				  struct dma_config *config)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uint32_t channel, periph, dma_group;
	bool enable_handshake;
	int ret;

	if (!config) {
		LOG_ERR("config is NULL");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/*
	 * Decode parameters:
	 * - From encoded_channel (cell 0): Physical DMA channel, dma_group, handshake
	 *   Bit layout: [reserved(19)][handshake(1)][dma_group(4)][channel(8)]
	 * - From config->dma_slot (cell 1): Peripheral request number (pure value)
	 */
	channel = ALIF_DMA_DECODE_CHANNEL(encoded_channel);
	dma_group = ALIF_DMA_DECODE_GROUP(encoded_channel);
	enable_handshake = ALIF_DMA_DECODE_HANDSHAKE(encoded_channel);
	periph = config->dma_slot;

	/* Validate event router parameters */
	if (periph >= cfg->num_channels) {
		LOG_ERR("Invalid periph %u (must be 0-%u)", periph, cfg->num_channels - 1);
		ret = -EINVAL;
		goto unlock;
	}

	if (dma_group >= cfg->num_groups) {
		LOG_ERR("Invalid dma_group %u (must be 0-%u)", dma_group, cfg->num_groups - 1);
		ret = -EINVAL;
		goto unlock;
	}

	LOG_INF("%s: Configuring periph=%u, dma_group=%u, dma_ch=%u, handshake=%s",
		dev->name, periph, dma_group, channel,
		enable_handshake ? "enabled" : "disabled");

	/* Configure event router hardware */
	ret = evtrtr_configure_channel(dev, periph,
					dma_group, enable_handshake);
	if (ret < 0) {
		LOG_ERR("Failed to configure event router hardware: %d", ret);
		goto unlock;
	}

	/*
	 * Delegate to DMA driver with decoded physical channel.
	 */
	ret = dma_config(cfg->dma_dev, channel, config);
	if (ret < 0) {
		LOG_ERR("DMA configuration failed: %d", ret);
		goto unlock;
	}

	LOG_DBG("Event router and DMA configured successfully");

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/**
 * @brief Start DMA transfer (delegates to DMA driver)
 */
static int evtrtr_alif_start(const struct device *dev, uint32_t encoded_channel)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uint32_t channel = ALIF_DMA_DECODE_CHANNEL(encoded_channel);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = dma_start(cfg->dma_dev, channel);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("DMA start failed: %d", ret);
	}

	return ret;
}

/**
 * @brief Stop DMA transfer (delegates to DMA driver)
 */
static int evtrtr_alif_stop(const struct device *dev, uint32_t encoded_channel)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uint32_t channel = ALIF_DMA_DECODE_CHANNEL(encoded_channel);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = dma_stop(cfg->dma_dev, channel);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("DMA stop failed: %d", ret);
	}

	return ret;
}

/**
 * @brief Reload DMA transfer (delegates to DMA driver)
 */
static int evtrtr_alif_reload(const struct device *dev, uint32_t encoded_channel,
			       uint32_t src, uint32_t dst, size_t size)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uint32_t channel = ALIF_DMA_DECODE_CHANNEL(encoded_channel);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = dma_reload(cfg->dma_dev, channel, src, dst, size);
	k_mutex_unlock(&data->lock);

	if (ret < 0) {
		LOG_ERR("DMA reload failed: %d", ret);
	}

	return ret;
}

/**
 * @brief Get DMA status (delegates to DMA driver)
 */
static int evtrtr_alif_get_status(const struct device *dev, uint32_t encoded_channel,
				   struct dma_status *stat)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uint32_t channel = ALIF_DMA_DECODE_CHANNEL(encoded_channel);
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = dma_get_status(cfg->dma_dev, channel, stat);
	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * @brief Control API: Configure event router channel without DMA transfer
 *
 * This function is for non-DMA use cases like UTIMER loopback where events
 * are routed between peripherals without involving DMA transfers.
 *
 * @note This API only manages DMA_CTRL register, not DMA_ACK_TYPE (handshake).
 *       This is appropriate because channel usage (DMA vs non-DMA) is statically
 *       determined at compile time via device tree and does not change at runtime.
 *       DMA channels use the standard DMA API which manages handshake separately.
 *
 * @param dev Event router device
 * @param channel Event router channel
 * @param dma_group Input source selection
 * @param enable true to enable channel, false to disable
 *
 * @return 0 on success, negative errno on error
 */
int alif_evtrtr_configure_channel(const struct device *dev,
				   uint32_t channel,
				   uint32_t dma_group,
				   bool enable)
{
	const struct evtrtr_alif_config *cfg;
	struct evtrtr_alif_data *data;
	uintptr_t reg_base;
	uint32_t ctrl;
	int ret = 0;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	cfg = dev->config;
	data = dev->data;
	reg_base = DEVICE_MMIO_GET(dev);
	k_mutex_lock(&data->lock, K_FOREVER);

	/* Validate parameters */
	if (channel >= cfg->num_channels) {
		LOG_ERR("Invalid channel %u (max %u)",
			channel, cfg->num_channels - 1);
		ret = -EINVAL;
		goto unlock;
	}

	if (dma_group >= cfg->num_groups) {
		LOG_ERR("Invalid dma_group %u (must be 0-%u)", dma_group, cfg->num_groups - 1);
		ret = -EINVAL;
		goto unlock;
	}

	if (enable) {
		/* Enable event router channel */
		ctrl = EVTRTR_DMA_CTRL_ENA |
		       (dma_group & EVTRTR_DMA_CTRL_GROUP_MASK);
		sys_write32(ctrl, EVTRTR_DMA_CTRL_REG(reg_base, channel));
		LOG_INF("%s: Enabled ch %u with dma_group %u", dev->name, channel, dma_group);
	} else {
		/* Disable event router channel */
		sys_write32(0, EVTRTR_DMA_CTRL_REG(reg_base, channel));
		LOG_INF("%s: Disabled ch %u", dev->name, channel);
	}

unlock:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* DMA API driver structure */
static const struct dma_driver_api evtrtr_alif_api = {
	.config = evtrtr_alif_configure,
	.start = evtrtr_alif_start,
	.stop = evtrtr_alif_stop,
	.reload = evtrtr_alif_reload,
	.get_status = evtrtr_alif_get_status,
};

/**
 * @brief Initialize event router device
 *
 * Called during system initialization. Clears all event router channels
 * and handshake enables to ensure clean slate.
 *
 * @param dev Event router device
 *
 * @return 0 on success
 */
static int evtrtr_alif_init(const struct device *dev)
{
	const struct evtrtr_alif_config *cfg = dev->config;
	struct evtrtr_alif_data *data = dev->data;
	uintptr_t reg_base;
	uint32_t i;

	/* Verify DMA controller device is ready */
	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA controller device %s is not ready", cfg->dma_dev->name);
		return -ENODEV;
	}

	/* Map MMIO region (for MMU support on Cortex-A cores) */
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	reg_base = DEVICE_MMIO_GET(dev);

	/* Initialize mutex */
	k_mutex_init(&data->lock);

	/* Clear all event router channels */
	for (i = 0; i < cfg->num_channels; i++) {
		sys_write32(0, EVTRTR_DMA_CTRL_REG(reg_base, i));
	}

	/* Clear all handshake enable registers (one per dma_group) */
	for (i = 0; i < cfg->num_groups; i++) {
		sys_write32(0, EVTRTR_DMA_ACK_TYPE_REG(reg_base, i));
	}

	/* Initialize DMA Request Control register - enable all request types */
	sys_write32(EVTRTR_DMA_REQ_CTRL_CB | EVTRTR_DMA_REQ_CTRL_CLB |
		    EVTRTR_DMA_REQ_CTRL_CS | EVTRTR_DMA_REQ_CTRL_CLS,
		    reg_base + EVTRTR_DMA_REQ_CTRL_OFFSET);

	LOG_INF("%s: Initialized at 0x%lx, %u channels, %u dma-groups",
		dev->name, (unsigned long)reg_base, cfg->num_channels, cfg->num_groups);

	return 0;
}

/* Device instantiation macros */
#define EVTRTR_ALIF_INIT(inst)							\
										\
	static struct evtrtr_alif_data evtrtr_alif_data_##inst;			\
										\
	static const struct evtrtr_alif_config evtrtr_alif_config_##inst = {	\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),			\
		.dma_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, dma_controller)), \
		.num_channels = DT_INST_PROP(inst, channels),			\
		.num_groups = DT_INST_PROP(inst, dma_groups),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      evtrtr_alif_init,					\
			      NULL,						\
			      &evtrtr_alif_data_##inst,				\
			      &evtrtr_alif_config_##inst,			\
			      POST_KERNEL,					\
			      CONFIG_EVENT_ROUTER_ALIF_INIT_PRIORITY,		\
			      &evtrtr_alif_api);

DT_INST_FOREACH_STATUS_OKAY(EVTRTR_ALIF_INIT)

/* Verify event router initializes after DMA controller */
BUILD_ASSERT(CONFIG_EVENT_ROUTER_ALIF_INIT_PRIORITY >= CONFIG_DMA_INIT_PRIORITY,
	     "CONFIG_EVENT_ROUTER_ALIF_INIT_PRIORITY must be >= CONFIG_DMA_INIT_PRIORITY");
