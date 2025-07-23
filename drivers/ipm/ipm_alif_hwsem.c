/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_hwsem

#define LOG_LEVEL CONFIG_IPM_LOG_LEVEL
#define LOG_MODULE_NAME ipm_alif_hwsem
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>

#define TIMEOUT 0xFF
#define DRV_HWSEM_CFG(d) ((const struct hwsem_device_config *)(d)->config)
#define DRV_HWSEM_DATA(d) ((struct hwsem_device_data *)(d)->data)
#define DRV_HWSEM_BASE(config) (struct hwsem_reg_block *)(config)->base;

typedef void (*init_func_t) (const struct device *);
typedef int (*hwsem_trylock_t)(const struct device *, uint32_t);
typedef int (*hwsem_lock_t)(const struct device *, uint32_t);
typedef int (*hwsem_unlock_t)(const struct device *, uint32_t);

struct hwsem_driver_api_t {
	hwsem_trylock_t trylock;
	hwsem_lock_t lock;
	hwsem_unlock_t unlock;
};

struct hwsem_device_config {
	uint8_t *base;
	int16_t irq_num;
	init_func_t init_func;
};

struct hwsem_device_data {
	volatile uint32_t id;
	volatile bool flag;
};

struct hwsem_reg_block {
	volatile uint32_t lock_r;
	volatile uint32_t unlock_r;
	volatile uint32_t reset_r;
};
static int hwsem_trylock(const struct device *, const uint32_t);
static int hwsem_lock(const struct device *, const uint32_t);
static int hwsem_unlock(const struct device *, const uint32_t);

static int hwsem_trylock(const struct device *d, const uint32_t id)
{
	if (!id) {
		LOG_ERR("%s: master id is empty\n", __func__);
		return -EINVAL;
	}
	uint8_t count = TIMEOUT;
	const struct hwsem_device_config *drv_cfg = DRV_HWSEM_CFG(d);
	struct hwsem_reg_block *reg_status = DRV_HWSEM_BASE(drv_cfg);
	struct hwsem_device_data *data = DRV_HWSEM_DATA(d);

	data->id = id;
	data->flag = false;

	if ((reg_status->lock_r == data->id) && reg_status->unlock_r) {
		/* HWSEM is already taken */
		LOG_INF("%s: Already locked HWSEM is locked again\n",
			 __func__);
		reg_status->lock_r = data->id;
		return 0;
	}
	/* Enable IRQ and wait till HWSEM is acquired */
	else {
		irq_enable(drv_cfg->irq_num);
		while (count) {
			if (data->flag) {
				/* HWSEM is locked */
				LOG_INF("%s: HWSEM locked!\n", __func__);
				break;
			}
			--count;
		}

		if (!count) {
			irq_disable(drv_cfg->irq_num);
			LOG_INF("%s: Timeout!\n", __func__);
			return -EBUSY;
		}

		return 0;
	}
}
static int hwsem_unlock(const struct device *d, const uint32_t id)
{
	const struct hwsem_device_config *drv_cfg = DRV_HWSEM_CFG(d);
	struct hwsem_reg_block *reg_status = DRV_HWSEM_BASE(drv_cfg);
	struct hwsem_device_data *data = DRV_HWSEM_DATA(d);

	if (!id) {
		LOG_ERR("%s: master id is empty\n", __func__);
		return -EINVAL;
	}
	if (!reg_status->unlock_r) {
		LOG_WRN("%s: trying to HWSEM that is not locked\n",
			__func__);
		return -1;
	}
	/* Unlocking actual locked HWSEM */
	if (reg_status->lock_r == data->id) {
		/* Do not raise interrupt after release. */
		/* Interrupt is raised after enabling irq */
		/* through hwsem_lock or hwsem_trylock */
		if (reg_status->unlock_r == 1) {
			irq_disable(drv_cfg->irq_num);
			compiler_barrier();
		}
		LOG_INF("%s: HWSEM unlocked!\n", __func__);
		reg_status->unlock_r = data->id;
		return 0;
	}

	LOG_ERR("%s: HWSEM is not owned by this core\n", __func__);
	return -1;
}
static int hwsem_lock(const struct device *d, const uint32_t id)
{
	const struct hwsem_device_config *drv_cfg = DRV_HWSEM_CFG(d);
	struct hwsem_reg_block *reg_status = DRV_HWSEM_BASE(drv_cfg);
	struct hwsem_device_data *data = DRV_HWSEM_DATA(d);

	if (!id) {
		LOG_ERR("%s: master id is empty\n", __func__);
		return -EINVAL;
	}
	data->id = id;
	data->flag = false;

	/* HWSEM is already locked */
	if ((reg_status->lock_r == data->id) && reg_status->unlock_r) {
		/* HWSEM is already taken */
		LOG_INF("%s: Already locked HWSEM is locked again\n",
			__func__);
		reg_status->lock_r = data->id;
		return 0;
	}
	/* Wait indefinitely until HWSEM is acquired */
	else {
		irq_enable(drv_cfg->irq_num);
		while (!data->flag) {
			/* Other cores might have taken HWSEM */
			/* and disabled interrupts */
			if (!irq_is_enabled(drv_cfg->irq_num)) {
				irq_enable(drv_cfg->irq_num);
			}
		}
		LOG_INF("%s: HWSEM locked!\n", __func__);
		return 0;
	}
}

static int hwsem_init(const struct device *d)
{
	const struct hwsem_device_config *config = DRV_HWSEM_CFG(d);

	config->init_func(d);
	return 0;
}

static void hwsem_isr(const struct device *d)
{
	const struct hwsem_device_config *config = DRV_HWSEM_CFG(d);
	struct hwsem_device_data *drv_data = DRV_HWSEM_DATA(d);
	struct hwsem_reg_block *reg_status = DRV_HWSEM_BASE(config);

	irq_disable(config->irq_num);
	reg_status->lock_r = drv_data->id;
	compiler_barrier();
	if (reg_status->lock_r == drv_data->id) {
		drv_data->flag = true;
	}
}

static const struct hwsem_driver_api_t hwsem_driver_api = {
	.trylock = hwsem_trylock,
	.lock = hwsem_lock,
	.unlock = hwsem_unlock,
};

#if defined(CONFIG_ALIF_HWSEM)
#define ALIF_HWSEM_DATA_INST(n)					\
	static void hwsem_##n##_init(const struct device *dev);		\
	static const struct hwsem_device_config hwsem_cfg_##n = {	\
		.base = (uint8_t *)DT_INST_REG_ADDR(n),			\
		.irq_num = DT_INST_IRQN(n),				\
		.init_func = hwsem_##n##_init,				\
	};								\
									\
	static struct hwsem_device_data hwsem_data_##n = {		\
		.id = 0x0,						\
		.flag = false,						\
	};								\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      hwsem_init,				\
			      NULL,					\
			      &hwsem_data_##n,				\
			      &hwsem_cfg_##n, POST_KERNEL,		\
			      CONFIG_ALIF_HWSEM_INIT_PRIORITY,		\
			      &hwsem_driver_api);			\
	static void hwsem_##n##_init(const struct device *dev)		\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0,						\
			    hwsem_isr,					\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_disable(DT_INST_IRQN(n));				\
	}
#endif

DT_INST_FOREACH_STATUS_OKAY(ALIF_HWSEM_DATA_INST)
