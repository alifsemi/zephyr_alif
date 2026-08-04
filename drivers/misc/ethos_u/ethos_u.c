/*
 * SPDX-FileCopyrightText: <text>Copyright 2021-2022, 2024 Arm Limited and/or its
 * affiliates <open-source-office@arm.com></text>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys_clock.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <ethosu_driver.h>
#include <ethosu_device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ethos_u, CONFIG_ARM_ETHOS_U_LOG_LEVEL);

#define DT_DRV_COMPAT arm_ethos_u

/*******************************************************************************
 * Re-implementation/Overrides __((weak)) symbol functions from ethosu_driver.c
 * To handle mutex and semaphores
 *******************************************************************************/

void *ethosu_mutex_create(void)
{
	struct k_mutex *mutex;

	mutex = k_malloc(sizeof(*mutex));
	if (mutex == NULL) {
		LOG_ERR("Failed allocate mutex");
		return NULL;
	}

	k_mutex_init(mutex);

	return (void *)mutex;
}

int ethosu_mutex_lock(void *mutex)
{
	int status;

	status = k_mutex_lock((struct k_mutex *)mutex, K_FOREVER);
	if (status != 0) {
		LOG_ERR("Failed to lock mutex with error - %d", status);
		return -1;
	}

	return 0;
}

int ethosu_mutex_unlock(void *mutex)
{
	k_mutex_unlock((struct k_mutex *)mutex);
	return 0;
}

void *ethosu_semaphore_create(void)
{
	struct k_sem *sem;

	sem = k_malloc(sizeof(*sem));
	if (sem == NULL) {
		LOG_ERR("Failed to allocate semaphore");
		return NULL;
	}

	k_sem_init(sem, 0, 100);

	return (void *)sem;
}

int ethosu_semaphore_take(void *sem, uint64_t timeout)
{
	int status;

	status = k_sem_take((struct k_sem *)sem, (timeout == ETHOSU_SEMAPHORE_WAIT_FOREVER)
							 ? K_FOREVER
							 : Z_TIMEOUT_TICKS(timeout));

	if (status != 0) {
		/* The Ethos-U driver expects the semaphore implementation to never fail except for
		 * when a timeout occurs, and the current ethosu_semaphore_take implementation makes
		 * no distinction, in terms of return codes, between a timeout and other semaphore
		 * take failures. Also, note that a timeout is virtually indistinguishable from
		 * other failures if the driver logging is disabled. Handling errors other than a
		 * timeout is therefore not covered here and is deferred to the application
		 * developer if necessary.
		 */
		if (status != -EAGAIN) {
			LOG_ERR("Failed to take semaphore with error - %d", status);
		}
		return -1;
	}

	return 0;
}

int ethosu_semaphore_give(void *sem)
{
	k_sem_give((struct k_sem *)sem);
	return 0;
}

void ethosu_semaphore_destroy(void *sem)
{
	k_free(sem);
}

void ethosu_mutex_destroy(void *mutex)
{
	k_free(mutex);
}

struct ethosu_dts_info {
	void *base_addr;
	bool secure_enable;
	bool privilege_enable;
	unsigned int irqn;
	void (*irq_config)(void);
	const struct device *clk_dev;
	clock_control_subsys_t clk_id;
};

struct ethosu_data {
	struct ethosu_driver drv;
};

void ethosu_zephyr_irq_handler(const struct device *dev)
{
	struct ethosu_data *data = dev->data;
	struct ethosu_driver *drv = &data->drv;

	ethosu_irq_handler(drv);
}

static int ethosu_zephyr_init(const struct device *dev)
{
	const struct ethosu_dts_info *config = dev->config;
	struct ethosu_data *data = dev->data;
	struct ethosu_driver *drv = &data->drv;
	struct ethosu_driver_version version;

	LOG_DBG("Ethos-U DTS info. base_address=0x%p, secure_enable=%u, privilege_enable=%u",
		config->base_addr, config->secure_enable, config->privilege_enable);

	if (config->clk_dev != NULL) {
		if (!device_is_ready(config->clk_dev)) {
			LOG_ERR("Clock device not ready");
			return -ENODEV;
		}
		if (clock_control_on(config->clk_dev, config->clk_id)) {
			LOG_ERR("Failed to enable NPU clock");
			return -EIO;
		}
	}

	ethosu_get_driver_version(&version);

	LOG_DBG("Version. major=%u, minor=%u, patch=%u", version.major, version.minor,
		version.patch);

	if (ethosu_init(drv, config->base_addr, NULL, 0, config->secure_enable,
			config->privilege_enable)) {
		LOG_ERR("Failed to initialize NPU with ethosu_init().");
		return -EINVAL;
	}

	config->irq_config();

	return 0;
}

#if defined(CONFIG_PM_DEVICE)
/**
 * @brief Ethos-U PM device action handler
 *
 * Handles power management state transitions for the Ethos-U device.
 * Coordinates with power domain via PM framework.
 *
 * @param dev Ethos-U device struct
 * @param action PM device action
 *
 * @return 0 if successful, negative errno otherwise
 */
static int ethosu_pm_action(const struct device *dev, enum pm_device_action action)
{
	int ret = 0;
	const struct ethosu_dts_info *config = dev->config;
	struct ethosu_data *data = dev->data;
	struct ethosu_driver *drv = &data->drv;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return ethosu_zephyr_init(dev);

	case PM_DEVICE_ACTION_SUSPEND:
		/*
		 * Refuse to suspend while the NPU is reserved or an inference is
		 * in flight. Tearing down in that state would free the driver
		 * semaphore under an active job, and ethosu_deregister_driver()
		 * would block on the global handle semaphore while holding the
		 * driver mutex, deadlocking the subsystem.
		 */
		if (drv->reserved || drv->job.state != ETHOSU_JOB_IDLE) {
			return -EBUSY;
		}

		/*
		 * Enable NPU clock and power gating for low-power STOP mode.
		 * This writes NPU registers, so it must run while the NPU is
		 * still powered and accessible (before ethosu_deinit()).
		 */
		if (ethosu_dev_set_clock_and_power(&drv->dev, ETHOSU_CLOCK_Q_ENABLE,
						   ETHOSU_POWER_Q_ENABLE) != ETHOSU_SUCCESS) {
			return -EIO;
		}

		/*
		 * Disable the NPU IRQ before freeing the driver semaphore so a
		 * stale/pending interrupt cannot call ethosu_semaphore_give() on
		 * freed memory.
		 */
		irq_disable(config->irqn);

		/*
		 * Symmetric teardown: deregister the handle and free the driver
		 * semaphore so the next RESUME's ethosu_init() starts from a clean
		 * state. Retained SRAM would otherwise make
		 * ethosu_register_driver() splice the driver onto itself.
		 */
		ethosu_deinit(drv);
		return 0;

	case PM_DEVICE_ACTION_TURN_OFF:
	case PM_DEVICE_ACTION_TURN_ON:
		break;

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

#define ETHOSU_DEVICE_INIT(n)                                                                      \
	static struct ethosu_data ethosu_data_##n;                                                 \
                                                                                                   \
	static void ethosu_zephyr_irq_config_##n(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ethosu_zephyr_irq_handler,  \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct ethosu_dts_info ethosu_dts_info_##n = {                                \
		.base_addr = (void *)DT_INST_REG_ADDR(n),                                          \
		.secure_enable = DT_INST_PROP(n, secure_enable),                                   \
		.privilege_enable = DT_INST_PROP(n, privilege_enable),                             \
		.irqn = DT_INST_IRQN(n),                                                           \
		.irq_config = &ethosu_zephyr_irq_config_##n,                                       \
		.clk_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, clocks),                           \
			(DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n))), (NULL)),			   \
		.clk_id = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, clocks),                            \
			((clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, clkid)), (NULL)),	   \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, ethosu_pm_action);                                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ethosu_zephyr_init, PM_DEVICE_DT_INST_GET(n), &ethosu_data_##n,   \
			      &ethosu_dts_info_##n, POST_KERNEL,                                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);

DT_INST_FOREACH_STATUS_OKAY(ETHOSU_DEVICE_INIT);
