/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_power_domain

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/power-domain/alif_power_domain.h>
#include <se_service.h>
#include <aipm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(power_domain, CONFIG_POWER_DOMAIN_LOG_LEVEL);

/* Maximum number of power domains supported by Alif SoCs */
#define ALIF_PD_MAX_DOMAINS 10

/**
 * @brief Per-domain configuration (from DTS)
 */
struct alif_pd_config {
	uint32_t domain_id;  /* ALIF_PD_SYST, ALIF_PD_SSE700_AON, etc. */
};

/**
 * @brief Per-domain runtime data
 */
struct alif_pd_data {
	struct k_spinlock lock;
};

/**
 * @brief Global shared state for all power domains
 * Since SE services operate on all domains together, we need shared state.
 */
struct alif_pd_global {
	atomic_t refcounts[ALIF_PD_MAX_DOMAINS];  /* Refcount per domain ID */
} alif_pd_global_state;

/**
 * @brief Convert power domain ID to SE mask
 */
static inline uint32_t pd_id_to_mask(uint32_t pd_id)
{
	return BIT(pd_id);
}

/**
 * @brief Update SE power domain configuration
 * @param pd_id Power domain ID (ALIF_PD_SYST, ALIF_PD_SSE700_AON, etc.)
 * @param enable True to enable domain, false to disable
 *
 * Note: SSE AON domain is always on when any core is active and cannot be
 * controlled via SE service. For SSE AON, this function returns success
 * without calling SE service to avoid unnecessary overhead.
 *
 * WORKAROUND: Due to SE firmware bug, only enable operations are performed.
 * Disable operations are skipped - SE firmware will handle domain disable
 * during set_off_cfg() execution. This will be fixed in future SE versions.
 */
 static int alif_pd_update_se(uint32_t pd_id, bool enable)
{
	run_profile_t runp;
	uint32_t pd_mask;
	bool is_enabled;
	int ret;

	/*
	 * SSE AON is always on when any core is running and cannot be controlled.
	 * Skip SE service call to avoid overhead - the PM framework will still
	 * handle device suspend/resume (e.g., LPUART reconfiguration after STOP mode).
	 */
	if (pd_id == ALIF_PD_SSE700_AON) {
		LOG_DBG("SSE AON domain always on - skipping SE service call");
		return 0;
	}

	pd_mask = pd_id_to_mask(pd_id);

	/* Read current SE configuration */
	ret = se_service_get_run_cfg(&runp);
	if (ret) {
		LOG_ERR("Failed to get SE run config: %d", ret);
		return -EIO;
	}

	/* Check if domain is already in the desired state */
	is_enabled = (runp.power_domains & pd_mask) != 0;
	if (is_enabled == enable) {
		LOG_DBG("Domain %u already in desired state (%s) - skipping SE call",
			pd_id, enable ? "enabled" : "disabled");
		return 0;
	}

	/*
	 * WORKAROUND: Only perform enable operations via set_run_cfg().
	 * Disable operations are skipped due to SE firmware bug.
	 *
	 * Rationale: SE firmware requires SYSTOP to be enabled when calling
	 * set_off_cfg(). If we disable SYSTOP here, the core cannot properly
	 * enter OFF states. SE firmware will disable SYSTOP internally as part
	 * of the OFF state transition.
	 *
	 * This will be removed once SE firmware is fixed to handle both
	 * enable and disable operations correctly.
	 */
	if (enable) {
		runp.power_domains |= pd_mask;
		LOG_DBG("Enabling domain %u via SE service", pd_id);

		/* Apply new configuration */
		ret = se_service_set_run_cfg(&runp);
		if (ret) {
			LOG_ERR("Failed to set SE run config: %d", ret);
			return -EIO;
		}
	} else {
		/*
		 * Skip disable operation - SE firmware will handle this during
		 * set_off_cfg() execution. The refcount is still decremented
		 * for proper tracking.
		 */
	}

	return 0;
}

/**
 * @brief Power domain PM action handler
 * Called by PM framework for runtime PM and system suspend/resume
 */
static int alif_pd_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct alif_pd_config *config = dev->config;
	uint32_t pd_id = config->domain_id;
	atomic_val_t old_refcount, new_refcount;
	int ret = 0;

	if (pd_id >= ALIF_PD_MAX_DOMAINS) {
		LOG_ERR("Invalid power domain ID %u (max %u)", pd_id, ALIF_PD_MAX_DOMAINS - 1);
		return -EINVAL;
	}

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/*
		 * Runtime PM get or system resume.
		 * Increment refcount and enable domain if transitioning 0->1.
		 */
		old_refcount = atomic_inc(&alif_pd_global_state.refcounts[pd_id]);
		new_refcount = old_refcount + 1;

		LOG_DBG("Domain %u RESUME: refcount %ld -> %ld",
			pd_id, (long)old_refcount, (long)new_refcount);

		if (old_refcount == 0) {
			/*
			 * First user - enable the domain.
			 * Note: For system resume, SYSTOP may already be enabled by
			 * application's pre_device_resume(). The update function will
			 * detect this and skip redundant SE call.
			 */
			ret = alif_pd_update_se(pd_id, true);
			if (ret) {
				/* Rollback refcount on failure */
				atomic_dec(&alif_pd_global_state.refcounts[pd_id]);
				LOG_ERR("Failed to enable domain %u", pd_id);
				return ret;
			}
			LOG_DBG("Enabled power domain %u", pd_id);
		}

		/* Notify child devices that power is available */
		pm_device_children_action_run(dev, PM_DEVICE_ACTION_TURN_ON, NULL);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		/*
		 * Runtime PM put or system suspend.
		 * First notify children, then decrement refcount and disable if 1->0.
		 *
		 * Note: For system PM (entering OFF states), the application's PM notifier
		 * will re-enable SYSTOP before calling set_off_cfg() to meet SE requirements.
		 */
		pm_device_children_action_run(dev, PM_DEVICE_ACTION_TURN_OFF, NULL);

		old_refcount = atomic_dec(&alif_pd_global_state.refcounts[pd_id]);
		new_refcount = old_refcount - 1;

		LOG_DBG("Domain %u SUSPEND: refcount %ld -> %ld",
			pd_id, (long)old_refcount, (long)new_refcount);

		if (old_refcount == 1) {
			/* Last user - disable the domain */
			ret = alif_pd_update_se(pd_id, false);
			if (ret) {
				/* Rollback refcount on failure */
				atomic_inc(&alif_pd_global_state.refcounts[pd_id]);
				LOG_ERR("Failed to disable domain %u", pd_id);
				return ret;
			}
			LOG_INF("Disabled power domain %u", pd_id);
		} else if (old_refcount == 0) {
			/* Should never happen - refcount underflow */
			LOG_ERR("Refcount underflow for domain %u", pd_id);
			atomic_inc(&alif_pd_global_state.refcounts[pd_id]);
			return -EINVAL;
		}
		break;

	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		/* No-op: handled by parent power domain if any */
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Initialize a power domain device
 */
static int alif_pd_init(const struct device *dev)
{
	const struct alif_pd_config *config = dev->config;
	uint32_t pd_id = config->domain_id;

	if (pd_id >= ALIF_PD_MAX_DOMAINS) {
		LOG_ERR("Invalid power domain ID %u (max %u)", pd_id, ALIF_PD_MAX_DOMAINS - 1);
		return -EINVAL;
	}

	/*
	 * Initialize refcount to 0. The PM framework will call RESUME during
	 * pm_device_driver_init() below, incrementing refcount to 1 and enabling
	 * the domain via SE service.
	 *
	 * This design supports future runtime PM: when peripherals call
	 * pm_device_runtime_get/put(), the refcount will track actual usage
	 * and enable/disable the domain accordingly.
	 */
	atomic_set(&alif_pd_global_state.refcounts[pd_id], 0);

	/*
	 * Use pm_device_driver_init() to set initial PM state.
	 * This will call TURN_ON + RESUME, setting refcount to 1 and enabling
	 * the domain via SE service (matching bootloader state for SYST/SSE_AON).
	 */
	return pm_device_driver_init(dev, alif_pd_pm_action);
}

/* Macro to define one power domain device instance */
#define ALIF_POWER_DOMAIN_DEVICE(inst)						\
	static const struct alif_pd_config alif_pd_config_##inst = {		\
		.domain_id = DT_INST_PROP(inst, domain_id),			\
	};									\
										\
	static struct alif_pd_data alif_pd_data_##inst;				\
										\
	PM_DEVICE_DT_INST_DEFINE(inst, alif_pd_pm_action);			\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      alif_pd_init,					\
			      PM_DEVICE_DT_INST_GET(inst),			\
			      &alif_pd_data_##inst,				\
			      &alif_pd_config_##inst,				\
			      PRE_KERNEL_1,					\
			      CONFIG_POWER_DOMAIN_ALIF_INIT_PRIORITY,		\
			      NULL);

/* Create one device instance per DT node */
DT_INST_FOREACH_STATUS_OKAY(ALIF_POWER_DOMAIN_DEVICE)
