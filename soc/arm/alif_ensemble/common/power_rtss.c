/*
 * Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#if defined(CONFIG_PM_S2RAM)
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/drivers/timer/system_timer.h>
#endif
#include <zephyr/logging/log.h>
#include <cmsis_core.h>
#include <pm_rtss.h>

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/* WICCONTROL register */
#if defined(CONFIG_RTSS_HP)
#define WICCONTROL                  (AON_RTSS_HP_CTRL)
#elif defined(CONFIG_RTSS_HE)
#define WICCONTROL                  (AON_RTSS_HE_CTRL)
#else
#error "Invalid CPU"
#endif

/* WIC bit positions in WICCONTROL */
/*!< WICCONTROL: bit 8 (architecture dependent) */
#define WICCONTROL_WIC_Pos (8U)
#define WICCONTROL_WIC_Msk (1U << WICCONTROL_WIC_Pos)

#if defined(CONFIG_PM_S2RAM)
static int pm_suspend_to_ram(void)
{
	pm_core_enter_deep_sleep_request_subsys_off();

	return 0;
}
#endif

/* Handle PM specific states */
__weak void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
#if defined(CONFIG_PM_S2RAM)
	case PM_STATE_SUSPEND_TO_RAM:
		__disable_irq();
		__set_BASEPRI(0);

		/* Save context and enter Standby mode */
		pm_s2ram_save_ext_regs();

		arch_pm_s2ram_suspend(pm_suspend_to_ram);

		/* Restore context */
		pm_s2ram_restore_ext_regs();

		/* Clear the WIC Sleep */
		sys_write32(_VAL2FLD(WICCONTROL_WIC, 0), WICCONTROL);

		/* Restore system clock as soon as we exit standby mode */
		sys_clock_idle_exit();
		break;
#endif
	case PM_STATE_SOFT_OFF:
		__disable_irq();
		__set_BASEPRI(0);
		pm_core_enter_deep_sleep_request_subsys_off();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}
}

/* Handle tasks after Low Power Mode Exit */
__weak void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	switch (state) {
#if defined(CONFIG_PM_S2RAM)
	case PM_STATE_SUSPEND_TO_RAM:
		__enable_irq();
		break;
#endif
	case PM_STATE_SOFT_OFF:
		__enable_irq();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now in active mode. Reenable interrupts which were disabled
	 * when OS started idling code.
	 */
	irq_unlock(0);
}
