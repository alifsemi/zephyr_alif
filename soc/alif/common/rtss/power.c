/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <soc_common.h>
#if defined(CONFIG_PM)
#include <zephyr/pm/pm.h>
#endif
#if defined(CONFIG_POWEROFF)
#include <zephyr/sys/poweroff.h>
#endif
#if defined(CONFIG_PM_S2RAM)
#include <zephyr/arch/common/pm_s2ram.h>
#endif

LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

/* WICCONTROL register */
#if defined(CONFIG_RTSS_HP)
#define WICCONTROL                  (AON_RTSS_HP_CTRL)
#elif defined(CONFIG_RTSS_HE)
#define WICCONTROL                  (AON_RTSS_HE_CTRL)
#endif

/**
 * @brief LPGPIO Interrupt numbers
 */
#define LPGPIO_COMB_IRQ_IRQn	57
#define LPGPIO_IRQ0_IRQn	171

/**
 * @brief WakeUp Interrupt Controller(WIC) Type:-
 */
#define PM_WIC_IS_EWIC	0	/*!< WIC used is EWIC */
#define PM_WIC_IS_IWIC	1	/*!< WIC used is IWIC */

/* WIC bit positions in WICCONTROL */
/*!< WICCONTROL: bit 8 (architecture dependent) */
#define WICCONTROL_WIC_Pos (8U)
#define WICCONTROL_WIC_Msk (1U << WICCONTROL_WIC_Pos)

/* IWIC bit positions in WICCONTROL */
/*!< WICCONTROL: bit 9 (architecture dependent) */
#define WICCONTROL_IWIC_Pos	(9U)
#define WICCONTROL_IWIC_Msk	(1U << WICCONTROL_IWIC_Pos)

/**
 * @brief Low power state values for LPSTATE field
 */
enum pm_lpstate {
	PM_LPSTATE_ON,         /* ON */
	PM_LPSTATE_ON_CLK_OFF, /* ON, clock is off */
	PM_LPSTATE_RET,        /* Retention (not supported) */
	PM_LPSTATE_OFF         /* OFF */
};

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpgpio), okay)
/**
 * @fn           uint32_t pm_prepare_lpgpio_nvic_mask(void)
 * @brief        Prepare NVIC mask for LPGPIO while going to subsystem off.
 * @return       LPGPIO NVIC mask state
 */
static uint32_t pm_prepare_lpgpio_nvic_mask(void)
{
	uint32_t lpgpio_nvic_mask_state = 0;

	/*
	 * LPGPIOs have to cause wakeup via a combined interrupt, as the
	 * main interrupts are not sent to the EWIC/IWIC
	 *
	 * Enable it transparently.
	 */

	/*
	 * We could use NVIC_GetEnableIRQ eight times, but would mean
	 * 8 register reads, instead peek directly to ISER.
	 *
	 *
	 * Read 0-7 interrupt enables depending on alignment of interrupt numbers
	 */

	uint32_t lpgpio_enables = NVIC->ISER[LPGPIO_IRQ0_IRQn >> 5]
				  >> (LPGPIO_IRQ0_IRQn & 0x1F);

	/*
	 * If split across two registers, combine enables
	 * from the second register
	 */
	if ((LPGPIO_IRQ0_IRQn & 0x1F) > 24) {
		lpgpio_enables |= NVIC->ISER[(LPGPIO_IRQ0_IRQn >> 5) + 1]
				  << (32 - (LPGPIO_IRQ0_IRQn & 0x1F));
	}

	lpgpio_enables &= 0xFF;

	/*
	 * If any LPGPIO is enabled, and the combined interrupt isn't,
	 * activate the combined one, and we'll restore it later
	 */
	if (lpgpio_enables && !NVIC_GetEnableIRQ(LPGPIO_COMB_IRQ_IRQn)) {
		NVIC_ClearPendingIRQ(LPGPIO_COMB_IRQ_IRQn);
		irq_enable(LPGPIO_COMB_IRQ_IRQn);
		lpgpio_nvic_mask_state |= 1;
	}

	return lpgpio_nvic_mask_state;
}

/**
 * @fn         void pm_restore_lpgpio_nvic_mask(uint32_t lpgpio_nvic_mask_state)
 * @brief      If the LPGPIO NVIC mask is set, disable the Combined Interrupt
 * @param[in]  lpgpio_nvic_mask_state LPGPIO NVIC mask state
 * @return     none
 */
static void pm_restore_lpgpio_nvic_mask(uint32_t lpgpio_nvic_mask_state)
{
	if (lpgpio_nvic_mask_state & 1) {
		irq_disable(LPGPIO_COMB_IRQ_IRQn);
	}
}
#endif

/**
 * @fn       uint16_t pm_core_enter_normal_sleep(void)
 * @brief    Power management API which performs normal sleep operation
 * @note     This function should be called with interrupts disabled.
 * @note     This function is provided for consistency with the
 *           deeper sleeps, which require interrupts disabled.
 *           In interrupt-enabled context, in bare-metal use, direct
 *           use of __WFE() may be more appropriate.
 * @return   This function return nothing
 */
static inline void pm_core_enter_normal_sleep(void)
{
	__WFI();
}

/**
 * @fn           void pm_core_enter_wic_sleep(PM_WIC wic)
 * @brief        Enter deep WIC-based sleep subroutine
 * @param[in]    wic 1 for IWIC sleep, 0 for EWIC.
 * @return       This function returns nothing, potentially causes power-down.
 */
static void pm_core_enter_wic_sleep(uint32_t wic)
{
	uint32_t regval;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpgpio), okay)
	/*
	 * See if we have any LPGPIO individual interrupts are enabled.
	 * If yes, enable the Combined interrupt.
	 */
	uint32_t lpgpio_nvic_mask_state = pm_prepare_lpgpio_nvic_mask();
#endif

	/* Set up WICCONTROL so that deep sleep is the required WIC sleep type */
	regval = _VAL2FLD(WICCONTROL_WIC, 1) | _VAL2FLD(WICCONTROL_IWIC, wic);
	sys_write32(regval, WICCONTROL);

	/* Setting DEEPSLEEP bit */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	/*Data Synchronization Barrier completes all instructions before this */
	__DSB();

	/* Instruction Synchronization Barrier flushes the pipeline in the
	 * processor, so that all instructions following the ISB are fetched from
	 * cache or memory
	 */
	__ISB();

	/* Put System into sleep mode */
	pm_core_enter_normal_sleep();

	/* Clearing DEEPSLEEP bit */
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	/* Clear WICCONTROL to disable WIC sleep */
	regval = _VAL2FLD(WICCONTROL_WIC, 0);
	sys_write32(regval, WICCONTROL);


	/* Data Synchronization Barrier completes all instructions before this */
	__DSB();

	/*
	 * Instruction Synchronization Barrier flushes the pipeline in the
	 * processor, so that all instructions following the ISB are fetched
	 * from cache or memory
	 */
	__ISB();
#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpgpio), okay)
	/*
	 * If we enabled the LPGPIO combined interrupt while going to deep sleep,
	 * disable it.
	 */
	pm_restore_lpgpio_nvic_mask(lpgpio_nvic_mask_state);
#endif
}

/**
 * @fn       void pm_core_enter_deep_sleep(void)
 * @brief    Power management API which performs deep sleep operation
 * @note     This function should be called with interrupts disabled
 *           This enters the deepest possible CPU sleep state, without
 *           losing CPU state. All CPU clocks can be stopped, including
 *           SysTick. CPU and subsystem power will remain on, and the
 *           clock continues to run to the Internal Wakeup Interrupt
 *           Controller (IWIC), which manages the wakeup.
 * @note     Possible IWIC wake sources are events, NMI, debug events
 *           and interrupts 0-63 only, subject to NVIC interrupt
 *           enable controls.
 * @return   This function returns nothing
 */
void pm_core_enter_deep_sleep(void)
{
	/* Entering any WIC sleep could potentially cause state loss,
	 * as it enables power saving on PDCORE. Unlike the other domains,
	 * there is no separate mechanism to indicate "retention" beyond
	 * setting CLPSTATE, so as we want a sleep call we need to
	 * ensure CLPSTATE is RET or higher. (See M55 TRM section 7.5)
	 *
	 * Further, in our design, the IWIC is actually in the same power domain
	 * as the CPU, so we need the IWIC+CPU to stay on to be able to wake,
	 * which means our minimum is actually ON with clock off.
	 *
	 * If caller has set CLPSTATE to RET or OFF (deeper sleep states),
	 * we temporarily raise it to ON_CLK_OFF to prevent state loss.
	 * After wakeup, we restore the original setting.
	 *
	 */
	uint32_t old_cpdlpstate = PWRMODCTL->CPDLPSTATE;

	PWRMODCTL->CPDLPSTATE = (old_cpdlpstate & ~PWRMODCTL_CPDLPSTATE_CLPSTATE_Msk) |
		_VAL2FLD(PWRMODCTL_CPDLPSTATE_CLPSTATE, PM_LPSTATE_ON_CLK_OFF);

	/* Trigger the IWIC sleep */
	pm_core_enter_wic_sleep(PM_WIC_IS_IWIC);

	/* Restore low power state (probably to all OFF) */
	PWRMODCTL->CPDLPSTATE = old_cpdlpstate;
}

/**
 * @fn       int pm_core_enter_deep_sleep_request_subsys_off(void)
 * @brief    Power management API which performs subsystem off operation
 *           This enters a deep sleep and indicates that it is okay for
 *           the CPU power, and hence potentially the entire subsystem's
 *           power, to be removed. Whether power actually is removed will
 *           depend on other factors - the CPU is not the only input
 *           to the decision.
 *
 *           If a wake-up source is signaled before power is removed,
 *           the function returns from its deep sleep.
 *
 *           If power is removed from the subsystem, the function does not
 *           return, and the CPU will reboot when/if the subsystem is next
 *           powered up, which could either be due to the local wakeup
 *           controller, or some other power-on request. Any wake-up sources
 *           will be indicated by a pending interrupt in the NVIC.
 *
 *           As there are many reasons the subsystem could wake, applications
 *           should be written to call this again on reboot when they find there
 *           are no wake reasons.
 *
 *           Where the system reboots from, can be controlled using the secure
 *           enclave APIs to set the initial vector table.
 *
 *           The RTSS-HE core can arrange for some or all of its TCM to be
 *           retained when the power is turned off by making calls to the
 *           secure enclave to configure the retention power.
 *
 *           The secure enclave can also arrange for various deep SoC sleep
 *           states to be entered if all subsystems have configured this, and
 *           enter sleep. So this call can lead to overall SoC sleep.
 *
 * @note     This function should be called with interrupts disabled.
 *           A cache clean operation is performed if necessary.
 * @note     This function will not return if the system goes off
 *           before a wake event. It will return if a wake event
 *           occurs before power off is possible.
 * @note     Possible EWIC wake sources are a limited selection
 *           of interrupts 0-63 - see the HWRM for details.
 *           The CPU may also reboot if power is automatically
 *           applied to the subsystem for other reasons aside from
 *           EWIC wakeup.
 *           The pending information from EWIC is transferred
 *           into the NVIC on startup, so interrupt handlers
 *           can respond to the cause as soon as they're
 *           unmasked by drivers.
 * @return   This function does not return for successful OFF,
 *           otherwise it will return -EBUSY.
 */
static int pm_core_enter_deep_sleep_request_subsys_off(void)
{
	uint32_t orig_ccr, orig_mscr, orig_demcr, orig_cppwr;

	/*
	 * We attempt to power off the subsystem by turning off all active
	 * indications from the CPU, taking its power domains PDCORE, PDEPU,
	 * PDRAMS and PDDEBUG to OFF. See Power chapter of M55 TRM for details.
	 *
	 * We assume all the LPSTATE indications are OFF as at boot, which will
	 * permit everything to go off. We assume that if it's set higher, it's
	 * because someone wants to block this. If they have modified it, and
	 * don't intend to block this, they should put it back to OFF before
	 * calling this.
	 */

	/*
	 * PDEPU OFF requires that we set the State Unknown 10 flag indicating
	 * it's okay to forget the FP/MVE state (S/D/Q registers, FPSR and VPR)
	 */
	orig_cppwr = ICB->CPPWR;
	if (!(orig_cppwr & ICB_CPPWR_SU10_Msk)) {
		/*
		 * Indicate we're okay to lose MVE/FP state. Note that MVE/FP
		 * instructions will fault after this, so we hope we're not doing
		 * anything that prompts the compiler to generate MVE/FP code
		 * during this function.
		 */
		ICB->CPPWR = orig_cppwr | (ICB_CPPWR_SU11_Msk |
					   ICB_CPPWR_SU10_Msk);
	}

	/* Stop new data cache allocations */
	orig_ccr = SCB->CCR;
	SCB->CCR = orig_ccr & (~SCB_CCR_DC_Msk);
	__DSB();
	__ISB();

	/* Check cache status */
	orig_mscr = MEMSYSCTL->MSCR;
	if (orig_mscr & MEMSYSCTL_MSCR_DCACTIVE_Msk) {
		/*
		 * Make sure nothing gets dirty any more -
		 * this should stabilize DCCLEAN
		 */
		MEMSYSCTL->MSCR = orig_mscr | MEMSYSCTL_MSCR_FORCEWT_Msk;
		__DSB();
		__ISB();

		if (!(MEMSYSCTL->MSCR & MEMSYSCTL_MSCR_DCCLEAN_Msk)) {
			/*
			 * Clean if data cache is active, and not known to be
			 * clean. This could be done earlier by
			 * pm_shut_down_dcache, before disabling IRQs. But if
			 * we're making this call, we're resigned to bad
			 * interrupt latency - we might be needing a full reboot
			 * to respond.
			 */
			SCB_CleanDCache();

			/*
			 * Should be good to manually mark clean now - M55 TRM
			 * tells us not to, but after some discussion with Arm,
			 * I believe we've taken enough care that this is valid
			 * at this point. (If the cache ISN'T clean, then we've
			 * failed on the shutdown).
			 */
			MEMSYSCTL->MSCR |= MEMSYSCTL_MSCR_DCCLEAN_Msk;
		}
	}

	/*
	 * Fully disable the caches, allowing PDRAMS OFF. (On B1 silicon it
	 * is important that we don't let the M55 request MEM_RET state by
	 * having PDRAMS at RET and PDCORE at OFF - the PPU will grant this,
	 * and the M55 will wrongly think its cache has been retained, and skip
	 * necessary auto-invalidation on the subsequent reset.) Restore FORCEWT
	 * now
	 */
	SCB->CCR = orig_ccr & ~(SCB_CCR_IC_Msk | SCB_CCR_DC_Msk);
	MEMSYSCTL->MSCR = (MEMSYSCTL->MSCR &
			   ~(MEMSYSCTL_MSCR_ICACTIVE_Msk |
			     MEMSYSCTL_MSCR_DCACTIVE_Msk  |
			     MEMSYSCTL_MSCR_FORCEWT_Msk)) |
			  (orig_mscr & MEMSYSCTL_MSCR_FORCEWT_Msk);
	/*
	 * Disable PMU/DWT - we know this is enabled at boot by system code
	 * using PMU timers, so we could never permit PDDEBUG OFF otherwise.
	 * When/if this is resolved, we should consider removing this, so
	 * as not to interfere with deliberate debugging.
	 */
	orig_demcr = DCB->DEMCR;
	DCB->DEMCR = orig_demcr & ~DCB_DEMCR_TRCENA_Msk;

	/*
	 * Assume automatic EWIC sequencing - NVIC masks transferred and EWIC
	 * enabled by M55.
	 */

	/* Trigger the EWIC sleep - may or may not return */
	pm_core_enter_wic_sleep(PM_WIC_IS_EWIC);

	/* If we return, restore enables */
	MEMSYSCTL->MSCR |= orig_mscr &
			   (MEMSYSCTL_MSCR_ICACTIVE_Msk |
			    MEMSYSCTL_MSCR_DCACTIVE_Msk);
	SCB->CCR = orig_ccr;
	DCB->DEMCR = orig_demcr;
	ICB->CPPWR = orig_cppwr;

	/* Make sure enables are synchronized */
	__DSB();
	__ISB();

	return -EBUSY;
}

#if defined(CONFIG_PM_S2RAM)
static int pm_suspend_to_ram(void)
{
	int ret;

	ret = pm_core_enter_deep_sleep_request_subsys_off();
	if (ret == -EBUSY) {
		LOG_DBG("Subsystem did not go to OFF state");
	}

	return ret;
}
#endif

#if defined(CONFIG_PM)
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

		break;
#endif
	case PM_STATE_SUSPEND_TO_IDLE:
		__disable_irq();
		__set_BASEPRI(0);

		/* Enter IWIC sleep - lighter than EWIC, no context save needed */
		pm_core_enter_deep_sleep();

		break;
	case PM_STATE_SOFT_OFF:
		__disable_irq();
		__set_BASEPRI(0);

		if (pm_core_enter_deep_sleep_request_subsys_off() == -EBUSY) {
			LOG_DBG("Subsystem did not go to OFF state %u", state);
		}
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
	case PM_STATE_SUSPEND_TO_IDLE:
		__enable_irq();
		break;
	case PM_STATE_SOFT_OFF:
		__enable_irq();
		break;
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	/*
	 * System is now in active mode. Re-enable interrupts which were
	 * disabled when OS started idling code.
	 */
	irq_unlock(0);
}
#endif

#if defined(CONFIG_POWEROFF)
void z_sys_poweroff(void)
{
	__disable_irq();
	__set_BASEPRI(0);

	if (pm_core_enter_deep_sleep_request_subsys_off() == -EBUSY) {
		LOG_DBG("Subsystem did not go to OFF state");
	}

	CODE_UNREACHABLE;
}
#endif
