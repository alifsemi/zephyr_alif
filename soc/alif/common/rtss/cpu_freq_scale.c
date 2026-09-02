/*
 * Copyright (c) 2026 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/cpu_freq/cpu_freq.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/clock_control_alif.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <se_service.h>

#if defined(CONFIG_SOC_FAMILY_BALLETTO)
#include <zephyr/dt-bindings/clock/alif_balletto_clocks.h>
#elif defined(CONFIG_SOC_SERIES_E1C)
#include <zephyr/dt-bindings/clock/alif_ensemble_e1c_clocks.h>
#else
#include <zephyr/dt-bindings/clock/alif_ensemble_clocks.h>
#endif

#include <zephyr/dt-bindings/misc/alif_aipm_common.h>

/* Governor timer ISR must not call this hook (device PM, SE). */
BUILD_ASSERT(!IS_ENABLED(CONFIG_CPU_FREQ),
	     "ALIF_CPU_FREQ_PSTATE is not IRQ-safe; leave CONFIG_CPU_FREQ disabled");

LOG_MODULE_REGISTER(cpu_freq_scale, CONFIG_SOC_LOG_LEVEL);

#define RUN_DEF DT_NODELABEL(aipm_run_default)

#define _ALIF_DT_CAT_DCDC(t)     _ALIF_DT_DCDC_##t
#define ALIF_DT_DCDC_MODE(t)     _ALIF_DT_CAT_DCDC(t)
#define _ALIF_DT_DCDC_off        DCDC_MODE_OFF
#define _ALIF_DT_DCDC_pfm_auto   DCDC_MODE_PFM_AUTO
#define _ALIF_DT_DCDC_pfm_forced DCDC_MODE_PFM_FORCED
#define _ALIF_DT_DCDC_pwm        DCDC_MODE_PWM

#define _ALIF_DT_CAT_AON_CLK(t)  _ALIF_DT_AON_CLK_##t
#define ALIF_DT_AON_CLK(t)       _ALIF_DT_CAT_AON_CLK(t)
#define _ALIF_DT_AON_CLK_lfrc    CLK_SRC_LFRC
#define _ALIF_DT_AON_CLK_lfxo    CLK_SRC_LFXO

#define _ALIF_DT_CAT_RUN_CLK(t)  _ALIF_DT_RUN_CLK_##t
#define ALIF_DT_RUN_CLK(t)       _ALIF_DT_CAT_RUN_CLK(t)
#define _ALIF_DT_RUN_CLK_hfrc    CLK_SRC_HFRC
#define _ALIF_DT_RUN_CLK_hfxo    CLK_SRC_HFXO
#define _ALIF_DT_RUN_CLK_pll     CLK_SRC_PLL

#define ALIF_CPU_CLK_HZ(freq)							\
	((freq) == ALIF_CLOCK_FREQ_400MHZ ? 400000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_300MHZ ? 300000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_200MHZ ? 200000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_160MHZ ? 160000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_120MHZ ? 120000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_100MHZ ? 100000000U :			\
	 (freq) == ALIF_CLOCK_FREQ_80MHZ  ? 80000000U :				\
	 (freq) == ALIF_CLOCK_FREQ_60MHZ  ? 60000000U :				\
	 (freq) == ALIF_CLOCK_FREQ_76_8_RC_MHZ ? 76800000U :			\
	 (freq) == ALIF_CLOCK_FREQ_38_4_RC_MHZ ? 38400000U :			\
	 (freq) == ALIF_CLOCK_FREQ_76_8_XO_MHZ ? 76800000U :			\
	 (freq) == ALIF_CLOCK_FREQ_38_4_XO_MHZ ? 38400000U : 0U)

#define ALIF_SCALED_CLK_HZ(freq)						\
	((freq) == ALIF_SCALED_FREQ_RC_ACTIVE_76_8_MHZ ? 76800000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_38_4_MHZ ? 38400000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_19_2_MHZ ? 19200000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_9_6_MHZ  ? 9600000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_4_8_MHZ  ? 4800000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_2_4_MHZ  ? 2400000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_1_2_MHZ  ? 1200000U :	\
	 (freq) == ALIF_SCALED_FREQ_RC_ACTIVE_0_6_MHZ  ? 600000U :	\
	 (freq) == ALIF_SCALED_FREQ_XO_LOW_DIV_38_4_MHZ ? 38400000U :	\
	 (freq) == ALIF_SCALED_FREQ_XO_LOW_DIV_19_2_MHZ ? 19200000U :	\
	 (freq) == ALIF_SCALED_FREQ_XO_HIGH_DIV_38_4_MHZ ? 38400000U :	\
	 (freq) == ALIF_SCALED_FREQ_XO_HIGH_DIV_19_2_MHZ ? 19200000U :	\
	 0U)

#if defined(CONFIG_SOC_FAMILY_BALLETTO)
#define ALIF_CLKID_IS_CPU(id)							\
	((id) == ALIF_LPUART_CLK || (id) == ALIF_LPSPI_CLK ||			\
	 (id) == ALIF_HCI_AHI_CLK || (id) == ALIF_CANFD0_160M_CLK ||		\
	 (id) == ALIF_CANFD1_160M_CLK)
#elif defined(CONFIG_SOC_SERIES_E1C)
#define ALIF_CLKID_IS_CPU(id)							\
	((id) == ALIF_LPUART_CLK || (id) == ALIF_LPSPI_CLK ||			\
	 (id) == ALIF_CANFD0_160M_CLK || (id) == ALIF_CANFD1_160M_CLK)
#else
#define ALIF_CLKID_IS_CPU(id)							\
	((id) == ALIF_LPUART_CLK || (id) == ALIF_LPSPI_CLK ||			\
	 (id) == ALIF_LPI3C_CLK || (id) == ALIF_LPI2C1_CLK ||			\
	 (id) == ALIF_LPUTIMER_CLK || (id) == ALIF_CANFD0_160M_CLK)
#endif

#define ALIF_NODE_USES_CPU_CLK(node)						\
	(ALIF_CLKID_IS_CPU(DT_CLOCKS_CELL_BY_IDX(node, 0, clkid)) ||		\
	 COND_CODE_1(DT_CLOCKS_HAS_IDX(node, 1),				\
		     (ALIF_CLKID_IS_CPU(DT_CLOCKS_CELL_BY_IDX(node, 1,		\
							     clkid))),		\
		     (0)))

struct alif_pstate_cfg {
	run_profile_t profile;
	uint32_t cpu_hz;
};

struct alif_clk_dep {
	const struct device *dev;
	bool uses_cpu_clk;
};

#define ALIF_CLK_DEP_IF_CTLR(node)						\
	COND_CODE_1(IS_EQ(DT_DEP_ORD(DT_CLOCKS_CTLR(node)),			\
			  DT_DEP_ORD(DT_NODELABEL(clockctrl))),			\
		    ({ .dev = DEVICE_DT_GET(node),				\
		       .uses_cpu_clk = ALIF_NODE_USES_CPU_CLK(node), },),	\
		    ())

#define ALIF_CLK_DEP_ENTRY(node)						\
	COND_CODE_1(DT_NODE_HAS_PROP(node, clocks),				\
		    (ALIF_CLK_DEP_IF_CTLR(node)), ())

/* Okay + clockctrl does not imply a driver. Weak so missing
 * DEVICE_DT_DEFINE (cdc200, dphy, …) resolves to NULL.
 */
#define ALIF_CLK_DEP_DECLARE_CTLR(node)						\
	COND_CODE_1(IS_EQ(DT_DEP_ORD(DT_CLOCKS_CTLR(node)),			\
			  DT_DEP_ORD(DT_NODELABEL(clockctrl))),			\
		    (extern const struct device DEVICE_DT_NAME_GET(node)	\
			     __weak;),						\
		    ())

#define ALIF_CLK_DEP_DECLARE(node)						\
	COND_CODE_1(DT_NODE_HAS_PROP(node, clocks),				\
		    (ALIF_CLK_DEP_DECLARE_CTLR(node)), ())

DT_FOREACH_STATUS_OKAY_NODE(ALIF_CLK_DEP_DECLARE)

static const struct alif_clk_dep clk_deps[] = {
	DT_FOREACH_STATUS_OKAY_NODE(ALIF_CLK_DEP_ENTRY)
	{ NULL, false },
};

#define ALIF_PSTATE_CFG_SYM(node) _CONCAT(__alif_pstate_cfg_, DT_DEP_ORD(node))

#define ALIF_PSTATE_RUN_CLK_SRC(node)						\
	COND_CODE_1(DT_NODE_HAS_PROP(node, clk_src),				\
		    (ALIF_DT_RUN_CLK(DT_STRING_TOKEN(node, clk_src))),		\
		    (ALIF_DT_RUN_CLK(DT_STRING_TOKEN(RUN_DEF, clk_src))))

#define ALIF_PSTATE_SCALED_FREQ(node)						\
	COND_CODE_1(DT_NODE_HAS_PROP(node, scaled_clk_freq),			\
		    (DT_PROP(node, scaled_clk_freq)),				\
		    (DT_PROP(RUN_DEF, scaled_clk_freq)))

/* HFRC/HFXO EXTSYS follows scaled-clk-freq. Enum 0 is 76.8 RC Active. */
#define ALIF_PSTATE_CPU_HZ(node)						\
	(((ALIF_PSTATE_RUN_CLK_SRC(node) != CLK_SRC_PLL) &&			\
	  (ALIF_PSTATE_SCALED_FREQ(node) != 0)) ?				\
	 ALIF_SCALED_CLK_HZ(ALIF_PSTATE_SCALED_FREQ(node)) :			\
	 ALIF_CPU_CLK_HZ(DT_PROP(node, cpu_clk_freq)))

#define ALIF_PSTATE_PROFILE(node) {						\
	.power_domains = DT_PROP(RUN_DEF, aipm_power_domains),			\
	.dcdc_voltage = COND_CODE_1(						\
		DT_NODE_HAS_PROP(node, dcdc_voltage),				\
		(DT_PROP(node, dcdc_voltage)),					\
		(DT_PROP(RUN_DEF, dcdc_voltage))),				\
	.dcdc_mode = COND_CODE_1(						\
		DT_NODE_HAS_PROP(node, dcdc_mode),				\
		(ALIF_DT_DCDC_MODE(DT_STRING_TOKEN(node, dcdc_mode))),		\
		(ALIF_DT_DCDC_MODE(DT_STRING_TOKEN(RUN_DEF, dcdc_mode)))),	\
	.aon_clk_src = ALIF_DT_AON_CLK(DT_STRING_TOKEN(RUN_DEF, aon_clk_src)),	\
	.run_clk_src = ALIF_PSTATE_RUN_CLK_SRC(node),				\
	.cpu_clk_freq = (clock_frequency_t)DT_PROP(node, cpu_clk_freq),		\
	.scaled_clk_freq = (scaled_clk_freq_t)ALIF_PSTATE_SCALED_FREQ(node),	\
	.memory_blocks = DT_PROP(RUN_DEF, memory_blocks),			\
	.ip_clock_gating = DT_PROP_OR(RUN_DEF, ip_clock_gating, 0),		\
	.phy_pwr_gating = DT_PROP_OR(RUN_DEF, phy_pwr_gating, 0),		\
	.vdd_ioflex_3V3 = (ioflex_mode_t)DT_PROP_OR(				\
		RUN_DEF, vdd_ioflex, ALIF_IOFLEX_LEVEL_1V8),			\
}

#define ALIF_DEFINE_PSTATE(node)						\
	static const struct alif_pstate_cfg ALIF_PSTATE_CFG_SYM(node) = {	\
		.profile = ALIF_PSTATE_PROFILE(node),				\
		.cpu_hz = ALIF_PSTATE_CPU_HZ(node),				\
	};									\
	PSTATE_DT_DEFINE(node, &ALIF_PSTATE_CFG_SYM(node));

DT_FOREACH_CHILD_STATUS_OKAY(DT_PATH(performance_states), ALIF_DEFINE_PSTATE)

static bool pm_ignore(int err)
{
	return err == 0 || err == -ENOSYS || err == -ENOTSUP || err == -EALREADY;
}

static void dependents_resume(const struct device *const *suspended, size_t n)
{
	int err;

	while (n > 0U) {
		n--;
		err = pm_device_action_run(suspended[n], PM_DEVICE_ACTION_RESUME);
		if (!pm_ignore(err)) {
			LOG_ERR("pm resume %s failed (%d)",
				suspended[n]->name, err);
		}
	}
}

/* Suspend clock dependents. -EALREADY means the app already parked the
 * device; do not resume it later. On failure, resume only what this
 * call suspended (reverse order).
 */
static int dependents_suspend(bool buses_move, const struct device **suspended,
			      size_t *n_suspended)
{
	int err;
	size_t i, n = 0;

	for (i = 0; i < ARRAY_SIZE(clk_deps); i++) {
		if (clk_deps[i].dev == NULL || !device_is_ready(clk_deps[i].dev)) {
			continue;
		}
		if (!buses_move && !clk_deps[i].uses_cpu_clk) {
			continue;
		}

		/* Same busy bit system PM uses. Abort: clocks must not
		 * move under an in-flight transfer
		 */
		if (pm_device_is_busy(clk_deps[i].dev)) {
			LOG_ERR("pm suspend %s busy", clk_deps[i].dev->name);
			dependents_resume(suspended, n);
			*n_suspended = 0U;
			return -EBUSY;
		}

		err = pm_device_action_run(clk_deps[i].dev,
					   PM_DEVICE_ACTION_SUSPEND);
		if (err == -ENOSYS || err == -ENOTSUP || err == -EALREADY) {
			continue;
		}
		if (err) {
			LOG_ERR("pm suspend %s failed (%d)",
				clk_deps[i].dev->name, err);
			dependents_resume(suspended, n);
			*n_suspended = 0U;
			return err;
		}
		suspended[n++] = clk_deps[i].dev;
	}

	*n_suspended = n;
	return 0;
}

static bool buses_move_from_last(const run_profile_t *dest)
{
	run_profile_t last;
	int err;

	err = se_service_get_last_set_run_cfg(&last);
	if (err) {
		/* No cache yet: treat as a bus change so dependents re-init. */
		return true;
	}

	return (last.run_clk_src != dest->run_clk_src) ||
	       (last.scaled_clk_freq != dest->scaled_clk_freq);
}

int cpu_freq_pstate_set(const struct pstate *state)
{
	const struct alif_pstate_cfg *cfg;
	const struct device *suspended[ARRAY_SIZE(clk_deps)];
	run_profile_t profile;
	size_t n_suspended = 0U;
	int err, key;
	bool buses_move;

	if (state == NULL || state->config == NULL) {
		return -EINVAL;
	}

	cfg = state->config;
	if (cfg->cpu_hz == 0U) {
		LOG_ERR("unsupported cpu-clk-freq");
		return -EINVAL;
	}

	profile = cfg->profile;
	buses_move = buses_move_from_last(&profile);

	err = dependents_suspend(buses_move, suspended, &n_suspended);
	if (err) {
		return err;
	}

	key = irq_lock();
	err = se_service_set_run_cfg_poll(&profile);
	if (err == 0) {
		if (buses_move) {
			alif_clock_sys_clk_cache_invalidate();
		} else {
			alif_clock_sys_clk_cache_invalidate_cpu();
		}
		z_sys_clock_hw_cycles_per_sec_update(cfg->cpu_hz);
	}
	irq_unlock(key);

	dependents_resume(suspended, n_suspended);

	if (err) {
		LOG_ERR("set_run_cfg_poll failed (%d)", err);
		return err;
	}

	LOG_DBG("pstate applied, cpu_hz=%u buses_move=%d", cfg->cpu_hz,
		buses_move);
	return 0;
}
