/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_COMMON_H_

/*
 * Common DT binding constants for Alif AIPM (run_profile_t / off_profile_t).
 * These values are SoC-invariant. SoC-specific constants (memory blocks, EWIC)
 * are in the per-SoC headers:
 *   alif_aipm_ensemble.h        — base Ensemble (E1/E3/E5/E7)
 *   alif_aipm_ensemble_gen2.h   — Ensemble Gen2 (E4/E6/E8)
 *   alif_aipm_ensemble_e1c.h    — Ensemble E1C
 *   alif_aipm_balletto_b1.h     — Balletto B1
 *
 * All values must stay in sync with aipm.h.
 */

/* ------------------------------------------------------------------ */
/* Power domain bitmasks (run_profile_t.power_domains)                 */
/* Bit positions match ALIF_PD_* in alif_power_domain.h               */
/* ------------------------------------------------------------------ */
#define ALIF_PD_VBAT_AON_MASK       (1 << 0)
#define ALIF_PD_SRAM_CTRL_AON_MASK  (1 << 1)
#define ALIF_PD_SSE700_AON_MASK     (1 << 2)
#define ALIF_PD_RTSS_HE_MASK        (1 << 3)
#define ALIF_PD_SRAMS_MASK          (1 << 4)
#define ALIF_PD_SESS_MASK           (1 << 5)
#define ALIF_PD_SYST_MASK           (1 << 6)
#define ALIF_PD_RTSS_HP_MASK        (1 << 7)
#define ALIF_PD_DBSS_MASK           (1 << 8)
#define ALIF_PD_APPS_MASK           (1 << 9)

/* ------------------------------------------------------------------ */
/* CPU clock frequency (run_profile_t.cpu_clk_freq = clock_frequency_t) */
/* Non-contiguous enum — do NOT cast DTS integer directly.             */
/* ------------------------------------------------------------------ */
#define ALIF_CLOCK_FREQ_800MHZ          0   /* Application CPU */
#define ALIF_CLOCK_FREQ_400MHZ          1
#define ALIF_CLOCK_FREQ_300MHZ          2
#define ALIF_CLOCK_FREQ_200MHZ          3
#define ALIF_CLOCK_FREQ_160MHZ          4
#define ALIF_CLOCK_FREQ_120MHZ          5
#define ALIF_CLOCK_FREQ_80MHZ           6
#define ALIF_CLOCK_FREQ_60MHZ           7
#define ALIF_CLOCK_FREQ_100MHZ          8   /* Peripheral clocks */
#define ALIF_CLOCK_FREQ_50MHZ           9
#define ALIF_CLOCK_FREQ_20MHZ           10
#define ALIF_CLOCK_FREQ_10MHZ           11
#define ALIF_CLOCK_FREQ_76_8_RC_MHZ     12  /* RC and XO clocks */
#define ALIF_CLOCK_FREQ_38_4_RC_MHZ     13
#define ALIF_CLOCK_FREQ_76_8_XO_MHZ     14
#define ALIF_CLOCK_FREQ_38_4_XO_MHZ     15
#define ALIF_CLOCK_FREQ_DISABLED        16

/* ------------------------------------------------------------------ */
/* Scaled clock frequency (run_profile_t.scaled_clk_freq)              */
/* ------------------------------------------------------------------ */
#define ALIF_SCALED_FREQ_RC_ACTIVE_76_8_MHZ     0   /* HFRC — ACTIVE mode */
#define ALIF_SCALED_FREQ_RC_ACTIVE_38_4_MHZ     1
#define ALIF_SCALED_FREQ_RC_ACTIVE_19_2_MHZ     2
#define ALIF_SCALED_FREQ_RC_ACTIVE_9_6_MHZ      3
#define ALIF_SCALED_FREQ_RC_ACTIVE_4_8_MHZ      4
#define ALIF_SCALED_FREQ_RC_ACTIVE_2_4_MHZ      5
#define ALIF_SCALED_FREQ_RC_ACTIVE_1_2_MHZ      6
#define ALIF_SCALED_FREQ_RC_ACTIVE_0_6_MHZ      7

#define ALIF_SCALED_FREQ_RC_STDBY_76_8_MHZ      8   /* HFRC — STANDBY mode */
#define ALIF_SCALED_FREQ_RC_STDBY_38_4_MHZ      9
#define ALIF_SCALED_FREQ_RC_STDBY_19_2_MHZ      10
#define ALIF_SCALED_FREQ_RC_STDBY_4_8_MHZ       11
#define ALIF_SCALED_FREQ_RC_STDBY_1_2_MHZ       12
#define ALIF_SCALED_FREQ_RC_STDBY_0_6_MHZ       13
#define ALIF_SCALED_FREQ_RC_STDBY_0_3_MHZ       14
#define ALIF_SCALED_FREQ_RC_STDBY_0_075_MHZ     15

#define ALIF_SCALED_FREQ_XO_LOW_DIV_38_4_MHZ    16  /* HFXO — LOW divider */
#define ALIF_SCALED_FREQ_XO_LOW_DIV_19_2_MHZ    17
#define ALIF_SCALED_FREQ_XO_LOW_DIV_9_6_MHZ     18
#define ALIF_SCALED_FREQ_XO_LOW_DIV_4_8_MHZ     19
#define ALIF_SCALED_FREQ_XO_LOW_DIV_2_4_MHZ     20
#define ALIF_SCALED_FREQ_XO_LOW_DIV_1_2_MHZ     21
#define ALIF_SCALED_FREQ_XO_LOW_DIV_0_6_MHZ     22
#define ALIF_SCALED_FREQ_XO_LOW_DIV_0_3_MHZ     23

#define ALIF_SCALED_FREQ_XO_HIGH_DIV_38_4_MHZ   24  /* HFXO — HIGH divider */
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_19_2_MHZ   25
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_9_6_MHZ    26
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_2_4_MHZ    27
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_0_6_MHZ    28
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_0_3_MHZ    29
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_0_15_MHZ   30
#define ALIF_SCALED_FREQ_XO_HIGH_DIV_0_0375_MHZ 31

#define ALIF_SCALED_FREQ_NONE                    32

/* ------------------------------------------------------------------ */
/* IP clock-gate bitmasks (run_profile_t.ip_clock_gating)              */
/* Mirror of ip_clock_gating_t masks in aipm.h                        */
/* ------------------------------------------------------------------ */
#define ALIF_IP_CLK_GATE_NPU_HP_MASK     (1 << 0)
#define ALIF_IP_CLK_GATE_NPU_HE_MASK     (1 << 1)
#define ALIF_IP_CLK_GATE_ISIM_MASK       (1 << 2)
#define ALIF_IP_CLK_GATE_OSPI_1_MASK     (1 << 3)
#define ALIF_IP_CLK_GATE_CANFD_MASK      (1 << 4)
#define ALIF_IP_CLK_GATE_SDC_MASK        (1 << 5)
#define ALIF_IP_CLK_GATE_USB_MASK        (1 << 6)
#define ALIF_IP_CLK_GATE_ETH_MASK        (1 << 7)
#define ALIF_IP_CLK_GATE_GPU_MASK        (1 << 8)
#define ALIF_IP_CLK_GATE_CDC200_MASK     (1 << 9)
#define ALIF_IP_CLK_GATE_CAMERA_MASK     (1 << 10)
#define ALIF_IP_CLK_GATE_MIPI_DSI_MASK   (1 << 11)
#define ALIF_IP_CLK_GATE_MIPI_CSI_MASK   (1 << 12)
#define ALIF_IP_CLK_GATE_LP_PERIPH_MASK  (1 << 13)

/* ------------------------------------------------------------------ */
/* PHY power-gate bitmasks (run_profile_t.phy_pwr_gating)              */
/* Mirror of phy_gating_t masks in aipm.h                             */
/* ------------------------------------------------------------------ */
#define ALIF_PHY_GATE_LDO_MASK           (1 << 0)
#define ALIF_PHY_GATE_USB_MASK           (1 << 1)
#define ALIF_PHY_GATE_MIPI_TX_DPHY_MASK  (1 << 2)
#define ALIF_PHY_GATE_MIPI_RX_DPHY_MASK  (1 << 3)
#define ALIF_PHY_GATE_MIPI_PLL_DPHY_MASK (1 << 4)

/* ------------------------------------------------------------------ */
/* IOFLEX voltage level (run_profile_t.vdd_ioflex_3V3 = ioflex_mode_t) */
/* Note: IOFLEX_LEVEL_3V3 = 0, IOFLEX_LEVEL_1V8 = 1 in aipm.h        */
/* ------------------------------------------------------------------ */
#define ALIF_IOFLEX_LEVEL_3V3  0
#define ALIF_IOFLEX_LEVEL_1V8  1

/* ------------------------------------------------------------------ */
/* PD0 Wakeup events (off_profile_t.wakeup_events) — same for all SoCs */
/* ------------------------------------------------------------------ */
#define ALIF_WE_SERTC    (1 << 4)
#define ALIF_WE_LPRTC    (1 << 5)
#define ALIF_WE_LPCMP   (1 << 6)
#define ALIF_WE_BOD      (1 << 7)
#define ALIF_WE_LPTIMER0 (1 << 8)
#define ALIF_WE_LPTIMER1 (1 << 9)
#define ALIF_WE_LPTIMER2 (1 << 10)
#define ALIF_WE_LPTIMER3 (1 << 11)
#define ALIF_WE_LPGPIO0  (1 << 16)
#define ALIF_WE_LPGPIO1  (1 << 17)
#define ALIF_WE_LPGPIO2  (1 << 18)
#define ALIF_WE_LPGPIO3  (1 << 19)
#define ALIF_WE_LPGPIO4  (1 << 20)
#define ALIF_WE_LPGPIO5  (1 << 21)
#define ALIF_WE_LPGPIO6  (1 << 22)
#define ALIF_WE_LPGPIO7  (1 << 23)

#define ALIF_WE_LPTIMER  0x00000F00
#define ALIF_WE_LPGPIO   0x00FF0000

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_COMMON_H_ */
