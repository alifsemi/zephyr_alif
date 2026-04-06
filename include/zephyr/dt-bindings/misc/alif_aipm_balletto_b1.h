/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_BALLETTO_B1_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_BALLETTO_B1_H_

/*
 * AIPM DT binding constants for Balletto B1 SoC.
 * Includes memory block bitmasks and EWIC event masks.
 * Values must stay in sync with the CONFIG_SOC_SERIES_B1 branch of aipm.h.
 */

#include <dt-bindings/misc/alif_aipm_common.h>

/* ------------------------------------------------------------------ */
/* Memory block bitmasks (run_profile_t.memory_blocks)                 */
/* ------------------------------------------------------------------ */
#define ALIF_SRAM2_MASK      (1 << 0)
#define ALIF_SRAM3_MASK      (1 << 1)
#define ALIF_SRAM4_1_MASK    (1 << 2)   /* M55-HE ITCM RET1 64kb */
#define ALIF_SRAM4_2_MASK    (1 << 3)   /* M55-HE ITCM RET2 64kb */
#define ALIF_SRAM4_3_MASK    (1 << 4)   /* M55-HE ITCM RET3 128kb */
#define ALIF_SRAM4_4_MASK    (1 << 5)   /* M55-HE ITCM RET4 256kb */
#define ALIF_SRAM5_1_MASK    (1 << 6)   /* M55-HE DTCM RET1 64kb */
#define ALIF_SRAM5_2_MASK    (1 << 7)   /* M55-HE DTCM RET2 64kb */
#define ALIF_SRAM5_3_MASK    (1 << 8)   /* M55-HE DTCM RET3 128kb */
#define ALIF_SRAM5_4_MASK    (1 << 9)   /* M55-HE DTCM RET4 256kb */
#define ALIF_SRAM5_5_MASK    (1 << 10)  /* M55-HE DTCM RET5 1024kb */
#define ALIF_MRAM_MASK       (1 << 11)
#define ALIF_OSPI0_MASK      (1 << 12)
#define ALIF_OSPI1_MASK      (1 << 13)
#define ALIF_SERAM_1_MASK    (1 << 14)
#define ALIF_SERAM_2_MASK    (1 << 15)
#define ALIF_SERAM_3_MASK    (1 << 16)
#define ALIF_SERAM_4_MASK    (1 << 17)
#define ALIF_FWRAM_MASK      (1 << 18)
#define ALIF_BACKUP4K_MASK   (1 << 19)

#define ALIF_SERAM_MASK      (ALIF_SERAM_1_MASK | ALIF_SERAM_2_MASK | \
			      ALIF_SERAM_3_MASK | ALIF_SERAM_4_MASK)

/* ------------------------------------------------------------------ */
/* EWIC event masks (off_profile_t.ewic_cfg)                           */
/* ------------------------------------------------------------------ */
#define ALIF_EWIC_RTC_SE             (1 << 0)
#define ALIF_EWIC_ES0_WAKEUP         (1 << 1)
#define ALIF_EWIC_ES0_OSC_EN         (1 << 2)
#define ALIF_EWIC_ES0_RADIO_EN       (1 << 3)
#define ALIF_EWIC_RTC_A              (1 << 4)
#define ALIF_EWIC_VBAT_TIMER         (0x1E << 4)
#define ALIF_EWIC_VBAT_GPIO          (0x1FE << 8)
#define ALIF_EWIC_VBAT_LP_CMP_IRQ    (1 << 17)
#define ALIF_EWIC_ES1_LP_I2C_IRQ     (1 << 18)
#define ALIF_EWIC_ES1_LP_UART_IRQ    (1 << 19)
#define ALIF_EWIC_BROWN_OUT          (1 << 20)
#define ALIF_EWIC_RTC_B              (1 << 21)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_BALLETTO_B1_H_ */
