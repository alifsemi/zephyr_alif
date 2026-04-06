/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_ENSEMBLE_GEN2_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_ENSEMBLE_GEN2_H_

/*
 * AIPM DT binding constants for Ensemble Gen2 SoCs (E4/E6/E8).
 * Includes memory block bitmasks and EWIC event masks.
 * Values must stay in sync with the CONFIG_ENSEMBLE_GEN2 branch of aipm.h.
 */

#include <dt-bindings/misc/alif_aipm_common.h>

/* ------------------------------------------------------------------ */
/* Memory block bitmasks (run_profile_t.memory_blocks)                 */
/* ------------------------------------------------------------------ */
#define ALIF_SRAM0_MASK          (1 << 0)
#define ALIF_SRAM1_MASK          (1 << 1)
#define ALIF_SRAM2_MASK          (1 << 2)
#define ALIF_SRAM3_MASK          (1 << 3)
#define ALIF_SRAM4_1_MASK        (1 << 4)   /* M55-HE ITCM RET1 128kb */
#define ALIF_SRAM4_2_MASK        (1 << 5)   /* M55-HE ITCM RET2 128kb */
#define ALIF_SRAM5_1_MASK        (1 << 6)   /* M55-HE DTCM RET1 128kb */
#define ALIF_SRAM5_2_MASK        (1 << 7)   /* M55-HE DTCM RET2 128kb */
#define ALIF_SRAM6A_MASK         (1 << 8)
#define ALIF_SRAM6B_MASK         (1 << 9)
#define ALIF_SRAM7_1_MASK        (1 << 10)
#define ALIF_SRAM7_2_MASK        (1 << 11)
#define ALIF_SRAM7_3_MASK        (1 << 12)
#define ALIF_SRAM8_MASK          (1 << 13)
#define ALIF_SRAM9_MASK          (1 << 14)
#define ALIF_MRAM_MASK           (1 << 15)
#define ALIF_OSPI0_MASK          (1 << 16)
#define ALIF_OSPI1_MASK          (1 << 17)
#define ALIF_SERAM_1_MASK        (1 << 18)
#define ALIF_SERAM_2_MASK        (1 << 19)
#define ALIF_FWRAM_MASK          (1 << 20)
#define ALIF_BACKUP4K_MASK       (1 << 21)
#define ALIF_SRAM0_1_RET_MASK    (1 << 22)
#define ALIF_SRAM0_2_RET_MASK    (1 << 23)
#define ALIF_SRAM0_3_RET_MASK    (1 << 24)
#define ALIF_SRAM0_4_RET_MASK    (1 << 25)
#define ALIF_SRAM1_RET_MASK      (1 << 26)

#define ALIF_SERAM_MASK          (ALIF_SERAM_1_MASK | ALIF_SERAM_2_MASK)

/* ------------------------------------------------------------------ */
/* EWIC event masks (off_profile_t.ewic_cfg)                           */
/* ------------------------------------------------------------------ */
#define ALIF_EWIC_RTC_SE             (1 << 0)
#define ALIF_EWIC_LPGPIO             (3 << 4)
#define ALIF_EWIC_RTC_A              (1 << 6)
#define ALIF_EWIC_VBAT_TIMER         (0xF << 7)
#define ALIF_EWIC_VBAT_GPIO          (0xFF << 11)
#define ALIF_EWIC_VBAT_LP_CMP_IRQ    (1 << 19)
#define ALIF_EWIC_ES1_LP_I2C_IRQ     (1 << 20)
#define ALIF_EWIC_ES1_LP_UART_IRQ    (1 << 21)
#define ALIF_EWIC_BROWN_OUT          (1 << 22)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MISC_ALIF_AIPM_ENSEMBLE_GEN2_H_ */
