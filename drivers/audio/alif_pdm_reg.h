/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_AUDIO_ALIF_PDM_REG_H_
#define ZEPHYR_DRIVERS_AUDIO_ALIF_PDM_REG_H_

#define PDM_CONFIG_REGISTER       (0x0)  /* PDM Audio Control Register 0  */
#define PDM_CTL_REGISTER          (0x4)  /* PDM Audio Control Register 1  */
#define PDM_THRESHOLD_REGISTER    (0x8)  /* FIFO Watermark Register    */
#define PDM_FIFO_STATUS_REGISTER  (0xC)  /* FIFO Status Register      */
#define PDM_ERROR_IRQ             (0x10) /* FIFO Error Interrupt Status Register  */
#define PDM_WARN_IRQ              (0x14) /* FIFO Warning Interrupt Status Register*/
#define PDM_AUDIO_DETECT_IRQ      (0x18) /* Audio Detection Interrupt Status Register */
#define PDM_INTERRUPT_REGISTER    (0x1C) /* Interrupt Enable Register  */
#define PDM_CH0_CH1_AUDIO_OUT     (0x20) /* Channels 0 and 1 Audio Output Register  */
#define PDM_CH2_CH3_AUDIO_OUT     (0x24) /* Channels 2 and 3 Audio Output Register  */
#define PDM_CH4_CH5_AUDIO_OUT     (0x28) /* Channels 4 and 5 Audio Output Register  */
#define PDM_CH6_CH7_AUDIO_OUT     (0x2C) /* Channels 6 and 7 Audio Output Register  */
#define PDM_CH_FIR_COEF           (0x40) /* Channel (n) FIR Filter Coefficient  */
#define PDM_CH_IIR_COEF_SEL       (0xC0) /* Channel (n) IIR Filter Coefficient  */
#define PDM_CH_PHASE              (0xC4) /* Channel (n) Phase Control Register  */
#define PDM_CH_GAIN               (0xC8) /* Channel (n) Gain Control Register  */
#define PDM_CH_PKDET_TH           (0xCC) /* Channel (n) Peak Detector Threshold Register  */
#define PDM_CH_PKDET_ITV          (0xD0) /* Channel (n) Peak Detector Interval Register  */

#define PDM_FIFO_CLEAR            (1U << 31U)   /* To clear FIFO clear bit  */
#define PDM_AUDIO_DETECT_IRQ_STAT (0xFFU << 8U) /* Audio detect interrupt  */
#define PDM_FIFO_ALMOST_FULL_IRQ  (0x1U << 0U)  /* FIFO almost full Interrupt*/
#define PDM_FIFO_OVERFLOW_IRQ     (0x1U << 1U)  /* FIFO overflow interrupt   */
#define PDM_BYPASS_IIR            (2U)    /* Bypass DC blocking IIR filter*/
#define PDM_CHANNEL_ENABLE        (0xFFU)       /* To check the which channel is enabled*/

#define PDM_CLK_MODE              (16U)  /* PDM clock frequency mode  */
#define MAX_DATA_ITEMS            (8U)   /* Max data items      */
#define MAX_NUM_CHANNELS          (8U)   /* Max number of channel   */
#define MAX_QUEUE_LEN             (100U) /* Max Queue length      */
#define PDM_CH_OFFSET             (0x100U)

#define PDM_CHANNEL_0             (1U << 0U)
#define PDM_CHANNEL_1             (1U << 1U)
#define PDM_CHANNEL_2             (1U << 2U)
#define PDM_CHANNEL_3             (1U << 3U)
#define PDM_CHANNEL_4             (1U << 4U)
#define PDM_CHANNEL_5             (1U << 5U)
#define PDM_CHANNEL_6             (1U << 6U)
#define PDM_CHANNEL_7             (1U << 7U)

#endif /* ZEPHYR_DRIVERS_AUDIO_ALIF_PDM_REG_H_ */
