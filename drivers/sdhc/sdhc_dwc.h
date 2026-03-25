/*
 * SPDX-FileCopyrightText: Copyright Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SDHC_SDHC_DWC_H_
#define ZEPHYR_DRIVERS_SDHC_SDHC_DWC_H_

#include "zephyr/drivers/sdhc.h"
#include <stdint.h>
#include <zephyr/sys/util.h>

/*
 * ============================================================
 * DWC SDHC Register Structure
 * ============================================================
 */
struct dwc_sdhc_regs {
	volatile uint32_t DWC_SDHC_SDMASA_R;              /* 0x000 SDMA System Address */
	volatile uint16_t DWC_SDHC_BLOCKSIZE_R;            /* 0x004 Block Size                   */
	volatile uint16_t DWC_SDHC_BLOCKCOUNT_R;           /* 0x006 16-bit Block Count           */
	volatile uint32_t DWC_SDHC_ARGUMENT_R;             /* 0x008 Argument                     */
	volatile uint16_t DWC_SDHC_XFER_MODE_R;            /* 0x00C Transfer Mode                */
	volatile uint16_t DWC_SDHC_CMD_R;                  /* 0x00E Command                      */

	volatile const uint32_t DWC_SDHC_RESP01_R;         /* 0x010 Response 0/1                 */
	volatile const uint32_t DWC_SDHC_RESP23_R;         /* 0x014 Response 2/3                 */
	volatile const uint32_t DWC_SDHC_RESP45_R;         /* 0x018 Response 4/5                 */
	volatile const uint32_t DWC_SDHC_RESP67_R;         /* 0x01C Response 6/7                 */
	volatile uint32_t DWC_SDHC_BUF_DATA_R;             /* 0x020 Buffer Data Port             */
	volatile const uint32_t DWC_SDHC_PSTATE_REG;       /* 0x024 Present State                */
	volatile uint8_t  DWC_SDHC_HOST_CTRL1_R;           /* 0x028 Host Control 1               */
	volatile uint8_t  DWC_SDHC_PWR_CTRL_R;             /* 0x029 Power Control                */
	volatile uint8_t  DWC_SDHC_BGAP_CTRL_R;            /* 0x02A Block Gap Control            */
	volatile uint8_t  DWC_SDHC_WUP_CTRL_R;             /* 0x02B Wakeup Control               */
	volatile uint16_t DWC_SDHC_CLK_CTRL_R;             /* 0x02C Clock Control                */
	volatile uint8_t  DWC_SDHC_TOUT_CTRL_R;            /* 0x02E Timeout Control              */
	volatile uint8_t  DWC_SDHC_SW_RST_R;               /* 0x02F Software Reset               */
	volatile uint16_t DWC_SDHC_NORMAL_INT_STAT_R;      /* 0x030 Normal Interrupt Status      */
	volatile uint16_t DWC_SDHC_ERROR_INT_STAT_R;       /* 0x032 Error Interrupt Status       */
	volatile uint16_t DWC_SDHC_NORMAL_INT_STAT_EN_R;   /* 0x034 Normal Int Status Enable     */
	volatile uint16_t DWC_SDHC_ERROR_INT_STAT_EN_R;    /* 0x036 Error Int Status Enable      */
	volatile uint16_t DWC_SDHC_NORMAL_INT_SIGNAL_EN_R; /* 0x038 Normal Int Signal Enable     */
	volatile uint16_t DWC_SDHC_ERROR_INT_SIGNAL_EN_R;  /* 0x03A Error Int Signal Enable      */
	volatile const uint16_t DWC_SDHC_AUTO_CMD_STAT_R;  /* 0x03C Auto CMD Status */
	volatile uint16_t DWC_SDHC_HOST_CTRL2_R;           /* 0x03E Host Control 2               */
	volatile const uint32_t DWC_SDHC_CAPABILITIES1_R;  /* 0x040 Capabilities 1 */
	volatile const uint32_t DWC_SDHC_CAPABILITIES2_R;  /* 0x044 Capabilities 2 */
	volatile const uint32_t DWC_SDHC_CURR_CAPABILITIES1_R; /* 0x048 Max Current Cap 1 */
	volatile const uint32_t RESERVED;
	volatile uint16_t DWC_SDHC_FORCE_AUTO_CMD_STAT_R;  /* 0x050 Force Auto CMD Error Status  */

	volatile uint16_t DWC_SDHC_FORCE_ERROR_INT_STAT_R; /* 0x052 Force Error Int Status       */
	volatile const uint8_t  DWC_SDHC_ADMA_ERR_STAT_R;  /* 0x054 ADMA Error Status */
	volatile const uint8_t  RESERVED1;
	volatile const uint16_t RESERVED2;
	volatile uint32_t DWC_SDHC_ADMA_SA_LOW_R;          /* 0x058 ADMA System Address (Low)    */
	volatile const uint32_t RESERVED3;
	volatile const uint16_t DWC_SDHC_PRESET_INIT_R;  /* 0x060 Preset Init */
	volatile const uint16_t DWC_SDHC_PRESET_DS_R;  /* 0x062 Preset Default Speed */
	volatile const uint16_t DWC_SDHC_PRESET_HS_R;  /* 0x064 Preset High Speed */
	volatile const uint16_t DWC_SDHC_PRESET_SDR12_R;  /* 0x066 Preset SDR12 */
	volatile const uint16_t DWC_SDHC_PRESET_SDR25_R;  /* 0x068 Preset SDR25 */
	volatile const uint16_t DWC_SDHC_PRESET_SDR50_R;  /* 0x06A Preset SDR50 */
	volatile const uint32_t RESERVED4[3];
	volatile uint32_t DWC_SDHC_ADMA_ID_LOW_R;          /* 0x078 ADMA3 Integrated Desc Addr   */
	volatile const uint32_t RESERVED5[27];
	volatile const uint16_t DWC_SDHC_P_VENDOR_SPECIFIC_AREA; /* 0x0E8 Vendor Specific Area */
	volatile const uint16_t RESERVED6;
	volatile const uint32_t RESERVED7[4];
	volatile const uint16_t DWC_SDHC_SLOT_INTR_STATUS_R; /* 0x0FC Slot Interrupt Status */
	volatile const uint16_t DWC_SDHC_HOST_CNTRL_VERS_R; /* 0x0FE Host Controller Version */
	volatile const uint32_t RESERVED8[256];
	volatile const uint32_t DWC_SDHC_MSHC_VER_ID_R;  /* 0x500 MSHC Version ID */
	volatile const uint32_t DWC_SDHC_MSHC_VER_TYPE_R;  /* 0x504 MSHC Version Type */
	volatile uint8_t  DWC_SDHC_MSHC_CTRL_R;            /* 0x508 MSHC Control                 */
	volatile const uint8_t  RESERVED9;
	volatile const uint16_t RESERVED10;
	volatile const uint32_t RESERVED11;
	volatile uint8_t  DWC_SDHC_MBIU_CTRL_R;            /* 0x510 MBIU Control                 */
	volatile const uint8_t  RESERVED12;
	volatile const uint16_t RESERVED13;
	volatile const uint32_t RESERVED14[6];
	volatile uint16_t DWC_SDHC_EMMC_CTRL_R;            /* 0x52C eMMC Control                 */
	volatile uint16_t DWC_SDHC_BOOT_CTRL_R;            /* 0x52E eMMC Boot Control            */
	volatile const uint32_t RESERVED15[655];
	volatile uint32_t DWC_SDHC_EMBEDDED_CTRL_R;        /* 0xF6C Embedded Control             */
};

/*
 * ============================================================
 * ADMA2 Descriptor
 * ============================================================
 */
typedef uint64_t adma2_desc_t;

#define DWC_SDHC_ADMA2_DESC_MAX_LEN          65536U
#define DWC_SDHC_ADMA2_DESC_VALID            BIT(0)
#define DWC_SDHC_ADMA2_DESC_END              BIT(1)
#define DWC_SDHC_ADMA2_DESC_INT              BIT(2)
#define DWC_SDHC_ADMA2_DESC_TRAN             BIT(5)

/* Build a 64-bit ADMA2 descriptor: attr[15:0] | len[31:16] | addr[63:32] */
#define DWC_SDHC_ADMA2_DESC(attr, len, addr) \
	(((uint64_t)(addr) << 32) | ((uint64_t)(len) << 16) | (uint64_t)(attr))

/*
 * ============================================================
 * Software Reset Register
 * ============================================================
 */
#define DWC_SDHC_SW_RST_ALL_Pos              0U
#define DWC_SDHC_SW_RST_ALL_Msk              BIT(DWC_SDHC_SW_RST_ALL_Pos)
#define DWC_SDHC_SW_RST_CMD_Pos              1U
#define DWC_SDHC_SW_RST_CMD_Msk              BIT(DWC_SDHC_SW_RST_CMD_Pos)
#define DWC_SDHC_SW_RST_DAT_Pos              2U
#define DWC_SDHC_SW_RST_DAT_Msk              BIT(DWC_SDHC_SW_RST_DAT_Pos)

/*
 * ============================================================
 * CMD Register
 * ============================================================
 */
#define DWC_SDHC_CMD_IDX_Pos                 8U
#define DWC_SDHC_CMD_R_CRC_CHK_EN_Msk        BIT(3)
#define DWC_SDHC_CMD_R_CMD_IDX_CHK_EN_Msk    0x10U
#define DWC_SDHC_CMD_R_DATA_PRES_SEL_Pos     5U
#define DWC_SDHC_CMD_R_DATA_PRES_SEL_Msk     BIT(DWC_SDHC_CMD_R_DATA_PRES_SEL_Pos)

/* Response Types (CMD register bits [1:0]) */
#define DWC_SDHC_RESP_NONE                   0U
#define DWC_SDHC_RESP_R136                   1U
#define DWC_SDHC_RESP_R48                    2U
#define DWC_SDHC_RESP_R48B                   3U

/*
 * ============================================================
 * Present State Register
 * ============================================================
 */
#define DWC_SDHC_CMD_INHIBIT_Msk             1U
#define DWC_SDHC_DAT_INHIBIT_Msk             2U
#define DWC_SDHC_CARD_INSRT_Msk              0x00010000U
#define DWC_SDHC_CMD_LINE_LVL_UP_Pos         23U
#define DWC_SDHC_CMD_LINE_LVL_UP_Msk         BIT(DWC_SDHC_CMD_LINE_LVL_UP_Pos)
#define DWC_SDHC_CMD_DATA_LINE_STATUS_Msk    0x01F00000U

/*
 * ===========================================================
 * Transfer Mode Register
 * ============================================================
 */
#define DWC_SDHC_XFER_MODE_DMA_EN_Pos        0U
#define DWC_SDHC_XFER_MODE_DMA_EN_Msk        BIT(DWC_SDHC_XFER_MODE_DMA_EN_Pos)
#define DWC_SDHC_XFER_MODE_BLK_CNT_EN_Pos    1U
#define DWC_SDHC_XFER_MODE_BLK_CNT_Msk       BIT(DWC_SDHC_XFER_MODE_BLK_CNT_EN_Pos)
#define DWC_SDHC_XFER_MODE_AUTO_CMD_EN_Pos   2U
#define DWC_SDHC_XFER_MODE_AUTO_CMD12        1U
#define DWC_SDHC_XFER_MODE_DATA_XFER_DIR_Pos 4U
#define DWC_SDHC_XFER_MODE_DATA_XFER_RD_Msk  BIT(DWC_SDHC_XFER_MODE_DATA_XFER_DIR_Pos)
#define DWC_SDHC_XFER_MODE_DATA_XFER_WR_Msk  0U
#define DWC_SDHC_XFER_MODE_MULTI_BLK_SEL_Pos 5U
#define DWC_SDHC_XFER_MODE_MULTI_BLK_SEL_Msk BIT(DWC_SDHC_XFER_MODE_MULTI_BLK_SEL_Pos)

/*
 * ============================================================
 * Power Control Register
 * ============================================================
 */
#define DWC_SDHC_PC_BUS_PWR_VDD1_Msk         0x00000001U
#define DWC_SDHC_PC_BUS_VSEL_3V3_Msk         0x0000000EU
#define DWC_SDHC_PC_BUS_VSEL_3V0_Msk         0x0000000CU
#define DWC_SDHC_PC_BUS_VSEL_1V8_Msk         0x0000000AU

/*
 * ============================================================
 * Clock Control Register
 * ============================================================
 */
#define DWC_SDHC_INTERNAL_CLK_EN_Msk         0x1U
#define DWC_SDHC_INTERNAL_CLK_STABLE_Msk     0x2U
#define DWC_SDHC_CLK_EN_Msk                  0x4U
#define DWC_SDHC_PLL_EN_Msk                  0x8U
#define DWC_SDHC_CLK_GEN_SEL_Pos             5U
#define DWC_SDHC_DIV_CLK_MODE                0x0U
#define DWC_SDHC_CLK_GEN_SEL_Msk             (DWC_SDHC_DIV_CLK_MODE << DWC_SDHC_CLK_GEN_SEL_Pos)
#define DWC_SDHC_UPPER_FREQ_SEL_Pos          6U
#define DWC_SDHC_FREQ_SEL_Pos                8U
#define DWC_SDHC_BASE_CLK_FREQ_Pos           7U
#define DWC_SDHC_BASE_CLK_FREQ_Msk           (0xFFU << DWC_SDHC_FREQ_SEL_Pos)
#define DWC_SDHC_DEFAULT_BASE_CLK_MHZ        100U
#define DWC_SDHC_CLK_STABLE_TIMEOUT_US       100000U

/* Clock Frequencies */
#define DWC_SDHC_CLK_400_KHZ                 400000U
#define DWC_SDHC_CLK_25_MHZ                  25000000U
#define DWC_SDHC_CLK_50_MHZ                  50000000U
#define DWC_SDHC_CLK_100_MHZ                 100000000U

/*
 * ============================================================
 * Host Control 1 Register
 * ============================================================
 */
#define DWC_SDHC_HOST_CTRL1_DATA_WIDTH_4BIT_Msk BIT(1)
#define DWC_SDHC_HOST_CTRL1_DMA_SEL_Msk         (0x3U << 3U)
#define DWC_SDHC_HOST_CTRL1_DMA_SEL_SDMA        (0x0U << 3U)
#define DWC_SDHC_HOST_CTRL1_DMA_SEL_ADMA32      (0x2U << 3U)
#define DWC_SDHC_HOST_CTRL1_HS_EN_Msk           BIT(2)
#define DWC_SDHC_HOST_CTRL1_EXT_DATA_WIDTH_Msk  BIT(5)

/*
 * ============================================================
 * Host Control 2 Register
 * ============================================================
 */
#define DWC_SDHC_HOST_CTRL2_UHS_MODE_Msk     (0x07U << 0U)
#define DWC_SDHC_HOST_CTRL2_UHS_MODE_SDR25   (0x01U << 0U)
#define DWC_SDHC_HOST_CTRL2_UHS_MODE_SDR50   (0x02U << 0U)
#define DWC_SDHC_HOST_CTRL2_UHS_MODE_SDR104  (0x03U << 0U)
#define DWC_SDHC_HOST_CTRL2_SIGNALING_EN_Msk BIT(3)
#define DWC_SDHC_HOST_CTRL2_EXEC_TUNING_Msk  BIT(6)
#define DWC_SDHC_HOST_CTRL2_SAMPLING_CLK_Msk BIT(7)
#define DWC_SDHC_HOST_CTRL2_VER4_EN_Msk      BIT(12)
#define DWC_SDHC_HOST_CTRL2_ASYNC_INT_EN_Msk BIT(14)

/*
 * ============================================================
 * Wakeup Control Register
 * ============================================================
 */
#define DWC_SDHC_WKUP_CARD_IRQ_Msk           0x00000001U
#define DWC_SDHC_WKUP_CARD_INSRT_Msk         0x00000002U
#define DWC_SDHC_WKUP_CARD_REM_Msk           0x00000004U

/*
 * ============================================================
 * Interrupt Masks
 * ============================================================
 */
/* Normal Interrupts */
#define DWC_SDHC_INTR_CC_Msk                0x0001U
#define DWC_SDHC_INTR_TC_Msk                0x0002U
#define DWC_SDHC_INTR_BGE_Msk               0x0004U
#define DWC_SDHC_INTR_DMA_Msk               0x0008U
#define DWC_SDHC_INTR_BWR_Msk               0x0010U
#define DWC_SDHC_INTR_BRR_Msk               0x0020U
#define DWC_SDHC_INTR_CARD_INSRT_Msk        0x0040U
#define DWC_SDHC_INTR_CARD_REM_Msk          0x0080U
#define DWC_SDHC_INTR_CARD_Msk              0x0100U
#define DWC_SDHC_NORM_INTR_ALL_Msk          0xFFFFU

/* Error Interrupts */
#define DWC_SDHC_ERROR_INTR_ALL_Msk         0xFFFFU

/*
 * ===========================================================
 * Capabilities Register 1 (0x040)
 * ============================================================
 */
#define DWC_SDHC_CAP1_MAX_BLK_LEN_Pos        16U
#define DWC_SDHC_CAP1_MAX_BLK_LEN_Msk        (0x03U << DWC_SDHC_CAP1_MAX_BLK_LEN_Pos)
#define DWC_SDHC_CAP1_8BIT_SUPPORT_Msk       BIT(18)
#define DWC_SDHC_CAP1_ADMA2_SUPPORT_Msk      BIT(19)
#define DWC_SDHC_CAP1_HIGH_SPEED_Msk         BIT(21)
#define DWC_SDHC_CAP1_SDMA_SUPPORT_Msk       BIT(22)
#define DWC_SDHC_CAP1_SUSPEND_RESUME_Msk     BIT(23)
#define DWC_SDHC_CAP1_VOL_3V3_Msk            BIT(24)
#define DWC_SDHC_CAP1_VOL_3V0_Msk            BIT(25)
#define DWC_SDHC_CAP1_VOL_1V8_Msk            BIT(26)
#define DWC_SDHC_CAP1_ASYNC_IRQ_Msk          BIT(29)

/*
 * ============================================================
 * Capabilities Register 2 (0x044)
 * ============================================================
 */
#define DWC_SDHC_CAP2_SDR50_SUPPORT_Msk      BIT(0)
#define DWC_SDHC_CAP2_SDR104_SUPPORT_Msk     BIT(1)
#define DWC_SDHC_CAP2_DDR50_SUPPORT_Msk      BIT(2)

/*
 * ============================================================
 * TIMEOUT_CTRL Register (0x00C)
 * ============================================================
 * time(in ms) to TOUT_CTRL register value conversion
 * 2^(tout+13) / 10^4 = time(in ms) for 10MHz TMCLK
 */
#define DWC_SDHC_SW_RST_TIMEOUT			0xFFFFU
#define DWC_SDHC_DATA_TIMEOUT_MS			10000
#define DWC_SDHC_TUNING_TIMEOUT_MS			1000
#define DWC_SDHC_1P8V_TIMEOUT_US			10000
#define DWC_SDHC_MAX_TIMEOUT			0xE

static inline uint8_t sdhc_dwc_ms_to_tout(uint32_t ms)
{
	if (!ms) {
		ms = 1;
	} else if (ms == SDHC_TIMEOUT_FOREVER) {
		return DWC_SDHC_MAX_TIMEOUT;
	}

	int bit_pos = LOG2(ms * 10000) - 13;

	return (bit_pos > DWC_SDHC_MAX_TIMEOUT) ? DWC_SDHC_MAX_TIMEOUT : bit_pos;
}

#endif /* ZEPHYR_DRIVERS_SDHC_SDHC_DWC_H_ */
