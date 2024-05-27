/* Copyright (C) 2024 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_COUNTER_DW_RTC_H_
#define ZEPHYR_DRIVERS_COUNTER_DW_RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* counter_dw_rtc.h - Designware RTC driver register definitions */
#define DW_RTC_REG_CCVR			(0x00)
#define DW_RTC_REG_CMR			(0x04)
#define DW_RTC_REG_CLR			(0x08)
#define DW_RTC_REG_CCR			(0x0c)
#define DW_RTC_REG_STAT			(0x10)
#define DW_RTC_REG_RSTAT		(0x14)
#define DW_RTC_REG_EOI			(0x18)
#define DW_RTC_REG_COMP_VERSION		(0x1c)
#define DW_RTC_REG_CPSR			(0x20)
#define DW_RTC_REG_CPCVR		(0x24)

#define Z_REG_READ(__sz) sys_read##__sz
#define Z_REG_WRITE(__sz) sys_write##__sz
#define Z_REG_SET_BIT sys_set_bit
#define Z_REG_CLEAR_BIT sys_clear_bit
#define Z_REG_TEST_BIT sys_test_bit

#define DEFINE_MM_REG_READ(__reg, __off, __sz)				\
	static inline uint32_t read_##__reg(uint32_t addr)		\
	{								\
		return Z_REG_READ(__sz)(addr + __off);			\
	}
#define DEFINE_MM_REG_WRITE(__reg, __off, __sz)				\
	static inline void write_##__reg(uint32_t data, uint32_t addr)	\
	{								\
		Z_REG_WRITE(__sz)(data, addr + __off);			\
	}

#define DEFINE_SET_BIT_OP(__reg_bit, __reg_off, __bit)			\
	static inline void set_bit_##__reg_bit(uint32_t addr)		\
	{								\
		Z_REG_SET_BIT(addr + __reg_off, __bit);			\
	}

#define DEFINE_CLEAR_BIT_OP(__reg_bit, __reg_off, __bit)		\
	static inline void clear_bit_##__reg_bit(uint32_t addr)		\
	{								\
		Z_REG_CLEAR_BIT(addr + __reg_off, __bit);		\
	}

#define DEFINE_TEST_BIT_OP(__reg_bit, __reg_off, __bit)			\
	static inline int test_bit_##__reg_bit(uint32_t addr)		\
	{								\
		return Z_REG_TEST_BIT(addr + __reg_off, __bit);		\
	}

/* Register bit settings */

/* CCR settings */
#define DW_RTC_CCR_IEN			(0)
#define DW_RTC_CCR_MASK			(1)
#define DW_RTC_CCR_EN			(2)
#define DW_RTC_CCR_WEN			(3)
#define DW_RTC_CCR_PSCLR_EN		(4)
#define DW_RTC_CCR_PROT_LEVEL		(5)

/* STAT settings */
#define DW_RTC_STAT_IS_BIT		(0)

/* VBAT settings */
#define VBAT_RTC_CLK_EN_CLK_EN_BIT	(0)

/* Register helpers */
DEFINE_MM_REG_READ(ccvr, DW_RTC_REG_CCVR, 32)
DEFINE_MM_REG_WRITE(cmr, DW_RTC_REG_CMR, 32)
DEFINE_MM_REG_READ(cmr, DW_RTC_REG_CMR, 32)
DEFINE_MM_REG_WRITE(clr, DW_RTC_REG_CLR, 32)
DEFINE_MM_REG_READ(clr, DW_RTC_REG_CLR, 32)
DEFINE_MM_REG_WRITE(ccr, DW_RTC_REG_CCR, 32)
DEFINE_MM_REG_READ(ccr, DW_RTC_REG_CCR, 32)
DEFINE_MM_REG_READ(stat, DW_RTC_REG_STAT, 32)
DEFINE_MM_REG_READ(rstat, DW_RTC_REG_RSTAT, 32)
DEFINE_MM_REG_READ(eoi, DW_RTC_REG_EOI, 32)
DEFINE_MM_REG_READ(comp_version, DW_RTC_REG_COMP_VERSION, 32)
DEFINE_MM_REG_WRITE(cpsr, DW_RTC_REG_CPSR, 32)
DEFINE_MM_REG_READ(cpcvr, DW_RTC_REG_CPCVR, 32)

/* ICR is on a unique bit */
DEFINE_TEST_BIT_OP(stat, DW_RTC_REG_STAT, DW_RTC_STAT_IS_BIT)
#define clear_interrupts(addr) read_eoi(addr)

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_COUNTER_DW_RTC_H_ */
