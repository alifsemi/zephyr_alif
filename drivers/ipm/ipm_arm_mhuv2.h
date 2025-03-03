/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_IPM_ARM_MHUV2_H_
#define ZEPHYR_DRIVERS_IPM_ARM_MHUV2_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/ipm.h>

#ifdef __cplusplus
extern "C" {
#define __I volatile	/* Defines 'read only' permissions */
#else
#define __I volatile const /* Defines 'read only' permissions */
#endif

#define __O volatile	/* Defines 'write only' permissions */
#define __IO volatile	/* Defines 'read / write' permissions */

#define MHU_ACC_REQ		0x1
#define MHU_ACC_RDY		0x1

#define MHU_CH_INT_EN		0x1
#define MHU_CH_INT_ST_SET	0x1
#define MHU_CH_INT_CLR		0x1

#define MHU_CLR_COMPLETE_CH	0xFFFFFFFF

#define NR2R_INTR		0x1
#define R2NR_INTR		0x2
#define CHCOMB_INTR		0x4
#define MAX_NUM_STAT_REG	124

#define MHU_CFG_DEF_Msk GENMASK(6, 0)

#define CHCOMB_INT_ST0_BEGIN    0
#define CHCOMB_INT_ST0_END      31
#define CHCOMB_INT_ST1_BEGIN    32
#define CHCOMB_INT_ST1_END      63
#define CHCOMB_INT_ST2_BEGIN    64
#define CHCOMB_INT_ST2_END      95
#define CHCOMB_INT_ST3_BEGIN    96
#define CHCOMB_INT_ST3_END      123

typedef void (*irq_init_func_t) (const struct device *dev);

struct MHUV2_SENDER_SLOT {
	__I	uint32_t CH_ST;     /* 0x00 */
		uint32_t Reserved0; /* 0x04 */
		uint32_t Reserved1; /* 0x08 */
	__O	uint32_t CH_SET;    /* 0x0C */
	__I	uint32_t CH_INT_ST; /* 0x10 */

	__IO	uint32_t CH_INT_CLR;/* 0x14 */
	__IO	uint32_t CH_INT_EN; /* 0x18 */
		uint32_t Reserved2; /* 0x1C */
};

struct MHUV2_SND {
	struct	MHUV2_SENDER_SLOT CHANNEL[MAX_NUM_STAT_REG];
		/* 0x0-0xF7C */
	__I	uint32_t MHU_CFG;
	__IO	uint32_t RESP_CFG;
	__IO	uint32_t ACCESS_REQUEST;
	__I	uint32_t ACCESS_READY;
	__I	uint32_t INT_ST;
	__O	uint32_t INT_CLR;
	__IO	uint32_t INT_EN;
		uint32_t Reserved0;
	__I	uint32_t CH_INT_ST0;
	__I	uint32_t CH_INT_ST1;
	__I	uint32_t CH_INT_ST2;
	__I	uint32_t CH_INT_ST3;
		uint32_t Reserved1[6];
	__I	uint32_t IIDR;
	__I	uint32_t AIDR;
	__I	uint32_t PID4;
	__I	uint32_t PID5;
	__I	uint32_t PID6;
	__I	uint32_t PID7;
	__I	uint32_t PID0;
	__I	uint32_t PID1;
	__I	uint32_t PID2;
	__I	uint32_t PID3;
	__I	uint32_t CID0;
	__I	uint32_t CID1;
	__I	uint32_t CID2;
	__I	uint32_t CID3;
};

struct MHUV2_RECEIVER_SLOT {
	__I	uint32_t CH_ST;	    /* 0x00 */
	__I	uint32_t CH_ST_MSK; /* 0x04 */
	__O	uint32_t CH_CLR;    /* 0x08 */
		uint32_t Reserved0; /* 0x0C */
	__I	uint32_t CH_MSK_ST; /* 0x10 */
	__O	uint32_t CH_MSK_SET;/* 0x14 */
	__O	uint32_t CH_MSK_CLR;/* 0x18 */
		uint32_t Reserved1; /* 0x1C */
};


struct MHUV2_REC {
	struct	MHUV2_RECEIVER_SLOT CHANNEL[MAX_NUM_STAT_REG];
		/* 0x0-0xF7C */
	__I	uint32_t MHU_CFG;
		uint32_t Reserved0[3];
	__I	uint32_t INT_ST;
	__O	uint32_t INT_CLR;
	__IO	uint32_t INT_EN;
		uint32_t Reserved1;
	__I	uint32_t CH_INT_ST0;
	__I	uint32_t CH_INT_ST1;
	__I	uint32_t CH_INT_ST2;
	__I	uint32_t CH_INT_ST3;
	__I	uint32_t Reserved2[6];
	__I	uint32_t IIDR;
	__I	uint32_t AIDR;
	__I	uint32_t PID4;
	__I	uint32_t PID5;
	__I	uint32_t PID6;
	__I	uint32_t PID7;
	__I	uint32_t PID0;
	__I	uint32_t PID1;
	__I	uint32_t PID2;
	__I	uint32_t PID3;
	__I	uint32_t CID0;
	__I	uint32_t CID1;
	__I	uint32_t CID2;
	__I	uint32_t CID3;
};

/* peripheral and component ID values */
#define MHU_SND_IIDR  0x0760043B
#define MHU_SND_AIDR  0x00000011
#define MHU_SND_PID4  0x00000004
#define MHU_SND_PID0  0x00000076
#define MHU_SND_PID1  0x000000B0
#define MHU_SND_PID2  0x0000000B
#define MHU_SND_PID3  0x00000000
#define MHU_SND_CID0  0x0000000D
#define MHU_SND_CID1  0x000000F0
#define MHU_SND_CID2  0x00000005
#define MHU_SND_CID3  0x000000B1

#define MHU_REC_PID4  0x04
#define MHU_REC_PID0  0x76
#define MHU_REC_PID1  0xB0
#define MHU_REC_PID2  0x0B
#define MHU_REC_PID3  0x00
#define MHU_REC_CID0  0x0D
#define MHU_REC_CID1  0xF0
#define MHU_REC_CID2  0x05
#define MHU_REC_CID3  0xB1
#define MHU_REC_AIDR  0x11
#define MHU_REC_IIDR  0x0760043B

struct mhuv2_device_config {
	DEVICE_MMIO_ROM;
	int16_t			irq_num;
	irq_init_func_t		irq_init_func;
	bool			irq_type;
};

/* Device data structure */
struct mhuv2_device_data {
	DEVICE_MMIO_RAM;
	uint8_t		max_ch;
	ipm_callback_t	callback;
	uint32_t	*user_data;
};

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* ZEPHYR_DRIVERS_IPM_ARM_MHUV2_H */
