/*
 * Copyright (c) 2026 Alif Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_JPEG_HANTRO_VC9000E_REGS_H_
#define ZEPHYR_DRIVERS_VIDEO_JPEG_HANTRO_VC9000E_REGS_H_

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* JPEG Register Offsets */
#define JPEG_SWREG0_OFFSET      0x000
#define JPEG_SWREG1_OFFSET      0x004
#define JPEG_SWREG4_OFFSET      0x010
#define JPEG_SWREG5_OFFSET      0x014
#define JPEG_SWREG8_OFFSET      0x020
#define JPEG_SWREG9_OFFSET      0x024
#define JPEG_SWREG12_OFFSET     0x030
#define JPEG_SWREG13_OFFSET     0x034
#define JPEG_SWREG14_OFFSET     0x038
#define JPEG_SWREG18_OFFSET     0x048
#define JPEG_SWREG20_OFFSET     0x050
#define JPEG_SWREG38_OFFSET     0x098
#define JPEG_SWREG80_OFFSET     0x140
#define JPEG_SWREG81_OFFSET     0x144
#define JPEG_SWREG82_OFFSET     0x148
#define JPEG_SWREG83_OFFSET     0x14C
#define JPEG_SWREG193_OFFSET    0x304
#define JPEG_SWREG210_OFFSET    0x348
#define JPEG_SWREG211_OFFSET    0x34C
#define JPEG_SWREG214_OFFSET    0x358
#define JPEG_SWREG246_OFFSET    0x3D8
#define JPEG_SWREG249_OFFSET    0x3E4
#define JPEG_SWREG261_OFFSET    0x414
#define JPEG_SWREG349_OFFSET    0x574

/* SWREG1 - Interrupt Control/Status */
#define JPEG_IRQ_STATUS                 BIT(0)
#define JPEG_IRQ_DISABLE                BIT(1)
#define JPEG_FRAME_RDY_STATUS           BIT(2)
#define JPEG_BUS_ERROR_STATUS           BIT(3)
#define JPEG_SW_RESET                   BIT(4)
#define JPEG_BUFFER_FULL                BIT(5)
#define JPEG_TIMEOUT                    BIT(6)
#define JPEG_IRQ_LINE_BUFFER            BIT(7)
#define JPEG_SLICE_RDY_STATUS           BIT(8)
#define JPEG_IRQ_FUSE_ERROR             BIT(9)
#define JPEG_TIMEOUT_INT                BIT(11)
#define JPEG_STRM_SEGMENT_RDY_INT       BIT(12)

#define JPEG_IRQ_TYPE_FRAME_RDY         BIT(16)
#define JPEG_IRQ_TYPE_SLICE_RDY         BIT(17)
#define JPEG_IRQ_TYPE_LINE_BUFFER       BIT(18)
#define JPEG_IRQ_TYPE_STRM_SEGMENT      BIT(19)
#define JPEG_IRQ_TYPE_TIMEOUT           BIT(20)
#define JPEG_IRQ_TYPE_BUS_ERROR         BIT(21)
#define JPEG_IRQ_TYPE_BUFFER_FULL       BIT(22)
#define JPEG_IRQ_TYPE_FUSE_ERROR        BIT(23)
#define JPEG_IRQ_TYPE_SW_RESET          BIT(24)

#define JPEG_IRQ_STATUS_MASK		(JPEG_IRQ_STATUS | JPEG_FRAME_RDY_STATUS | \
					 JPEG_BUS_ERROR_STATUS | JPEG_BUFFER_FULL |\
					 JPEG_TIMEOUT         | JPEG_IRQ_FUSE_ERROR)

#define JPEG_IRQ_EN_MASK                (JPEG_IRQ_TYPE_FRAME_RDY   | \
					 JPEG_IRQ_TYPE_BUS_ERROR   | \
					 JPEG_IRQ_TYPE_BUFFER_FULL | \
					 JPEG_IRQ_TYPE_TIMEOUT | JPEG_IRQ_TYPE_LINE_BUFFER)

/* SWREG4 - Encoding Mode */
#define JPEG_SW_ENC_MODE_POS            29
#define JPEG_SW_ENC_MODE_MASK           (0x7UL << JPEG_SW_ENC_MODE_POS)
#define JPEG_SW_ENC_MODE_JPEG           0x4

/* SWREG5 - Picture Size and Enable */
#define JPEG_PIC_WIDTH_POS              20
#define JPEG_PIC_WIDTH_MASK             (0xFFFUL << JPEG_PIC_WIDTH_POS)
#define JPEG_PIC_HEIGHT_POS             8
#define JPEG_PIC_WH_PIXEL_SHIFT         3
#define JPEG_PIC_WH_FIELD_WIDTH         12
#define JPEG_PIC_WH_MASK                0xFFF
#define JPEG_PIC_HEIGHT_MASK            (0xFFFUL << JPEG_PIC_HEIGHT_POS)
#define JPEG_ENC_ENABLE                 BIT(0)

/* SWREG18 - JPEG Mode */
#define JPEG_MODE_POS                   24
#define JPEG_MODE_MASK                  BIT(JPEG_MODE_POS)
#define JPEG_SLICE_MODE                 BIT(25)
#define JPEG_MODE_420                   0

/* SWREG20 - Row Length and Format */
#define JPEG_ROWLENGTH_POS              17
#define JPEG_ROWLENGTH_MASK             (0x7FFFUL << JPEG_ROWLENGTH_POS)
#define JPEG_ROWLENGTH_FIELD_WIDTH      15
#define JPEG_ROWLENGTH_FIELD_MASK       0x7FFF
#define JPEG_LOSSLESS_EN                BIT(16)
#define JPEG_CODING_MODE_POS            14
#define JPEG_CODING_MODE_MASK           (0x3UL << JPEG_CODING_MODE_POS)
#define JPEG_CODING_MODE_420            0

/* SWREG38 - Input Format and Rotation */
#define JPEG_INPUT_FORMAT_POS           28
#define JPEG_INPUT_FORMAT_MASK          (0xFUL << JPEG_INPUT_FORMAT_POS)
#define JPEG_INPUT_FORMAT_YUV420SP      1

#define JPEG_YFILL_POS                  1
#define JPEG_YFILL_MASK                 (0x7UL << JPEG_YFILL_POS)
#define JPEG_YFILL_FIELD_WIDTH          3
#define JPEG_YFILL_FIELD_MASK           0x7
#define JPEG_XFILL_POS                  4
#define JPEG_XFILL_MASK                 (0x3UL << JPEG_XFILL_POS)
#define JPEG_XFILL_FIELD_WIDTH          2
#define JPEG_XFILL_FIELD_MASK           0x3

/* SWREG81 - AXI Burst Length */
#define JPEG_MAX_BURST_POS              24
#define JPEG_MAX_BURST_MASK             (0xFFUL << JPEG_MAX_BURST_POS)

/* SWREG193 - Fill MSB */
#define JPEG_XFILL_MSB_POS              6
#define JPEG_XFILL_MSB_MASK             (0x3UL << JPEG_XFILL_MSB_POS)
#define JPEG_YFILL_MSB_POS              4
#define JPEG_YFILL_MSB_MASK             (0x3UL << JPEG_YFILL_MSB_POS)

/* SWREG210 - Luma Stride */
#define JPEG_LUMA_STRIDE_POS            12
#define JPEG_LUMA_STRIDE_MASK           (0xFFFFFUL << JPEG_LUMA_STRIDE_POS)

/* SWREG211 - Chroma Stride */
#define JPEG_CHROMA_STRIDE_POS          12
#define JPEG_CHROMA_STRIDE_MASK         (0xFFFFFUL << JPEG_CHROMA_STRIDE_POS)

/* SWREG214 - Hardware Capabilities */
#define JPEG_HW_LOSSLESS_SUPPORT        BIT(31)
#define JPEG_HW_MAX_WIDTH_MASK          0x1FF

/* SWREG246 - AXI Write Outstanding */
#define JPEG_AXI_WR_OUTSTANDING_POS     6
#define JPEG_AXI_WR_OUTSTANDING_MASK    (0xFFUL << JPEG_AXI_WR_OUTSTANDING_POS)

/* SWREG249 - Picture Size MSB */
#define JPEG_PIC_WIDTH_MSB_POS          9
#define JPEG_PIC_WIDTH_MSB_MASK         (0x3UL << JPEG_PIC_WIDTH_MSB_POS)
#define JPEG_PIC_HEIGHT_MSB_POS         7
#define JPEG_PIC_HEIGHT_MSB_MASK        (0x3UL << JPEG_PIC_HEIGHT_MSB_POS)
#define JPEG_ROWLENGTH_MSB_POS          5
#define JPEG_ROWLENGTH_MSB_MASK         (0x3UL << JPEG_ROWLENGTH_MSB_POS)

/* SWREG261 - AXI Read Outstanding */
#define JPEG_AXI_RD_OUTSTANDING_POS     4
#define JPEG_AXI_RD_OUTSTANDING_MASK    (0xFFUL << JPEG_AXI_RD_OUTSTANDING_POS)

/* SWREG349 - SBI Control */
#define JPEG_SBI_WAIT_FRAME_START       BIT(31)

/* YUV420 Format Constants */
#define JPEG_YUV420_NUM_COMPONENTS      3
#define JPEG_YUV420_CHROMA_DIV          2
#define JPEG_YUV420_FRAME_SIZE(stride, height)  ((stride) * (height) * 3 / 2)

/* Hardware identification values */
#define JPEG_HW_ID                      0x90001000
#define JPEG_HW_VERSION                 0x00C0C200

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_VIDEO_JPEG_HANTRO_VC9000E_REGS_H_ */
