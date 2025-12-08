/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __ZEPHYR_INCLUDE_DRIVERS_ALIF_VIDEO_H__
#define __ZEPHYR_INCLUDE_DRIVERS_ALIF_VIDEO_H__

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/video.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VIDEO_CID_ALIF_CSI_DPHY_FREQ         (VIDEO_CID_PRIVATE_BASE + 0)
#define VIDEO_CID_ALIF_CSI_CURR_CAM          (VIDEO_CID_PRIVATE_BASE + 1)

/* Additional supported formats */
#define VIDEO_PIX_FMT_RGB888_PLANAR_PRIVATE  (VIDEO_FOURCC('P', 'R', 'G', 'B'))
#define VIDEO_PIX_FMT_BGGR10P                (VIDEO_FOURCC('p', 'B', 'A', 'A'))
#define VIDEO_PIX_FMT_GBRG10P                (VIDEO_FOURCC('p', 'G', 'A', 'A'))
#define VIDEO_PIX_FMT_GRBG10P                (VIDEO_FOURCC('p', 'g', 'A', 'A'))
#define VIDEO_PIX_FMT_RGGB10P                (VIDEO_FOURCC('p', 'R', 'A', 'A'))
#define VIDEO_PIX_FMT_BGGR12P                (VIDEO_FOURCC('p', 'B', 'C', 'C'))
#define VIDEO_PIX_FMT_GBRG12P                (VIDEO_FOURCC('p', 'G', 'C', 'C'))
#define VIDEO_PIX_FMT_GRBG12P                (VIDEO_FOURCC('p', 'g', 'C', 'C'))
#define VIDEO_PIX_FMT_RGGB12P                (VIDEO_FOURCC('p', 'R', 'C', 'C'))
#define VIDEO_PIX_FMT_BGGR14P                (VIDEO_FOURCC('p', 'B', 'E', 'E'))
#define VIDEO_PIX_FMT_GBRG14P                (VIDEO_FOURCC('p', 'G', 'E', 'E'))
#define VIDEO_PIX_FMT_GRBG14P                (VIDEO_FOURCC('p', 'g', 'E', 'E'))
#define VIDEO_PIX_FMT_RGGB14P                (VIDEO_FOURCC('p', 'R', 'E', 'E'))
#define VIDEO_PIX_FMT_BGGR10                 (VIDEO_FOURCC('B', 'G', '1', '0'))
#define VIDEO_PIX_FMT_GBRG10                 (VIDEO_FOURCC('G', 'B', '1', '0'))
#define VIDEO_PIX_FMT_GRBG10                 (VIDEO_FOURCC('B', 'A', '1', '0'))
#define VIDEO_PIX_FMT_RGGB10                 (VIDEO_FOURCC('R', 'G', '1', '0'))
#define VIDEO_PIX_FMT_BGGR12                 (VIDEO_FOURCC('B', 'G', '1', '2'))
#define VIDEO_PIX_FMT_GBRG12                 (VIDEO_FOURCC('G', 'B', '1', '2'))
#define VIDEO_PIX_FMT_GRBG12                 (VIDEO_FOURCC('B', 'A', '1', '2'))
#define VIDEO_PIX_FMT_RGGB12                 (VIDEO_FOURCC('R', 'G', '1', '2'))
#define VIDEO_PIX_FMT_BGGR14                 (VIDEO_FOURCC('B', 'G', '1', '4'))
#define VIDEO_PIX_FMT_GBRG14                 (VIDEO_FOURCC('G', 'B', '1', '4'))
#define VIDEO_PIX_FMT_GRBG14                 (VIDEO_FOURCC('G', 'R', '1', '4'))
#define VIDEO_PIX_FMT_RGGB14                 (VIDEO_FOURCC('R', 'G', '1', '4'))
#define VIDEO_PIX_FMT_BGGR16                 (VIDEO_FOURCC('B', 'Y', 'R', '2'))
#define VIDEO_PIX_FMT_GBRG16                 (VIDEO_FOURCC('G', 'B', '1', '6'))
#define VIDEO_PIX_FMT_GRBG16                 (VIDEO_FOURCC('G', 'R', '1', '6'))
#define VIDEO_PIX_FMT_RGGB16                 (VIDEO_FOURCC('R', 'G', '1', '6'))
#define VIDEO_PIX_FMT_Y10                    (VIDEO_FOURCC('Y', '1', '0', ' '))
#define VIDEO_PIX_FMT_Y12                    (VIDEO_FOURCC('Y', '1', '2', ' '))
#define VIDEO_PIX_FMT_Y14                    (VIDEO_FOURCC('Y', '1', '4', ' '))
#define VIDEO_PIX_FMT_YVYU                   (VIDEO_FOURCC('Y', 'V', 'Y', 'U'))
#define VIDEO_PIX_FMT_VYUY                   (VIDEO_FOURCC('V', 'Y', 'U', 'Y'))
#define VIDEO_PIX_FMT_UYVY                   (VIDEO_FOURCC('U', 'Y', 'V', 'Y'))
#define VIDEO_PIX_FMT_NV12                   (VIDEO_FOURCC('N', 'V', '1', '2'))
#define VIDEO_PIX_FMT_NV21                   (VIDEO_FOURCC('N', 'V', '2', '1'))
#define VIDEO_PIX_FMT_NV16                   (VIDEO_FOURCC('N', 'V', '1', '6'))
#define VIDEO_PIX_FMT_NV61                   (VIDEO_FOURCC('N', 'V', '6', '1'))
#define VIDEO_PIX_FMT_NV24                   (VIDEO_FOURCC('N', 'V', '2', '4'))
#define VIDEO_PIX_FMT_NV42                   (VIDEO_FOURCC('N', 'V', '4', '2'))
#define VIDEO_PIX_FMT_YUV422P                (VIDEO_FOURCC('4', '2', '2', 'P'))
#define VIDEO_PIX_FMT_YUV420                 (VIDEO_FOURCC('Y', 'U', '1', '2'))
#define VIDEO_PIX_FMT_YVU420                 (VIDEO_FOURCC('Y', 'V', '1', '2'))

enum csi2_data_type {
	/* Data Type Non-Image data */
	CSI2_DT_EMBEDDED = 0x12, /* Embedded 8-bit non-Image data */
	/* Data Type YUV */
	CSI2_DT_YUV420_8_BIT = 0x18,        /* YUV420 8-bit */
	CSI2_DT_YUV420_10_BIT = 0x19,       /* YUV420 10-bit */
	CSI2_DT_YUV420_LEGACY_8_BIT = 0x1A, /* YUV420 8-bit Legacy */
	CSI2_DT_YUV420_CSPS_8_BIT = 0x1C,   /* YUV420 8-bit CSPS */
	CSI2_DT_YUV420_CSPS_10_BIT = 0x1D,  /* YUV420 10-bit CSPS */
	CSI2_DT_YUV422_8_BIT = 0x1E,        /* YUV422 8-bit */
	CSI2_DT_YUV422_10_BIT = 0x1F,       /* YUV422 10-bit */
	/* Data Type RGB */
	CSI2_DT_RGB444 = 0x20, /* RGB444 */
	CSI2_DT_RGB555 = 0x21, /* RGB555 */
	CSI2_DT_RGB565 = 0x22, /* RGB565 */
	CSI2_DT_RGB666 = 0x23, /* RGB666 */
	CSI2_DT_RGB888 = 0x24, /* RGB888 */
	/* Data Type RAW */
	CSI2_DT_RAW6 = 0x28,  /* RAW6 */
	CSI2_DT_RAW7 = 0x29,  /* RAW7 */
	CSI2_DT_RAW8 = 0x2A,  /* RAW8 */
	CSI2_DT_RAW10 = 0x2B, /* RAW10 */
	CSI2_DT_RAW12 = 0x2C, /* RAW12 */
	CSI2_DT_RAW14 = 0x2D, /* RAW14 */
	CSI2_DT_RAW16 = 0x2E, /* RAW16 */
};

enum csi2_ipi_interface {
	CSI2_IPI_MODE_48_BIT_IFX,
	CSI2_IPI_MODE_16_BIT_IFX,
};

enum cpi_csi_color_mode {
	CPI_COLOR_MODE_CONFIG_IPI16_RAW6 = 0,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW7,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW8,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW10,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW12,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW14,
	CPI_COLOR_MODE_CONFIG_IPI16_RAW16,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB444,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB555,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB565,
	CPI_COLOR_MODE_CONFIG_IPI48_RGB666,
	CPI_COLOR_MODE_CONFIG_IPI48_XRGB888,
	CPI_COLOR_MODE_CONFIG_IPI48_RGBX888,
	CPI_COLOR_MODE_CONFIG_IPI48_RAW32,
	CPI_COLOR_MODE_CONFIG_IPI48_RAW48,
};

enum cpi_data_mode {
	CPI_DATA_MODE_1_BIT = 0,
	CPI_DATA_MODE_2_BIT,
	CPI_DATA_MODE_4_BIT,
	CPI_DATA_MODE_8_BIT,
	CPI_DATA_MODE_16_BIT,
	CPI_DATA_MODE_32_BIT,
	CPI_DATA_MODE_64_BIT,
};

struct cpi_csi2_mode_settings {
	enum csi2_data_type dt;
	enum csi2_ipi_interface ipi_ifx;
	enum cpi_csi_color_mode col_mode;
	enum cpi_data_mode data_mode;
	uint32_t bits_per_pixel;
};

static const struct cpi_csi2_mode_settings data_mode_settings[] = {
	{CSI2_DT_RAW6, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW6,
	 CPI_DATA_MODE_8_BIT, 6},
	{CSI2_DT_RAW7, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW7,
	 CPI_DATA_MODE_8_BIT, 7},
	{CSI2_DT_RAW8, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW8,
	 CPI_DATA_MODE_8_BIT, 8},
	{CSI2_DT_RAW10, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW10,
	 CPI_DATA_MODE_16_BIT, 10 },
	{CSI2_DT_RAW12, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW12,
	 CPI_DATA_MODE_16_BIT, 12},
	{CSI2_DT_RAW14, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW14,
	 CPI_DATA_MODE_16_BIT, 14},
	{CSI2_DT_RAW16, CSI2_IPI_MODE_16_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI16_RAW16,
	 CPI_DATA_MODE_16_BIT, 16},
	{CSI2_DT_RGB444, CSI2_IPI_MODE_48_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI48_RGB444,
	 CPI_DATA_MODE_16_BIT, 12},
	{CSI2_DT_RGB555, CSI2_IPI_MODE_48_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI48_RGB555,
	 CPI_DATA_MODE_16_BIT, 15},
	{CSI2_DT_RGB565, CSI2_IPI_MODE_48_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI48_RGB565,
	 CPI_DATA_MODE_16_BIT, 16},
	{CSI2_DT_RGB666, CSI2_IPI_MODE_48_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI48_RGB666,
	 CPI_DATA_MODE_32_BIT, 18},
	{CSI2_DT_RGB888, CSI2_IPI_MODE_48_BIT_IFX, CPI_COLOR_MODE_CONFIG_IPI48_XRGB888,
	 CPI_DATA_MODE_32_BIT, 24},
};

size_t fourcc_to_plane_size(uint32_t fourcc, uint8_t plane_id, size_t buffer_size);
int fourcc_to_numplanes(uint32_t fourcc);
unsigned int pix_fmt_bpp(uint32_t fourcc);

#ifdef __cplusplus
}
#endif

#endif /* __ZEPHYR_INCLUDE_DRIVERS_ALIF_VIDEO_H__ */
