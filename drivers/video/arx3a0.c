/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT onnn_arx3a0
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(arx3a0);

#define ARX3A0_CHIP_ID_VAL 0x0353

/* Sysctl registers */
#define ARX3A0_CHIP_ID                          0x3000
#define ARX3A0_MODE_SELECT_REGISTER             0x0100
#define ARX3A0_SOFTWARE_RESET_REGISTER          0x0103
#define ARX3A0_RESET_REGISTER                   0x301A
#define ARX3A0_MIPI_CONFIG_REGISTER             0x31BE
#define ARX3A0_COARSE_INTEGRATION_TIME_REGISTER 0x3012
#define ARX3A0_GLOBAL_GAIN_REGISTER             0x305E
#define ARX3A0_DATA_PEDESTAL_REGISTER           0x301E

/* Bitfields for MIPI_CONFIG_REGISTER. */
#define ARX3A0_MIPI_CONFIG_MIPI_MIRROR_2LANES     BIT(10)
#define ARX3A0_MIPI_CONFIG_TEST_MIPI_START_CHKSUM BIT(9)
#define ARX3A0_MIPI_CONFIG_LP11_ON_STANDBY        BIT(7)
#define ARX3A0_MIPI_CONFIG_FRAME_CNT_RESET        BIT(1)
#define ARX3A0_MIPI_CONFIG_FRAME_CNT_EN           BIT(0)

/* Camera Control registers */
#define ARX3A0_CAM_OUTPUT_FORMAT 0x0112

/* Camera output format */
#define ARX3A0_CAM_OUTPUT_FORMAT_RAW10 (0x0A0A)
#define ARX3A0_CAM_OUTPUT_FORMAT_RAW8  (0x0808)

#define ARX3A0_FRAME_RATE 5

struct arx3a0_config {
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec power_gpio;
	struct i2c_dt_spec i2c;
};

struct arx3a0_data {
	struct video_format fmt;
	bool is_streaming;
};

struct arx3a0_reg {
	uint16_t addr;
	uint16_t value_size;
	uint32_t value;
};

static struct arx3a0_reg arx3a0_560_regs[] = {
#if (ARX3A0_FRAME_RATE == 90)
	{0x304C, 2, 0x3000}, /* OPTM_RECORD */
	{0x304A, 2, 0x0070}, /* OTPM_CONTROL */
	{0x0103, 2, 0x01},   /* SOFTWARE_RESET */

	{0xFFFF, 2, 500}, /* DELAY= 100 */

	{0x3ED0, 2, 0x0748}, /* RESERVED_MFR_3ED0 */
	{0x3ED6, 2, 0x3136}, /* RESERVED_MFR_3ED6 */
	{0x3EDC, 2, 0x1020}, /* RESERVED_MFR_3EDC */
	{0x3EDE, 2, 0x1D2A}, /* RESERVED_MFR_3EDE */
	{0x3EE0, 2, 0x282A}, /* RESERVED_MFR_3EE0 */
	{0x3EE2, 2, 0x2821}, /* RESERVED_MFR_3EE2 */
	{0x3EC8, 2, 0x0401}, /* RESERVED_MFR_3EC8 */
	{0x3ED2, 2, 0x3903}, /* RESERVED_MFR_3ED2 */
	{0x3EC0, 2, 0x0011}, /* RESERVED_MFR_3EC0 */
	{0x3ECA, 2, 0x826F}, /* RESERVED_MFR_3ECA */
	{0x3EBC, 2, 0xA8AA}, /* RESERVED_MFR_3EBC */
	{0x3EC4, 2, 0x1000}, /* RESERVED_MFR_3EC4 */
	{0x3EBA, 2, 0x0044}, /* RESERVED_MFR_3EBA */
	{0x3ED0, 2, 0x0745}, /* RESERVED_MFR_3ED0 */
	{0x3ED4, 2, 0x0016}, /* RESERVED_MFR_3ED4 */
	{0x3EC6, 2, 0x80F2}, /* RESERVED_MFR_3EC6 */
	{0x3ED8, 2, 0x55FF}, /* RESERVED_MFR_3ED8 */
	{0x3EE6, 2, 0x8000}, /* RESERVED_MFR_3EE6 */
	{0x30D2, 2, 0x0000}, /* RESERVED_MFR_30D2 */
	{0x31E0, 2, 0x00F1}, /* PIX_DEF_ID */
	{0x31E6, 2, 0xA35F}, /* PIX_DEF_ID_2 */
	{0x3180, 2, 0x9096}, /* RESERVED_MFR_3180 */
	{0x3120, 2, 0x0001}, /* GAIN_DITHER_CONTROL */
	{0x301E, 2, 0x002A}, /* DATA_PEDESTAL_ */
	{0x3D00, 2, 0x0436}, /* RESERVED_MFR_3D00 */
	{0x3D02, 2, 0x435A}, /* RESERVED_MFR_3D02 */
	{0x3D04, 2, 0xFFFF}, /* RESERVED_MFR_3D04 */
	{0x3D06, 2, 0xFFFF}, /* RESERVED_MFR_3D06 */
	{0x3D08, 2, 0x2180}, /* RESERVED_MFR_3D08 */
	{0x3D0A, 2, 0x0005}, /* RESERVED_MFR_3D0A */
	{0x3D0C, 2, 0x108F}, /* RESERVED_MFR_3D0C */
	{0x3D0E, 2, 0x0802}, /* RESERVED_MFR_3D0E */
	{0x3D10, 2, 0x5248}, /* RESERVED_MFR_3D10 */
	{0x3D12, 2, 0x801B}, /* RESERVED_MFR_3D12 */
	{0x3D14, 2, 0x006F}, /* RESERVED_MFR_3D14 */
	{0x3D16, 2, 0x8269}, /* RESERVED_MFR_3D16 */
	{0x3D18, 2, 0x6A82}, /* RESERVED_MFR_3D18 */
	{0x3D1A, 2, 0x5148}, /* RESERVED_MFR_3D1A */
	{0x3D1C, 2, 0x5A80}, /* RESERVED_MFR_3D1C */
	{0x3D1E, 2, 0x5902}, /* RESERVED_MFR_3D1E */
	{0x3D20, 2, 0x8082}, /* RESERVED_MFR_3D20 */
	{0x3D22, 2, 0x3060}, /* RESERVED_MFR_3D22 */
	{0x3D24, 2, 0x8567}, /* RESERVED_MFR_3D24 */
	{0x3D26, 2, 0x5C20}, /* RESERVED_MFR_3D26 */
	{0x3D28, 2, 0x4880}, /* RESERVED_MFR_3D28 */
	{0x3D2A, 2, 0x0284}, /* RESERVED_MFR_3D2A */
	{0x3D2C, 2, 0x6084}, /* RESERVED_MFR_3D2C */
	{0x3D2E, 2, 0x5C91}, /* RESERVED_MFR_3D2E */
	{0x3D30, 2, 0x5980}, /* RESERVED_MFR_3D30 */
	{0x3D32, 2, 0x5883}, /* RESERVED_MFR_3D32 */
	{0x3D34, 2, 0x6462}, /* RESERVED_MFR_3D34 */
	{0x3D36, 2, 0x8056}, /* RESERVED_MFR_3D36 */
	{0x3D38, 2, 0x8162}, /* RESERVED_MFR_3D38 */
	{0x3D3A, 2, 0x8422}, /* RESERVED_MFR_3D3A */
	{0x3D3C, 2, 0x20A2}, /* RESERVED_MFR_3D3C */
	{0x3D3E, 2, 0x2220}, /* RESERVED_MFR_3D3E */
	{0x3D40, 2, 0x804B}, /* RESERVED_MFR_3D40 */
	{0x3D42, 2, 0x8110}, /* RESERVED_MFR_3D42 */
	{0x3D44, 2, 0x0943}, /* RESERVED_MFR_3D44 */
	{0x3D46, 2, 0x9243}, /* RESERVED_MFR_3D46 */
	{0x3D48, 2, 0x8050}, /* RESERVED_MFR_3D48 */
	{0x3D4A, 2, 0x9A4B}, /* RESERVED_MFR_3D4A */
	{0x3D4C, 2, 0x8563}, /* RESERVED_MFR_3D4C */
	{0x3D4E, 2, 0x8363}, /* RESERVED_MFR_3D4E */
	{0x3D50, 2, 0x8422}, /* RESERVED_MFR_3D50 */
	{0x3D52, 2, 0x20A2}, /* RESERVED_MFR_3D52 */
	{0x3D54, 2, 0x61C6}, /* RESERVED_MFR_3D54 */
	{0x3D56, 2, 0x6F99}, /* RESERVED_MFR_3D56 */
	{0x3D58, 2, 0x3009}, /* RESERVED_MFR_3D58 */
	{0x3D5A, 2, 0x1FF6}, /* RESERVED_MFR_3D5A */
	{0x3D5C, 2, 0x20ED}, /* RESERVED_MFR_3D5C */
	{0x3D5E, 2, 0x0874}, /* RESERVED_MFR_3D5E */
	{0x3D60, 2, 0x8230}, /* RESERVED_MFR_3D60 */
	{0x3D62, 2, 0x609B}, /* RESERVED_MFR_3D62 */
	{0x3D64, 2, 0x3060}, /* RESERVED_MFR_3D64 */
	{0x3D66, 2, 0x4600}, /* RESERVED_MFR_3D66 */
	{0x3D68, 2, 0x3783}, /* RESERVED_MFR_3D68 */
	{0x3D6A, 2, 0x7070}, /* RESERVED_MFR_3D6A */
	{0x3D6C, 2, 0x8040}, /* RESERVED_MFR_3D6C */
	{0x3D6E, 2, 0x4A44}, /* RESERVED_MFR_3D6E */
	{0x3D70, 2, 0x8003}, /* RESERVED_MFR_3D70 */
	{0x3D72, 2, 0x0086}, /* RESERVED_MFR_3D72 */
	{0x3D74, 2, 0x4588}, /* RESERVED_MFR_3D74 */
	{0x3D76, 2, 0x46BA}, /* RESERVED_MFR_3D76 */
	{0x3D78, 2, 0x0300}, /* RESERVED_MFR_3D78 */
	{0x3D7A, 2, 0xFFD7}, /* RESERVED_MFR_3D7A */
	{0x3D7C, 2, 0x4688}, /* RESERVED_MFR_3D7C */
	{0x3D7E, 2, 0x4588}, /* RESERVED_MFR_3D7E */
	{0x3D80, 2, 0x4492}, /* RESERVED_MFR_3D80 */
	{0x3D82, 2, 0x4A9B}, /* RESERVED_MFR_3D82 */
	{0x3D84, 2, 0x4070}, /* RESERVED_MFR_3D84 */
	{0x3D86, 2, 0x8040}, /* RESERVED_MFR_3D86 */
	{0x3D88, 2, 0x4AAD}, /* RESERVED_MFR_3D88 */
	{0x3D8A, 2, 0x0070}, /* RESERVED_MFR_3D8A */
	{0x3D8C, 2, 0xAE47}, /* RESERVED_MFR_3D8C */
	{0x3D8E, 2, 0x8547}, /* RESERVED_MFR_3D8E */
	{0x3D90, 2, 0xAD78}, /* RESERVED_MFR_3D90 */
	{0x3D92, 2, 0x6B85}, /* RESERVED_MFR_3D92 */
	{0x3D94, 2, 0x6A80}, /* RESERVED_MFR_3D94 */
	{0x3D96, 2, 0x6984}, /* RESERVED_MFR_3D96 */
	{0x3D98, 2, 0x6B8A}, /* RESERVED_MFR_3D98 */
	{0x3D9A, 2, 0x6B80}, /* RESERVED_MFR_3D9A */
	{0x3D9C, 2, 0x6980}, /* RESERVED_MFR_3D9C */
	{0x3D9E, 2, 0x6A85}, /* RESERVED_MFR_3D9E */
	{0x3DA0, 2, 0x7C93}, /* RESERVED_MFR_3DA0 */
	{0x3DA2, 2, 0x846B}, /* RESERVED_MFR_3DA2 */
	{0x3DA4, 2, 0x8465}, /* RESERVED_MFR_3DA4 */
	{0x3DA6, 2, 0x46FF}, /* RESERVED_MFR_3DA6 */
	{0x3DA8, 2, 0xAA65}, /* RESERVED_MFR_3DA8 */
	{0x3DAA, 2, 0x9C79}, /* RESERVED_MFR_3DAA */
	{0x3DAC, 2, 0x4A00}, /* RESERVED_MFR_3DAC */
	{0x3DAE, 2, 0x2180}, /* RESERVED_MFR_3DAE */
	{0x3DB0, 2, 0x44AC}, /* RESERVED_MFR_3DB0 */
	{0x3DB2, 2, 0x7070}, /* RESERVED_MFR_3DB2 */
	{0x3DB4, 2, 0x2180}, /* RESERVED_MFR_3DB4 */
	{0x3DB6, 2, 0x0005}, /* RESERVED_MFR_3DB6 */
	{0x3DB8, 2, 0x108F}, /* RESERVED_MFR_3DB8 */
	{0x3DBA, 2, 0x0802}, /* RESERVED_MFR_3DBA */
	{0x3DBC, 2, 0x5248}, /* RESERVED_MFR_3DBC */
	{0x3DBE, 2, 0x801B}, /* RESERVED_MFR_3DBE */
	{0x3DC0, 2, 0x006F}, /* RESERVED_MFR_3DC0 */
	{0x3DC2, 2, 0x8269}, /* RESERVED_MFR_3DC2 */
	{0x3DC4, 2, 0x6A82}, /* RESERVED_MFR_3DC4 */
	{0x3DC6, 2, 0x5148}, /* RESERVED_MFR_3DC6 */
	{0x3DC8, 2, 0x5A80}, /* RESERVED_MFR_3DC8 */
	{0x3DCA, 2, 0x5902}, /* RESERVED_MFR_3DCA */
	{0x3DCC, 2, 0x8082}, /* RESERVED_MFR_3DCC */
	{0x3DCE, 2, 0x3060}, /* RESERVED_MFR_3DCE */
	{0x3DD0, 2, 0x8567}, /* RESERVED_MFR_3DD0 */
	{0x3DD2, 2, 0x5C20}, /* RESERVED_MFR_3DD2 */
	{0x3DD4, 2, 0x4880}, /* RESERVED_MFR_3DD4 */
	{0x3DD6, 2, 0x0284}, /* RESERVED_MFR_3DD6 */
	{0x3DD8, 2, 0x6084}, /* RESERVED_MFR_3DD8 */
	{0x3DDA, 2, 0x5C91}, /* RESERVED_MFR_3DDA */
	{0x3DDC, 2, 0x5980}, /* RESERVED_MFR_3DDC */
	{0x3DDE, 2, 0x5883}, /* RESERVED_MFR_3DDE */
	{0x3DE0, 2, 0x6462}, /* RESERVED_MFR_3DE0 */
	{0x3DE2, 2, 0x8056}, /* RESERVED_MFR_3DE2 */
	{0x3DE4, 2, 0x8162}, /* RESERVED_MFR_3DE4 */
	{0x3DE6, 2, 0x8422}, /* RESERVED_MFR_3DE6 */
	{0x3DE8, 2, 0x209C}, /* RESERVED_MFR_3DE8 */
	{0x3DEA, 2, 0x2220}, /* RESERVED_MFR_3DEA */
	{0x3DEC, 2, 0x514B}, /* RESERVED_MFR_3DEC */
	{0x3DEE, 2, 0x8110}, /* RESERVED_MFR_3DEE */
	{0x3DF0, 2, 0x0943}, /* RESERVED_MFR_3DF0 */
	{0x3DF2, 2, 0x9843}, /* RESERVED_MFR_3DF2 */
	{0x3DF4, 2, 0x8050}, /* RESERVED_MFR_3DF4 */
	{0x3DF6, 2, 0x8B51}, /* RESERVED_MFR_3DF6 */
	{0x3DF8, 2, 0x8D4B}, /* RESERVED_MFR_3DF8 */
	{0x3DFA, 2, 0x9063}, /* RESERVED_MFR_3DFA */
	{0x3DFC, 2, 0x8363}, /* RESERVED_MFR_3DFC */
	{0x3DFE, 2, 0x8422}, /* RESERVED_MFR_3DFE */
	{0x3E00, 2, 0x209C}, /* RESERVED_MFR_3E00 */
	{0x3E02, 2, 0x61D3}, /* RESERVED_MFR_3E02 */
	{0x3E04, 2, 0x1FB6}, /* RESERVED_MFR_3E04 */
	{0x3E06, 2, 0x20ED}, /* RESERVED_MFR_3E06 */
	{0x3E08, 2, 0x3809}, /* RESERVED_MFR_3E08 */
	{0x3E0A, 2, 0x524B}, /* RESERVED_MFR_3E0A */
	{0x3E0C, 2, 0x0014}, /* RESERVED_MFR_3E0C */
	{0x3E0E, 2, 0x4580}, /* RESERVED_MFR_3E0E */
	{0x3E10, 2, 0x4681}, /* RESERVED_MFR_3E10 */
	{0x3E12, 2, 0x3060}, /* RESERVED_MFR_3E12 */
	{0x3E14, 2, 0x9D30}, /* RESERVED_MFR_3E14 */
	{0x3E16, 2, 0x6083}, /* RESERVED_MFR_3E16 */
	{0x3E18, 2, 0x4645}, /* RESERVED_MFR_3E18 */
	{0x3E1A, 2, 0x0017}, /* RESERVED_MFR_3E1A */
	{0x3E1C, 2, 0x8170}, /* RESERVED_MFR_3E1C */
	{0x3E1E, 2, 0x7070}, /* RESERVED_MFR_3E1E */
	{0x3E20, 2, 0x7070}, /* RESERVED_MFR_3E20 */
	{0x3E22, 2, 0x7070}, /* RESERVED_MFR_3E22 */
	{0x3E24, 2, 0x7070}, /* RESERVED_MFR_3E24 */
	{0x3E26, 2, 0x7070}, /* RESERVED_MFR_3E26 */
	{0x3E28, 2, 0x7070}, /* RESERVED_MFR_3E28 */
	{0x3E2A, 2, 0x7070}, /* RESERVED_MFR_3E2A */
	{0x3E2C, 2, 0x7070}, /* RESERVED_MFR_3E2C */
	{0x3E2E, 2, 0x7070}, /* RESERVED_MFR_3E2E */
	{0x3E30, 2, 0x7070}, /* RESERVED_MFR_3E30 */
	{0x3E32, 2, 0x7070}, /* RESERVED_MFR_3E32 */
	{0x3E34, 2, 0x7070}, /* RESERVED_MFR_3E34 */
	{0x3E36, 2, 0x7070}, /* RESERVED_MFR_3E36 */
	{0x3E38, 2, 0x7070}, /* RESERVED_MFR_3E38 */
	{0x3E3A, 2, 0x7070}, /* RESERVED_MFR_3E3A */
	{0x3E3C, 2, 0x7070}, /* RESERVED_MFR_3E3C */
	{0x3E3E, 2, 0x7070}, /* RESERVED_MFR_3E3E */
	{0x3E40, 2, 0x7070}, /* RESERVED_MFR_3E40 */
	{0x3E42, 2, 0x7070}, /* RESERVED_MFR_3E42 */
	{0x3E44, 2, 0x7070}, /* RESERVED_MFR_3E44 */
	{0x3E46, 2, 0x7070}, /* RESERVED_MFR_3E46 */
	{0x3E48, 2, 0x7070}, /* RESERVED_MFR_3E48 */
	{0x3E4A, 2, 0x7070}, /* RESERVED_MFR_3E4A */
	{0x3E4C, 2, 0x7070}, /* RESERVED_MFR_3E4C */
	{0x3E4E, 2, 0x7070}, /* RESERVED_MFR_3E4E */
	{0x3E50, 2, 0x7070}, /* RESERVED_MFR_3E50 */
	{0x3E52, 2, 0x7070}, /* RESERVED_MFR_3E52 */
	{0x3E54, 2, 0x7070}, /* RESERVED_MFR_3E54 */
	{0x3E56, 2, 0x7070}, /* RESERVED_MFR_3E56 */
	{0x3E58, 2, 0x7070}, /* RESERVED_MFR_3E58 */
	{0x3E5A, 2, 0x7070}, /* RESERVED_MFR_3E5A */
	{0x3E5C, 2, 0x7070}, /* RESERVED_MFR_3E5C */
	{0x3E5E, 2, 0x7070}, /* RESERVED_MFR_3E5E */
	{0x3E60, 2, 0x7070}, /* RESERVED_MFR_3E60 */
	{0x3E62, 2, 0x7070}, /* RESERVED_MFR_3E62 */
	{0x3E64, 2, 0x7070}, /* RESERVED_MFR_3E64 */
	{0x3E66, 2, 0x7070}, /* RESERVED_MFR_3E66 */
	{0x3E68, 2, 0x7070}, /* RESERVED_MFR_3E68 */
	{0x3E6A, 2, 0x7070}, /* RESERVED_MFR_3E6A */
	{0x3E6C, 2, 0x7070}, /* RESERVED_MFR_3E6C */
	{0x3E6E, 2, 0x7070}, /* RESERVED_MFR_3E6E */
	{0x3E70, 2, 0x7070}, /* RESERVED_MFR_3E70 */
	{0x3E72, 2, 0x7070}, /* RESERVED_MFR_3E72 */
	{0x3E74, 2, 0x7070}, /* RESERVED_MFR_3E74 */
	{0x3E76, 2, 0x7070}, /* RESERVED_MFR_3E76 */
	{0x3E78, 2, 0x7070}, /* RESERVED_MFR_3E78 */
	{0x3E7A, 2, 0x7070}, /* RESERVED_MFR_3E7A */
	{0x3E7C, 2, 0x7070}, /* RESERVED_MFR_3E7C */
	{0x3E7E, 2, 0x7070}, /* RESERVED_MFR_3E7E */
	{0x3E80, 2, 0x7070}, /* RESERVED_MFR_3E80 */
	{0x3E82, 2, 0x7070}, /* RESERVED_MFR_3E82 */
	{0x3E84, 2, 0x7070}, /* RESERVED_MFR_3E84 */
	{0x3E86, 2, 0x7070}, /* RESERVED_MFR_3E86 */
	{0x3E88, 2, 0x7070}, /* RESERVED_MFR_3E88 */
	{0x3E8A, 2, 0x7070}, /* RESERVED_MFR_3E8A */
	{0x3E8C, 2, 0x7070}, /* RESERVED_MFR_3E8C */
	{0x3E8E, 2, 0x7070}, /* RESERVED_MFR_3E8E */
	{0x3E90, 2, 0x7070}, /* RESERVED_MFR_3E90 */
	{0x3E92, 2, 0x7070}, /* RESERVED_MFR_3E92 */
	{0x3E94, 2, 0x7070}, /* RESERVED_MFR_3E94 */
	{0x3E96, 2, 0x7070}, /* RESERVED_MFR_3E96 */
	{0x3E98, 2, 0x7070}, /* RESERVED_MFR_3E98 */
	{0x3E9A, 2, 0x7070}, /* RESERVED_MFR_3E9A */
	{0x3E9C, 2, 0x7070}, /* RESERVED_MFR_3E9C */
	{0x3E9E, 2, 0x7070}, /* RESERVED_MFR_3E9E */
	{0x3EA0, 2, 0x7070}, /* RESERVED_MFR_3EA0 */
	{0x3EA2, 2, 0x7070}, /* RESERVED_MFR_3EA2 */
	{0x3EA4, 2, 0x7070}, /* RESERVED_MFR_3EA4 */
	{0x3EA6, 2, 0x7070}, /* RESERVED_MFR_3EA6 */
	{0x3EA8, 2, 0x7070}, /* RESERVED_MFR_3EA8 */
	{0x3EAA, 2, 0x7070}, /* RESERVED_MFR_3EAA */
	{0x3EAC, 2, 0x7070}, /* RESERVED_MFR_3EAC */
	{0x3EAE, 2, 0x7070}, /* RESERVED_MFR_3EAE */
	{0x3EB0, 2, 0x7070}, /* RESERVED_MFR_3EB0 */
	{0x3EB2, 2, 0x7070}, /* RESERVED_MFR_3EB2 */

	/* PLL Setting */
	{0x0300, 2, 0x000C}, /* VT_PIX_CLK_DIV */
	{0x0302, 2, 0x0001}, /* VT_SYS_CLK_DIV */
	{0x0304, 2, 0x0101}, /* PRE_PLL_CLK_DIV2, PRE_PLL_CLK_DIV1 */
	{0x0306, 2, 0x2830}, /* PLL_MULTIPLIER2, PLL_MULTIPLIER1 */
	{0x0308, 2, 0x000A}, /* OP_PIX_CLK_DIV */
	{0x030A, 2, 0x0001}, /* OP_SYS_CLK_DIV */

	{0x31B0, 2, 0x0047}, /* FRAME_PREAMBLE */
	{0x31B2, 2, 0x0026}, /* LINE_PREAMBLE */
	{0x31B4, 2, 0x328C}, /* MIPI_TIMING_0 */
	{0x31B6, 2, 0x3308}, /* MIPI_TIMING_1 */
	{0x31B8, 2, 0x1C12}, /* MIPI_TIMING_2 */
	{0x31BA, 2, 0x1452}, /* MIPI_TIMING_3 */
	{0x31BC, 2, 0x8488}, /* MIPI_TIMING_4 */

	/* Group_Parameter Hold */
	{0x0104, 2, 0x0001},

	/* Mode Configuration */
	{0x0122, 2, 0x0A0A}, /* CCP_DATA_FORMAT */
	{0x31AE, 2, 0x0202}, /* SERIAL_FROMAT */

	{0x3004, 2, 0x0004}, /* X_ADDR_START */
	{0x3008, 2, 0x0234}, /* X_ADDR_END */
	{0x3002, 2, 0x0004}, /* Y_ADDR_START */
	{0x3006, 2, 0x0233}, /* Y_ADDR_END */
	{0x034C, 2, 0x0230}, /* X_OUTPUT_SIZE */
	{0x034E, 2, 0x0230}, /* Y_OUTPUT_SIZE */

	{0x300C, 2, 0x02EA}, /* LINE_LENGTH_PCK */
	{0x300A, 2, 0x04A6}, /* FRAME_LENGTH_LINES */
	{0x3040, 2, 0x0001}, /* Y_ODD_INC = 1 */
	{0x30A4, 2, 0x0001}, /* Y_EVEN_INC_ */
	{0x30AA, 2, 0x0000}, /* DIGITAL_HSKIP */
	{0x0400, 2, 0x0000}, /* SCALE_ENABLE */
	{0x0404, 2, 0x0010}, /* SCALE_M */
	{0x317A, 2, 0x416E}, /*
			      * SF_BIN_ENABLE0(FIELD_WR= ANALOG_CONTROL6,
			      *		PARALLEL_SHUTTER)
			      */
	{0x3F3C, 2, 0x0003}, /*
			      * SF_BIN_ENABLE1(FIELD_WR= ANALOG_CONTROL9,
			      *		ANA_BLOCKS_ENABLE)
			      */
	{0x3012, 2, 0x0441}, /* COARSE_INTEGRATION_TIME */
	{0x3730, 2, 0x0000}, /* FB_CTRL */
	{0x305E, 2, 0x2035}, /* GLOBAL_GAIN */

	/* Group_Parameter Hold */
	{0x0104, 2, 0x0000},
	{/* NULL terminated */}

#elif (ARX3A0_FRAME_RATE == 5)
	/* LOAD= PLL_360Fps_80MHz_80MHz_20MHz */
	{0x300, 2, 0xA},
	{0x302, 2, 0x1},
	{0x304, 2, 0x101},
	{0x306, 2, 0x2828},
	{0x308, 2, 0xA},
	{0x30A, 2, 0x1},

	/* LOAD= MIPI_TIMING_10bit */
	{0x31B0, 2, 0x47},
	{0x31B2, 2, 0x26},
	{0x31B4, 2, 0x328C},
	{0x31B6, 2, 0x32E8},
	{0x31B8, 2, 0x1C12},
	{0x31BA, 2, 0x1452},
	{0x31BC, 2, 0x8488},

	/* LOAD= Analog_Setup_Recommended */
	{0x3ED0, 2, 0x748},
	{0x3ED6, 2, 0x3136},
	{0x3EDC, 2, 0x1020},
	{0x3EDE, 2, 0x1D2A},
	{0x3EE0, 2, 0x282A},
	{0x3EE2, 2, 0x2821},
	{0x3EC8, 2, 0x401},
	{0x3ED2, 2, 0x3903},
	{0x3EC0, 2, 0x0011},
	{0x3ECA, 2, 0x826F},
	{0x3EBC, 2, 0xA8AA},
	{0x3EC4, 2, 0x1000},
	{0x3EBA, 2, 0x44},

	/* LOAD= Corrections_Recommended */
	{0x3ED0, 2, 0x745},
	{0x3ED4, 2, 0x16},
	{0x3EC6, 2, 0x80F2},
	{0x3ED8, 2, 0x55FF},
	{0x3EE6, 2, 0x8000},
	{0x30D2, 2, 0x0},
	{0x31E0, 2, 0x00F1},
	{0x31E6, 2, 0xA35F},
	{0x3180, 2, 0x9096},
	{0x3120, 2, 0x1},
	{0x301E, 2, 0x2A},

	/* LOAD= Pixel_Timing_Recommended_10bit */
	{0x3D00, 2, 0x0436},
	{0x3D02, 2, 0x435A},
	{0x3D04, 2, 0xFFFF},
	{0x3D06, 2, 0xFFFF},
	{0x3D08, 2, 0x2180},
	{0x3D0A, 2, 0x0005},
	{0x3D0C, 2, 0x108F},
	{0x3D0E, 2, 0x0802},
	{0x3D10, 2, 0x5248},
	{0x3D12, 2, 0x801B},
	{0x3D14, 2, 0x006F},
	{0x3D16, 2, 0x8269},
	{0x3D18, 2, 0x6A82},
	{0x3D1A, 2, 0x5148},
	{0x3D1C, 2, 0x5A80},
	{0x3D1E, 2, 0x5902},
	{0x3D20, 2, 0x8082},
	{0x3D22, 2, 0x3060},
	{0x3D24, 2, 0x8567},
	{0x3D26, 2, 0x5C20},
	{0x3D28, 2, 0x4880},
	{0x3D2A, 2, 0x0284},
	{0x3D2C, 2, 0x6084},
	{0x3D2E, 2, 0x5C91},
	{0x3D30, 2, 0x5980},
	{0x3D32, 2, 0x5883},
	{0x3D34, 2, 0x6462},
	{0x3D36, 2, 0x8056},
	{0x3D38, 2, 0x8162},
	{0x3D3A, 2, 0x8422},
	{0x3D3C, 2, 0x20A2},
	{0x3D3E, 2, 0x2220},
	{0x3D40, 2, 0x804B},
	{0x3D42, 2, 0x8110},
	{0x3D44, 2, 0x0943},
	{0x3D46, 2, 0x9243},
	{0x3D48, 2, 0x8050},
	{0x3D4A, 2, 0x9A4B},
	{0x3D4C, 2, 0x8563},
	{0x3D4E, 2, 0x8363},
	{0x3D50, 2, 0x8422},
	{0x3D52, 2, 0x20A2},
	{0x3D54, 2, 0x61C6},
	{0x3D56, 2, 0x6F99},
	{0x3D58, 2, 0x3009},
	{0x3D5A, 2, 0x1FF6},
	{0x3D5C, 2, 0x20ED},
	{0x3D5E, 2, 0x0874},
	{0x3D60, 2, 0x8230},
	{0x3D62, 2, 0x609B},
	{0x3D64, 2, 0x3060},
	{0x3D66, 2, 0x4600},
	{0x3D68, 2, 0x3783},
	{0x3D6A, 2, 0x7070},
	{0x3D6C, 2, 0x8040},
	{0x3D6E, 2, 0x4A44},
	{0x3D70, 2, 0x8003},
	{0x3D72, 2, 0x0086},
	{0x3D74, 2, 0x4588},
	{0x3D76, 2, 0x46BA},
	{0x3D78, 2, 0x0300},
	{0x3D7A, 2, 0xFFD7},
	{0x3D7C, 2, 0x4688},
	{0x3D7E, 2, 0x4588},
	{0x3D80, 2, 0x4492},
	{0x3D82, 2, 0x4A9B},
	{0x3D84, 2, 0x4070},
	{0x3D86, 2, 0x8040},
	{0x3D88, 2, 0x4AAD},
	{0x3D8A, 2, 0x0070},
	{0x3D8C, 2, 0xAE47},
	{0x3D8E, 2, 0x8547},
	{0x3D90, 2, 0xAD78},
	{0x3D92, 2, 0x6B85},
	{0x3D94, 2, 0x6A80},
	{0x3D96, 2, 0x6984},
	{0x3D98, 2, 0x6B8A},
	{0x3D9A, 2, 0x6B80},
	{0x3D9C, 2, 0x6980},
	{0x3D9E, 2, 0x6A85},
	{0x3DA0, 2, 0x7C93},
	{0x3DA2, 2, 0x846B},
	{0x3DA4, 2, 0x8465},
	{0x3DA6, 2, 0x46FF},
	{0x3DA8, 2, 0xAA65},
	{0x3DAA, 2, 0x9C79},
	{0x3DAC, 2, 0x4A00},
	{0x3DAE, 2, 0x2180},
	{0x3DB0, 2, 0x44AC},
	{0x3DB2, 2, 0x7070},
	{0x3DB4, 2, 0x2180},
	{0x3DB6, 2, 0x0005},
	{0x3DB8, 2, 0x108F},
	{0x3DBA, 2, 0x0802},
	{0x3DBC, 2, 0x5248},
	{0x3DBE, 2, 0x801B},
	{0x3DC0, 2, 0x006F},
	{0x3DC2, 2, 0x8269},
	{0x3DC4, 2, 0x6A82},
	{0x3DC6, 2, 0x5148},
	{0x3DC8, 2, 0x5A80},
	{0x3DCA, 2, 0x5902},
	{0x3DCC, 2, 0x8082},
	{0x3DCE, 2, 0x3060},
	{0x3DD0, 2, 0x8567},
	{0x3DD2, 2, 0x5C20},
	{0x3DD4, 2, 0x4880},
	{0x3DD6, 2, 0x0284},
	{0x3DD8, 2, 0x6084},
	{0x3DDA, 2, 0x5C91},
	{0x3DDC, 2, 0x5980},
	{0x3DDE, 2, 0x5883},
	{0x3DE0, 2, 0x6462},
	{0x3DE2, 2, 0x8056},
	{0x3DE4, 2, 0x8162},
	{0x3DE6, 2, 0x8422},
	{0x3DE8, 2, 0x209C},
	{0x3DEA, 2, 0x2220},
	{0x3DEC, 2, 0x514B},
	{0x3DEE, 2, 0x8110},
	{0x3DF0, 2, 0x0943},
	{0x3DF2, 2, 0x9843},
	{0x3DF4, 2, 0x8050},
	{0x3DF6, 2, 0x8B51},
	{0x3DF8, 2, 0x8D4B},
	{0x3DFA, 2, 0x9063},
	{0x3DFC, 2, 0x8363},
	{0x3DFE, 2, 0x8422},
	{0x3E00, 2, 0x209C},
	{0x3E02, 2, 0x61D3},
	{0x3E04, 2, 0x1FB6},
	{0x3E06, 2, 0x20ED},
	{0x3E08, 2, 0x3809},
	{0x3E0A, 2, 0x524B},
	{0x3E0C, 2, 0x0014},
	{0x3E0E, 2, 0x4580},
	{0x3E10, 2, 0x4681},
	{0x3E12, 2, 0x3060},
	{0x3E14, 2, 0x9D30},
	{0x3E16, 2, 0x6083},
	{0x3E18, 2, 0x4645},
	{0x3E1A, 2, 0x0017},
	{0x3E1C, 2, 0x8170},
	{0x3E1E, 2, 0x7070},
	{0x3E20, 2, 0x7070},
	{0x3E22, 2, 0x7070},
	{0x3E24, 2, 0x7070},
	{0x3E26, 2, 0x7070},
	{0x3E28, 2, 0x7070},
	{0x3E2A, 2, 0x7070},
	{0x3E2C, 2, 0x7070},
	{0x3E2E, 2, 0x7070},
	{0x3E30, 2, 0x7070},
	{0x3E32, 2, 0x7070},
	{0x3E34, 2, 0x7070},
	{0x3E36, 2, 0x7070},
	{0x3E38, 2, 0x7070},
	{0x3E3A, 2, 0x7070},
	{0x3E3C, 2, 0x7070},
	{0x3E3E, 2, 0x7070},
	{0x3E40, 2, 0x7070},
	{0x3E42, 2, 0x7070},
	{0x3E44, 2, 0x7070},
	{0x3E46, 2, 0x7070},
	{0x3E48, 2, 0x7070},
	{0x3E4A, 2, 0x7070},
	{0x3E4C, 2, 0x7070},
	{0x3E4E, 2, 0x7070},
	{0x3E50, 2, 0x7070},
	{0x3E52, 2, 0x7070},
	{0x3E54, 2, 0x7070},
	{0x3E56, 2, 0x7070},
	{0x3E58, 2, 0x7070},
	{0x3E5A, 2, 0x7070},
	{0x3E5C, 2, 0x7070},
	{0x3E5E, 2, 0x7070},
	{0x3E60, 2, 0x7070},
	{0x3E62, 2, 0x7070},
	{0x3E64, 2, 0x7070},
	{0x3E66, 2, 0x7070},
	{0x3E68, 2, 0x7070},
	{0x3E6A, 2, 0x7070},
	{0x3E6C, 2, 0x7070},
	{0x3E6E, 2, 0x7070},
	{0x3E70, 2, 0x7070},
	{0x3E72, 2, 0x7070},
	{0x3E74, 2, 0x7070},
	{0x3E76, 2, 0x7070},
	{0x3E78, 2, 0x7070},
	{0x3E7A, 2, 0x7070},
	{0x3E7C, 2, 0x7070},
	{0x3E7E, 2, 0x7070},
	{0x3E80, 2, 0x7070},
	{0x3E82, 2, 0x7070},
	{0x3E84, 2, 0x7070},
	{0x3E86, 2, 0x7070},
	{0x3E88, 2, 0x7070},
	{0x3E8A, 2, 0x7070},
	{0x3E8C, 2, 0x7070},
	{0x3E8E, 2, 0x7070},
	{0x3E90, 2, 0x7070},
	{0x3E92, 2, 0x7070},
	{0x3E94, 2, 0x7070},
	{0x3E96, 2, 0x7070},
	{0x3E98, 2, 0x7070},
	{0x3E9A, 2, 0x7070},
	{0x3E9C, 2, 0x7070},
	{0x3E9E, 2, 0x7070},
	{0x3EA0, 2, 0x7070},
	{0x3EA2, 2, 0x7070},
	{0x3EA4, 2, 0x7070},
	{0x3EA6, 2, 0x7070},
	{0x3EA8, 2, 0x7070},
	{0x3EAA, 2, 0x7070},
	{0x3EAC, 2, 0x7070},
	{0x3EAE, 2, 0x7070},
	{0x3EB0, 2, 0x7070},
	{0x3EB2, 2, 0x7070},
	{0x3EB4, 2, 0x7070},
	{0x0104, 2, 0x0001}, /* Group_Parameter Hold */
	{0x0344, 2, 0x0004}, /* X_ADDR_START */
	{0x0346, 2, 0x0004}, /* Y_ADDR_START */
	{0x0348, 2, 0x0233}, /* X_ADDR_END */
	{0x034A, 2, 0x0233}, /* Y_ADDR_END */
	{0x034C, 2, 0x0230}, /* X_OUTPUT_SIZE */
	{0x034E, 2, 0x0230}, /* Y_OUTPUT_SIZE */
	{0x3040, 2, 0x41},   /* Y_ODD_INC */
	{0x30A4, 2, 0x1},    /* Y_EVEN_INC */
	{0x342, 2, 0x2F8},   /* line_length_pck */
	{0x340, 2, 0x248},   /* frame_length_lines ??*/
	{0x3012, 2, 0x0107}, /* coarse_integration_time ?? */
	{0x112, 2, 0xA0A},   /* RAW10 */
	{0x202, 2, 0x366A},
	{0x340, 2, 0xA47B},
	{0x342, 2, 0x2F8},
	{0x344, 2, 0x4},
	{0x346, 2, 0x4},
	{0x348, 2, 0x233},
	{0x34A, 2, 0x233},
	{0x34C, 2, 0x230},
	{0x34E, 2, 0x230},
	{0x382, 2, 0x1},
	{0x386, 2, 0x1},
	{0x400, 2, 0x0},
	{0x402, 2, 0x0},
	{0x404, 2, 0x10},
	{0x3000, 2, 0x353},
	{0x3002, 2, 0x4},
	{0x3004, 2, 0x4},
	{0x3006, 2, 0x233},
	{0x3008, 2, 0x233},
	{0x300A, 2, 0xA47B},
	{0x300C, 2, 0x2F8},
	{0x3012, 2, 0x366A},
	{0x3018, 2, 0x0},
	{0x301A, 2, 0x1C},
	{0x301C, 2, 0x1},
	{0x301D, 2, 0x0},
	{0x301E, 2, 0x2A},
	{0x3021, 2, 0x0},
	{0x3022, 2, 0x0},
	{0x3023, 2, 0x0},
	{0x3026, 2, 0xFFFF},
	{0x3028, 2, 0x4},
	{0x3032, 2, 0x100},
	{0x303A, 2, 0xA},
	{0x303B, 2, 0xF7},
	{0x303C, 2, 0x0},
	{0x303E, 2, 0x0},
	{0x3040, 2, 0x41},
	{0x3044, 2, 0x10C0},
	{0x3046, 2, 0x608},
	{0x3048, 2, 0x8},
	{0x304A, 2, 0x60},
	{0x304C, 2, 0x200},
	{0x305E, 2, 0x2000},
	{0x3064, 2, 0x5840},
	{0x3068, 2, 0x0},
	{0x306A, 2, 0x0},
	{0x306E, 2, 0x9000},
	{0x3070, 2, 0x0},
	{0x3072, 2, 0x0},
	{0x3074, 2, 0x0},
	{0x3076, 2, 0x0},
	{0x3078, 2, 0x0},
	{0x307A, 2, 0x0},
	{0x307E, 2, 0x20},
	{0x3088, 2, 0x1},
	{0x30A0, 2, 0x1},
	{0x30A2, 2, 0x1},
	{0x30A4, 2, 0x1},
	{0x30A6, 2, 0x1},
	{0x30AA, 2, 0x0},
	{0x30B0, 2, 0x400},
	{0x30BC, 2, 0x0},
	{0x30BE, 2, 0x0},
	{0x30C0, 2, 0x2000},
	{0x30C2, 2, 0x0},
	{0x30E8, 2, 0x0},
	{0x30EA, 2, 0x0},
	{0x30EC, 2, 0x5AE7},
	{0x30F8, 2, 0x33},
	{0x30FA, 2, 0xFC4C},
	{0x3120, 2, 0x1},
	{0x3122, 2, 0x7},
	{0x3124, 2, 0x1A7},
	{0x3126, 2, 0x0},
	{0x3128, 2, 0x1CF},
	{0x312A, 2, 0x4567},
	{0x312C, 2, 0x89AB},
	{0x312E, 2, 0xCDEF},
	{0x3152, 2, 0x10},
	{0x3154, 2, 0x3200},
	{0x3156, 2, 0xC8F7},
	{0x3158, 2, 0x0},
	{0x315A, 2, 0x0},
	{0x315C, 2, 0x0},
	{0x315E, 2, 0x0},
	{0x3160, 2, 0xEC},
	{0x3162, 2, 0x317},
	{0x3164, 2, 0x0},
	{0x0104, 2, 0x0000}, /* Group_Parameter Hold */
	{/* NULL terminated */}
#else
#error "FPS Configuration not found"
#endif /* (ARX3A0_FRAME_RATE == 90) */
};

static inline int i2c_burst_read16_dt(const struct i2c_dt_spec *spec, uint16_t start_addr,
				      uint8_t *buf, uint32_t num_bytes)
{
	uint8_t addr_buffer[2];

	addr_buffer[1] = start_addr & 0xFF;
	addr_buffer[0] = start_addr >> 8;
	return i2c_write_read_dt(spec, addr_buffer, sizeof(addr_buffer), buf, num_bytes);
}

static inline int i2c_burst_write16_dt(const struct i2c_dt_spec *spec, uint16_t start_addr,
				       const uint8_t *buf, uint32_t num_bytes)
{
	uint8_t addr_buffer[2];
	struct i2c_msg msg[2];
	int ret;

	addr_buffer[1] = start_addr & 0xFF;
	addr_buffer[0] = start_addr >> 8;
	msg[0].buf = addr_buffer;
	msg[0].len = 2U;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)buf;
	msg[1].len = num_bytes;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	/*
	 * Ideally we should send data with a callback API for I2C. But since
	 * it is not implemented for DWC I2C driver, providing a hacky solution
	 * of adding a 1ms delay as a wait period for transfer to complete.
	 */
	ret = i2c_transfer_dt(spec, msg, 2);

	k_msleep(1);
	return ret;
}

static int arx3a0_write_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_size,
			    void *value)
{
	const struct arx3a0_config *cfg = dev->config;

	switch (reg_size) {
	case 1:
		break;
	case 2:
		*(uint16_t *)value = sys_cpu_to_be16(*(uint16_t *)value);
		break;
	case 4:
		*(uint32_t *)value = sys_cpu_to_be32(*(uint32_t *)value);
		break;
	default:
		return -ENOTSUP;
	}

	return i2c_burst_write16_dt(&cfg->i2c, reg_addr, value, reg_size);
}

static int arx3a0_read_reg(const struct device *dev, uint16_t reg_addr, uint8_t reg_size,
			   void *value)
{
	const struct arx3a0_config *cfg = dev->config;
	int err;

	if (reg_size > 4) {
		return -ENOTSUP;
	}

	err = i2c_burst_read16_dt(&cfg->i2c, reg_addr, value, reg_size);
	if (err) {
		return err;
	}

	switch (reg_size) {
	case 1:
		break;
	case 2:
		*(uint16_t *)value = sys_be16_to_cpu(*(uint16_t *)value);
		break;
	case 4:
		*(uint32_t *)value = sys_be32_to_cpu(*(uint32_t *)value);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int arx3a0_write_all(const struct device *dev, struct arx3a0_reg *reg)
{
	int i = 0;

	while (reg[i].value_size) {
		int err;

		err = arx3a0_write_reg(dev, reg[i].addr, reg[i].value_size, &reg[i].value);
		if (err) {
			LOG_ERR("Failed to write R0x%04x register. ret - %d", reg[i].addr, err);
			return err;
		}

		i++;
	}

	return 0;
}

static int arx3a0_set_output_format(const struct device *dev, int pixel_format)
{
	int ret = 0;
	uint16_t output_format;

	if (pixel_format != VIDEO_PIX_FMT_Y10P) {
		LOG_ERR("Image format not supported");
		return -ENOTSUP;
	}
	output_format = ARX3A0_CAM_OUTPUT_FORMAT_RAW10;

	ret = arx3a0_write_reg(dev, ARX3A0_CAM_OUTPUT_FORMAT, sizeof(output_format),
			       &output_format);

	return ret;
}

static int arx3a0_set_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct arx3a0_data *drv_data = dev->data;
	int ret;

	LOG_DBG("ARX3A0 Set Format. fmt - %c%c%c%c", (char)fmt->pixelformat,
		(char)(fmt->pixelformat >> 8), (char)(fmt->pixelformat >> 16),
		(char)(fmt->pixelformat >> 24));
	if (fmt->pixelformat != VIDEO_PIX_FMT_Y10P) {
		LOG_ERR("Unsupported output pixel format");
		return -ENOTSUP;
	}

	/* we only support one format size for now */
	if (fmt->height != 560 || fmt->width != 560) {
		LOG_ERR("Unsupported output size format");
		return -ENOTSUP;
	}

	if (!memcmp(&drv_data->fmt, fmt, sizeof(drv_data->fmt))) {
		/* nothing to do */
		return 0;
	}

	drv_data->fmt = *fmt;

	/* Configure Sensor */
	ret = arx3a0_write_all(dev, arx3a0_560_regs);
	if (ret) {
		LOG_ERR("Unable to write arx3a0 config. ret - %d", ret);
		return ret;
	}

	/* Set output format */
	ret = arx3a0_set_output_format(dev, fmt->pixelformat);
	if (ret) {
		LOG_ERR("Unable to set output format. ret - %d", ret);
		return ret;
	}
	drv_data->is_streaming = false;

	return 0;
}

static int arx3a0_get_fmt(const struct device *dev, enum video_endpoint_id ep,
			  struct video_format *fmt)
{
	struct arx3a0_data *drv_data = dev->data;

	*fmt = drv_data->fmt;

	return 0;
}

static int arx3a0_stream_start(const struct device *dev)
{
	struct arx3a0_data *data = dev->data;
	uint16_t val;
	int ret;

	if (data->is_streaming) {
		LOG_DBG("Already Streaming!");
		return 0;
	}

	val = 1;
	ret = arx3a0_write_reg(dev, ARX3A0_MODE_SELECT_REGISTER, 1, &val);
	if (ret) {
		LOG_ERR("Failed to start Stream!");
		return ret;
	}

	LOG_DBG("Stream Started");
	data->is_streaming = true;

	return 0;
}

static int arx3a0_stream_stop(const struct device *dev)
{
	struct arx3a0_data *data = dev->data;
	uint16_t val;
	int ret;

	if (!data->is_streaming) {
		LOG_DBG("Already Stopped!");
		return 0;
	}

	val = 0;
	ret = arx3a0_write_reg(dev, ARX3A0_MODE_SELECT_REGISTER, 1, &val);
	if (ret) {
		LOG_ERR("Failed to stop streaming!");
		return ret;
	}

	LOG_DBG("Stream Stopped");
	data->is_streaming = false;

	return 0;
}

static int arx3a0_set_stream(const struct device *dev, bool enable)
{
	if (enable) {
		return arx3a0_stream_start(dev);
	} else {
		return arx3a0_stream_stop(dev);
	}
}

#define ARX3A0_VIDEO_FORMAT_CAP(width, height, format)                                             \
	{                                                                                          \
		.pixelformat = (format), .width_min = (width), .width_max = (width),               \
		.height_min = (height), .height_max = (height), .width_step = 0, .height_step = 0  \
	}

static const struct video_format_cap fmts[] = {
	ARX3A0_VIDEO_FORMAT_CAP(560, 560, VIDEO_PIX_FMT_Y10P), /* RAW10 MIPI */
	{0}};

static int arx3a0_get_caps(const struct device *dev, enum video_endpoint_id ep,
			   struct video_caps *caps)
{
	caps->format_caps = fmts;
	return 0;
}

static int arx3a0_set_camera_gain(const struct device *dev, uint32_t gain)
{
	static uint32_t current_integration_time;
	static uint32_t max_integration_time;

	uint32_t resulting_gain;
	uint32_t digital_gain;
	uint32_t fine_gain = gain;
	uint32_t coarse_gain;
	uint32_t val;
	int ret;

	if (max_integration_time == 0) {
		uint32_t reset;

		ret = arx3a0_read_reg(dev, ARX3A0_COARSE_INTEGRATION_TIME_REGISTER, 2,
				      &current_integration_time);
		if (ret) {
			return ret;
		}
		max_integration_time = current_integration_time;

		reset = 0;
		ret = arx3a0_read_reg(dev, ARX3A0_RESET_REGISTER, 2, &reset);
		if (ret) {
			return ret;
		}

		/* Unlock the data pedestal setting. */
		val = reset & (~BIT(3));
		ret = arx3a0_write_reg(dev, ARX3A0_RESET_REGISTER, 2, &val);
		if (ret) {
			return ret;
		}

		/* Set the Data-Pedestal register value to zero. */
		val = 0;
		ret = arx3a0_write_reg(dev, ARX3A0_DATA_PEDESTAL_REGISTER, 2, &val);
		if (ret) {
			return ret;
		}

		/* Restore the lock. */
		ret = arx3a0_write_reg(dev, ARX3A0_RESET_REGISTER, 2, &reset);
		if (ret) {
			return ret;
		}
	}

	if (gain != 0) {
		/* Request to set gain */
		/*
		 * From the Design Guide:
		 * Total Gain = (R0x305E[0:3]/16 + 1) * 2^R0x0305E[4:6] *
		 *	(R0x305E[7:15)/64
		 *
		 * First clamp analogue gain, using digital gain to get more if
		 * necessary. Otherwise digital gain is used to fine adjust.
		 */
		uint32_t new_integration_time = max_integration_time;

		if (gain < 0x10000) {
			/* Minimum gain is 1.0 */
			fine_gain = 0x10000;

			new_integration_time =
				(uint32_t)(((float)max_integration_time * gain) * 0x1p-16f + 0.5f);
		} else if (gain > 0x80000) {
			/* Maximum gain is 8.0 */
			fine_gain = 0x80000;
		}

		/* Set integration time */
		if (new_integration_time != current_integration_time) {
			ret = arx3a0_write_reg(dev, ARX3A0_COARSE_INTEGRATION_TIME_REGISTER, 2,
					       &new_integration_time);
			if (ret) {
				return ret;
			}
			current_integration_time = new_integration_time;
		}

		/*
		 * Get coarse analogue power of two, leaving fine gain in
		 * [0x10000, 0x1FFFF].
		 */
		coarse_gain = 0;
		while (fine_gain >= 0x20000) {
			coarse_gain++;
			fine_gain /= 2;
		}

		/* Round down to 16 steps of fine gain. */
		fine_gain = (fine_gain - 0x10000) / 0x1000;

		/*
		 * Use digital gain to extend gain beyond the analogue limits of
		 * x1 to x8, or to fine-tune within that range.
		 *
		 * We don't let digital gain go below 1.0 - it just loses
		 * information, and clamping it lets an auto-gain controller see
		 * that we are unable to improve exposure by such lowering.
		 * Another camera might be able to usefully set gain to <1.0, so
		 * a controller could try it.
		 *
		 * (When we're fine tuning, digital gain is always >= 1.0,
		 * because we round down analogue gain, so it can only go below
		 * 1.0 by the user requesting total gain < 1.0).
		 */
		val = ((fine_gain + 16) << coarse_gain) * 0x1000;
		digital_gain = (64 * gain + (val / 2)) / val;

		if (digital_gain > 0x1FF) {
			/* Maximum digital gain is just under 8.0*/
			digital_gain = 0x1FF;
		} else if (digital_gain < 64) {
			/* Digital gain >= 1.0, as per discussion above. */
			digital_gain = 64;
		}

		val = (digital_gain << 7) | (coarse_gain << 4) | fine_gain;
		ret = arx3a0_write_reg(dev, ARX3A0_GLOBAL_GAIN_REGISTER, 2, &val);
		if (ret) {
			return ret;
		}
	} else {
		ret = arx3a0_read_reg(dev, ARX3A0_GLOBAL_GAIN_REGISTER, 2, &val);
		if (ret) {
			return ret;
		}

		digital_gain = val >> 7;
		coarse_gain = (val >> 4) & 7;
		fine_gain = val & 0xf;
	}

	resulting_gain = ((fine_gain + 16) << coarse_gain) * digital_gain * 64;
	if (current_integration_time != max_integration_time) {
		resulting_gain = (uint32_t)(((float)resulting_gain * current_integration_time) /
						    max_integration_time +
					    0.5f);
	}

	ARG_UNUSED(resulting_gain);

	return 0;
}

static int arx3a0_get_camera_gain(const struct device *dev, uint32_t *gain)
{
	uint32_t val;
	uint32_t digital_gain;
	uint32_t coarse_gain;
	uint32_t fine_gain;
	int ret;

	ret = arx3a0_read_reg(dev, ARX3A0_GLOBAL_GAIN_REGISTER, 2, &val);
	if (ret) {
		return ret;
	}

	/* Register is 16-bit; keep only valid bits. */
	val &= 0xFFFFU;

	/* ARX3A0_GLOBAL_GAIN_REGISTER bitfields:
	 * [15:7] digital gain (9 bits)
	 * [6:4]  coarse analog gain (3 bits)
	 * [3:0]  fine analog gain (4 bits)
	 */
	digital_gain = (val >> 7) & 0x1FFU;
	coarse_gain = (val >> 4) & 0x07U;
	fine_gain = val & 0x0FU;

	*gain = ((fine_gain + 16U) << coarse_gain) * digital_gain * 64U;

	return 0;
}

static int arx3a0_set_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid) {
	case VIDEO_CID_GAIN:
		return arx3a0_set_camera_gain(dev, (uint32_t)value);
	default:
		return -ENOTSUP;
	}
}

static int arx3a0_get_ctrl(const struct device *dev, unsigned int cid, void *value)
{
	switch (cid) {
	case VIDEO_CID_GAIN:
		return arx3a0_get_camera_gain(dev, (uint32_t *)value);
	default:
		return -ENOTSUP;
	}
}

static DEVICE_API(video, arx3a0_driver_api) = {
	.set_format = arx3a0_set_fmt,
	.get_format = arx3a0_get_fmt,
	.get_caps = arx3a0_get_caps,
	.set_stream = arx3a0_set_stream,
	.set_ctrl = arx3a0_set_ctrl,
	.get_ctrl = arx3a0_get_ctrl,
};

static int arx3a0_hard_reseten(const struct device *dev)
{
	const struct arx3a0_config *cfg = dev->config;
	int ret;

	/* Following the GPIO reset sequence as per the datasheet. */
	if (cfg->reset_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure reset GPIO. ret - %d", ret);
			return ret;
		}
	}

	if (cfg->power_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Could not configure power GPIO. ret - %d", ret);
			return ret;
		}
	}

	if (cfg->reset_gpio.port) {
		gpio_pin_set_dt(&cfg->reset_gpio, 0);
		k_msleep(2);
	}

	if (cfg->power_gpio.port) {
		gpio_pin_set_dt(&cfg->power_gpio, 1);
		k_msleep(1);
	}

	if (cfg->reset_gpio.port) {
		gpio_pin_set_dt(&cfg->reset_gpio, 1);
	}

	/* no power control, wait for camera ready */
	k_msleep(100);

	return 0;
}

static int arx3a0_soft_reseten(const struct device *dev)
{
	uint16_t value = 1;
	int ret;

	ret = arx3a0_write_reg(dev, ARX3A0_SOFTWARE_RESET_REGISTER, 1, &value);
	if (ret) {
		return ret;
	}

	k_msleep(1);

	return 0;
}

static int arx3a0_init(const struct device *dev)
{
	const struct arx3a0_config *config = dev->config;
	struct arx3a0_data *data = dev->data;
	uint16_t val;
	int ret;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("Bus device is not ready");
		return -ENODEV;
	}

	ret = arx3a0_hard_reseten(dev);
	if (ret) {
		LOG_ERR("Failed to Hard-Reset camera sensor. ret - %d", ret);
		return ret;
	}

	ret = arx3a0_soft_reseten(dev);
	if (ret) {
		LOG_ERR("Failed to Soft-Reset camera sensor. ret - %d", ret);
		return ret;
	}

	ret = arx3a0_read_reg(dev, ARX3A0_CHIP_ID, sizeof(val), &val);
	if (ret) {
		LOG_ERR("Unable to read chip ID. ret - %d", ret);
		return -ENODEV;
	}

	if (val != ARX3A0_CHIP_ID_VAL) {
		LOG_ERR("Wrong ID: %04x (exp %04x)", val, ARX3A0_CHIP_ID_VAL);
		return -ENODEV;
	}

	/* Put sensor in standby mode. */
	val = 0;
	ret = arx3a0_write_reg(dev, ARX3A0_MODE_SELECT_REGISTER, 1, &val);
	if (ret) {
		LOG_ERR("Failed to put sensor in standby. ret - %d", ret);
		return ret;
	}

	/* Enable LP11 state on STANDBY mode. */
	ret = arx3a0_read_reg(dev, ARX3A0_MIPI_CONFIG_REGISTER, 2, &val);
	if (ret) {
		LOG_ERR("Failed to read MIPI Configuration register. "
			"ret - %d",
			ret);
		return ret;
	}
	val |= ARX3A0_MIPI_CONFIG_LP11_ON_STANDBY;
	ret = arx3a0_write_reg(dev, ARX3A0_MIPI_CONFIG_REGISTER, 2, &val);
	if (ret) {
		LOG_ERR("Failed to write to MIPI Configuration register. "
			"ret - %d",
			ret);
		return ret;
	}

	/* start streaming. */
	val = 1;
	ret = arx3a0_write_reg(dev, ARX3A0_MODE_SELECT_REGISTER, 1, &val);
	if (ret) {
		LOG_ERR("Failed to start any stream. ret - %d", ret);
		return ret;
	}

	k_msleep(50);

	/* Suspend any stream */
	val = 0;
	ret = arx3a0_write_reg(dev, ARX3A0_MODE_SELECT_REGISTER, 1, &val);
	if (ret) {
		LOG_ERR("Failed to suspend any stream. ret - %d", ret);
		return ret;
	}

	data->is_streaming = false;
	k_msleep(500);

	return 0;
}

#define ARX3A0_DEVICE_DEFINE(i)                                                 \
	static const struct arx3a0_config arx3a0_cfg_##i = {                    \
		.i2c = I2C_DT_SPEC_INST_GET(i),                                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(i, reset_gpios, {}),     \
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(i, power_gpios, {}),     \
	};                                                                      \
                                                                                \
static struct arx3a0_data arx3a0_data_##i;                                      \
                                                                                \
DEVICE_DT_INST_DEFINE(i, &arx3a0_init, NULL, &arx3a0_data_##i, &arx3a0_cfg_##i, \
		POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &arx3a0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ARX3A0_DEVICE_DEFINE)
