/*
 * Copyright (C) 2026 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_VIDEO_ISP_CTRL_PARAMS_H__
#define ZEPHYR_INCLUDE_DRIVERS_VIDEO_ISP_CTRL_PARAMS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file isp_ctrl_params.h
 * @brief Application-facing ISP parameter structs.
 *
 * These structs are passed via video_set_ctrl() / video_get_ctrl() using the
 * VIDEO_CID_ALIF_ISP_* control IDs defined in video_alif.h.
 * No ISP library headers are required by the application.
 */

/** @brief ISP operation mode. */
enum isp_op_mode {
	ISP_OP_AUTO   = 0,
	ISP_OP_MANUAL = 1,
};

/* --------------------------------------------------------------------------
 * AE (Auto Exposure)
 * --------------------------------------------------------------------------
 */

/** @brief One node of the AE exposure route table. */
struct isp_ae_route_node {
	uint32_t int_time; /**< Integration time (us) */
	uint32_t again;    /**< Analog gain (1x = 1024) */
	uint32_t dgain;    /**< Digital gain (1x = 256) */
};

/** @brief Maximum number of AE route nodes (ISP_AE_ROUTE_MAX_NODES). */
#define ISP_AE_ROUTE_MAX_NODES 16

/** @brief AE operating mode (maps to ISP_AE_MODE_E). */
enum isp_ae_mode {
	ISP_AE_MODE_FIX_FRAME_RATE = 0, /**< Fixed frame rate */
	ISP_AE_MODE_SLOW_SHUTTER   = 1, /**< Slow shutter (not currently supported) */
};

/** @brief AE scene-change delay (maps to ISP_AE_DELAY_S). */
struct isp_ae_delay {
	uint16_t black_delay_frame; /**< Delay frames when scene darkens */
	uint16_t white_delay_frame; /**< Delay frames when scene gets brighter */
};

/** @brief Exposure (AE) parameters. */
struct isp_ae_param {
	/** ISP_OP_AUTO or ISP_OP_MANUAL */
	uint8_t op_mode;

	/** Manual mode: integration time (microseconds) */
	uint32_t int_time;
	/** Manual mode: analog gain (sensor-specific fixed-point, 1x = 1024) */
	uint32_t again;
	/** Manual mode: digital gain (sensor-specific fixed-point, 1x = 256) */
	uint32_t dgain;

	/** Auto mode: target luminance [0, 255] */
	uint8_t  ae_target;
	/** Auto mode: convergence damping when over target [0, 255] */
	uint8_t  damp_over;
	/** Auto mode: convergence damping when under target [0, 255] */
	uint8_t  damp_under;
	/** Auto mode: tolerance band to stop converging [0, 100] */
	uint8_t  tolerance;
	/** Auto mode: AE run interval in frames (default 1) */
	uint8_t  run_interval;

	/** Auto mode: min/max integration time (us) */
	uint32_t int_time_min;
	uint32_t int_time_max;
	/** Auto mode: min/max analog gain */
	uint32_t again_min;
	uint32_t again_max;
	/** Auto mode: min/max digital gain */
	uint32_t dgain_min;
	uint32_t dgain_max;

	/** Auto mode: antiflicker — 0: disable, 1: enable */
	uint8_t  antiflicker_enable;
	/** Auto mode: antiflicker frequency (Hz, e.g. 50 or 60) */
	uint32_t antiflicker_freq;

	/**
	 * Auto mode: AE exposure route table.
	 * Defines the priority order in which int_time / again / dgain are
	 * increased. Set ae_route_total_nodes = 0 to disable (use defaults).
	 */
	uint32_t  ae_route_total_nodes;
	struct isp_ae_route_node ae_route[ISP_AE_ROUTE_MAX_NODES];

	/** Auto mode: gain threshold for route switching (not currently supported) */
	uint32_t gain_threshold;
	/** Auto mode: AE operating mode */
	enum isp_ae_mode ae_mode;
	/** Auto mode: scene-change delay (not currently supported) */
	struct isp_ae_delay ae_delay_attr;
	/** Auto mode: 5x5 luminance weight table */
	uint8_t  weight[5][5];
};

/* --------------------------------------------------------------------------
 * WB (White Balance)
 * --------------------------------------------------------------------------
 */

/** @brief AWB center-line calibration data. */
struct isp_awb_center_line {
	int32_t rg_param;   /**< R/G ratio of center line */
	int32_t bg_param;   /**< B/G ratio of center line */
	int32_t dist_param; /**< Distance parameter */
};

/** @brief AWB white-pixel curve (16-point, used for left/right edge of orange box). */
struct isp_awb_wp_curve {
	int32_t rg[16];   /**< Horizontal coordinate projections */
	int32_t dist[16]; /**< Distances from sampling point to Rg */
};

/** @brief AWB white-pixel range (left + right curve). */
struct isp_awb_wp_range {
	struct isp_awb_wp_curve left;
	struct isp_awb_wp_curve right;
};

/** @brief Illuminant type index (maps to ISP_ILLUMINANT_TYPE_E). */
enum isp_illuminant_type {
	ISP_ILLUMINANT_A    = 0,
	ISP_ILLUMINANT_TL84 = 1,
	ISP_ILLUMINANT_CWF  = 2,
	ISP_ILLUMINANT_D50  = 3,
	ISP_ILLUMINANT_D65  = 4,
	ISP_ILLUMINANT_COUNT,
};

/** @brief AWB gain for one illuminant. */
struct isp_awb_illuminant {
	enum isp_illuminant_type illu_type; /**< Illuminant type index */
	uint32_t color_temp; /**< Color temperature (K) */
	uint16_t r_gain;     /**< Red gain [256..1023] */
	uint16_t gr_gain;    /**< Green-red gain [256..1023] */
	uint16_t gb_gain;    /**< Green-blue gain [256..1023] */
	uint16_t b_gain;     /**< Blue gain [256..1023] */
};

/** @brief Full AWB calibration data embedded in auto WB params. */
struct isp_awb_calib {
	struct isp_awb_center_line center_line;
	int32_t rg_min; /**< Minimum Rg in indoor scenes */
	int32_t rg_max; /**< Maximum Rg boundary of orange box */
	struct isp_awb_wp_range wp_range0;
	struct isp_awb_wp_range wp_range1;
	/** Per-illuminant gain table (A, TL84, CWF, D50, D65) */
	struct isp_awb_illuminant illuminant[ISP_ILLUMINANT_COUNT];
};

/** @brief White balance gain parameters. */
struct isp_wb_param {
	/** 0: disable WB, 1: enable */
	uint8_t  enable;
	/** ISP_OP_AUTO or ISP_OP_MANUAL */
	uint8_t op_mode;

	/** Manual mode: red channel gain (fixed-point, 1x = 256) */
	uint16_t r_gain;
	/** Manual mode: green red channel gain (fixed-point, 1x = 256) */
	uint16_t gr_gain;
	/** Manual mode: green blue channel gain (fixed-point, 1x = 256) */
	uint16_t gb_gain;
	/** Manual mode: blue channel gain (fixed-point, 1x = 256) */
	uint16_t b_gain;

	/** Auto mode: number of interval frames to run AWB (default 1) */
	uint8_t  run_interval;
	/** Auto mode: convergence speed [0..256] */
	uint16_t speed;
	/** Auto mode: tolerance band to stop converging [0..100] */
	uint8_t  tolerance;
	/** Auto mode: initial color temperature (K) */
	uint32_t init_color_temp;
	/** Auto mode: full AWB calibration data */
	struct isp_awb_calib calib;
};

/* --------------------------------------------------------------------------
 * BLS (Black Level Subtraction)
 * --------------------------------------------------------------------------
 */

/** @brief Number of auto-strength entries in BLS/FLT auto tables. */
#define ISP_AUTO_STRENGTH_NUM 16

/** @brief Black level subtraction parameters. */
struct isp_bls_param {
	/** 0: disable BLS, 1: enable */
	uint8_t  enable;
	/** ISP_OP_AUTO or ISP_OP_MANUAL */
	uint8_t op_mode;
	/**
	 * Manual mode: black level per Bayer channel [0, 4095]
	 * Index: 0=R, 1=Gb, 2=Gr, 3=B
	 */
	int32_t  black_level[4];
	/**
	 * Auto mode: black level table indexed by ISO/gain strength.
	 * auto_black_level[i][0..3] = R, Gb, Gr, B levels for strength i.
	 * Range [0, 4095], 16 entries (ISP_AUTO_STRENGTH_NUM).
	 */
	uint32_t auto_black_level[ISP_AUTO_STRENGTH_NUM][4];
};

/* --------------------------------------------------------------------------
 * DMSC (Demosaicing)
 * --------------------------------------------------------------------------
 */

/** @brief Demosaicing parameters. */
struct isp_dmsc_param {
	/** 0: disable DMSC, 1: enable */
	uint8_t  enable;
	/**
	 * Texture detection threshold [0, 255].
	 * 0x00 = maximum edge sensitivity; 0xFF = no texture detection.
	 */
	uint8_t  threshold;

	/** CAC (Chromatic Aberration Correction) enable */
	uint8_t  cac_enable;
	/** CAC horizontal clip mode [0, 1] */
	uint8_t  cac_h_clip_mode;
	/** CAC vertical clip mode [0, 2] */
	uint8_t  cac_v_clip_mode;
	/** CAC h_count preload [1, 4095] */
	uint16_t cac_h_start;
	/** CAC v_count preload [1, 4095] */
	uint16_t cac_v_start;
	/** CAC coefficients A/B/C for blue and red (4-bit fractional) */
	int16_t  cac_a_blue;
	int16_t  cac_a_red;
	int16_t  cac_b_blue;
	int16_t  cac_b_red;
	int16_t  cac_c_blue;
	int16_t  cac_c_red;
	/** CAC normalization shift/factor for X and Y axes */
	uint8_t  cac_x_norm_shift;
	uint8_t  cac_x_norm_factor;
	uint8_t  cac_y_norm_shift;
	uint8_t  cac_y_norm_factor;
};

/* --------------------------------------------------------------------------
 * FLT (Filter: Denoise + Sharpen)
 * --------------------------------------------------------------------------
 */

/** @brief Filter (denoise + sharpen) parameters. */
struct isp_flt_param {
	/** 0: disable filter, 1: enable */
	uint8_t  enable;
	/** ISP_OP_AUTO or ISP_OP_MANUAL */
	uint8_t op_mode;
	/** Manual mode: denoise level [0, 10] */
	uint8_t  denoise_level;
	/** Manual mode: sharpen level [0, 10] */
	uint8_t  sharpen_level;
	/**
	 * Auto mode: per-strength denoise level table [0, 10].
	 * 16 entries (ISP_AUTO_STRENGTH_NUM), indexed by ISP gain step.
	 */
	uint8_t  auto_denoise_level[ISP_AUTO_STRENGTH_NUM];
	/**
	 * Auto mode: per-strength sharpen level table [0, 10].
	 * 16 entries (ISP_AUTO_STRENGTH_NUM), indexed by ISP gain step.
	 */
	uint8_t  auto_sharpen_level[ISP_AUTO_STRENGTH_NUM];
};

/* --------------------------------------------------------------------------
 * CCM (Color Correction Matrix)
 * --------------------------------------------------------------------------
 */

/** @brief CCM coefficients for one illuminant (used in auto CCM table). */
struct isp_ccm_illuminant {
	/** 3x3 color matrix [-1024..1023], 7-bit fractional (128 = 1.0) */
	int16_t  color_matrix[9];
	/** RGB offsets [-2048..2047] */
	int16_t  r_offset;
	int16_t  g_offset;
	int16_t  b_offset;
	/** Associated color temperature (K) [2000..10000] */
	uint32_t color_temp;
};

/** @brief Color correction matrix parameters. */
struct isp_ccm_param {
	/** ISP_OP_AUTO or ISP_OP_MANUAL */
	uint8_t op_mode;
	/**
	 * Manual mode: 3x3 color matrix coefficients, 7-bit fractional
	 * (range -1024..1023, where 128 = 1.0)
	 */
	int16_t  color_matrix[9];
	/** Manual mode: RGB offset coefficients (range -2048..2047) */
	int16_t  r_offset;
	int16_t  g_offset;
	int16_t  b_offset;
	/**
	 * Auto mode: per-illuminant CCM table.
	 * ISP_ILLUMINANT_COUNT entries (A, TL84, CWF, D50, D65).
	 * The ISP selects the matrix by interpolating between color temperatures.
	 */
	struct isp_ccm_illuminant auto_ccm[ISP_ILLUMINANT_COUNT];
};

/* --------------------------------------------------------------------------
 * GAMMA OUT
 * --------------------------------------------------------------------------
 */

/** @brief Gamma-out curve parameters. */
struct isp_gamma_param {
	/** 0: disable gamma, 1: enable */
	uint8_t  enable;
	/** 17-point gamma curve, values [0, 4095] */
	uint16_t gamma_y[17];
};

/* --------------------------------------------------------------------------
 * CSM (Color Space Matrix)
 * --------------------------------------------------------------------------
 */

/** @brief CSM standard type. */
enum isp_csm_type {
	ISP_CSM_BT601  = 0,
	ISP_CSM_BT709  = 1,
	ISP_CSM_USER   = 2,
};

/** @brief CSM range type. */
enum isp_csm_range {
	ISP_CSM_RANGE_LIMITED = 0,
	ISP_CSM_RANGE_FULL    = 1,
};

/** @brief Color space matrix parameters. */
struct isp_csm_param {
	enum isp_csm_type  type;
	enum isp_csm_range range;
	/**
	 * User-defined CSM coefficients (range -256..255, 8-bit fractional).
	 * Only used when type == ISP_CSM_USER.
	 */
	int16_t coef[9];
};

/* --------------------------------------------------------------------------
 * AEM (Auto Exposure Measurement)
 * --------------------------------------------------------------------------
 */

/** @brief AE measurement window parameters. */
struct isp_aem_param {
	/** 0: disable AEM, 1: enable */
	uint8_t  enable;
	/**
	 * Luminance calculation mode:
	 *   0: Y = 16 + 0.25R + 0.5G + 0.1094B
	 *   1: Y = (R + G + B) * 0.332
	 */
	uint8_t  alt_mode;
	/** Measurement window horizontal offset */
	uint16_t h_offs;
	/** Measurement window vertical offset */
	uint16_t v_offs;
	/** Measurement window horizontal block size */
	uint16_t h_size;
	/** Measurement window vertical block size */
	uint16_t v_size;
};

/* --------------------------------------------------------------------------
 * WBM (White Balance Measurement)
 * --------------------------------------------------------------------------
 */

/** @brief WBM measurement mode. */
enum isp_wbm_mode {
	ISP_WBM_MODE_YCBCR = 0,
	ISP_WBM_MODE_RGB   = 1,
};

/** @brief White balance measurement parameters. */
struct isp_wbm_param {
	/** 0: disable WBM, 1: enable */
	uint8_t           enable;
	/** Measurement mode: YCbCr or RGB */
	enum isp_wbm_mode meas_mode;
	/** Measurement window horizontal offset */
	uint16_t h_offs;
	/** Measurement window vertical offset */
	uint16_t v_offs;
	/** Measurement window horizontal size */
	uint16_t h_size;
	/** Measurement window vertical size */
	uint16_t v_size;
	/** White pixel range: max Y (YCbCr mode) */
	uint8_t  max_y;
	/** White pixel range: Cr reference / max R (depends on mode) */
	uint8_t  ref_cr_max_r;
	/** White pixel range: min Y / max G (depends on mode) */
	uint8_t  min_y_max_g;
	/** White pixel range: Cb reference / max B (depends on mode) */
	uint8_t  ref_cb_max_b;
	/** White pixel range: max chroma sum (YCbCr only) */
	uint8_t  max_c_sum;
	/** White pixel range: min chroma (YCbCr only) */
	uint8_t  min_c;
};

/* --------------------------------------------------------------------------
 * Auto Route — part of ISP_CALIB_MODULE_S
 * --------------------------------------------------------------------------
 */

/**
 * @brief ISP auto route table.
 *
 * Maps 16 gain-strength levels to ISP module enable bitmasks.
 * Passed alongside the other module params via VIDEO_CID_ALIF_ISP_SET.
 * ISP_AUTO_STRENGTH_NUM entries.
 */
struct isp_auto_route_param {
	/** Auto route bitmask per strength level [ISP_AUTO_STRENGTH_NUM] */
	uint32_t auto_route[ISP_AUTO_STRENGTH_NUM];
};

/* --------------------------------------------------------------------------
 * Aggregate ISP parameter container
 * --------------------------------------------------------------------------
 */

/**
 * @brief Bitmask of modules present in struct isp_params.
 *
 * Set the corresponding bit before calling video_set_ctrl() /
 * video_get_ctrl() to indicate which module sub-structs are valid.
 */
#define ISP_PARAM_MASK_AE        (1U << 0)
#define ISP_PARAM_MASK_WB        (1U << 1)
#define ISP_PARAM_MASK_BLS       (1U << 2)
#define ISP_PARAM_MASK_DMSC      (1U << 3)
#define ISP_PARAM_MASK_FLT       (1U << 4)
#define ISP_PARAM_MASK_CCM       (1U << 5)
#define ISP_PARAM_MASK_GAMMA_OUT (1U << 6)
#define ISP_PARAM_MASK_CSM       (1U << 7)
#define ISP_PARAM_MASK_AEM       (1U << 8)
#define ISP_PARAM_MASK_WBM       (1U << 9)
#define ISP_PARAM_MASK_AUTO_ROUTE (1U << 10)
#define ISP_PARAM_MASK_ALL        (0x7FFU)

/**
 * @brief Single aggregate struct passed to VIDEO_CID_ALIF_ISP_SET /
 *        VIDEO_CID_ALIF_ISP_GET.
 *
 * The caller sets @p valid_mask to the OR of the ISP_PARAM_MASK_* bits for
 * the modules it wants to set or read.  Sub-structs for modules whose bit is
 * clear are ignored by the driver.
 *
 * Example — set only the AE module:
 * @code
 *   struct isp_params p = {
 *       .valid_mask = ISP_PARAM_MASK_AE,
 *       .ae = { .op_mode = ISP_OP_MANUAL, .int_time = 33000, ... },
 *   };
 *   video_set_ctrl(isp_dev, VIDEO_CID_ALIF_ISP_SET, &p);
 * @endcode
 *
 * Example — read AE + WB at the same time:
 * @code
 *   struct isp_params p = { .valid_mask = ISP_PARAM_MASK_AE | ISP_PARAM_MASK_WB };
 *   video_get_ctrl(isp_dev, VIDEO_CID_ALIF_ISP_GET, &p);
 * @endcode
 */
struct isp_params {
	/** Bitmask of ISP_PARAM_MASK_* flags indicating valid sub-structs. */
	uint32_t valid_mask;

	struct isp_ae_param    ae;
	struct isp_wb_param    wb;
	struct isp_bls_param   bls;
	struct isp_dmsc_param  dmsc;
	struct isp_flt_param   flt;
	struct isp_ccm_param   ccm;
	struct isp_gamma_param gamma_out;
	struct isp_csm_param   csm;
	struct isp_aem_param        aem;
	struct isp_wbm_param        wbm;
	struct isp_auto_route_param auto_route;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_VIDEO_ISP_CTRL_PARAMS_H__ */
