# Copyright (c) 2026 Arm Debug
#
# SPDX-License-Identifier: Apache-2.0

set(ALIF_DFP_SVD_DIR "${WEST_TOPDIR}/tools/alif_ensemble-cmsis-dfp/Debug/SVD")
set(SOC_SVD_FILE "")

if(CONFIG_SOC_AE822FA0E5597XX0_RTSS_HP)
  set(SOC_SVD_FILE "${ALIF_DFP_SVD_DIR}/AE822FA0E5597BS0_CM55_HP_View.svd")
elseif(CONFIG_SOC_AE822FA0E5597XX0_RTSS_HE)
  set(SOC_SVD_FILE "${ALIF_DFP_SVD_DIR}/AE822FA0E5597BS0_CM55_HE_View.svd")
elseif(CONFIG_SOC_AE722F80F55D5XX_RTSS_HP)
  set(SOC_SVD_FILE "${ALIF_DFP_SVD_DIR}/AE722F80F55D5LS_CM55_HP_View.svd")
elseif(CONFIG_SOC_AE722F80F55D5XX_RTSS_HE)
  set(SOC_SVD_FILE "${ALIF_DFP_SVD_DIR}/AE722F80F55D5LS_CM55_HE_View.svd")
endif()

if(SOC_SVD_FILE)
  if(EXISTS "${SOC_SVD_FILE}")
    message(STATUS "Alif SVD file: ${SOC_SVD_FILE}")
    build_info(vendor-specific alif svdfile VALUE ${SOC_SVD_FILE})
  else()
    message(WARNING "Alif SVD file not found: ${SOC_SVD_FILE}")
  endif()
endif()
