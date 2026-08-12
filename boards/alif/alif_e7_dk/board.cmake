# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_AE722F80F55D5XX_RTSS_HP)
board_runner_args(alif_flash "--device=AE722F80F55D5_HP")
elseif(CONFIG_SOC_AE722F80F55D5XX_RTSS_HE)
board_runner_args(alif_flash "--device=AE722F80F55D5_HE")
elseif(CONFIG_SOC_AE302F80F55D5XX_RTSS_HP)
board_runner_args(alif_flash "--device=AE302F80F55D5_HP")
elseif(CONFIG_SOC_AE302F80F55D5XX_RTSS_HE)
board_runner_args(alif_flash "--device=AE302F80F55D5_HE")
endif()

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)

