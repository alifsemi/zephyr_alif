# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_E3_DK_RTSS_HP)
board_runner_args(alif_flash "--device=AE302F80F55D5_HP" "--speed=4000")
elseif(CONFIG_SOC_E3_DK_RTSS_HE)
board_runner_args(alif_flash "--device=AE302F80F55D5_HE" "--speed=4000")
endif()

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)
