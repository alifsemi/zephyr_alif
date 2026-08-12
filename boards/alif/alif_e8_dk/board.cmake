# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_AE822FA0E5597XX0_RTSS_HP)
board_runner_args(alif_flash "--device=AE822FA0E5597_HP")
elseif(CONFIG_SOC_AE822FA0E5597XX0_RTSS_HE)
board_runner_args(alif_flash "--device=AE822FA0E5597_HE")
elseif(CONFIG_SOC_AE402FA0E5597XX0_RTSS_HP)
board_runner_args(alif_flash "--device=AE402FA0E5597_HP")
elseif(CONFIG_SOC_AE402FA0E5597XX0_RTSS_HE)
board_runner_args(alif_flash "--device=AE402FA0E5597_HE")
endif()

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)

