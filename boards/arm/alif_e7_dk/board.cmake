# SPDX-License-Identifier: Apache-2.0

include(${ZEPHYR_BASE}/boards/common/alif_flash.board.cmake)

if(CONFIG_SOC_E7_DK_RTSS_HP)
  board_runner_args(jlink "--device=Cortex-M55" "--speed=55000" "--gdb-port=2341")
elseif(CONFIG_SOC_E7_DK_RTSS_HE)
  board_runner_args(jlink "--device=Cortex-M55" "--speed=55000" "--gdb-port=2331")
endif()

set(BOARD_FLASH_RUNNER jlink)
set(BOARD_DEBUG_RUNNER jlink)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
