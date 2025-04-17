# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=AE722F80F55D5_HP")
board_runner_args(jlink "--iface=JTAG")
board_runner_args(jlink "--speed=4000")
board_runner_args(jlink "--tool-opt=-jtagconf -1,-1")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
