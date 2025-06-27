# Copyright (c) 2025 Alif Semiconductor
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_interface_library_named(DAVE2D)
zephyr_library()
zephyr_include_directories(
  .
  d0/inc
  d1/inc
  d2/inc
)

zephyr_library_sources(
  d2/src/dave_64bitoperation.c
  d2/src/dave_blit.c
  d2/src/dave_box.c
  d2/src/dave_box.h
  d2/src/dave_circle.c
  d2/src/dave_context.c
  d2/src/dave_curve.c
  d2/src/dave_curve.h
  d2/src/dave_dlist.c
  d2/src/dave_driver.c
  d2/src/dave_edge.c
  d2/src/dave_edge.h
  d2/src/dave_errorcodes.c
  d2/src/dave_gradient.c
  d2/src/dave_hardware.c
  d2/src/dave_intern.h
  d2/src/dave_line.c
  d2/src/dave_line.h
  d2/src/dave_math.c
  d2/src/dave_memory.c
  d2/src/dave_pattern.c
  d2/src/dave_pattern.h
  d2/src/dave_perfcount.c
  d2/src/dave_polyline.c
  d2/src/dave_quad.c
  d2/src/dave_quad.h
  d2/src/dave_rbuffer.c
  d2/src/dave_render.c
  d2/src/dave_texture.c
  d2/src/dave_triangle.c
  d2/src/dave_utility.c
  d2/src/dave_viewport.c
  d2/src/dave_wedge.c
)

zephyr_library_sources_ifdef(
  CONFIG_WITH_MM_FIXED_RANGE
  d0/src/dave_d0lib.c
  d0/src/dave_d0_mm_fixed_range.c
)

zephyr_library_sources_ifdef(
  CONFIG_WITH_MM_FIXED_RANGE_FIXED_BLKCNT
  d0/src/dave_d0lib.c
  d0/src/dave_d0_mm_fixed_range_fixed_blkcnt.c
)

zephyr_library_sources_ifdef(
  CONFIG_WITH_MM_DYNAMIC
  d0/src/dave_d0lib.c
  d0/src/dave_d0_mm_dynamic.c
)
