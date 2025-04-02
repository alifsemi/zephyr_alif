/*
 * Copyright (C) 2025 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ospi0), okay)
#if DT_SAME_NODE(DT_NODELABEL(ospi0), DT_PARENT(DT_NODELABEL(ospi_flash)))
#if DT_NODE_EXISTS(DT_NODELABEL(ext_flash_xip))

SECTION_PROLOGUE(CONFIG_OSPI0_XIP_REGION, DT_PROP_BY_IDX(DT_NODELABEL(ext_flash_xip), reg, 0), ALIGN(8))
{
	KEEP(*(.alif_extflash_OSPI0))
	KEEP(*(".alif_extflash_OSPI0.*"))
} GROUP_LINK_IN(OSPI0)

#endif /* DT_NODE_EXISTS(DT_NODELABEL(ext_flash_xip)) */
#endif /* DT_SAME_NODE(DT_NODELABEL(ospi0), DT_PARENT(DT_NODELABEL(ospi_flash))) */
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(ospi0), okay) */


#if DT_NODE_HAS_STATUS(DT_NODELABEL(ospi1), okay)
#if DT_SAME_NODE(DT_NODELABEL(ospi1), DT_PARENT(DT_NODELABEL(ospi_flash)))
#if DT_NODE_EXISTS(DT_NODELABEL(ext_flash_xip))

SECTION_PROLOGUE(CONFIG_OSPI1_XIP_REGION, DT_PROP_BY_IDX(DT_NODELABEL(ext_flash_xip), reg, 0), ALIGN(8))
{
	KEEP(*(.alif_extflash_OSPI1))
	KEEP(*(".alif_extflash_OSPI1.*"))
} GROUP_LINK_IN(OSPI1)

#endif /* DT_NODE_EXISTS(DT_NODELABEL(ext_flash_xip)) */
#endif /* DT_SAME_NODE(DT_NODELABEL(ospi1), DT_PARENT(DT_NODELABEL(ospi_flash))) */
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(ospi1), okay) */
