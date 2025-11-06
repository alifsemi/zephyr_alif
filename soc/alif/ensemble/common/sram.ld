/*
* Copyright (C) 2024 Alif Semiconductor.
* SPDX-License-Identifier: Apache-2.0
*/

SECTIONS {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram0), okay)
GROUP_START(SRAM0)
SECTION_PROLOGUE(_ALIF_SRAM0_SECTION_NAME, (NOLOAD),)
	{
KEEP(*(.alif_sram0))
KEEP(*(".alif_sram0.*"))
	} GROUP_LINK_IN(SRAM0)

GROUP_END(SRAM0)
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(sram0), okay) */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram1), okay)
GROUP_START(SRAM1)
SECTION_PROLOGUE(_ALIF_SRAM1_SECTION_NAME, (NOLOAD),)
	{
KEEP(*(.alif_sram1))
KEEP(*(".alif_sram1.*"))
	} GROUP_LINK_IN(SRAM1)

GROUP_END(SRAM1)
#endif /* DT_NODE_HAS_STATUS(DT_NODELABEL(sram1), okay) */
}
