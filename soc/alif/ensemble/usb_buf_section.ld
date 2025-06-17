/* usb_buf_section.ld */

SECTION_PROLOGUE(.usb_ram,(NOLOAD), SUBALIGN(32)) {
    __usb_ram_start = .;
    *(.usb_ram)
    __usb_ram_end = .;
} GROUP_LINK_IN(SRAM0)
