MEMORY
{
	/* instruction tightly-coupled memory */
	ITCM (rwx):  ORIGIN = 0x00000000, LENGTH = 512K
	/* data tightly-coupled memory */
	DTCM (rwx):  ORIGIN = 0x20000000, LENGTH = 512K
	/* on-chip RAM, a.k.a. RAM2 */
	OCRAM (rwx): ORIGIN = 0x20200000, LENGTH = 512K
	/* emulated flash */
	FLASH (rwx): ORIGIN = 0x60000000, LENGTH = 7936K
	/* external RAM */
	ERAM (rwx):  ORIGIN = 0x70000000, LENGTH = 16384K
}

ENTRY(image_vector_table)

SECTIONS
{
	.text.headers : {
		KEEP(*(.flash_config))
		FILL(0xFF)
		. = ORIGIN(FLASH) + 0x1000;
		KEEP(*(.image_vector_table))
		KEEP(*(.boot_data))
		KEEP(*(.boot_dcd))
		. = ALIGN(1024);
	} > FLASH

	.text.code : {
		KEEP(*(.reset_handler))
		*(.flashmem*)
		. = ALIGN(4);
		KEEP(*(.init))
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;
		__init_array_start = .;
		KEEP (*(.init_array))
		__init_array_end = .;
		. = ALIGN(4);
	} > FLASH

	.text.progmem : {
		*(.progmem*)
		. = ALIGN(4);
	} > FLASH

	.text.itcm : {
		. = . + 32; /* MPU to trap NULL pointer deref */
		*(.fastrun)
		*(.text*)
		. = ALIGN(16);
	} > ITCM  AT> FLASH

	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > ITCM  AT> FLASH

	.data : {
		*(.endpoint_queue)   
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.data*)))
		KEEP(*(.vector_table))
	} > DTCM  AT> FLASH

	.bss ALIGN(4) : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(COMMON)))
		. = ALIGN(32);
		. = . + 32; /* MPU to trap stack overflow */
	} > DTCM

	.bss.dma (NOLOAD) : {
		*(.hab_log)
		*(.dmabuffers)
		. = ALIGN(32);
	} > OCRAM

	.bss.extram (NOLOAD) : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.externalram)))
		. = ALIGN(32);
	} > ERAM

	.text.csf : {
		FILL(0xFF)
		. = ALIGN(1024);
		KEEP(*(.hab_csf))
		__text_csf_end = .;
	} > FLASH

	_text_start_ptr = ADDR(.text.itcm);
	_text_end_ptr = ADDR(.text.itcm) + SIZEOF(.text.itcm) + SIZEOF(.ARM.exidx);
	_text_loadaddr_start_ptr = LOADADDR(.text.itcm);

	_data_start_ptr = ADDR(.data);
	_data_end_ptr = ADDR(.data) + SIZEOF(.data);
	_data_loadaddr_start_ptr = LOADADDR(.data);

	_bss_start_ptr = ADDR(.bss);
	_bss_end_ptr = ADDR(.bss) + SIZEOF(.bss);

	_heap_start_ptr = ADDR(.bss.dma) + SIZEOF(.bss.dma);
	_heap_end_ptr = ORIGIN(OCRAM) + LENGTH(OCRAM);

	_extram_start_ptr = ADDR(.bss.extram);
	_extram_end_ptr = ADDR(.bss.extram) + SIZEOF(.bss.extram);

	_itcm_block_count = (SIZEOF(.text.itcm) + SIZEOF(.ARM.exidx) + 0x7FFF) >> 15;
	_flexram_bank_config = 0xAAAAAAAA | ((1 << (_itcm_block_count * 2)) - 1);
	_estack = ORIGIN(DTCM) + ((16 - _itcm_block_count) << 15);

	_flash_image_length = __text_csf_end - ORIGIN(FLASH);
	_teensy_model_identifier = 0x25;

	.debug_info     0 : { *(.debug_info) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }

}
