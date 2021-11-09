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
	EXTRAM (rwx):  ORIGIN = 0x70000000, LENGTH = 16384K
}

ENTRY(image_vector_table)

SECTIONS
{
	.text.headers : {
		KEEP(*(.flash_config))
		FILL(0xFF)
		. = ORIGIN(FLASH) + 0x1000;
		KEEP(*(.img_vec))
		KEEP(*(.boot_data))
		KEEP(*(.boot_dcd))
		. = ALIGN(1024);
	} > FLASH

	.text.code : {
		KEEP(*(.startup_code))
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

	.data : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.data*)))
		. = ALIGN(16);
	} > DTCM  AT> FLASH

	.bss ALIGN(4) : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(COMMON)))
		. = ALIGN(32);
		. = . + 32; /* MPU to trap stack overflow */
	} > DTCM

	.bss.dma (NOLOAD) : {
		*(.dmabuffers)
		. = ALIGN(32);
	} > OCRAM

	.bss.extram (NOLOAD) : {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.extram)))
		. = ALIGN(32);
	} > EXTRAM

	/* this is the start and end of the itcm memory, to which data will
	 * be copied on startup
	 */
	_itcm_start_ptr = ADDR(.text.itcm);
	_itcm_end_ptr = ADDR(.text.itcm) + SIZEOF(.text.itcm);

	/* this is the start of the itcm data in flash, from which it will
	 * be copied into the actual itcm memory on startup
	 */
	_itcm_data_start_ptr = LOADADDR(.text.itcm);

	/* this is the start and end of the dtcm memory, to which data will
	 * be copied on startup
	 */
	_dtcm_start_ptr = ADDR(.data);
	_dtcm_end_ptr = ADDR(.data) + SIZEOF(.data);

	/* this is the start of the dtcm data in flash, from which it will
	 * be copied into the actual dtcm memory on startup
	 */
	_dtcm_data_start_ptr = LOADADDR(.data);

	_bss_start_ptr = ADDR(.bss);
	_bss_end_ptr = ADDR(.bss) + SIZEOF(.bss);

	_heap_start_ptr = ADDR(.bss.dma) + SIZEOF(.bss.dma);
	_heap_end_ptr = ORIGIN(OCRAM) + LENGTH(OCRAM);

	_extram_start_ptr = ADDR(.bss.extram);
	_extram_end_ptr = ADDR(.bss.extram) + SIZEOF(.bss.extram);

	_itcm_block_count = (SIZEOF(.text.itcm) + 0x7FFE) >> 15;
	_flexram_bank_config = 0xAAAAAAAA | ((1 << (_itcm_block_count * 2)) - 1);
	_stack_end_ptr = ORIGIN(DTCM) + ((16 - _itcm_block_count) << 15);

	_flash_image_length = SIZEOF(.text.progmem) + SIZEOF(.text.itcm) + SIZEOF(.data);
	_teensy_model_identifier = 0x25;

	.debug_info     0 : { *(.debug_info) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }

}
