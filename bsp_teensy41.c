#include "stdlib.h"
#include "imxrt.h"

/* Teensy 4.1 board support package
 * NOTE: all binary values are in little-endian order by default
 * SEE: imxrt1062_processor_reference_manual.pdf
 * SEE: armv7m_architecture_reference_manual.pdf
 * SEE: cortex_m7_technical_reference_manual.pdf
 */ 

/* symbols defined in teensy41.ld linker script */
extern unsigned long _text_start_ptr;
extern unsigned long _text_end_ptr;
extern unsigned long _text_loadaddr_start_ptr;

extern unsigned long _data_start_ptr;
extern unsigned long _data_end_ptr;
extern unsigned long _data_loadaddr_start_ptr;

extern unsigned long _bss_start_ptr;
extern unsigned long _bss_end_ptr;

extern unsigned long _heap_start_ptr;
extern unsigned long _heap_end_ptr;

extern unsigned long _extram_start_ptr;
extern unsigned long _extram_end_ptr;

extern unsigned long _itcm_block_count;
extern unsigned long _flexram_bank_config;
extern unsigned long _estack;

extern unsigned long _flash_image_length;
extern unsigned long _teensy_model_identifier;

/* symbols expected to be defined */
extern int main(void);

/* forward declarations */
void reset_handler(void);

/* The "high assurance boot" mode requires a command sequence file so that
 * it knows what instructions it has to perform. this is it
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg279, section 9.12
 */
__attribute__ ((section(".hab_csf"), used))
const u32 hab_csf[768] = {
	0,
};

/* The boot data structure is used by the bootloader to figure out how the
 * user program image is to be read
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, page262, section 9.7.1.2
 */
__attribute__ ((section(".boot_data"), used))
const u32 boot_data[] = {
	0x60000000,			/* start */
	(u32)&_flash_image_length,	/* length */
	0,				/* plugin */

};

/* The device configuration data table allows for configuration of peripherals
 * by the bootloader, before any user code is executed
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg262, section 9.7.2
 */
__attribute__ ((section(".boot_dcd"), used))
const u32 boot_dcd[] = {
	0x410400D1,			/* header */
};

/* The image vector table is used by the bootloader to figure out whether
 * there is any additional configuration to be done, where the user program
 * is in memory, and where to jump to once said program has been loaded
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg260, section 9.7.1
 */
__attribute__ ((section(".image_vector_table"), used))
const u32 image_vector_table[] = {
	0x412000D1,			/* header */
	(u32)&reset_handler,		/* entry */
	0,				/* reserved 1 */
	(u32)&boot_dcd,			/* device configuration data */
	(u32)&boot_data,		/* boot data */
	(u32)&image_vector_table,	/* self */
	(u32)&hab_csf,			/* csf */
	0,				/* reserved 2 */
};

/* The FlexSPI configuration block allows for configuration of the memory
 * devices present on the teensy41
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg226, section 9.6.3.1
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg230, section 9.6.3.2
 */
__attribute__ ((section(".flash_config"), used))
const u8 flexspi_config_block[512] = {
	/* common memory configuration block (pg226, section 9.6.3.1)
	 */
	0x42, 0x46, 0x43, 0x46,		/* tag */
	0x56, 0x01, 0x00, 0x00,		/* version */
	0x00, 0x00, 0x00, 0x00,		/* reserved */
	0x01,				/* readSampleClkSrc */
	0x03,				/* csHoldTime */
	0x03,				/* csSetupTime */
	0x00,				/* columnAddressWidth */
	0x00,				/* deviceModeCfgEnable */
	0x00,				/* reserved */
	0x00, 0x00,			/* waitTimeCfgCommands */
	0x00, 0x00, 0x00, 0x00,		/* deviceModeSeq */
	0x00, 0x00, 0x00, 0x00,		/* deviceModeArg */
	0x00,				/* configCmdEnable */
	0x00, 0x00, 0x00,		/* reserved */
	0x00, 0x00, 0x00, 0x00,		/* configCmdSeqs */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,		/* reserved */
	0x00, 0x00, 0x00, 0x00,		/* cfgCmdArgs */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,		/* reserved */
	0x00, 0x00, 0x00, 0x00,		/* controllerMiscOption */
	0x01,				/* deviceType */
	0x04,				/* sflashPadType */
	0x08,				/* serialClkFreq */
	0x00,				/* lutCustomSeqEnable */
	0x00, 0x00, 0x00, 0x00,		/* reserved */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x00,		/* sflashA1Size */
	0x00, 0x00, 0x00, 0x00,		/* sflashA2Size */
	0x00, 0x00, 0x00, 0x00,		/* sflashB1Size */
	0x00, 0x00, 0x00, 0x00,		/* sflashB2Size */
	0x00, 0x00, 0x00, 0x00,		/* csPadSettingOverride */
	0x00, 0x00, 0x00, 0x00,		/* sclkPadSettingOverride */
	0x00, 0x00, 0x00, 0x00,		/* dataPadSettingOverride */
	0x00, 0x00, 0x00, 0x00,		/* dqsPadSettingOverride */
	0x00, 0x00, 0x00, 0x00,		/* timeoutInMs */
	0x00, 0x00, 0x00, 0x00,		/* commandInterval */
	0x00, 0x00, 0x00, 0x00,		/* dataValidTime */
	0x00, 0x00,			/* busyOffset */
	0x00, 0x00,			/* busyBitPolarity */

	/* TODO: not too sure where these values come from  */
	0xEB, 0x04, 0x18, 0x0A,		/* lookupTable[0] */
	0x06, 0x32, 0x04, 0x26,		/* lookupTable[1] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[2] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[3] */

	0x05, 0x04, 0x04, 0x24,		/* lookupTable[4] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[5] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[6] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[7] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[8] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[9] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[10] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[11] */

	0x06, 0x04, 0x00, 0x00,		/* lookupTable[12] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[13] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[14] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[15] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[16] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[17] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[18] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[19] */

	0x20, 0x04, 0x18, 0x08,		/* lookupTable[20] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[21] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[22] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[23] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[24] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[25] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[26] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[27] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[28] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[29] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[30] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[31] */

	0xD8, 0x04, 0x18, 0x08,		/* lookupTable[32] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[33] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[34] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[35] */

	0x02, 0x04, 0x18, 0x08,		/* lookupTable[36] */
	0x04, 0x20, 0x00, 0x00,		/* lookupTable[37] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[38] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[39] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[40] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[41] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[42] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[43] */

	0x60, 0x04, 0x00, 0x00,		/* lookupTable[44] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[45] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[46] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[47] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[48] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[49] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[50] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[51] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[52] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[53] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[54] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[55] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[56] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[57] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[58] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[59] */

	0x00, 0x00, 0x00, 0x00,		/* lookupTable[60] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[61] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[62] */
	0x00, 0x00, 0x00, 0x00,		/* lookupTable[63] */

	0x00, 0x00, 0x00, 0x00,		/* LUT[0]: Read */
	0x00, 0x00, 0x00, 0x00,		/* LUT[1]: ReadStatus */
	0x00, 0x00, 0x00, 0x00,		/* LUT[3]: WriteEnable */
	0x00, 0x00, 0x00, 0x00,		/* LUT[5]: EraseSector */

	0x00, 0x00, 0x00, 0x00,		/* LUT[9]: PageProgram */
	0x00, 0x00, 0x00, 0x00,		/* LUT[11]: ChipErase */
	0x00, 0x00, 0x00, 0x00,		/* LUT[15]: Dummy */
	0x00, 0x00, 0x00, 0x00,		/* reserved */

	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00,		
	0x00, 0x00, 0x00, 0x00,		/* LUT[13]: NOR_CMD_LUT_SEQ_IDX_READ_SFDP */
	0x00, 0x00, 0x00, 0x00,		/* LUT[14]: NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD */
	0x00, 0x00, 0x00, 0x00,		/* reseved */

	/* serial NOR configuration block (pg230, section 9.6.3.2)
	 * ---
	 */
	0x00, 0x01, 0x00, 0x00,		/* pageSize */
	0x00, 0x00, 0x00, 0x00,		/* sectorSize */
	0x01, 0x00, 0x00, 0x00,		/* ipCmdSerialClkFreq */
	0x00, 0x00, 0x00, 0x00,		/* reseved */

	0x00, 0x00, 0x01, 0x00,		/* TODO: blockSize? */
	0x00, 0x00, 0x00, 0x00,		/* reserved */
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,

	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
};

typedef void (*volatile nvic_isr_t)(void);

void unused_interrupt_handler(void);

/* The interrupt vector table stores an interrupt handler for every interrupt
 * available on the teensy41
 * ---
 *  SEE: armv7m_architecture_reference_manual.pdf, pg525, section B1.5.2
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg43, section 4.3
 */
__attribute__ ((section(".vector_table"), used, aligned(1024)))
nvic_isr_t interrupt_vector_table[NVIC_NUM_EXCEPTIONS + NVIC_NUM_INTERRUPTS] = {
	&unused_interrupt_handler,	/* Reset */
	&unused_interrupt_handler,	/* NMI */
	&unused_interrupt_handler,	/* HardFault */
	&unused_interrupt_handler,	/* MemManage */
	&unused_interrupt_handler,	/* BusFault */
	&unused_interrupt_handler,	/* UsageFault */
	0,				/* reserved */
	0,
	0,
	0,
	0,
	&unused_interrupt_handler,	/* SVCall */
	&unused_interrupt_handler,	/* DebugMonitor */
	0,				/* reserved */
	&unused_interrupt_handler,	/* PendSV */
	&unused_interrupt_handler,	/* SysTick */

	/* external interrupts */
};

/* The reset handler is our entrypoint into our user code, and will be invoked
 * by the bootloader once it has loaded our program into memory
 */
__attribute__ ((section(".reset_handler"), used))
void reset_handler(void) {

	/* TODO: enabling and initialising MPU? */
	IOMUXC_GPR_GPR17 = (uint32_t)&_flexram_bank_config;
	IOMUXC_GPR_GPR16 = 0x00200007;
	IOMUXC_GPR_GPR14 = 0x00AA0000;
	__asm__ volatile("mov sp, %0" : : "r" ((uint32_t)&_estack) : );
	__asm__ volatile("dsb":::"memory");
	__asm__ volatile("isb":::"memory");

	/* enable bandgap-based self-bias circuit in the power management unit
	 * for best noise performance
	 * ---
	 *  SEE: imxrt1062_processor_reference_manual.pdf, pg1175, section 16.6.5
	 */
	PMU_MISC0_SET = 1 << 3;

	/* enabling and initialising FPU */
	SCB_CPACR |= 0x00F00000;

	/* copy over instructions from flash to ITCM module, and variables
	 * from flash to DTCM module
	 */
	u32 *from, *until, *to;

	from = &_text_start_ptr;
	until = &_text_end_ptr;
	to = &_text_loadaddr_start_ptr;
	while (from < until)
		*from++ = *to++;

	from = &_data_start_ptr;
	until = &_data_end_ptr;
	to = &_data_loadaddr_start_ptr;
	while (from < until)
		*from++ = *to++;

	/* clear bss area in flash */
	from = &_bss_start_ptr;
	until = &_bss_end_ptr;
	while (from < until)
		*from++ = 0;

	/* reset external interrupts in NVIC */
	for (size_t i = 0; i < NVIC_NUM_INTERRUPTS; i++) {
		attachInterruptVector(i, &unused_interrupt_handler);
		NVIC_SET_PRIORITY(i, 128);
	}

	SCB_VTOR = (u32)interrupt_vector_table;

	/* enable exception handling */
	SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA | SCB_SHCSR_BUSFAULTENA | SCB_SHCSR_USGFAULTENA;

	main();

	/* we should never reach this point, as main should not return, but
	 * if we do then we hang indefinitely waiting for interrupts
	 */
	while (1) __asm__ ("WFI");
}

__attribute__ ((naked))
void unused_interrupt_handler(void) {
	// TODO: implement some kind of debug log
	while (1) __asm__ ("WFI");
}
