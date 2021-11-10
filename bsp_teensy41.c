#include "stdlib.h"
#include "imxrt.h"

#define __IMXRT1062__

/* teensy 4.1 board support package
 * NOTE: all binary values are in little-endian order by default
 * SEE: imxrt1060_processor_reference_manual.pdf
 * SEE: armv7m_architecture_reference_manual.pdf
 * SEE: cortex_m7_technical_reference_manual.pdf
 */ 

/* symbols defined in teensy41.ld linker script */
extern unsigned long _itcm_start_ptr;
extern unsigned long _itcm_end_ptr;
extern unsigned long _itcm_data_start_ptr;

extern unsigned long _dtcm_start_ptr;
extern unsigned long _dtcm_end_ptr;
extern unsigned long _dtcm_data_start_ptr;

extern unsigned long _bss_start_ptr;
extern unsigned long _bss_end_ptr;

extern unsigned long _heap_start_ptr;
extern unsigned long _heap_end_ptr;

extern unsigned long _extram_start_ptr;
extern unsigned long _extram_end_ptr;

extern unsigned long _flexram_bank_config;
extern unsigned long _stack_end_ptr;

extern unsigned long _flash_image_length;
extern unsigned long _teensy_model_identifier;

/* symbols expected to be defined */
extern int main(void);

/* forward declarations */
void reset(void);
void pendsv_handler(void);
void systick_handler(void);
void unused_interrupt_handler(void);

/* exception handlers, to be defined in user code */
extern void reset_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void nmi_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void hard_fault_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void mem_manage_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void bus_fault_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void usage_fault_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void svcall_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));
extern void debug_monitor_handler(void) __attribute__ ((weak, alias("unused_interrupt_handler")));

/* the boot data structure is used by the bootloader to figure out how the
 * user program image is to be read
 * ---
 *  SEE: imxrt1060_processor_reference_manual.pdf, page262, section 9.7.1.2
 */
__attribute__ ((section(".boot_data"), used))
const u32 boot_data[] = {
	0x60000000,			/* start */
	(u32)&_flash_image_length,	/* length */
	0,				/* plugin */

};

/* the boot device configuration block is used by the booloader to setup
 * peripherals before the reset handler is called
 * ---
 *  SEE: imxrt1060_processor_reference_manual.pdf, page262, section 9.7.2
 */
__attribute__ ((section(".boot_data"), used))
const u32 boot_dcd[] = {
	0x410400D2,			/* header */
};

/* the image vector table is used by the bootloader to figure out whether
 * there is any additional configuration to be done, where the user program
 * is in memory, and where to jump to once said program has been loaded
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg260, section 9.7.1
 */
__attribute__ ((section(".img_vec"), used))
const u32 image_vector_table[] = {
	0x432000D1,			/* header */
	(u32)&reset,			/* entry */
	0,				/* reserved 1 */
	(u32)&boot_dcd,			/* device configuration data */
	(u32)&boot_data,		/* boot data */
	(u32)&image_vector_table,	/* self */
	0,				/* csf */
	0,				/* reserved 2 */
};

/* the FlexSPI configuration block allows for configuration of the memory
 * devices present on the teensy41
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg226, section 9.6.3.1
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg230, section 9.6.3.2
 */
__attribute__ ((section(".flash_config"), used))
const u8 flexspi_config_block[512] = {
	/* common memory configuration block (pg226, section 9.6.3.1)
	 */
	0x46, 0x43, 0x46, 0x42,		/* tag */
	0x00, 0x00, 0x01, 0x56,		/* version */
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
	0x00, 0x10, 0x00, 0x00,		/* sectorSize */
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

/* the interrupt vector table stores an interrupt handler for every interrupt
 * available on the teensy41
 * ---
 *  SEE: armv7m_architecture_reference_manual.pdf, pg525, section B1.5.2
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg43, section 4.3
 */
__attribute__ ((section(".isr_vec"), used, aligned(1024)))
nvic_isr_t interrupt_vector_table[NVIC_NUM_EXCEPTIONS + NVIC_NUM_INTERRUPTS] = {
	&reset_handler,			/* Reset */
	&nmi_handler,			/* NMI */
	&hard_fault_handler,		/* HardFault */
	&mem_manage_handler,		/* MemManage */
	&bus_fault_handler,		/* BusFault */
	&usage_fault_handler,		/* UsageFault */
	NULL,				/* reserved */
	NULL,
	NULL,
	NULL,
	NULL,
	&svcall_handler,		/* SVCall */
	&debug_monitor_handler,		/* DebugMonitor */
	NULL,				/* reserved */
	&pendsv_handler,		/* PendSV */
	&systick_handler,		/* SysTick */

	/* external interrupts */
};


/* helper functions
 */
static inline void memcpy(u32 *restrict dst, u32 *restrict dstend, const u32 *restrict src);
static inline void memset(u32 *restrict dst, u32 *restrict dstend, const u32 val);

static inline void reset_clock_dividers(void);
static inline void initialise_cache(void);
static inline void initialise_systick(void);
//static inline void initialise_psram(void);

/* the reset method is our entrypoint into our user code, and will be invoked
 * by the bootloader once it has loaded our program into memory
 */
__attribute__ ((section(".startup_code"), optimize("no-tree-loop-distribute-patterns")))
void reset(void) {

	/* enabling flexram banks
	 */
	IOMUXC_GPR_GPR17 = (u32)&_flexram_bank_config;
	IOMUXC_GPR_GPR16 = 0x00200007;
	IOMUXC_GPR_GPR14 = 0x00AA0000;

	/* set the stack pointer to the bottom of the stack
	 */
	__asm__ volatile("mov sp, %0" : : "r" ((u32)&_stack_end_ptr) : );
	__asm__ volatile("dsb":::"memory");
	__asm__ volatile("isb":::"memory");

	/* enable bandgap-based self-bias circuit in the power management unit
	 * for best noise performance
	 * ---
	 *  SEE: imxrt1062_processor_reference_manual.pdf, pg1175, section 16.6.5
	 */
	PMU_MISC0_SET = 1 << 3;

	/* enabling and initialising FPU
	 * ---
	 *  SEE: cortex_m7_technical_reference_manual.pdf, pg61, section 4.1.2
	 */
	SCB_CPACR |= 0x00F00000;
	__asm__ volatile("dsb":::"memory");
	__asm__ volatile("isb":::"memory");

	/* copy over instructions from flash to ITCM module, and variables
	 * from flash to DTCM module
	 */
	memcpy(&_itcm_start_ptr, &_itcm_end_ptr, &_itcm_data_start_ptr);
	memcpy(&_dtcm_start_ptr, &_dtcm_end_ptr, &_dtcm_data_start_ptr);

	/* clear bss area in flash */
	memset(&_bss_start_ptr, &_bss_end_ptr, 0);

	/* reset external interrupts in NVIC */
	for (size_t i = 0; i < NVIC_NUM_INTERRUPTS; i++) {
		attachInterruptVector(i, &unused_interrupt_handler);
		NVIC_SET_PRIORITY(i, 128);
	}
	SCB_VTOR = (u32)interrupt_vector_table;

	/* enable exception handling */
	SCB_SHCSR |= SCB_SHCSR_MEMFAULTENA | SCB_SHCSR_BUSFAULTENA | SCB_SHCSR_USGFAULTENA;

	reset_clock_dividers();

	/* reset and configure the clock sources for the periodic interrupt 
	 * timer (PIT), general programmable timers (GPT), and the UART to run
	 * off of the undivided osc_clk clock source
	 * ---
	 *  SEE: imxrt1060_processor_reference_manual.pdf, pg1057, section 14.7.7
	 */
	const u32 perclk_divider_mask = CCM_CSCMR1_PERCLK_PODF(0x3F);
	const u32 uartclk_divider_mask = CCM_CSCDR1_UART_CLK_PODF(0x3f);
	CCM_CSCMR1 = (CCM_CSCMR1 & ~perclk_divider_mask) | CCM_CSCMR1_PERCLK_CLK_SEL;
	CCM_CSCDR1 = (CCM_CSCDR1 & ~uartclk_divider_mask) | CCM_CSCDR1_UART_CLK_SEL;

	/* configuring the IOMUX controller to select banks 6, 7, 8,and 9
	 * from the GPIO pins (use fast gpio over regular gpio)
	 * ---
	 *  SEE: imxrt1060_processor_reference_manual.pdf, pg375, section 11.4.27
	 */
	const u32 gpio_mask = 0xFFFFFFFF;
	IOMUXC_GPR_GPR26 = gpio_mask;
	IOMUXC_GPR_GPR27 = gpio_mask;
	IOMUXC_GPR_GPR28 = gpio_mask;
	IOMUXC_GPR_GPR29 = gpio_mask;

	initialise_cache();
	initialise_systick();

	main();

	/* we should never reach this point, as main should not return, but
	 * if we do then we hang indefinitely waiting for interrupts
	 */
	while (1) __asm__ ("wfi");
}

/* reset the clock dividers (PFDs) associated with the system clock
 * (528MHz) and USB1 (480MHz)
 * ---
 *  SEE: imxrt1062_processor_reference_manual.pdf, pg1122, section 14.8.16
 */
FLASHMEM static inline void reset_clock_dividers(void) {
	const u32 pfd_clkgate_mask = (1 << 31) | (1 << 23) | (1 << 15) | (1 << 7);
	const u32 pll2_pfd_reset = 0x1018101B;
	const u32 pll3_pfd_reset = 0x1311100C;

	CCM_ANALOG_PFD_528_SET |= pfd_clkgate_mask;
	CCM_ANALOG_PFD_528 = pll2_pfd_reset; /* also resets clkgate bits */
	CCM_ANALOG_PFD_480_SET |= pfd_clkgate_mask;
	CCM_ANALOG_PFD_480 = pll3_pfd_reset; /* also resets clkgate bits */

}

/* the systick frequency is 100kHz
 * ---
 *  SEE: imxrt1060_processor_reference_manual.pdf, pg986, section 13.3.2.1
 */
#define SYSTICK_EXT_FREQ 100000

/* operating frequencies for the processor and various buses
 * TODO: implement clockspeed changing?
 */
volatile u32 CPU_FREQ = 396000000;
volatile u32 BUS_FREQ = 132000000;

volatile u32 systick_cycle_count;
volatile u32 systick_millis_count;

void systick_handler(void) {
	systick_cycle_count = ARM_DWT_CYCCNT;
	systick_millis_count++;
}

void pendsv_handler(void) {

}

/* initialise the systick reload value register (SYST_RVR), current value
 * register (SYST_CVR), and control and status register (SYST_CSR), and set
 * priorities for the systick and pendsv interrupts
 * ---
 *  SEE: armv7m_architecture_reference_manual.pdf, pg621, section B3.3.3
 */
FLASHMEM static inline void initialise_systick(void) {
	/* this gives us a 1kHz clock and resets the current count
	 */
	SYST_RVR = (SYSTICK_EXT_FREQ / 100) - 1;
	SYST_CVR = 0;
	
	SYST_CSR = SYST_CSR_TICKINT | SYST_CSR_ENABLE;

	/* we set the priority of the systick and pendsv interrupts to 32
	 */
	SCB_SHPR3 = 0x20200000;

	/* we enable debugging and monitoring blocks, enable the cycle counter,
	 * and set the current systick cycle count to 0
	 */
	ARM_DEMCR |= ARM_DEMCR_TRCENA;
	ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
	systick_cycle_count = ARM_DWT_CYCCNT;
}

#define NOEXEC		SCB_MPU_RASR_XN
#define NOACCESS	SCB_MPU_RASR_AP(0)
#define READONLY	SCB_MPU_RASR_AP(7)
#define READWRITE	SCB_MPU_RASR_AP(3)
#define MEM_CACHE_WT	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C
#define MEM_CACHE_WB	SCB_MPU_RASR_TEX(0) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_CACHE_WBWA	SCB_MPU_RASR_TEX(1) | SCB_MPU_RASR_C | SCB_MPU_RASR_B
#define MEM_NOCACHE	SCB_MPU_RASR_TEX(1)
#define DEV_NOCACHE	SCB_MPU_RASR_TEX(2)

#define SIZE_32B	(SCB_MPU_RASR_SIZE(4) | SCB_MPU_RASR_ENABLE)
#define SIZE_128K	(SCB_MPU_RASR_SIZE(16) | SCB_MPU_RASR_ENABLE)
#define SIZE_512K	(SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)
#define SIZE_1M		(SCB_MPU_RASR_SIZE(19) | SCB_MPU_RASR_ENABLE)
#define SIZE_16M	(SCB_MPU_RASR_SIZE(23) | SCB_MPU_RASR_ENABLE)
#define SIZE_64M	(SCB_MPU_RASR_SIZE(25) | SCB_MPU_RASR_ENABLE)
#define SIZE_4G		(SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE)
#define REGION(n)	(SCB_MPU_RBAR_REGION(n) | SCB_MPU_RBAR_VALID)

/* initialise and enable the data and instruction caches
 * ---
 *  SEE: cortex_m7_technical_reference_manual.pdf, pg62, section 4.1.3
 */
FLASHMEM static inline void initialise_cache(void) {
	/* configure MPU regions
	 */
	SCB_MPU_CTRL = 0;

	u32 i = 0;

	/* global memory region
	 * ---
	 *  SEE: https://developer.arm.com/docs/146793866/10/why-does-the-cortex-m7-initiate-axim-read-accesses-to-memory-addresses-that-do-not-fall-under-a-defined-mpu-region
	 */
	SCB_MPU_RBAR = 0x00000000 | REGION(i++);
	SCB_MPU_RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_4G;

	/* ITCM */
	SCB_MPU_RBAR = 0x00000000 | REGION(i++);
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | SIZE_512K;

	/* TODO: trap regions should be created last, because the hardware gives
	 * priority to the higher number ones.
	 */

	/* Null pointer deref trap */
	SCB_MPU_RBAR = 0x00000000 | REGION(i++);
	SCB_MPU_RASR =  DEV_NOCACHE | NOACCESS | SIZE_32B;

	/* Boot ROM */
	SCB_MPU_RBAR = 0x00200000 | REGION(i++);
	SCB_MPU_RASR = MEM_CACHE_WT | READONLY | SIZE_128K;

	/* DTCM */
	SCB_MPU_RBAR = 0x20000000 | REGION(i++);
	SCB_MPU_RASR = MEM_NOCACHE | READWRITE | NOEXEC | SIZE_512K;

	/* Stack overflow trap */
	SCB_MPU_RBAR = ((u32)&_bss_end_ptr) | REGION(i++);
	SCB_MPU_RASR = SCB_MPU_RASR_TEX(0) | NOACCESS | NOEXEC | SIZE_32B;

	/* RAM (AXI bus) */
	SCB_MPU_RBAR = 0x20200000 | REGION(i++);
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_1M;

	/* Peripherals */
	SCB_MPU_RBAR = 0x40000000 | REGION(i++);
	SCB_MPU_RASR = DEV_NOCACHE | READWRITE | NOEXEC | SIZE_64M;

	/* PSRAM */
	SCB_MPU_RBAR = 0x60000000 | REGION(i++);
	SCB_MPU_RASR = MEM_CACHE_WBWA | READONLY | SIZE_16M;

	/* FlexSPI2 */
	SCB_MPU_RBAR = 0x70000000 | REGION(i++);
	SCB_MPU_RASR = MEM_CACHE_WBWA | READWRITE | NOEXEC | SIZE_16M;

	SCB_MPU_CTRL = SCB_MPU_CTRL_ENABLE;

	__asm__ ("dsb":::"memory");
	__asm__ ("isb":::"memory");

	/* TODO: invalidate data cache
	 */
#if 0
	SCB_ID_CSSELR = 0;
	__asm__ ("dsb":::"memory");

	const u32 cache_line_size = SCB_ID_CCSIDR & 0x7;
	const u32 cache_line_words = cache_line_size & 0x4;
	const u32 cache_ways = SCB_ID_CCSIDR & (0x3ff << 3);
	const u32 cache_sets = SCB_ID_CCSIDR & (0x7ffff << 13);

	__asm__ ("dsb":::"memory");
	__asm__ ("isb":::"memory");
#endif

	/* invalidate instruction cache
	 */
	SCB_CACHE_ICIALLU = 0;
	__asm__ ("dsb":::"memory");
	__asm__ ("isb":::"memory");

	/* enable data and instruction caches
	 */
	SCB_CCR |= (SCB_CCR_IC | SCB_CCR_DC);
	__asm__ ("dsb":::"memory");
	__asm__ ("isb":::"memory");

}

__attribute__ ((naked))
void unused_interrupt_handler(void) {
	/* TODO: implement some kind of debug log
	 */
	while (1) __asm__ ("wfi");
}

/* copies words starting from src into the region bounded by dst and dstend
 */
__attribute__ ((section(".startup_code")))
static inline void memcpy(u32 *restrict dst, u32 *restrict dstend, const u32 *restrict src) {
	/* we cannot copy zero bytes or less, so ignore the call
	 */
	if (dst >= dstend) return;

	while (dst < dstend)
		*dst++ = *src++;
}

/* copies the given value into the region bounded by dst and dstend
 */
__attribute__ ((section(".startup_code")))
static inline void memset(u32 *restrict dst, u32 *restrict dstend, const u32 val) {
	/* we cannot set zero bytes or less, so ignore the call
	 */
	if (dst >= dstend) return;

	while (dst < dstend)
		*dst++ = val;
}
