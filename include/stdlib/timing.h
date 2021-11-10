#ifndef HYPERMAN_STDLIB_TIMING_H
#define HYPERMAN_STDLIB_TIMING_H

#include "stdlib.h"

__attribute__ ((always_inline, used))
static inline void stdlib_delay_sec(u32 sec);
static inline void stdlib_delay_sec(u32 sec) {
	const u32 start = ARM_DWT_CYCCNT, cycles = CPU_FREQ * sec;
	while (ARM_DWT_CYCCNT - start < cycles);
}

__attribute__ ((always_inline, used))
static inline void stdlib_delay_msec(u32 msec);
static inline void stdlib_delay_msec(u32 msec) {
	const u32 start = ARM_DWT_CYCCNT, cycles = (CPU_FREQ / 1000) * msec;
	while (ARM_DWT_CYCCNT - start < cycles);
}

__attribute__ ((always_inline, used))
static inline void stdlib_delay_usec(u32 usec);
static inline void stdlib_delay_usec(u32 usec) {
	const u32 start = ARM_DWT_CYCCNT, cycles = (CPU_FREQ / 1000000) * usec;
	while (ARM_DWT_CYCCNT - start < cycles);
}

#endif /* HYPERMAN_STDLIB_TIMING_H */
