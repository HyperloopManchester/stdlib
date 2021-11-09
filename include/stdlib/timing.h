#ifndef HYPERMAN_STDLIB_TIMING_H
#define HYPERMAN_STDLIB_TIMING_H

#include "stdlib.h"
#include "imxrt.h"

__attribute__ ((always_inline, used))
static inline void stdlib_delay_sec(u32 sec);
static inline void stdlib_delay_sec(u32 sec) {
	const u32 cycles = CPU_FREQ * sec;
	for (volatile u32 i = 0; i < cycles; i++);
}

__attribute__ ((always_inline, used))
static inline void stdlib_delay_msec(u32 msec);
static inline void stdlib_delay_msec(u32 msec) {
	const u32 cycles = (CPU_FREQ / 1000) * msec;
	for (volatile u32 i = 0; i < cycles; i++);
}

__attribute__ ((always_inline, used))
static inline void stdlib_delay_usec(u32 usec);
static inline void stdlib_delay_usec(u32 usec) {
	const u32 cycles = (CPU_FREQ / 1000000) * usec;
	for (volatile u32 i = 0; i < cycles; i++);
}

#endif /* HYPERMAN_STDLIB_TIMING_H */
