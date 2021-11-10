#ifndef HYPERMAN_STDLIB_H
#define HYPERMAN_STDLIB_H

#include "imxrt.h"
#include "imxrt_pins.h"

#include <stdalign.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdnoreturn.h>

/* Integral types */
typedef uint32_t b32;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

typedef float    f32;
typedef double   f64;

/* String types */
struct str_t {
	char *str; /* null terminated */
	size_t len;
};

/* performs a shallow copy of the given string in "in" to the string in "out"
 * ---
 *  in: a pointer to the original string
 *  out: a pointer to the shallow copy of the original string
 */
//extern inline void str_copy(const struct str_t *in, struct str_t *out);

/* attempts to deep copy the given string in "in" to the string in "out"
 * ---
 *  in: a pointer to the original string
 *  out: a pointer to the deep copy of the original string
 */
//extern inline b32 try_str_clone(const struct str_t *in, struct str_t *out);

/* Processor register types */
typedef uint32_t reg32;
typedef uint64_t reg64;

/* Utility definitions */
#define ARRLEN(arr) (sizeof(arr) / sizeof(arr[0]))

/* Linker Section Modifiers
 * FASTRUN: code stored in flash that will be copied into the ITCM on startup
 * DMAMEM: ???
 * PROGMEM: variables stored in flash to preserve RAM
 * FLASHMEM: code stored in flash to preserve RAM
 * EXTMEM: variables stored in external PSRAM memory modules (may not exist)
 */
#define FASTRUN __attribute__ ((section(".fastrun")))
#define DMAMEM __attribute__ ((section(".dmamem")))
#define PROGMEM __attribute__ ((section(".progmem")))
#define FLASHMEM __attribute__ ((section(".flash")))
#define EXTMEM __attribute__ ((section(".extmem")))

/* global variables */
extern volatile u32 CPU_FREQ;
extern volatile u32 BUS_FREQ;

#endif /* HYPERMAN_STDLIB_H */
