#ifndef HYPERMAN_STDLIB_H
#define HYPERMAN_STDLIB_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
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
	char *str;
	size_t len;
};

struct wstr_t {
	wchar_t *str;
	size_t len;
};

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

int stdlib_add(int a, int b);

#endif /* HYPERMAN_STDLIB_H */
