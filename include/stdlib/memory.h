#ifndef HYPERMAN_STDLIB_MEMORY_H
#define HYPERMAN_STDLIB_MEMORY_H

#include "stdlib.h"

/* define a 64 KiB size heap */
#define HEAP_SIZE 65536

/* attempts to allocate a region on the heap, returning NULL on failure
 * ---
 *  size: the size of the region that should be allocated
 */
extern void *malloc(size_t size);

/* attempts to reallocate a region on the heap, returning NULL on failure
 * ---
 *  oldptr: a pointer to the previously allocated region, or NULL
 *  size: the new size to which the allocation should be shrunk or extended
 */
extern void *realloc(void *oldptr, size_t size);

/* frees a previously allocated region
 * ---
 *  ptr: a pointer to the previously allocated region, or NULL
 */
extern void free(void *ptr);

/* attempts to allocate a region in external RAM, returning NULL on failure
 * ---
 *  size: the size of the region that should be allocated
 */
extern void *extmem_malloc(size_t size);

/* attempts to reallocate a region in external RAM, returning NULL on failure
 * ---
 *  oldptr: a pointer to the previously allocated region, or NULL
 *  size: the new size to which the allocation should be shrunk or extended
 */
extern void *extmem_realloc(void *oldptr, size_t size);

/* frees a region previously allocated in external RAM
 * ---
 *  ptr: a pointer to the previously allocated region, or NULL
 */
extern void extmem_free(void *ptr);

#endif /* HYPERMAN_STDLIB_MEMORY_H */
