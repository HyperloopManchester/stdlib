#include "stdlib/memory.h"

/* our actual heap on which we allocate
 */
u8 heap[HEAP_SIZE];

/* used to keep track of heap allocations
 */
struct stdlib_heap_region;
struct stdlib_heap_region {
	/* TODO */
	u8 *ptr;
	size_t len;
	struct stdlib_heap_region *next;
};

/* dummy heap region from which we search for actual allocations
 */
__attribute__ ((used))
static struct stdlib_heap_region root_heap_region = {
	.ptr = NULL,
	.len = 0,
	.next = NULL,
};

void *malloc(size_t size) {
	/* TODO */
	(void)size; /* to silence unused variable warnings, remove me */
	return NULL;
}

void *realloc(void *oldptr, size_t size) {
	/* TODO */
	(void)oldptr; /* to silence unused variable warnings, remove me */
	(void)size; /* to silence unused variable warnings, remove me */
	return NULL;
}

void free(void *ptr) {
	/* TODO */
	(void)ptr; /* to silence unused variable warnings, remove me */ 
}
