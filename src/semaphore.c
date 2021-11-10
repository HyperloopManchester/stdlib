#include "stdlib/threading.h"

struct stdlib_semaphore {
	size_t slots;
	size_t taken;
};

b32 stdlib_semaphore_try_take(struct stdlib_semaphore *semaphore) {
	/* TODO */
	(void)semaphore; /* to silence unused variable warnings, remove me */
	return false;
}

b32 stdlib_semaphore_try_take_n(struct stdlib_semaphore *semaphore, size_t n) {
	/* TODO */
	(void)semaphore; /* to silence unused variable warnings, remove me */
	(void)n; /* to silence unused variable warnings, remove me */
	return false;
}

void stdlib_semaphore_release(struct stdlib_semaphore *semaphore) {
	/* TODO */
	(void)semaphore; /* to silence unused variable warnings, remove me */
}

void stdlib_semaphore_release_n(struct stdlib_semaphore *semaphore, size_t n) {
	/* TODO */
	(void)semaphore; /* to silence unused variable warnings, remove me */
	(void)n; /* to silence unused variable warnings, remove me */
}
