#include "stdlib/threading.h"

struct stdlib_mutex {
	size_t lock; /* lock value */
};

b32 stdlib_mutex_try_lock(struct stdlib_mutex *mutex) {
	/* TODO */
	(void)mutex; /* to silence unused variable warnings, remove me */
	return false;
}

void stdlib_mutex_unlock(struct stdlib_mutex *mutex) {
	/* TODO */
	(void)mutex; /* to silence unused variable warnings, remove me */
}
