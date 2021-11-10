#ifndef HYPERMAN_STDLIB_THREADING_H
#define HYPERMAN_STDLIB_THREADING_H

#include "stdlib.h"

/* used to guard a resource that at most 1 thread can access at a time
 */
struct stdlib_mutex;

/* attempts to lock a mutex
 * ---
 *  mutex: the mutex instance to try to get a lock on
 */
extern b32 stdlib_mutex_try_lock(struct stdlib_mutex *mutex);

/* unlocks a previously locked mutex
 * ---
 *  mutex: the mutex instance that was previously locked
 */
extern void stdlib_mutex_unlock(struct stdlib_mutex *mutex);

/* used to distribute a given number of slots to multiple threads, ensuring
 * that the number of threads never exceeds the number of slots
 */
struct stdlib_semaphore;

/* attempts to accquire a single slot from a semaphore
 * ---
 *  semaphore: the semaphore from which to accquire the slot
 */
extern b32 stdlib_semaphore_try_take(struct stdlib_semaphore *semaphore);

/* attempts to accquire multiple slots from a semaphore
 * ---
 *  semaphore: the semaphore from which to accquire slots
 *  n: the number of slots to accquire
 */
extern b32 stdlib_semaphore_try_take_n(struct stdlib_semaphore *semaphore,
					size_t n);

/* releases a single slot to a semaphore
 * ---
 *  semaphore: the semaphore instance to which to release the slot
 */
extern void stdlib_semaphore_release(struct stdlib_semaphore *semaphore);

/* releases multiple slots to a semaphore
 * ---
 *  semaphore: the semaphore instance to which to release the slots
 *  n: the number of slots to release
 */
extern void stdlib_semaphore_release_n(struct stdlib_semaphore *semaphore,
				       size_t n);

/* a thread control block, to save state whilst a thread is not scheduled in
 */
struct stdlib_thread_control_block {
	/* TODO: complete me */
	reg32 sp;
};

/* a thread that can be scheduled in and out
 */
struct stdlib_thread {
	struct stdlib_thread_control_block tcb;
	u32 priority;
	u32 id;
};

#endif /* HYPERMAN_STDLIB_THREADING_H */
