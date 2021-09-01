#ifndef HYPERMAN_STDLIB_TEST_H
#define HYPERMAN_STDLIB_TEST_H

#include <assert.h>
#include <stdio.h>

#define TEST_BEGIN() \
	printf("*************** %s ***************\n", __FILE__); \
	int test_result = 0;

#define TEST_END() \
	printf("*************** %s ***************\n", __FILE__); \
	return test_result;

#define PASS() return 1;
#define FAIL() return 0;

#define TEST(test_name) int test_name(void)

#define TEST_RUN(test) \
{ \
	printf("%s: ", #test); \
	if (test()) { \
		printf("OK\n"); \
	} else { \
		printf("FAILED\n"); \
		test_result = 1; \
	} \
}

/* Assertions */
#define TEST_ASSERT(condition, message) \
	assert((condition) && message);

#define TEST_ASSERT_NE(condition, message) \
	assert(!(condition) && message);

#endif // HYPERMAN_STDLIB_TEST_H
