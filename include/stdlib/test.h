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

#define PASS() return 0;
#define FAIL() return 1;

#define TEST(name) int name(void)

#define TEST_RUN(name) \
{ \
	printf("%s: ", #name); \
	if (name()) { \
		printf("FAILED\n"); \
		test_result = 1; \
	} else { \
		printf("OK\n"); \
	} \
}

/* Assertions */
#define ASSERT(condition, message) \
{ \
	if (!(condition)) { \
		printf("%s\n", message); \
		FAIL(); \
	} \
}

#define ASSERT_NE(condition, message) \
{ \
	if ((condition)) { \
		printf("%s\n", message); \
		FAIL(); \
	} \
}

#endif /* HYPERMAN_STDLIB_TEST_H */
