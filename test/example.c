#include "stdlib.h"
#include "stdlib/test.h"

// minimal "test" possible
TEST(test_example_foo) {
	PASS();
}

// using TEST_ASSERT to assert that a condition is true
TEST(test_example_bar) {
	int a = 42;

	TEST_ASSERT(a == 42, "a should always have the value of 42");

	PASS();
}

// using TEST_ASSERT_NE to assert that a condition is false
TEST(test_example_baz) {
	int a = 42;

	TEST_ASSERT_NE(a != 42, "a should always have the value of 42");

	PASS();
}

// PASS() will make a test pass, FAIL() will make a test fail
TEST(test_example_unused) {
	FAIL();
}

// example of including a set of test cases
int main(void) {
	TEST_BEGIN();

	TEST_RUN(test_example_foo);
	TEST_RUN(test_example_bar);
	TEST_RUN(test_example_baz);
	// TEST_RUN(test_example_unused);

	TEST_END();
}
