#include "stdlib/test.h"

TEST(example) {
	int a = 42;

	ASSERT(a == 42, "a == 42");
	ASSERT_NE(a == 24, "a != 24");

	// FAIL();

	PASS();
}

int main(void) {
	TEST_BEGIN();

	TEST_RUN(example);

	TEST_END();
}
