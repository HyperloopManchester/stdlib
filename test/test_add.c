#include "stdlib.h"
#include "stdlib/test.h"

TEST(add) {
	int a = 42, b = 69;

	ASSERT(stdlib_add(a, b) == 111, "42 + 69 == 111");

	PASS();
}

int main(void) {
	TEST_BEGIN();

	TEST_RUN(add);

	TEST_END();
}
