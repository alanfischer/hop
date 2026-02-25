#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/fixed16.h>
#include <hop/scalar_traits.h>

using namespace hop;

static void test_basic_arithmetic() {
	auto a = fixed16::from_int(3);
	auto b = fixed16::from_int(4);
	assert((a + b) == fixed16::from_int(7));
	assert((a - b) == fixed16::from_int(-1));
	assert((a * b) == fixed16::from_int(12));
	assert((b / a).to_int() == 1); // 4/3 = 1 in integer
	printf("  basic arithmetic: OK\n");
}

static void test_fixed_mul_div() {
	auto half = fixed16::from_raw(32768); // 0.5
	auto two = fixed16::from_int(2);
	auto one = fixed16::from_int(1);
	assert(half * two == one);
	assert(one / two == half);
	printf("  mul/div: OK\n");
}

static void test_from_milli() {
	auto v = fixed16::from_milli(500);
	// Should be approximately 0.5
	float f = v.to_float();
	assert(f > 0.49f && f < 0.51f);
	printf("  from_milli: OK\n");
}

static void test_comparisons() {
	auto a = fixed16::from_int(1);
	auto b = fixed16::from_int(2);
	assert(a < b);
	assert(b > a);
	assert(a <= a);
	assert(a >= a);
	assert(a != b);
	assert(a == a);
	printf("  comparisons: OK\n");
}

static void test_negation() {
	auto a = fixed16::from_int(5);
	auto b = -a;
	assert(b == fixed16::from_int(-5));
	assert(a + b == fixed16::from_int(0));
	printf("  negation: OK\n");
}

static void test_traits_abs() {
	using tr = scalar_traits<fixed16>;
	auto neg = fixed16::from_int(-7);
	auto pos = tr::abs(neg);
	assert(pos == fixed16::from_int(7));
	printf("  traits abs: OK\n");
}

static void test_traits_sqrt() {
	using tr = scalar_traits<fixed16>;
	auto four = fixed16::from_int(4);
	auto result = tr::sqrt(four);
	float f = result.to_float();
	assert(f > 1.9f && f < 2.1f);
	printf("  traits sqrt: OK\n");
}

static void test_traits_sin_cos() {
	using tr = scalar_traits<fixed16>;
	auto zero = fixed16::from_raw(0);
	auto s = tr::sin(zero);
	auto c = tr::cos(zero);
	assert(std::fabs(s.to_float()) < 0.01f);
	assert(std::fabs(c.to_float() - 1.0f) < 0.01f);
	printf("  traits sin/cos: OK\n");
}

int main() {
	printf("test_fixed16:\n");
	test_basic_arithmetic();
	test_fixed_mul_div();
	test_from_milli();
	test_comparisons();
	test_negation();
	test_traits_abs();
	test_traits_sqrt();
	test_traits_sin_cos();
	printf("ALL PASSED\n");
	return 0;
}
