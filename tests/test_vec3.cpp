#include <hop/hop.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace hop;

template<typename T>
static void test_basic_ops() {
	using tr = scalar_traits<T>;
	vec3<T> a{tr::one(), tr::two(), tr::three()};
	vec3<T> b{tr::one(), tr::one(), tr::one()};
	vec3<T> r;

	add(r, a, b);
	assert(r.x == tr::two());
	assert(r.y == tr::three());

	sub(r, a, b);
	assert(r.x == T{});
	assert(r.y == tr::one());

	mul(r, a, tr::two());
	assert(r.x == tr::two());

	printf("  vec3 basic ops: OK\n");
}

template<typename T>
static void test_dot_cross() {
	using tr = scalar_traits<T>;
	vec3<T> x{tr::one(), T{}, T{}};
	vec3<T> y{T{}, tr::one(), T{}};

	T d = dot(x, y);
	assert(d == T{});

	vec3<T> r;
	cross(r, x, y);
	// x cross y = z
	assert(r.z == tr::one());
	assert(r.x == T{});
	assert(r.y == T{});

	printf("  vec3 dot/cross: OK\n");
}

template<typename T>
static void test_length_normalize() {
	using tr = scalar_traits<T>;
	vec3<T> v{tr::three(), tr::four(), T{}};
	T len = length(v);
	float flen = tr::to_float(len);
	assert(flen > 4.9f && flen < 5.1f);

	normalize(v);
	T len2 = length(v);
	float flen2 = tr::to_float(len2);
	assert(flen2 > 0.95f && flen2 < 1.05f);

	printf("  vec3 length/normalize: OK\n");
}

int main() {
	printf("test_vec3 (float):\n");
	test_basic_ops<float>();
	test_dot_cross<float>();
	test_length_normalize<float>();

	printf("test_vec3 (fixed16):\n");
	test_basic_ops<fixed16>();
	test_dot_cross<fixed16>();
	test_length_normalize<fixed16>();

	printf("ALL PASSED\n");
	return 0;
}
