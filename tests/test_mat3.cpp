#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/fixed16.h>
#include <hop/math/mat3.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

using namespace hop;

template <typename T> static bool approx(T a, T b, float tol) {
	using tr = scalar_traits<T>;
	return std::fabs(tr::to_float(a) - tr::to_float(b)) < tol;
}

template <typename T> static void test_identity(const char * label) {
	printf("  identity[%s]: ", label);
	using tr = scalar_traits<T>;
	mat3<T> m;
	for (int r = 0; r < 3; ++r)
		for (int c = 0; c < 3; ++c)
			assert(m.at(r, c) == (r == c ? tr::one() : T {}));
	printf("OK\n");
}

template <typename T> static void test_multiply_identity(const char * label, float tol) {
	printf("  multiply_identity[%s]: ", label);
	using tr = scalar_traits<T>;
	mat3<T> m(
	    tr::one(), tr::two(), tr::three(),
	    -tr::one(), tr::half(), T {},
	    tr::three(), -tr::two(), tr::one());
	mat3<T> ident;
	mat3<T> r;
	mul(r, m, ident);
	for (int i = 0; i < 9; ++i)
		assert(approx(r.data[i], m.data[i], tol));
	mul(r, ident, m);
	for (int i = 0; i < 9; ++i)
		assert(approx(r.data[i], m.data[i], tol));
	printf("OK\n");
}

template <typename T> static void test_transpose(const char * label) {
	printf("  transpose[%s]: ", label);
	using tr = scalar_traits<T>;
	mat3<T> m(
	    tr::one(), tr::two(), tr::three(),
	    -tr::one(), tr::half(), T {},
	    tr::three(), -tr::two(), tr::one());
	mat3<T> t1;
	transpose(t1, m);
	mat3<T> t2;
	transpose(t2, t1);
	// transpose of transpose == original
	for (int i = 0; i < 9; ++i)
		assert(t2.data[i] == m.data[i]);
	// In-place: transpose(m, m) must match a separate transpose.
	mat3<T> in_place;
	in_place.set(m);
	transpose(in_place, in_place);
	for (int i = 0; i < 9; ++i)
		assert(in_place.data[i] == t1.data[i]);
	printf("OK\n");
}

template <typename T> static void test_axis_angle_rotation(const char * label, float tol) {
	printf("  axis_angle_rotation[%s]: ", label);
	using tr = scalar_traits<T>;
	// 90° about z: (1,0,0) -> (0,1,0)
	mat3<T> m;
	vec3<T> axis { T {}, T {}, tr::one() };
	set_mat3_from_axis_angle(m, axis, tr::half_pi());
	vec3<T> v { tr::one(), T {}, T {} };
	vec3<T> r;
	mul(r, m, v);
	assert(approx(r.x, T {}, tol));
	assert(approx(r.y, tr::one(), tol));
	assert(approx(r.z, T {}, tol));
	printf("OK\n");
}

template <typename T> static void test_rotation_is_orthonormal(const char * label, float tol) {
	printf("  rotation_is_orthonormal[%s]: ", label);
	using tr = scalar_traits<T>;
	// Arbitrary axis-angle rotation
	vec3<T> axis { tr::one(), tr::one(), tr::one() };
	T inv_len = tr::one() / tr::sqrt(tr::three());
	axis.x *= inv_len; axis.y *= inv_len; axis.z *= inv_len;
	mat3<T> r;
	set_mat3_from_axis_angle(r, axis, tr::half_pi());
	mat3<T> rt;
	transpose(rt, r);
	mat3<T> prod;
	mul(prod, r, rt);
	// R * R^T should equal identity
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			assert(approx(prod.at(i, j), i == j ? tr::one() : T {}, tol));
	printf("OK\n");
}

template <typename T> static void test_invert(const char * label, float tol) {
	printf("  invert[%s]: ", label);
	using tr = scalar_traits<T>;
	// Start with a rotation matrix (always invertible, and its inverse is transpose)
	vec3<T> axis { T {}, tr::one(), T {} };
	mat3<T> r;
	set_mat3_from_axis_angle(r, axis, tr::half_pi());
	mat3<T> inv;
	bool ok = invert(inv, r);
	assert(ok);
	mat3<T> prod;
	mul(prod, r, inv);
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			assert(approx(prod.at(i, j), i == j ? tr::one() : T {}, tol));
	// In-place: invert(r, r) must match a separate invert.
	mat3<T> in_place;
	in_place.set(r);
	ok = invert(in_place, in_place);
	assert(ok);
	for (int i = 0; i < 9; ++i)
		assert(approx(in_place.data[i], inv.data[i], tol));
	printf("OK\n");
}

template <typename T> static void run_all_tests(const char * label, float tol) {
	printf(" [%s]\n", label);
	test_identity<T>(label);
	test_multiply_identity<T>(label, tol);
	test_transpose<T>(label);
	test_axis_angle_rotation<T>(label, tol);
	test_rotation_is_orthonormal<T>(label, tol);
	test_invert<T>(label, tol);
}

int main() {
	printf("test_mat3:\n");
	run_all_tests<float>("float", 1e-4f);
	run_all_tests<fixed16>("fixed16", 0.02f);
	printf("ALL PASSED\n");
	return 0;
}
