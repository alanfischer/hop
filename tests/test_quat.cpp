#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/fixed16.h>
#include <hop/math/mat3.h>
#include <hop/math/quat.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

using namespace hop;

template <typename T> static bool approx(T a, T b, float tol) {
	using tr = scalar_traits<T>;
	return std::fabs(tr::to_float(a) - tr::to_float(b)) < tol;
}

template <typename T> static bool approx_vec(const vec3<T> & a, const vec3<T> & b, float tol) {
	return approx(a.x, b.x, tol) && approx(a.y, b.y, tol) && approx(a.z, b.z, tol);
}

template <typename T> static void test_identity(const char * label) {
	printf("  identity[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q;
	assert(q.x == T {} && q.y == T {} && q.z == T {} && q.w == tr::one());
	printf("OK\n");
}

// q * q_conj = {0,0,0, |q|^2}
template <typename T> static void test_conjugate(const char * label, float tol) {
	printf("  conjugate[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q { tr::half(), -tr::quarter(), tr::half(), tr::half() };
	quat<T> c;
	conjugate(c, q);
	quat<T> prod;
	mul(prod, q, c);
	assert(approx(prod.x, T {}, tol));
	assert(approx(prod.y, T {}, tol));
	assert(approx(prod.z, T {}, tol));
	T ls = length_squared(q);
	assert(approx(prod.w, ls, tol));
	printf("OK\n");
}

// Axis-angle round trip: build quat from (axis, theta), rotate (1,0,0), check against hand result.
template <typename T> static void test_axis_angle_rotation(const char * label, float tol) {
	printf("  axis_angle_rotation[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q;
	vec3<T> axis { T {}, T {}, tr::one() };
	set_quat_from_axis_angle(q, axis, tr::half_pi());
	vec3<T> v { tr::one(), T {}, T {} };
	vec3<T> r;
	mul(r, q, v);
	assert(approx(r.x, T {}, tol));
	assert(approx(r.y, tr::one(), tol));
	assert(approx(r.z, T {}, tol));
	printf("OK\n");
}

// Quaternion-to-matrix: rotating a vector via quat and via the derived matrix should agree.
template <typename T> static void test_to_matrix_rotates_same(const char * label, float tol) {
	printf("  to_matrix_rotates_same[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q;
	vec3<T> axis { tr::one(), tr::one(), T {} };
	T inv_len = tr::one() / tr::sqrt(tr::two());
	axis.x *= inv_len; axis.y *= inv_len;
	set_quat_from_axis_angle(q, axis, tr::half_pi());
	mat3<T> m;
	set_mat3_from_quat(m, q);
	vec3<T> v { T {}, T {}, tr::one() };
	vec3<T> rq;
	mul(rq, q, v);
	vec3<T> rm;
	mul(rm, m, v);
	assert(approx_vec(rq, rm, tol));
	printf("OK\n");
}

// Matrix -> quat round trip: build rotation matrix, extract quat, rotate via both, compare.
template <typename T> static void test_matrix_quat_roundtrip(const char * label, float tol) {
	printf("  matrix_quat_roundtrip[%s]: ", label);
	using tr = scalar_traits<T>;
	vec3<T> axis { T {}, tr::one(), T {} };  // unit +Y
	mat3<T> m;
	set_mat3_from_axis_angle(m, axis, tr::half_pi());
	quat<T> q;
	set_quat_from_mat3(q, m);
	// Rotate (1,0,0) by both — should give (0,0,-1) for +90° about Y (right-handed)
	vec3<T> v { tr::one(), T {}, T {} };
	vec3<T> rq;
	mul(rq, q, v);
	vec3<T> rm;
	mul(rm, m, v);
	assert(approx_vec(rq, rm, tol));
	printf("OK\n");
}

// Normalize sets length to 1.
template <typename T> static void test_normalize(const char * label, float tol) {
	printf("  normalize[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q { tr::half(), tr::half(), tr::half(), tr::half() };  // length = 1 already
	// Scale it
	q.x *= tr::two(); q.y *= tr::two(); q.z *= tr::two(); q.w *= tr::two();
	normalize(q);
	assert(approx(length(q), tr::one(), tol));
	printf("OK\n");
}

// Slerp endpoints: t=0 returns q1, t=1 returns q2.
template <typename T> static void test_slerp_endpoints(const char * label, float tol) {
	printf("  slerp_endpoints[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q1;  // identity
	vec3<T> axis { T {}, T {}, tr::one() };
	quat<T> q2;
	set_quat_from_axis_angle(q2, axis, tr::half_pi());
	quat<T> r;
	slerp(r, q1, q2, T {});
	assert(approx(r.x, q1.x, tol));
	assert(approx(r.y, q1.y, tol));
	assert(approx(r.z, q1.z, tol));
	assert(approx(r.w, q1.w, tol));
	slerp(r, q1, q2, tr::one());
	assert(approx(r.x, q2.x, tol));
	assert(approx(r.y, q2.y, tol));
	assert(approx(r.z, q2.z, tol));
	assert(approx(r.w, q2.w, tol));
	printf("OK\n");
}

// Slerp midpoint produces a unit quat halfway in angle.
template <typename T> static void test_slerp_midpoint(const char * label, float tol) {
	printf("  slerp_midpoint[%s]: ", label);
	using tr = scalar_traits<T>;
	vec3<T> axis { T {}, T {}, tr::one() };
	quat<T> q1;  // identity
	quat<T> q2;
	set_quat_from_axis_angle(q2, axis, tr::half_pi());
	quat<T> mid;
	slerp(mid, q1, q2, tr::half());
	// mid should have unit length
	assert(approx(length(mid), tr::one(), tol));
	// Rotating (1,0,0) by mid should give the 45° rotation result: (cos 45, sin 45, 0)
	vec3<T> v { tr::one(), T {}, T {} };
	vec3<T> r;
	mul(r, mid, v);
	T sqrt2_over_2 = tr::sqrt(tr::two()) * tr::half();
	assert(approx(r.x, sqrt2_over_2, tol));
	assert(approx(r.y, sqrt2_over_2, tol));
	assert(approx(r.z, T {}, tol));
	printf("OK\n");
}

// Composition: (q1 * q2) * v == q1 * (q2 * v)
template <typename T> static void test_composition(const char * label, float tol) {
	printf("  composition[%s]: ", label);
	using tr = scalar_traits<T>;
	vec3<T> z { T {}, T {}, tr::one() };
	vec3<T> y { T {}, tr::one(), T {} };
	quat<T> qz, qy;
	set_quat_from_axis_angle(qz, z, tr::half_pi());
	set_quat_from_axis_angle(qy, y, tr::half_pi());
	quat<T> composed;
	mul(composed, qz, qy);
	vec3<T> v { tr::one(), T {}, T {} };
	// Applied separately
	vec3<T> r1;
	mul(r1, qy, v);
	vec3<T> r2;
	mul(r2, qz, r1);
	// Applied via composition
	vec3<T> rc;
	mul(rc, composed, v);
	assert(approx_vec(r2, rc, tol));
	printf("OK\n");
}

template <typename T> static void run_all_tests(const char * label, float tol) {
	printf(" [%s]\n", label);
	test_identity<T>(label);
	test_conjugate<T>(label, tol);
	test_axis_angle_rotation<T>(label, tol);
	test_to_matrix_rotates_same<T>(label, tol);
	test_matrix_quat_roundtrip<T>(label, tol);
	test_normalize<T>(label, tol);
	test_slerp_endpoints<T>(label, tol);
	test_slerp_midpoint<T>(label, tol);
	test_composition<T>(label, tol);
}

int main() {
	printf("test_quat:\n");
	run_all_tests<float>("float", 1e-4f);
	run_all_tests<fixed16>("fixed16", 0.02f);
	printf("ALL PASSED\n");
	return 0;
}
