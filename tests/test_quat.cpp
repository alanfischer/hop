#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// Integration pattern: q_new = normalize(q + 0.5 * ω_quat * q * dt) applies a
// small rotation over time. After integrating 100 small steps of a constant
// angular velocity (0, 0, pi) for 1s total, we should end up ~180° rotated
// around Z: rotating (1,0,0) should give (-1, 0, 0).
//
// Tolerance is loose — forward Euler is O(dt) accurate and accumulates noticeable
// drift over this many steps even at float. See test_rotation_drift for detailed
// numbers. We use this as an end-to-end smoke test; numerical accuracy lives
// in the drift test.
template <typename T> static void test_integration_pattern(const char * label) {
	printf("  integration_pattern[%s]: ", label);
	using tr = scalar_traits<T>;
	quat<T> q;  // identity
	T dt = tr::from_milli(10);  // 10ms steps
	int steps = 100;             // 1 second total
	vec3<T> omega { T {}, T {}, tr::pi() };  // pi rad/s around Z => full pi rotation in 1s
	for (int i = 0; i < steps; ++i) {
		quat<T> omega_q { omega.x, omega.y, omega.z, T {} };
		quat<T> omega_times_q;
		mul(omega_times_q, omega_q, q);
		quat<T> dq = omega_times_q * (tr::half() * dt);
		q += dq;
		normalize(q);
	}
	vec3<T> v { tr::one(), T {}, T {} };
	vec3<T> r;
	mul(r, q, v);
	// After pi rad rotation around Z, (1,0,0) -> (-1, 0, 0)
	// 1% tolerance covers forward-Euler integration error at both float and fixed16.
	float tol = std::is_same_v<T, fixed16> ? 0.03f : 0.01f;
	assert(approx(r.x, -tr::one(), tol));
	assert(approx(r.y, T {}, tol));
	assert(approx(r.z, T {}, tol));
	printf("OK\n");
}

// Drift measurement: integrate ω = 2π rad/s around Z for N = 10 seconds = 10 full
// rotations at dt = 10ms (1000 sub-steps). After each whole-second mark (1 full
// rotation), the quaternion should be back at identity — any residual is pure
// numerical drift. Prints peak drift so we can see how much fixed16 wanders.
template <typename T> static void test_rotation_drift(const char * label) {
	printf("  rotation_drift[%s]: ", label);
	using tr = scalar_traits<T>;
	// Per-rev detail is noisy during ordinary CI; enable with HOP_VERBOSE=1.
	bool verbose = std::getenv("HOP_VERBOSE") != nullptr;
	quat<T> q;  // identity
	T dt = tr::from_milli(10);
	vec3<T> omega { T {}, T {}, tr::two_pi() };  // 1 rev/s around Z

	float peak_angle_err = 0.0f;
	float peak_length_err = 0.0f;
	float peak_xy_err = 0.0f;

	int steps_per_rev = 100;
	int total_revs = 10;
	if (verbose) printf("\n");
	for (int rev = 0; rev < total_revs; ++rev) {
		for (int i = 0; i < steps_per_rev; ++i) {
			quat<T> omega_q { omega.x, omega.y, omega.z, T {} };
			quat<T> omega_times_q;
			mul(omega_times_q, omega_q, q);
			q += omega_times_q * (tr::half() * dt);
			normalize(q);
		}
		// At end of each revolution, q should be ≈ identity.
		// Measure the error.
		float len = tr::to_float(length(q));
		float length_err = std::fabs(len - 1.0f);

		// Angle from identity = 2 * acos(|w|).  Near identity, ~2 * sqrt(1 - w^2).
		float w_abs = std::fabs(tr::to_float(q.w));
		if (w_abs > 1.0f) w_abs = 1.0f;
		float angle_err = 2.0f * std::acos(w_abs);

		// Rotating (1,0,0) — after full rev should give (1,0,0). Measure xy drift.
		vec3<T> v { tr::one(), T {}, T {} };
		vec3<T> r;
		mul(r, q, v);
		float x = tr::to_float(r.x);
		float y = tr::to_float(r.y);
		float xy_err = std::sqrt((x - 1.0f) * (x - 1.0f) + y * y);

		peak_length_err = std::max(peak_length_err, length_err);
		peak_angle_err = std::max(peak_angle_err, angle_err);
		peak_xy_err = std::max(peak_xy_err, xy_err);

		if (verbose) {
			printf("    rev %2d: |q|-1=%.2e, angle=%.3f rad (%.2f°), v_err=%.2e\n",
			       rev + 1, length_err, angle_err, angle_err * 180.0f / 3.14159265f, xy_err);
		}
	}
	printf("peak |q|-1=%.2e, angle=%.2f°, v_err=%.2e ",
	       peak_length_err, peak_angle_err * 180.0f / 3.14159265f, peak_xy_err);

	// Regression guard: observed behavior on 2026-04-22 with forward-Euler
	// integration. These are headroom-padded, not tight — if we change the
	// integration scheme or normalization, numbers should improve.
	//
	// Float (100 steps/rev × 10 revs):   |q|-1 ~ 6e-8,  angle ~ 1.2°,  v_err ~ 2.1%
	// Fixed16 (same):                    |q|-1 ~ 2e-5,  angle ~ 8.6°,  v_err ~ 15%
	//
	// The drift is dominated by forward-Euler error, not precision. In practice
	// we'd use a smaller dt or RK2/Heun for tighter integration.
	if constexpr (std::is_same_v<T, fixed16>) {
		assert(peak_length_err < 5e-4f);
		assert(peak_angle_err < 0.25f);      // rad (~14°)
		assert(peak_xy_err < 0.25f);
	} else {
		assert(peak_length_err < 1e-6f);
		assert(peak_angle_err < 0.05f);      // rad (~3°)
		assert(peak_xy_err < 0.05f);
	}
	printf("OK\n");
}

// Axis-angle round-trip: build quat from axis+angle, extract via
// get_axis_angle_from_quat, check both match. Near identity (small angle) the
// old acos(w)*2 path lost precision; this guards the atan2 replacement.
template <typename T> static void test_axis_angle_from_quat(const char * label, float tol) {
	printf("  axis_angle_from_quat[%s]: ", label);
	using tr = scalar_traits<T>;
	T eps;
	if constexpr (std::is_same_v<T, fixed16>)
		eps = fixed16::from_raw(4);
	else
		eps = T(1e-6);

	// 90° about +Z
	{
		vec3<T> axis_in { T {}, T {}, tr::one() };
		quat<T> q;
		set_quat_from_axis_angle(q, axis_in, tr::half_pi());
		vec3<T> axis_out;
		T angle = get_axis_angle_from_quat(axis_out, q, eps);
		assert(approx(angle, tr::half_pi(), tol));
		assert(approx_vec(axis_out, axis_in, tol));
	}

	// Identity → angle ≈ 0, axis falls back to (1,0,0).
	{
		quat<T> q;  // identity
		vec3<T> axis_out;
		T angle = get_axis_angle_from_quat(axis_out, q, eps);
		assert(approx(angle, T {}, tol));
		assert(axis_out.x == tr::one() && axis_out.y == T {} && axis_out.z == T {});
	}

	// Small-angle rotation around +Y. Under acos(w)*2 the angle collapses to 0
	// well before atan2 does, so pick a rotation tiny enough to expose the old
	// regime: 1/100th of half_pi ≈ 0.0157 rad ≈ 0.9°.
	{
		vec3<T> axis_in { T {}, tr::one(), T {} };
		T small_angle = tr::half_pi() / tr::from_int(100);
		quat<T> q;
		set_quat_from_axis_angle(q, axis_in, small_angle);
		vec3<T> axis_out;
		T angle = get_axis_angle_from_quat(axis_out, q, eps);
		float tol_small = std::is_same_v<T, fixed16> ? 5e-3f : 1e-4f;
		assert(approx(angle, small_angle, tol_small));
		// Axis should still point (approximately) along +Y.
		assert(approx_vec(axis_out, axis_in, std::is_same_v<T, fixed16> ? 0.1f : 1e-3f));
	}

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
	test_integration_pattern<T>(label);
	test_rotation_drift<T>(label);
	test_axis_angle_from_quat<T>(label, tol);
	test_composition<T>(label, tol);
}

int main() {
	printf("test_quat:\n");
	run_all_tests<float>("float", 1e-4f);
	run_all_tests<fixed16>("fixed16", 0.02f);
	printf("ALL PASSED\n");
	return 0;
}
