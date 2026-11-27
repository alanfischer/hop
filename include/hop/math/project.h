#pragma once

#include <hop/math/segment.h>
#include <hop/math/support.h>
#include <hop/scalar_traits.h>

#include <type_traits>

namespace hop {

// Project point onto segment
template <typename T>
inline void project(vec3<T> & result, const segment<T> & seg, const vec3<T> & point, bool limit_to_segment) {
	const vec3<T> & o = seg.origin;
	const vec3<T> & d = seg.direction;

	T zero_val {};
	if (d.x == zero_val && d.y == zero_val && d.z == zero_val) {
		result = o;
	} else {
		T u = (d.x * (point.x - o.x) + d.y * (point.y - o.y) + d.z * (point.z - o.z)) /
		      (d.x * d.x + d.y * d.y + d.z * d.z);
		if (limit_to_segment) {
			using tr = scalar_traits<T>;
			if (u < zero_val)
				u = zero_val;
			else if (u > tr::one())
				u = tr::one();
		}
		result.x = o.x + u * d.x;
		result.y = o.y + u * d.y;
		result.z = o.z + u * d.z;
	}
}

// n / den clamped to [0, 1], for den > 0. Clamping on the numerator first means
// the division only ever runs with |n| < den, so it can't overflow fixed-point.
template <typename T>
inline T project_unit_ratio(T n, T den) {
	using tr = scalar_traits<T>;
	if (n <= T {})
		return T {};
	if (n >= den)
		return tr::one();
	return n / den;
}

// Project closest points between two segments (Ericson, Real-Time Collision
// Detection §5.1.9). point1 lies on seg1, point2 on seg2.
template <typename T>
inline void project(vec3<T> & point1, vec3<T> & point2, const segment<T> & seg1, const segment<T> & seg2, T epsilon) {
	using tr = scalar_traits<T>;
	const T zero {};
	const T one = tr::one();

	// a and c are squared lengths, so the degenerate (point-like) test compares
	// against epsilon², not epsilon — otherwise any segment shorter than √epsilon
	// (≈3 cm at epsilon 0.001) collapses to its origin.
	const T eps2 = tr::epsilon_squared(epsilon); // representable even in fixed16
	if (dot(seg1.direction, seg1.direction) <= eps2) {
		point1 = seg1.origin;
		project(point2, seg2, point1, true);
		return;
	}
	if (dot(seg2.direction, seg2.direction) <= eps2) {
		point2 = seg2.origin;
		project(point1, seg1, point2, true);
		return;
	}

	// The solve below forms degree-4 products of the inputs (a·c, b·e, ...). In
	// fixed16 a 0.1-long segment leaves those only a few bits of precision, and a
	// ~13-long one overflows them. The parameters u1/u2 are invariant under a
	// uniform scale, so fixed16 rescales by a power of two first, sized on the
	// segment directions (see below) — the same idea as gjk_fit_scale. float,
	// double and fixed32 have the range and precision already and skip it.
	vec3<T> r, d1 = seg1.direction, d2 = seg2.direction;
	sub(r, seg1.origin, seg2.origin);
	if constexpr (std::is_same_v<T, fixed16>) {
		// Done on the raw bits: the scale is a power of two, so it's a shift. OR-ing
		// the magnitudes gives the same top bit as their max, without compares.
		auto mag = [](const vec3<T> & v) {
			auto m1 = [](int32_t x) { return x < 0 ? 0u - static_cast<uint32_t>(x) : static_cast<uint32_t>(x); };
			return m1(v.x.raw) | m1(v.y.raw) | m1(v.z.raw);
		};
		auto topbit = [](uint32_t m) { // index of m's highest set bit (0 for m == 0)
			int top = 0;
			if (m >> 16) { m >>= 16; top += 16; }
			if (m >> 8) { m >>= 8; top += 8; }
			if (m >> 4) { m >>= 4; top += 4; }
			if (m >> 2) { m >>= 2; top += 2; }
			if (m >> 1) { top += 1; }
			return top;
		};
		// The offset r only enters the products to the first power (d = d1·r, e = d2·r),
		// so scale on the directions alone: lift short ones into [1, 2), shrink long ones
		// only past |d| >= 8, then cap the shift so r can't overflow.
		const int kd = topbit(mag(d1) | mag(d2)) - fixed16::bits;
		int shift = -kd;
		if (shift < 0) shift = kd > 2 ? 2 - kd : 0;
		const int topr = topbit(mag(r));
		if (topr + shift > fixed16::bits + 8) shift = fixed16::bits + 8 - topr;
		if (shift > 0) {
			auto up = [shift](vec3<T> & v) { // shift as unsigned: `<<` on a negative is UB
				v.x.raw = static_cast<int32_t>(static_cast<uint32_t>(v.x.raw) << shift);
				v.y.raw = static_cast<int32_t>(static_cast<uint32_t>(v.y.raw) << shift);
				v.z.raw = static_cast<int32_t>(static_cast<uint32_t>(v.z.raw) << shift);
			};
			up(d1); up(d2); up(r);
		} else if (shift < 0) {
			auto down = [shift](vec3<T> & v) { v.x.raw >>= -shift; v.y.raw >>= -shift; v.z.raw >>= -shift; };
			down(d1); down(d2); down(r);
		}
	}
	const T a = dot(d1, d1);
	const T c = dot(d2, d2);
	const T b = dot(d1, d2);
	const T d = dot(d1, r);
	const T e = dot(d2, r);

	// denom = a·c·sin²θ. The parallel test must be relative to a·c: denom scales
	// with length⁴, so an absolute cutoff (the old 1e-3) declared short segments
	// "parallel" up to ~16° apart (0.37 vs 0.3 long) and then reported a pair that
	// isn't the closest. sin²θ < 1e-6 (≈0.06°) is parallel; the fixed-point
	// products underflow that to 0, i.e. "parallel only when denom vanishes".
	// Either way u2 below is derived from u1, so even a noisy u1 on near-parallel
	// input yields a consistent pair (u2 optimal for u1, then u1 re-fit if u2 clamps).
	const T denom = a * c - b * b;
	const T parallel = a * c * (tr::from_milli(1) * tr::from_milli(1));
	T u1 = denom > parallel ? project_unit_ratio(b * e - c * d, denom) : zero;

	// Closest point on seg2's line to seg1(u1); if that falls off seg2, clamp it
	// and re-fit u1 to the clamped endpoint.
	const T n2 = b * u1 + e;
	T u2;
	if (n2 <= zero) {
		u2 = zero;
		u1 = project_unit_ratio(-d, a);
	} else if (n2 >= c) {
		u2 = one;
		u1 = project_unit_ratio(b - d, a);
	} else {
		u2 = n2 / c;
	}

	mul(point1, seg1.direction, u1);
	add(point1, seg1.origin);
	mul(point2, seg2.direction, u2);
	add(point2, seg2.origin);
}

} // namespace hop
