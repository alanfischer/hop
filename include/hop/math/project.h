#pragma once

#include <hop/math/math_ops.h>

namespace hop {

// Project point onto segment
template<typename T>
inline void project(vec3<T>& result, const segment<T>& seg, const vec3<T>& point, bool limit_to_segment) {
	const vec3<T>& o = seg.origin;
	const vec3<T>& d = seg.direction;

	T zero_val{};
	if (d.x == zero_val && d.y == zero_val && d.z == zero_val) {
		result = o;
	} else {
		T u = (d.x * (point.x - o.x) + d.y * (point.y - o.y) + d.z * (point.z - o.z)) /
		      (d.x * d.x + d.y * d.y + d.z * d.z);
		if (limit_to_segment) {
			using tr = scalar_traits<T>;
			if (u < zero_val) u = zero_val;
			else if (u > tr::one()) u = tr::one();
		}
		result.x = o.x + u * d.x;
		result.y = o.y + u * d.y;
		result.z = o.z + u * d.z;
	}
}

// Project closest points between two segments
template<typename T>
inline void project(vec3<T>& point1, vec3<T>& point2, const segment<T>& seg1, const segment<T>& seg2, T epsilon) {
	T a = dot(seg1.direction, seg1.direction);
	T b = dot(seg1.direction, seg2.direction);
	T c = dot(seg2.direction, seg2.direction);
	using tr = scalar_traits<T>;
	T zero_val{};
	T one_val = tr::one();

	if (a <= epsilon) {
		point1 = seg1.origin;
		project(point2, seg2, point1, true);
		return;
	} else if (c < epsilon) {
		point2 = seg2.origin;
		project(point1, seg1, point2, true);
		return;
	}

	sub(point1, seg1.origin, seg2.origin);
	T d = dot(seg1.direction, point1);
	T e = dot(seg2.direction, point1);
	T denom = a * c - b * b;

	T u1N{}, u2N{};
	T u1D = denom, u2D = denom;

	if (denom < tr::from_milli(1)) {
		u1N = zero_val;
		u1D = one_val;
		u2N = e;
		u2D = c;
	} else {
		u1N = b * e - c * d;
		u2N = a * e - b * d;

		if (u1N < zero_val) {
			u1N = zero_val;
			u2N = e;
			u2D = c;
		} else if (u1N > u1D) {
			u1N = u1D;
			u2N = e + b;
			u2D = c;
		}
	}

	if (u2N < zero_val) {
		u2N = zero_val;
		if (-d < zero_val) {
			u1N = zero_val;
		} else if (-d > a) {
			u1N = u1D;
		} else {
			u1N = -d;
			u1D = a;
		}
	} else if (u2N > u2D) {
		u2N = u2D;
		if ((-d + b) < zero_val) {
			u1N = zero_val;
		} else if ((-d + b) > a) {
			u1N = u1D;
		} else {
			u1N = -d + b;
			u1D = a;
		}
	}

	T u1 = u1N / u1D;
	T u2 = u2N / u2D;

	mul(point1, seg1.direction, u1);
	add(point1, seg1.origin);
	mul(point2, seg2.direction, u2);
	add(point2, seg2.origin);
}

} // namespace hop
