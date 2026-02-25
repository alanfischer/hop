#pragma once

#include <hop/math/math_ops.h>

namespace hop {

// test_inside
template <typename T> inline bool test_inside(const plane<T> & p, const vec3<T> & point) {
	return dot(point, p.normal) <= p.distance;
}

template <typename T> inline bool test_inside(const sphere<T> & s, const vec3<T> & point) {
	return length_squared(point, s.origin) <= square(s.radius);
}

template <typename T> inline bool test_inside(const aa_box<T> & box, const vec3<T> & point) {
	return point.x >= box.mins.x && point.y >= box.mins.y && point.z >= box.mins.z && point.x <= box.maxs.x &&
	       point.y <= box.maxs.y && point.z <= box.maxs.z;
}

// test_intersection
template <typename T> inline bool test_intersection(const aa_box<T> & a, const aa_box<T> & b) {
	return !(a.mins.x > b.maxs.x || a.mins.y > b.maxs.y || a.mins.z > b.maxs.z || b.mins.x > a.maxs.x ||
	         b.mins.y > a.maxs.y || b.mins.z > a.maxs.z);
}

// find_intersection: segment vs plane
template <typename T>
inline T find_intersection(const segment<T> & seg, const plane<T> & p, vec3<T> & point, vec3<T> & normal) {
	using tr = scalar_traits<T>;
	T d = dot(p.normal, seg.direction);
	T zero_val {};
	if (d != zero_val) {
		T t = (p.distance - dot(p.normal, seg.origin)) / d;
		mul(point, seg.direction, t);
		add(point, seg.origin);
		normal = p.normal;
		if (t < zero_val || t > tr::one())
			return tr::one();
		return t;
	}
	return tr::one();
}

// find_intersection: segment vs sphere
template <typename T>
inline T find_intersection(const segment<T> & seg, const sphere<T> & sph, vec3<T> & point, vec3<T> & normal) {
	using tr = scalar_traits<T>;
	T zero_val {};
	const auto & so = seg.origin;
	const auto & sd = seg.direction;
	const auto & sp = sph.origin;

	vec3<T> diff;
	sub(diff, so, sp);
	T a = length_squared(sd);
	if (a <= zero_val)
		return tr::one();

	T b = dot(diff, sd);
	T c = length_squared(diff) - square(sph.radius);
	T time1 = tr::one();

	T discr = b * b - a * c;
	if (discr < zero_val) {
		return tr::one();
	} else if (discr > zero_val) {
		T root = tr::sqrt(discr);
		T inv_a = tr::one() / a;
		time1 = ((-b) - root) * inv_a;
		T time2 = ((-b) + root) * inv_a;

		if (time1 > tr::one() || time2 < zero_val) {
			return tr::one();
		} else if (time1 >= zero_val) {
			mul(point, sd, time1);
			add(point, so);
		} else {
			// Segment starts inside sphere — use exit point
			time1 = time2;
			mul(point, sd, time1);
			add(point, so);
		}
	} else {
		time1 = (-b) / a;
		if (zero_val <= time1 && time1 <= tr::one()) {
			mul(point, sd, time1);
			add(point, so);
		} else {
			return tr::one();
		}
	}

	if (time1 != tr::one()) {
		sub(normal, point, sp);
		normalize_carefully(normal, zero_val);
	}
	return time1;
}

// find_intersection: segment vs aa_box
template <typename T>
inline T find_intersection(const segment<T> & seg, const aa_box<T> & box, vec3<T> & point, vec3<T> & normal) {
	using tr = scalar_traits<T>;
	const auto & so = seg.origin;
	const auto & sd = seg.direction;
	const auto & bmn = box.mins;
	const auto & bmx = box.maxs;
	T zero_val {};
	T neg_one = -tr::one();

	bool inside = true;
	char qx, qy, qz;
	int which_plane;
	T max_tx {}, max_ty {}, max_tz {};
	T cand_x {}, cand_y {}, cand_z {};
	T cnorm_x = neg_one, cnorm_y = neg_one, cnorm_z = neg_one;
	T time = tr::one();

	// X
	if (so.x <= bmn.x) {
		qx = 0;
		cand_x = bmn.x;
		inside = false;
	} else if (so.x >= bmx.x) {
		qx = 1;
		cand_x = bmx.x;
		cnorm_x = tr::one();
		inside = false;
	} else {
		qx = 2;
	}
	// Y
	if (so.y <= bmn.y) {
		qy = 0;
		cand_y = bmn.y;
		inside = false;
	} else if (so.y >= bmx.y) {
		qy = 1;
		cand_y = bmx.y;
		cnorm_y = tr::one();
		inside = false;
	} else {
		qy = 2;
	}
	// Z
	if (so.z <= bmn.z) {
		qz = 0;
		cand_z = bmn.z;
		inside = false;
	} else if (so.z >= bmx.z) {
		qz = 1;
		cand_z = bmx.z;
		cnorm_z = tr::one();
		inside = false;
	} else {
		qz = 2;
	}

	if (inside)
		return zero_val;

	if (qx != 2 && sd.x != zero_val)
		max_tx = (cand_x - so.x) / sd.x;
	else
		max_tx = neg_one;
	if (qy != 2 && sd.y != zero_val)
		max_ty = (cand_y - so.y) / sd.y;
	else
		max_ty = neg_one;
	if (qz != 2 && sd.z != zero_val)
		max_tz = (cand_z - so.z) / sd.z;
	else
		max_tz = neg_one;

	if (max_tx > max_ty && max_tx > max_tz) {
		which_plane = 0;
		time = max_tx;
		normal.set(cnorm_x, zero_val, zero_val);
	} else if (max_ty > max_tz) {
		which_plane = 1;
		time = max_ty;
		normal.set(zero_val, cnorm_y, zero_val);
	} else {
		which_plane = 2;
		time = max_tz;
		normal.set(zero_val, zero_val, cnorm_z);
	}

	if (time < zero_val || time > tr::one())
		return tr::one();

	if (which_plane != 0) {
		point.x = so.x + time * sd.x;
		if (point.x < bmn.x || point.x > bmx.x)
			return tr::one();
	} else {
		point.x = cand_x;
	}
	if (which_plane != 1) {
		point.y = so.y + time * sd.y;
		if (point.y < bmn.y || point.y > bmx.y)
			return tr::one();
	} else {
		point.y = cand_y;
	}
	if (which_plane != 2) {
		point.z = so.z + time * sd.z;
		if (point.z < bmn.z || point.z > bmx.z)
			return tr::one();
	} else {
		point.z = cand_z;
	}

	return time;
}

} // namespace hop
