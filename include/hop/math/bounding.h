#pragma once

#include <hop/math/mat3.h>
#include <hop/math/support.h>

namespace hop {

// AABB enclosing `box` after rotating it by `R` about the origin. Standard
// rotated-AABB: the rotated center R·c, expanded by |R|·half-extents. Identity
// R reproduces `box` exactly. Safe to call with &out == &box.
template <typename T> inline void rotate_aabb(aa_box<T> & out, const aa_box<T> & box, const mat3<T> & R) {
	using tr = scalar_traits<T>;
	vec3<T> c, h;
	c.x = (box.mins.x + box.maxs.x) * tr::half();
	c.y = (box.mins.y + box.maxs.y) * tr::half();
	c.z = (box.mins.z + box.maxs.z) * tr::half();
	h.x = (box.maxs.x - box.mins.x) * tr::half();
	h.y = (box.maxs.y - box.mins.y) * tr::half();
	h.z = (box.maxs.z - box.mins.z) * tr::half();
	vec3<T> rc;
	mul(rc, R, c);
	vec3<T> rh;
	rh.x = tr::abs(R.at(0, 0)) * h.x + tr::abs(R.at(0, 1)) * h.y + tr::abs(R.at(0, 2)) * h.z;
	rh.y = tr::abs(R.at(1, 0)) * h.x + tr::abs(R.at(1, 1)) * h.y + tr::abs(R.at(1, 2)) * h.z;
	rh.z = tr::abs(R.at(2, 0)) * h.x + tr::abs(R.at(2, 1)) * h.y + tr::abs(R.at(2, 2)) * h.z;
	out.mins.x = rc.x - rh.x; out.maxs.x = rc.x + rh.x;
	out.mins.y = rc.y - rh.y; out.maxs.y = rc.y + rh.y;
	out.mins.z = rc.z - rh.z; out.maxs.z = rc.z + rh.z;
}

template <typename T> inline void find_bounding_box(aa_box<T> & r, const sphere<T> & s) {
	T radius = s.radius;
	r.mins.set(-radius, -radius, -radius);
	r.maxs.set(radius, radius, radius);
	add(r, s.origin);
}

template <typename T> inline void find_bounding_box(aa_box<T> & r, const capsule<T> & c) {
	T radius = c.radius;
	const auto & d = c.direction;
	T zero_val {};

	if (d.x < zero_val) {
		r.mins.x = d.x - radius;
		r.maxs.x = radius;
	} else {
		r.mins.x = -radius;
		r.maxs.x = d.x + radius;
	}
	if (d.y < zero_val) {
		r.mins.y = d.y - radius;
		r.maxs.y = radius;
	} else {
		r.mins.y = -radius;
		r.maxs.y = d.y + radius;
	}
	if (d.z < zero_val) {
		r.mins.z = d.z - radius;
		r.maxs.z = radius;
	} else {
		r.mins.z = -radius;
		r.maxs.z = d.z + radius;
	}

	add(r, c.origin);
}

} // namespace hop
