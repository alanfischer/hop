#pragma once

#include <hop/math/math_ops.h>

namespace hop {

template<typename T>
inline void find_bounding_box(aa_box<T>& r, const sphere<T>& s) {
	T radius = s.radius;
	r.mins.set(-radius, -radius, -radius);
	r.maxs.set(radius, radius, radius);
	add(r, s.origin);
}

template<typename T>
inline void find_bounding_box(aa_box<T>& r, const capsule<T>& c) {
	T radius = c.radius;
	const auto& d = c.direction;
	T zero_val{};

	if (d.x < zero_val) { r.mins.x = d.x - radius; r.maxs.x = radius; }
	else { r.mins.x = -radius; r.maxs.x = d.x + radius; }
	if (d.y < zero_val) { r.mins.y = d.y - radius; r.maxs.y = radius; }
	else { r.mins.y = -radius; r.maxs.y = d.y + radius; }
	if (d.z < zero_val) { r.mins.z = d.z - radius; r.maxs.z = radius; }
	else { r.mins.z = -radius; r.maxs.z = d.z + radius; }

	add(r, c.origin);
}

} // namespace hop
