#pragma once

#include <hop/math/vec3.h>

namespace hop {

template<typename T>
struct segment {
	vec3<T> origin;
	vec3<T> direction;

	constexpr segment() = default;
	constexpr segment(const segment& s) : origin(s.origin), direction(s.direction) {}

	segment& set(const segment& s) { origin = s.origin; direction = s.direction; return *this; }

	segment& set_start_end(const vec3<T>& start, const vec3<T>& end) {
		origin = start;
		direction.set(end.x - start.x, end.y - start.y, end.z - start.z);
		return *this;
	}

	segment& set_start_dir(const vec3<T>& start, const vec3<T>& dir) {
		origin = start;
		direction = dir;
		return *this;
	}

	segment& reset() { origin.reset(); direction.reset(); return *this; }

	vec3<T> get_end_point() const {
		return {origin.x + direction.x, origin.y + direction.y, origin.z + direction.z};
	}

	vec3<T>& get_end_point(vec3<T>& result) const {
		result.x = origin.x + direction.x;
		result.y = origin.y + direction.y;
		result.z = origin.z + direction.z;
		return result;
	}

	bool operator==(const segment& s) const { return origin == s.origin && direction == s.direction; }
	bool operator!=(const segment& s) const { return !(*this == s); }
};

} // namespace hop
