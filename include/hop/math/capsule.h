#pragma once

#include <hop/math/vec3.h>

namespace hop {

template<typename T>
struct capsule {
	vec3<T> origin;
	vec3<T> direction;
	T radius{};

	constexpr capsule() = default;
	constexpr capsule(const vec3<T>& o, const vec3<T>& d, T r) : origin(o), direction(d), radius(r) {}

	capsule& set(const capsule& c) { origin = c.origin; direction = c.direction; radius = c.radius; return *this; }
	capsule& set(const vec3<T>& o, const vec3<T>& d, T r) { origin = o; direction = d; radius = r; return *this; }
	capsule& reset() { origin.reset(); direction.reset(); radius = T{}; return *this; }

	bool operator==(const capsule& c) const { return origin == c.origin && direction == c.direction && radius == c.radius; }
	bool operator!=(const capsule& c) const { return !(*this == c); }

	capsule operator+(const vec3<T>& v) const { return {origin + v, direction, radius}; }
	void operator+=(const vec3<T>& v) { origin += v; }
	capsule operator-(const vec3<T>& v) const { return {origin - v, direction, radius}; }
	void operator-=(const vec3<T>& v) { origin -= v; }
};

} // namespace hop
