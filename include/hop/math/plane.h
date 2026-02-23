#pragma once

#include <hop/math/vec3.h>

namespace hop {

template<typename T>
struct plane {
	vec3<T> normal;
	T distance{};

	constexpr plane() = default;
	constexpr plane(const vec3<T>& n, T d) : normal(n), distance(d) {}
	constexpr plane(T x, T y, T z, T d) : normal(x, y, z), distance(d) {}

	plane& set(const plane& p) { normal = p.normal; distance = p.distance; return *this; }
	plane& set(const vec3<T>& n, T d) { normal = n; distance = d; return *this; }
	plane& set(T x, T y, T z, T d) { normal.set(x, y, z); distance = d; return *this; }
	plane& reset() { normal.reset(); distance = T{}; return *this; }

	bool operator==(const plane& p) const { return normal == p.normal && distance == p.distance; }
	bool operator!=(const plane& p) const { return !(*this == p); }
};

} // namespace hop
