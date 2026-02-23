#pragma once

#include <vector>
#include <hop/math/plane.h>

namespace hop {

template<typename T>
struct convex_solid {
	std::vector<plane<T>> planes;

	convex_solid() = default;

	convex_solid& set(const convex_solid& cs) {
		planes = cs.planes;
		return *this;
	}
};

} // namespace hop
