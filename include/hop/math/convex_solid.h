#pragma once

#include <hop/math/plane.h>
#include <hop/math/vec3.h>
#include <vector>

namespace hop {

// Convex polyhedron defined by its bounding half-spaces. `planes` is the
// authoritative representation. `vertices` is a cache populated lazily by
// ensure_vertices() (called from support() and bounding-box queries).
//
// If you mutate `planes` directly after the cache has been populated, clear
// `vertices` (or call rebuild_vertices()) — the cache is not auto-invalidated.
template <typename T> struct convex_solid {
	std::vector<plane<T>> planes;
	mutable std::vector<vec3<T>> vertices;

	convex_solid() = default;

	convex_solid & set(const convex_solid & cs) {
		planes = cs.planes;
		vertices = cs.vertices;
		return *this;
	}
};

} // namespace hop
