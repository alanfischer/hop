#pragma once

#include <hop/math/plane.h>
#include <hop/math/vec3.h>
#include <vector>

namespace hop {

// Convex polyhedron defined by its bounding half-spaces. `planes` is the
// authoritative representation. `vertices` is an optional cache of the plane
// intersections that lie inside every other half-space; support() and
// bounding-box queries use it when present and fall back to on-the-fly
// enumeration when empty.
//
// Populate by calling rebuild_vertices() (in math_ops.h) after setting planes.
// Mutating `planes` directly invalidates the cache — the user should re-run
// rebuild_vertices() or clear() it.
template <typename T> struct convex_solid {
	std::vector<plane<T>> planes;
	std::vector<vec3<T>> vertices;

	convex_solid() = default;

	convex_solid & set(const convex_solid & cs) {
		planes = cs.planes;
		vertices = cs.vertices;
		return *this;
	}
};

} // namespace hop
