#pragma once

#include <hop/math/aa_box.h>
#include <hop/math/capsule.h>
#include <hop/math/convex_solid.h>
#include <hop/math/plane.h>
#include <hop/math/sphere.h>
#include <hop/math/vec3.h>

#include <type_traits>

namespace hop {

// Support functions: furthest point on a shape in a given direction
template <typename T> inline void support(vec3<T> & result, const aa_box<T> & box, const vec3<T> & d) {
	using tr = scalar_traits<T>;
	result.x = d.x > T {} ? box.maxs.x : (d.x < T {} ? box.mins.x : (box.mins.x + box.maxs.x) * tr::half());
	result.y = d.y > T {} ? box.maxs.y : (d.y < T {} ? box.mins.y : (box.mins.y + box.maxs.y) * tr::half());
	result.z = d.z > T {} ? box.maxs.z : (d.z < T {} ? box.mins.z : (box.mins.z + box.maxs.z) * tr::half());
}

template <typename T> inline void support(vec3<T> & result, const sphere<T> & sph, const vec3<T> & d) {
	result = d;
	normalize_carefully(result, T {});
	mul(result, sph.radius);
	add(result, sph.origin);
}

template <typename T> inline void support(vec3<T> & result, const capsule<T> & cap, const vec3<T> & d) {
	vec3<T> end;
	add(end, cap.origin, cap.direction);
	if (dot(end, d) > dot(cap.origin, d)) {
		result = end;
	} else {
		result = cap.origin;
	}
	vec3<T> nd = d;
	normalize_carefully(nd, T {});
	mul(nd, cap.radius);
	add(result, nd);
}

// Enumerate every plane triple-intersection that lies inside all other half-
// spaces. Shared by convex_solid support/get_bound and by rebuild_vertices.
// Callback receives each vertex by value.
template <typename T, typename Callback>
inline void for_each_convex_solid_vertex(const convex_solid<T> & cs, T epsilon, Callback && cb) {
	auto & planes = cs.planes;
	int sz = static_cast<int>(planes.size());
	for (int i = 0; i < sz - 2; ++i) {
		for (int j = i + 1; j < sz - 1; ++j) {
			for (int k = j + 1; k < sz; ++k) {
				const plane<T> & p1 = planes[i];
				const plane<T> & p2 = planes[j];
				const plane<T> & p3 = planes[k];

				// Solve the 3x3 system p_n · r = p_d via Cramer's rule.
				vec3<T> p2xp3;
				cross(p2xp3, p2.normal, p3.normal);
				T den = dot(p1.normal, p2xp3);
				if (den < epsilon && den > -epsilon)
					continue;
				vec3<T> p3xp1, p1xp2;
				cross(p3xp1, p3.normal, p1.normal);
				cross(p1xp2, p1.normal, p2.normal);
				mul(p1xp2, p3.distance);
				mul(p2xp3, p1.distance);
				mul(p3xp1, p2.distance);
				vec3<T> r;
				add(r, p1xp2, p2xp3);
				add(r, p3xp1);
				div(r, den);

				bool legal = true;
				for (int l = 0; l < sz; ++l) {
					if (l != i && l != j && l != k) {
						if ((dot(planes[l].normal, r) - planes[l].distance) > epsilon) {
							legal = false;
							break;
						}
					}
				}
				if (legal)
					cb(r);
			}
		}
	}
}

// Populate cs.vertices by enumerating the plane intersections. Normally called
// implicitly by ensure_vertices() on first support()/get_bound(); exposed for
// users who want to pay the O(n^4) enumeration up front (e.g. at load time).
template <typename T> inline void rebuild_vertices(convex_solid<T> & cs) {
	T epsilon;
	if constexpr (std::is_same_v<T, fixed16>)
		epsilon = fixed16::from_raw(1 << 4);
	else
		epsilon = T(0.0001);

	cs.vertices.clear();
	for_each_convex_solid_vertex(cs, epsilon, [&](const vec3<T> & v) { cs.vertices.push_back(v); });
}

// Lazy cache population. cs.vertices is mutable so this is callable from a
// const reference — first support()/get_bound() pays the enumeration, every
// subsequent call iterates the cached vector.
template <typename T> inline void ensure_vertices(const convex_solid<T> & cs) {
	if (!cs.vertices.empty() || cs.planes.empty())
		return;
	T epsilon;
	if constexpr (std::is_same_v<T, fixed16>)
		epsilon = fixed16::from_raw(1 << 4);
	else
		epsilon = T(0.0001);
	for_each_convex_solid_vertex(cs, epsilon, [&](const vec3<T> & v) { cs.vertices.push_back(v); });
}

template <typename T> inline void support(vec3<T> & result, const convex_solid<T> & cs, const vec3<T> & d) {
	ensure_vertices(cs);
	if (cs.vertices.empty()) {
		result.reset();
		return;
	}
	result = cs.vertices[0];
	T best_dot = dot(result, d);
	for (size_t i = 1; i < cs.vertices.size(); ++i) {
		T dp = dot(cs.vertices[i], d);
		if (dp > best_dot) {
			result = cs.vertices[i];
			best_dot = dp;
		}
	}
}

} // namespace hop
