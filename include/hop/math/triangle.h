#pragma once

#include <hop/math/project.h>
#include <hop/math/segment.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

// Triangle closest-point primitives. Pure geometry (no shape/solid types), so
// they live in math/ next to the segment projections and are reusable by any
// closest-point query — GJK's simplex step, the trimesh swept contact, etc.

namespace hop {

// Closest point to p on the triangle's three edges, as barycentric weights — the
// fallback for a degenerate (zero-area) triangle, which is just its longest edge.
template <typename T>
inline void closest_point_triangle_edges(const vec3<T> & p, const vec3<T> & a, const vec3<T> & b,
                                         const vec3<T> & c, T bary[3]) {
	using tr = scalar_traits<T>;
	const T zero {};
	const vec3<T> * v[3] = {&a, &b, &c};
	T best {};
	bool have = false;
	for (int i = 0; i < 3; ++i) {
		const int j = (i + 1) % 3;
		vec3<T> e, ep;
		sub(e, *v[j], *v[i]);
		sub(ep, p, *v[i]);
		const T len2 = length_squared(e);
		const T n = dot(e, ep);
		T t = zero; // a zero-length edge is just its start vertex
		if (len2 > zero)
			t = (n <= zero) ? zero : (n >= len2 ? tr::one() : n / len2);
		vec3<T> q;
		mul(q, e, t);
		add(q, *v[i]);
		vec3<T> diff;
		sub(diff, p, q);
		const T d2 = length_squared(diff);
		if (!have || d2 < best) {
			have = true;
			best = d2;
			bary[0] = bary[1] = bary[2] = zero;
			bary[i] = tr::one() - t;
			bary[j] = t;
		}
	}
}

// Closest point on triangle (a,b,c) to p, as barycentric weights bary[0..2]
// (sum to 1, so the point is bary[0]*a + bary[1]*b + bary[2]*c). Christer
// Ericson, Real-Time Collision Detection §5.1.5 (Voronoi-region test).
//
// Degenerate triangles (coincident or collinear vertices, common in authored
// meshes) are safe: each edge branch requires its denominator — that edge's
// squared length — to be positive, so a zero-length edge falls through to the
// next region instead of dividing 0/0 (NaN in float, a saturated garbage weight
// in fixed-point), and a zero-area face falls back to the closest edge.
template <typename T>
inline void closest_point_triangle(const vec3<T> & p, const vec3<T> & a, const vec3<T> & b,
                                   const vec3<T> & c, T bary[3]) {
	using tr = scalar_traits<T>;
	const T zero {};
	const T one = tr::one();

	vec3<T> ab, ac, ap;
	sub(ab, b, a);
	sub(ac, c, a);
	sub(ap, p, a);
	T d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= zero && d2 <= zero) { bary[0] = one; bary[1] = zero; bary[2] = zero; return; } // vertex A

	vec3<T> bp;
	sub(bp, p, b);
	T d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= zero && d4 <= d3) { bary[0] = zero; bary[1] = one; bary[2] = zero; return; } // vertex B

	T vc = d1 * d4 - d3 * d2;
	if (vc <= zero && d1 >= zero && d3 <= zero && d1 - d3 > zero) { // edge AB
		T v = d1 / (d1 - d3);
		bary[0] = one - v; bary[1] = v; bary[2] = zero;
		return;
	}

	vec3<T> cp;
	sub(cp, p, c);
	T d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= zero && d5 <= d6) { bary[0] = zero; bary[1] = zero; bary[2] = one; return; } // vertex C

	T vb = d5 * d2 - d1 * d6;
	if (vb <= zero && d2 >= zero && d6 <= zero && d2 - d6 > zero) { // edge AC
		T w = d2 / (d2 - d6);
		bary[0] = one - w; bary[1] = zero; bary[2] = w;
		return;
	}

	T va = d3 * d6 - d5 * d4;
	if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero && (d4 - d3) + (d5 - d6) > zero) { // edge BC
		T w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		bary[0] = zero; bary[1] = one - w; bary[2] = w;
		return;
	}

	const T area = va + vb + vc; // = |ab × ac|², zero for a degenerate triangle
	if (area <= zero) {
		closest_point_triangle_edges(p, a, b, c, bary);
		return;
	}
	T denom = one / area; // face interior
	T v = vb * denom, w = vc * denom;
	bary[0] = one - v - w; bary[1] = v; bary[2] = w;
}

// Closest point on triangle (a,b,c) to p — the point itself.
template <typename T>
inline void closest_point_triangle(vec3<T> & out, const vec3<T> & p, const vec3<T> & a,
                                   const vec3<T> & b, const vec3<T> & c) {
	T w[3];
	closest_point_triangle(p, a, b, c, w);
	out.x = w[0] * a.x + w[1] * b.x + w[2] * c.x;
	out.y = w[0] * a.y + w[1] * b.y + w[2] * c.y;
	out.z = w[0] * a.z + w[1] * b.z + w[2] * c.z;
}

// Squared closest distance between segment [p,q] and triangle (a,b,c). Fills the
// closest point on the segment (cs) and on the triangle (ct). Handles the
// segment piercing the triangle interior (distance 0) plus the boundary cases:
// each segment endpoint vs the triangle and the segment vs each triangle edge.
template <typename T>
inline T closest_segment_triangle(const vec3<T> & p, const vec3<T> & q, const vec3<T> & a,
                                  const vec3<T> & b, const vec3<T> & c,
                                  vec3<T> & cs, vec3<T> & ct, T epsilon) {
	using tr = scalar_traits<T>;
	const T zero {};

	vec3<T> pq;
	sub(pq, q, p);

	// Pierce test: if the segment crosses the triangle interior the distance is 0,
	// which the boundary tests below would miss (they only see the edges).
	vec3<T> ab, ac, fn;
	sub(ab, b, a);
	sub(ac, c, a);
	cross(fn, ab, ac);
	if (normalize_carefully(fn, fn, epsilon)) { // skip if the triangle is degenerate
		T denom = dot(fn, pq);
		if (denom > epsilon || denom < -epsilon) {
			vec3<T> ap;
			sub(ap, a, p);
			T tt = dot(fn, ap) / denom;
			if (tt >= zero && tt <= tr::one()) {
				vec3<T> x;
				mul(x, pq, tt);
				add(x, p);
				vec3<T> tx;
				closest_point_triangle(tx, x, a, b, c);
				vec3<T> diff;
				sub(diff, x, tx);
				if (length_squared(diff) <= tr::epsilon_squared(epsilon)) { cs = x; ct = x; return zero; } // epsilon² underflows in fixed16
			}
		}
	}

	T best {};
	bool have = false;
	vec3<T> tp, sc, tc;
	auto consider = [&](const vec3<T> & sp, const vec3<T> & on_tri) {
		vec3<T> d;
		sub(d, sp, on_tri);
		T d2 = length_squared(d);
		if (!have || d2 < best) { best = d2; cs = sp; ct = on_tri; have = true; }
	};

	closest_point_triangle(tp, p, a, b, c); consider(p, tp);
	closest_point_triangle(tp, q, a, b, c); consider(q, tp);

	segment<T> spine;
	spine.set_start_end(p, q);
	segment<T> edge;
	edge.set_start_end(a, b); project(sc, tc, spine, edge, epsilon); consider(sc, tc);
	edge.set_start_end(b, c); project(sc, tc, spine, edge, epsilon); consider(sc, tc);
	edge.set_start_end(c, a); project(sc, tc, spine, edge, epsilon); consider(sc, tc);
	return best;
}

} // namespace hop
