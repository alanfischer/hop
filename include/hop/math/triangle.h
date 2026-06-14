#pragma once

#include <hop/math/project.h>
#include <hop/math/segment.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

// Triangle closest-point primitives. Pure geometry (no shape/solid types), so
// they live in math/ next to the segment projections and are reusable by any
// closest-point query — GJK's simplex step, the trimesh swept contact, etc.

namespace hop {

// Closest point on triangle (a,b,c) to p, as barycentric weights bary[0..2]
// (sum to 1, so the point is bary[0]*a + bary[1]*b + bary[2]*c). Christer
// Ericson, Real-Time Collision Detection §5.1.5 (Voronoi-region test).
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
	if (vc <= zero && d1 >= zero && d3 <= zero) { // edge AB
		T v = d1 / (d1 - d3);
		bary[0] = one - v; bary[1] = v; bary[2] = zero;
		return;
	}

	vec3<T> cp;
	sub(cp, p, c);
	T d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= zero && d5 <= d6) { bary[0] = zero; bary[1] = zero; bary[2] = one; return; } // vertex C

	T vb = d5 * d2 - d1 * d6;
	if (vb <= zero && d2 >= zero && d6 <= zero) { // edge AC
		T w = d2 / (d2 - d6);
		bary[0] = one - w; bary[1] = zero; bary[2] = w;
		return;
	}

	T va = d3 * d6 - d5 * d4;
	if (va <= zero && (d4 - d3) >= zero && (d5 - d6) >= zero) { // edge BC
		T w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		bary[0] = zero; bary[1] = one - w; bary[2] = w;
		return;
	}

	T denom = one / (va + vb + vc); // face interior
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
				if (length_squared(diff) <= epsilon * epsilon) { cs = x; ct = x; return zero; }
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
