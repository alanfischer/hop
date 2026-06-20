#pragma once

#include <hop/math/triangle.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

// GJK closest-point distance + conservative-advancement sweep, used by the
// shape-vs-convex_solid collision pairs (wired up in collide.h).
//
// The plane-inflation path (build_convex_box_minkowski + trace_convex_solid)
// is exact on faces but reports a *face* normal at every contact, and inflating
// a convex's planes by a scalar radius produces a sharp corner that pokes past
// the true rounded Minkowski surface. Near an edge or vertex that yields the
// wrong normal (no ride-up over a ledge) and a too-early stop (phantom block),
// and an unbounded plane set has no closing face at all.
//
// GJK works from the *support function* (the vertex hull) instead, so the
// contact normal is the true direction between the closest features — correct
// at edges and vertices, and automatically bounded (an unbounded plane set
// still has a finite vertex hull).
//
// This header is pure geometry: it knows nothing about hop::shape. Callers pass
// two support callables `void(const vec3<T> & dir, vec3<T> & out)` returning the
// core (radius-excluded) support point of each shape, and a combined radius for
// the rounded margin. The shape-type dispatch that builds those callables lives
// in collide.h, alongside the rest of the shape-aware code.

namespace hop {

// Accumulate a scaled vector: r += a * c. (vec3.h's madd overwrites r rather
// than accumulating, so weighted sums need this.)
template <typename T>
inline void gjk_acc(vec3<T> & r, const vec3<T> & a, T c) {
	r.x += a.x * c;
	r.y += a.y * c;
	r.z += a.z * c;
}

// Largest power-of-two scale `s` that brings every simplex coordinate to |c| <= 4.
// The closest-feature barycentric weights are invariant under a uniform scaling
// about the origin (the query point maps to itself), so dividing the simplex
// points by `s` before the closest-point math leaves the result unchanged while
// keeping the degree-4 area products (and the degree-6 sign products in
// gjk_origin_inside_face) inside fixed16's ±32768 range. Without this a large
// polytope (e.g. a box with 40-unit half-extents) overflows those products, GJK
// reduces to the wrong simplex feature, and the sweep loses the contact entirely.
// A no-op (s == 1) whenever the simplex already fits — so float is unaffected.
template <typename T>
inline void gjk_fit_track(T & m, const vec3<T> & p) {
	using tr = scalar_traits<T>;
	m = tr::max_val(m, tr::abs(p.x));
	m = tr::max_val(m, tr::abs(p.y));
	m = tr::max_val(m, tr::abs(p.z));
}
template <typename T, typename... Ps>
inline T gjk_fit_scale(const Ps &... pts) {
	using tr = scalar_traits<T>;
	T m {};
	(gjk_fit_track(m, pts), ...);
	T s = tr::one();
	T bound = tr::four(); // = thresh * s, doubled in lockstep so the guard needs no multiply
	while (m > bound) {
		s = s + s;
		bound = bound + bound;
	}
	return s;
}

// Closest point to the origin on a triangle (a,b,c), as barycentric weights —
// the closest-to-origin specialization of math/triangle.h's closest_point_triangle.
// The _raw form assumes the points are already within range (the caller scaled
// them); gjk_closest_triangle scales first. gjk_closest_tetra works on points it
// has already scaled, so it calls _raw to avoid a redundant rescale.
template <typename T>
inline void gjk_closest_triangle_raw(const vec3<T> & a, const vec3<T> & b, const vec3<T> & c, T bary[3]) {
	vec3<T> origin;
	origin.reset();
	closest_point_triangle(origin, a, b, c, bary);
}
template <typename T>
inline void gjk_closest_triangle(const vec3<T> & a, const vec3<T> & b, const vec3<T> & c, T bary[3]) {
	T s = gjk_fit_scale<T>(a, b, c);
	vec3<T> as, bs, cs;
	div(as, a, s);
	div(bs, b, s);
	div(cs, c, s);
	gjk_closest_triangle_raw(as, bs, cs, bary);
}

// True if the origin lies on the inner side of plane (a,b,c) — the side that
// the opposite tetra vertex `d` is on. Strict: an origin exactly on the face
// plane counts as *outside*, so a face/edge touch is resolved as a zero-distance
// contact (with a valid normal) rather than misclassified as deep penetration.
template <typename T>
inline bool gjk_origin_inside_face(const vec3<T> & a, const vec3<T> & b, const vec3<T> & c, const vec3<T> & d) {
	vec3<T> ab, ac, n, ad;
	sub(ab, b, a);
	sub(ac, c, a);
	cross(n, ab, ac);
	sub(ad, d, a);
	T signp = -dot(n, a);   // n · (origin - a)
	T signd = dot(n, ad);   // n · (d - a)
	return signp * signd > T {};
}

// Closest point to the origin on a tetrahedron (a,b,c,d). Returns barycentric
// weights in bary[0..3]; sets `inside` when the origin is contained (distance
// zero — deep core penetration).
template <typename T>
inline void gjk_closest_tetra(const vec3<T> & a_in, const vec3<T> & b_in, const vec3<T> & c_in,
                              const vec3<T> & d_in, T bary[4], bool & inside) {
	using tr = scalar_traits<T>;
	const T zero {};

	// Scale the simplex to a safe magnitude before the closest-point math. The
	// face-side tests below (gjk_origin_inside_face) form degree-6 sign products
	// of the vertex coordinates, which overflow fixed16 for a large polytope; the
	// barycentric weights are scale-invariant about the origin, so this only keeps
	// the intermediates in range (see gjk_fit_scale). No-op for float / small simplices.
	T s = gjk_fit_scale<T>(a_in, b_in, c_in, d_in);
	vec3<T> a, b, c, d;
	div(a, a_in, s);
	div(b, b_in, s);
	div(c, c_in, s);
	div(d, d_in, s);

	inside = true;
	T best = tr::default_max_position_component();
	best = best * best;
	bary[0] = bary[1] = bary[2] = bary[3] = zero;

	// Map a triangle's 3 weights back into the tetra's 4-slot weight vector.
	auto consider = [&](const vec3<T> & p0, const vec3<T> & p1, const vec3<T> & p2,
	                    int i0, int i1, int i2) {
		T tb[3];
		gjk_closest_triangle_raw(p0, p1, p2, tb); // points already scaled by the tetra above
		vec3<T> q;
		mul(q, p0, tb[0]);
		gjk_acc(q, p1, tb[1]);
		gjk_acc(q, p2, tb[2]);
		T dsq = length_squared(q);
		if (dsq < best) {
			best = dsq;
			bary[0] = bary[1] = bary[2] = bary[3] = zero;
			bary[i0] = tb[0]; bary[i1] = tb[1]; bary[i2] = tb[2];
		}
	};

	// For each face, if the origin is outside it, the closest point lies on
	// that face (or its boundary). Winding chosen so the opposite vertex is the
	// "inside" reference.
	if (!gjk_origin_inside_face(a, b, c, d)) { inside = false; consider(a, b, c, 0, 1, 2); }
	if (!gjk_origin_inside_face(a, c, d, b)) { inside = false; consider(a, c, d, 0, 2, 3); }
	if (!gjk_origin_inside_face(a, d, b, c)) { inside = false; consider(a, d, b, 0, 3, 1); }
	if (!gjk_origin_inside_face(b, d, c, a)) { inside = false; consider(b, d, c, 1, 3, 2); }
}

// GJK core distance between two convex shapes (radius excluded). `supportA` /
// `supportB` return each shape's core support point in world space; `xA` is an
// extra translation applied to shape A (the conservative-advancement sweep
// offset). `seed_dir` warm-starts the search (pass the previous step's normal;
// a zero seed falls back to an arbitrary axis).
//
// On return `dist` is the distance between the cores and `normal_out` is the
// unit separating axis pointing outward from B toward A (= -v̂, where v is the
// closest point of the Minkowski difference B⊖A to the origin). `deep` is set
// only when the cores genuinely interpenetrate (the simplex encloses the
// origin) — a *touching* contact (closest point ≈ origin reached from a
// face/edge) is reported as dist≈0 with a valid normal, so zero-radius pairs
// still get a usable contact direction.
template <typename T, typename SupA, typename SupB>
inline void gjk_core_distance(SupA && supportA, SupB && supportB, const vec3<T> & xA,
                              const vec3<T> & seed_dir, T epsilon,
                              T & dist, vec3<T> & normal_out, bool & deep) {
	using tr = scalar_traits<T>;
	const T zero {};
	deep = false;
	normal_out.reset();
	dist = zero;

	// Minkowski-difference support: supportB(d) - (supportA(-d) + xA).
	auto cso = [&](const vec3<T> & d, vec3<T> & oy) {
		vec3<T> nd, a, b;
		neg(nd, d);
		supportA(nd, a);
		add(a, xA);
		supportB(d, b);
		sub(oy, b, a);
	};

	const T eps2 = epsilon * epsilon;

	vec3<T> y[4];
	vec3<T> dir = seed_dir;
	if (length_squared(dir) < eps2) {
		dir.reset();
		dir.x = tr::one();
	}
	cso(dir, y[0]);
	int n = 1;
	vec3<T> v = y[0];
	vec3<T> sep_axis = v; // last well-separated axis, for the touching case

	// Normalize the separating axis once, at exit, into the outward normal.
	auto finalize = [&]() {
		vec3<T> nv;
		if (normalize_carefully(nv, sep_axis, epsilon))
			neg(normal_out, nv); // outward from B toward A = -v̂
	};

	const int max_iter = 32;
	for (int iter = 0; iter < max_iter; ++iter) {
		T vlen2 = length_squared(v);
		if (vlen2 < eps2) {
			// Closest point is the origin — touching. sep_axis holds the last
			// separated axis (this iteration's v is degenerate), so don't fold it in.
			dist = zero;
			finalize();
			return;
		}
		sep_axis = v;

		// Search toward the origin.
		vec3<T> sdir, ny;
		neg(sdir, v);
		cso(sdir, ny);

		// Termination: the new support is no closer to the origin than v.
		T proj = vlen2 - dot(ny, v); // = dot(v, v - ny)
		T vlen = tr::sqrt(vlen2);    // single sqrt; reused for the guard and dist
		if (proj <= epsilon * vlen) {
			dist = vlen;
			finalize();
			return;
		}

		// Add support point, then reduce to the closest feature.
		y[n] = ny;
		++n;

		T bary[4];
		if (n == 2) {
			vec3<T> ab;
			sub(ab, y[1], y[0]);
			T denom = length_squared(ab);
			T t = (denom > zero) ? (-dot(y[0], ab) / denom) : zero;
			t = tr::clamp(zero, tr::one(), t);
			bary[0] = tr::one() - t; bary[1] = t;
		} else if (n == 3) {
			gjk_closest_triangle(y[0], y[1], y[2], bary);
		} else { // n == 4
			bool inside = false;
			gjk_closest_tetra(y[0], y[1], y[2], y[3], bary, inside);
			if (inside) {
				deep = true; // genuine interpenetration — caller falls back
				dist = zero;
				return;
			}
		}

		// Compact in place to the supporting feature (write index m <= read i),
		// then recompute v.
		int m = 0;
		for (int i = 0; i < n; ++i) {
			if (bary[i] > epsilon) {
				y[m] = y[i];
				bary[m] = bary[i];
				++m;
			}
		}
		T wsum = zero;
		for (int i = 0; i < m; ++i)
			wsum += bary[i];
		if (wsum <= zero) {
			dist = zero;
			finalize();
			return;
		}
		vec3<T> nv;
		nv.reset();
		for (int i = 0; i < m; ++i)
			gjk_acc(nv, y[i], bary[i] / wsum);
		n = m;
		v = nv;
	}

	// Iteration budget exhausted: report the best separation found.
	sep_axis = v;
	dist = length(v);
	finalize();
}

// Result of a conservative-advancement sweep.
template <typename T>
struct gjk_sweep_result {
	bool valid;     // false => deep core penetration; caller falls back
	bool hit;
	T time;         // TOI in [0,1]
	T depth;        // penetration depth (only meaningful at time == 0)
	vec3<T> normal; // outward from B toward A (the mover), world space
};

// Conservative advancement of a rounded shape sweeping by `motion` against a
// static target, given a `closest` callable that reports the core (radius-
// excluded) separation at a candidate offset:
//
//   closest(const vec3<T> & xA, const vec3<T> & seed, T & dist, vec3<T> & n, bool & deep)
//     xA   — the sweep offset to apply to the moving shape this step
//     seed — a warm-start search direction (the previous step's normal)
//     dist — core distance between the shapes at xA (>= 0)
//     n    — unit contact normal, outward from the target toward the mover
//     deep — set when the cores genuinely interpenetrate (no usable normal)
//
// This owns the swept-contact *semantics* shared by every narrowphase pair:
// penetration reporting, the "block only on an approaching contact" rule, the
// tolerance, and the half-tolerance undershoot. The closest-point *method* is
// the caller's choice — GJK simplex for general convex (gjk_sweep), or an
// analytic segment-vs-triangle test for a trimesh. combined_radius is the sum of
// the rounded margins (sphere/capsule radii) plus any query margin.
template <typename T, typename ClosestFn>
inline void conservative_advance(gjk_sweep_result<T> & res, const vec3<T> & motion,
                                 T combined_radius, T epsilon, ClosestFn && closest) {
	using tr = scalar_traits<T>;
	const T zero {};
	const T one = tr::one();
	res.valid = true;
	res.hit = false;
	res.time = one;
	res.depth = zero;

	const T tol = tr::max_val(epsilon, tr::from_milli(1));
	const T eps2 = epsilon * epsilon;
	// "Tangential" is measured relative to the motion length, not an absolute
	// epsilon: a body settling under gravity has a tiny-but-real approach (~the
	// per-tick gravity sliver) that an absolute cutoff would wrongly discard,
	// leaving it unsupported. The ratio (≈cos of the motion-vs-normal angle) is
	// scale-free — only a motion almost parallel to the surface counts as tangent.
	const T motion_len = length(motion);
	const T tangential_cut = motion_len * tr::from_milli(1);
	// Warm-start with an arbitrary axis; subsequent steps reuse the previous
	// step's normal, so an iterative closest-point method starts near the answer.
	vec3<T> seed;
	seed.reset();
	seed.x = one;

	T t = zero;
	const int max_ca = 32;
	for (int iter = 0; iter < max_ca; ++iter) {
		vec3<T> xA;
		mul(xA, motion, t); // mover's sweep offset at this TOI
		T dist;
		vec3<T> n; // outward from the target toward the mover (unit)
		bool deep = false;
		closest(xA, seed, dist, n, deep);
		if (deep) {
			res.valid = false; // deep core overlap — let the caller's fallback recover
			return;
		}
		if (length_squared(n) < eps2) {
			res.valid = false; // no separating axis recovered — fall back
			return;
		}
		seed = n; // warm start the next CA step

		T sep = dist - combined_radius; // gap between the rounded surfaces

		// Genuine penetration (the rounded surfaces already overlap): report at the
		// current TOI with the overlap depth so callers can depenetrate, regardless
		// of motion direction. Advancement never steps into overlap, so this only
		// fires at t == 0 (a body that starts the tick embedded).
		if (sep < -tol) {
			res.hit = true;
			res.time = t;
			res.normal = n;
			res.depth = -sep;
			return;
		}

		// Only a contact the motion moves *into* can block a swept query. A
		// tangential or separating contact must NOT stop the sweep, or a body
		// resting on a surface sticks at t == 0 and can't slide, walk, or step off
		// the edge. (This matches the plane-inflation path, where a tangential ray
		// never enters the inflated convex.) Resting/ground contact is reported
		// separately by the caller's dedicated down-probe, not by this motion cast.
		T approach = -dot(motion, n); // gap closes at rate `approach` (n points target->mover)
		if (approach <= tangential_cut) {
			res.hit = false;
			res.time = one;
			return;
		}

		// Approaching: a contact within tolerance stops the sweep here.
		if (sep <= tol) {
			res.hit = true;
			res.time = t;
			res.normal = n;
			res.depth = tr::max_val(zero, -sep);
			return;
		}
		// Undershoot by half the tolerance so the next evaluation lands while the
		// shapes are still cleanly separated — at exact contact the closest feature
		// collapses to an edge/vertex and the recovered normal tilts. Stopping a
		// hair short keeps it on the contacting face.
		T dt = (sep - tol * tr::half()) / approach;
		if (dt < zero)
			dt = zero;
		t += dt;
		if (t >= one) {
			res.hit = false;
			res.time = one;
			return;
		}
	}
	// Did not converge within the iteration budget — be conservative, no hit.
	res.hit = false;
	res.time = one;
}

// Sweep shape A (support `supportA`, at its start placement, moving by `motion`)
// against static shape B (support `supportB`), using GJK for the closest-point
// step. combined_radius is the sum of the rounded margins plus any query margin.
template <typename T, typename SupA, typename SupB>
inline void gjk_sweep(gjk_sweep_result<T> & res, SupA && supportA, SupB && supportB,
                      const vec3<T> & motion, T combined_radius, T epsilon) {
	conservative_advance(res, motion, combined_radius, epsilon,
	    [&](const vec3<T> & xA, const vec3<T> & seed, T & dist, vec3<T> & n, bool & deep) {
		    gjk_core_distance<T>(supportA, supportB, xA, seed, epsilon, dist, n, deep);
	    });
}

} // namespace hop
