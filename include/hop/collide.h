#pragma once

#include <hop/collision.h>
#include <hop/math/bounding.h>
#include <hop/math/gjk.h>
#include <hop/math/intersect.h>
#include <hop/math/project.h>
#include <hop/math/support.h>
#include <hop/scalar_traits.h>
#include <hop/shape.h>
#include <hop/solid.h>

// Reusable swept-collision routines. Pure functions of geometry + epsilon —
// they hold no state, so they can be called outside the simulator (e.g. from
// editor tooling, query layers, or alternate broad phases). The simulator
// composes these with its spatial collection and manager hooks.
//
// Two layers live here:
//   - Shape-vs-segment primitives: trace_aa_box / sphere / capsule / convex.
//   - Solid-pair dispatchers: test_segment / test_solid pick the right
//     primitive for each shape pair and fold per-shape hits into `result`.
//
// merge_collision is the simulator-level fold used between solids (it
// respects an average_normals flag, unlike the unconditional intra-solid
// averaging inside test_segment/test_solid).

namespace hop {

template <typename T>
void trace_aa_box(collision<T> & c, const segment<T> & seg, const aa_box<T> & box) {
	if (test_inside(box, seg.origin)) {
		T dix = scalar_traits<T>::abs(seg.origin.x - box.mins.x);
		T diy = scalar_traits<T>::abs(seg.origin.y - box.mins.y);
		T diz = scalar_traits<T>::abs(seg.origin.z - box.mins.z);
		T dax = scalar_traits<T>::abs(seg.origin.x - box.maxs.x);
		T day = scalar_traits<T>::abs(seg.origin.y - box.maxs.y);
		T daz = scalar_traits<T>::abs(seg.origin.z - box.maxs.z);

		auto neg_x = constants<T>::neg_x_unit_vec3();
		auto neg_y = constants<T>::neg_y_unit_vec3();
		auto neg_z = constants<T>::neg_z_unit_vec3();
		auto pos_x = constants<T>::x_unit_vec3();
		auto pos_y = constants<T>::y_unit_vec3();
		auto pos_z = constants<T>::z_unit_vec3();

		vec3<T> face_normal;
		T depth = diz;
		face_normal.set(neg_z);
		if (daz < depth) { depth = daz; face_normal.set(pos_z); }
		if (dix < depth) { depth = dix; face_normal.set(neg_x); }
		if (dax < depth) { depth = dax; face_normal.set(pos_x); }
		if (diy < depth) { depth = diy; face_normal.set(neg_y); }
		if (day < depth) { depth = day; face_normal.set(pos_y); }

		if (length_squared(seg.direction) > T {} && dot(seg.direction, face_normal) >= T {})
			return;

		c.time = T {};
		c.depth = depth;
		c.normal.set(face_normal);
		c.point.set(seg.origin);
	} else {
		c.time = find_intersection(seg, box, c.point, c.normal);
	}
}

template <typename T>
void trace_sphere(collision<T> & c, const segment<T> & seg, const sphere<T> & sph, T epsilon) {
	using tr = scalar_traits<T>;
	const T one = tr::one();
	if (test_inside(sph, seg.origin)) {
		vec3<T> n;
		n.set(seg.origin);
		sub(n, sph.origin);
		T dist = length(n);
		if (!normalize_carefully(n, epsilon)) {
			// Origin exactly at sphere center — use negative direction as normal
			normalize(n, seg.direction);
			neg(n);
		}
		if (dot(n, seg.direction) <= epsilon) {
			c.time = T {};
			c.depth = sph.radius - dist;
			c.point.set(seg.origin);
			c.normal.set(n);
		} else {
			c.time = one;
		}
	} else {
		c.time = find_intersection(seg, sph, c.point, c.normal);
	}
}

template <typename T>
void trace_capsule(collision<T> & c, const segment<T> & seg, const capsule<T> & cap, T epsilon) {
	vec3<T> p1, p2;
	segment<T> s;
	s.origin.set(cap.origin);
	s.direction.set(cap.direction);
	project(p1, p2, s, seg, epsilon);
	sphere<T> sph;
	sph.set(p1, cap.radius);
	trace_sphere(c, seg, sph, epsilon);
}

template <typename T>
void trace_capsule_capsule(collision<T> & c,
                           const segment<T> & seg,
                           const vec3<T> & base,
                           const vec3<T> & D1,
                           const vec3<T> & D2,
                           T radius,
                           T epsilon) {
	using tr = scalar_traits<T>;
	const T one = tr::one();
	T zero_val {};

	// The Minkowski sum of two capsule spines forms a parallelogram with vertices:
	//   V0 = base,  V1 = base + D2,  V2 = base + D2 - D1,  V3 = base - D1
	// inflated by the combined radius.
	//
	// We trace the 4 edges as capsules, plus an analytical interior-interior solve
	// for when both closest-point parameters are in (0,1).

	c.time = one;

	capsule<T> edge_cap;
	collision<T> edge_col;
	auto trace_edge = [&](const vec3<T> & origin, const vec3<T> & dir) {
		edge_cap.set(origin, dir, radius);
		edge_col.reset();
		trace_capsule(edge_col, seg, edge_cap, epsilon);
		if (edge_col.time < c.time) {
			c.time = edge_col.time;
			c.point.set(edge_col.point);
			c.normal.set(edge_col.normal);
		}
	};

	vec3<T> v3;
	sub(v3, base, D1);
	vec3<T> v1;
	add(v1, base, D2);
	vec3<T> neg_D1;
	neg(neg_D1, D1);

	// V0->V1 = capsule(base, D2, R) — sh1-start vs sh2-spine
	trace_edge(base, D2);
	// V3->V2 = capsule(base - D1, D2, R) — sh1-end vs sh2-spine
	trace_edge(v3, D2);
	// V0->V3 = capsule(base, -D1, R) — sh2-start vs sh1-spine
	trace_edge(base, neg_D1);
	// V1->V2 = capsule(base + D2, -D1, R) — sh2-end vs sh1-spine
	trace_edge(v1, neg_D1);

	// Interior-interior analytical solve:
	// When both closest-point parameters s,u are in (0,1), the closest-point
	// vector between the two spines is:
	//   diff(t) = G(t) + s(t)*D1 - u(t)*D2
	// where G(t) = seg.origin + t*V - base (moving point minus base), and
	// s(t), u(t) are linear in t from the 2x2 Cramer system.
	// We solve |diff(t)|² = R² as a quadratic in t.

	T a11 = dot(D1, D1); // |D1|²
	T a12 = dot(D1, D2); // D1·D2
	T a22 = dot(D2, D2); // |D2|²
	T det = a11 * a22 - a12 * a12;

	// Skip if spines are (nearly) parallel — edges already cover this case
	if (tr::abs(det) > epsilon) {
		T inv_det = one / det;
		vec3<T> V;
		V.set(seg.direction); // motion direction

		// G0 = seg.origin - base (initial vector from base to moving point)
		vec3<T> G0;
		sub(G0, seg.origin, base);

		// The closest-point system gives:
		//   s*a11 - u*a12 = -(D1·G)   =>  g1 - t*v1
		//   s*a12 - u*a22 = -(D2·G)   =>  g2 - t*v2
		// where g1 = -D1·G0 = D1·(base-seg.origin), v1 = D1·V, etc.
		T v1 = dot(D1, V);
		T v2 = dot(D2, V);
		T g1 = -dot(D1, G0);
		T g2 = -dot(D2, G0);

		// Cramer's rule for s0, u0 (at t=0):
		T s0 = (a22 * g1 - a12 * g2) * inv_det;
		T u0 = (a12 * g1 - a11 * g2) * inv_det;
		// Rates ds/dt, du/dt:
		T ds = (-a22 * v1 + a12 * v2) * inv_det;
		T du = (-a12 * v1 + a11 * v2) * inv_det;

		// P0 = diff(0) = G0 + s0*D1 - u0*D2
		vec3<T> P0;
		P0.set(G0);
		vec3<T> tmp;
		mul(tmp, D1, s0);
		add(P0, tmp);
		mul(tmp, D2, u0);
		sub(P0, tmp);

		// dP = d(diff)/dt = V + ds*D1 - du*D2
		vec3<T> dP;
		dP.set(V);
		mul(tmp, D1, ds);
		add(dP, tmp);
		mul(tmp, D2, du);
		sub(dP, tmp);

		// Quadratic: |P0 + t*dP|² = R²
		// A*t² + B*t + C = 0
		T A = dot(dP, dP);
		T B = tr::two() * dot(P0, dP);
		T C = dot(P0, P0) - radius * radius;

		T disc = B * B - tr::four() * A * C;
		if (disc >= zero_val && A > epsilon) {
			T sqrt_disc = tr::sqrt(disc);
			T t_hit = (-B - sqrt_disc) / (tr::two() * A);

			if (t_hit >= zero_val && t_hit < c.time) {
				// Validate s(t) and u(t) are in [0,1]
				T s_t = s0 + ds * t_hit;
				T u_t = u0 + du * t_hit;

				if (s_t > zero_val && s_t < one && u_t > zero_val && u_t < one) {
					// Normal = normalized diff(t_hit)
					vec3<T> diff;
					mul(diff, dP, t_hit);
					add(diff, P0);

					vec3<T> norm;
					norm.set(diff);
					if (normalize_carefully(norm, epsilon)) {
						c.time = t_hit;
						mul(c.point, seg.direction, t_hit);
						add(c.point, seg.origin);
						c.normal.set(norm);
					}
				}
			}
		}
	}
}

template <typename T>
void trace_convex_solid(collision<T> & c, const segment<T> & seg, const convex_solid<T> & cs, T epsilon) {
	using tr = scalar_traits<T>;
	const T one = tr::one();
	T zero_val {};
	T half_epsilon = epsilon * tr::half();
	c.time = one;

	// Check if segment origin is inside the convex solid (all planes)
	bool inside = true;
	T closest_dist = -tr::default_max_position_component();
	int closest_plane = -1;
	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T d = dot(cs.planes[i].normal, seg.origin) - cs.planes[i].distance;
		if (d > -half_epsilon) {
			inside = false;
			break;
		}
		if (d > closest_dist) {
			closest_dist = d;
			closest_plane = i;
		}
	}
	if (inside && closest_plane >= 0) {
		c.time = zero_val;
		c.depth = -closest_dist;
		c.point.set(seg.origin);
		c.normal.set(cs.planes[closest_plane].normal);
		return;
	}

	for (int i = 0; i < static_cast<int>(cs.planes.size()); ++i) {
		T denom = dot(cs.planes[i].normal, seg.direction);
		if (denom >= zero_val)
			continue; // Only accept entry planes (segment moving into plane)
		T t = (cs.planes[i].distance - dot(cs.planes[i].normal, seg.origin)) / denom;
		if (t >= zero_val && t <= one) {
			vec3<T> u;
			mul(u, seg.direction, t);
			add(u, seg.origin);

			bool b = true;
			for (int j = 0; j < static_cast<int>(cs.planes.size()); ++j) {
				if (i == j)
					continue;
				if (dot(cs.planes[j].normal, u) - cs.planes[j].distance > zero_val) {
					b = false;
					break;
				}
			}
			if (b && t < c.time) {
				c.time = t;
				c.point.set(u);
				c.normal.set(cs.planes[i].normal);
			}
		}
	}
}

// Build the exact Minkowski sum convex ⊕ (box centered at origin with
// half-extents `box_half`) into `out`. The sum has two families of faces:
// each of the convex's N faces, inflated by the box's support along that
// normal (= |n.x|*h.x + |n.y|*h.y + |n.z|*h.z); and the 6 axis-aligned box
// faces, each at distance `sup(convex, axis) + box_half[axis]`. Omitting
// the second family leaves the polytope unbounded along axis directions,
// producing false-positive static overlaps that freeze the swept trace.
template <typename T>
void build_convex_box_minkowski(convex_solid<T> & out,
                                const convex_solid<T> & convex,
                                const vec3<T> & box_half) {
	const aa_box<T> box(-box_half.x, -box_half.y, -box_half.z, box_half.x, box_half.y, box_half.z);
	out.planes = convex.planes;
	out.planes.reserve(convex.planes.size() + 6);
	vec3<T> sup;
	for (auto & p : out.planes) {
		support(sup, box, p.normal);
		p.distance = p.distance + dot(sup, p.normal);
	}
	const vec3<T> axes[6] = {
		constants<T>::x_unit_vec3(),     constants<T>::neg_x_unit_vec3(),
		constants<T>::y_unit_vec3(),     constants<T>::neg_y_unit_vec3(),
		constants<T>::z_unit_vec3(),     constants<T>::neg_z_unit_vec3(),
	};
	const T half_along[6] = { box_half.x, box_half.x, box_half.y, box_half.y, box_half.z, box_half.z };
	for (int i = 0; i < 6; ++i) {
		support(sup, convex, axes[i]);
		out.planes.push_back(plane<T>(axes[i], dot(sup, axes[i]) + half_along[i]));
	}
}

// Dispatch helpers for test_solid's convex-vs-* branches. `inflated_cs` is
// the target-side convex solid with its planes already inflated by the
// Minkowski-sum amount for the opposing shape (scalar radius, or per-plane
// `support(shape, p.normal)` for convex-vs-convex). Both helpers normalize
// col.point to s1's center at impact.
//
// Forward: sh1 is primitive, sh2 is convex. sh1_offset is sh1's reference
// point in s1's local frame (already includes lp1).
template <typename T>
void trace_forward_convex(collision<T> & col,
                          const segment<T> & seg,
                          const vec3<T> & s2_position,
                          const vec3<T> & lp2,
                          const convex_solid<T> & inflated_cs,
                          const vec3<T> & sh1_offset,
                          T epsilon) {
	using tr = scalar_traits<T>;
	segment<T> tmp;
	tmp.set(seg);
	sub(tmp.origin, s2_position);
	sub(tmp.origin, lp2);
	add(tmp.origin, sh1_offset);
	trace_convex_solid(col, tmp, inflated_cs, epsilon);
	if (col.time < tr::one()) {
		vec3<T> travel;
		mul(travel, seg.direction, col.time);
		add(col.point, seg.origin, travel);
	}
}

// Inverted: sh1 is convex, sh2 is primitive. Traces sh2's reference point
// backwards against sh1's convex. sh2_offset is sh2's reference point in
// s2's local frame (does NOT include lp_delta — helper adds it).
template <typename T>
void trace_inverted_convex(collision<T> & col,
                           const segment<T> & seg,
                           const vec3<T> & s1_position,
                           const vec3<T> & s2_position,
                           const vec3<T> & lp_delta,
                           const convex_solid<T> & inflated_cs,
                           const vec3<T> & sh2_offset,
                           T epsilon) {
	using tr = scalar_traits<T>;
	segment<T> tmp;
	mul(tmp.direction, seg.direction, -tr::one());
	tmp.origin.set(s2_position);
	sub(tmp.origin, s1_position);
	add(tmp.origin, lp_delta);
	add(tmp.origin, sh2_offset);
	collision<T> icol;
	icol.time = tr::one();
	trace_convex_solid(icol, tmp, inflated_cs, epsilon);
	if (icol.time < tr::one()) {
		col.time = icol.time;
		col.depth = icol.depth;
		col.normal.set(icol.normal);
		neg(col.normal);
		vec3<T> travel;
		mul(travel, seg.direction, icol.time);
		add(col.point, seg.origin, travel);
	}
}

// Core (radius-excluded) support point of a primitive shape in its OWN local
// frame. Sphere/capsule contribute only their skeleton (point/segment); their
// radius is handled by gjk_sweep as a combined margin. Shape-type dispatch that
// keeps math/gjk.h shape-agnostic.
template <typename T>
inline void gjk_local_support(vec3<T> & out, const shape<T> * sh, const vec3<T> & dir) {
	switch (sh->get_type()) {
		case shape_type::box:
			support(out, sh->get_box(), dir);
			break;
		case shape_type::sphere:
			out = sh->get_sphere().origin;
			break;
		case shape_type::capsule: {
			const auto & c = sh->get_capsule();
			vec3<T> end;
			add(end, c.origin, c.direction);
			out = (dot(end, dir) > dot(c.origin, dir)) ? end : c.origin;
			break;
		}
		case shape_type::convex_solid:
			support(out, sh->get_convex_solid(), dir);
			break;
		default:
			out.reset();
			break;
	}
}

// World support, axis-aligned (no rotation): base = solid_position + local_position.
template <typename T>
inline void gjk_core_support(vec3<T> & out, const shape<T> * sh, const vec3<T> & base, const vec3<T> & dir) {
	gjk_local_support(out, sh, dir);
	add(out, base);
}

// World support, oriented by R (with Rt = Rᵀ precomputed): the shape is rotated
// about its local origin then placed at base, so support_world(d) =
// R·support_local(Rᵀ·d) + base. Used when a solid and/or shape carries a static
// rotation; the result is in world space, so GJK runs unchanged and its normal
// comes out world-space too (no post-rotation needed).
template <typename T>
inline void gjk_core_support(vec3<T> & out, const shape<T> * sh, const mat3<T> & R, const mat3<T> & Rt,
                             const vec3<T> & base, const vec3<T> & dir) {
	vec3<T> ld, ls;
	mul(ld, Rt, dir);
	gjk_local_support(ls, sh, ld);
	mul(out, R, ls);
	add(out, base);
}

template <typename T>
inline T gjk_core_radius(const shape<T> * sh) {
	switch (sh->get_type()) {
		case shape_type::sphere:
			return sh->get_sphere().radius;
		case shape_type::capsule:
			return sh->get_capsule().radius;
		default:
			return T {};
	}
}

inline bool is_rounded_shape(shape_type t) { return t == shape_type::sphere || t == shape_type::capsule; }
inline bool is_polytope_shape(shape_type t) { return t == shape_type::box || t == shape_type::convex_solid; }

// The pairs the GJK path owns: exactly one side rounded (sphere/capsule), the
// other a polytope (box/convex). Rounded×rounded keeps its exact analytic
// solver (trace_sphere/capsule); polytope×polytope keeps the cheap inflation
// path (zero combined radius would make GJK resolve contact at dist≈0, which
// loses precision in fixed-point). The non-zero rounded radius is what keeps GJK
// well-conditioned, so it is the defining property of eligibility — not a paste
// pattern across branches.
inline bool gjk_eligible_pair(shape_type a, shape_type b) {
	return (is_rounded_shape(a) && is_polytope_shape(b)) || (is_polytope_shape(a) && is_rounded_shape(b));
}

// GJK closest-point sweep for a rounded-shape (sphere / capsule) vs polytope
// (convex_solid or box) pair. Replaces the plane-inflation / AABB-expansion path
// with a vertex-based conservative advancement: correct edge/vertex contact
// normals (so a capsule rides up a thin ledge instead of hitting a fabricated
// vertical wall) and bounded (an ill-formed or unbounded plane set can't
// phantom-block, since the support function uses the finite vertex hull). A=s1
// sweeps by seg.direction; B=s2 is static. Fills `col` (col.point is s1's
// reference position at impact, matching the rest of test_solid) and returns
// true. Returns false on deep core penetration, so the caller falls back to the
// robust inflate/AABB path for recovery.
//
// test_solid dispatches here once, for the pairs gjk_eligible_pair() selects
// (one rounded shape, one polytope). On a false return the caller's per-pair
// fallback chain runs the cheap plane-inflation / AABB path; the `use_gjk` flag
// on test_solid forces that path globally.
// Analytic core closest point between a rounded shape (sphere center / capsule
// spine) and an oriented box, in the form conservative_advance's closest functor
// wants: `dist` is the core separation, `n` the unit axis outward from the box
// toward the rounded shape, `deep` set when the rounded core reaches inside the box.
//
// These are the fixed-point-robust replacement for GJK on the rounded×box pairs.
// GJK reconstructs the (small) closest vector as v = Σ wᵢ·yᵢ, a weighted average of
// the box's far vertices; for a large box that is a small difference of large terms,
// and fixed16 loses it to cancellation — the sweep then misses the resting contact
// and the shape tunnels (see math/gjk.h). Clamping/projecting onto the box and
// subtracting computes that small vector directly, exact at any box size.

// Transform a world point into the box's local frame.
template <typename T>
inline void box_to_local(vec3<T> & out, const vec3<T> & p, const vec3<T> & box_base,
                         const mat3<T> & RboxT, bool identity) {
	if (identity) {
		sub(out, p, box_base);
	} else {
		vec3<T> d;
		sub(d, p, box_base);
		mul(out, RboxT, d);
	}
}

// Clamp a box-local point into the box; returns true when it was already inside
// (no axis clamped) — genuine core penetration, with no usable separating axis.
template <typename T>
inline bool box_clamp_local(const aa_box<T> & box, const vec3<T> & pl, vec3<T> & cl) {
	using tr = scalar_traits<T>;
	cl.x = tr::clamp(box.mins.x, box.maxs.x, pl.x);
	cl.y = tr::clamp(box.mins.y, box.maxs.y, pl.y);
	cl.z = tr::clamp(box.mins.z, box.maxs.z, pl.z);
	return cl.x == pl.x && cl.y == pl.y && cl.z == pl.z;
}

// Turn a box-local closest pair (pl on the rounded core, cl on the box) into the
// distance and the world-space outward (box→core) unit normal.
template <typename T>
inline void box_local_result(const vec3<T> & pl, const vec3<T> & cl, const mat3<T> & Rbox,
                             bool identity, T & dist, vec3<T> & n, T epsilon) {
	vec3<T> diff_l;
	sub(diff_l, pl, cl); // box surface -> rounded core, box-local
	vec3<T> diff;
	if (identity)
		diff.set(diff_l);
	else
		mul(diff, Rbox, diff_l);
	dist = length(diff);
	n.set(diff);
	normalize_carefully(n, epsilon);
}

// Sphere case: closest point on the box to the sphere center.
template <typename T>
inline void box_point_core_closest(const vec3<T> & center, const vec3<T> & box_base,
                                   const mat3<T> & Rbox, const mat3<T> & RboxT,
                                   const aa_box<T> & box, bool identity,
                                   T & dist, vec3<T> & n, bool & deep, T epsilon) {
	vec3<T> pl;
	box_to_local(pl, center, box_base, RboxT, identity);
	vec3<T> cl;
	deep = box_clamp_local(box, pl, cl);
	if (deep) {
		dist = T {};
		n.reset();
		return;
	}
	box_local_result(pl, cl, Rbox, identity, dist, n, epsilon);
}

// Capsule case: closest point between the spine segment (p0→p1) and the box, by
// alternating projection — clamp the current spine point onto the box, project that
// back onto the spine, repeat. Both sets are convex so this converges to the closest
// pair; the box's axis-aligned faces make it fast (a few steps). A degenerate spine
// (p0 == p1) collapses to the sphere case in one step.
template <typename T>
inline void box_segment_core_closest(const vec3<T> & p0, const vec3<T> & p1, const vec3<T> & box_base,
                                     const mat3<T> & Rbox, const mat3<T> & RboxT,
                                     const aa_box<T> & box, bool identity,
                                     T & dist, vec3<T> & n, bool & deep, T epsilon) {
	using tr = scalar_traits<T>;
	vec3<T> l0, l1;
	box_to_local(l0, p0, box_base, RboxT, identity);
	box_to_local(l1, p1, box_base, RboxT, identity);
	vec3<T> d;
	sub(d, l1, l0); // spine direction, box-local
	T dd = length_squared(d);
	vec3<T> pl(l0), cl;
	if (dd > T {}) {
		const int max_iter = 8;
		for (int i = 0; i < max_iter; ++i) {
			box_clamp_local(box, pl, cl); // closest box point to the current spine point
			vec3<T> r;
			sub(r, cl, l0);
			T t = tr::clamp(T {}, tr::one(), dot(r, d) / dd); // project it back onto the spine
			vec3<T> next;
			mul(next, d, t);
			add(next, l0);
			vec3<T> moved;
			sub(moved, next, pl);
			pl.set(next);
			if (length_squared(moved) <= tr::epsilon_squared(epsilon))
				break; // converged
		}
	}
	deep = box_clamp_local(box, pl, cl);
	if (deep) {
		dist = T {};
		n.reset();
		return;
	}
	box_local_result(pl, cl, Rbox, identity, dist, n, epsilon);
}

template <typename T>
inline bool trace_pair_gjk(collision<T> & col, const segment<T> & seg,
                                  solid<T> * s1, solid<T> * s2, shape<T> * sh1, shape<T> * sh2,
                                  const vec3<T> & lp1, const vec3<T> & lp2,
                                  T margin, T epsilon) {
	using tr = scalar_traits<T>;
	// Each shape's world rotation = solid_orientation · shape_local_rotation; its
	// world origin = solid_position + solid_orientation · local_position. (For
	// identity orientations this is exactly the old translate-only placement.)
	mat3<T> Ra, Rb;
	mul(Ra, s1->get_orientation(), sh1->get_local_rotation());
	mul(Rb, s2->get_orientation(), sh2->get_local_rotation());
	vec3<T> base_a, base_b, off;
	mul(off, s1->get_orientation(), lp1);
	add(base_a, seg.origin, off);
	mul(off, s2->get_orientation(), lp2);
	add(base_b, s2->get_position(), off);

	T combined_radius = gjk_core_radius(sh1) + gjk_core_radius(sh2) + margin;
	gjk_sweep_result<T> res;
	const mat3<T> identity;

	// Rounded (sphere/capsule) × box: use the analytic closest point instead of GJK.
	// GJK's closest-point reconstruction loses the (small) contact vector to
	// fixed-point cancellation on a large box (see box_point_core_closest); the
	// analytic form is exact at any box size, so a sphere/capsule resting on or
	// sliding across a big box top keeps its contact under fixed16. conservative_advance
	// still owns the swept semantics — only the closest-point method changes.
	auto is_rounded = [](shape_type t) { return t == shape_type::sphere || t == shape_type::capsule; };
	const bool sh1_rounded_sh2_box = is_rounded(sh1->get_type()) && sh2->get_type() == shape_type::box;
	const bool sh1_box_sh2_rounded = sh1->get_type() == shape_type::box && is_rounded(sh2->get_type());
	if (sh1_rounded_sh2_box || sh1_box_sh2_rounded) {
		// Roles: the mover is shape A (swept by xA); the box may be the mover or the
		// target. box_*_core_closest returns the axis outward from the box toward the
		// rounded core, so when the box is the mover we negate it (conservative_advance
		// wants outward from B(target) toward A).
		const bool box_is_target = sh1_rounded_sh2_box;
		shape<T> * rsh = box_is_target ? sh1 : sh2;
		const mat3<T> & Rbox = box_is_target ? Rb : Ra;
		const mat3<T> & Rr = box_is_target ? Ra : Rb;
		const vec3<T> & box_base0 = box_is_target ? base_b : base_a; // box origin, pre-sweep
		const vec3<T> & rnd_base0 = box_is_target ? base_a : base_b; // rounded origin, pre-sweep
		const aa_box<T> & box = (box_is_target ? sh2 : sh1)->get_box();
		const bool box_id = (Rbox == identity);
		const bool rnd_id = (Rr == identity);
		mat3<T> RboxT;
		if (!box_id)
			transpose(RboxT, Rbox);
		// Rounded core endpoints in the rounded shape's local frame (e1 == e0 for a sphere).
		const bool is_capsule = rsh->get_type() == shape_type::capsule;
		vec3<T> e0, e1;
		if (is_capsule) {
			e0.set(rsh->get_capsule().origin);
			add(e1, e0, rsh->get_capsule().direction);
		} else {
			e0.set(rsh->get_sphere().origin);
			e1.set(e0);
		}
		conservative_advance<T>(res, seg.direction, combined_radius, epsilon,
		    [&](const vec3<T> & xA, const vec3<T> &, T & dist, vec3<T> & n, bool & deep) {
			    // Only the mover (shape A) gets the sweep offset.
			    vec3<T> rnd_base(rnd_base0), bbase(box_base0);
			    if (box_is_target)
				    add(rnd_base, xA);
			    else
				    add(bbase, xA);
			    vec3<T> p0, p1; // rounded core endpoints in world
			    if (rnd_id) {
				    add(p0, rnd_base, e0);
				    add(p1, rnd_base, e1);
			    } else {
				    vec3<T> t;
				    mul(t, Rr, e0);
				    add(p0, rnd_base, t);
				    mul(t, Rr, e1);
				    add(p1, rnd_base, t);
			    }
			    if (is_capsule)
				    box_segment_core_closest(p0, p1, bbase, Rbox, RboxT, box, box_id, dist, n, deep, epsilon);
			    else
				    box_point_core_closest(p0, bbase, Rbox, RboxT, box, box_id, dist, n, deep, epsilon);
			    if (!box_is_target)
				    neg(n);
		    });
	} else if (Ra != identity || Rb != identity) {
		// Oriented: bake each shape's rotation into its support (world-space).
		mat3<T> Rat, Rbt;
		transpose(Rat, Ra);
		transpose(Rbt, Rb);
		auto support_a = [&](const vec3<T> & dir, vec3<T> & o) { gjk_core_support(o, sh1, Ra, Rat, base_a, dir); };
		auto support_b = [&](const vec3<T> & dir, vec3<T> & o) { gjk_core_support(o, sh2, Rb, Rbt, base_b, dir); };
		gjk_sweep<T>(res, support_a, support_b, seg.direction, combined_radius, epsilon);
	} else {
		// Identity fast path — no rotation math.
		auto support_a = [&](const vec3<T> & dir, vec3<T> & o) { gjk_core_support(o, sh1, base_a, dir); };
		auto support_b = [&](const vec3<T> & dir, vec3<T> & o) { gjk_core_support(o, sh2, base_b, dir); };
		gjk_sweep<T>(res, support_a, support_b, seg.direction, combined_radius, epsilon);
	}
	if (!res.valid)
		return false;
	if (res.hit) {
		col.time = res.time;
		col.normal.set(res.normal);
		col.depth = res.depth;
		vec3<T> travel;
		mul(travel, seg.direction, res.time);
		add(col.point, seg.origin, travel);
	} else {
		col.time = tr::one();
	}
	return true;
}

// Intra-shape merge used by test_segment and test_solid. Unlike merge_collision
// (inter-solid), this always averages normals at equal times — co-located shapes
// on the same solid genuinely want the averaged normal. `modify_scope` is held
// across the per-pair loop and gates whether trigger_scope bits accumulate.
template <typename T>
void merge_intra_pair(collision<T> & result, const collision<T> & col, T epsilon, bool & modify_scope) {
	using tr = scalar_traits<T>;
	int trigger_scope = result.trigger_scope;
	if (col.time < tr::one()) {
		if (col.time < result.time) {
			result.set(col);
		} else if (result.time == col.time) {
			add(result.normal, col.normal);
			if (!normalize_carefully(result.normal, epsilon))
				result.set(col);
		}
		modify_scope |= (col.time == T {});
	}
	result.trigger_scope = modify_scope ? (trigger_scope | col.trigger_scope) : trigger_scope;
}

template <typename T>
void test_segment(collision<T> & result, const segment<T> & seg, solid<T> * s, T epsilon) {
	using tr = scalar_traits<T>;
	collision<T> col;
	col.collider = s;
	const T one = tr::one();

	auto & shapes = s->get_shapes();
	int n = static_cast<int>(shapes.size());
	bool modify_scope = false;

	for (int i = 0; i < n; ++i) {
		auto * sh = shapes[i].get();
		const vec3<T> & lp = sh->get_local_position();
		switch (sh->get_type()) {
		case shape_type::box: {
			aa_box<T> box;
			box.set(sh->get_box());
			add(box, s->get_position());
			add(box, lp);
			trace_aa_box(col, seg, box);
			break;
		}
		case shape_type::sphere: {
			sphere<T> sph;
			sph.set(sh->get_sphere());
			add(sph, s->get_position());
			add(sph, lp);
			trace_sphere(col, seg, sph, epsilon);
			break;
		}
		case shape_type::capsule: {
			capsule<T> cap;
			cap.set(sh->get_capsule());
			add(cap, s->get_position());
			add(cap, lp);
			trace_capsule(col, seg, cap, epsilon);
			break;
		}
		case shape_type::convex_solid: {
			convex_solid<T> cs;
			cs.set(sh->get_convex_solid());
			segment<T> tmp;
			tmp.set(seg);
			sub(tmp.origin, s->get_position());
			sub(tmp.origin, lp);
			trace_convex_solid(col, tmp, cs, epsilon);
			if (col.time < one) {
				add(col.point, s->get_position());
				add(col.point, lp);
			}
			break;
		}
		case shape_type::traceable: {
			mat3<T> R;
			mul(R, s->get_orientation(), sh->get_local_rotation());
			vec3<T> traceable_origin, roff;
			mul(roff, s->get_orientation(), lp);
			add(traceable_origin, s->get_position(), roff);
			sh->get_traceable()->trace_segment(col, traceable_origin, R, seg);
			modify_scope = true;
			break;
		}
		}

		// Segment traces have no Minkowski expansion: impact == point
		if (col.time < one)
			col.impact.set(col.point);

		// Trigger zones: OR collidee's trigger bits into the result on static
		// overlap (t == 0). Lets callers tag a solid as a damage zone / volume
		// and read which zones a trace ended up inside via result.trigger_scope.
		if (col.time == T {})
			col.trigger_scope |= s->get_trigger_scope();

		merge_intra_pair(result, col, epsilon, modify_scope);
	}
}

// `margin` inflates both solids' shapes (Minkowski-grows the contact boundary by
// that distance), so a swept/overlap test reports contact when the true surfaces
// are within `margin` rather than only on touch. Used by speculative-contacts
// discovery to enumerate near-resting contacts the bare swept query misses; the
// caller recovers the true signed gap as (margin − reported depth). Default 0 =
// exact shapes, the behaviour every existing caller relies on.
template <typename T>
void test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2, T epsilon, T margin = T {}, bool use_gjk = true) {
	using tr = scalar_traits<T>;
	collision<T> col;
	col.collider = s2;
	const T one = tr::one();
	T zero_val {};

	// Inflate a Minkowski AABB / convex by the margin (no-op when margin == 0, so
	// the exact-shape fast path is unaffected). Combined-radius pairs (sphere,
	// capsule) add the margin to their radius directly at the call site.
	auto inflate_box = [&](aa_box<T> & b) {
		if (margin > zero_val) {
			b.mins.x -= margin; b.mins.y -= margin; b.mins.z -= margin;
			b.maxs.x += margin; b.maxs.y += margin; b.maxs.z += margin;
		}
	};
	auto inflate_planes = [&](convex_solid<T> & c) {
		if (margin > zero_val)
			for (auto & p : c.planes)
				p.distance = p.distance + margin;
	};

	// Squared segment length is invariant across all shape pairs in this call;
	// hoist it for the sphere/sphere fast reject below.
	T dir_sq = length_squared(seg.direction);

	auto & shapes1 = s1->get_shapes();
	auto & shapes2 = s2->get_shapes();
	int n1 = static_cast<int>(shapes1.size());
	int n2 = static_cast<int>(shapes2.size());
	bool modify_scope = false;

	for (int i = 0; i < n1; ++i) {
		auto * sh1 = shapes1[i].get();
		for (int j = 0; j < n2; ++j) {
			auto * sh2 = shapes2[j].get();
			modify_scope = false;

			const vec3<T> & lp1 = sh1->get_local_position();
			const vec3<T> & lp2 = sh2->get_local_position();
			vec3<T> lp_delta;
			sub(lp_delta, lp2, lp1);

			// Traceable paths route through the traceable callback regardless of
			// the other side's primitive type, so dispatch on traceable-ness first.
			if (sh1->get_type() == shape_type::traceable) {
				segment<T> iseg;
				vec3<T> roff;
				mul(roff, s2->get_orientation(), lp2);
				add(iseg.origin, s2->get_position(), roff);
				mul(iseg.direction, seg.direction, -tr::one());
				mat3<T> R;
				mul(R, s1->get_orientation(), sh1->get_local_rotation());
				vec3<T> tr_origin;
				mul(roff, s1->get_orientation(), lp1);
				add(tr_origin, seg.origin, roff);
				sh1->get_traceable()->trace_solid(col, s2, tr_origin, R, iseg, margin);
				col.invert();
				sub(iseg.origin, col.point);
				add(col.point, seg.origin, iseg.origin);
				modify_scope = true;
			} else if (sh2->get_type() == shape_type::traceable) {
				mat3<T> R;
				mul(R, s2->get_orientation(), sh2->get_local_rotation());
				vec3<T> tr_origin, roff;
				mul(roff, s2->get_orientation(), lp2);
				add(tr_origin, s2->get_position(), roff);
				sh2->get_traceable()->trace_solid(col, s1, tr_origin, R, seg, margin);
				modify_scope = true;
			}
			// Accurate GJK for the rounded×polytope pairs (one decision, not a guard
			// pasted into each branch). trace_pair_gjk is shape-agnostic — it reads
			// both shapes via the support functions — so the eligible set is named
			// once here. It declines (returns false) on deep core penetration, and
			// is skipped when narrowphase is set cheap; either way we fall through to
			// the per-pair fallback chain below.
			else if (use_gjk && gjk_eligible_pair(sh1->get_type(), sh2->get_type())
			         && trace_pair_gjk(col, seg, s1, s2, sh1, sh2, lp1, lp2, margin, epsilon)) {
				// handled by GJK
			}
			// AABox vs *
			else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::box) {
				aa_box<T> box;
				box.set(sh2->get_box());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
				inflate_box(box);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::sphere) {
				aa_box<T> box;
				find_bounding_box(box, sh2->get_sphere());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
				inflate_box(box);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::capsule) {
				aa_box<T> box;
				sh2->get_bound(box);
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
				inflate_box(box);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::convex_solid) {
				vec3<T> half;
				sub(half, sh1->get_box().maxs, sh1->get_box().mins);
				mul(half, tr::half());
				convex_solid<T> cs;
				build_convex_box_minkowski(cs, sh2->get_convex_solid(), half);
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->get_box().mins, sh1->get_box().maxs);
				mul(sh1_offset, tr::half());
				add(sh1_offset, lp1);
				inflate_planes(cs);
				trace_forward_convex(col, seg, s2->get_position(), lp2, cs, sh1_offset, epsilon);
			}
			// Sphere vs *
			else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::box) {
				aa_box<T> box1;
				find_bounding_box(box1, sh1->get_sphere());
				aa_box<T> box;
				box.set(sh2->get_box());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				inflate_box(box);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::sphere) {
				vec3<T> origin;
				origin.set(s2->get_position());
				add(origin, lp_delta);
				sub(origin, sh1->get_sphere().origin);
				add(origin, sh2->get_sphere().origin);
				T r_sum = sh1->get_sphere().radius + sh2->get_sphere().radius + margin;
				// Fast reject: if start-position centers are too far apart for the
				// swept sphere to ever reach the target sphere this tick, skip the
				// quadratic root solve. Conservative no-sqrt bound:
				//   2·(r_sum² + |dir|²) ≥ (r_sum + |dir|)²   (AM-GM)
				// — looser than the sqrt-form by a factor of ≤2 but free.
				vec3<T> diff;
				sub(diff, seg.origin, origin);
				T limit = (r_sum * r_sum + dir_sq) * tr::two();
				if (length_squared(diff) > limit) {
					col.time = one;  // explicit miss; merge below leaves result untouched
				} else {
					sphere<T> sph;
					sph.set(origin, r_sum);
					trace_sphere(col, seg, sph, epsilon);
				}
			} else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::capsule) {
				vec3<T> origin;
				origin.set(s2->get_position());
				add(origin, lp_delta);
				sub(origin, sh1->get_sphere().origin);
				add(origin, sh2->get_capsule().origin);
				capsule<T> cap;
				cap.set(origin, sh2->get_capsule().direction, sh2->get_capsule().radius + sh1->get_sphere().radius + margin);
				trace_capsule(col, seg, cap, epsilon);
			} else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::convex_solid) {
				convex_solid<T> cs;
				cs.set(sh2->get_convex_solid());
				for (auto & p : cs.planes)
					p.distance = p.distance + sh1->get_sphere().radius;
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->get_sphere().origin, lp1);
				inflate_planes(cs);
				trace_forward_convex(col, seg, s2->get_position(), lp2, cs, sh1_offset, epsilon);
			}
			// Capsule vs *
			else if (sh1->get_type() == shape_type::capsule && sh2->get_type() == shape_type::box) {
				aa_box<T> box1;
				sh1->get_bound(box1);
				aa_box<T> box;
				box.set(sh2->get_box());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, box1.mins);
				sub(box.mins, box1.maxs);
				inflate_box(box);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::capsule && sh2->get_type() == shape_type::sphere) {
				vec3<T> origin;
				origin.set(s2->get_position());
				add(origin, lp_delta);
				sub(origin, sh1->get_capsule().origin);
				add(origin, sh2->get_sphere().origin);
				vec3<T> dir;
				dir.set(sh1->get_capsule().direction);
				neg(dir);
				capsule<T> cap;
				cap.set(origin, dir, sh1->get_capsule().radius + sh2->get_sphere().radius + margin);
				trace_capsule(col, seg, cap, epsilon);
			} else if (sh1->get_type() == shape_type::capsule && sh2->get_type() == shape_type::convex_solid) {
				// Fallback (deep core penetration): capsule support relative to its
				// A endpoint, r + max(0, n·D). Inflate each plane and sweep A through
				// the resulting convex. Wrong normal near edges, but recovers depth.
				convex_solid<T> cs;
				cs.set(sh2->get_convex_solid());
				const vec3<T> & D = sh1->get_capsule().direction;
				for (auto & p : cs.planes) {
					T nd = dot(p.normal, D);
					p.distance = p.distance + sh1->get_capsule().radius + tr::max_val(zero_val, nd);
				}
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->get_capsule().origin, lp1);
				inflate_planes(cs);
				trace_forward_convex(col, seg, s2->get_position(), lp2, cs, sh1_offset, epsilon);
			} else if (sh1->get_type() == shape_type::capsule && sh2->get_type() == shape_type::capsule) {
				vec3<T> base;
				base.set(s2->get_position());
				add(base, lp_delta);
				sub(base, sh1->get_capsule().origin);
				add(base, sh2->get_capsule().origin);
				trace_capsule_capsule(col,
				                      seg,
				                      base,
				                      sh1->get_capsule().direction,
				                      sh2->get_capsule().direction,
				                      sh1->get_capsule().radius + sh2->get_capsule().radius + margin,
				                      epsilon);
			}
			else if (sh1->get_type() == shape_type::convex_solid && sh2->get_type() == shape_type::box) {
				vec3<T> half;
				sub(half, sh2->get_box().maxs, sh2->get_box().mins);
				mul(half, tr::half());
				convex_solid<T> cs;
				build_convex_box_minkowski(cs, sh1->get_convex_solid(), half);
				vec3<T> sh2_offset;
				add(sh2_offset, sh2->get_box().mins, sh2->get_box().maxs);
				mul(sh2_offset, tr::half());
				inflate_planes(cs);
				trace_inverted_convex(col, seg, s1->get_position(), s2->get_position(), lp_delta, cs, sh2_offset, epsilon);
			}
			// Convex solid vs sphere
			else if (sh1->get_type() == shape_type::convex_solid && sh2->get_type() == shape_type::sphere) {
				convex_solid<T> cs;
				cs.set(sh1->get_convex_solid());
				for (auto & p : cs.planes)
					p.distance = p.distance + sh2->get_sphere().radius;
				inflate_planes(cs);
				trace_inverted_convex(col, seg, s1->get_position(), s2->get_position(), lp_delta, cs, sh2->get_sphere().origin, epsilon);
			}
			// Convex solid vs capsule (mirror of capsule × convex)
			else if (sh1->get_type() == shape_type::convex_solid && sh2->get_type() == shape_type::capsule) {
				convex_solid<T> cs;
				cs.set(sh1->get_convex_solid());
				const vec3<T> & D = sh2->get_capsule().direction;
				for (auto & p : cs.planes) {
					T nd = dot(p.normal, D);
					p.distance = p.distance + sh2->get_capsule().radius + tr::max_val(zero_val, nd);
				}
				inflate_planes(cs);
				trace_inverted_convex(col, seg, s1->get_position(), s2->get_position(), lp_delta, cs, sh2->get_capsule().origin, epsilon);
			}
			// Minkowski difference sh2 ⊕ (-sh1) — locus of (sh1.pos - sh2.pos) at
			// which the shapes overlap. Bounded by two plane families:
			//   sh2's faces (n, d) inflated to (n, d + sup(sh1, -n))
			//   sh1's faces (n, d) contributed as (-n, d + sup(sh2, -n))
			// Both families are required: omitting the second leaves the polytope
			// unbounded along sh1's face normals, the same false-positive static
			// overlap that box-vs-convex hits without its axis planes.
			else if (sh1->get_type() == shape_type::convex_solid && sh2->get_type() == shape_type::convex_solid) {
				const auto & a = sh1->get_convex_solid();
				const auto & b = sh2->get_convex_solid();
				convex_solid<T> cs;
				cs.planes = b.planes;
				cs.planes.reserve(b.planes.size() + a.planes.size());
				vec3<T> sup;
				vec3<T> neg_n;
				for (auto & p : cs.planes) {
					neg(neg_n, p.normal);
					support(sup, a, neg_n);
					p.distance = p.distance + dot(sup, neg_n);
				}
				for (const auto & p1 : a.planes) {
					neg(neg_n, p1.normal);
					support(sup, b, neg_n);
					cs.planes.push_back(plane<T>(neg_n, p1.distance + dot(sup, neg_n)));
				}
				inflate_planes(cs);
				trace_forward_convex(col, seg, s2->get_position(), lp2, cs, lp1, epsilon);
			}

			// Compute impact point for solid traces.
			// col.point represents s1's center at impact time; sh1's world contact
			// surface is offset from that by sh1's local_position + support(shape, -normal).
			if (col.time < one && sh1->get_type() != shape_type::traceable && sh2->get_type() != shape_type::traceable) {
				vec3<T> sup;
				vec3<T> neg_n;
				neg(neg_n, col.normal);
				support(sup, *sh1, neg_n);
				add(col.impact, col.point, sup);
				add(col.impact, lp1);
			}
			// For a traceable pair, trace_solid is responsible for filling col.impact
			// with the world contact point on its surface (see the traceable contract
			// in traceable.h). We must NOT overwrite it with col.point (the mover's
			// origin at impact) — that breaks lever-arm math (kinematic carry / angular
			// response) for anything resting on or pushed by a trimesh/heightfield.

			// Per-pair reset of col.trigger_scope so stale bits from a previous
			// shape pair don't leak in. Then OR collidee's trigger bits on
			// static overlap. Works for primitive AND traceable pairs so
			// trimesh trigger volumes report the same way as primitive ones.
			col.trigger_scope = 0;
			if (col.time == zero_val)
				col.trigger_scope = s2->get_trigger_scope();

			merge_intra_pair(result, col, epsilon, modify_scope);
		}
	}
}

// Fold a per-pair `col` into the running `result`. Earlier hits replace;
// equal-time hits average normals when `average_normals` is on. trigger_scope
// bits OR in regardless of whether `col` is the closer hit — `result.set(col)`
// would overwrite trigger_scope, so we save/restore it around the merge.
// `unblended_normal` (optional) receives the contact's first/earliest collider's
// true normal. result.normal can become a blend of several equal-time colliders'
// normals when average_normals is on, but result.collider/.depth/.point stay the
// first collider's; this hands callers (the contact solver) that collider's
// un-blended normal so they can resolve along the true pair direction without
// re-testing. Kept in lockstep with result.collider — set on every result.set(col).
template <typename T>
void merge_collision(collision<T> & result, const collision<T> & col, T epsilon, bool average_normals,
                     vec3<T> * unblended_normal = nullptr) {
	using tr = scalar_traits<T>;
	int trigger_scope = result.trigger_scope;
	// Adopt col as the contact and, in lockstep, capture its un-blended normal —
	// keeps the out-param tracking exactly the result.set(col) paths.
	auto adopt = [&] {
		result.set(col);
		if (unblended_normal)
			unblended_normal->set(col.normal);
	};
	if (col.time < tr::one()) {
		if (col.time < result.time) {
			adopt();
		} else if (average_normals && result.time == col.time) {
			add(result.normal, col.normal);
			if (!normalize_carefully(result.normal, epsilon))
				adopt();
		}
	}
	result.trigger_scope = trigger_scope | col.trigger_scope;
}

} // namespace hop
