#pragma once

#include <hop/collision.h>
#include <hop/math/bounding.h>
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
			vec3<T> traceable_origin;
			add(traceable_origin, s->get_position(), lp);
			sh->get_traceable()->trace_segment(col, traceable_origin, seg);
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

template <typename T>
void test_solid(collision<T> & result, solid<T> * s1, const segment<T> & seg, solid<T> * s2, T epsilon) {
	using tr = scalar_traits<T>;
	collision<T> col;
	col.collider = s2;
	const T one = tr::one();
	T zero_val {};

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
				add(iseg.origin, s2->get_position(), lp2);
				mul(iseg.direction, seg.direction, -tr::one());
				vec3<T> tr_origin;
				add(tr_origin, seg.origin, lp1);
				sh1->get_traceable()->trace_solid(col, s2, tr_origin, iseg);
				col.invert();
				sub(iseg.origin, col.point);
				add(col.point, seg.origin, iseg.origin);
				modify_scope = true;
			} else if (sh2->get_type() == shape_type::traceable) {
				vec3<T> tr_origin;
				add(tr_origin, s2->get_position(), lp2);
				sh2->get_traceable()->trace_solid(col, s1, tr_origin, seg);
				modify_scope = true;
			}
			// AABox vs *
			else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::box) {
				aa_box<T> box;
				box.set(sh2->get_box());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::sphere) {
				aa_box<T> box;
				find_bounding_box(box, sh2->get_sphere());
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::box && sh2->get_type() == shape_type::capsule) {
				aa_box<T> box;
				sh2->get_bound(box);
				add(box, s2->get_position());
				add(box, lp_delta);
				sub(box.maxs, sh1->get_box().mins);
				sub(box.mins, sh1->get_box().maxs);
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
				trace_aa_box(col, seg, box);
			} else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::sphere) {
				vec3<T> origin;
				origin.set(s2->get_position());
				add(origin, lp_delta);
				sub(origin, sh1->get_sphere().origin);
				add(origin, sh2->get_sphere().origin);
				T r_sum = sh1->get_sphere().radius + sh2->get_sphere().radius;
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
				cap.set(origin, sh2->get_capsule().direction, sh2->get_capsule().radius + sh1->get_sphere().radius);
				trace_capsule(col, seg, cap, epsilon);
			} else if (sh1->get_type() == shape_type::sphere && sh2->get_type() == shape_type::convex_solid) {
				convex_solid<T> cs;
				cs.set(sh2->get_convex_solid());
				for (auto & p : cs.planes)
					p.distance = p.distance + sh1->get_sphere().radius;
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->get_sphere().origin, lp1);
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
				cap.set(origin, dir, sh1->get_capsule().radius + sh2->get_sphere().radius);
				trace_capsule(col, seg, cap, epsilon);
			} else if (sh1->get_type() == shape_type::capsule && sh2->get_type() == shape_type::convex_solid) {
				// Capsule support relative to its A endpoint: r + max(0, n·D).
				// Inflate each plane and sweep A through the resulting convex —
				// same recipe as sphere/box/convex × convex. Wrong normal near
				// edges/vertices of the polyhedron, but never misses contacts.
				convex_solid<T> cs;
				cs.set(sh2->get_convex_solid());
				const vec3<T> & D = sh1->get_capsule().direction;
				for (auto & p : cs.planes) {
					T nd = dot(p.normal, D);
					p.distance = p.distance + sh1->get_capsule().radius + tr::max_val(zero_val, nd);
				}
				vec3<T> sh1_offset;
				add(sh1_offset, sh1->get_capsule().origin, lp1);
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
				                      sh1->get_capsule().radius + sh2->get_capsule().radius,
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
				trace_inverted_convex(col, seg, s1->get_position(), s2->get_position(), lp_delta, cs, sh2_offset, epsilon);
			}
			// Convex solid vs sphere
			else if (sh1->get_type() == shape_type::convex_solid && sh2->get_type() == shape_type::sphere) {
				convex_solid<T> cs;
				cs.set(sh1->get_convex_solid());
				for (auto & p : cs.planes)
					p.distance = p.distance + sh2->get_sphere().radius;
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
			} else if (col.time < one) {
				col.impact.set(col.point);
			}

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
template <typename T>
void merge_collision(collision<T> & result, const collision<T> & col, T epsilon, bool average_normals) {
	using tr = scalar_traits<T>;
	int trigger_scope = result.trigger_scope;
	if (col.time < tr::one()) {
		if (col.time < result.time) {
			result.set(col);
		} else if (average_normals && result.time == col.time) {
			add(result.normal, col.normal);
			if (!normalize_carefully(result.normal, epsilon))
				result.set(col);
		}
	}
	result.trigger_scope = trigger_scope | col.trigger_scope;
}

} // namespace hop
