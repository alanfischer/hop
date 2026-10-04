#pragma once

#include <cstdint>
#include <hop/math/support.h>
#include <utility>

// How many contact points one touch slot's manifold can hold. Four is the standard
// choice: a box resting on a face has four corners under it, and three already hold a
// plane, so this is the point at which more buys nothing. It is a compile-time constant
// because a touch slot embeds the array — 12 slots per solid, and hop runs on targets
// where that footprint is not free. Setting it to 1 restores the pre-manifold BEHAVIOUR
// exactly — one contact per partner — which test_manifold holds to by compiling itself
// twice. The footprint comes back to within 16 bytes a slot rather than exactly: a point
// still carries the feature id a single contact has no use for.
#ifndef HOP_MAX_MANIFOLD_POINTS
#define HOP_MAX_MANIFOLD_POINTS 4
#endif

namespace hop {

inline constexpr int max_manifold_points = HOP_MAX_MANIFOLD_POINTS;
static_assert(max_manifold_points >= 1 && max_manifold_points <= 8,
              "HOP_MAX_MANIFOLD_POINTS must be in [1, 8]");

template <typename T> class solid;

// One point of a contact manifold, in the form the discovery pass hands the touch
// cache. Each point carries its OWN normal and its OWN signed gap — that is what lets
// four rows under a box see four different gaps and level it, where one row can only
// rock it. `id` is the feature ID; see collision::feature_id for the contract.
template <typename T> struct contact_point {
	vec3<T> impact;      // world contact point on the partner's surface
	vec3<T> normal;      // partner -> self, this point's own separating direction
	T separation {};     // signed gap along normal: 0 touching, <0 penetrating
	uint32_t id = 0;     // feature ID; the warm-start key across ticks
};

// `collider` / `collidee` are non-owning. The simulator owns its solids
// (via shared_ptr in simulator::solids_) and clears these pointers during
// remove_solid() so they never dangle past a tick. Callbacks may copy them
// freely; just don't cache one across a remove_solid() call.

// What one reported contact means. Two kinds of query fill this in and they agree on
// every field below, which is worth saying plainly because the fields are easy to read
// as more independent than they are:
//
//   test_segment  — a ray or a point (a zero-length segment) against a solid.
//   test_solid    — a solid swept along a segment against another solid.
//
// `time`   fraction of the segment at which the contact happens.
//          one() = no contact at all; everything below only means something when it is
//          less than that. 0 = the contact is AT the segment's start, which is two
//          different situations — the start was inside the collidee, or it simply met a
//          surface with no distance to travel first. `started_inside` is what separates
//          them; do not try to read that out of the time.
// `depth`  how far past the surface the contact sits, along `normal`. 0 means touching
//          rather than overlapping, which is what an ordinary crossing reports and what
//          a mover resting exactly on a surface reports at zero margin. Positive means
//          real penetration. If the query passed a `margin`, depth is measured against
//          the inflated surface instead, so the true signed gap is (margin - depth).
// `point`  where the contact is, in world space. For a crossing that is on the
//          collidee's surface; for an overlap at the start it is the query's own origin,
//          which is INSIDE the collidee — the surface is `point + normal * depth`.
// `impact` the same place, except for a swept solid that carries an orientation, where
//          it is the feature of the MOVER that actually touches. Segment queries have no
//          Minkowski expansion, and a mover with no orientation of its own reports its
//          centre, so in both of those impact == point.
// `normal` unit, pointing from the collidee back toward the collider — against the
//          direction of travel for a sweep, out of the collidee for an overlap.
//
// `collider` / `collidee` are non-owning; see the note above on their lifetime.

template <typename T> struct collision {
	using tr = scalar_traits<T>;

	T time = tr::one();
	T depth = T {};
	vec3<T> point;
	vec3<T> impact;
	vec3<T> normal;
	vec3<T> velocity;
	solid<T> * collider = nullptr;
	solid<T> * collidee = nullptr;
	// OR of every statically-overlapping (t == 0) collidee's trigger_scope.
	// Use to detect whether a trace ended up inside any tagged trigger volume.
	// Always 0 if no static overlap occurred.
	int trigger_scope = 0;
	// The segment began inside the collidee's blocking volume. Orthogonal to `time`: a
	// non-convex collidee can report a crossing further along AND have begun inside
	// something, and both facts are wanted — the ordinary case for a BSP, where one
	// solid holds a whole map.
	//
	// Segment path only. The solid path needs no equivalent because `depth` already
	// carries it there: at zero margin a swept trace reports depth 0 for resting on a
	// surface and positive only for real overlap, which is what the solver's
	// start-penetration branch keys on. A segment has no extent to measure a depth from.
	//
	// And `depth` cannot stand in for it here: it describes the reported CONTACT rather
	// than the origin (a crossing reports depth 0 however buried the start was), and
	// `test_inside` is non-strict, so a segment starting exactly ON a surface is inside
	// it with depth exactly 0 — the commonest pose there is.
	bool started_inside = false;

	collision & set(const collision & c) {
		time = c.time;
		depth = c.depth;
		point.set(c.point);
		impact.set(c.impact);
		normal.set(c.normal);
		velocity.set(c.velocity);
		collider = c.collider;
		collidee = c.collidee;
		trigger_scope = c.trigger_scope;
		started_inside = c.started_inside;
		return *this;
	}

	collision & reset() {
		time = tr::one();
		depth = T {};
		point.reset();
		impact.reset();
		normal.reset();
		velocity.reset();
		collider = nullptr;
		collidee = nullptr;
		trigger_scope = 0;
		started_inside = false;
		return *this;
	}

	// started_inside is a property of the segment, not of the pair's ordering, so it
	// survives the swap unchanged.
	void invert() {
		std::swap(collider, collidee);
		neg(normal);
		neg(velocity);
	}
};

} // namespace hop
