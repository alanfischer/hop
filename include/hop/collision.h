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
		return *this;
	}

	void invert() {
		std::swap(collider, collidee);
		neg(normal);
		neg(velocity);
	}
};

} // namespace hop
