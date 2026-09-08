#pragma once

#include <hop/math/support.h>
#include <cassert>
#include <utility>

namespace hop {

template <typename T> class solid;

// `collider` / `collidee` are non-owning. The simulator owns its solids
// (via shared_ptr in simulator::solids_) and clears these pointers during
// remove_solid() so they never dangle past a tick. Callbacks may copy them
// freely; just don't cache one across a remove_solid() call.

template <typename T> struct collision {
	using tr = scalar_traits<T>;

	T time = tr::one();
	T depth = T {};
	vec3<T> point;
	// The contact as ONE point: a position, a direction, a surface velocity.
	// When patch_count > 0 these are only a representative — anything that RESOLVES
	// the contact must use the patch, or a resting box is held at one point and
	// cannot stop rocking, which is what the patch is for.
	vec3<T> impact;
	vec3<T> normal;
	vec3<T> velocity;
	solid<T> * collider = nullptr;
	solid<T> * collidee = nullptr;
	// OR of every statically-overlapping (t == 0) collidee's trigger_scope.
	// Use to detect whether a trace ended up inside any tagged trigger volume.
	// Always 0 if no static overlap occurred.
	int trigger_scope = 0;

	// How WIDE the contact is, where the geometry can resolve it: each point carries
	// its own lever arm, and that is the width one point cannot have. Four, because a
	// convex face-vs-face patch reduces to at most four points preserving its area
	// (as Bullet, PhysX and Godot all cap it). patch_count == 0 means impact/normal
	// are the whole contact.
	static constexpr int max_patch_points = 4;

	// `feature` says WHICH point this is, so the same corner keeps its warm start.
	// Producers must derive it from stable geometry (face + corner), never iteration
	// order, and keep it non-zero. Uniqueness is required only within one contact:
	// merge_intra_pair keeps one winning shape pair per (body, partner), so two
	// traceables alternating as winner can alias ids across ticks, costing a tick of
	// stale warm start.
	static constexpr int no_feature = 0;

	struct patch_point {
		vec3<T> impact;
		vec3<T> normal;
		T depth {};
		int feature = no_feature;
	};

	patch_point patch[max_patch_points];
	int patch_count = 0;

	// impact/normal in patch_point form, so a consumer can write both cases alike.
	patch_point representative() const {
		patch_point p;
		p.impact.set(impact);
		p.normal.set(normal);
		p.depth = depth;
		p.feature = no_feature;
		return p;
	}

	// Overflow is a producer bug (a clipped face is <= max_patch_points), so this
	// asserts in debug and clamps in release rather than reporting it.
	void add_patch_point(const vec3<T> & p_impact, const vec3<T> & p_normal, T p_depth, int feature) {
		assert(patch_count < max_patch_points && "patch overflow: reduce the patch first");
		assert(feature != no_feature && "0 is reserved for the single-point contact");
		if (patch_count >= max_patch_points) return;
		patch_point & e = patch[patch_count++];
		e.impact.set(p_impact);
		e.normal.set(p_normal);
		e.depth = p_depth;
		e.feature = feature;
	}

	void clear_patch() { patch_count = 0; }

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
		// Only the points present, so a single-point contact copies what it always did.
		patch_count = c.patch_count;
		for (int i = 0; i < patch_count; ++i) patch[i] = c.patch[i];
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
		patch_count = 0;
		return *this;
	}

	void invert() {
		std::swap(collider, collidee);
		neg(normal);
		neg(velocity);
		for (int i = 0; i < patch_count; ++i) neg(patch[i].normal);
	}
};

} // namespace hop
