#pragma once

#include <hop/math/support.h>
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
	vec3<T> point;
	vec3<T> impact;
	vec3<T> normal;
	vec3<T> velocity;
	solid<T> * collider = nullptr;
	solid<T> * collidee = nullptr;
	int scope = 0;

	collision & set(const collision & c) {
		time = c.time;
		point.set(c.point);
		impact.set(c.impact);
		normal.set(c.normal);
		velocity.set(c.velocity);
		collider = c.collider;
		collidee = c.collidee;
		scope = c.scope;
		return *this;
	}

	collision & reset() {
		time = tr::one();
		point.reset();
		impact.reset();
		normal.reset();
		velocity.reset();
		collider = nullptr;
		collidee = nullptr;
		scope = 0;
		return *this;
	}

	void invert() {
		std::swap(collider, collidee);
		neg(normal);
		neg(velocity);
	}
};

} // namespace hop
