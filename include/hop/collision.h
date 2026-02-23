#pragma once

#include <memory>
#include <hop/math/math_ops.h>

namespace hop {

template<typename T> class solid;

template<typename T>
struct collision {
	using tr = scalar_traits<T>;

	T time = tr::one();
	vec3<T> point;
	vec3<T> normal;
	vec3<T> velocity;
	std::shared_ptr<solid<T>> collider;
	std::shared_ptr<solid<T>> collidee;
	int scope = 0;

	collision& set(const collision& c) {
		time = c.time;
		point.set(c.point);
		normal.set(c.normal);
		velocity.set(c.velocity);
		collider = c.collider;
		collidee = c.collidee;
		scope = c.scope;
		return *this;
	}

	collision& reset() {
		time = tr::one();
		point.reset();
		normal.reset();
		velocity.reset();
		collider = nullptr;
		collidee = nullptr;
		scope = 0;
		return *this;
	}

	void invert() {
		collider.swap(collidee);
		neg(normal);
		neg(velocity);
	}
};

} // namespace hop
