#pragma once

#include <hop/collision.h>

namespace hop {

template <typename T> class collision_listener {
public:
	virtual ~collision_listener() = default;
	virtual void on_collision(const collision<T> & c) = 0;
};

} // namespace hop
