#pragma once

#include <hop/collision.h>

namespace hop {

template <typename T> class solid;

template <typename T> class traceable {
public:
	virtual ~traceable() = default;
	virtual void get_bound(aa_box<T> & result) = 0;
	virtual void trace_segment(collision<T> & result, const vec3<T> & position, const segment<T> & seg) = 0;
	virtual void trace_solid(collision<T> & result, solid<T> * s, const vec3<T> & position, const segment<T> & seg) = 0;
};

} // namespace hop
