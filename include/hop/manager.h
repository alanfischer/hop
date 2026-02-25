#pragma once

#include <hop/collision.h>

namespace hop {

template <typename T> class solid;

template <typename T> class manager {
public:
	virtual ~manager() = default;

	virtual int find_solids_in_aa_box(const aa_box<T> & box, solid<T> * solids[], int max_solids) = 0;
	virtual void trace_segment(collision<T> & result, const segment<T> & seg, int collide_with_bits) = 0;
	virtual void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int collide_with_bits) = 0;
	virtual void pre_update(int dt, T fdt) = 0;
	virtual void post_update(int dt, T fdt) = 0;
	virtual void pre_update(solid<T> * s, int dt, T fdt) = 0;
	virtual void intra_update(solid<T> * s, int dt, T fdt) = 0;
	virtual bool collision_response(solid<T> * s, vec3<T> & position, vec3<T> & remainder, collision<T> & col) = 0;
	virtual void post_update(solid<T> * s, int dt, T fdt) = 0;
};

} // namespace hop
