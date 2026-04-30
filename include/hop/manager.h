#pragma once

#include <hop/collision.h>
#include <hop/math/segment.h>

#include <vector>

namespace hop {

template <typename T> class solid;

// Pluggable spatial / engine-integration interface. A manager hooks into the
// simulator at three independent axes — implementations are free to use any
// subset and leave the rest as no-ops:
//
//   1. Broad-phase acceleration. Override find_solids_in_aa_box to replace
//      the simulator's linear scan with a BVH, octree, BSP, etc. Return -1
//      to fall through to the default scan. See bvh_manager.
//
//   2. External / custom geometry. Override trace_segment and trace_solid
//      to merge in collisions against geometry that is NOT in the simulator's
//      solid list — e.g. a host engine's level BSP, a heightfield, or a
//      static trimesh registered outside hop. The simulator calls these
//      after its own per-solid loop and folds the result into the same
//      collision<T>. Managers that only accelerate broad-phase (like
//      bvh_manager) leave them as no-ops.
//
//   3. Lifecycle / response hooks. The pre/post/intra_update and
//      collision_response methods let a host engine observe or override
//      per-tick and per-solid behavior (e.g. routing collision response
//      through an engine's character controller).
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

	// Optional: a per-tick iteration order for the simulator's update loop.
	// Returning non-null commits to including every solid the simulator should
	// update — solids in the simulator but absent from this list will be
	// skipped. Pointers must stay valid until the next manager mutation
	// (add/remove/rebuild). Default null = simulator uses its own solids_ order.
	virtual const std::vector<solid<T> *> * get_iteration_order() const { return nullptr; }
};

} // namespace hop
