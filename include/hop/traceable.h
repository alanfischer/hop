#pragma once

#include <hop/collision.h>
#include <hop/math/segment.h>

namespace hop {

template <typename T> class solid;

// Interface for custom collision geometry that a solid can be swept against.
//
// Implementors must handle BOTH swept and static-overlap queries:
//
//   trace_segment — raycasting; `seg.direction` is always non-zero.
//
//   trace_solid   — solid-vs-traceable swept test.  `seg.origin` is the
//                   solid's starting position; `seg.direction` is its
//                   displacement for this frame.
//
//                   IMPORTANT: `seg.direction` may be zero (static overlap /
//                   point-in-shape query).  The built-in shape testers (sphere,
//                   capsule, aa_box) all handle zero direction via a dedicated
//                   test_inside path and return t=0 when the solid is already
//                   overlapping.  Traceable implementations must do the same —
//                   a direction-based denom guard (e.g. `if denom >= 0: skip`)
//                   will silently discard every triangle when direction is zero,
//                   making the overlap test always report "clear".
//
//                   Recommended pattern: check `dot(dir, dir) == T{}` at the
//                   top of trace_solid and run a static Minkowski-sum overlap
//                   check (see HopTrimeshTraceable for a reference
//                   implementation).
template <typename T> class traceable {
public:
	virtual ~traceable() = default;
	virtual void get_bound(aa_box<T> & result) = 0;
	virtual void trace_segment(collision<T> & result, const vec3<T> & position, const segment<T> & seg) = 0;
	virtual void trace_solid(collision<T> & result, solid<T> * s, const vec3<T> & position, const segment<T> & seg) = 0;
};

} // namespace hop
