#pragma once

#include <hop/collision.h>
#include <hop/math/mat3.h>
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
//
//                   When returning t=0 (overlap), also set `result.depth` to
//                   the penetration depth — the distance the solid must move
//                   along `result.normal` to reach the surface.  The simulator
//                   uses this for position correction.  Leaving it at 0 falls
//                   back to the old epsilon-nudge behaviour, which is safe but
//                   slower to resolve deep penetrations.
//
//                   `margin` (default 0 = exact surface) Minkowski-inflates the
//                   geometry outward by that distance, so a solid whose surface
//                   is within `margin` of the mesh registers as an overlap (t=0)
//                   even when not actually penetrating.  This is how the
//                   speculative-contacts pipeline discovers near-resting
//                   contacts against a mesh.  Report `result.depth` measured
//                   against the *inflated* surface (depth = margin - true_gap);
//                   the simulator recovers the true signed gap as
//                   (margin - depth).  Swept hits (t > 0) should likewise stop
//                   at the inflated surface.  Treat margin == 0 as the exact
//                   mesh so existing (non-speculative) callers are unaffected.
template <typename T> class traceable {
public:
	virtual ~traceable() = default;
	virtual void get_bound(aa_box<T> & result) = 0;
	// `orientation` is the traceable's world rotation (solid_orientation ·
	// shape_local_rotation). The geometry is authored in the traceable's local
	// frame; implementors transform the query into that frame with its transpose
	// and map results back. Identity (the common case) is an exact no-op.
	virtual void trace_segment(collision<T> & result, const vec3<T> & position, const mat3<T> & orientation, const segment<T> & seg) = 0;
	virtual void trace_solid(collision<T> & result, solid<T> * s, const vec3<T> & position, const mat3<T> & orientation, const segment<T> & seg, T margin) = 0;
};

} // namespace hop
