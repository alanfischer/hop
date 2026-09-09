#pragma once

#include <hop/hop.h>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "hop_bsp_format.h"

// A traceable implementation for GoldSrc BSP hulls.
//
// GoldSrc precomputes four collision trees per model: hull 0 (the point hull,
// stored as nodes + leafs) and hulls 1..3 (clipnodes), each already Minkowski-
// expanded for one fixed box size. A mover is therefore reduced to a POINT and
// swept through whichever hull matches its bounding box — that is what gives
// GoldSrc its exact plane normals and true 18-unit steps, with no trimesh seam
// jitter.
//
// The descent runs in native GoldSrc space (Z-up, inches) against the planes
// exactly as they sit in the file. The Godot-space query is transformed in and
// the result transformed back out; no plane is ever pre-baked, so no float error
// accumulates across a map's thousands of planes.

namespace hopbsp {

// Trace crosspoints are placed this far on the near side of the plane, so a
// trace that stops on a surface never ends up microscopically inside it. Quake's
// DIST_EPSILON, in GoldSrc units.
inline constexpr double DIST_EPSILON = 0.03125;

// Slop band on the "is this mover stuck?" test, in GoldSrc units. Contents is a
// strict d < 0, so a point exactly ON a plane is outside it — but a mover RESTING on
// a floor is exactly that point, and nothing survives the trip through a rotated
// entity's transform exactly. A standing player traces as the point at feet+36, which
// is precisely the floor's expanded top plane, so a hair of float error puts them
// microscopically inside the floor they are standing on, everywhere, all the time.
//
// On open floor that costs nothing: the escape is straight up with ~zero depth. It
// turns vicious next to a wall, because the wall blocks the upward candidate AND every
// sideways candidate is measured at the same height, so each one lands a hair inside
// the same floor and is rejected too. The only candidate left is the one that leaves
// the model altogether — which is how a rider standing in the back corner of
// ww_golem's cockpit got fired out sideways through its hull.
//
// Half of DIST_EPSILON, the epsilon the traces already keep their endpoints off
// surfaces by, so a point this trace could legitimately produce is never called stuck.
inline constexpr double STUCK_SLOP = DIST_EPSILON * 0.5;

// How close to parallel an axis must be to the contact surface before its end stops
// being decidable — see box_support_point. A cosine, so this is an angle: 0.08 is
// about 4.5 degrees of tilt.
inline constexpr double CONTACT_TIE_BAND = 0.08;

// Engine-baked hull box sizes (Half-Life). These live in the engine, not the
// file: the compiler expanded the clipnode trees for exactly these boxes, so a
// consumer has to know them to pick a hull and offset the traced point.
struct hull_size {
	double mins[3];
	double maxs[3];
};
inline constexpr hull_size HULL_SIZES[4] = {
	{ {   0,   0,   0 }, {  0,  0,  0 } },  // 0 — point hull
	{ { -16, -16, -36 }, { 16, 16, 36 } },  // 1 — standing player (32x32x72)
	{ { -32, -32, -32 }, { 32, 32, 32 } },  // 2 — large monster (64x64x64)
	{ { -16, -16, -18 }, { 16, 16, 18 } },  // 3 — crouched player (32x32x36)
};

// GoldSrc's SV_HullForEntity sizing rule, on a mover's GoldSrc-space box size.
//
// One deviation, at the bottom end. GoldSrc's rule is `sx <= 8 -> hull 0`, which works
// there only because entities are built to fit the hulls: a projectile is SetSize(0,0,0)
// and a player is exactly 32 wide, so nothing lands in between. A host engine's bodies
// are sized by whatever the art wanted, and anything between 8 and 32 units wide used to
// be handed hull 3 — a 32x32x36 box, several times its size, expanded around a corner
// rather than its centre. A 12-unit satchel stopped 6 units short of one face of a wall
// and 26 short of the other.
//
// A sized hull is only the right answer for a mover that actually matches it, so below
// 32 units the trace uses hull 0 and inflates it by the mover's own radius instead (see
// trace_solid). That is what GoldSrc does with projectiles, and it is exact rather than
// approximate. Players and monsters are unaffected: 32 wide still picks 1 or 3 by
// height, over 36 still picks 2.
//
// It is not purely a fidelity improvement — hull 0 is the only hull without CLIP
// brushes in it, so a body that moves down here stops being blocked by clip geometry.
// For projectiles that is correct and is what GoldSrc does. A small body that DOES want
// clip brushes has to be sized to a hull, same as in Half-Life.
inline int hull_for_size(double sx, double sz) {
	if (sx < 32.0) return 0;
	if (sx <= 36.0) return (sz <= 36.0) ? 3 : 1;
	return 2;
}

// One hull's tree. hull 0 descends `nodes` and reads contents out of `leafs`;
// hulls 1..3 descend `clipnodes`, whose negative children ARE the contents.
// Both are addressed through child(), so the trace is written once.
struct hull {
	const hop_bsp::BSPPlane *planes = nullptr;
	const hop_bsp::BSPNode *nodes = nullptr;
	const hop_bsp::BSPLeaf *leafs = nullptr;
	const hop_bsp::BSPClipNode *clipnodes = nullptr;
	int node_count = 0;   // entries in whichever tree array is live
	int leaf_count = 0;
	int root = 0;

	bool valid() const { return planes != nullptr && (nodes != nullptr || clipnodes != nullptr); }

	int planenum(int num) const {
		return nodes ? nodes[num].planenum : clipnodes[num].planenum;
	}

	// Child `side` of node `num`. >= 0 is another node; < 0 is a contents value.
	int child(int num, int side) const {
		if (clipnodes) return clipnodes[num].children[side];
		int c = nodes[num].children[side];
		if (c >= 0) return c;
		int leaf = -1 - c;
		if (leaf < 0 || leaf >= leaf_count) return hop_bsp::CONTENTS_SOLID;
		return (int)leafs[leaf].contents;
	}
};

// Which contents values stop a trace, as a bitmask over -contents (SOLID = -2 →
// bit 2). GoldSrc blocks on CONTENTS_SOLID alone; sky, water, slime and lava are
// passable, which is exactly why a bullet flies through a sky brush on hull 0
// while a player still collides with it on hull 1 (the compiler writes sky as
// solid in the clipnode trees). Exposed so a caller can build, say, a
// sky-blocking or water-blocking view of the same map.
inline constexpr int blocking_bit(int contents) {
	// Contents are small negatives (-1..-15). Anything outside that is corruption;
	// bit 0 is unused by any real contents value, so it never matches a mask.
	return (contents >= 0 || contents < -30) ? 1 : (1 << (-contents));
}
inline constexpr int BLOCK_SOLID = blocking_bit(hop_bsp::CONTENTS_SOLID);

struct hull_trace {
	double fraction = 1.0;
	double endpos[3] = { 0, 0, 0 };
	double normal[3] = { 0, 0, 0 };
	bool allsolid = true;  // every point along the trace was inside blocking contents
	bool hit = false;      // fraction < 1 and `normal` is meaningful
};

// Signed distance of `p` from a plane. Solid is the back side (d < 0).
inline double plane_dist(const hop_bsp::BSPPlane &pl, const double p[3]) {
	return (pl.type < 3)
		? p[pl.type] - pl.dist
		: pl.normal[0] * p[0] + pl.normal[1] * p[1] + pl.normal[2] * p[2] - pl.dist;
}

// Contents at `p` with every plane distance biased by `bias`. This does NOT inflate
// the hull, and it is worth being blunt about why, because the name it used to have
// said it did: solid is the back side of a plane only when the descent goes that way,
// and roughly half of any brush's planes have solid on the FRONT. Biasing every
// distance the same way therefore grows the solid on some faces and shrinks it on the
// others — a brush does not inflate, it TRANSLATES by `bias` along each axis.
//
// A nonzero bias is therefore only ever the STUCK_SLOP band, which asks a directional
// question ("is a mover resting ON this surface, rather than in it") that a symmetric
// inflation cannot express anyway, and which the six-axis probes in hull_push_out are
// what actually decide. Symmetric inflation is expressed instead by the crosspoint
// back-off in recursive_hull_check, which needs no bias at all.
inline int hull_point_contents_biased(const hull &h, int num, const double p[3], double bias) {
	while (num >= 0) {
		if (num >= h.node_count) return hop_bsp::CONTENTS_SOLID;
		double d = plane_dist(h.planes[h.planenum(num)], p) - bias;
		num = h.child(num, d < 0 ? 1 : 0);
	}
	return num;
}

// Contents at point `p` (GoldSrc space), descending from node `num`.
inline int hull_point_contents(const hull &h, int num, const double p[3]) {
	return hull_point_contents_biased(h, num, p, 0.0);
}

// The nearest bounding plane of the leaf containing `p`, as an outward push-out
// (`normal`, `depth`). BSP leaves are convex, so the splitting plane crossed with
// the smallest margin during the descent IS the closest surface of that LEAF.
// Returns false if `p` isn't inside blocking contents.
//
// Only a fallback for hull_push_out — the nearest face of a leaf is very often not a
// way out of the SOLID, see there.
inline bool hull_nearest_leaf_plane(const hull &h, int num, const double p[3], int blocking,
                                    double normal[3], double &depth) {
	double best = 1e30;
	double best_n[3] = { 0, 0, 1 };
	while (num >= 0) {
		if (num >= h.node_count) return false;
		const hop_bsp::BSPPlane &pl = h.planes[h.planenum(num)];
		double d = plane_dist(pl, p);
		// best_n is the way OUT through this plane: descending to the back means
		// the plane bounds the cell from above, so out is +normal; to the front
		// means it bounds from below, so out is -normal.
		double mag = d < 0 ? -d : d;
		if (mag < best) {
			best = mag;
			double s = d < 0 ? 1.0 : -1.0;
			best_n[0] = pl.normal[0] * s;
			best_n[1] = pl.normal[1] * s;
			best_n[2] = pl.normal[2] * s;
		}
		num = h.child(num, d < 0 ? 1 : 0);
	}
	if ((blocking & blocking_bit(num)) == 0) return false;
	normal[0] = best_n[0];
	normal[1] = best_n[1];
	normal[2] = best_n[2];
	depth = best;
	return true;
}

// The mover's own axes, in the hull's frame. A GoldSrc hull trace reduces the mover
// to a point and expands every plane by how far the mover reaches along that plane's
// normal — and "how far" depends on which way the mover is TURNED. The expansion is a
// Minkowski sum, and the Minkowski sum of a tilted box is not the sum of its bounding
// box: a 7x1x3 bone standing on its end reaches 3.5 units down, lying flat it reaches
// 0.5, and a trace that reads the unrotated box gets 0.5 either way.
//
// Column j is the mover's local axis j expressed in GoldSrc space. `oriented` is false
// for the identity, which is every non-rotating body — players, projectiles, doors —
// and takes the cheap path through box_support unchanged.
struct mover_basis {
	double axis[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	bool oriented = false;
};

// The support function of a box: how far half-extents `half`, turned by `b`, reach
// past their centre along `n`. Axis-aligned this is the familiar |n|.half; turned, it
// is the same sum taken against the mover's own axes, sum_j |n.m_j| * half_j.
template <typename N>
inline double box_support(const N n[3], const double half[3], const mover_basis &b) {
	if (!b.oriented) {
		const double nx = n[0] < 0 ? -(double)n[0] : (double)n[0];
		const double ny = n[1] < 0 ? -(double)n[1] : (double)n[1];
		const double nz = n[2] < 0 ? -(double)n[2] : (double)n[2];
		return nx * half[0] + ny * half[1] + nz * half[2];
	}
	double sum = 0;
	for (int j = 0; j < 3; ++j) {
		const double d = (double)n[0] * b.axis[0][j] + (double)n[1] * b.axis[1][j] +
		                 (double)n[2] * b.axis[2][j];
		sum += (d < 0 ? -d : d) * half[j];
	}
	return sum;
}

// The box feature that touches a surface whose outward normal is `n`, as an offset
// from the box's centre: its support point along -n.
//
// The TIE is the whole point of this function. When one of the box's axes lies
// parallel to the surface, "which end of it touches" has no answer — both ends do,
// equally — and picking one anyway puts the contact at a CORNER of a box that is lying
// FLAT. The floor then holds the box up off to one side of its own centre, and the
// impulse that should just support it becomes a torque instead. That is what spun a
// gib resting on flat ground to 37 rad/s when the walk-back tried the raw corner, and
// it is why the witness has sat at the centre ever since.
//
// hop already solves this everywhere else: support() for an aa_box returns the axis
// MIDPOINT when the direction component is exactly zero (math/support.h), which is
// what makes debris bed down on a face against the aa_box floor. Two things differ
// against a BSP plane. The dot is float noise near zero rather than zero, so the tie
// needs a band and not an equality. And a hard edge on that band would snap the
// contact from face centre to corner as a body settled through it, so ramp across it
// instead: a single contact point is standing in for the resultant of a pressure
// PATCH, and that resultant really does travel continuously out from the middle of a
// face toward an edge as the body tips onto it.
inline void box_support_point(const double n[3], const double half[3],
                              const mover_basis &b, double out[3]) {
	out[0] = out[1] = out[2] = 0;
	for (int j = 0; j < 3; ++j) {
		const double m[3] = { b.axis[0][j], b.axis[1][j], b.axis[2][j] };
		const double c = n[0] * m[0] + n[1] * m[1] + n[2] * m[2];
		const double a = c < 0 ? -c : c;
		// Smoothstep from "tied, take the midpoint" to "decided, take the end".
		double w = a / CONTACT_TIE_BAND;
		if (w >= 1.0) w = 1.0;
		else w = w * w * (3.0 - 2.0 * w);
		if (w <= 0.0) continue;
		const double k = (c < 0 ? half[j] : -half[j]) * w;
		out[0] += k * m[0];
		out[1] += k * m[1];
		out[2] += k * m[2];
	}
}

// The turned box's own axis-aligned bound, per axis: its reach along each world axis.
// This is what bounds an eject and what the six-axis overlap probes measure with, both
// of which ask "how far does the mover reach THAT way" in hull axes rather than the
// mover's own.
inline void box_world_half(const double half[3], const mover_basis &b, double out[3]) {
	if (!b.oriented) {
		for (int i = 0; i < 3; ++i) out[i] = half[i];
		return;
	}
	for (int i = 0; i < 3; ++i) {
		double e[3] = { 0, 0, 0 };
		e[i] = 1.0;
		out[i] = box_support(e, half, b);
	}
}

// How a sweep treats surfaces it passes near. Resolved once by hull_sweep and carried
// down the descent: both members are invariant for a whole trace, and re-deriving them
// per node in a self-recursive walk is work the compiler cannot hoist.
//
// THREE different mechanisms, which is why they are separate fields rather than one
// signed number.
//
// `half` is the mover's own box, and it is the only one that expands the hull as a
// Minkowski sum: every plane is pushed out by the box's extent along that plane's
// normal, on BOTH sides, so the descent itself sees a mover of size rather than a
// point. That per-plane, per-side push is the part a single biased distance cannot
// express (see hull_point_contents_biased) — and it has to be in the DESCENT, not
// just the crosspoint: a walk that decides "both ends are on the front side" from the
// bare plane never reaches the split at all, so a mover skimming a floor is waved
// through until its traced point is genuinely inside the brush. That is what dropped
// a corpse half its own height into the floor before anything stopped it.
//
// `grow` is the speculative contact margin. It is deliberately NOT an expansion: it
// only holds the crosspoint that much further off a plane the segment does cross, so
// a mover that gets stopped is stopped a hair early. Bodies resting against surfaces
// are found by the overlap path, not by inflating every sweep.
//
// `bias` is the STUCK_SLOP plane TRANSLATION, whose question is directional ("is this
// mover resting on the surface, or in it") and which no symmetric expansion expresses.
struct sweep_skin {
	double half[3] = { 0, 0, 0 };  // mover half-extents (GoldSrc units); expands the hull
	double grow = 0;               // >= 0: crosspoint back-off (speculative margin)
	double bias = 0;               // <= 0: the STUCK_SLOP plane translation, or 0 for none
	bool boxed = false;            // any half-extent nonzero — keeps the point walk free of the math
	mover_basis basis;             // the mover's axes; identity for everything that does not turn
};

// How far the mover's box reaches past its centre along `pl`'s normal: the amount
// that plane moves outward when the hull is expanded for this box. Axial planes
// (pl.type < 3, the overwhelming majority in a BSP) read one extent directly — but
// only for an axis-aligned mover, since a turned box does not line up with them.
inline double plane_offset(const hop_bsp::BSPPlane &pl, const double half[3],
                           const mover_basis &b) {
	if (!b.oriented && pl.type < 3) return half[pl.type];
	return box_support(pl.normal, half, b);
}

// Quake's SV_RecursiveHullCheck. Sweeps the point p1→p2 (GoldSrc space) through
// the hull, splitting the segment at every plane crossing. Returns false once the
// trace has been stopped.
inline bool recursive_hull_check(const hull &h, int num, double p1f, double p2f,
                                 const double p1[3], const double p2[3],
                                 int blocking, hull_trace &tr, sweep_skin skin) {
	if (num < 0) {
		if ((blocking & blocking_bit(num)) == 0) tr.allsolid = false;
		return true;
	}
	if (num >= h.node_count) return false;

	const hop_bsp::BSPPlane &pl = h.planes[h.planenum(num)];
	const double t1 = plane_dist(pl, p1) - skin.bias;
	const double t2 = plane_dist(pl, p2) - skin.bias;
	// The plane, pushed out by the mover's box. `off` is 0 for a point walk, which
	// is every trace against a sized hull (1..3): those trees are already expanded
	// for their box by the compiler, so expanding again would double-count.
	const double off = skin.boxed ? plane_offset(pl, skin.half, skin.basis) : 0.0;

	if (t1 >= off && t2 >= off)   return recursive_hull_check(h, h.child(num, 0), p1f, p2f, p1, p2, blocking, tr, skin);
	if (t1 < -off && t2 < -off)   return recursive_hull_check(h, h.child(num, 1), p1f, p2f, p1, p2, blocking, tr, skin);

	// Split, keeping the crosspoint this far on the near side of the EXPANDED plane.
	// Which child the mover starts in is read from the direction of travel rather
	// than the sign of t1: inside the band [-off, off] its box straddles the plane
	// and the sign says nothing about where it came from.
	const double eps = DIST_EPSILON + skin.grow;
	int side;
	double frac;
	if (t1 > t2) {          // travelling toward the back side
		side = 0;
		frac = (t1 - off - eps) / (t1 - t2);
	} else if (t1 < t2) {   // travelling toward the front side
		side = 1;
		frac = (t1 + off + eps) / (t1 - t2);
	} else {
		// Exactly parallel and inside the band — a box sliding along a surface it
		// rests on, the most ordinary pose there is. Walk the side it is on over the
		// whole segment; the far-side test below then lands at p2, so any contact it
		// reports is at fraction 1 and costs the mover no motion. Blocking here
		// instead would freeze every body the moment it touched a floor.
		side = (t1 >= 0) ? 0 : 1;
		frac = 1.0;
	}
	if (frac < 0) frac = 0;
	if (frac > 1) frac = 1;

	double midf = p1f + (p2f - p1f) * frac;
	double mid[3];
	for (int i = 0; i < 3; ++i) mid[i] = p1[i] + frac * (p2[i] - p1[i]);

	if (!recursive_hull_check(h, h.child(num, side), p1f, midf, p1, mid, blocking, tr, skin))
		return false;

	if ((blocking & blocking_bit(
	         hull_point_contents_biased(h, h.child(num, side ^ 1), mid, skin.bias))) == 0)
		return recursive_hull_check(h, h.child(num, side ^ 1), midf, p2f, mid, p2, blocking, tr, skin);

	if (tr.allsolid) return false;  // never got out of the solid area

	// The far side is blocking: this is the impact. The normal faces back along
	// the side we approached from.
	if (side == 0) {
		tr.normal[0] = pl.normal[0]; tr.normal[1] = pl.normal[1]; tr.normal[2] = pl.normal[2];
	} else {
		tr.normal[0] = -pl.normal[0]; tr.normal[1] = -pl.normal[1]; tr.normal[2] = -pl.normal[2];
	}
	tr.hit = true;

	// Back up if the crosspoint still landed inside solid — rare, but the epsilon
	// nudge can't save every degenerate plane pair.
	while ((blocking & blocking_bit(hull_point_contents_biased(h, h.root, mid, skin.bias))) != 0) {
		frac -= 0.1;
		if (frac < 0) {
			tr.fraction = midf;
			for (int i = 0; i < 3; ++i) tr.endpos[i] = mid[i];
			return false;
		}
		midf = p1f + (p2f - p1f) * frac;
		for (int i = 0; i < 3; ++i) mid[i] = p1[i] + frac * (p2[i] - p1[i]);
	}

	tr.fraction = midf;
	for (int i = 0; i < 3; ++i) tr.endpos[i] = mid[i];
	return false;
}

// Full point sweep through one hull, GoldSrc space in and out.
inline hull_trace hull_sweep_skin(const hull &h, const double start[3], const double end[3],
                                  int blocking, sweep_skin skin) {
	hull_trace tr;
	for (int i = 0; i < 3; ++i) tr.endpos[i] = end[i];
	if (!h.valid()) { tr.allsolid = false; return tr; }
	recursive_hull_check(h, h.root, 0.0, 1.0, start, end, blocking, tr, skin);
	return tr;
}

// A skin for a mover of size `half` (GoldSrc units), with `grow` as its speculative
// crosspoint back-off. All-zero half extents give the plain point walk back.
inline sweep_skin box_skin(const double half[3], double grow,
                           const mover_basis &basis = mover_basis{}) {
	sweep_skin s;
	s.grow = grow;
	s.basis = basis;
	for (int i = 0; i < 3; ++i) {
		s.half[i] = half[i];
		if (half[i] != 0.0) s.boxed = true;
	}
	return s;
}

// The ordinary point sweep, stopped `grow` short of whatever it crosses.
inline hull_trace hull_sweep(const hull &h, const double start[3], const double end[3],
                             int blocking = BLOCK_SOLID, double grow = 0) {
	return hull_sweep_skin(h, start, end, blocking, sweep_skin{ { 0, 0, 0 }, grow, 0.0, false });
}

// The same sweep for a mover with size: the hull is expanded by `half` as the walk
// descends it (see sweep_skin), so the box stops ON surfaces instead of passing
// through them until its centre is inside.
inline hull_trace hull_sweep_box(const hull &h, const double start[3], const double end[3],
                                 int blocking, const double half[3], double grow,
                                 const mover_basis &basis = mover_basis{}) {
	return hull_sweep_skin(h, start, end, blocking, box_skin(half, grow, basis));
}

// A sweep against the hull translated inward by STUCK_SLOP — the same band the stuck
// test and the push-out already measure against, so a mover resting exactly ON a
// surface is not treated as starting inside it. Named rather than spelled as a
// negative margin: it is a different mechanism, not a smaller one.
inline hull_trace hull_sweep_stuck_band(const hull &h, const double start[3],
                                        const double end[3], int blocking,
                                        const double half[3] = nullptr,
                                        const mover_basis &basis = mover_basis{}) {
	sweep_skin skin = half ? box_skin(half, 0.0, basis) : sweep_skin{};
	skin.bias = -STUCK_SLOP;
	return hull_sweep_skin(h, start, end, blocking, skin);
}

// A sweep whose start sits EXACTLY on the surface it is driving into.
//
// Quake's hull walk calls a start on a plane solid and gives up with no surface at all,
// so `!hit` reads as "nothing in the way" — and that pose is precisely where
// depenetration leaves a body. Back off by twice DIST_EPSILON (the epsilon traces
// already keep endpoints off surfaces by), trace from strictly outside, and re-express
// the fraction on the caller's segment, which is shorter by `back` at the front.
//
// Moving AWAY from a flush surface needs no special case: backing off puts the start
// inside the solid it is leaving, the walk gives up again, and the move stays free.
inline hull_trace hull_sweep_off_surface(const hull &h, const double start[3],
                                         const double end[3], int blocking,
                                         const double half[3] = nullptr,
                                         const mover_basis &basis = mover_basis{}) {
	hull_trace miss;
	miss.hit = false;
	double dir[3], len2 = 0;
	for (int i = 0; i < 3; ++i) {
		dir[i] = end[i] - start[i];
		len2 += dir[i] * dir[i];
	}
	if (len2 <= 0) return miss;
	const double len = std::sqrt(len2);
	const double back = DIST_EPSILON * 2.0;
	double from[3];
	for (int i = 0; i < 3; ++i) from[i] = start[i] - dir[i] / len * back;
	hull_trace nudged = half ? hull_sweep_box(h, from, end, blocking, half, 0.0, basis)
	                         : hull_sweep(h, from, end, blocking, 0.0);
	if (!nudged.hit) return miss;
	nudged.fraction = (nudged.fraction * (len + back) - back) / len;
	if (nudged.fraction < 0.0) nudged.fraction = 0.0;
	return nudged;
}


// How far `p` sits inside blocking contents along `dir`, or -1 when it is still
// inside at `limit`.
//
// Measured by sweeping the point from the far end BACK to `p`, which is the ordinary
// empty→solid trace the hull walk is built for and stops exactly on the surface.
// Sweeping outward from `p` instead would start inside solid — Quake's startsolid,
// which reports no surface and no distance, and is the whole reason a point already
// embedded has to be handled separately in the first place.
inline double hull_inside_distance(const hull &h, int root, const double p[3],
                                   const double dir[3], double limit, int blocking) {
	const double away[3] = {
		p[0] + dir[0] * limit, p[1] + dir[1] * limit, p[2] + dir[2] * limit
	};
	// Shrunk by STUCK_SLOP for the same reason hull_push_out is (see there): a
	// candidate that lands ON a floor is a place the mover can be, and rejecting it
	// leaves only candidates that exit the model completely.
	if ((blocking & blocking_bit(hull_point_contents_biased(h, root, away, -STUCK_SLOP))) != 0)
		return -1.0;
	hull_trace tr = hull_sweep_stuck_band(h, away, p, blocking);
	if (!tr.hit) return -1.0;
	// The crosspoint sits DIST_EPSILON on the empty side, so the surface is that much
	// further in than where the trace stopped.
	const double d = limit * (1.0 - tr.fraction) - DIST_EPSILON;
	return d > 0.0 ? d : 0.0;
}

// The shortest push that actually gets `p` OUT of blocking contents (`normal`,
// `depth`), searched along the six axes out to the mover's own `extent` on each.
// Returns false if `p` isn't inside blocking contents.
//
// The obvious rule — nearest bounding plane of the leaf `p` landed in — is wrong far
// more often than it looks. The compiler cuts a solid region into many convex cells,
// so most of a leaf's faces open onto the next cell of the same brush: surveyed over
// ww_golem's golem body and its worldspawn, the nearest leaf plane is a dead end
// about two thirds of the time, at every penetration depth. A small distance is no
// evidence at all that the direction is any good.
//
// The worst case is the one that bit. A player standing on a floor traces as the
// point at feet+36 (hull 1), which is EXACTLY the floor's expanded top plane — so
// when a moving brush entity squeezes them into a wall rising off that floor, a
// zero-distance vertical plane wins over the wall face they actually came through,
// and the push-out comes back with depth 0 pointing into the floor they are standing
// on. Nothing then ejects them, and _body_test_motion reads the vertical normal as
// "flush, not moving into it" and hands back the whole motion — a rider walking out
// through the back of ww_golem's cockpit while it strode along.
//
// Probing instead of deriving means the answer is verified by construction: each
// candidate is a direction we have just traced to open air. Six axis sweeps is what
// nearest_surface already spends on every resting contact, and this runs only on the
// rare path where something is genuinely embedded.
//
// A single axis is often not enough on its own — wedged in a corner where a floor
// meets a wall, every axis probe walks along one of the two surfaces and stays in
// solid, so all six are rejected. That case is handled by the caller instead: the
// recovery loop in _body_test_motion resolves the deepest overlap and re-probes, so
// pass one clears the floor and pass two then finds the wall by a plain axis probe.
// Decomposing a corner over iterations that way is what keeps every reported contact
// a REAL surface. Answering it here in one shot with a diagonal was tried and undone:
// it resolves the same displacement, but the normal it reports is a blend of two
// surfaces belonging to neither, which is precisely what leaves a caller unable to
// tell the floor it stands on from the wall pressing into it.
//
// `in_solid`, when given, receives the contents test this opens with — the same
// question callers otherwise ask again straight afterwards, at a full root-to-leaf
// descent per traceable per trace.
inline bool hull_push_out(const hull &h, int root, const double p[3], int blocking,
                          const double extent[3], double normal[3], double &depth,
                          bool *in_solid = nullptr) {
	const bool solid =
		(blocking & blocking_bit(hull_point_contents_biased(h, root, p, -STUCK_SLOP))) != 0;
	if (in_solid) *in_solid = solid;
	if (!solid)
		return false;

	double best = 0.0;
	double best_dir[3] = { 0, 0, 0 };
	bool found = false;
	auto consider = [&](const double dir[3], double limit) {
		const double d = hull_inside_distance(h, root, p, dir, limit, blocking);
		if (d < 0.0) return;
		for (int i = 0; i < 3; ++i) {
			const double mag = dir[i] < 0 ? -dir[i] : dir[i];
			if (mag * d > extent[i]) return;  // overruns the mover's own size on this axis
		}
		if (!found || d < best) {
			best = d;
			best_dir[0] = dir[0];
			best_dir[1] = dir[1];
			best_dir[2] = dir[2];
			found = true;
		}
	};

	static const double AXES[6][3] = {
		{ 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
	};
	for (int i = 0; i < 6; ++i) consider(AXES[i], extent[i / 2]);

	if (found) {
		normal[0] = best_dir[0];
		normal[1] = best_dir[1];
		normal[2] = best_dir[2];
		depth = best;
		return true;
	}
	return hull_nearest_leaf_plane(h, root, p, blocking, normal, depth);
}

// One map's stripped BSP, owned once and shared by every traceable built from it.
// A map has hundreds of brush entities and the blob runs to megabytes, so this is
// never copied per body. hop owns the bytes — it never points into another
// extension's parse, which could be freed underneath it.
struct map_data {
	std::vector<uint8_t> bytes;
	hop_bsp::BlobView view;

	bool load(const uint8_t *blob, size_t size) {
		bytes.assign(blob, blob + size);
		if (!hop_bsp::parse_blob(bytes.data(), bytes.size(), view)) { bytes.clear(); return false; }
		return true;
	}
	bool valid() const { return view.valid(); }
};

// True when a trace stopped on the far side of a sky brush — it left the world
// through the sky and struck the void beyond, which is CONTENTS_SOLID like
// everything outside the map. The endpoint sits DIST_EPSILON short of that plane,
// i.e. still inside the sky volume, so its contents is what tells us.
//
// GoldSrc removes a projectile that reaches sky rather than detonating it, which
// is the same statement as "the world does not block you on the way out". Only
// meaningful for a hull that treats sky as passable: for a view that blocks sky
// (the layer-9 body) this is never true, and hulls 1..3 have no sky at all — the
// compiler bakes it solid there, so players still collide with it.
inline bool stopped_against_sky(const hull &h, const hull_trace &tr, int blocking) {
	if (!tr.hit || (blocking & blocking_bit(hop_bsp::CONTENTS_SKY)) != 0) return false;
	return hull_point_contents(h, h.root, tr.endpos) == hop_bsp::CONTENTS_SKY;
}

} // namespace hopbsp

// ---------------------------------------------------------------------------

template <typename T>
class HopBspTraceable : public hop::traceable<T> {
	using tr = hop::scalar_traits<T>;

public:
	// `scale` is the importer's GoldSrc-units → metres factor.
	bool build(std::shared_ptr<hopbsp::map_data> map, int model_index, T scale,
	           int blocking = hopbsp::BLOCK_SOLID) {
		map_ = std::move(map);
		blocking_ = blocking;
		scale_ = (double)scale;
		inv_scale_ = scale_ != 0.0 ? 1.0 / scale_ : 0.0;
		if (!map_ || !map_->valid()) { map_.reset(); return false; }
		view_ = map_->view;
		if (model_index < 0 || model_index >= view_.model_count) { map_.reset(); return false; }
		model_ = &view_.models[model_index];

		for (int i = 0; i < 4; ++i) {
			hulls_[i].planes = view_.planes;
			hulls_[i].leafs = view_.leafs;
			hulls_[i].leaf_count = view_.leaf_count;
			hulls_[i].root = model_->headnode[i];
			if (i == 0) {
				hulls_[i].nodes = view_.nodes;
				hulls_[i].node_count = view_.node_count;
			} else {
				hulls_[i].clipnodes = view_.clipnodes;
				hulls_[i].node_count = view_.clipnode_count;
			}
		}

		// Godot-space bound from the model's GoldSrc box, grown by the largest hull
		// so a mover reduced to a point is still inside the bound where it matters.
		hop::vec3<T> a = gs_to_godot(model_->mins[0] - 32, model_->mins[1] - 32, model_->mins[2] - 36);
		hop::vec3<T> b = gs_to_godot(model_->maxs[0] + 32, model_->maxs[1] + 32, model_->maxs[2] + 36);
		bound_.set(a, a);
		bound_.merge(b);
		return true;
	}

	// Convenience for callers holding raw bytes (tests): loads a private copy.
	bool build(const uint8_t *blob, size_t size, int model_index, T scale,
	           int blocking = hopbsp::BLOCK_SOLID) {
		auto map = std::make_shared<hopbsp::map_data>();
		if (!map->load(blob, size)) return false;
		return build(std::move(map), model_index, scale, blocking);
	}

	void get_bound(hop::aa_box<T> &result) override { result = bound_; }

	void trace_segment(hop::collision<T> &result,
	                   const hop::vec3<T> &position,
	                   const hop::mat3<T> &orientation,
	                   const hop::segment<T> &seg) override {
		if (model_ == nullptr) return;
		hop::vec3<T> lo, ld;
		to_local(seg.origin, seg.direction, position, orientation, lo, ld);

		double s[3], e[3];
		godot_to_gs(lo, s);
		hop::vec3<T> lend;
		hop::add(lend, lo, ld);
		godot_to_gs(lend, e);

		// A ray is a point mover: hull 0, no offset.
		hopbsp::hull_trace ht = hopbsp::hull_sweep(hulls_[0], s, e, blocking_);
		if (!ht.hit || (T)ht.fraction >= result.time) return;
		if (hopbsp::stopped_against_sky(hulls_[0], ht, blocking_)) return;

		result.time = (T)ht.fraction;
		hop::vec3<T> hit_local = gs_to_godot(ht.endpos[0], ht.endpos[1], ht.endpos[2]);
		hop::vec3<T> n_local = gs_dir_to_godot(ht.normal[0], ht.normal[1], ht.normal[2]);
		to_world(hit_local, n_local, position, orientation, result.point, result.normal);
		result.impact = result.point;
	}

	void trace_solid(hop::collision<T> &result,
	                 hop::solid<T> *s,
	                 const hop::vec3<T> &position,
	                 const hop::mat3<T> &orientation,
	                 const hop::segment<T> &seg, T margin) override {
		if (model_ == nullptr) return;

		// The mover's own box, in GoldSrc units, picks the hull. Its offset is what
		// turns the mover into a point: GoldSrc traces (origin + clip_mins - mins).
		double mins[3], maxs[3];
		solid_box_gs(s, mins, maxs);

		// The mover's own axes in this model's frame, for a body that turns under
		// torque. Everything else — players, doors, projectiles — gets the identity
		// and the axis-aligned path below, bit for bit as before.
		const hopbsp::mover_basis basis = mover_basis_gs(s, orientation);

		int hi = hopbsp::hull_for_size(maxs[0] - mins[0], maxs[2] - mins[2]);
		// Hulls 1..3 are trees the map compiler already expanded for one fixed box, and
		// that baked expansion cannot be turned — which is why a tipped plate collides
		// fat on its thin axis in GoldSrc too. Hull 0 is the point hull, where the
		// expansion is still ours to compute per plane as the walk descends, so a mover
		// that turns is traced there whatever its size. It costs CLIP brushes (they
		// exist only in 1..3), which is the right trade for debris and the reason this
		// is gated on rotates_dynamically() rather than on having an orientation.
		if (basis.oriented) hi = 0;
		const hopbsp::hull &h = hulls_[hi];
		if (!h.valid()) return;
		// The box this trace is effectively expanded around, and how the mover maps
		// onto it. Decided once, here, because four things downstream depend on it —
		// the trace point, the inflation, the eject budget, and the witness walk-back
		// — and when each decided for itself they did not all agree.
		//
		//   offset   add to the mover's origin to get the point that is traced
		//   box_*    the box the hull behaves as if it were expanded for
		//   half     the mover's own half-extents, which expand the hull as it is walked
		double offset[3], box_mins[3], box_maxs[3];
		double half[3] = { 0, 0, 0 };

		if (hi == 0) {
			// Hull 0 is the point hull, and GoldSrc's low-corner alignment is a poor
			// deal there: it puts the trace point on a CORNER of the mover's box, so
			// the same body stops its full width short of one face of a wall and clips
			// through the other. A hull-0 mover is traced from its box CENTRE instead,
			// and the tree expanded by the mover's own half-extents as the walk
			// descends it (sweep_skin). Expansion is symmetric by construction; a
			// translation never can be.
			//
			// Per-plane, so a box that is not a cube is exact rather than approximated
			// by one radius: a corpse capsule is 24 units wide and 40 tall, and a single
			// inscribed radius left it resting 8 units inside the floor on the axis it
			// under-covered — while the overlap probe measured the same body against a
			// different surface and shoved it back out, over and over. That disagreement
			// is what made corpses bounce.
			for (int i = 0; i < 3; ++i) {
				offset[i] = (mins[i] + maxs[i]) * 0.5;
				half[i] = (maxs[i] - mins[i]) * 0.5;
			}
			// The tree contributes nothing, so the effective box is the mover's own,
			// recentred on the traced point.
			for (int i = 0; i < 3; ++i) {
				box_mins[i] = mins[i] - offset[i];
				box_maxs[i] = maxs[i] - offset[i];
			}
		} else {
			// The three sized hulls are boxes a mover is meant to match, and there
			// GoldSrc's own rule is right: trace (origin + mins - clip_mins), aligning
			// the two low corners. Where the boxes match that is identical to centring.
			//
			// The offset is ZERO for a centre-origin box, which is what makes it easy
			// to get backwards and never notice — GoldSrc's own player is centre-origin.
			// Godot bodies are not: WizardWars offsets the player's collider upward so
			// the origin is at the feet, and with the sign flipped the trace point lands
			// a full half-height BELOW the feet, i.e. inside the floor, every frame.
			for (int i = 0; i < 3; ++i) {
				box_mins[i] = hopbsp::HULL_SIZES[hi].mins[i];
				box_maxs[i] = hopbsp::HULL_SIZES[hi].maxs[i];
				offset[i] = mins[i] - box_mins[i];
			}
		}

		// The offset is a WORLD-space translation of the mover, so on a rotated model
		// it has to arrive in the model's frame rotated — see rotate_offset_to_local.
		rotate_offset_to_local(orientation, offset);

		hop::vec3<T> lo, ld;
		to_local(seg.origin, seg.direction, position, orientation, lo, ld);

		double start[3], end[3];
		godot_to_gs(lo, start);
		hop::vec3<T> lend;
		hop::add(lend, lo, ld);
		godot_to_gs(lend, end);
		for (int i = 0; i < 3; ++i) { start[i] += offset[i]; end[i] += offset[i]; }

		// The speculative contact margin, in GoldSrc units. It widens what counts as a
		// contact; the mover's own size is `half`, and the two are added wherever a
		// distance is measured from the traced point (the overlap probes below).
		const double margin_gs = (double)margin * inv_scale_;

		const bool zero_dir = hop::length_squared(ld) == T {};

		// A mover that STARTS inside solid is reported as an overlap, whatever its
		// direction. Quake instead reports startsolid and leaves the trace clear,
		// because its callers check that flag and refuse the move — hop's caller has
		// no such flag, so "clear" reads as "the whole path is free" and the mover
		// sails through the wall it is embedded in. Standing on a floor and sinking
		// a hair into the expanded hull is enough to start it: each frame gravity
		// sweeps freely, sinking it deeper, until it drops out of the world. That is
		// exactly what happened to bots on ww_2fort's ramps.
		//
		// Resting exactly ON the surface is not inside it (contents test is d < 0),
		// so this costs the normal standing case nothing. hull_push_out works against
		// the bare hull, no margin: only REAL penetration counts as stuck, where
		// testing the inflated hull would call a mover merely within `margin` of a
		// wall stuck and stall it dead.
		//
		// The mover's own box size per axis. It bounds how far one query may eject a
		// mover: past its own size a body is not overlapping a surface, it is buried,
		// and the honest response is to walk it out over a few frames rather than
		// launch it. hull_push_out searches within it; the cap below also covers the
		// leaf-plane fallback, which has no such bound of its own.
		//
		// Being buried is reachable in play: CLIP brushes exist only in hulls 1..3, so
		// a spot a player stands on quite happily under the trimesh colliders can be
		// solid here, and everyone inside one gets ejected the moment hulls come on.
		// Launching them is how you end up through a wall.
		//
		// Hull 0's box is a point, which would give a hull-0 mover no eject budget at
		// all. Its own box is the honest bound there, and it is the box the trace is
		// now centred on.
		// A turned box reaches further along a hull axis than its own half-extent on
		// that axis, so the budget is its world-aligned bound, not its local one.
		double extent[3];
		{
			double wh[3];
			hopbsp::box_world_half(half, basis, wh);
			for (int i = 0; i < 3; ++i)
				extent[i] = basis.oriented ? 2.0 * wh[i] : box_maxs[i] - box_mins[i];
		}

		double n[3], depth = 0;
		bool start_in_solid = false;
		const bool inside = hopbsp::hull_push_out(h, h.root, start, blocking_, extent, n, depth,
		                                          &start_in_solid);

		// hull_push_out answers "is there a way out of here", NOT "is the mover in
		// solid" — it returns false both for open space and for a point buried where no
		// probe found an exit, and reading the second as the first hands a deeply
		// embedded mover a clear sweep. The contents test answers the actual question,
		// with the same STUCK_SLOP band, so a mover resting ON a surface is not "in" it.
		const bool point_solid = inside || start_in_solid;

		// `inside` with zero depth means the mover is exactly ON a surface, not in it —
		// the ordinary state of standing on a floor. Reporting that as an overlap returns
		// without sweeping, leaving the caller with "t == 0" and one normal to guess
		// from; guessing frees the whole move whenever the motion does not drive into
		// THAT normal, including the part driving into a surface nobody looked at.
		//
		// So for a swept query only a real penetration is an overlap; merely touching
		// falls through to the sweep, which answers what was asked. Recovery is
		// unaffected — it probes with a zero-length segment, where the contact report is
		// the whole point. "Zero" means within STUCK_SLOP, the stuck test's own band.
		const bool touching_only = inside && !zero_dir && depth <= hopbsp::STUCK_SLOP;

		if ((point_solid && !touching_only) || zero_dir) {
			if (inside) {
				double cap = 0;
				for (int i = 0; i < 3; ++i)
					cap += (n[i] < 0 ? -n[i] : n[i]) * extent[i];
				// How far the box reaches past its centre along n, turned as it is.
				const double reach = hopbsp::box_support(n, half, basis);
				if (cap > 0 && depth > cap) depth = cap;
				// hull_push_out measured the traced POINT out of the solid; the body
				// around it still has to clear the surface, so add its own reach.
				depth += margin_gs + reach;
			} else if (point_solid) {
				// In solid, with no way out that the push-out could find. Report the
				// overlap with no direction and no depth: "you are inside something and
				// I cannot tell you which way out" is a different answer from "nothing
				// is there", and the caller has to be able to tell them apart. It reads
				// the zero normal and creeps rather than committing to a step it has no
				// information about.
				n[0] = n[1] = n[2] = 0;
				depth = 0;
			} else {
				// Not inside, but a surface the BODY reaches — its own half-extent on
				// that axis, plus `margin` — still counts as a contact. Sweeping a probe
				// along every candidate normal is overkill, so use the six axes and take
				// the deepest. This is the contact a resting body has, so a hull-0 mover
				// asks for it even at zero margin: its box has reach of its own.
				if (margin_gs <= 0 && half[0] == 0 && half[1] == 0 && half[2] == 0) return;
				if (!nearest_surface(h, start, margin_gs, half, basis, n, depth)) return;
				if (depth <= 0) return;
			}
			if (T {} >= result.time) return;
			result.time = T {};
			result.depth = (T)(depth * scale_);
			hop::vec3<T> n_local = gs_dir_to_godot(n[0], n[1], n[2]);
			hop::vec3<T> p_local = gs_to_godot(start[0] - offset[0], start[1] - offset[1], start[2] - offset[2]);
			to_world(p_local, n_local, position, orientation, result.point, result.normal);
			// This is the contact a body at REST holds, so it is the one that decides
			// whether debris left on an edge tips onto a face. `start` is the traced
			// point, which for hull 0 is the box centre; the witness is the feature of
			// the box that actually touches. A mover with no orientation of its own
			// keeps the centre it has always reported.
			result.impact = result.point;
			if (basis.oriented && (n[0] != 0 || n[1] != 0 || n[2] != 0)) {
				double sp[3];
				hopbsp::box_support_point(n, half, basis, sp);
				hop::vec3<T> impact_local =
					gs_to_godot(start[0] + sp[0], start[1] + sp[1], start[2] + sp[2]);
				hop::vec3<T> ignored;
				to_world(impact_local, n_local, position, orientation, result.impact, ignored);
			}
			return;
		}

		hopbsp::hull_trace ht = hopbsp::hull_sweep_box(h, start, end, blocking_, half, margin_gs, basis);

		// `margin_gs` inflates the hull, so a mover merely RESTING against a wall
		// starts the sweep inside the inflated solid even though it is not touching
		// the real surface. Quake's walk answers a trace that begins in solid with
		// allsolid and no surface at all — and !hit reads here as "nothing in the
		// way", which hands back the whole motion. Being flush against a wall is the
		// most ordinary state in the game, and it silently disabled that wall.
		//
		// It went unnoticed because a mover normally leans on ONE surface and the
		// motion that matters is tangential to it. Wedge one into a corner and the
		// wall it is flush with disables the wall it is walking into: a rider pressed
		// into the back corner of ww_golem's cockpit sat within the margin of the side
		// wall, so the sweep straight through the REAR wall came back clear and they
		// walked out of the golem.
		//
		// Re-run against the bare hull when that happens. The start is genuinely
		// outside the real geometry (hull_push_out already established it is not
		// embedded), so this is an ordinary empty-to-solid trace and it stops on the
		// surface the mover is actually driving into.
		if (!ht.hit && ht.allsolid && margin_gs > 0)
			ht = hopbsp::hull_sweep_box(h, start, end, blocking_, half, 0.0, basis);

		// Still no answer: the start is EXACTLY on the surface it is driving into.
		// The hull walk calls a start on the plane solid and gives up, so a mover
		// sitting flush on a face and pushing into it is told the whole path is clear —
		// and it walks straight through. This is not a rare pose: it is precisely where
		// depenetration leaves a body. On ww_golem the recovery lifted a rider out of
		// the cockpit's side wall onto its face, and the same tick's sweep then handed
		// back the full step through 15 cm of wall, out of the golem.
		//
		// Back the start off along the motion by DIST_EPSILON and trace again — the
		// same nudge Quake keeps every trace endpoint off surfaces by, and for the same
		// reason. From strictly outside it is an ordinary empty-to-solid trace that
		// stops on the face. A mover flush against a surface and moving AWAY needs no
		// special case: backing off puts the start inside the solid it is leaving, the
		// walk gives up again, and the move stays free, which is correct.
		if (!ht.hit && ht.allsolid) {
			hopbsp::hull_trace nudged = hopbsp::hull_sweep_off_surface(h, start, end, blocking_, half, basis);
			if (nudged.hit) ht = nudged;
		}

		// Still nothing, because the solid the start is in is not the one the motion
		// drives into: backing off ALONG THE MOTION only clears a surface ahead, so a
		// mover touching a SIDE wall while travelling parallel to it is still inside.
		//
		// Underneath, two tests disagree about the same point. The stuck test and the
		// push-out measure against a hull shrunk by STUCK_SLOP, so a mover sub-slop
		// inside a face is deliberately "not in solid" to them — that is what stops
		// resting on a surface reading as buried in it. The sweep uses no such band, so
		// the identical point is allsolid to it and it gives up without looking ahead.
		// Neither answer is wrong alone; holding both is.
		//
		// So ask the sweep against the same shrunk hull. Costs STUCK_SLOP of accuracy on
		// where it stops (a quarter of a millimetre) against handing back the whole
		// motion. Only reachable in the band (0, STUCK_SLOP): deeper and the overlap is
		// reported above, shallower and the start is not in solid at all — which is
		// where traces routinely leave bodies, DIST_EPSILON being twice it.
		if (!ht.hit && ht.allsolid && !inside)
			ht = hopbsp::hull_sweep_stuck_band(h, start, end, blocking_, half, basis);

		if (!ht.hit || (T)ht.fraction >= result.time) return;
		if (hopbsp::stopped_against_sky(h, ht, blocking_)) return;

		result.time = (T)ht.fraction;
		result.depth = T {};
		// A sweep that yields NO motion means the mover is already up against this
		// surface, and a caller with a speculative margin reads `depth` as how far
		// inside the margin shell it sits — reporting zero tells hop's solver it has a
		// whole margin of room, every tick, so a resting body creeps into the floor
		// forever at margin-per-tick and is periodically shoved back out. (That creep
		// was the second half of the bouncing corpse.) Measure the real gap along the
		// normal we just hit — one probe, only on the touching path — and report it in
		// the same terms the overlap branch does. Zero margin (the public query) keeps
		// an exact depth of zero, since a mover that is merely touching is not inside.
		if (margin_gs > 0 && ht.fraction <= 0) {
			double reach = 0;
			for (int i = 0; i < 3; ++i)
				reach += (ht.normal[i] < 0 ? -ht.normal[i] : ht.normal[i]) * half[i];
			const double limit = reach + margin_gs;
			const double dir[3] = { -ht.normal[0], -ht.normal[1], -ht.normal[2] };
			const double gap = surface_gap(h, start, dir, limit);
			if (gap >= 0 && limit - gap > 0) result.depth = (T)((limit - gap) * scale_);
		}
		hop::vec3<T> n_local = gs_dir_to_godot(ht.normal[0], ht.normal[1], ht.normal[2]);
		// endpos is where the POINT stopped; undo the hull offset to get the mover's
		// origin, and place the witness point on the surface it stopped against.
		hop::vec3<T> p_local = gs_to_godot(ht.endpos[0] - offset[0], ht.endpos[1] - offset[1], ht.endpos[2] - offset[2]);
		to_world(p_local, n_local, position, orientation, result.point, result.normal);

		// The witness point: endpos sits on the EXPANDED surface, so walk back onto the
		// real geometry — straight DOWN THE NORMAL, by however far the box reaches that
		// way. The box is box_mins/box_maxs, the one this trace was actually expanded
		// around: the engine's hull box for a sized hull, the mover's own for hull 0.
		//
		// Which walk-back this is matters more than it looks, because the solver takes a
		// contact's lever arm as (impact − centre) and every contact is resolved about it.
		//
		//   * Leaving hull 0 out of the walk-back entirely (what this did) puts the point
		//     at the mover's CENTRE. Fine for a projectile, which only ever reads the
		//     contact as a position; useless for a tumbling rigid body, whose every
		//     contact then arrives with a zero arm and applies no torque at all. Gibs
		//     landed flat, slid, and never tipped, whatever they hit.
		//   * Taking the box's CORNER — its support point in −n, per axis — is exact for
		//     the plane the sweep stopped on, and catastrophic for a body resting on it:
		//     a floor normal says nothing about the two tangential axes, so the corner
		//     picks a lever arm ACROSS the face and the normal impulse holding the body
		//     up becomes a torque. A gib landing on a flat floor span up to 37 rad/s and
		//     kept gaining.
		//
		// Down the normal is the honest answer for a shape whose contact patch a hull
		// trace cannot resolve: the arm is parallel to the normal, so the support impulse
		// makes no torque, and friction across the face still tips the body as it should.
		//
		// result.impact also feeds velocity_at_local for moving-platform carry, where it
		// is the same improvement — a rider samples a turning platform's ω×r at the point
		// it stands on rather than at a corner of its own bounding box.
		double box_half[3];
		for (int i = 0; i < 3; ++i) box_half[i] = (box_maxs[i] - box_mins[i]) * 0.5;
		double w[3];
		if (basis.oriented) {
			// The feature of the turned box that met the surface. On a face contact
			// every axis ties and this reduces to the centre-down-the-normal point
			// below, which is what keeps a flat landing from torquing itself.
			double sp[3];
			hopbsp::box_support_point(ht.normal, box_half, basis, sp);
			for (int i = 0; i < 3; ++i) w[i] = ht.endpos[i] + sp[i];
		} else {
			// Axis-aligned mover: down the normal by the box's reach that way, exactly
			// as before. Players and doors are bit-identical through here.
			const double reach = hopbsp::box_support(ht.normal, box_half, basis);
			for (int i = 0; i < 3; ++i) w[i] = ht.endpos[i] - ht.normal[i] * reach;
		}
		hop::vec3<T> impact_local = gs_to_godot(w[0], w[1], w[2]);
		hop::vec3<T> ignored;
		to_world(impact_local, n_local, position, orientation, result.impact, ignored);
	}

	// Contents at a Godot-space point, for tests and callers that want the raw
	// CONTENTS_* value rather than a trace.
	int contents_at_godot(const hop::vec3<T> &p) const {
		double gs[3];
		godot_to_gs(p, gs);
		return hopbsp::hull_point_contents(hulls_[0], hulls_[0].root, gs);
	}

private:
	// GoldSrc (Z-up, inches) ↔ Godot (Y-up, metres). The axis swap is its own
	// inverse and has determinant +1, so directions use the same map both ways and
	// normals need no special handling.
	hop::vec3<T> gs_to_godot(double x, double y, double z) const {
		hop::vec3<T> v;
		v.set((T)(-x * scale_), (T)(z * scale_), (T)(y * scale_));
		return v;
	}
	hop::vec3<T> gs_dir_to_godot(double x, double y, double z) const {
		hop::vec3<T> v;
		v.set((T)(-x), (T)z, (T)y);
		return v;
	}
	void godot_to_gs(const hop::vec3<T> &v, double out[3]) const {
		out[0] = -(double)v.x * inv_scale_;
		out[1] = (double)v.z * inv_scale_;
		out[2] = (double)v.y * inv_scale_;
	}

	// The hull offset in the model's axes. GoldSrc applies it in WORLD space —
	// SV_ClipMoveToEntity builds its matrix at the offset and inverse-transforms the
	// world endpoints through it, giving local + R⁻¹(mins - clip_mins).
	//
	// Unrotated it sends a standing player's 36-unit "raise the point to my centre"
	// along whichever model axis is up in the FILE. Identical on an unrotated brush;
	// on a plate authored lying down and rotated upright to close (ww_monoliths' spawn
	// gates) the raise goes sideways through the door instead, stopping the player 72
	// units off one face and putting their eye through the other.
	//
	// Only the offset rotates. The tree's own expansion stays in model space, where the
	// compiler baked it — which is why a tipped plate collides fat on its thin axis in
	// GoldSrc too.
	void rotate_offset_to_local(const hop::mat3<T> &orientation, double offset[3]) const {
		static const hop::mat3<T> identity;
		if (orientation == identity) return;
		const hop::vec3<T> w = gs_dir_to_godot(offset[0], offset[1], offset[2]);
		hop::mat3<T> Rt;
		hop::transpose(Rt, orientation);
		hop::vec3<T> l;
		hop::mul(l, Rt, w);
		// gs_dir_to_godot's axis swap is its own inverse, so the way back is the same map.
		offset[0] = -(double)l.x;
		offset[1] = (double)l.z;
		offset[2] = (double)l.y;
	}

	void to_local(const hop::vec3<T> &origin, const hop::vec3<T> &dir,
	              const hop::vec3<T> &position, const hop::mat3<T> &orientation,
	              hop::vec3<T> &lo, hop::vec3<T> &ld) const {
		static const hop::mat3<T> identity;
		hop::vec3<T> rel;
		hop::sub(rel, origin, position);
		if (orientation != identity) {
			hop::mat3<T> Rt;
			hop::transpose(Rt, orientation);
			hop::mul(lo, Rt, rel);
			hop::mul(ld, Rt, dir);
		} else {
			lo = rel;
			ld = dir;
		}
	}

	void to_world(const hop::vec3<T> &p_local, const hop::vec3<T> &n_local,
	              const hop::vec3<T> &position, const hop::mat3<T> &orientation,
	              hop::vec3<T> &p_out, hop::vec3<T> &n_out) const {
		static const hop::mat3<T> identity;
		if (orientation != identity) {
			hop::mul(p_out, orientation, p_local);
			hop::add(p_out, position);
			hop::mul(n_out, orientation, n_local);
		} else {
			hop::add(p_out, p_local, position);
			n_out = n_local;
		}
	}

	// The mover's axes, expressed in this model's GoldSrc frame.
	//
	// `orientation` is the TRACEABLE's world rotation, so the mover's rotation relative
	// to the geometry is orientation^T . solid_orientation — the same conjugation
	// to_local does to the segment, applied to a basis rather than a point. Each column
	// then goes through the Y-up/Z-up axis swap, which is its own inverse and has
	// determinant +1, so it needs no special handling for a basis either.
	//
	// Gated on rotates_dynamically(): a body with no inertia never integrates an
	// orientation, so it gets the identity and every trace it makes is unchanged.
	hopbsp::mover_basis mover_basis_gs(hop::solid<T> *s, const hop::mat3<T> &orientation) const {
		hopbsp::mover_basis b;
		if (s == nullptr || !s->rotates_dynamically()) return b;

		hop::mat3<T> Rm = s->get_orientation();
		static const hop::mat3<T> identity;
		if (orientation != identity) {
			hop::mat3<T> Rt, rel;
			hop::transpose(Rt, orientation);
			hop::mul(rel, Rt, Rm);
			Rm = rel;
		}
		if (Rm == identity) return b;

		// box_support pairs column j with half[j], and `half` is indexed in GOLDSRC
		// axes — solid_box_gs already permuted it, so half[1] is the Godot-Z extent and
		// half[2] the Godot-Y one. The columns have to take the same permutation or
		// each body axis is measured with another axis's extent: a bone tipped on its
		// edge asked how far it reached downward and was told its thickness, which is
		// what it would have answered lying flat.
		//
		// Only the ORDER is permuted here. Each column's components go through the
		// direction swap, and its sign is irrelevant — box_support takes |n.m_j|.
		static const int COL[3] = { 0, 2, 1 };  // gs x,y,z <- Godot local x,z,y
		for (int j = 0; j < 3; ++j) {
			const int c = COL[j];
			b.axis[0][j] = -(double)Rm.at(0, c);
			b.axis[1][j] = (double)Rm.at(2, c);
			b.axis[2][j] = (double)Rm.at(1, c);
		}
		b.oriented = true;
		return b;
	}

	// The mover's own box, in GoldSrc units. hop maintains this union for us and
	// folds in each shape's local position/rotation, which a hand-rolled loop over
	// the intrinsic shape bounds would miss — and the box is what picks the hull.
	void solid_box_gs(hop::solid<T> *s, double mins[3], double maxs[3]) const {
		hop::aa_box<T> box;
		if (s != nullptr) box = s->get_local_bound();
		else { box.mins.set(T {}, T {}, T {}); box.maxs.set(T {}, T {}, T {}); }
		// Axis swap maps godot X to -gs X, so the box ends swap on that axis.
		double lo[3], hi[3];
		lo[0] = -(double)box.maxs.x * inv_scale_; hi[0] = -(double)box.mins.x * inv_scale_;
		lo[1] = (double)box.mins.z * inv_scale_;  hi[1] = (double)box.maxs.z * inv_scale_;
		lo[2] = (double)box.mins.y * inv_scale_;  hi[2] = (double)box.maxs.y * inv_scale_;
		for (int i = 0; i < 3; ++i) { mins[i] = lo[i]; maxs[i] = hi[i]; }
	}

	// Deepest blocking surface the mover's box reaches, by six axis probes: on each
	// axis it looks out `half` (the box's own reach that way) plus `margin`, and
	// reports how far past the surface that reaches. Only used for the speculative
	// (touching, not penetrating) contact case.
	//
	// Per-axis rather than one radius because the box need not be a cube — a 24x24x40
	// corpse capsule reaches 20 units down and 12 sideways, and measuring the floor
	// with the sideways number is what left it resting inside the floor.
	bool nearest_surface(const hopbsp::hull &h, const double p[3], double margin,
	                     const double half[3], const hopbsp::mover_basis &basis,
	                     double n[3], double &depth) const {
		static const double axes[6][3] = {
			{ 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 }
		};
		// The probes run along the HULL's axes, so what bounds each one is the turned
		// box's reach that way, not its own half-extent on a local axis.
		double wh[3];
		hopbsp::box_world_half(half, basis, wh);
		bool found = false;
		depth = 0.0;
		for (int ai = 0; ai < 6; ++ai) {
			const double limit = wh[ai / 2] + margin;
			if (limit <= 0) continue;
			double t_n[3];
			const double gap = surface_gap(h, p, axes[ai], limit, t_n);
			if (gap < 0) continue;
			const double d = limit - gap;  // how far past the surface the box reaches
			if (d > depth) {
				depth = d;
				n[0] = t_n[0]; n[1] = t_n[1]; n[2] = t_n[2];
				found = true;
			}
		}
		return found;
	}

	// Distance from `p` to the first blocking surface along `dir`, or -1 if none
	// within `limit`. `dir` is a unit GoldSrc-space direction; `out_n`, when given,
	// receives the surface normal.
	double surface_gap(const hopbsp::hull &h, const double p[3], const double dir[3],
	                   double limit, double *out_n = nullptr) const {
		const double e[3] = { p[0] + dir[0] * limit, p[1] + dir[1] * limit, p[2] + dir[2] * limit };
		hopbsp::hull_trace t = hopbsp::hull_sweep(h, p, e, blocking_);
		if (!t.hit && t.allsolid) {
			// `p` is exactly ON this surface. The hull walk calls a start that sits on
			// a plane solid and gives up with no surface at all, so the one contact a
			// resting mover definitely has — the floor under it — was the one contact
			// this probe could not see. Nothing then reported the mover as resting, so
			// nothing lifted it off the plane, and on ww_golem it sat on the seam where
			// the cockpit's rear wall meets its deck: inside neither, blocked by
			// neither, and out the back.
			//
			// Same remedy as the swept path: back off a hair along the probe and ask
			// again, which puts the start strictly outside and returns the real plane.
			// The distance is zero by construction — we started on it.
			t = hopbsp::hull_sweep_off_surface(h, p, e, blocking_);
		}
		if (!t.hit) return -1.0;
		if (out_n) { out_n[0] = t.normal[0]; out_n[1] = t.normal[1]; out_n[2] = t.normal[2]; }
		return t.fraction * limit;
	}

	std::shared_ptr<hopbsp::map_data> map_;  // keeps the shared bytes alive
	hop_bsp::BlobView view_;
	const hop_bsp::BSPModel *model_ = nullptr;
	hopbsp::hull hulls_[4];
	hop::aa_box<T> bound_;
	int blocking_ = hopbsp::BLOCK_SOLID;
	double scale_ = 1.0;
	double inv_scale_ = 1.0;
};
