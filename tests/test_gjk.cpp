// Direct unit tests for the GJK closest-point sweep (math/gjk.h) used by the
// rounded-shape × convex_solid collision pairs. These assert on the two things
// the GJK path exists to get right and the plane-inflation path gets wrong:
//   - the contact NORMAL at an edge/ledge (ride-up), and
//   - bounding (an unbounded/open convex must not phantom-block from afar).
// Run in float, fixed16 and fixed32 so the fixed-point build is covered too.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/collide.h> // gjk_core_support / trace_pair_convex_gjk live here
#include <hop/hop.h>

using namespace hop;

// --- helpers ---------------------------------------------------------------

template <typename T> static convex_solid<T> box_convex(T half) {
	using tr = scalar_traits<T>;
	convex_solid<T> cs;
	const T h = half;
	cs.planes.push_back(plane<T>(vec3<T>(tr::one(), T {}, T {}), h));
	cs.planes.push_back(plane<T>(vec3<T>(-tr::one(), T {}, T {}), h));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, tr::one(), T {}), h));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, -tr::one(), T {}), h));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, tr::one()), h));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, -tr::one()), h));
	return cs;
}

// Box convex with per-axis half-extents (planes at distance hx/hy/hz).
template <typename T> static convex_solid<T> box_convex_ext(T hx, T hy, T hz) {
	using tr = scalar_traits<T>;
	convex_solid<T> cs;
	cs.planes.push_back(plane<T>(vec3<T>(tr::one(), T {}, T {}), hx));
	cs.planes.push_back(plane<T>(vec3<T>(-tr::one(), T {}, T {}), hx));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, tr::one(), T {}), hy));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, -tr::one(), T {}), hy));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, tr::one()), hz));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, -tr::one()), hz));
	return cs;
}

// Sweep shape A (at base_a, moving `motion`) against static shape B (at base_b)
// with combined rounded radius `cr`, via the same support callables collide.h
// builds for the real collision path.
template <typename T>
static gjk_sweep_result<T> sweep(const shape<T> & a, vec3<T> base_a, vec3<T> motion,
                                 const shape<T> & b, vec3<T> base_b, T cr) {
	using tr = scalar_traits<T>;
	auto sa = [&](const vec3<T> & d, vec3<T> & o) { gjk_core_support(o, &a, base_a, d); };
	auto sb = [&](const vec3<T> & d, vec3<T> & o) { gjk_core_support(o, &b, base_b, d); };
	gjk_sweep_result<T> r;
	gjk_sweep<T>(r, sa, sb, motion, cr, tr::from_milli(1)); // 1mm, = float default epsilon
	return r;
}

template <typename T> static vec3<T> v3(float x, float y, float z) {
	using tr = scalar_traits<T>;
	// build from milli so fixed-point keeps fractional values
	return vec3<T>(tr::from_milli((int)lroundf(x * 1000)),
	               tr::from_milli((int)lroundf(y * 1000)),
	               tr::from_milli((int)lroundf(z * 1000)));
}

// --- tests -----------------------------------------------------------------

// Sphere dropped straight onto a box-convex top: rests with an upward normal at
// TOI ~0.5 (sphere center stops a radius above the y=1 face).
template <typename T> static void test_gjk_sphere_drop(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_sphere_drop[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one()));
	shape<T> sph(sphere<T>(v3<T>(0, 0, 0), tr::half()));
	auto r = sweep<T>(sph, v3<T>(0, 3, 0), v3<T>(0, -3, 0), box, v3<T>(0, 0, 0), tr::half());
	float ny = tr::to_float(r.normal.y);
	float t = tr::to_float(r.time);
	printf("hit=%d t=%.3f ny=%.3f ", r.hit, t, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.85f);          // ~ +Y
	assert(t > 0.4f && t < 0.6f); // ~ 0.5
	printf("OK\n");
}

// Capsule dropped onto the box top: same upward normal.
template <typename T> static void test_gjk_capsule_drop(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_capsule_drop[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one()));
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(cap, v3<T>(0, 3, 0), v3<T>(0, -3, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d ny=%.3f ", r.hit, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.85f);
	printf("OK\n");
}

// THE BOARDING FIX: a capsule whose lower cap is just below a ledge's top edge,
// moving horizontally into it, must get a normal with a real UPWARD component
// (so the slide rides it up) — not the pure-horizontal wall normal the
// plane-inflation path produced.
template <typename T> static void test_gjk_edge_rideup(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_edge_rideup[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one())); // top face at y=1, +x face at x=1
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	// lower cap centre at y=1.3 (bottom point y=0.9, below the y=1 top), to the
	// +x side, sweeping in -x toward the top edge at (1,1,0).
	auto r = sweep<T>(cap, v3<T>(2, 1.3f, 0), v3<T>(-3, 0, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	float nx = tr::to_float(r.normal.x);
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d n=(%.3f,%.3f) ", r.hit, nx, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.25f); // <-- the ride-up component the inflate path lacked
	assert(nx > 0.2f);  // also pushed back out along +x
	printf("OK\n");
}

// Moving away from the box: no contact.
template <typename T> static void test_gjk_miss(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_miss[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one()));
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(cap, v3<T>(0, 3, 0), v3<T>(0, 3, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	printf("hit=%d ", r.hit);
	assert(r.valid && !r.hit);
	printf("OK\n");
}

// BOUNDING: an unbounded convex (4 side planes + a top, OPEN bottom) must not
// phantom-block a capsule sitting well below it. The plane-inflation path treats
// any point inside the footprint and below the top as "inside" (a false hit);
// GJK uses the finite vertex hull (the top quad), so the far-below capsule misses.
template <typename T> static void test_gjk_unbounded_no_phantom(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_unbounded_no_phantom[%s]: ", label);
	convex_solid<T> col;
	const T h = tr::half();
	col.planes.push_back(plane<T>(vec3<T>(tr::one(), T {}, T {}), h));
	col.planes.push_back(plane<T>(vec3<T>(-tr::one(), T {}, T {}), h));
	col.planes.push_back(plane<T>(vec3<T>(T {}, T {}, tr::one()), h));
	col.planes.push_back(plane<T>(vec3<T>(T {}, T {}, -tr::one()), h));
	col.planes.push_back(plane<T>(vec3<T>(T {}, tr::one(), T {}), tr::one())); // top y<=1, no bottom
	shape<T> column(col);
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(cap, v3<T>(0, -5, 0), v3<T>(0.3f, 0, 0), column, v3<T>(0, 0, 0), tr::from_milli(400));
	printf("hit=%d ", r.hit);
	assert(r.valid && !r.hit);
	printf("OK\n");
}

// A genuine tall wall: walking into its side gives a horizontal block, NOT a
// spurious ride-up.
template <typename T> static void test_gjk_wall_side(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_wall_side[%s]: ", label);
	convex_solid<T> w;
	const T h = tr::half();
	w.planes.push_back(plane<T>(vec3<T>(tr::one(), T {}, T {}), h));
	w.planes.push_back(plane<T>(vec3<T>(-tr::one(), T {}, T {}), h));
	w.planes.push_back(plane<T>(vec3<T>(T {}, tr::one(), T {}), tr::from_int(5)));
	w.planes.push_back(plane<T>(vec3<T>(T {}, -tr::one(), T {}), tr::from_int(5)));
	w.planes.push_back(plane<T>(vec3<T>(T {}, T {}, tr::one()), h));
	w.planes.push_back(plane<T>(vec3<T>(T {}, T {}, -tr::one()), h));
	shape<T> wall(w);
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(cap, v3<T>(3, 0, 0), v3<T>(-3, 0, 0), wall, v3<T>(0, 0, 0), tr::from_milli(400));
	float nx = tr::to_float(r.normal.x);
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d n=(%.3f,%.3f) ", r.hit, nx, ny);
	assert(r.valid && r.hit);
	assert(nx > 0.85f);          // horizontal block
	assert(std::fabs(ny) < 0.2f); // no spurious vertical
	printf("OK\n");
}

// Mirror orientation: a convex_solid is the mover, a static capsule the target.
// Exercises gjk with the operands swapped; normal points from the capsule (B)
// toward the convex (A) = +Y when the convex drops on top.
template <typename T> static void test_gjk_convex_mover_vs_capsule(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_convex_mover_vs_capsule[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one()));
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(box, v3<T>(0, 5, 0), v3<T>(0, -4, 0), cap, v3<T>(0, 0, 0), tr::from_milli(400));
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d ny=%.3f ", r.hit, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.85f);
	printf("OK\n");
}

// Same as the convex drop, but against a box shape (sphere × box pair).
template <typename T> static void test_gjk_sphere_box_drop(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_sphere_box_drop[%s]: ", label);
	shape<T> box(aa_box<T>(-tr::one(), -tr::one(), -tr::one(), tr::one(), tr::one(), tr::one()));
	shape<T> sph(sphere<T>(v3<T>(0, 0, 0), tr::half()));
	auto r = sweep<T>(sph, v3<T>(0, 3, 0), v3<T>(0, -3, 0), box, v3<T>(0, 0, 0), tr::half());
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d ny=%.3f ", r.hit, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.85f);
	printf("OK\n");
}

// LARGE-BOX FIXED-POINT REGRESSION: a sphere resting (slightly overlapping) on the
// top face of a *large* box, off-axis in x. The CSO support points are then far
// from the origin (~box half-extent), so the simplex closest-point math forms
// degree-4 area products (and degree-6 face-side products in the tetra) that
// overflow fixed16's ±32768 range. Pre-fix, GJK diverged from its default x-axis
// warm-start and reported a bogus large distance with the wrong normal, so the
// resting contact was lost and the sphere tunnelled through the box top. The sweep
// here uses the same x-axis seed conservative_advance starts from, so it exercises
// exactly that path; the contact must be found with an upward normal.
template <typename T> static void test_gjk_large_box_contact(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_large_box_contact[%s]: ", label);
	const T h = tr::from_int(40); // 40-unit half-extents: far CSO support points
	shape<T> box(aa_box<T>(-h, -h, -h, h, h, h)); // top face at y=40
	shape<T> sph(sphere<T>(v3<T>(0, 0, 0), tr::half()));
	// Centre 0.45 above the top face (0.05 overlap for radius 0.5), off-axis in x.
	auto r = sweep<T>(sph, v3<T>(2.75f, 40.45f, 0), v3<T>(0, -0.1f, 0), box, v3<T>(0, 0, 0), tr::half());
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d valid=%d ny=%.3f depth=%.4f ", r.hit, r.valid, ny, tr::to_float(r.depth));
	assert(r.valid && r.hit);     // pre-fix: missed entirely (bogus large distance)
	assert(ny > 0.85f);           // upward contact normal, not a divergent garbage axis
	printf("OK\n");
}

// THE BOX EQUIVALENT OF THE BOARDING FIX: capsule mounting a box's top edge must
// get an upward normal. The old AABB-expansion path (trace_aa_box) returned only
// axis-aligned normals, so ride-up over a box edge was impossible.
template <typename T> static void test_gjk_capsule_box_edge_rideup(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_capsule_box_edge_rideup[%s]: ", label);
	shape<T> box(aa_box<T>(-tr::one(), -tr::one(), -tr::one(), tr::one(), tr::one(), tr::one()));
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	auto r = sweep<T>(cap, v3<T>(2, 1.3f, 0), v3<T>(-3, 0, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	float nx = tr::to_float(r.normal.x);
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d n=(%.3f,%.3f) ", r.hit, nx, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.25f); // <-- ride-up; trace_aa_box could only give ny == 0
	assert(nx > 0.2f);
	printf("OK\n");
}

// The narrowphase flag really switches behavior: at a box's top edge, GJK
// (use_gjk=true) yields an upward ride-up normal; the cheap AABB path
// (use_gjk=false) yields a flat, axis-aligned normal (no upward component).
// Driven through hop::test_solid so the actual dispatch is exercised.
template <typename T> static void test_gjk_flag_switches_path(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_flag_switches_path[%s]: ", label);

	auto box = std::make_shared<solid<T>>();
	box->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(-tr::one(), -tr::one(), -tr::one(), tr::one(), tr::one(), tr::one())));
	box->set_position(v3<T>(0, 0, 0));

	auto cap_solid = std::make_shared<solid<T>>();
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	cap_solid->add_shape(std::make_shared<shape<T>>(c));
	cap_solid->set_position(v3<T>(2, 1.3f, 0));

	segment<T> seg;
	seg.origin = v3<T>(2, 1.3f, 0);   // capsule (mover) start
	seg.direction = v3<T>(-3, 0, 0);  // sweep toward the box edge

	const T eps = tr::from_milli(1);

	collision<T> acc;
	acc.time = tr::one();
	hop::test_solid(acc, cap_solid.get(), seg, box.get(), eps, T {}, /*use_gjk=*/true);
	float acc_ny = tr::to_float(acc.normal.y);

	collision<T> cheap;
	cheap.time = tr::one();
	hop::test_solid(cheap, cap_solid.get(), seg, box.get(), eps, T {}, /*use_gjk=*/false);
	float cheap_ny = tr::to_float(cheap.normal.y);

	printf("acc_ny=%.3f cheap_ny=%.3f ", acc_ny, cheap_ny);
	assert(acc.time < tr::one() && cheap.time < tr::one()); // both contact
	assert(acc_ny > 0.25f);                                  // GJK rides up
	assert(std::fabs(cheap_ny) < 0.1f);                      // cheap path is flat
	printf("OK\n");
}

// ANALYTIC SPHERE×BOX, driven through hop::test_solid so the real dispatch (which
// routes sphere×box to the analytic clamp, not GJK) is exercised. A sphere resting
// just inside the top face of a *large* box, off BOTH lateral axes (the case the
// spin carry walks the contact into), must report an upward contact in fixed16.
// Pre-fix, GJK reconstructed the contact vector as a weighted average of the box's
// far vertices and lost it to cancellation: the distance came back ~0.19 too large
// with a sideways normal, so the contact was dropped and the rider tunnelled. The
// scaling fix in math/gjk.h covers the single-axis-offset case; this two-axis case
// needs the analytic path. Swept tangentially (the carry direction) to match the
// failing scenario.
template <typename T> static void test_gjk_large_box_offaxis_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_large_box_offaxis_solid[%s]: ", label);
	const T h = tr::from_int(40);
	auto box = std::make_shared<solid<T>>();
	box->add_shape(std::make_shared<shape<T>>(aa_box<T>(-h, -h, -h, h, h, h))); // top face at y=40
	box->set_position(v3<T>(0, 0, 0));

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(tr::one());
	sph->add_shape(std::make_shared<shape<T>>(sphere<T>(v3<T>(0, 0, 0), tr::half())));
	// Off-axis in BOTH x and z; centre 0.45 above the top (0.05 overlap for radius 0.5).
	sph->set_position(v3<T>(2.75f, 40.45f, 1.5f));

	segment<T> seg;
	seg.origin = v3<T>(2.75f, 40.45f, 1.5f);
	seg.direction = v3<T>(2, -0.1f, 0); // slide tangentially with a small downward sliver

	const T eps = tr::from_milli(1);
	collision<T> c;
	c.time = tr::one();
	hop::test_solid(c, sph.get(), seg, box.get(), eps, eps * tr::from_int(8), /*use_gjk=*/true);
	float ny = tr::to_float(c.normal.y);
	printf("time=%.3f ny=%.3f depth=%.4f ", tr::to_float(c.time), ny, tr::to_float(c.depth));
	assert(c.time <= T {});  // overlapping the inflated shell ⇒ contact at t==0
	assert(ny > 0.95f);      // clean upward normal (pre-fix fixed16: sideways, contact lost)
	printf("OK\n");
}

// The capsule analogue of test_gjk_large_box_offaxis_solid: a horizontal capsule
// resting just inside the top of a *large* box, off both lateral axes. Capsule×box
// routes through the analytic segment-vs-box closest point (alternating projection);
// like sphere×box it has to keep the resting contact under fixed16 where GJK lost it.
template <typename T> static void test_gjk_large_box_offaxis_capsule(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_large_box_offaxis_capsule[%s]: ", label);
	const T h = tr::from_int(40);
	auto box = std::make_shared<solid<T>>();
	box->add_shape(std::make_shared<shape<T>>(aa_box<T>(-h, -h, -h, h, h, h))); // top face at y=40
	box->set_position(v3<T>(0, 0, 0));

	auto cap_solid = std::make_shared<solid<T>>();
	cap_solid->set_mass(tr::one());
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(1, 0, 0), tr::from_milli(400)); // horizontal spine, radius 0.4
	cap_solid->add_shape(std::make_shared<shape<T>>(c));
	// Spine 0.35 above the top (0.05 overlap for radius 0.4), off both lateral axes.
	cap_solid->set_position(v3<T>(2.75f, 40.35f, 1.5f));

	segment<T> seg;
	seg.origin = v3<T>(2.75f, 40.35f, 1.5f);
	seg.direction = v3<T>(0, -0.1f, 2); // slide tangentially (+z) with a small downward sliver

	const T eps = tr::from_milli(1);
	collision<T> col;
	col.time = tr::one();
	hop::test_solid(col, cap_solid.get(), seg, box.get(), eps, eps * tr::from_int(8), /*use_gjk=*/true);
	float ny = tr::to_float(col.normal.y);
	printf("time=%.3f ny=%.3f depth=%.4f ", tr::to_float(col.time), ny, tr::to_float(col.depth));
	assert(col.time <= T {}); // contact at t==0
	assert(ny > 0.95f);       // upward normal (pre-fix fixed16: contact lost off-axis on a big box)
	printf("OK\n");
}

// THE "CAN'T GET OFF" REGRESSION: a capsule resting on a convex surface and
// moving horizontally (tangent to the contact) must NOT be blocked — otherwise
// it sticks at t==0 and can neither walk across the platform nor step off its
// edge. The contact is real, but the sweep isn't moving into it.
template <typename T> static void test_gjk_rest_tangential_free(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_rest_tangential_free[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one())); // top face at y=1
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	// capsule resting exactly on the top face (bottom point at y=1), moving +x.
	auto r = sweep<T>(cap, v3<T>(0, 1.4f, 0), v3<T>(3, 0, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	printf("hit=%d time=%.3f ", r.hit, tr::to_float(r.time));
	assert(r.valid && !r.hit); // free to slide along / off the surface
	printf("OK\n");
}

// Penetration must still be reported (with depth + normal) regardless of motion
// direction, so the caller's recovery can depenetrate. Here the capsule starts
// embedded in the box top and moves tangentially.
template <typename T> static void test_gjk_penetration_reports(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_penetration_reports[%s]: ", label);
	shape<T> box(box_convex<T>(tr::one())); // top face at y=1
	capsule<T> c;
	c.set(v3<T>(0, 0, 0), v3<T>(0, 1, 0), tr::from_milli(400));
	shape<T> cap(c);
	// lower cap centre at y=1.2: 0.2 above the top face, within the 0.4 radius,
	// so the rounded surface overlaps the box by ~0.2.
	auto r = sweep<T>(cap, v3<T>(0, 1.2f, 0), v3<T>(3, 0, 0), box, v3<T>(0, 0, 0), tr::from_milli(400));
	float ny = tr::to_float(r.normal.y);
	float depth = tr::to_float(r.depth);
	printf("hit=%d depth=%.3f ny=%.3f ", r.hit, depth, ny);
	assert(r.valid && r.hit);
	assert(depth > 0.1f); // ~0.2 overlap surfaced for recovery
	assert(ny > 0.85f);   // pushed out along +Y
	printf("OK\n");
}

// conservative_advance is a shared primitive: it owns the swept-contact
// semantics (penetration / block-only-when-approaching / undershoot) and takes
// any closest-distance functor. gjk_sweep feeds it a GJK simplex; the trimesh
// path feeds it an analytic segment-vs-triangle test. Drive it here with a
// hand-written analytic functor (distance to the plane y=0) to guard that
// contract independently of GJK.
template <typename T> static void test_ca_custom_closest(const char * label) {
	using tr = scalar_traits<T>;
	printf("  ca_custom_closest[%s]: ", label);
	// A radius-0.5 mover dropping from y=3 toward the plane y=0 along -Y.
	const T radius = tr::from_milli(500);
	gjk_sweep_result<T> r;
	conservative_advance<T>(r, v3<T>(0, -3, 0), radius, tr::from_milli(1),
	    [&](const vec3<T> & xA, const vec3<T> & /*seed*/, T & dist, vec3<T> & n, bool & deep) {
		    deep = false;
		    dist = tr::from_int(3) + xA.y; // height above the plane (start y=3)
		    n.reset();
		    n.y = tr::one(); // plane normal, toward the mover
	    });
	float t = tr::to_float(r.time);
	float ny = tr::to_float(r.normal.y);
	printf("hit=%d t=%.3f ny=%.3f ", r.hit, t, ny);
	assert(r.valid && r.hit);
	assert(ny > 0.99f);             // plane normal
	assert(t > 0.79f && t < 0.85f); // contact when height == 0.5 → t ≈ 0.833
	printf("OK\n");
}

// math/triangle.h primitives (relocated out of the hop-godot trimesh traceable):
// point-vs-triangle and segment-vs-triangle closest distance.
template <typename T> static void test_triangle_primitives(const char * label) {
	using tr = scalar_traits<T>;
	printf("  triangle_primitives[%s]: ", label);
	// Triangle in the y=0 plane.
	vec3<T> a = v3<T>(0, 0, 0), b = v3<T>(1, 0, 0), c = v3<T>(0, 0, 1);

	// Point above the interior projects straight down onto the face.
	vec3<T> cp;
	closest_point_triangle(cp, v3<T>(0.2f, 5, 0.2f), a, b, c);
	assert(std::fabs(tr::to_float(cp.x) - 0.2f) < 0.02f);
	assert(std::fabs(tr::to_float(cp.y)) < 0.02f);
	assert(std::fabs(tr::to_float(cp.z) - 0.2f) < 0.02f);

	// Point beyond vertex B resolves to B.
	closest_point_triangle(cp, v3<T>(3, 2, 0), a, b, c);
	assert(std::fabs(tr::to_float(cp.x) - 1.0f) < 0.02f && std::fabs(tr::to_float(cp.z)) < 0.02f);

	// Segment piercing the triangle interior → distance 0.
	vec3<T> cs, ct;
	T d2 = closest_segment_triangle(v3<T>(0.2f, 1, 0.2f), v3<T>(0.2f, -1, 0.2f),
	                                a, b, c, cs, ct, tr::from_milli(1));
	assert(tr::to_float(d2) < 0.01f);

	// Segment held 1 unit above the interior → squared distance 1, contact on face.
	d2 = closest_segment_triangle(v3<T>(0.2f, 1, 0.2f), v3<T>(0.3f, 1, 0.3f),
	                              a, b, c, cs, ct, tr::from_milli(1));
	printf("d2_above=%.3f ", tr::to_float(d2));
	assert(std::fabs(tr::to_float(d2) - 1.0f) < 0.05f);
	assert(std::fabs(tr::to_float(ct.y)) < 0.02f); // closest tri point is on the face
	printf("OK\n");
}

// Static rotation: a box solid rotated 90° about Y has its long (x) axis turned
// into z, so a sphere swept in -x stops nearer than the unrotated box. Verifies
// the orientation is honored AND that solid.orientation and shape.local_rotation
// compose to the same result.
template <typename T> static void test_gjk_solid_orientation(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_solid_orientation[%s]: ", label);
	const T one = tr::one();
	const T z {};
	const T neg = -one;
	// Ry(90°) = [[0,0,1],[0,1,0],[-1,0,0]] (row-major ctor args).
	mat3<T> Ry90(z, z, one, z, one, z, neg, z, z);

	auto sphere_mover = [&]() {
		auto s = std::make_shared<solid<T>>();
		s->add_shape(std::make_shared<shape<T>>(sphere<T>(v3<T>(0, 0, 0), tr::from_milli(500))));
		s->set_position(v3<T>(5, 0, 0));
		return s;
	};
	auto box_solid = [&](int mode) { // 0=none, 1=solid orientation, 2=shape rotation
		auto s = std::make_shared<solid<T>>();
		s->add_shape(std::make_shared<shape<T>>(box_convex_ext<T>(tr::from_int(2), one, one)));
		s->set_position(v3<T>(0, 0, 0));
		if (mode == 1) s->set_orientation(Ry90);
		else if (mode == 2) s->get_shapes()[0]->set_local_rotation(Ry90);
		return s;
	};
	segment<T> seg;
	seg.origin = v3<T>(5, 0, 0);
	seg.direction = v3<T>(-6, 0, 0);
	const T eps = tr::from_milli(1);
	auto run = [&](int mode) -> collision<T> {
		auto box = box_solid(mode);
		auto mv = sphere_mover();
		collision<T> r;
		r.time = one;
		hop::test_solid(r, mv.get(), seg, box.get(), eps, T {}, true);
		return r;
	};

	collision<T> unrot = run(0), solidR = run(1), shapeR = run(2);
	float t_unrot = tr::to_float(unrot.time), t_solid = tr::to_float(solidR.time), t_shape = tr::to_float(shapeR.time);
	printf("unrot=%.3f solidR=%.3f shapeR=%.3f n.x=%.2f ", t_unrot, t_solid, t_shape, tr::to_float(solidR.normal.x));
	assert(unrot.time < one && std::fabs(t_unrot - 0.4167f) < 0.04f);   // +x face at x=2 → center 2.5
	assert(solidR.time < one && std::fabs(t_solid - 0.5833f) < 0.04f);  // rotated: x-face at x=1 → center 1.5
	assert(tr::to_float(solidR.normal.x) > 0.9f);                       // outward +x
	assert(std::fabs(t_shape - t_solid) < 0.02f);                       // composition: shape == solid
	printf("OK\n");
}

// Oriented polytope×polytope (the Phase 5 CSO path). A unit-cube target rotated
// 45° about Z turns its +x face into a corner that reaches √2≈1.414 along x
// (vs 1.0 unrotated), so an axis-aligned box mover swept in -x must stop EARLIER
// against the rotated target than the unrotated one — proving orientation is
// honored rather than collapsed to the world AABB. Also checks box vs convex
// agreement and the rotated-mover impact (the oriented col.impact branch).
template <typename T> static void test_oriented_polytope(const char * label) {
	using tr = scalar_traits<T>;
	printf("  oriented_polytope[%s]: ", label);
	const T one = tr::one();
	const T z {};
	const T neg = -one;
	// Rz(45°): [[c,-s,0],[s,c,0],[0,0,1]] (row-major ctor args).
	const T c45 = tr::from_milli(707), s45 = tr::from_milli(707);
	mat3<T> Rz45(c45, -s45, z, s45, c45, z, z, z, one);
	(void)neg;
	const T eps = tr::from_milli(1);

	auto unit_box_shape = [&]() { return std::make_shared<shape<T>>(aa_box<T>(v3<T>(-1, -1, -1), v3<T>(1, 1, 1))); };
	auto unit_convex_shape = [&]() { return std::make_shared<shape<T>>(box_convex<T>(one)); };
	auto half_box_shape = [&]() { return std::make_shared<shape<T>>(aa_box<T>(v3<T>(-0.5f, -0.5f, -0.5f), v3<T>(0.5f, 0.5f, 0.5f))); };

	segment<T> seg;
	seg.origin = v3<T>(5, 0, 0);
	seg.direction = v3<T>(-6, 0, 0);

	// Mover = half-cube (axis-aligned) at x=5; target = unit cube at origin.
	auto run = [&](std::shared_ptr<shape<T>> mover_sh, std::shared_ptr<shape<T>> target_sh, bool rotate_target) -> collision<T> {
		auto mv = std::make_shared<solid<T>>();
		mv->add_shape(mover_sh);
		mv->set_position(v3<T>(5, 0, 0));
		auto tg = std::make_shared<solid<T>>();
		tg->add_shape(target_sh);
		tg->set_position(v3<T>(0, 0, 0));
		if (rotate_target)
			tg->set_orientation(Rz45);
		collision<T> r;
		r.time = one;
		hop::test_solid(r, mv.get(), seg, tg.get(), eps, T {}, true);
		return r;
	};

	// box×box: rotated target stops earlier than unrotated.
	collision<T> bb_unrot = run(half_box_shape(), unit_box_shape(), false);
	collision<T> bb_rot = run(half_box_shape(), unit_box_shape(), true);
	float t_un = tr::to_float(bb_unrot.time), t_rot = tr::to_float(bb_rot.time);
	printf("bb_un=%.3f bb_rot=%.3f n.x=%.2f ", t_un, t_rot, tr::to_float(bb_rot.normal.x));
	assert(std::fabs(t_un - 0.5833f) < 0.04f);  // face at x=1 → center 1.5
	assert(std::fabs(t_rot - 0.5143f) < 0.04f); // corner at x=√2 → center 1.914
	assert(t_rot < t_un - 0.04f);               // rotation genuinely changes the result
	assert(tr::to_float(bb_rot.normal.x) > 0.9f);

	// convex×convex and box×convex agree with box×box on the rotated target.
	collision<T> cc_rot = run(unit_convex_shape(), unit_convex_shape(), true); // mover is unit convex now
	collision<T> bc_rot = run(half_box_shape(), unit_convex_shape(), true);
	// cc uses a unit-cube mover (half 1), so its contact center is 1.414+1 = 2.414 → t=0.431.
	assert(std::fabs(tr::to_float(cc_rot.time) - 0.4310f) < 0.05f);
	assert(std::fabs(tr::to_float(bc_rot.time) - t_rot) < 0.04f); // box mover vs convex target == box×box

	// Rotated MOVER impact: unit cube rotated 45° about Z swept -x into an
	// axis-aligned unit box. The leading corner reaches -√2; contact at center
	// x=2.414, and col.impact must sit on that corner at x≈1.0 (= target +x face),
	// exercising the orientation-aware impact branch.
	{
		auto mv = std::make_shared<solid<T>>();
		mv->add_shape(unit_box_shape());
		mv->set_position(v3<T>(5, 0, 0));
		mv->set_orientation(Rz45);
		auto tg = std::make_shared<solid<T>>();
		tg->add_shape(unit_box_shape());
		collision<T> r;
		r.time = one;
		hop::test_solid(r, mv.get(), seg, tg.get(), eps, T {}, true);
		printf("rotmover t=%.3f impact.x=%.3f ", tr::to_float(r.time), tr::to_float(r.impact.x));
		assert(std::fabs(tr::to_float(r.time) - 0.4310f) < 0.05f);
		assert(std::fabs(tr::to_float(r.impact.x) - 1.0f) < 0.08f);
		assert(std::fabs(tr::to_float(r.impact.y)) < 0.08f);
	}
	printf("OK\n");
}

template <typename T> static void run_gjk_tests(const char * label) {
	printf(" [%s]\n", label);
	test_gjk_sphere_drop<T>(label);
	test_gjk_capsule_drop<T>(label);
	test_gjk_edge_rideup<T>(label);
	test_gjk_miss<T>(label);
	test_gjk_unbounded_no_phantom<T>(label);
	test_gjk_wall_side<T>(label);
	test_gjk_convex_mover_vs_capsule<T>(label);
	test_gjk_sphere_box_drop<T>(label);
	test_gjk_large_box_contact<T>(label);
	test_gjk_capsule_box_edge_rideup<T>(label);
	test_gjk_flag_switches_path<T>(label);
	test_gjk_large_box_offaxis_solid<T>(label);
	test_gjk_large_box_offaxis_capsule<T>(label);
	test_gjk_rest_tangential_free<T>(label);
	test_gjk_penetration_reports<T>(label);
	test_ca_custom_closest<T>(label);
	test_triangle_primitives<T>(label);
	test_gjk_solid_orientation<T>(label);
	test_oriented_polytope<T>(label);
}

int main() {
	printf("test_gjk:\n");
	run_gjk_tests<float>("float");
	run_gjk_tests<fixed16>("fixed16");
	run_gjk_tests<fixed32>("fixed32");
	printf("ALL PASSED\n");
	return 0;
}
