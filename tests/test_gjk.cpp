// Direct unit tests for the GJK closest-point sweep (math/gjk.h) used by the
// rounded-shape × convex_solid collision pairs. These assert on the two things
// the GJK path exists to get right and the plane-inflation path gets wrong:
//   - the contact NORMAL at an edge/ledge (ride-up), and
//   - bounding (an unbounded/open convex must not phantom-block from afar).
// Run in float, fixed16 and fixed32 so the fixed-point build is covered too.

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <type_traits>
#include <vector>
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

// Regular octahedron (dual of a cube): 8 octant face planes, vertices distance `r`
// from the center. Used to exercise the rounded×convex inflation fallback against an
// oriented convex.
template <typename T> static convex_solid<T> octahedron(T r) {
	using tr = scalar_traits<T>;
	convex_solid<T> cs;
	const T n = tr::one() / tr::sqrt(tr::from_int(3));
	const T d = r * n;
	for (int sx = -1; sx <= 1; sx += 2)
		for (int sy = -1; sy <= 1; sy += 2)
			for (int sz = -1; sz <= 1; sz += 2)
				cs.planes.push_back(plane<T>(vec3<T>(n * tr::from_int(sx), n * tr::from_int(sy), n * tr::from_int(sz)), d));
	return cs;
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

// --- issue #118 regressions -------------------------------------------------

// Reference distance between two segments: the distance from seg1(u) to seg2 is
// convex in u, so a ternary search (in double, with an exact point-to-segment
// projection onto seg2) converges to the true minimum.
static float ref_segment_distance(const float o1[3], const float d1[3], const float o2[3], const float d2[3]) {
	const double dd2 = double(d2[0]) * d2[0] + double(d2[1]) * d2[1] + double(d2[2]) * d2[2];
	auto f = [&](double u) {
		const double p[3] = {o1[0] + u * d1[0], o1[1] + u * d1[1], o1[2] + u * d1[2]};
		double t = dd2 > 0 ? ((p[0] - o2[0]) * d2[0] + (p[1] - o2[1]) * d2[1] + (p[2] - o2[2]) * d2[2]) / dd2 : 0;
		t = t < 0 ? 0 : (t > 1 ? 1 : t);
		const double q[3] = {o2[0] + t * d2[0] - p[0], o2[1] + t * d2[1] - p[1], o2[2] + t * d2[2] - p[2]};
		return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
	};
	double lo = 0, hi = 1;
	for (int i = 0; i < 200; ++i) {
		const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
		if (f(m1) < f(m2)) hi = m2; else lo = m1;
	}
	return static_cast<float>(std::min(f(0.5 * (lo + hi)), std::min(f(0.0), f(1.0))));
}

template <typename T> static float seg_seg_distance(const float o1[3], const float d1[3], const float o2[3],
                                                    const float d2[3]) {
	using tr = scalar_traits<T>;
	segment<T> s1, s2;
	s1.origin = v3<T>(o1[0], o1[1], o1[2]);
	s1.direction = v3<T>(d1[0], d1[1], d1[2]);
	s2.origin = v3<T>(o2[0], o2[1], o2[2]);
	s2.direction = v3<T>(d2[0], d2[1], d2[2]);
	vec3<T> p1, p2, diff;
	project(p1, p2, s1, s2, tr::from_milli(1));
	sub(diff, p1, p2);
	const float dist = tr::to_float(length(diff));
	assert(std::isfinite(dist));
	return dist;
}

// Segment-segment closest pair (math/project.h). The parallel test used to be an
// absolute denom < 1e-3, and denom scales with length⁴, so short segments up to
// ~16° apart took the "parallel" path and reported a pair that isn't the closest.
template <typename T> static void test_segment_segment_closest(const char * label) {
	printf("  segment_segment_closest[%s]: ", label);
	// fixed16's 16.16 format limits the solve to ~mm; float and fixed32 are exact.
	const float tol = std::is_same<T, fixed16>::value ? 1e-2f : 1e-3f;

	// The issue's case: a 0.37 crouched-capsule spine crossed at 15° by a 0.3 edge.
	// They intersect (distance 0); the old code reported ~0.039.
	{
		const float th = 15.0f * 3.14159265f / 180.0f, cx = cosf(th), sy = sinf(th);
		const float o1[3] = {0, 0, 0}, d1[3] = {0.37f, 0, 0};
		const float o2[3] = {0.3f - 0.15f * cx, -0.15f * sy, 0}, d2[3] = {0.3f * cx, 0.3f * sy, 0};
		const float dist = seg_seg_distance<T>(o1, d1, o2, d2);
		printf("cross15=%.4f ", dist);
		assert(dist < tol);
	}

	// Long spine vs a short per-tick motion segment (trace_capsule's shape of call).
	{
		const float o1[3] = {0, 0, 0}, d1[3] = {1, 0, 0};
		const float o2[3] = {0.4f, 0.1f, 0.0f}, d2[3] = {0.05f * 0.96f, -0.05f * 0.28f, 0};
		const float dist = seg_seg_distance<T>(o1, d1, o2, d2);
		const float ref = ref_segment_distance(o1, d1, o2, d2);
		printf("spine_vs_motion=%.4f(ref %.4f) ", dist, ref);
		assert(std::fabs(dist - ref) < tol);
	}

	// A segment shorter than √epsilon must not collapse to its origin (a and c are
	// squared lengths; they used to be compared against epsilon, not epsilon²).
	{
		const float o1[3] = {-0.015f, 0, 0}, d1[3] = {0.03f, 0, 0};
		const float o2[3] = {0, 0, -0.5f}, d2[3] = {0, 0, 1};
		const float dist = seg_seg_distance<T>(o1, d1, o2, d2);
		printf("short=%.4f ", dist);
		assert(dist < tol); // they cross; the old code reported 0.015
	}

	// Exactly parallel: overlapping (distance = offset) and end-to-end (gap).
	{
		const float o1[3] = {0, 0, 0}, d1[3] = {1, 0, 0};
		const float o2[3] = {0.25f, 0.1f, 0}, d2[3] = {1, 0, 0};
		assert(std::fabs(seg_seg_distance<T>(o1, d1, o2, d2) - 0.1f) < tol);
		const float o3[3] = {1.5f, 0, 0};
		assert(std::fabs(seg_seg_distance<T>(o1, d1, o3, d1) - 0.5f) < tol);
	}

	// Seeded sweep of near-parallel pairs at short-to-medium lengths.
	unsigned seed = 12345u;
	auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return ((seed >> 8) & 0xFFFF) / 32767.5f - 1.0f; };
	float worst = 0;
	for (float L : {0.1f, 0.2f, 0.4f, 1.0f}) {
		for (int k = 0; k < 300; ++k) {
			float d1[3] = {rnd(), rnd(), rnd()};
			float n1 = sqrtf(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]) + 1e-6f;
			float d2[3];
			for (int i = 0; i < 3; ++i) {
				d1[i] = d1[i] / n1 * L;
				d2[i] = d1[i] * 0.8f + rnd() * 0.15f * L;
			}
			const float o1[3] = {lroundf(rnd() * 500) / 1000.0f, lroundf(rnd() * 500) / 1000.0f, lroundf(rnd() * 500) / 1000.0f};
			float o2[3] = {o1[0] + rnd() * 0.3f * L, o1[1] + rnd() * 0.3f * L, o1[2] + rnd() * 0.3f * L};
			// v3<T> builds from milli units, so quantize first: both sides then see the same segments.
			for (int i = 0; i < 3; ++i) {
				d1[i] = lroundf(d1[i] * 1000) / 1000.0f;
				d2[i] = lroundf(d2[i] * 1000) / 1000.0f;
				o2[i] = lroundf(o2[i] * 1000) / 1000.0f;
			}
			const float err = seg_seg_distance<T>(o1, d1, o2, d2) - ref_segment_distance(o1, d1, o2, d2);
			worst = std::max(worst, std::fabs(err));
		}
	}
	printf("sweep_worst=%.5f ", worst);
	assert(worst < tol);

	// Unbalanced inputs: one input far larger than the others. Scaling every input by
	// the largest (offset included) shrank the short ones into fixed16's rounding: a
	// 0.3 pair 10 apart was off by ~29 cm and a 0.6 pair 50 apart by ~59 cm, both fine
	// on main. The first case is trace_capsule's call for a short spine and a fast move.
	{
		struct regime { const char * name; float l1, l2, sep; };
		const regime regimes[] = { {"spine_vs_long_sweep", 0.1f, 10.0f, 0.5f}, {"short_far", 0.3f, 0.3f, 10.0f},
			                       {"short_very_far", 0.6f, 0.6f, 50.0f} };
		for (const regime & g : regimes) {
			float w = 0;
			for (int k = 0; k < 2000; ++k) {
				float d1[3] = {rnd(), rnd(), rnd()}, d2[3] = {rnd(), rnd(), rnd()}, o1[3], o2[3];
				const float n1 = sqrtf(d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2]) + 1e-6f;
				const float n2 = sqrtf(d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2]) + 1e-6f;
				for (int i = 0; i < 3; ++i) {
					o1[i] = lroundf(rnd() * 1000) / 1000.0f;
					o2[i] = lroundf((o1[i] + rnd() * g.sep) * 1000) / 1000.0f;
					d1[i] = lroundf(d1[i] / n1 * g.l1 * 1000) / 1000.0f;
					d2[i] = lroundf(d2[i] / n2 * g.l2 * 1000) / 1000.0f;
				}
				// Measure the projected pair in float: at 10-50 apart fixed16's own length()
				// error would swamp the projection's.
				using tr = scalar_traits<T>;
				segment<T> s1, s2;
				s1.origin = v3<T>(o1[0], o1[1], o1[2]);
				s1.direction = v3<T>(d1[0], d1[1], d1[2]);
				s2.origin = v3<T>(o2[0], o2[1], o2[2]);
				s2.direction = v3<T>(d2[0], d2[1], d2[2]);
				vec3<T> p1, p2;
				project(p1, p2, s1, s2, tr::from_milli(1));
				const float dx = tr::to_float(p1.x) - tr::to_float(p2.x), dy = tr::to_float(p1.y) - tr::to_float(p2.y),
				            dz = tr::to_float(p1.z) - tr::to_float(p2.z);
				const float err = sqrtf(dx * dx + dy * dy + dz * dz) - ref_segment_distance(o1, d1, o2, d2);
				w = std::max(w, std::fabs(err));
			}
			printf("%s=%.5f ", g.name, w);
			assert(w < tol);
		}
	}

	// Long segments: the degree-4 products overflowed fixed16 (40² · 40² ≫ 32767)
	// before the power-of-two rescale; two perpendicular 40-long spines 1 apart.
	{
		const float o1[3] = {-20, 0, 0}, d1[3] = {40, 0, 0};
		const float o2[3] = {5, 1, -20}, d2[3] = {0, 0, 40};
		const float dist = seg_seg_distance<T>(o1, d1, o2, d2);
		printf("long=%.4f ", dist);
		assert(std::fabs(dist - 1.0f) < tol);
	}
	printf("OK\n");
}

// Float reference: distance from p to the union of a triangle's three edges.
static float ref_edges_distance(const float p[3], const float v[3][3]) {
	float best = 1e30f;
	for (int i = 0; i < 3; ++i) {
		const float d[3] = {v[(i + 1) % 3][0] - v[i][0], v[(i + 1) % 3][1] - v[i][1], v[(i + 1) % 3][2] - v[i][2]};
		static const float zero[3] = {0, 0, 0};
		best = std::min(best, ref_segment_distance(p, zero, v[i], d));
	}
	return best;
}

// Degenerate triangles (math/triangle.h). Coincident vertices used to divide 0/0
// on an edge branch — NaN weights in float, a saturated garbage weight in fixed —
// and closest_segment_triangle then latched that NaN as its "best".
template <typename T> static void test_degenerate_triangles(const char * label) {
	using tr = scalar_traits<T>;
	printf("  degenerate_triangles[%s]: ", label);
	const float tol = std::is_same<T, fixed16>::value ? 1e-2f : 1e-3f;
	// A==B, A==C, B==C, all three coincident, collinear (C between and C beyond).
	const float tris[][3][3] = {
	    {{0, 0, 0}, {0, 0, 0}, {1, 0, 0}},
	    {{0, 0, 0}, {1, 0, 0}, {0, 0, 0}},
	    {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}},
	    {{0.3f, 0.2f, 0.1f}, {0.3f, 0.2f, 0.1f}, {0.3f, 0.2f, 0.1f}},
	    {{0, 0, 0}, {1, 0, 0}, {0.5f, 0, 0}},
	    {{0, 0, 0}, {0.5f, 0, 0}, {1, 0, 0}},
	};
	const float pts[][3] = {{0.5f, 1, 0}, {-0.5f, 0.3f, 0.2f}, {1.7f, -0.4f, 0}, {0.25f, 0, 0.5f}, {0.5f, 0, 0}};
	int checked = 0;
	for (const auto & t : tris) {
		const vec3<T> a = v3<T>(t[0][0], t[0][1], t[0][2]), b = v3<T>(t[1][0], t[1][1], t[1][2]),
		              c = v3<T>(t[2][0], t[2][1], t[2][2]);
		for (const auto & pf : pts) {
			const vec3<T> p = v3<T>(pf[0], pf[1], pf[2]);
			T w[3];
			closest_point_triangle(p, a, b, c, w);
			const float w0 = tr::to_float(w[0]), w1 = tr::to_float(w[1]), w2 = tr::to_float(w[2]);
			assert(std::isfinite(w0) && std::isfinite(w1) && std::isfinite(w2));
			assert(std::fabs(w0 + w1 + w2 - 1.0f) < tol);
			vec3<T> cp;
			closest_point_triangle(cp, p, a, b, c);
			vec3<T> diff;
			sub(diff, p, cp);
			const float dist = tr::to_float(length(diff));
			assert(std::fabs(dist - ref_edges_distance(pf, t)) < tol);

			// Segment version: a short vertical segment starting at p, rising away.
			vec3<T> q = p, cs, ct;
			q.y = q.y + tr::from_milli(100);
			const float d2 = tr::to_float(closest_segment_triangle(p, q, a, b, c, cs, ct, tr::from_milli(1)));
			assert(std::isfinite(d2));
			assert(d2 <= dist * dist + tol);
			++checked;
		}
	}
	// The GJK triangle step with a duplicated support point.
	{
		T w[3];
		gjk_closest_triangle(v3<T>(1, 1, 0), v3<T>(1, 1, 0), v3<T>(-1, 1, 0), w);
		const float w0 = tr::to_float(w[0]), w1 = tr::to_float(w[1]), w2 = tr::to_float(w[2]);
		assert(std::isfinite(w0) && std::isfinite(w1) && std::isfinite(w2));
		assert(std::fabs(w0 + w1 - 0.5f) < tol && std::fabs(w2 - 0.5f) < tol); // closest is (0,1,0)
	}
	printf("cases=%d OK\n", checked);
}

// GJK stalls (#118). Clouds from a seeded random search, with exact distances
// from a QP solve. On the old code the float cases spun to max_iter (a vertex
// with weight below `epsilon` was dropped, so the same support came back each
// iteration) and the fixed16 ones were off by 6-33 cm (a repeated support
// point re-entered the simplex and corrupted v).
struct gjk_cloud_case {
	const char * what;
	float dist; // exact distance from the origin to the hull
	std::vector<std::array<float, 3>> pts;
};
static const std::vector<gjk_cloud_case> & gjk_cloud_cases() {
	static const std::vector<gjk_cloud_case> cases = {
	    {"float stall", 0.06771f, {{0.861f, -0.299f, -0.082f}, {0.088f, 0.239f, -0.16f}, {0.548f, -0.628f, 0.111f}, {0.559f, -0.496f, -0.212f}, {0.807f, 0.259f, 0.495f}, {0.966f, 0.014f, 0.272f}, {0.631f, -0.32f, -0.438f}, {0.021f, 0.151f, -0.308f}, {0.389f, -0.684f, -0.416f}, {0.785f, -0.351f, 0.07f}, {0.094f, -0.07f, 0.249f}, {0.211f, -0.043f, -0.114f}, {0.564f, -0.28f, 0.477f}}},
	    {"float stall", 0.02360f, {{0.092f, 0.319f, -0.122f}, {-0.694f, -0.293f, 0.372f}, {-0.021f, 0.009f, 0.006f}, {-0.675f, 0.326f, 0.364f}, {-0.18f, -0.188f, 0.049f}, {0.135f, 0.273f, 0.205f}, {-0.613f, -0.494f, -0.101f}, {-0.236f, 0.207f, -0.237f}, {0.115f, 0.228f, 0.499f}, {-0.67f, 0.112f, 0.368f}, {-0.781f, 0.22f, -0.269f}, {-0.105f, -0.438f, 0.474f}, {-0.326f, -0.208f, 0.202f}, {-0.061f, 0.156f, 0.622f}}},
	    {"fixed16 error 0.33", 0.44873f, {{0.495f, 0.685f, -0.369f}, {-0.656f, 1.295f, 0.951f}, {-1.586f, 3.654f, -0.219f}, {1.392f, 3.617f, 2.086f}, {0.968f, 0.056f, -1.028f}, {1.33f, 2.151f, 2.387f}, {-1.077f, 2.992f, 2.348f}, {-1.194f, 2.286f, 1.297f}, {-0.865f, 0.622f, -0.805f}, {0.163f, 3.802f, 1.23f}, {1.435f, 0.617f, 1.803f}, {0.686f, 3.098f, 1.488f}, {0.582f, 0.624f, 2.534f}, {-0.512f, 3.739f, 0.449f}}},
	    {"fixed16 error 0.06", 0.84470f, {{-3.894f, -0.229f, 0.559f}, {-0.603f, -0.154f, 1.153f}, {-1.262f, 1.592f, 1.095f}, {-2.031f, 0.958f, -1.101f}, {-3.917f, -0.106f, 0.395f}, {-3.846f, 1.871f, 1.594f}, {-3.253f, 1.103f, -0.478f}, {-0.543f, -1.741f, -1.178f}, {-2.909f, -1.124f, 0.397f}, {-2.044f, -0.85f, 2.167f}, {-3.658f, -0.504f, 0.961f}, {-3.692f, 2.006f, 2.201f}, {-1.534f, -0.78f, 1.711f}, {-4.16f, 0.811f, -1.318f}, {-2.319f, 0.96f, -0.479f}}},
	    {"fixed16 error 0.07", 1.51050f, {{0.424f, -0.137f, -1.887f}, {1.053f, -3.291f, 1.132f}, {3.787f, -2.622f, -1.876f}, {0.816f, -3.46f, -0.798f}, {0.97f, -0.379f, -1.701f}, {2.628f, -1.124f, 0.97f}, {4.128f, -2.711f, -1.173f}, {3.753f, -3.113f, -0.37f}, {2.166f, -3.768f, -1.236f}, {3.153f, -1.246f, -1.172f}, {3.772f, -2.166f, -2.007f}, {3.862f, -0.593f, -1.608f}, {0.832f, -3.53f, 1.183f}, {1.548f, -3.076f, 0.38f}, {4.124f, -3.574f, -1.348f}}},
	    {"float stall", 1.85863f, {{1.233f, 3.022f, -2.83f}, {-1.502f, 1.491f, -3.729f}, {1.1f, 0.262f, -3.673f}, {-0.3f, 2.715f, -3.311f}, {-0.483f, -0.709f, -3.976f}, {-2.126f, 2.718f, -1.649f}, {-2.502f, 1.811f, -2.881f}, {-0.713f, -0.17f, -1.708f}, {0.459f, 1.623f, -4.911f}}},
	};
	return cases;
}

template <typename T> static void test_gjk_no_stall(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_no_stall[%s]: ", label);
	const bool f16 = std::is_same<T, fixed16>::value;
	for (const auto & c : gjk_cloud_cases()) {
		std::vector<vec3<T>> pts;
		for (const auto & p : c.pts)
			pts.push_back(v3<T>(p[0], p[1], p[2]));
		// Shape A is a point at the origin, so the Minkowski difference is the cloud.
		auto sup_a = [](const vec3<T> &, vec3<T> & out) { out.reset(); };
		auto sup_b = [&](const vec3<T> & d, vec3<T> & out) {
			T best {};
			bool have = false;
			for (const auto & p : pts) {
				T s = dot(p, d);
				if (!have || s > best) { best = s; out = p; have = true; }
			}
		};
		T dist;
		vec3<T> n, zero;
		zero.reset();
		bool deep = false;
		gjk_core_distance<T>(sup_a, sup_b, zero, zero, tr::from_milli(1), dist, n, deep);
		assert(!deep);
		const float d = tr::to_float(dist);
		// Converged: |v| (an upper bound) meets the support lower bound along v̂ = -n.
		float lower = 1e30f;
		for (const auto & p : c.pts)
			lower = std::min(lower, -(p[0] * tr::to_float(n.x) + p[1] * tr::to_float(n.y) + p[2] * tr::to_float(n.z)));
		const float gap = d - lower;
		if (!f16)
			assert(gap <= 0.0025f + 0.001f * d);
		assert(std::fabs(d - c.dist) < (f16 ? 0.02f : 0.002f));
	}
	printf("cases=%zu OK\n", gjk_cloud_cases().size());
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

// Oriented polytope×polytope (the CSO path). A unit-cube target rotated
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

// PHANTOM-PENETRATION REGRESSION (gjk_closest_tetra): when the origin lies OUTSIDE
// the simplex (shapes separated), the closest face's barycentric weights must be
// recovered — i.e. they must not all come back zero. gjk_closest_tetra seeded its
// running minimum with best = default_max_position_component()², which OVERFLOWS
// fixed16 (max ≈ 32767, squared wraps int32) into garbage; the `dsq < best` test
// then rejected every face and the weights stayed all-zero. The GJK loop reads
// all-zero weights as wsum == 0 → dist 0, fabricating a deep contact of depth ==
// combined_radius for shapes that are actually apart. In demo_bounce the fast leash
// capsule got such a phantom hit against the floating octahedron, and the per-tick
// push-out then marched the octahedron straight down through the floor.
//
// Drive gjk_closest_tetra directly with a tetra sitting entirely in the +octant, so
// the origin is well outside it (closest feature is vertex a). Pre-fix fixed16: the
// returned weights summed to 0; post-fix they recover the real closest point.
template <typename T> static void test_gjk_tetra_origin_outside(const char * label) {
	using tr = scalar_traits<T>;
	printf("  gjk_tetra_origin_outside[%s]: ", label);
	vec3<T> a(tr::one(), tr::one(), tr::one());
	vec3<T> b(tr::two(), tr::one(), tr::one());
	vec3<T> c(tr::one(), tr::two(), tr::one());
	vec3<T> d(tr::one(), tr::one(), tr::two());
	T bary[4];
	bool inside = true;
	gjk_closest_tetra(a, b, c, d, bary, inside);
	T wsum = bary[0] + bary[1] + bary[2] + bary[3];
	// Reconstruct the closest point and verify it is the near vertex a = (1,1,1).
	vec3<T> q;
	mul(q, a, bary[0]);
	gjk_acc(q, b, bary[1]);
	gjk_acc(q, c, bary[2]);
	gjk_acc(q, d, bary[3]);
	printf("inside=%d wsum=%.3f q=(%.2f,%.2f,%.2f) ", inside, tr::to_float(wsum),
	       tr::to_float(q.x), tr::to_float(q.y), tr::to_float(q.z));
	assert(!inside);                                       // origin is NOT contained
	assert(tr::to_float(wsum) > 0.5f);                    // pre-fix fixed16: 0 (all-zero weights)
	assert(tr::to_float(q.x) > 0.9f && tr::to_float(q.x) < 1.1f); // closest point ≈ vertex a
	printf("OK\n");
}

// ORIENTED-CONVEX FALLBACK REGRESSION: the rounded×convex inflation fallback
// (use_gjk=false, also GJK's deep-penetration backstop) must respect the convex
// solid's orientation. It used to trace the primitive against the convex's LOCAL
// planes with translate-only placement, silently dropping the rotation — so a sphere
// resting against a rotated face got a normal pointing at the *unrotated* face. Here a
// sphere sits deep inside an octahedron toward one local face; rotating the whole
// configuration by θ about Z must rotate the contact normal by θ too. Pre-fix the
// rotated normal came back ≈ the unrotated one.
template <typename T> static void test_fallback_convex_orientation(const char * label) {
	using tr = scalar_traits<T>;
	printf("  fallback_convex_orientation[%s]: ", label);
	auto normal_at = [&](float ang) {
		auto cv = std::make_shared<solid<T>>();
		cv->set_infinite_mass();
		cv->add_shape(std::make_shared<shape<T>>(octahedron<T>(tr::from_milli(1500)))); // r = 1.5
		quat<T> q;
		set_quat_from_axis_angle(q, v3<T>(0, 0, 1), tr::from_milli((int)lroundf(ang * 1000)));
		cv->set_orientation_from_quat(q);
		cv->set_position(v3<T>(0, 0, 0));
		auto sp = std::make_shared<solid<T>>();
		sp->set_mass(tr::one());
		sp->add_shape(std::make_shared<shape<T>>(sphere<T>(v3<T>(0, 0, 0), tr::from_milli(400))));
		// Deep offset 0.3 along the (rotated) local +X, so the nearest face is unambiguous.
		vec3<T> off = v3<T>(0.3f * cosf(ang), 0.3f * sinf(ang), 0);
		sp->set_position(off);
		segment<T> seg;
		seg.set_start_dir(off, v3<T>(0, 0, 0));
		collision<T> c;
		c.time = tr::one();
		hop::test_solid(c, sp.get(), seg, cv.get(), tr::from_milli(1), T {}, /*use_gjk=*/false);
		return c.normal;
	};
	const float th = 0.5235988f; // 30° — not an octahedron symmetry, so rotation is observable
	vec3<T> n0 = normal_at(0.0f);
	vec3<T> nth = normal_at(th);
	// Expected: n0 rotated by +θ about Z.
	float ex = tr::to_float(n0.x) * cosf(th) - tr::to_float(n0.y) * sinf(th);
	float ey = tr::to_float(n0.x) * sinf(th) + tr::to_float(n0.y) * cosf(th);
	float dx = tr::to_float(nth.x) - ex, dy = tr::to_float(nth.y) - ey;
	printf("n0=(%.2f,%.2f) nth=(%.2f,%.2f) expected=(%.2f,%.2f) ", tr::to_float(n0.x), tr::to_float(n0.y),
	       tr::to_float(nth.x), tr::to_float(nth.y), ex, ey);
	assert(sqrtf(dx * dx + dy * dy) < 0.1f); // pre-fix: nth ≈ n0 (unrotated) ⇒ far from expected
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
	test_segment_segment_closest<T>(label);
	test_degenerate_triangles<T>(label);
	test_gjk_no_stall<T>(label);
	test_gjk_solid_orientation<T>(label);
	test_oriented_polytope<T>(label);
	test_gjk_tetra_origin_outside<T>(label);
	test_fallback_convex_orientation<T>(label);
}

int main() {
	printf("test_gjk:\n");
	run_gjk_tests<float>("float");
	run_gjk_tests<fixed16>("fixed16");
	run_gjk_tests<fixed32>("fixed32");
	printf("ALL PASSED\n");
	return 0;
}
