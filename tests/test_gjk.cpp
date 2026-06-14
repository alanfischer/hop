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
	test_gjk_capsule_box_edge_rideup<T>(label);
	test_gjk_flag_switches_path<T>(label);
	test_gjk_rest_tangential_free<T>(label);
	test_gjk_penetration_reports<T>(label);
	test_ca_custom_closest<T>(label);
}

int main() {
	printf("test_gjk:\n");
	run_gjk_tests<float>("float");
	run_gjk_tests<fixed16>("fixed16");
	run_gjk_tests<fixed32>("fixed32");
	printf("ALL PASSED\n");
	return 0;
}
