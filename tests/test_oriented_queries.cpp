#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

// Rotation in the query paths.
//
// test_segment traces against geometry authored in each shape's OWN frame, so a
// shape carrying a rotation (from its solid's orientation, its own local_rotation,
// or both) has to have the segment carried into that frame and the hit carried back
// out. It used to do neither for box / sphere / capsule / convex_solid — only the
// traceable branch composed R — so every ray and point query silently answered for
// the UNROTATED shape.
//
// Every case below is paired with an unrotated control, so a test that passes for
// the wrong reason (probe misplaced, shape absent) shows up as the control failing.

template <typename T> static float f(T v) { return scalar_traits<T>::to_float(v); }

static bool approx(float a, float b, float tol) { return std::fabs(a - b) < tol; }

// A long, thin box: half-extents (2, 0.5, 0.3). A 90° yaw swaps its long axis from
// X to Z, so a probe out at z = 1.5 is inside it only when the rotation is honored.
template <typename T> static aa_box<T> long_box() {
	using tr = scalar_traits<T>;
	return aa_box<T>(vec3<T>(-tr::from_int(2), -tr::half(), -tr::from_milli(300)),
	                 vec3<T>(tr::from_int(2), tr::half(), tr::from_milli(300)));
}

// Straight down through (x, *, z), from y=+2 to y=-2.
template <typename T> static segment<T> down_through(T x, T z) {
	using tr = scalar_traits<T>;
	segment<T> seg;
	seg.set_start_end(vec3<T>(x, tr::from_int(2), z), vec3<T>(x, -tr::from_int(2), z));
	return seg;
}

template <typename T> static mat3<T> yaw90() {
	using tr = scalar_traits<T>;
	mat3<T> r;
	set_mat3_from_axis_angle(r, vec3<T>(T {}, tr::one(), T {}), tr::pi() * tr::half());
	return r;
}

// --- test_segment ------------------------------------------------------------

// The rotation may live on the solid, on the shape, or be split across both; all
// three compose to the same world placement and must give the same answer.
enum class where { solid_orientation, shape_rotation };

template <typename T> static void segment_vs_rotated_box(const char * label, where w) {
	using tr = scalar_traits<T>;
	printf("  segment_vs_rotated_box[%s,%s]: ", label,
	       w == where::solid_orientation ? "solid" : "shape");

	auto s = std::make_shared<solid<T>>();
	s->set_infinite_mass();
	s->add_shape(std::make_shared<shape<T>>(long_box<T>()));

	segment<T> seg = down_through<T>(T {}, tr::from_milli(1500));

	// Control: unrotated, the probe at z=1.5 is well outside the 0.3-deep box.
	collision<T> miss;
	miss.reset();
	test_segment(miss, seg, s.get(), tr::from_milli(1));
	assert(f(miss.time) >= 1.0f);

	if (w == where::solid_orientation)
		s->set_orientation(yaw90<T>());
	else
		s->get_shapes()[0]->set_local_rotation(yaw90<T>());

	collision<T> hit;
	hit.reset();
	test_segment(hit, seg, s.get(), tr::from_milli(1));
	// Yawed, the long axis runs along Z and the probe passes through the box's top
	// face at y = +0.5 — a quarter of the way down a 4-long segment.
	assert(f(hit.time) < 1.0f);
	assert(approx(f(hit.point.y), 0.5f, 0.05f));
	assert(approx(f(hit.normal.y), 1.0f, 0.05f));
	printf("t=%.3f point.y=%.3f n.y=%.3f OK\n", f(hit.time), f(hit.point.y), f(hit.normal.y));
}

// local_position is an offset in the SOLID's frame, so an oriented solid rotates it.
// (The old code added it unrotated, which put the shape in the wrong place the moment
// the solid turned.)
template <typename T> static void segment_vs_offset_rotated_box(const char * label) {
	using tr = scalar_traits<T>;
	printf("  segment_vs_offset_rotated_box[%s]: ", label);

	auto s = std::make_shared<solid<T>>();
	s->set_infinite_mass();
	auto sh = std::make_shared<shape<T>>(long_box<T>());
	vec3<T> lp(tr::from_int(3), T {}, T {});
	sh->set_local_position(lp);
	s->add_shape(sh);
	mat3<T> R = yaw90<T>();
	s->set_orientation(R);

	// Where the shape's frame origin actually lands: solid_position + R·local_position.
	vec3<T> base;
	mul(base, R, lp);

	// The box's long axis is along Z after the yaw, so probe through base + 1.5 in Z.
	segment<T> seg = down_through<T>(base.x, base.z + tr::from_milli(1500));
	collision<T> hit;
	hit.reset();
	test_segment(hit, seg, s.get(), tr::from_milli(1));
	assert(f(hit.time) < 1.0f);
	assert(approx(f(hit.point.y), 0.5f, 0.05f));

	// Control: the same probe placed as if local_position had NOT been rotated.
	segment<T> wrong = down_through<T>(lp.x, lp.z + tr::from_milli(1500));
	collision<T> miss;
	miss.reset();
	test_segment(miss, wrong, s.get(), tr::from_milli(1));
	assert(f(miss.time) >= 1.0f);
	printf("base=(%.1f,%.1f,%.1f) t=%.3f OK\n", f(base.x), f(base.y), f(base.z), f(hit.time));
}

template <typename T> static void segment_vs_rotated_convex(const char * label) {
	using tr = scalar_traits<T>;
	printf("  segment_vs_rotated_convex[%s]: ", label);

	// Same geometry as long_box(), spelled out as planes so this test doesn't lean on
	// the helper it is also exercising indirectly.
	const aa_box<T> b = long_box<T>();
	convex_solid<T> cs;
	cs.planes.push_back(plane<T>(vec3<T>(tr::one(), T {}, T {}), b.maxs.x));
	cs.planes.push_back(plane<T>(vec3<T>(-tr::one(), T {}, T {}), -b.mins.x));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, tr::one(), T {}), b.maxs.y));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, -tr::one(), T {}), -b.mins.y));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, tr::one()), b.maxs.z));
	cs.planes.push_back(plane<T>(vec3<T>(T {}, T {}, -tr::one()), -b.mins.z));
	// Vertices are left to ensure_vertices(), which derives them from the planes.

	auto s = std::make_shared<solid<T>>();
	s->set_infinite_mass();
	s->add_shape(std::make_shared<shape<T>>(cs));

	segment<T> seg = down_through<T>(T {}, tr::from_milli(1500));
	collision<T> miss;
	miss.reset();
	test_segment(miss, seg, s.get(), tr::from_milli(1));
	assert(f(miss.time) >= 1.0f);

	s->set_orientation(yaw90<T>());
	collision<T> hit;
	hit.reset();
	test_segment(hit, seg, s.get(), tr::from_milli(1));
	assert(f(hit.time) < 1.0f);
	assert(approx(f(hit.point.y), 0.5f, 0.05f));
	printf("t=%.3f point.y=%.3f OK\n", f(hit.time), f(hit.point.y));
}

template <typename T> static void segment_vs_rotated_capsule(const char * label) {
	using tr = scalar_traits<T>;
	printf("  segment_vs_rotated_capsule[%s]: ", label);

	// Spine from (-1.5,0,0) to (1.5,0,0), radius 0.3 — long in X, thin elsewhere.
	capsule<T> cap(vec3<T>(-tr::from_milli(1500), T {}, T {}),
	               vec3<T>(tr::from_int(3), T {}, T {}), tr::from_milli(300));
	auto s = std::make_shared<solid<T>>();
	s->set_infinite_mass();
	s->add_shape(std::make_shared<shape<T>>(cap));

	segment<T> seg = down_through<T>(T {}, tr::from_int(1));
	collision<T> miss;
	miss.reset();
	test_segment(miss, seg, s.get(), tr::from_milli(1));
	assert(f(miss.time) >= 1.0f);

	s->set_orientation(yaw90<T>());
	collision<T> hit;
	hit.reset();
	test_segment(hit, seg, s.get(), tr::from_milli(1));
	assert(f(hit.time) < 1.0f);
	assert(approx(f(hit.point.y), 0.3f, 0.05f));
	printf("t=%.3f point.y=%.3f OK\n", f(hit.time), f(hit.point.y));
}

// --- test_solid ---------------------------------------------------------------

// Deep interior overlap of a rounded shape and a rotated box: GJK declines here, so
// this exercises the fallback specifically.
// The shallow case is GJK's, not the fallback's — pin it so a fallback change can't
// quietly take over the pair.
template <typename T> static void sphere_grazing_rotated_box(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_grazing_rotated_box[%s]: ", label);

	auto box = std::make_shared<solid<T>>();
	box->set_infinite_mass();
	box->add_shape(std::make_shared<shape<T>>(long_box<T>()));
	box->set_orientation(yaw90<T>());

	// Just above the yawed box's top face (y = 0.5), overlapping it by a hair.
	auto sph = std::make_shared<solid<T>>();
	sph->set_infinite_mass();
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::from_milli(100))));
	vec3<T> at(T {}, tr::from_milli(550), tr::from_milli(1500));
	sph->set_position(at);

	segment<T> zseg;
	zseg.set_start_end(at, at);
	collision<T> hit;
	hit.reset();
	test_solid(hit, sph.get(), zseg, box.get(), tr::from_milli(1));
	assert(f(hit.time) == 0.0f);
	assert(approx(f(hit.normal.y), 1.0f, 0.2f));
	printf("t=%.3f n.y=%.3f OK\n", f(hit.time), f(hit.normal.y));
}

// simulator::trace_solid's own broad phase. It grows the segment box by the mover's
// bound — which must be the ORIENTATION-rotated one, or a yawed mover gets a box too
// small and pointing the wrong way, and anything it should touch away from the trace
// origin is rejected before the oriented narrowphase runs. (The per-tick sweep already
// used world_bound_ for exactly this reason; the public query entry point did not.)
template <typename T> static void trace_solid_broadphase_is_oriented(const char * label) {
	using tr = scalar_traits<T>;
	printf("  trace_solid_broadphase_is_oriented[%s]: ", label);

	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	// 1-unit target cube at the origin.
	auto target = std::make_shared<solid<T>>();
	target->set_infinite_mass();
	target->set_collision_scope(1);
	target->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half()))));
	sim.add_solid(target);

	// The long thin box from long_box(), yawed 90° and parked out along +Z so its far
	// end reaches back into the target. Unrotated it would lie along X and touch
	// nothing — the broad phase must not answer for that one.
	auto mover = std::make_shared<solid<T>>();
	mover->set_infinite_mass();
	mover->set_collision_scope(0);
	mover->set_collide_with_scope(1);
	mover->add_shape(std::make_shared<shape<T>>(long_box<T>()));
	mover->set_orientation(yaw90<T>());
	vec3<T> at(T {}, T {}, tr::from_milli(1800));
	mover->set_position(at);
	sim.add_solid(mover);

	segment<T> zseg;
	zseg.set_start_end(at, at);
	collision<T> col;
	col.reset();
	sim.trace_solid(col, mover.get(), zseg, 1);
	assert(f(col.time) < 1.0f);

	// Control: unrotated, the same box lies along X and misses.
	mover->set_orientation(mat3<T>());
	collision<T> miss;
	miss.reset();
	sim.trace_solid(miss, mover.get(), zseg, 1);
	assert(f(miss.time) >= 1.0f);
	printf("t=%.3f OK\n", f(col.time));
}


// The contact point on a FACE has to say WHERE on the face. Taking it from the traced
// shape's support along -n cannot: the direction is perpendicular to the face's own
// axes, so the tangential position collapses to the face centre. On a long blade that
// puts every contact at the hub, and the lever arm the solver turns into w x r with it.
template <typename T> static void face_contact_point_is_where_it_touches(const char * label) {
	using tr = scalar_traits<T>;
	printf("  face_contact_point_is_where_it_touches[%s]: ", label);

	// A 4 m blade, 0.08 thick, centred at the origin.
	auto blade = std::make_shared<solid<T>>();
	blade->set_infinite_mass();
	blade->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_int(2), -tr::from_milli(40), -tr::from_milli(40)),
	              vec3<T>(tr::from_int(2), tr::from_milli(40), tr::from_milli(40)))));

	// A ball resting just above the blade's top face, well out along its length.
	const T out_along = tr::from_milli(1700);
	auto ball = std::make_shared<solid<T>>();
	ball->set_infinite_mass();
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::from_milli(120))));
	ball->set_position(vec3<T>(out_along, tr::from_milli(150), T {}));

	// Trace the BLADE upward into the ball, so the polytope is the traced shape — the
	// case where the support query has nothing tangential to say.
	segment<T> up;
	vec3<T> from(T {}, T {}, T {});
	vec3<T> to(T {}, tr::from_milli(60), T {});
	up.set_start_end(from, to);

	collision<T> hit;
	hit.reset();
	test_solid(hit, blade.get(), up, ball.get(), tr::from_milli(1));
	assert(f(hit.time) < 1.0f);                        // it touches at all
	assert(approx(f(hit.normal.y), -1.0f, 0.2f));      // on the blade's top face

	// The contact is 1.7 out along the blade, not at its hub. Without this the point
	// lands at x ~ 0 and the blade's lever arm collapses to its half-thickness.
	assert(approx(f(hit.impact.x), f(out_along), 0.2f));
	printf("impact=(%.3f %.3f) t=%.3f OK\n", f(hit.impact.x), f(hit.impact.y), f(hit.time));
}

template <typename T> static void run_all(const char * label) {
	segment_vs_rotated_box<T>(label, where::solid_orientation);
	segment_vs_rotated_box<T>(label, where::shape_rotation);
	segment_vs_offset_rotated_box<T>(label);
	segment_vs_rotated_convex<T>(label);
	segment_vs_rotated_capsule<T>(label);
	sphere_grazing_rotated_box<T>(label);
	trace_solid_broadphase_is_oriented<T>(label);
	face_contact_point_is_where_it_touches<T>(label);
}

int main() {
	printf("test_oriented_queries:\n");
	run_all<float>("float");
	run_all<fixed16>("fixed16");
	printf("ALL PASSED\n");
	return 0;
}
