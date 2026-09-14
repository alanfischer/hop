// Contact manifolds: does a resting body get more than one contact point, and does that
// make it stop?
//
// The bug this phase exists for is a box tilted three degrees on a floor turning at
// several rad/s forever — see the header of examples/demo_ragdoll.cpp, whose control
// table is the headline measurement. This file is the unit-level version: the clip
// itself, the warm-start key, the reduction's determinism, and the shapes a manifold
// cannot help.
//
// The whole file is compiled TWICE (see tests/CMakeLists.txt): once at the default
// max_manifold_points = 4 and once at 1. At 1 the manifold machinery must disappear
// entirely and leave the pre-phase behaviour — one contact per partner — which is the
// parity claim the storage change was allowed to make.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <hop/hop.h>

using namespace hop;
using T = double;
using tr = scalar_traits<T>;
using V = vec3<T>;

static const T kEps = (T)0.001;
static const T kMargin = kEps * 8;  // the simulator's spec_margin_ default

static V v(T x, T y, T z) { V r; r.set(x, y, z); return r; }

static std::shared_ptr<solid<T>> make_box(T hx, T hy, T hz, V at, bool spins = true) {
	auto s = std::make_shared<solid<T>>();
	s->add_shape(std::make_shared<shape<T>>(aa_box<T>(v(-hx, -hy, -hz), v(hx, hy, hz))));
	s->set_position(at);
	s->set_mass((T)1);
	if (spins)
		s->set_inertia(v((T)0.01, (T)0.01, (T)0.01));
	return s;
}

static std::shared_ptr<solid<T>> make_floor() {
	auto s = std::make_shared<solid<T>>();
	s->add_shape(std::make_shared<shape<T>>(aa_box<T>(v(-50, -1, -50), v(50, 0, 50))));
	s->set_position(v(0, 0, 0));
	s->set_infinite_mass();
	s->set_coefficient_of_gravity(T {});
	return s;
}

// The contact the existing narrowphase would have reported: straight up, touching.
static collision<T> upward_contact(solid<T> * floor) {
	collision<T> c;
	c.reset();
	c.time = T {};
	c.normal = v(0, 1, 0);
	c.collider = floor;
	return c;
}

static int manifold_of(contact_point<T> * out, solid<T> * mover, solid<T> * floor) {
	collision<T> c = upward_contact(floor);
	return manifold_for_solids(out, max_manifold_points, c, mover, floor, kMargin, kEps);
}

// --- the clip -------------------------------------------------------------

// A box resting flat on a floor is held at FOUR points, one under each corner of its
// bottom face. This is the whole phase in one assertion: one point can only rock a box,
// four level it.
static void test_flat_box_reports_four_points() {
	auto floor = make_floor();
	auto box = make_box(0.1, 0.1, 0.1, v(0, 0.1, 0));
	contact_point<T> pts[max_manifold_points];
	const int n = manifold_of(pts, box.get(), floor.get());
	if (max_manifold_points == 1) {
		assert(n == 0 && "at one point per partner the clip must not run at all");
		printf("  flat_box_reports_four_points ok (parity build: clip disabled)\n");
		return;
	}
	assert(n == 4);
	for (int i = 0; i < n; ++i) {
		assert(std::fabs((double)pts[i].separation) < 1e-6 && "all four corners are touching");
		assert(std::fabs((double)pts[i].normal.y - 1.0) < 1e-9);
		assert(std::fabs((double)pts[i].impact.y) < 1e-6 && "the contact sits on the surface");
	}
	// And they are spread, not stacked: the four corners of a 0.2 m face.
	T spread = T {};
	for (int i = 0; i < n; ++i)
		for (int j = i + 1; j < n; ++j)
			spread = std::max(spread, (T)std::sqrt((double)length_squared(pts[i].impact, pts[j].impact)));
	assert(spread > 0.25 && "the manifold spans the face diagonal");
	printf("  flat_box_reports_four_points ok (spread %.3f m)\n", (double)spread);
}

// Tip it onto an edge and honesty requires TWO. The other two corners of that face are
// centimetres in the air, and a point there would be a fiction holding the box level.
static void test_box_on_an_edge_reports_two() {
	if (max_manifold_points == 1) { printf("  box_on_an_edge_reports_two skipped (parity build)\n"); return; }
	auto floor = make_floor();
	auto box = make_box(0.1, 0.1, 0.1, v(0, 0, 0));
	mat3<T> r;
	set_mat3_from_axis_angle(r, v(0, 0, 1), (T)(45.0 * 3.14159265358979 / 180.0));
	box->set_orientation(r);
	box->set_position(v(0, (T)(0.1 * std::sqrt(2.0)), 0));  // resting on the lower edge
	contact_point<T> pts[max_manifold_points];
	const int n = manifold_of(pts, box.get(), floor.get());
	assert(n == 2 && "a box on an edge touches along a line, so two points");
	printf("  box_on_an_edge_reports_two ok\n");
}

// And balanced on a corner, ONE. No clipping scheme changes that — there IS one point —
// which is why the shapes that can only ever touch at one point get rolling resistance
// instead of a manifold.
static void test_box_on_a_corner_reports_one() {
	if (max_manifold_points == 1) { printf("  box_on_a_corner_reports_one skipped (parity build)\n"); return; }
	auto floor = make_floor();
	auto box = make_box(0.1, 0.1, 0.1, v(0, 0, 0));
	// Turn the body diagonal (1,1,1) onto -Y, so a corner points straight down.
	V from = v(1, 1, 1), to = v(0, -1, 0);
	normalize(from);
	V axis;
	cross(axis, from, to);
	normalize(axis);
	mat3<T> r;
	set_mat3_from_axis_angle(r, axis, (T)std::acos((double)dot(from, to)));
	box->set_orientation(r);
	box->set_position(v(0, (T)(0.1 * std::sqrt(3.0)), 0));
	contact_point<T> pts[max_manifold_points];
	const int n = manifold_of(pts, box.get(), floor.get());
	assert(n == 1 && "a box on a corner touches at one point and always will");
	printf("  box_on_a_corner_reports_one ok\n");
}

// A compound body spanning two shapes of ONE partner must keep points from both, or it
// pivots about whichever shape won. This is why the reduction runs over the union of the
// shape pairs rather than per pair.
static void test_compound_partner_keeps_both_shapes() {
	if (max_manifold_points == 1) { printf("  compound_partner_keeps_both_shapes skipped (parity build)\n"); return; }
	auto ground = std::make_shared<solid<T>>();
	auto left = std::make_shared<shape<T>>(aa_box<T>(v(-0.5, -0.2, -0.5), v(-0.05, 0, 0.5)));
	auto right = std::make_shared<shape<T>>(aa_box<T>(v(0.05, -0.2, -0.5), v(0.5, 0, 0.5)));
	ground->add_shape(left);
	ground->add_shape(right);
	ground->set_infinite_mass();
	ground->set_coefficient_of_gravity(T {});
	ground->set_position(v(0, 0, 0));
	auto plank = make_box(0.4, 0.02, 0.1, v(0, 0.02, 0));
	contact_point<T> pts[max_manifold_points];
	const int n = manifold_of(pts, plank.get(), ground.get());
	assert(n >= 2);
	bool on_left = false, on_right = false;
	for (int i = 0; i < n; ++i) {
		if (pts[i].impact.x < -0.04) on_left = true;
		if (pts[i].impact.x > 0.04) on_right = true;
	}
	assert(on_left && on_right && "a plank bridging two blocks is held by both");
	printf("  compound_partner_keeps_both_shapes ok (%d points)\n", n);
}

// The reduction has to be a function of its inputs alone. A manifold that reduced
// differently run to run would desync a server from a client replaying the same tick —
// the same reason solid::solve_id_ exists. Ties break by index, never by float
// comparison order.
static void test_reduction_is_deterministic() {
	if (max_manifold_points == 1) { printf("  reduction_is_deterministic skipped (parity build)\n"); return; }
	auto floor = make_floor();
	auto box = make_box(0.1, 0.1, 0.1, v(0.37, 0.1, -0.21));
	mat3<T> r;
	set_mat3_from_axis_angle(r, v(0.5774, 0.5774, 0.5774), (T)0.05);
	box->set_orientation(r);
	contact_point<T> a[max_manifold_points], b[max_manifold_points];
	const int na = manifold_of(a, box.get(), floor.get());
	const int nb = manifold_of(b, box.get(), floor.get());
	assert(na == nb && na > 0);
	for (int i = 0; i < na; ++i) {
		assert(a[i].id == b[i].id && "same inputs, same points, same order");
		assert(a[i].separation == b[i].separation);
		assert(a[i].impact == b[i].impact);
	}
	printf("  reduction_is_deterministic ok (%d points)\n", na);
}

// --- the bug, as a test ---------------------------------------------------

struct world {
	simulator<T> sim;
	std::shared_ptr<solid<T>> floor;
	world() {
		sim.set_gravity(v(0, -20, 0));
		sim.set_default_contact_mode(contact_mode::speculative);
		floor = make_floor();
		sim.add_solid(floor);
		floor->set_collision_scope(1);
		floor->set_collide_with_scope(0);
	}
	std::shared_ptr<solid<T>> drop(std::shared_ptr<solid<T>> s) {
		sim.add_solid(s);
		s->set_contact_mode(contact_mode::speculative);
		s->set_collision_scope(0);
		s->set_collide_with_scope(1);
		return s;
	}
};

// THE test. A box tilted 0.05 rad used to turn at ~5 rad/s for as long as you ran it,
// because it was held up at one corner and gravity levered against it. It must now come
// to rest, sleep, and never spin up on the way.
static void test_a_tilted_box_sleeps() {
	world w;
	auto box = make_box(0.04, 0.04, 0.04, v(0, 0.5, 0));
	mat3<T> r;
	set_mat3_from_axis_angle(r, v(0.5774, 0.5774, 0.5774), (T)0.05);
	box->set_orientation(r);
	w.drop(box);

	int slept_at = -1;
	double worst_spin_after_landing = 0.0;
	for (int i = 0; i < 600; ++i) {
		w.sim.update((T)(1.0 / 60.0));
		// After it has had time to settle. A box dropped 46 cm onto a corner genuinely
		// tumbles for a second first, and that is not the bug — the bug is what it is
		// still doing at ten seconds.
		if (i > 150) {
			const double spin = std::sqrt((double)length_squared(box->get_angular_velocity()));
			worst_spin_after_landing = std::max(worst_spin_after_landing, spin);
		}
		if (slept_at < 0 && !box->active())
			slept_at = i;
	}
	if (max_manifold_points == 1) {
		// The parity build is allowed to keep the bug; what it must NOT do is behave
		// differently from the engine before the phase landed.
		printf("  a_tilted_box_sleeps: parity build spins at %.3f rad/s (the bug, preserved)\n",
		       worst_spin_after_landing);
	assert(worst_spin_after_landing > 1.0 && "at one point the corner lever is still there");
		return;
	}
	assert(slept_at >= 0 && slept_at < 300 && "a resting box sleeps");
	assert(worst_spin_after_landing < 0.001 && "and stays stopped, rather than rocking forever");
	assert(length_squared(box->get_angular_velocity()) == T {} && "sleep means rest, spin included");
	printf("  a_tilted_box_sleeps ok (asleep at tick %d, worst |w| %.4f)\n",
	       slept_at, worst_spin_after_landing);
}

// The warm-start contract. A body that is not moving must produce the SAME feature IDs
// tick after tick, or its points trade accumulators and buzz — which looks exactly like
// the bug this phase fixes.
static void test_feature_ids_are_stable_and_warm_start() {
	world w;
	auto box = make_box(0.1, 0.1, 0.1, v(0, 0.101, 0));
	w.drop(box);
	box->set_stay_active(true);  // keep it in the solve so the cache keeps refreshing

	std::vector<uint32_t> first;
	for (int i = 0; i < 120; ++i) {
		w.sim.update((T)(1.0 / 60.0));
		if (i != 80 && i != 100)
			continue;
		std::vector<uint32_t> ids;
		bool any_load = false;
		for (int k = 0; k < box->get_touch_count(); ++k) {
			const auto & slot = box->get_touch(k);
			for (int q = 0; q < slot.point_count; ++q) {
				ids.push_back(slot.points[q].id);
				any_load |= slot.points[q].accum_n > T {};
			}
		}
		assert(!ids.empty());
		assert(any_load && "a box on a floor is carrying load through its manifold");
		if (i == 80)
			first = ids;
		else
			assert(ids == first && "a still body regenerates the same manifold, point for point");
	}
	printf("  feature_ids_are_stable_and_warm_start ok (%d points)\n", (int)first.size());
}

// A pile of DYNAMIC bodies. This is the phase's known cost, and it reports rather than
// asserts, so the number is in plain sight and moves the day the solver can hold a
// multi-point contact between two free bodies (see the limitation note in collide.h).
//
// A stack held by single points is stable for an unflattering reason: the one contact
// resolves at the face centre and cannot torque anything. With a manifold each body
// settles anywhere within the slop band rather than flat, the tilt is a ramp for what is
// above, and the load ROLLS down it about alternating edges — so the contact never slips
// and friction has nothing to oppose. Nothing in WizardWars is exposed to it: a ragdoll's
// bones and a gib carry collision_mask = WORLD and rest only on static geometry.
static void test_a_dynamic_stack_is_a_known_limitation() {
	world w;
	std::vector<std::shared_ptr<solid<T>>> boxes;
	for (int i = 0; i < 5; ++i)
		boxes.push_back(w.drop(make_box(0.1, 0.1, 0.1, v(0, 0.1 + i * 0.2, 0))));
	for (auto & b : boxes) {
		b->set_collision_scope(1);
		b->set_collide_with_scope(1);
	}
	for (int i = 0; i < 1200; ++i)
		w.sim.update((T)(1.0 / 60.0));
	double worst = 0.0;
	for (int i = 0; i < 5; ++i)
		worst = std::max(worst, std::fabs((double)boxes[i]->get_position().y - (0.1 + i * 0.2)));
	printf("  a_dynamic_stack_is_a_known_limitation: five boxes, worst %.3f m off after 20 s%s\n",
	       worst, max_manifold_points == 1 ? " (single-point build)" : "");
	// The floor is the one thing that must hold whatever else happens: nothing may end
	// up BELOW it, or a body has been pushed through static geometry, which is a
	// different and much worse failure than a stack coming apart.
	for (int i = 0; i < 5; ++i)
		assert((double)boxes[i]->get_position().y > 0.09 && "nothing fell through the floor");
}

// A body resting on STATIC geometry — the case the phase exists for, and the one that has
// to be right. Four rows under it, settled flat, asleep, and no sign of the slop-band
// tilt the dynamic stack above suffers from.
static void test_a_box_on_static_geometry_settles_flat() {
	world w;
	auto block = make_box(0.5, 0.1, 0.5, v(0, 0.1, 0), /*spins*/ false);
	block->set_infinite_mass();
	block->set_coefficient_of_gravity(T {});
	w.sim.add_solid(block);
	block->set_collision_scope(1);
	block->set_collide_with_scope(0);

	auto box = make_box(0.1, 0.1, 0.1, v(0, 0.45, 0));
	mat3<T> r;
	set_mat3_from_axis_angle(r, v(0.5774, 0.5774, 0.5774), (T)0.05);
	box->set_orientation(r);
	w.drop(box);

	for (int i = 0; i < 600; ++i)
		w.sim.update((T)(1.0 / 60.0));
	V up;
	mul(up, box->get_orientation(), v(0, 1, 0));
	const double tilt = std::acos(std::min(1.0, (double)up.y));
	const double drift = std::sqrt((double)(box->get_position().x * box->get_position().x +
	                                        box->get_position().z * box->get_position().z));
	if (max_manifold_points == 1) {
		printf("  a_box_on_static_geometry_settles_flat: single-point build rests %.4f rad off level\n", tilt);
		return;
	}
	assert(!box->active() && "it sleeps");
	assert(tilt < 0.01 && "and settles flat, not perched on a corner");
	assert(drift < 0.01 && "and does not walk off the block");
	printf("  a_box_on_static_geometry_settles_flat ok (tilt %.5f rad, drift %.5f m)\n", tilt, drift);
}

// --- the shapes a manifold cannot help ------------------------------------

// A capsule lying on its side has a genuine two-point manifold — the endpoints of its
// inner segment. Stood on its end it has one, and no clipping changes that.
static void test_a_lying_capsule_has_two_points() {
	if (max_manifold_points == 1) { printf("  a_lying_capsule_has_two_points skipped (parity build)\n"); return; }
	auto floor = make_floor();
	auto rod = std::make_shared<solid<T>>();
	rod->add_shape(std::make_shared<shape<T>>(capsule<T>(v(-0.09, 0, 0), v(0.18, 0, 0), (T)0.01)));
	rod->set_mass((T)1);
	rod->set_inertia(v((T)0.002, (T)0.004, (T)0.004));
	rod->set_position(v(0, 0.01, 0));
	contact_point<T> pts[max_manifold_points];
	int n = manifold_of(pts, rod.get(), floor.get());
	assert(n == 2 && "a rod lying down touches at both ends");
	assert(pts[0].impact.x * pts[1].impact.x < 0 && "one witness per end, not two at one end");

	// Stand it up and the honest answer is one point, which the single-contact path
	// already gives — so the manifold declines rather than inventing a second.
	mat3<T> r;
	set_mat3_from_axis_angle(r, v(0, 0, 1), (T)(90.0 * 3.14159265358979 / 180.0));
	rod->set_orientation(r);
	rod->set_position(v(0, 0.10, 0));
	n = manifold_of(pts, rod.get(), floor.get());
	assert(n == 0 && "a capsule on its end has one point, and the clip says so by declining");
	printf("  a_lying_capsule_has_two_points ok\n");
}

// And it has to SETTLE, which is the thing bug 2 of plans/rotating_gibs.md says it never
// does: "a capsule resting on a floor creates spin from nothing". Before manifolds a rod
// dropped on a floor held 19-22 rad/s indefinitely whatever attitude it landed in.
static void test_a_dropped_rod_comes_to_rest() {
	for (double tilt_deg : { 0.0, 20.0, 60.0, 85.0 }) {
		world w;
		auto rod = std::make_shared<solid<T>>();
		rod->add_shape(std::make_shared<shape<T>>(capsule<T>(v(-0.09, 0, 0), v(0.18, 0, 0), (T)0.01)));
		rod->set_mass((T)1);
		rod->set_inertia(v((T)0.002, (T)0.004, (T)0.004));
		if (tilt_deg != 0) {
			mat3<T> m;
			set_mat3_from_axis_angle(m, v(0, 0, 1), (T)(tilt_deg * 3.14159265358979 / 180.0));
			rod->set_orientation(m);
		}
		rod->set_position(v(0, 0.35, 0));
		w.drop(rod);
		for (int i = 0; i < 900; ++i)
			w.sim.update((T)(1.0 / 60.0));
		const double spin = std::sqrt((double)length_squared(rod->get_angular_velocity()));
		if (max_manifold_points == 1) {
			printf("  a_dropped_rod_comes_to_rest: single-point build still turns at %.1f rad/s (tilt %.0f)\n",
			       spin, tilt_deg);
			continue;
		}
		assert(!rod->active() && "a rod dropped on a floor comes to rest");
		assert(spin == 0.0 && "dead still, whatever attitude it landed in");
	}
	if (max_manifold_points > 1)
		printf("  a_dropped_rod_comes_to_rest ok (asleep and |w| 0 at every tilt)\n");
}

// A sphere touches a floor at exactly one point however it is clipped, so it gets a
// different mechanism: resistance to spin at the contact. It must STOP the ball, and it
// must never reverse the spin — a rolling-resistance term that overshoots is a motor.
static void test_rolling_resistance_stops_a_sphere() {
	auto run = [](T mu_roll, double & final_spin, double & sign_flips) {
		world w;
		auto ball = std::make_shared<solid<T>>();
		ball->add_shape(std::make_shared<shape<T>>(sphere<T>(v(0, 0, 0), (T)0.1)));
		ball->set_mass((T)1);
		const T I = (T)(0.4 * 1.0 * 0.1 * 0.1);
		ball->set_inertia(v(I, I, I));
		ball->set_position(v(0, 0.1, 0));
		ball->set_coefficient_of_rolling_friction(mu_roll);
		w.drop(ball);
		w.floor->set_coefficient_of_rolling_friction(mu_roll);
		ball->set_angular_velocity(v(0, 0, -8));
		ball->set_velocity(v(0.8, 0, 0));
		double last = -8.0;
		sign_flips = 0;
		for (int i = 0; i < 900; ++i) {
			w.sim.update((T)(1.0 / 60.0));
			const double wz = (double)ball->get_angular_velocity().z;
			if (last < -0.05 && wz > 0.05)
				sign_flips += 1;
			last = wz;
		}
		final_spin = std::sqrt((double)length_squared(ball->get_angular_velocity()));
	};
	double free_spin = 0, damped_spin = 0, flips = 0, ignored = 0;
	run(T {}, free_spin, ignored);
	run((T)0.2, damped_spin, flips);
	assert(damped_spin < free_spin * 0.5 && "rolling resistance actually stops the ball");
	assert(flips == 0 && "and only ever removes spin — it must not drive it backwards");
	printf("  rolling_resistance_stops_a_sphere ok (free |w| %.3f, damped |w| %.3f)\n",
	       free_spin, damped_spin);
}

// The footprint claim the storage change was allowed to make: at one point per partner a
// touch slot is what it always was, and the engine behaves as it always did.
static void test_parity_at_one_point() {
	auto floor = make_floor();
	auto box = make_box(0.1, 0.1, 0.1, v(0, 0.1, 0));
	contact_point<T> pts[max_manifold_points];
	const int n = manifold_of(pts, box.get(), floor.get());
	if (max_manifold_points == 1)
		assert(n == 0 && "nothing to clip, nothing to reduce");
	else
		assert(n > 1);
	printf("  parity_at_one_point ok (max_manifold_points=%d)\n", max_manifold_points);
}

int main() {
	printf("test_manifold (max_manifold_points = %d)\n", max_manifold_points);
	test_flat_box_reports_four_points();
	test_box_on_an_edge_reports_two();
	test_box_on_a_corner_reports_one();
	test_compound_partner_keeps_both_shapes();
	test_reduction_is_deterministic();
	test_a_tilted_box_sleeps();
	test_feature_ids_are_stable_and_warm_start();
	test_a_box_on_static_geometry_settles_flat();
	test_a_dynamic_stack_is_a_known_limitation();
	test_a_lying_capsule_has_two_points();
	test_a_dropped_rod_comes_to_rest();
	test_rolling_resistance_stops_a_sphere();
	test_parity_at_one_point();
	printf("test_manifold: all passed\n");
	return 0;
}
