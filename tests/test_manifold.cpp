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
	// The floor is a parameter so the same world can be stood on a box solid or on
	// traceable geometry (see make_traceable_floor) with nothing else differing.
	explicit world(std::shared_ptr<solid<T>> on = make_floor()) {
		sim.set_gravity(v(0, -20, 0));
		sim.set_default_contact_mode(contact_mode::speculative);
		floor = std::move(on);
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
// A stack of n 0.2 m boxes, resting on the floor and on each other, settled for 20 s.
// Shared by the two stack tests, which differ only in how deep they go.
static std::vector<std::shared_ptr<solid<T>>> settle_a_stack(world & w, int n) {
	std::vector<std::shared_ptr<solid<T>>> boxes;
	for (int i = 0; i < n; ++i) {
		auto b = w.drop(make_box(0.1, 0.1, 0.1, v(0, 0.1 + i * 0.2, 0)));
		b->set_collision_scope(1);
		b->set_collide_with_scope(1);
		boxes.push_back(b);
	}
	for (int i = 0; i < 1200; ++i)
		w.sim.update((T)(1.0 / 60.0));
	return boxes;
}

static void test_a_dynamic_stack_is_a_known_limitation() {
	world w;
	const auto boxes = settle_a_stack(w, 5);
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

// Two dynamic boxes, one on the other, at the SHIPPING solver settings. This is the
// shallow end of the stack limitation above, and it is the case shock propagation used to
// knock over: with the phase running on the manifold's four rows the top box was off by a
// fifth of a metre after 20 s (it toppled); left to the single-point contacts the phase is
// meant for, it drifts a millimetre or so and stays a stack. The bottom box is not the
// story — it rests on static geometry, which test_a_box_on_static_geometry_settles_flat
// covers far more thoroughly.
static void test_two_dynamic_boxes_survive_the_shock_phase() {
	world w;
	assert(w.sim.get_shock_iterations() > 0 && "the phase this guards is on by default");
	const auto boxes = settle_a_stack(w, 2);
	const double off = std::fabs((double)boxes[1]->get_position().y - 0.3);
	assert(off < 0.01 && "the top box is still on the bottom one after 20 s");
	printf("  two_dynamic_boxes_survive_the_shock_phase ok (top box %.4f m off after 20 s)\n", off);
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

// --- the pair decides, not the caller -------------------------------------

// A ground plane, so a manifold can be asked for with a TRACEABLE on either side of the
// pair. Only trace_segment is exercised: that is what the traceable manifold probes with.
class plane_traceable : public traceable<T> {
public:
	void get_bound(aa_box<T> & result) override {
		result.mins.set((T)-50, (T)-1, (T)-50);
		result.maxs.set((T)50, T {}, (T)50);
	}
	void trace_segment(collision<T> & result, const vec3<T> & position, const mat3<T> &,
	                   const segment<T> & seg) override {
		const T y = position.y;
		if (seg.direction.y >= T {})
			return;
		const T t = (y - seg.origin.y) / seg.direction.y;
		if (t < T {} || t >= (T)1)
			return;
		result.time = t;
		result.normal = v(0, 1, 0);
		V travel;
		mul(travel, seg.direction, t);
		add(result.point, seg.origin, travel);
		result.impact.set(result.point);
	}
	// The solid half of the contract, so a body can be SIMULATED against this floor and
	// not only probed against it. Level and unrotated like trace_segment above, and
	// shape-agnostic: the mover's lowest point is its support point, whatever it is of.
	void trace_solid(collision<T> & result, solid<T> * s, const vec3<T> & position, const mat3<T> &,
	                 const segment<T> & seg, T margin) override {
		const mat3<T> & R = s->get_orientation();
		mat3<T> Rt;
		transpose(Rt, R);
		V down;
		mul(down, Rt, v(0, -1, 0));
		V lowest;
		bool have = false;
		for (auto & sh : s->get_shapes()) {
			V local_sup, sup;
			support_in_solid(local_sup, *sh, down);
			mul(sup, R, local_sup);
			if (!have || sup.y < lowest.y) {
				lowest = sup;
				have = true;
			}
		}
		if (!have)
			return;
		auto witness = [&](const V & mover_origin) {
			result.impact.set(mover_origin.x + lowest.x, position.y, mover_origin.z + lowest.z);
		};
		const T gap = seg.origin.y + lowest.y - position.y;
		if (gap <= margin) {
			if (result.time > T {}) {
				result.time = T {};
				result.point.set(seg.origin);
				result.normal = v(0, 1, 0);
				result.depth = margin - gap;
				witness(seg.origin);
			}
			return;
		}
		if (seg.direction.y >= T {})
			return;
		const T t = (margin - gap) / seg.direction.y;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			mul(result.point, seg.direction, t);
			add(result.point, seg.origin);
			result.normal = v(0, 1, 0);
			witness(result.point);
		}
	}
};

// The other floor `world` can be built on: the same level surface as a TRACEABLE, which
// is a corpse on a BSP hull rather than on a collision shape. Passed to world's
// constructor so everything else about the two — gravity, contact mode, scopes — is the
// same object and not a copy of it, which is what makes the floor the only variable.
static std::shared_ptr<solid<T>> make_traceable_floor() {
	auto s = std::make_shared<solid<T>>();
	s->add_shape(std::make_shared<shape<T>>(std::make_unique<plane_traceable>()));
	s->set_infinite_mass();
	s->set_coefficient_of_gravity(T {});
	s->set_position(v(0, 0, 0));
	return s;
}

static std::shared_ptr<solid<T>> make_rod(V at) {
	auto rod = std::make_shared<solid<T>>();
	rod->add_shape(std::make_shared<shape<T>>(capsule<T>(v(-0.09, 0, 0), v(0.18, 0, 0), (T)0.01)));
	rod->set_mass((T)1);
	rod->set_inertia(v((T)0.002, (T)0.004, (T)0.004));
	rod->set_position(at);
	return rod;
}

// Each body used to ask for the manifold with ITSELF as the mover, which reached a
// different branch and got a different answer: a rod under a slab clipped two points
// asked from the capsule's side and NONE from the box's. Asking from either side must
// now return the same points with the normals reversed.
static void assert_both_sides_agree(const char * what, solid<T> * s1, solid<T> * s2,
                                    V normal_toward_s1, int expect) {
	collision<T> c1;
	c1.reset();
	c1.time = T {};
	c1.normal = normal_toward_s1;
	c1.collider = s2;
	collision<T> c2;
	c2.reset();
	c2.time = T {};
	neg(c2.normal, normal_toward_s1);
	c2.collider = s1;

	contact_point<T> from_1[max_manifold_points];
	contact_point<T> from_2[max_manifold_points];
	const int n1 = manifold_for_solids(from_1, max_manifold_points, c1, s1, s2, kMargin, kEps);
	const int n2 = manifold_for_solids(from_2, max_manifold_points, c2, s2, s1, kMargin, kEps);
	assert(n1 == expect && "asked from one side");
	assert(n2 == expect && "asked from the other");
	for (int i = 0; i < n1; ++i) {
		assert(from_1[i].id == from_2[i].id && "same feature, same warm-start key");
		assert(from_1[i].impact == from_2[i].impact && "same point, to the bit");
		assert(from_1[i].separation == from_2[i].separation && "same gap");
		V sum;
		add(sum, from_1[i].normal, from_2[i].normal);
		assert(length_squared(sum) == T {} && "and exactly opposite normals");
	}
	printf("  both_sides_agree[%s] ok (%d points each way)\n", what, n1);
}

static void test_both_sides_see_the_same_manifold() {
	if (max_manifold_points == 1) { printf("  both_sides_see_the_same_manifold skipped (parity build)\n"); return; }
	world w;  // solids need distinct solve ids, which is what orders the pair

	// Two faces. Even here the sides disagreed: each clipped its own face as the
	// reference, so the points landed under different feature ids.
	auto lower = make_box(0.5, 0.1, 0.5, v(0, 0.1, 0));
	auto upper = make_box(0.1, 0.1, 0.1, v(0, 0.3, 0));
	w.sim.add_solid(lower);
	w.sim.add_solid(upper);
	assert_both_sides_agree("box under box", upper.get(), lower.get(), v(0, 1, 0), 4);

	// THE case from the bug: the capsule brings two witnesses and the slab the reference
	// face, whichever of them is asking.
	auto rod = make_rod(v(0, 0, 0));
	auto slab = make_box(0.5, 0.1, 0.5, v(0, 0.11, 0));
	w.sim.add_solid(rod);
	w.sim.add_solid(slab);
	assert_both_sides_agree("rod under slab", rod.get(), slab.get(), v(0, -1, 0), 2);

	// And a traceable, which used to be reachable only as the partner — a box on level
	// geometry is a corpse on a map.
	auto ground = std::make_shared<solid<T>>();
	ground->add_shape(std::make_shared<shape<T>>(std::make_unique<plane_traceable>()));
	ground->set_infinite_mass();
	ground->set_position(v(0, 0, 0));
	auto crate = make_box(0.1, 0.1, 0.1, v(0, 0.1, 0));
	w.sim.add_solid(ground);
	w.sim.add_solid(crate);
	assert_both_sides_agree("box on level geometry", crate.get(), ground.get(), v(0, 1, 0), 4);
}

// A rod lying on TRACEABLE geometry is held at both ends, exactly as it is on a box.
// The witness machinery behind both is the same call — build_contact_witnesses, which
// answers for a capsule as readily as for a box — so a capsule on a map floor has no
// business getting a different answer from a capsule on a crate.
static void test_a_lying_capsule_on_traceable_geometry_has_two_points() {
	if (max_manifold_points == 1) { printf("  a_lying_capsule_on_traceable_geometry_has_two_points skipped (parity build)\n"); return; }
	world w;  // solids need distinct solve ids, which is what orders the pair
	auto ground = std::make_shared<solid<T>>();
	ground->add_shape(std::make_shared<shape<T>>(std::make_unique<plane_traceable>()));
	ground->set_infinite_mass();
	ground->set_position(v(0, 0, 0));
	auto rod = make_rod(v(0, 0.01, 0));
	w.sim.add_solid(ground);
	w.sim.add_solid(rod);
	assert_both_sides_agree("rod on level geometry", rod.get(), ground.get(), v(0, 1, 0), 2);
}

// One manifold per pair per tick, held by BOTH sides: the second body to discover the
// contact mirrors the first's points rather than clipping its own.
static void test_the_two_slots_hold_one_manifold() {
	if (max_manifold_points == 1) { printf("  the_two_slots_hold_one_manifold skipped (parity build)\n"); return; }
	world w;
	auto lower = w.drop(make_box(0.1, 0.1, 0.1, v(0, 0.1, 0)));
	auto upper = w.drop(make_box(0.1, 0.1, 0.1, v(0, 0.3, 0)));
	for (auto & b : { lower, upper }) {
		b->set_collision_scope(1);
		b->set_collide_with_scope(1);
		b->set_stay_active(true);  // keep both caches refreshing so there is something to compare
	}
	int checked = 0;
	std::vector<uint32_t> ids, last_ids;
	int id_changes = 0;
	for (int i = 0; i < 240; ++i) {
		w.sim.update((T)(1.0 / 60.0));
		if (i < 60)
			continue;  // let them land first
		const auto * mine = (const typename solid<T>::touch *)nullptr;
		const auto * theirs = (const typename solid<T>::touch *)nullptr;
		for (int k = 0; k < lower->get_touch_count(); ++k)
			if (lower->get_touch(k).partner == upper.get()) mine = &lower->get_touch(k);
		for (int k = 0; k < upper->get_touch_count(); ++k)
			if (upper->get_touch(k).partner == lower.get()) theirs = &upper->get_touch(k);
		if (!mine || !theirs)
			continue;
		assert(mine->point_count == theirs->point_count && "one manifold, both sides");
		ids.clear();
		for (int q = 0; q < mine->point_count; ++q) {
			assert(mine->points[q].id == theirs->points[q].id && "point for point, in the same order");
			assert(mine->points[q].impact == theirs->points[q].impact);
			V sum;
			add(sum, mine->points[q].normal, theirs->points[q].normal);
			assert(length_squared(sum) == T {} && "each side stores it pointing at itself");
			ids.push_back(mine->points[q].id);
		}
		// And the same manifold TICK TO TICK, which is the warm-start contract. Both
		// bodies are awake, so the side that clips alternates with update()'s traversal
		// flip — and without slack in clip_manifold's reference tie the ids swapped with
		// it every tick.
		if (!last_ids.empty() && ids != last_ids)
			++id_changes;
		last_ids = ids;
		++checked;
	}
	assert(checked > 100 && "the pair was actually in contact for the run");
	assert(id_changes == 0 && "and the same manifold tick after tick, whoever clipped it");
	printf("  the_two_slots_hold_one_manifold ok (%d ticks compared, ids never moved)\n", checked);
}

// --- the shapes a manifold cannot help ------------------------------------

// A capsule lying on its side has a genuine two-point manifold — the endpoints of its
// inner segment. Stood on its end it has one, and no clipping changes that.
static void test_a_lying_capsule_has_two_points() {
	if (max_manifold_points == 1) { printf("  a_lying_capsule_has_two_points skipped (parity build)\n"); return; }
	auto floor = make_floor();
	auto rod = make_rod(v(0, 0.01, 0));
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
//
// Run on BOTH floors. A box solid and a traceable are the same surface as far as a rod
// lying on it is concerned, and they reach the manifold by different routes — clipping
// against a face, and probing the geometry — so a rod that settles on one and not the
// other has found a difference in the route, which is what map geometry used to be.
static void test_a_dropped_rod_comes_to_rest_on(const char * floor, std::shared_ptr<solid<T>> (*make)()) {
	for (double tilt_deg : { 0.0, 20.0, 60.0, 85.0 }) {
		world w(make());  // a fresh floor per drop: a solid belongs to one simulator
		auto rod = make_rod(v(0, 0.35, 0));
		if (tilt_deg != 0) {
			mat3<T> m;
			set_mat3_from_axis_angle(m, v(0, 0, 1), (T)(tilt_deg * 3.14159265358979 / 180.0));
			rod->set_orientation(m);
		}
		w.drop(rod);
		for (int i = 0; i < 900; ++i)
			w.sim.update((T)(1.0 / 60.0));
		const double spin = std::sqrt((double)length_squared(rod->get_angular_velocity()));
		if (max_manifold_points == 1) {
			printf("  a_dropped_rod_comes_to_rest[%s]: single-point build still turns at %.1f rad/s (tilt %.0f)\n",
			       floor, spin, tilt_deg);
			continue;
		}
		assert(rod->get_position().y > 0.0 && "it stayed on top of the floor");
		assert(!rod->active() && "a rod dropped on a floor comes to rest");
		assert(spin == 0.0 && "dead still, whatever attitude it landed in");
	}
	if (max_manifold_points > 1)
		printf("  a_dropped_rod_comes_to_rest[%s] ok (asleep and |w| 0 at every tilt)\n", floor);
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
	test_two_dynamic_boxes_survive_the_shock_phase();
	test_a_dynamic_stack_is_a_known_limitation();
	test_both_sides_see_the_same_manifold();
	test_the_two_slots_hold_one_manifold();
	test_a_lying_capsule_has_two_points();
	test_a_dropped_rod_comes_to_rest_on("box floor", make_floor);
	test_a_dropped_rod_comes_to_rest_on("map geometry", make_traceable_floor);
	test_a_lying_capsule_on_traceable_geometry_has_two_points();
	test_rolling_resistance_stops_a_sphere();
	test_parity_at_one_point();
	printf("test_manifold: all passed\n");
	return 0;
}
