// Tests for compound colliders — solids with multiple shapes, and shapes
// with non-zero local_position within their owning solid.
//
// These tests verify that adding local_position to a shape is equivalent to
// placing the same geometry via the solid's position, and that compound
// solids collide correctly at the right spots.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

static bool approx(float a, float b, float tol = 0.05f) { return std::fabs(a - b) < tol; }

// Helper: floor wall at z=0
template <typename T> static std::shared_ptr<solid<T>> make_floor(simulator<T> & sim) {
	using tr = scalar_traits<T>;
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T {});
	wall->set_coefficient_of_restitution(tr::one());
	wall->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(tr::from_int(-10), tr::from_int(-10), -tr::one()),
	              vec3<T>(tr::from_int(10), tr::from_int(10), T {}))));
	sim.add_solid(wall);
	return wall;
}

// -------- Equivalence tests: shape at local_position == shape placed by solid position --------

// A sphere at local_position=(3,0,0) within a solid at origin should behave the
// same way as a sphere at local_position=(0,0,0) within a solid at (3,0,0).
template <typename T> static void test_sphere_local_position_equivalence(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_local_position_equivalence[%s]: ", label);

	// Setup A: sphere offset via local_position, solid at origin
	simulator<T> simA;
	simA.set_gravity({ T {}, T {}, T {} });
	auto sA = std::make_shared<solid<T>>();
	sA->set_mass(tr::one());
	sA->set_coefficient_of_restitution(tr::one());
	sA->set_coefficient_of_restitution_override(true);
	auto sphA = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	sphA->set_local_position({ tr::from_int(3), T {}, T {} });
	sA->add_shape(sphA);
	sA->set_position({ T {}, T {}, T {} });
	sA->set_velocity({ T {}, T {}, T {} });
	simA.add_solid(sA);

	auto tA = std::make_shared<solid<T>>();
	tA->set_infinite_mass();
	tA->set_coefficient_of_gravity(T {});
	tA->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	tA->set_position({ tr::from_int(5), T {}, T {} });
	simA.add_solid(tA);
	sA->set_velocity({ tr::from_int(2), T {}, T {} });

	for (int i = 0; i < 100; ++i) simA.update(10);
	float xA = tr::to_float(sA->get_position().x);

	// Setup B: sphere at solid origin, solid at (3,0,0)
	simulator<T> simB;
	simB.set_gravity({ T {}, T {}, T {} });
	auto sB = std::make_shared<solid<T>>();
	sB->set_mass(tr::one());
	sB->set_coefficient_of_restitution(tr::one());
	sB->set_coefficient_of_restitution_override(true);
	sB->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sB->set_position({ tr::from_int(3), T {}, T {} });
	simB.add_solid(sB);

	auto tB = std::make_shared<solid<T>>();
	tB->set_infinite_mass();
	tB->set_coefficient_of_gravity(T {});
	tB->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	tB->set_position({ tr::from_int(5), T {}, T {} });
	simB.add_solid(tB);
	sB->set_velocity({ tr::from_int(2), T {}, T {} });

	for (int i = 0; i < 100; ++i) simB.update(10);
	float xB = tr::to_float(sB->get_position().x);

	// sB starts 3 ahead of sA (sB.pos=3, sA.pos=0 but its sphere is at x=3).
	// After bouncing, sA's position vs sB's position should differ by exactly 3.
	printf("xA=%.3f xB=%.3f (xB-xA=%.3f) ", xA, xB, xB - xA);
	assert(approx(xB - xA, 3.0f, 0.05f));
	printf("OK\n");
}

// Same but for box shape
template <typename T> static void test_box_local_position_equivalence(const char * label) {
	using tr = scalar_traits<T>;
	printf("  box_local_position_equivalence[%s]: ", label);

	simulator<T> simA;
	simA.set_gravity({ T {}, T {}, T {} });
	auto sA = std::make_shared<solid<T>>();
	sA->set_mass(tr::one());
	sA->set_coefficient_of_restitution(tr::one());
	sA->set_coefficient_of_restitution_override(true);
	auto boxA = std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half())));
	boxA->set_local_position({ tr::from_int(2), T {}, T {} });
	sA->add_shape(boxA);
	sA->set_position({ T {}, T {}, T {} });
	sA->set_velocity({ tr::from_int(2), T {}, T {} });
	simA.add_solid(sA);

	auto wA = std::make_shared<solid<T>>();
	wA->set_infinite_mass();
	wA->set_coefficient_of_gravity(T {});
	wA->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half()))));
	wA->set_position({ tr::from_int(5), T {}, T {} });
	simA.add_solid(wA);

	for (int i = 0; i < 100; ++i) simA.update(10);
	float xA = tr::to_float(sA->get_position().x);

	simulator<T> simB;
	simB.set_gravity({ T {}, T {}, T {} });
	auto sB = std::make_shared<solid<T>>();
	sB->set_mass(tr::one());
	sB->set_coefficient_of_restitution(tr::one());
	sB->set_coefficient_of_restitution_override(true);
	sB->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half()))));
	sB->set_position({ tr::from_int(2), T {}, T {} });
	sB->set_velocity({ tr::from_int(2), T {}, T {} });
	simB.add_solid(sB);

	auto wB = std::make_shared<solid<T>>();
	wB->set_infinite_mass();
	wB->set_coefficient_of_gravity(T {});
	wB->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half()))));
	wB->set_position({ tr::from_int(5), T {}, T {} });
	simB.add_solid(wB);

	for (int i = 0; i < 100; ++i) simB.update(10);
	float xB = tr::to_float(sB->get_position().x);

	printf("xA=%.3f xB=%.3f (xB-xA=%.3f) ", xA, xB, xB - xA);
	assert(approx(xB - xA, 2.0f, 0.05f));
	printf("OK\n");
}

// -------- Compound collider tests --------

// Dumbbell: two spheres offset in +x and -x, on a single solid. Tests that
// each sphere independently registers collisions on its side.
template <typename T> static void test_dumbbell_collides_both_sides(const char * label) {
	using tr = scalar_traits<T>;
	printf("  dumbbell_collides_both_sides[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	// Dumbbell at origin, not moving
	auto dumb = std::make_shared<solid<T>>();
	dumb->set_infinite_mass();
	dumb->set_coefficient_of_gravity(T {});
	dumb->set_coefficient_of_restitution(tr::one());
	auto left = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	left->set_local_position({ -tr::from_int(2), T {}, T {} });
	dumb->add_shape(left);
	auto right = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	right->set_local_position({ tr::from_int(2), T {}, T {} });
	dumb->add_shape(right);
	sim.add_solid(dumb);

	// Test 1: ball approaches from left side, should hit the left sphere at x≈-2.5
	auto ballL = std::make_shared<solid<T>>();
	ballL->set_mass(tr::one());
	ballL->set_coefficient_of_restitution(tr::one());
	ballL->set_coefficient_of_restitution_override(true);
	ballL->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	ballL->set_position({ -tr::from_int(5), T {}, T {} });
	ballL->set_velocity({ tr::from_int(3), T {}, T {} });
	sim.add_solid(ballL);

	// Ball starts 2m from contact; at 3 m/s needs ~670ms. 100 steps of 10ms gives margin.
	for (int i = 0; i < 100; ++i) sim.update(10);
	float vxL = tr::to_float(ballL->get_velocity().x);
	printf("vxL=%.2f ", vxL);
	assert(vxL < 0.0f); // bounced back

	// Test 2: ball approaches from right side, should hit the right sphere at x≈2.5
	simulator<T> sim2;
	sim2.set_gravity({ T {}, T {}, T {} });
	auto dumb2 = std::make_shared<solid<T>>();
	dumb2->set_infinite_mass();
	dumb2->set_coefficient_of_gravity(T {});
	dumb2->set_coefficient_of_restitution(tr::one());
	auto left2 = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	left2->set_local_position({ -tr::from_int(2), T {}, T {} });
	dumb2->add_shape(left2);
	auto right2 = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	right2->set_local_position({ tr::from_int(2), T {}, T {} });
	dumb2->add_shape(right2);
	sim2.add_solid(dumb2);

	auto ballR = std::make_shared<solid<T>>();
	ballR->set_mass(tr::one());
	ballR->set_coefficient_of_restitution(tr::one());
	ballR->set_coefficient_of_restitution_override(true);
	ballR->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	ballR->set_position({ tr::from_int(5), T {}, T {} });
	ballR->set_velocity({ -tr::from_int(3), T {}, T {} });
	sim2.add_solid(ballR);

	for (int i = 0; i < 100; ++i) sim2.update(10);
	float vxR = tr::to_float(ballR->get_velocity().x);
	printf("vxR=%.2f ", vxR);
	assert(vxR > 0.0f); // bounced back
	printf("OK\n");
}

// Character: capsule body + sphere feet offset in +z from the capsule base.
// Drop it onto the floor; the feet should make contact, not the body.
template <typename T> static void test_character_compound_lands_on_feet(const char * label) {
	using tr = scalar_traits<T>;
	printf("  character_compound_lands_on_feet[%s]: ", label);
	simulator<T> sim;
	make_floor(sim);

	auto ch = std::make_shared<solid<T>>();
	ch->set_mass(tr::one());
	ch->set_coefficient_of_restitution(T {});  // inelastic: settle on floor
	ch->set_coefficient_of_restitution_override(true);
	ch->set_coefficient_of_static_friction(T {});
	ch->set_coefficient_of_dynamic_friction(T {});

	// Capsule body: 1.4 tall, radius 0.3, offset upward by 0.4
	// So body occupies z in [0.4 - 0.3, 0.4 + 1.4 + 0.3] = [0.1, 2.1] relative to solid origin.
	auto body = std::make_shared<shape<T>>(hop::capsule<T>(
	    vec3<T>(), vec3<T>(T {}, T {}, tr::from_milli(1400)), tr::from_milli(300)));
	body->set_local_position({ T {}, T {}, tr::from_milli(400) });
	ch->add_shape(body);

	// Feet spheres: radius 0.2, offset downward by 0.2 so they occupy z in [-0.4, 0]
	auto foot_l = std::make_shared<shape<T>>(hop::sphere<T>(tr::from_milli(200)));
	foot_l->set_local_position({ tr::from_milli(200), T {}, -tr::from_milli(200) });
	ch->add_shape(foot_l);
	auto foot_r = std::make_shared<shape<T>>(hop::sphere<T>(tr::from_milli(200)));
	foot_r->set_local_position({ -tr::from_milli(200), T {}, -tr::from_milli(200) });
	ch->add_shape(foot_r);

	ch->set_position({ T {}, T {}, tr::from_int(4) });  // drop from z=4
	sim.add_solid(ch);

	// Let it settle
	for (int i = 0; i < 400; ++i) sim.update(10);

	float z = tr::to_float(ch->get_position().z);
	float vz = tr::to_float(ch->get_velocity().z);
	printf("z=%.3f vz=%.3f ", z, vz);
	// Character's origin should settle such that the feet (radius 0.2, offset -0.2)
	// touch the floor. So origin z should be ~0.4 (foot bottom = origin + offset - radius = z - 0.4 = 0)
	assert(z >= 0.35f && z <= 0.55f);
	assert(std::fabs(vz) < 1.0f);
	printf("OK\n");
}

// Compound with mixed shape types: box + sphere. Verify segment trace hits both.
template <typename T> static void test_compound_segment_trace(const char * label) {
	using tr = scalar_traits<T>;
	printf("  compound_segment_trace[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s = std::make_shared<solid<T>>();
	s->set_infinite_mass();
	s->set_coefficient_of_gravity(T {});
	// Box at (-2, 0, 0), sphere at (2, 0, 0)
	auto bx = std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	              vec3<T>(tr::half(), tr::half(), tr::half())));
	bx->set_local_position({ -tr::from_int(2), T {}, T {} });
	s->add_shape(bx);
	auto sp = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	sp->set_local_position({ tr::from_int(2), T {}, T {} });
	s->add_shape(sp);
	sim.add_solid(s);

	// Trace segment from left, should first hit the box at x ≈ -2.5
	segment<T> segL;
	segL.origin = { -tr::from_int(5), T {}, T {} };
	segL.direction = { tr::from_int(10), T {}, T {} };
	collision<T> colL;
	sim.trace_segment(colL, segL, -1);
	float impactLx = tr::to_float(colL.impact.x);
	printf("impactL.x=%.3f ", impactLx);
	assert(approx(impactLx, -2.5f, 0.1f));

	// Trace segment from right, should first hit the sphere at x ≈ 2.5
	segment<T> segR;
	segR.origin = { tr::from_int(5), T {}, T {} };
	segR.direction = { -tr::from_int(10), T {}, T {} };
	collision<T> colR;
	sim.trace_segment(colR, segR, -1);
	float impactRx = tr::to_float(colR.impact.x);
	printf("impactR.x=%.3f ", impactRx);
	assert(approx(impactRx, 2.5f, 0.1f));

	printf("OK\n");
}

// Verify impact point is correct when sh1 has non-zero local_position.
// Drop a sphere-solid with sphere offset by (0,0,0.5) onto floor: the solid's
// center should settle at z≈0 (since sphere is lifted 0.5 above origin in local),
// and the impact point should be at world z≈0 (on the floor surface).
template <typename T> static void test_impact_with_local_position(const char * label) {
	using tr = scalar_traits<T>;
	printf("  impact_with_local_position[%s]: ", label);
	simulator<T> sim;
	make_floor(sim);

	auto s = std::make_shared<solid<T>>();
	s->set_mass(tr::one());
	s->set_coefficient_of_restitution(T {});
	s->set_coefficient_of_restitution_override(true);
	auto sph = std::make_shared<shape<T>>(hop::sphere<T>(tr::half()));
	sph->set_local_position({ T {}, T {}, tr::half() });  // sphere lifted 0.5 within solid
	s->add_shape(sph);
	s->set_position({ T {}, T {}, tr::from_int(4) });

	vec3<T> last_impact;
	bool got_impact = false;
	s->set_collision_callback([&](const collision<T> & c) {
		last_impact = c.impact;
		got_impact = true;
	});

	sim.add_solid(s);

	// scope_report_collisions is required for set_collision_callback to fire.
	for (int i = 0; i < 400; ++i) sim.update(10, simulator<T>::scope_report_collisions);

	assert(got_impact);
	float iz = tr::to_float(last_impact.z);
	printf("impact.z=%.3f ", iz);
	// Impact point is the sphere-floor contact; should be at z ≈ 0 (floor surface)
	assert(std::fabs(iz) < 0.15f);
	printf("OK\n");
}

// Regression: capsule-vs-convex_solid picks the earliest of two spine-endpoint
// traces. An earlier version struct-assigned the chosen sub-collision into the
// shared col, clobbering col.collider (set at the top of test_solid) to null.
// The callback should see a non-null collider after a capsule/convex_solid hit.
template <typename T> static void test_capsule_vs_convex_preserves_collider(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_vs_convex_preserves_collider[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	// Target: infinite-mass convex cube at x=5
	auto target = std::make_shared<solid<T>>();
	target->set_infinite_mass();
	target->set_coefficient_of_gravity(T {});
	hop::convex_solid<T> cs;
	T one = tr::one(), zero {}, neg_one = -tr::one(), half = tr::half();
	cs.planes.push_back(hop::plane<T>(one, zero, zero, half));
	cs.planes.push_back(hop::plane<T>(neg_one, zero, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, one, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, neg_one, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, zero, one, half));
	cs.planes.push_back(hop::plane<T>(zero, zero, neg_one, half));
	target->add_shape(std::make_shared<shape<T>>(cs));
	target->set_position({ tr::from_int(2), T {}, T {} });
	sim.add_solid(target);

	// Mover: capsule sliding +x toward the cube
	auto mover = std::make_shared<solid<T>>();
	mover->set_mass(tr::one());
	mover->set_coefficient_of_restitution(tr::one());
	mover->set_coefficient_of_restitution_override(true);
	mover->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(), vec3<T>(T {}, T {}, tr::from_int(1)), tr::from_milli(300))));
	mover->set_position({ T {}, T {}, T {} });
	mover->set_velocity({ tr::from_int(5), T {}, T {} });

	solid<T> * seen_collider = nullptr;
	mover->set_collision_callback([&](const collision<T> & c) {
		if (!seen_collider)
			seen_collider = c.collider;
	});
	sim.add_solid(mover);

	for (int i = 0; i < 200; ++i) sim.update(10, simulator<T>::scope_report_collisions);

	assert(seen_collider);  // callback fired (collider was not wiped to null)
	assert(seen_collider == target.get());
	printf("OK\n");
}

template <typename T> static void run_all_tests(const char * label) {
	printf(" [%s]\n", label);
	test_sphere_local_position_equivalence<T>(label);
	test_box_local_position_equivalence<T>(label);
	test_dumbbell_collides_both_sides<T>(label);
	test_character_compound_lands_on_feet<T>(label);
	test_compound_segment_trace<T>(label);
	test_impact_with_local_position<T>(label);
	test_capsule_vs_convex_preserves_collider<T>(label);
}

int main() {
	printf("test_compound:\n");
	run_all_tests<float>("float");
	run_all_tests<fixed16>("fixed16");
	printf("ALL PASSED\n");
	return 0;
}
