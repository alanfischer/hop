#include <hop/hop.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace hop;

static constexpr float EPS = 0.1f;

static bool approx(float a, float b, float tol = EPS) {
	return std::fabs(a - b) < tol;
}

// Helper: create a floor wall at z=0
template<typename T>
static std::shared_ptr<solid<T>> make_floor(simulator<T>& sim) {
	using tr = scalar_traits<T>;
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T{});
	wall->set_coefficient_of_restitution(tr::one());
	wall->add_shape(std::make_shared<shape<T>>(aa_box<T>(
		vec3<T>(tr::from_int(-10), tr::from_int(-10), -tr::one()),
		vec3<T>(tr::from_int(10), tr::from_int(10), T{})
	)));
	sim.add_solid(wall);
	return wall;
}

// Test: sphere drops onto floor and bounces (COR=1 should preserve energy)
static void test_sphere_floor_bounce() {
	printf("  sphere_floor_bounce: ");
	using tr = scalar_traits<float>;
	simulator<float> sim;

	make_floor(sim);

	auto sphere = std::make_shared<solid<float>>();
	sphere->set_mass(1.0f);
	sphere->set_coefficient_of_restitution(1.0f);
	sphere->set_coefficient_of_restitution_override(true);
	sphere->set_coefficient_of_static_friction(0.0f);
	sphere->set_coefficient_of_dynamic_friction(0.0f);
	sphere->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	sphere->set_position({0.0f, 0.0f, 5.0f});
	sim.add_solid(sphere);

	// Run for 2 seconds
	for (int i = 0; i < 200; ++i) sim.update(10);

	// With COR=1, the sphere should still be bouncing (z > 0.5)
	float z = sphere->get_position().z;
	printf("z=%.2f ", z);
	assert(z > 0.4f); // Must be above the floor
	printf("OK\n");
}

// Test: box-box collision — two boxes moving toward each other
static void test_box_box_collision() {
	printf("  box_box_collision: ");
	simulator<float> sim;
	sim.set_gravity({0, 0, 0}); // No gravity

	auto box1 = std::make_shared<solid<float>>();
	box1->set_mass(1.0f);
	box1->set_coefficient_of_restitution(1.0f);
	box1->set_coefficient_of_restitution_override(true);
	box1->set_coefficient_of_static_friction(0.0f);
	box1->set_coefficient_of_dynamic_friction(0.0f);
	box1->add_shape(std::make_shared<shape<float>>(aa_box<float>(
		vec3<float>(-0.5f, -0.5f, -0.5f), vec3<float>(0.5f, 0.5f, 0.5f)
	)));
	box1->set_position({-3.0f, 0.0f, 0.0f});
	box1->set_velocity({2.0f, 0.0f, 0.0f});
	sim.add_solid(box1);

	auto box2 = std::make_shared<solid<float>>();
	box2->set_mass(1.0f);
	box2->set_coefficient_of_restitution(1.0f);
	box2->set_coefficient_of_restitution_override(true);
	box2->set_coefficient_of_static_friction(0.0f);
	box2->set_coefficient_of_dynamic_friction(0.0f);
	box2->add_shape(std::make_shared<shape<float>>(aa_box<float>(
		vec3<float>(-0.5f, -0.5f, -0.5f), vec3<float>(0.5f, 0.5f, 0.5f)
	)));
	box2->set_position({3.0f, 0.0f, 0.0f});
	box2->set_velocity({-2.0f, 0.0f, 0.0f});
	sim.add_solid(box2);

	for (int i = 0; i < 200; ++i) sim.update(10);

	// After elastic collision with equal masses, velocities should swap
	// box1 should be moving -x, box2 should be moving +x
	float v1x = box1->get_velocity().x;
	float v2x = box2->get_velocity().x;
	printf("v1x=%.2f v2x=%.2f ", v1x, v2x);
	assert(v1x < -1.0f); // Should be approximately -2
	assert(v2x > 1.0f);  // Should be approximately +2
	printf("OK\n");
}

// Test: sphere-sphere collision
static void test_sphere_sphere_collision() {
	printf("  sphere_sphere_collision: ");
	simulator<float> sim;
	sim.set_gravity({0, 0, 0});

	auto s1 = std::make_shared<solid<float>>();
	s1->set_mass(1.0f);
	s1->set_coefficient_of_restitution(1.0f);
	s1->set_coefficient_of_restitution_override(true);
	s1->set_coefficient_of_static_friction(0.0f);
	s1->set_coefficient_of_dynamic_friction(0.0f);
	s1->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s1->set_position({-3.0f, 0.0f, 0.0f});
	s1->set_velocity({3.0f, 0.0f, 0.0f});
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<float>>();
	s2->set_mass(1.0f);
	s2->set_coefficient_of_restitution(1.0f);
	s2->set_coefficient_of_restitution_override(true);
	s2->set_coefficient_of_static_friction(0.0f);
	s2->set_coefficient_of_dynamic_friction(0.0f);
	s2->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s2->set_position({3.0f, 0.0f, 0.0f});
	s2->set_velocity({-3.0f, 0.0f, 0.0f});
	sim.add_solid(s2);

	for (int i = 0; i < 200; ++i) sim.update(10);

	float v1x = s1->get_velocity().x;
	float v2x = s2->get_velocity().x;
	printf("v1x=%.2f v2x=%.2f ", v1x, v2x);
	assert(v1x < -1.0f);
	assert(v2x > 1.0f);
	printf("OK\n");
}

// Test: capsule-box collision
static void test_capsule_box_collision() {
	printf("  capsule_box_collision: ");
	simulator<float> sim;

	make_floor(sim);

	auto cap = std::make_shared<solid<float>>();
	cap->set_mass(1.0f);
	cap->set_coefficient_of_restitution(0.8f);
	cap->set_coefficient_of_restitution_override(true);
	cap->set_coefficient_of_static_friction(0.0f);
	cap->set_coefficient_of_dynamic_friction(0.0f);
	cap->add_shape(std::make_shared<shape<float>>(hop::capsule<float>(
		vec3<float>(), vec3<float>(0, 0, 1.0f), 0.3f
	)));
	cap->set_position({0.0f, 0.0f, 5.0f});
	sim.add_solid(cap);

	for (int i = 0; i < 200; ++i) sim.update(10);

	// Capsule should have bounced and still be above floor
	float z = cap->get_position().z;
	printf("z=%.2f ", z);
	assert(z > 0.2f);
	printf("OK\n");
}

// Test: restitution — COR=0 should stop bouncing
static void test_inelastic_collision() {
	printf("  inelastic_collision: ");
	simulator<float> sim;

	auto floor = make_floor(sim);
	floor->set_coefficient_of_restitution(0.0f);

	auto sphere = std::make_shared<solid<float>>();
	sphere->set_mass(1.0f);
	sphere->set_coefficient_of_restitution(0.0f);
	sphere->set_coefficient_of_restitution_override(true);
	sphere->set_coefficient_of_static_friction(0.0f);
	sphere->set_coefficient_of_dynamic_friction(0.0f);
	sphere->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	sphere->set_position({0.0f, 0.0f, 3.0f});
	sim.add_solid(sphere);

	// Run for 3 seconds
	for (int i = 0; i < 300; ++i) sim.update(10);

	// With COR=0, the sphere should come to rest near z=0.5 (radius)
	float z = sphere->get_position().z;
	float vz = sphere->get_velocity().z;
	printf("z=%.2f vz=%.2f ", z, vz);
	assert(z < 1.5f); // Should be resting
	assert(std::fabs(vz) < 1.0f); // Velocity should be small
	printf("OK\n");
}

// Test: deactivation (sleeping)
static void test_deactivation() {
	printf("  deactivation: ");
	simulator<float> sim;

	make_floor(sim);

	auto sphere = std::make_shared<solid<float>>();
	sphere->set_mass(1.0f);
	sphere->set_coefficient_of_restitution(0.0f);
	sphere->set_coefficient_of_restitution_override(true);
	sphere->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	sphere->set_position({0.0f, 0.0f, 2.0f});
	sim.add_solid(sphere);

	// Run until sphere deactivates
	for (int i = 0; i < 500; ++i) sim.update(10);

	bool active = sphere->active();
	printf("active=%d ", active);
	assert(!active); // Should have deactivated after coming to rest
	printf("OK\n");
}

// Test: scope filtering — non-matching scopes should not collide
static void test_scope_filtering() {
	printf("  scope_filtering: ");
	simulator<float> sim;
	sim.set_gravity({0, 0, 0});

	auto s1 = std::make_shared<solid<float>>();
	s1->set_mass(1.0f);
	s1->set_collision_scope(1);
	s1->set_collide_with_scope(1);
	s1->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s1->set_position({-2.0f, 0.0f, 0.0f});
	s1->set_velocity({3.0f, 0.0f, 0.0f});
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<float>>();
	s2->set_mass(1.0f);
	s2->set_collision_scope(2); // Different scope
	s2->set_collide_with_scope(2);
	s2->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s2->set_position({2.0f, 0.0f, 0.0f});
	s2->set_velocity({-3.0f, 0.0f, 0.0f});
	sim.add_solid(s2);

	for (int i = 0; i < 100; ++i) sim.update(10);

	// They should pass through each other (scopes don't match)
	float x1 = s1->get_position().x;
	float x2 = s2->get_position().x;
	printf("x1=%.2f x2=%.2f ", x1, x2);
	assert(x1 > 0.0f);  // s1 passed through to +x
	assert(x2 < 0.0f);  // s2 passed through to -x
	printf("OK\n");
}

// Test: constraint (spring)
static void test_constraint() {
	printf("  constraint: ");
	simulator<float> sim;
	sim.set_gravity({0, 0, 0});

	auto s1 = std::make_shared<solid<float>>();
	s1->set_mass(1.0f);
	s1->set_collide_with_scope(0);
	s1->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s1->set_position({-2.0f, 0.0f, 0.0f});
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<float>>();
	s2->set_mass(1.0f);
	s2->set_collide_with_scope(0);
	s2->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	s2->set_position({2.0f, 0.0f, 0.0f});
	sim.add_solid(s2);

	auto c = std::make_shared<constraint<float>>(s1, s2);
	c->set_spring_constant(10.0f);
	c->set_damping_constant(1.0f);
	sim.add_constraint(c);

	for (int i = 0; i < 200; ++i) sim.update(10);

	// Spring should pull them together
	float dist = std::fabs(s1->get_position().x - s2->get_position().x);
	printf("dist=%.2f ", dist);
	assert(dist < 4.0f); // Should be closer than initial 4.0
	printf("OK\n");
}

// Test: add/remove solid
static void test_add_remove_solid() {
	printf("  add_remove_solid: ");
	simulator<float> sim;
	sim.set_gravity({0, 0, 0});

	auto s1 = std::make_shared<solid<float>>();
	s1->set_mass(1.0f);
	s1->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	sim.add_solid(s1);
	assert(sim.get_num_solids() == 1);

	auto s2 = std::make_shared<solid<float>>();
	s2->set_mass(1.0f);
	s2->add_shape(std::make_shared<shape<float>>(hop::sphere<float>(0.5f)));
	sim.add_solid(s2);
	assert(sim.get_num_solids() == 2);

	sim.remove_solid(s1);
	assert(sim.get_num_solids() == 1);

	sim.remove_solid(s2);
	assert(sim.get_num_solids() == 0);

	printf("OK\n");
}

// Test: fixed16 box-floor collision
static void test_fixed16_collision() {
	printf("  fixed16_collision: ");
	using tr = scalar_traits<fixed16>;
	simulator<fixed16> sim;

	// Floor
	auto floor = std::make_shared<solid<fixed16>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(fixed16{});
	floor->set_coefficient_of_restitution(tr::from_milli(800));
	floor->add_shape(std::make_shared<shape<fixed16>>(aa_box<fixed16>(
		vec3<fixed16>(tr::from_int(-10), tr::from_int(-10), -tr::one()),
		vec3<fixed16>(tr::from_int(10), tr::from_int(10), fixed16{})
	)));
	sim.add_solid(floor);

	// Dropping box
	auto box = std::make_shared<solid<fixed16>>();
	box->set_mass(tr::one());
	box->set_coefficient_of_restitution(tr::from_milli(800));
	box->set_coefficient_of_restitution_override(true);
	box->set_coefficient_of_static_friction(fixed16{});
	box->set_coefficient_of_dynamic_friction(fixed16{});
	box->add_shape(std::make_shared<shape<fixed16>>(aa_box<fixed16>(
		vec3<fixed16>(-tr::half(), -tr::half(), -tr::half()),
		vec3<fixed16>(tr::half(), tr::half(), tr::half())
	)));
	box->set_position({fixed16{}, fixed16{}, tr::from_int(5)});
	sim.add_solid(box);

	for (int i = 0; i < 200; ++i) sim.update(10);

	float z = tr::to_float(box->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.3f); // Should be bouncing above floor
	printf("OK\n");
}

int main() {
	printf("test_collision:\n");
	test_sphere_floor_bounce();
	test_box_box_collision();
	test_sphere_sphere_collision();
	test_capsule_box_collision();
	test_inelastic_collision();
	test_deactivation();
	test_scope_filtering();
	test_constraint();
	test_add_remove_solid();
	test_fixed16_collision();
	printf("ALL PASSED\n");
	return 0;
}
