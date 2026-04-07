#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

static constexpr float EPS = 0.1f;

static bool approx(float a, float b, float tol = EPS) { return std::fabs(a - b) < tol; }

// A traceable floor at z=0 for testing traceable dispatch paths.
// Implements collision by tracing the other solid's bounding box lowest z
// extent against the floor plane.
template <typename T> class test_floor_traceable : public hop::traceable<T> {
	using tr = scalar_traits<T>;

public:
	void get_bound(aa_box<T> & result) override {
		result.mins = { tr::from_int(-10), tr::from_int(-10), -tr::one() };
		result.maxs = { tr::from_int(10), tr::from_int(10), T {} };
	}

	void trace_segment(collision<T> & result, const vec3<T> & position, const segment<T> & seg) override {
		T floor_z = position.z;
		T dz = seg.direction.z;
		if (dz >= T {})
			return;
		T t = (floor_z - seg.origin.z) / dz;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			mul(result.point, seg.direction, t);
			add(result.point, seg.origin);
			result.normal = { T {}, T {}, tr::one() };
		}
	}

	void trace_solid(collision<T> & result, solid<T> * s, const vec3<T> & position, const segment<T> & seg) override {
		T floor_z = position.z;
		// Find the solid's lowest z extent from its shapes
		T lowest_z = T {};
		for (int i = 0; i < s->get_num_shapes(); ++i) {
			aa_box<T> bound;
			s->get_shape(i)->get_bound(bound);
			if (bound.mins.z < lowest_z)
				lowest_z = bound.mins.z;
		}
		T start_z = seg.origin.z + lowest_z;
		T dz = seg.direction.z;
		if (dz >= T {})
			return;
		T t = (floor_z - start_z) / dz;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			mul(result.point, seg.direction, t);
			add(result.point, seg.origin);
			result.normal = { T {}, T {}, tr::one() };
		}
	}
};

// Helper: create a traceable floor at z=0
template <typename T>
static std::shared_ptr<solid<T>> make_traceable_floor(simulator<T> & sim, test_floor_traceable<T> & traceable) {
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T {});
	wall->set_coefficient_of_restitution(scalar_traits<T>::one());
	wall->add_shape(std::make_shared<shape<T>>(&traceable));
	sim.add_solid(wall);
	return wall;
}

// Helper: create a floor wall at z=0
template <typename T> static std::shared_ptr<solid<T>> make_floor(simulator<T> & sim) {
	using tr = scalar_traits<T>;
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T {});
	wall->set_coefficient_of_restitution(tr::one());
	wall->add_shape(std::make_shared<shape<T>>(aa_box<T>(vec3<T>(tr::from_int(-10), tr::from_int(-10), -tr::one()),
	                                                     vec3<T>(tr::from_int(10), tr::from_int(10), T {}))));
	sim.add_solid(wall);
	return wall;
}

// Helper: create a convex cube with given half-extent
template <typename T> static hop::convex_solid<T> make_convex_cube(T half) {
	using tr = scalar_traits<T>;
	T one = tr::one();
	T zero = T {};
	T neg_one = -one;
	hop::convex_solid<T> cs;
	cs.planes.push_back(hop::plane<T>(one, zero, zero, half));
	cs.planes.push_back(hop::plane<T>(neg_one, zero, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, one, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, neg_one, zero, half));
	cs.planes.push_back(hop::plane<T>(zero, zero, one, half));
	cs.planes.push_back(hop::plane<T>(zero, zero, neg_one, half));
	return cs;
}

// Helper: create a convex floor at z=0 as a convex_solid (thick slab)
template <typename T> static std::shared_ptr<solid<T>> make_convex_floor(simulator<T> & sim) {
	using tr = scalar_traits<T>;
	T one = tr::one();
	T zero = T {};
	T neg_one = -one;
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(zero);
	wall->set_coefficient_of_restitution(one);
	// Convex slab: 6 planes forming a 20x20x1 box centered at z=-0.5
	hop::convex_solid<T> cs;
	cs.planes.push_back(hop::plane<T>(zero, zero, one, zero));              // top face at z=0
	cs.planes.push_back(hop::plane<T>(zero, zero, neg_one, one));           // bottom face at z=-1
	cs.planes.push_back(hop::plane<T>(one, zero, zero, tr::from_int(10))); // +x
	cs.planes.push_back(hop::plane<T>(neg_one, zero, zero, tr::from_int(10))); // -x
	cs.planes.push_back(hop::plane<T>(zero, one, zero, tr::from_int(10))); // +y
	cs.planes.push_back(hop::plane<T>(zero, neg_one, zero, tr::from_int(10))); // -y
	wall->add_shape(std::make_shared<shape<T>>(cs));
	sim.add_solid(wall);
	return wall;
}

// Test: sphere drops onto floor and bounces (COR=1 should preserve energy)
template <typename T> static void test_sphere_floor_bounce(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_floor_bounce[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	auto sphere = std::make_shared<solid<T>>();
	sphere->set_mass(tr::one());
	sphere->set_coefficient_of_restitution(tr::one());
	sphere->set_coefficient_of_restitution_override(true);
	sphere->set_coefficient_of_static_friction(T {});
	sphere->set_coefficient_of_dynamic_friction(T {});
	sphere->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sphere->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(sphere);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(sphere->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.4f);
	printf("OK\n");
}

// Test: box-box collision — two boxes moving toward each other
template <typename T> static void test_box_box_collision(const char * label) {
	using tr = scalar_traits<T>;
	printf("  box_box_collision[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto box1 = std::make_shared<solid<T>>();
	box1->set_mass(tr::one());
	box1->set_coefficient_of_restitution(tr::one());
	box1->set_coefficient_of_restitution_override(true);
	box1->set_coefficient_of_static_friction(T {});
	box1->set_coefficient_of_dynamic_friction(T {});
	box1->add_shape(
	    std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	box1->set_position({ -tr::from_int(3), T {}, T {} });
	box1->set_velocity({ tr::from_int(2), T {}, T {} });
	sim.add_solid(box1);

	auto box2 = std::make_shared<solid<T>>();
	box2->set_mass(tr::one());
	box2->set_coefficient_of_restitution(tr::one());
	box2->set_coefficient_of_restitution_override(true);
	box2->set_coefficient_of_static_friction(T {});
	box2->set_coefficient_of_dynamic_friction(T {});
	box2->add_shape(
	    std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	box2->set_position({ tr::from_int(3), T {}, T {} });
	box2->set_velocity({ -tr::from_int(2), T {}, T {} });
	sim.add_solid(box2);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float v1x = tr::to_float(box1->get_velocity().x);
	float v2x = tr::to_float(box2->get_velocity().x);
	printf("v1x=%.2f v2x=%.2f ", v1x, v2x);
	assert(v1x < -1.0f);
	assert(v2x > 1.0f);
	printf("OK\n");
}

// Test: sphere-sphere collision
template <typename T> static void test_sphere_sphere_collision(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_sphere_collision[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->set_coefficient_of_restitution(tr::one());
	s1->set_coefficient_of_restitution_override(true);
	s1->set_coefficient_of_static_friction(T {});
	s1->set_coefficient_of_dynamic_friction(T {});
	s1->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s1->set_position({ -tr::from_int(3), T {}, T {} });
	s1->set_velocity({ tr::from_int(3), T {}, T {} });
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->set_coefficient_of_restitution(tr::one());
	s2->set_coefficient_of_restitution_override(true);
	s2->set_coefficient_of_static_friction(T {});
	s2->set_coefficient_of_dynamic_friction(T {});
	s2->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s2->set_position({ tr::from_int(3), T {}, T {} });
	s2->set_velocity({ -tr::from_int(3), T {}, T {} });
	sim.add_solid(s2);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float v1x = tr::to_float(s1->get_velocity().x);
	float v2x = tr::to_float(s2->get_velocity().x);
	printf("v1x=%.2f v2x=%.2f ", v1x, v2x);
	assert(v1x < -1.0f);
	assert(v2x > 1.0f);
	printf("OK\n");
}

// Test: capsule-box collision
template <typename T> static void test_capsule_box_collision(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_box_collision[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	auto cap = std::make_shared<solid<T>>();
	cap->set_mass(tr::one());
	cap->set_coefficient_of_restitution(tr::from_milli(800));
	cap->set_coefficient_of_restitution_override(true);
	cap->set_coefficient_of_static_friction(T {});
	cap->set_coefficient_of_dynamic_friction(T {});
	cap->add_shape(std::make_shared<shape<T>>(hop::capsule<T>(vec3<T>(), vec3<T>(T {}, T {}, tr::one()), tr::from_milli(300))));
	cap->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(cap);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(cap->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.2f);
	printf("OK\n");
}

// Test: restitution — COR=0 should stop bouncing
template <typename T> static void test_inelastic_collision(const char * label) {
	using tr = scalar_traits<T>;
	printf("  inelastic_collision[%s]: ", label);
	simulator<T> sim;

	auto floor = make_floor(sim);
	floor->set_coefficient_of_restitution(T {});

	auto sphere = std::make_shared<solid<T>>();
	sphere->set_mass(tr::one());
	sphere->set_coefficient_of_restitution(T {});
	sphere->set_coefficient_of_restitution_override(true);
	sphere->set_coefficient_of_static_friction(T {});
	sphere->set_coefficient_of_dynamic_friction(T {});
	sphere->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sphere->set_position({ T {}, T {}, tr::from_int(3) });
	sim.add_solid(sphere);

	for (int i = 0; i < 300; ++i)
		sim.update(10);

	float z = tr::to_float(sphere->get_position().z);
	float vz = tr::to_float(sphere->get_velocity().z);
	printf("z=%.2f vz=%.2f ", z, vz);
	assert(z < 1.5f);
	assert(std::fabs(vz) < 1.0f);
	printf("OK\n");
}

// Test: deactivation (sleeping)
template <typename T> static void test_deactivation(const char * label) {
	using tr = scalar_traits<T>;
	printf("  deactivation[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	auto sphere = std::make_shared<solid<T>>();
	sphere->set_mass(tr::one());
	sphere->set_coefficient_of_restitution(T {});
	sphere->set_coefficient_of_restitution_override(true);
	sphere->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sphere->set_position({ T {}, T {}, tr::from_int(2) });
	sim.add_solid(sphere);

	for (int i = 0; i < 500; ++i)
		sim.update(10);

	bool active = sphere->active();
	printf("active=%d ", active);
	assert(!active);
	printf("OK\n");
}

// Test: scope filtering — non-matching scopes should not collide
template <typename T> static void test_scope_filtering(const char * label) {
	using tr = scalar_traits<T>;
	printf("  scope_filtering[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->set_collision_scope(1);
	s1->set_collide_with_scope(1);
	s1->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s1->set_position({ -tr::from_int(2), T {}, T {} });
	s1->set_velocity({ tr::from_int(3), T {}, T {} });
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->set_collision_scope(2);
	s2->set_collide_with_scope(2);
	s2->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s2->set_position({ tr::from_int(2), T {}, T {} });
	s2->set_velocity({ -tr::from_int(3), T {}, T {} });
	sim.add_solid(s2);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float x1 = tr::to_float(s1->get_position().x);
	float x2 = tr::to_float(s2->get_position().x);
	printf("x1=%.2f x2=%.2f ", x1, x2);
	assert(x1 > 0.0f);
	assert(x2 < 0.0f);
	printf("OK\n");
}

// Test: constraint (spring)
template <typename T> static void test_constraint(const char * label) {
	using tr = scalar_traits<T>;
	printf("  constraint[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->set_collide_with_scope(0);
	s1->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s1->set_position({ -tr::from_int(2), T {}, T {} });
	sim.add_solid(s1);

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->set_collide_with_scope(0);
	s2->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	s2->set_position({ tr::from_int(2), T {}, T {} });
	sim.add_solid(s2);

	auto c = std::make_shared<constraint<T>>(s1, s2);
	c->set_spring_constant(tr::from_int(10));
	c->set_damping_constant(tr::one());
	sim.add_constraint(c);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float dist = std::fabs(tr::to_float(s1->get_position().x) - tr::to_float(s2->get_position().x));
	printf("dist=%.2f ", dist);
	assert(dist < 4.0f);
	printf("OK\n");
}

// Test: add/remove solid
template <typename T> static void test_add_remove_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  add_remove_solid[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sim.add_solid(s1);
	assert(sim.get_num_solids() == 1);

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sim.add_solid(s2);
	assert(sim.get_num_solids() == 2);

	sim.remove_solid(s1);
	assert(sim.get_num_solids() == 1);

	sim.remove_solid(s2);
	assert(sim.get_num_solids() == 0);

	printf("OK\n");
}

// Test: impact point for sphere dropping onto floor
template <typename T> static void test_impact_sphere_on_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  impact_sphere_on_floor[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	collision<T> last_col;
	bool got = false;

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(tr::one());
	sph->set_coefficient_of_restitution(tr::one());
	sph->set_coefficient_of_restitution_override(true);
	sph->set_coefficient_of_static_friction(T {});
	sph->set_coefficient_of_dynamic_friction(T {});
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sph->set_position({ T {}, T {}, tr::from_int(2) });
	sph->set_collision_callback([&](const collision<T> & c) {
		last_col.set(c);
		got = true;
	});
	sim.add_solid(sph);

	for (int i = 0; i < 200 && !got; ++i)
		sim.update(10, simulator<T>::scope_report_collisions);

	assert(got);
	float pz = tr::to_float(last_col.point.z);
	float iz = tr::to_float(last_col.impact.z);
	printf("point.z=%.2f impact.z=%.2f ", pz, iz);
	assert(approx(pz, 0.5f, 0.15f));
	assert(approx(iz, 0.0f, 0.15f));
	printf("OK\n");
}

// Test: impact point for segment trace (should equal point)
template <typename T> static void test_impact_segment_trace(const char * label) {
	using tr = scalar_traits<T>;
	printf("  impact_segment_trace[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	collision<T> result;
	segment<T> seg;
	seg.origin = { T {}, T {}, tr::from_int(5) };
	seg.direction = { T {}, T {}, -tr::from_int(10) };
	sim.trace_segment(result, seg);

	float pz = tr::to_float(result.point.z);
	float iz = tr::to_float(result.impact.z);
	printf("point.z=%.2f impact.z=%.2f ", pz, iz);
	assert(approx(pz, 0.0f, 0.01f));
	assert(approx(iz, pz, 0.001f));
	printf("OK\n");
}

// Test: impact point for box-on-floor collision
template <typename T> static void test_impact_box_on_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  impact_box_on_floor[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	collision<T> last_col;
	bool got = false;

	auto box = std::make_shared<solid<T>>();
	box->set_mass(tr::one());
	box->set_coefficient_of_restitution(tr::one());
	box->set_coefficient_of_restitution_override(true);
	box->set_coefficient_of_static_friction(T {});
	box->set_coefficient_of_dynamic_friction(T {});
	box->add_shape(
	    std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	box->set_position({ T {}, T {}, tr::from_int(3) });
	box->set_collision_callback([&](const collision<T> & c) {
		last_col.set(c);
		got = true;
	});
	sim.add_solid(box);

	for (int i = 0; i < 200 && !got; ++i)
		sim.update(10, simulator<T>::scope_report_collisions);

	assert(got);
	float pz = tr::to_float(last_col.point.z);
	float iz = tr::to_float(last_col.impact.z);
	printf("point.z=%.2f impact.z=%.2f ", pz, iz);
	assert(approx(pz, 0.5f, 0.15f));
	assert(approx(iz, 0.0f, 0.15f));
	printf("OK\n");
}

// Test: sphere-capsule collision — sphere hits a static capsule head-on
template <typename T> static void test_sphere_capsule_collision(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_capsule_collision[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto cap = std::make_shared<solid<T>>();
	cap->set_infinite_mass();
	cap->set_coefficient_of_gravity(T {});
	cap->set_coefficient_of_restitution(tr::one());
	cap->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, -tr::one()), vec3<T>(T {}, T {}, tr::from_int(2)), tr::half())));
	cap->set_position({ T {}, T {}, T {} });
	sim.add_solid(cap);

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(tr::one());
	sph->set_coefficient_of_restitution(tr::one());
	sph->set_coefficient_of_restitution_override(true);
	sph->set_coefficient_of_static_friction(T {});
	sph->set_coefficient_of_dynamic_friction(T {});
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sph->set_position({ tr::from_int(5), T {}, T {} });
	sph->set_velocity({ -tr::from_int(5), T {}, T {} });
	sim.add_solid(sph);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float vx = tr::to_float(sph->get_velocity().x);
	printf("vx=%.2f ", vx);
	assert(vx > 1.0f);
	printf("OK\n");
}

// Test: two perpendicular capsules — one drops onto the other's midpoint.
template <typename T> static void test_capsule_capsule_perpendicular(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_capsule_perpendicular[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto c1 = std::make_shared<solid<T>>();
	c1->set_infinite_mass();
	c1->set_coefficient_of_gravity(T {});
	c1->set_coefficient_of_restitution(tr::one());
	c1->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(-tr::from_int(2), T {}, T {}), vec3<T>(tr::from_int(4), T {}, T {}), tr::from_milli(300))));
	c1->set_position({ T {}, T {}, T {} });
	sim.add_solid(c1);

	auto c2 = std::make_shared<solid<T>>();
	c2->set_mass(tr::one());
	c2->set_coefficient_of_restitution(tr::one());
	c2->set_coefficient_of_restitution_override(true);
	c2->set_coefficient_of_static_friction(T {});
	c2->set_coefficient_of_dynamic_friction(T {});
	c2->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, -tr::from_int(2), T {}), vec3<T>(T {}, tr::from_int(4), T {}), tr::from_milli(300))));
	c2->set_position({ T {}, T {}, tr::from_int(3) });
	c2->set_velocity({ T {}, T {}, -tr::from_int(5) });
	sim.add_solid(c2);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float z = tr::to_float(c2->get_position().z);
	float vz = tr::to_float(c2->get_velocity().z);
	printf("z=%.2f vz=%.2f ", z, vz);
	assert(z > 0.3f);
	assert(vz > 0.0f);
	printf("OK\n");
}

// Test: two parallel capsules head-on
template <typename T> static void test_capsule_capsule_parallel(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_capsule_parallel[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto c1 = std::make_shared<solid<T>>();
	c1->set_mass(tr::one());
	c1->set_coefficient_of_restitution(tr::one());
	c1->set_coefficient_of_restitution_override(true);
	c1->set_coefficient_of_static_friction(T {});
	c1->set_coefficient_of_dynamic_friction(T {});
	c1->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, -tr::one()), vec3<T>(T {}, T {}, tr::from_int(2)), tr::half())));
	c1->set_position({ -tr::from_int(5), T {}, T {} });
	c1->set_velocity({ tr::from_int(3), T {}, T {} });
	sim.add_solid(c1);

	auto c2 = std::make_shared<solid<T>>();
	c2->set_mass(tr::one());
	c2->set_coefficient_of_restitution(tr::one());
	c2->set_coefficient_of_restitution_override(true);
	c2->set_coefficient_of_static_friction(T {});
	c2->set_coefficient_of_dynamic_friction(T {});
	c2->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, -tr::one()), vec3<T>(T {}, T {}, tr::from_int(2)), tr::half())));
	c2->set_position({ tr::from_int(5), T {}, T {} });
	c2->set_velocity({ -tr::from_int(3), T {}, T {} });
	sim.add_solid(c2);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float v1x = tr::to_float(c1->get_velocity().x);
	float v2x = tr::to_float(c2->get_velocity().x);
	printf("v1x=%.2f v2x=%.2f ", v1x, v2x);
	assert(v1x < -1.0f);
	assert(v2x > 1.0f);
	printf("OK\n");
}

// Test: ray (segment trace) hits a convex_solid wall
template <typename T> static void test_ray_convex_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  ray_convex_solid[%s]: ", label);
	simulator<T> sim;

	make_convex_floor(sim);

	collision<T> result;
	segment<T> seg;
	seg.origin = { T {}, T {}, tr::from_int(5) };
	seg.direction = { T {}, T {}, -tr::from_int(10) };
	sim.trace_segment(result, seg);

	float time = tr::to_float(result.time);
	float pz = tr::to_float(result.point.z);
	float nz = tr::to_float(result.normal.z);
	printf("time=%.3f point.z=%.2f ", time, pz);
	assert(time < 1.0f);
	assert(approx(pz, 0.0f, 0.01f));
	assert(approx(nz, 1.0f, 0.01f));
	printf("OK\n");
}

// Test: ray misses a convex_solid (parallel to surface)
template <typename T> static void test_ray_convex_solid_miss(const char * label) {
	using tr = scalar_traits<T>;
	printf("  ray_convex_solid_miss[%s]: ", label);
	simulator<T> sim;

	make_convex_floor(sim);

	collision<T> result;
	segment<T> seg;
	seg.origin = { T {}, T {}, tr::from_int(5) };
	seg.direction = { tr::from_int(10), T {}, T {} };
	sim.trace_segment(result, seg);

	float time = tr::to_float(result.time);
	printf("time=%.3f ", time);
	assert(time >= 1.0f);
	printf("OK\n");
}

// Test: sphere drops onto convex_solid floor and bounces
template <typename T> static void test_sphere_convex_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_convex_solid[%s]: ", label);
	simulator<T> sim;

	make_convex_floor(sim);

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(tr::one());
	sph->set_coefficient_of_restitution(tr::one());
	sph->set_coefficient_of_restitution_override(true);
	sph->set_coefficient_of_static_friction(T {});
	sph->set_coefficient_of_dynamic_friction(T {});
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sph->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(sph);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(sph->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.4f);
	printf("OK\n");
}

// Test: capsule drops onto convex_solid floor
template <typename T> static void test_capsule_convex_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_convex_solid[%s]: ", label);
	simulator<T> sim;

	make_convex_floor(sim);

	auto cap = std::make_shared<solid<T>>();
	cap->set_mass(tr::one());
	cap->set_coefficient_of_restitution(tr::from_milli(800));
	cap->set_coefficient_of_restitution_override(true);
	cap->set_coefficient_of_static_friction(T {});
	cap->set_coefficient_of_dynamic_friction(T {});
	cap->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, T {}), vec3<T>(T {}, T {}, tr::one()), tr::from_milli(300))));
	cap->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(cap);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(cap->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.2f);
	printf("OK\n");
}

// Test: aa_box drops onto convex_solid floor
template <typename T> static void test_box_convex_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  box_convex_solid[%s]: ", label);
	simulator<T> sim;

	make_convex_floor(sim);

	auto box = std::make_shared<solid<T>>();
	box->set_mass(tr::one());
	box->set_coefficient_of_restitution(tr::from_milli(800));
	box->set_coefficient_of_restitution_override(true);
	box->set_coefficient_of_static_friction(T {});
	box->set_coefficient_of_dynamic_friction(T {});
	box->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	box->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(box);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(box->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.3f);
	printf("OK\n");
}

// Test: sphere hits an angled convex wall (wedge shape)
template <typename T> static void test_sphere_convex_wedge(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_convex_wedge[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	T one = tr::one();
	T zero = T {};
	T neg_one = -one;

	auto wedge = std::make_shared<solid<T>>();
	wedge->set_infinite_mass();
	wedge->set_coefficient_of_gravity(zero);
	wedge->set_coefficient_of_restitution(one);
	hop::convex_solid<T> cs;
	T n = tr::from_milli(707); // ~1/sqrt(2)
	cs.planes.push_back(hop::plane<T>(n, zero, n, zero));          // angled face through origin
	cs.planes.push_back(hop::plane<T>(neg_one, zero, zero, tr::from_int(5)));  // back
	cs.planes.push_back(hop::plane<T>(zero, zero, neg_one, tr::from_int(5)));  // bottom
	cs.planes.push_back(hop::plane<T>(zero, one, zero, tr::from_int(10)));     // +y
	cs.planes.push_back(hop::plane<T>(zero, neg_one, zero, tr::from_int(10))); // -y
	wedge->add_shape(std::make_shared<shape<T>>(cs));
	sim.add_solid(wedge);

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(one);
	sph->set_coefficient_of_restitution(one);
	sph->set_coefficient_of_restitution_override(true);
	sph->set_coefficient_of_static_friction(zero);
	sph->set_coefficient_of_dynamic_friction(zero);
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::from_milli(300))));
	sph->set_position({ tr::from_int(3), zero, tr::from_int(3) });
	sph->set_velocity({ -tr::from_int(5), zero, -tr::from_int(5) });
	sim.add_solid(sph);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float vx = tr::to_float(sph->get_velocity().x);
	float vz = tr::to_float(sph->get_velocity().z);
	printf("vx=%.2f vz=%.2f ", vx, vz);
	assert(vx > 1.0f || vz > 1.0f);
	printf("OK\n");
}

// Test: moving convex_solid drops onto a static aa_box floor
template <typename T> static void test_convex_solid_vs_box(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_vs_box[%s]: ", label);
	simulator<T> sim;

	make_floor(sim);

	auto conv = std::make_shared<solid<T>>();
	conv->set_mass(tr::one());
	conv->set_coefficient_of_restitution(tr::from_milli(800));
	conv->set_coefficient_of_restitution_override(true);
	conv->set_coefficient_of_static_friction(T {});
	conv->set_coefficient_of_dynamic_friction(T {});
	conv->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	conv->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(conv);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(conv->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.3f);
	printf("OK\n");
}

// Test: moving convex_solid hits a static sphere
template <typename T> static void test_convex_solid_vs_sphere(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_vs_sphere[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto sph = std::make_shared<solid<T>>();
	sph->set_infinite_mass();
	sph->set_coefficient_of_gravity(T {});
	sph->set_coefficient_of_restitution(tr::one());
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::one())));
	sph->set_position({ T {}, T {}, T {} });
	sim.add_solid(sph);

	auto conv = std::make_shared<solid<T>>();
	conv->set_mass(tr::one());
	conv->set_coefficient_of_restitution(tr::one());
	conv->set_coefficient_of_restitution_override(true);
	conv->set_coefficient_of_static_friction(T {});
	conv->set_coefficient_of_dynamic_friction(T {});
	conv->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	conv->set_position({ tr::from_int(5), T {}, T {} });
	conv->set_velocity({ -tr::from_int(5), T {}, T {} });
	sim.add_solid(conv);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float vx = tr::to_float(conv->get_velocity().x);
	printf("vx=%.2f ", vx);
	assert(vx > 1.0f);
	printf("OK\n");
}

// Test: moving convex_solid hits a static capsule
template <typename T> static void test_convex_solid_vs_capsule(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_vs_capsule[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto cap = std::make_shared<solid<T>>();
	cap->set_infinite_mass();
	cap->set_coefficient_of_gravity(T {});
	cap->set_coefficient_of_restitution(tr::one());
	cap->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, -tr::one()), vec3<T>(T {}, T {}, tr::from_int(2)), tr::half())));
	cap->set_position({ T {}, T {}, T {} });
	sim.add_solid(cap);

	auto conv = std::make_shared<solid<T>>();
	conv->set_mass(tr::one());
	conv->set_coefficient_of_restitution(tr::one());
	conv->set_coefficient_of_restitution_override(true);
	conv->set_coefficient_of_static_friction(T {});
	conv->set_coefficient_of_dynamic_friction(T {});
	conv->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	conv->set_position({ tr::from_int(5), T {}, T {} });
	conv->set_velocity({ -tr::from_int(5), T {}, T {} });
	sim.add_solid(conv);

	for (int i = 0; i < 100; ++i)
		sim.update(10);

	float vx = tr::to_float(conv->get_velocity().x);
	printf("vx=%.2f ", vx);
	assert(vx > 1.0f);
	printf("OK\n");
}

// Test: two convex solids colliding — not implemented, should pass through
template <typename T> static void test_convex_solid_vs_convex_solid(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_vs_convex_solid[%s]: ", label);
	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto c1 = std::make_shared<solid<T>>();
	c1->set_mass(tr::one());
	c1->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	c1->set_position({ -tr::from_int(5), T {}, T {} });
	c1->set_velocity({ tr::from_int(3), T {}, T {} });
	sim.add_solid(c1);

	auto c2 = std::make_shared<solid<T>>();
	c2->set_mass(tr::one());
	c2->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	c2->set_position({ tr::from_int(5), T {}, T {} });
	c2->set_velocity({ -tr::from_int(3), T {}, T {} });
	sim.add_solid(c2);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float x1 = tr::to_float(c1->get_position().x);
	float x2 = tr::to_float(c2->get_position().x);
	printf("x1=%.2f x2=%.2f (pass-through expected) ", x1, x2);
	printf("OK\n");
}

// Test: sphere drops onto a traceable floor and bounces
template <typename T> static void test_sphere_traceable_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  sphere_traceable_floor[%s]: ", label);
	test_floor_traceable<T> traceable;
	simulator<T> sim;

	make_traceable_floor(sim, traceable);

	auto sph = std::make_shared<solid<T>>();
	sph->set_mass(tr::one());
	sph->set_coefficient_of_restitution(tr::one());
	sph->set_coefficient_of_restitution_override(true);
	sph->set_coefficient_of_static_friction(T {});
	sph->set_coefficient_of_dynamic_friction(T {});
	sph->add_shape(std::make_shared<shape<T>>(hop::sphere<T>(tr::half())));
	sph->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(sph);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(sph->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.4f);
	printf("OK\n");
}

// Test: box drops onto a traceable floor and bounces
template <typename T> static void test_box_traceable_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  box_traceable_floor[%s]: ", label);
	test_floor_traceable<T> traceable;
	simulator<T> sim;

	make_traceable_floor(sim, traceable);

	auto box = std::make_shared<solid<T>>();
	box->set_mass(tr::one());
	box->set_coefficient_of_restitution(tr::one());
	box->set_coefficient_of_restitution_override(true);
	box->set_coefficient_of_static_friction(T {});
	box->set_coefficient_of_dynamic_friction(T {});
	box->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	box->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(box);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(box->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.3f);
	printf("OK\n");
}

// Test: capsule drops onto a traceable floor and bounces
template <typename T> static void test_capsule_traceable_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  capsule_traceable_floor[%s]: ", label);
	test_floor_traceable<T> traceable;
	simulator<T> sim;

	make_traceable_floor(sim, traceable);

	auto cap = std::make_shared<solid<T>>();
	cap->set_mass(tr::one());
	cap->set_coefficient_of_restitution(tr::from_milli(800));
	cap->set_coefficient_of_restitution_override(true);
	cap->set_coefficient_of_static_friction(T {});
	cap->set_coefficient_of_dynamic_friction(T {});
	cap->add_shape(std::make_shared<shape<T>>(
	    hop::capsule<T>(vec3<T>(T {}, T {}, T {}), vec3<T>(T {}, T {}, tr::one()), tr::from_milli(300))));
	cap->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(cap);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(cap->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.2f);
	printf("OK\n");
}

// Test: convex_solid drops onto a traceable floor and bounces
template <typename T> static void test_convex_solid_traceable_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_traceable_floor[%s]: ", label);
	test_floor_traceable<T> traceable;
	simulator<T> sim;

	make_traceable_floor(sim, traceable);

	auto conv = std::make_shared<solid<T>>();
	conv->set_mass(tr::one());
	conv->set_coefficient_of_restitution(tr::from_milli(800));
	conv->set_coefficient_of_restitution_override(true);
	conv->set_coefficient_of_static_friction(T {});
	conv->set_coefficient_of_dynamic_friction(T {});
	conv->add_shape(std::make_shared<shape<T>>(make_convex_cube(tr::half())));
	conv->set_position({ T {}, T {}, tr::from_int(5) });
	sim.add_solid(conv);

	for (int i = 0; i < 200; ++i)
		sim.update(10);

	float z = tr::to_float(conv->get_position().z);
	printf("z=%.2f ", z);
	assert(z > 0.3f);
	printf("OK\n");
}

// Test: segment trace against a traceable floor
template <typename T> static void test_ray_traceable_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  ray_traceable_floor[%s]: ", label);
	test_floor_traceable<T> traceable;
	simulator<T> sim;

	make_traceable_floor(sim, traceable);

	collision<T> result;
	segment<T> seg;
	seg.origin = { T {}, T {}, tr::from_int(5) };
	seg.direction = { T {}, T {}, -tr::from_int(10) };
	sim.trace_segment(result, seg);

	float time = tr::to_float(result.time);
	float pz = tr::to_float(result.point.z);
	printf("time=%.3f point.z=%.2f ", time, pz);
	assert(time < 1.0f);
	assert(approx(pz, 0.0f, 0.01f));
	printf("OK\n");
}

template <typename T> static void run_all_tests(const char * label) {
	printf(" [%s]\n", label);
	test_sphere_floor_bounce<T>(label);
	test_box_box_collision<T>(label);
	test_sphere_sphere_collision<T>(label);
	test_capsule_box_collision<T>(label);
	test_inelastic_collision<T>(label);
	test_deactivation<T>(label);
	test_scope_filtering<T>(label);
	test_constraint<T>(label);
	test_add_remove_solid<T>(label);
	test_impact_sphere_on_floor<T>(label);
	test_impact_segment_trace<T>(label);
	test_impact_box_on_floor<T>(label);
	test_sphere_capsule_collision<T>(label);
	test_capsule_capsule_perpendicular<T>(label);
	test_capsule_capsule_parallel<T>(label);
	test_ray_convex_solid<T>(label);
	test_ray_convex_solid_miss<T>(label);
	test_sphere_convex_solid<T>(label);
	test_capsule_convex_solid<T>(label);
	test_box_convex_solid<T>(label);
	test_sphere_convex_wedge<T>(label);
	test_convex_solid_vs_box<T>(label);
	test_convex_solid_vs_sphere<T>(label);
	test_convex_solid_vs_capsule<T>(label);
	test_convex_solid_vs_convex_solid<T>(label);
	test_sphere_traceable_floor<T>(label);
	test_box_traceable_floor<T>(label);
	test_capsule_traceable_floor<T>(label);
	test_convex_solid_traceable_floor<T>(label);
	test_ray_traceable_floor<T>(label);
}

// Test: collision filter prevents two spheres from colliding
template <typename T> static void test_collision_filter(const char * label) {
	using tr = scalar_traits<T>;
	printf("  collision_filter[%s]: ", label);

	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	// Two spheres heading toward each other
	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->set_coefficient_of_restitution(tr::one());
	s1->add_shape(std::make_shared<shape<T>>(sphere<T>({ T {}, T {}, T {} }, tr::one())));
	s1->set_position({ -tr::from_int(3), T {}, T {} });
	s1->set_velocity({ tr::from_int(10), T {}, T {} });

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->set_coefficient_of_restitution(tr::one());
	s2->add_shape(std::make_shared<shape<T>>(sphere<T>({ T {}, T {}, T {} }, tr::one())));
	s2->set_position({ tr::from_int(3), T {}, T {} });
	s2->set_velocity({ -tr::from_int(10), T {}, T {} });

	// Set filter: s1 rejects s2
	auto * s2_raw = s2.get();
	s1->set_collision_filter([s2_raw](solid<T> * other) -> bool {
		return other != s2_raw;
	});

	sim.add_solid(s1);
	sim.add_solid(s2);

	// Step enough for them to pass through each other
	for (int i = 0; i < 20; ++i)
		sim.update(16);

	// Without filter they'd bounce apart. With filter, s1 passes through s2.
	// s1 started at x=-3 moving +10, after 320ms should be around x=-3+3.2=0.2
	// If they collided, s1 would be moving in -x direction.
	float v1x = tr::to_float(s1->get_velocity().x);
	printf("v1x=%.2f ", v1x);
	assert(v1x > 0.0f); // Still moving in +x, didn't bounce

	printf("OK\n");
}

// Test: collision filter is bidirectional (both must agree)
template <typename T> static void test_collision_filter_bidirectional(const char * label) {
	using tr = scalar_traits<T>;
	printf("  collision_filter_bidi[%s]: ", label);

	simulator<T> sim;
	sim.set_gravity({ T {}, T {}, T {} });

	auto s1 = std::make_shared<solid<T>>();
	s1->set_mass(tr::one());
	s1->set_coefficient_of_restitution(tr::one());
	s1->add_shape(std::make_shared<shape<T>>(sphere<T>({ T {}, T {}, T {} }, tr::one())));
	s1->set_position({ -tr::from_int(3), T {}, T {} });
	s1->set_velocity({ tr::from_int(10), T {}, T {} });

	auto s2 = std::make_shared<solid<T>>();
	s2->set_mass(tr::one());
	s2->set_coefficient_of_restitution(tr::one());
	s2->add_shape(std::make_shared<shape<T>>(sphere<T>({ T {}, T {}, T {} }, tr::one())));
	s2->set_position({ tr::from_int(3), T {}, T {} });
	s2->set_velocity({ -tr::from_int(10), T {}, T {} });

	// Only s2 rejects s1 — should still prevent collision since both must agree
	auto * s1_raw = s1.get();
	s2->set_collision_filter([s1_raw](solid<T> * other) -> bool {
		return other != s1_raw;
	});

	sim.add_solid(s1);
	sim.add_solid(s2);

	for (int i = 0; i < 20; ++i)
		sim.update(16);

	float v1x = tr::to_float(s1->get_velocity().x);
	printf("v1x=%.2f ", v1x);
	assert(v1x > 0.0f); // Still moving in +x

	printf("OK\n");
}

template <typename T> static void run_filter_tests(const char * label) {
	test_collision_filter<T>(label);
	test_collision_filter_bidirectional<T>(label);
}

int main() {
	printf("test_collision:\n");
	run_all_tests<float>("float");
	run_all_tests<fixed16>("fixed16");
	printf("test_collision_filter:\n");
	run_filter_tests<float>("float");
	run_filter_tests<fixed16>("fixed16");
	printf("ALL PASSED\n");
	return 0;
}
