#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

template <typename T> static void test_gravity_drop() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });

	// Create a solid with a sphere shape
	auto s = std::make_shared<solid<T>>();
	s->set_mass(tr::one());
	s->set_position({ T {}, T {}, tr::from_int(10) });
	s->set_collide_with_scope(0); // No collision, pure freefall

	auto sh = std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() });
	s->add_shape(sh);

	sim->add_solid(s);

	// Update for 1 second (1000ms) in 10ms steps
	for (int i = 0; i < 100; ++i) {
		sim->update(tr::from_milli(10));
	}

	// Expected: z ≈ 10 - 0.5 * 9.81 * 1^2 = 10 - 4.905 = 5.095
	float z = tr::to_float(s->get_position().z);
	printf("  gravity drop: z = %.3f (expected ~5.1)\n", z);
	assert(z > 4.0f && z < 6.5f);
	printf("  gravity drop: OK\n");
}

template <typename T> static void test_trigger_scope() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, T {} });

	// Damage zone: a sphere tagged with trigger bit 0x4. Has zero collide_with
	// so it's a pass-through volume — physics objects don't bounce off it.
	auto zone = std::make_shared<solid<T>>();
	zone->set_infinite_mass();
	zone->set_position({ T {}, T {}, T {} });
	zone->set_trigger_scope(0x4);
	zone->set_collide_with_scope(0); // pure trigger
	zone->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::from_int(2) }));
	sim->add_solid(zone);

	// Player: a small sphere we'll move into the zone.
	auto player = std::make_shared<solid<T>>();
	player->set_mass(tr::one());
	player->set_position({ tr::from_int(5), T {}, T {} });
	player->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::half() }));
	sim->add_solid(player);

	// Static overlap query: player at (5,0,0), zone at origin r=2 — no overlap.
	collision<T> r;
	segment<T> probe;
	probe.set_start_dir(player->get_position(), { T {}, T {}, T {} });
	sim->trace_solid(r, player.get(), probe, -1);
	assert(r.trigger_scope == 0);
	printf("  trigger_scope outside: trigger_scope=0x%x (expected 0)\n", r.trigger_scope);

	// Move player to origin — now overlapping the zone.
	player->set_position({ T {}, T {}, T {} });
	r.reset();
	probe.set_start_dir(player->get_position(), { T {}, T {}, T {} });
	sim->trace_solid(r, player.get(), probe, -1);
	assert(r.trigger_scope == 0x4);
	printf("  trigger_scope inside: trigger_scope=0x%x (expected 0x4)\n", r.trigger_scope);
	printf("  trigger_scope: OK\n");
}

// A manager that injects a static floor plane at world z=0 as EXTERNAL geometry
// (NOT a solid in the simulator's list). Exercises the speculative pipeline's
// manager-query discovery path: the discovered contact has no owning solid, so
// it can only be resolved against the simulator's immovable world anchor.
template <typename T> class manager_floor : public hop::manager<T> {
	using tr = scalar_traits<T>;

public:
	int find_solids_in_aa_box(const aa_box<T> &, solid<T> *[], int) override { return -1; }
	void trace_segment(collision<T> &, const segment<T> &, int) override {}
	void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int, T margin) override {
		// Floor plane at z=0, inflated upward by the speculative margin so a body
		// resting within the margin registers as an overlap. Mirrors the recipe in
		// test_collision's traceable floor, but reports through the manager path.
		T surface_z = margin;
		T lowest_z = T {};
		for (auto & shape : s->get_shapes()) {
			aa_box<T> bound;
			shape->get_bound(bound);
			if (bound.mins.z < lowest_z)
				lowest_z = bound.mins.z;
		}
		T start_z = seg.origin.z + lowest_z;
		if (start_z <= surface_z) {
			if (result.time > T {}) {
				result.time = T {};
				result.point.set(seg.origin);
				result.normal = { T {}, T {}, tr::one() };
				result.depth = surface_z - start_z;
			}
			return;
		}
		T dz = seg.direction.z;
		if (dz >= T {})
			return;
		T t = (surface_z - start_z) / dz;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			mul(result.point, seg.direction, t);
			add(result.point, seg.origin);
			result.normal = { T {}, T {}, tr::one() };
		}
	}
	void pre_update(T) override {}
	void post_update(T) override {}
	void pre_update(solid<T> *, T) override {}
	void intra_update(solid<T> *, T) override {}
	bool collision_response(solid<T> *, vec3<T> &, vec3<T> &, collision<T> &) override { return false; }
	void post_update(solid<T> *, T) override {}
};

// Drop a sphere onto manager-injected (non-solid) floor geometry with the
// speculative pipeline on. The contact is discovered only via the manager query
// in integrate_and_discover and resolved against the world anchor — verifies the
// body rests on the surface (no tunnelling, no sink) and settles to sleep, which
// can only happen if the world-anchor touch counts as load-bearing support.
template <typename T> static void test_speculative_manager_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  speculative_manager_floor[%s]: ", label);

	manager_floor<T> floor;
	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	sim->set_manager(&floor);

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::from_int(5) });
	ball->set_coefficient_of_restitution(T {});  // no bounce: settle quickly
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	for (int i = 0; i < 300; ++i)
		sim->update(tr::from_milli(16));

	float z = tr::to_float(ball->get_position().z);
	float vz = tr::to_float(ball->get_velocity().z);
	bool asleep = !ball->active();
	printf("z=%.3f vz=%.4f asleep=%d (expected z~1.0)\n", z, vz, asleep ? 1 : 0);
	// Rests on the radius-1 sphere's contact with the z=0 plane: center near z=1.
	// Generous band to stay robust across float/fixed and the margin shell.
	assert(z > 0.9f && z < 1.15f);
	assert(std::fabs(vz) < 0.1f);
	assert(asleep);
	printf("  speculative_manager_floor[%s]: OK\n", label);
}

// Injects the same z=0 floor but CLAIMS every contact via collision_response.
// Verifies the speculative pipeline calls the hook and that a claimed contact is
// excluded from the solver: with no impulse applied, nothing stops the body.
template <typename T> class claiming_floor : public manager_floor<T> {
public:
	int response_calls = 0;
	bool collision_response(solid<T> *, vec3<T> &, vec3<T> &, collision<T> &) override {
		++response_calls;
		return true;  // claim it; do nothing, so the solver must not resolve it
	}
};

template <typename T> static void test_speculative_manager_response(const char * label) {
	using tr = scalar_traits<T>;
	printf("  speculative_manager_response[%s]: ", label);

	claiming_floor<T> floor;
	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	sim->set_manager(&floor);

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::from_int(3) });
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	for (int i = 0; i < 120; ++i)
		sim->update(tr::from_milli(16));

	float z = tr::to_float(ball->get_position().z);
	printf("response_calls=%d z=%.2f (expected hook fired, body fell through z<0)\n", floor.response_calls, z);
	assert(floor.response_calls > 0);  // the hook fires under the speculative pipeline
	assert(z < 0.0f);                  // claimed contact => no solver impulse => not stopped
	printf("  speculative_manager_response[%s]: OK\n", label);
}

// Mixed contact modes in one simulator: a finite-mass sweep_slide "character" is
// pushed by a speculative ball through the shared velocity solve. Verifies (a) the
// character is influenced by physics — it is shoved in +x from rest, the behavior a
// kinematic (inv_mass=0) character controller would NOT give — and (b) the ball
// does not tunnel through it. Exercises the per-body dispatch and the mixed-pair
// seam (sweep_slide owns its own position via the snap/slide; the impulse exchange
// still uses its real finite mass).
template <typename T> static void test_mixed_modes_push(const char * label) {
	using tr = scalar_traits<T>;
	printf("  mixed_modes_push[%s]: ", label);

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, T {} });  // zero-g: isolate the push

	// Character: sweep_slide, finite mass, at rest at the origin.
	auto character = std::make_shared<solid<T>>();
	character->set_mass(tr::one());
	character->set_position({ T {}, T {}, T {} });
	character->set_coefficient_of_restitution(T {});  // inelastic: clean momentum transfer
	character->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(character);
	character->set_contact_mode(contact_mode::sweep_slide);

	// Ball: speculative, finite mass, approaching from -x.
	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ tr::from_int(-4), T {}, T {} });
	ball->set_velocity({ tr::from_int(5), T {}, T {} });
	ball->set_coefficient_of_restitution(T {});
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);
	ball->set_contact_mode(contact_mode::speculative);

	for (int i = 0; i < 120; ++i)
		sim->update(tr::from_milli(16));

	float cx  = tr::to_float(character->get_position().x);
	float cvx = tr::to_float(character->get_velocity().x);
	float bx  = tr::to_float(ball->get_position().x);
	printf("char x=%.2f vx=%.3f  ball x=%.2f (char shoved +x, ball stays behind)\n", cx, cvx, bx);
	assert(cx > 0.1f);    // the sweep_slide character was pushed by the speculative ball
	assert(cvx > 0.0f);   // ... and is still carrying that motion
	assert(bx < cx);      // the ball never tunnelled past the character
	printf("  mixed_modes_push[%s]: OK\n", label);
}

template <typename T> static void test_dual_instantiation() {
	// Just verify both can be instantiated in the same TU
	simulator<T> sim;
	solid<T> s;
	shape<T> sh;
	constraint<T> c;
	collision<T> col;
	printf("  dual instantiation: OK\n");
}

int main() {
	printf("test_simulator (float):\n");
	test_gravity_drop<float>();
	test_trigger_scope<float>();
	test_speculative_manager_floor<float>("float");
	test_speculative_manager_response<float>("float");
	test_mixed_modes_push<float>("float");
	test_dual_instantiation<float>();

	printf("test_simulator (fixed16):\n");
	test_dual_instantiation<fixed16>();
	// Note: fixed16 gravity drop is more sensitive to overflow
	// with large position values, so we test it carefully
	{
		using tr = scalar_traits<fixed16>;
		auto sim = std::make_shared<simulator<fixed16>>();
		auto s = std::make_shared<solid<fixed16>>();
		s->set_mass(tr::one());
		s->set_position({ fixed16 {}, fixed16 {}, tr::from_int(10) });
		s->set_collide_with_scope(0);

		auto sh = std::make_shared<shape<fixed16>>(hop::sphere<fixed16> { vec3<fixed16> {}, tr::one() });
		s->add_shape(sh);
		sim->add_solid(s);

		for (int i = 0; i < 100; ++i) {
			sim->update(tr::from_milli(10));
		}
		float z = tr::to_float(s->get_position().z);
		printf("  fixed16 gravity drop: z = %.3f (expected ~5.1)\n", z);
		assert(z > 3.0f && z < 7.0f); // More relaxed bounds for fixed16
	}
	test_speculative_manager_floor<fixed16>("fixed16");
	test_speculative_manager_response<fixed16>("fixed16");
	test_mixed_modes_push<fixed16>("fixed16");

	printf("ALL PASSED\n");
	return 0;
}
