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
		sim->update(10);
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
			sim->update(10);
		}
		float z = tr::to_float(s->get_position().z);
		printf("  fixed16 gravity drop: z = %.3f (expected ~5.1)\n", z);
		assert(z > 3.0f && z < 7.0f); // More relaxed bounds for fixed16
	}

	printf("ALL PASSED\n");
	return 0;
}
