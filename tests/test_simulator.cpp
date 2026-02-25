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
