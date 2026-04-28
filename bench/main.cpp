#include "bench.h"
#include <cstdio>
#include <hop/hop.h>
#include <memory>

using namespace hop;

// ----------------------------------------------------------------------------
// Scenario helpers
// ----------------------------------------------------------------------------

template <typename T> static std::shared_ptr<solid<T>> make_solid_with_shape(typename shape<T>::ptr sh, const vec3<T> & pos) {
	using tr = scalar_traits<T>;
	auto s = std::make_shared<solid<T>>();
	s->set_mass(tr::one());
	s->set_position(pos);
	s->add_shape(sh);
	return s;
}

template <typename T> static convex_solid<T> make_unit_cube_convex() {
	using tr = scalar_traits<T>;
	convex_solid<T> cs;
	cs.planes.push_back({ { tr::one(), T {}, T {} }, tr::one() });
	cs.planes.push_back({ { -tr::one(), T {}, T {} }, tr::one() });
	cs.planes.push_back({ { T {}, tr::one(), T {} }, tr::one() });
	cs.planes.push_back({ { T {}, -tr::one(), T {} }, tr::one() });
	cs.planes.push_back({ { T {}, T {}, tr::one() }, tr::one() });
	cs.planes.push_back({ { T {}, T {}, -tr::one() }, tr::one() });
	return cs;
}

// ----------------------------------------------------------------------------
// Scenario 1: per-pair narrow-phase sweeps.
// Isolates the shape-vs-shape cost by calling simulator::test_solid() directly.
// ----------------------------------------------------------------------------

template <typename T> static void bench_narrow_phase(const char * label) {
	using tr = scalar_traits<T>;
	printf("[narrow_phase %s]\n", label);

	auto sim = std::make_shared<simulator<T>>();

	// Unit shapes: sphere r=1, box [-1,1]^3, capsule along +Y r=0.5, convex cube.
	auto sph_shape = std::make_shared<shape<T>>(sphere<T>{ vec3<T>{}, tr::one() });
	auto box_shape = std::make_shared<shape<T>>(aa_box<T>{ tr::one() });
	auto cap_shape = std::make_shared<shape<T>>(capsule<T>{ vec3<T>{}, { T {}, tr::one(), T {} }, tr::half() });
	auto cs = make_unit_cube_convex<T>();
	auto convex_shape = std::make_shared<shape<T>>(cs);

	// A pre-built pair: solid1 at origin, solid2 offset +4 on X. Sweep +5 X.
	struct pair { std::shared_ptr<solid<T>> s1, s2; };
	auto make_pair = [&](typename shape<T>::ptr sh1, typename shape<T>::ptr sh2) -> pair {
		auto s1 = make_solid_with_shape<T>(sh1, vec3<T>{});
		auto s2 = make_solid_with_shape<T>(sh2, vec3<T>{ tr::from_int(4), T {}, T {} });
		sim->add_solid(s1);
		sim->add_solid(s2);
		return { s1, s2 };
	};

	// test_solid takes a segment (start, delta). Sweep s1 toward s2.
	segment<T> seg;
	seg.set_start_dir(vec3<T>{}, vec3<T>{ tr::from_int(5), T {}, T {} });

	{
		auto p = make_pair(sph_shape, box_shape);
		bench::go("sphere vs box", 100000, [&] {
			collision<T> r;
			sim->test_solid(r, p.s1.get(), seg, p.s2.get());
		});
	}
	{
		auto p = make_pair(cap_shape, cap_shape);
		bench::go("capsule vs capsule", 100000, [&] {
			collision<T> r;
			sim->test_solid(r, p.s1.get(), seg, p.s2.get());
		});
	}
	{
		auto p = make_pair(box_shape, box_shape);
		bench::go("box vs box", 100000, [&] {
			collision<T> r;
			sim->test_solid(r, p.s1.get(), seg, p.s2.get());
		});
	}
	{
		auto p = make_pair(convex_shape, convex_shape);
		bench::go("convex vs convex", 10000, [&] {
			collision<T> r;
			sim->test_solid(r, p.s1.get(), seg, p.s2.get());
		});
	}
}

// ----------------------------------------------------------------------------
// Scenario 2: full tick of a representative scene.
// 10 dynamic spheres bouncing inside a 6-wall box room under gravity.
// ----------------------------------------------------------------------------

template <typename T> static void bench_full_tick(const char * label) {
	using tr = scalar_traits<T>;
	printf("[full_tick %s]\n", label);

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });

	// Static walls: 6 thin boxes forming a 20×20×20 room centered at origin.
	T r = tr::from_int(10);
	T t = tr::half();  // wall thickness
	auto add_wall = [&](const vec3<T> & center, const vec3<T> & half_extent) {
		auto s = std::make_shared<solid<T>>();
		s->set_infinite_mass();
		s->set_position(center);
		s->add_shape(std::make_shared<shape<T>>(aa_box<T>{ -half_extent.x, -half_extent.y, -half_extent.z,
		                                                  half_extent.x, half_extent.y, half_extent.z }));
		sim->add_solid(s);
	};
	add_wall({ T {}, T {}, -r }, { r, r, t });  // floor
	add_wall({ T {}, T {},  r }, { r, r, t });  // ceiling
	add_wall({ -r, T {}, T {} }, { t, r, r });
	add_wall({  r, T {}, T {} }, { t, r, r });
	add_wall({ T {}, -r, T {} }, { r, t, r });
	add_wall({ T {},  r, T {} }, { r, t, r });

	// 10 dynamic spheres at staggered positions with small starting velocities.
	for (int i = 0; i < 10; ++i) {
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		T x = tr::from_int(i - 5);
		T z = tr::from_int((i % 3) * 2);
		s->set_position({ x, T {}, z });
		s->set_velocity({ tr::from_int((i & 1) ? 1 : -1), tr::from_int((i & 2) ? 1 : -1), T {} });
		s->add_shape(std::make_shared<shape<T>>(sphere<T>{ vec3<T>{}, tr::half() }));
		sim->add_solid(s);
	}

	// Measure ms/tick over a long run so the number is stable.
	bench::go("sim->update(10ms)", 5000, [&] { sim->update(10); });
}

// ----------------------------------------------------------------------------
// Scenario 3: stress — N dynamic spheres packed tightly in a small room.
// Exercises the broad phase (O(n²) linear fallback vs. bvh_manager) and the
// collision loop, since neighbor overlap is the normal state. Sweep N to see
// how tick cost scales, and run each N with and without BVH broad-phase.
// ----------------------------------------------------------------------------

template <typename T> static void setup_stress_scene(simulator<T> & sim, int n) {
	using tr = scalar_traits<T>;
	sim.set_gravity({ T {}, T {}, -tr::from_milli(9810) });

	// Small room so spheres stay packed.
	T r = tr::from_int(5);
	T t = tr::half();
	auto add_wall = [&](const vec3<T> & center, const vec3<T> & half_extent) {
		auto s = std::make_shared<solid<T>>();
		s->set_infinite_mass();
		s->set_position(center);
		s->add_shape(std::make_shared<shape<T>>(aa_box<T>{ -half_extent.x, -half_extent.y, -half_extent.z,
		                                                  half_extent.x, half_extent.y, half_extent.z }));
		sim.add_solid(s);
	};
	add_wall({ T {}, T {}, -r }, { r, r, t });
	add_wall({ T {}, T {},  r }, { r, r, t });
	add_wall({ -r, T {}, T {} }, { t, r, r });
	add_wall({  r, T {}, T {} }, { t, r, r });
	add_wall({ T {}, -r, T {} }, { r, t, r });
	add_wall({ T {},  r, T {} }, { r, t, r });

	// Grid of small spheres, radius 0.3 at 1-unit spacing — guaranteed overlap
	// as they settle under gravity.
	int side = 1;
	while (side * side * side < n) ++side;
	T half = tr::half();
	int placed = 0;
	for (int ix = 0; ix < side && placed < n; ++ix)
	for (int iy = 0; iy < side && placed < n; ++iy)
	for (int iz = 0; iz < side && placed < n; ++iz) {
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		T x = tr::from_int(ix) - tr::from_int(side) * half;
		T y = tr::from_int(iy) - tr::from_int(side) * half;
		T z = tr::from_int(iz) - tr::from_int(side) * half;
		s->set_position({ x, y, z });
		// Small asymmetric initial velocity so they jostle.
		s->set_velocity({ tr::from_int((placed & 1) ? 1 : -1),
		                  tr::from_int((placed & 2) ? 1 : -1),
		                  T {} });
		s->add_shape(std::make_shared<shape<T>>(sphere<T>{ vec3<T>{}, tr::from_milli(300) }));
		sim.add_solid(s);
		++placed;
	}
}

template <typename T> static void bench_stress(const char * label) {
	printf("[stress %s]\n", label);

	// Tune iterations so each sweep runs in ~1s even at the largest N.
	struct config { int n; int iters; };
	config configs[] = { { 50, 2000 }, { 100, 1000 }, { 200, 500 } };

	for (auto c : configs) {
		// Linear broad-phase (simulator's default O(n) fallback per active solid).
		{
			auto sim = std::make_shared<simulator<T>>();
			setup_stress_scene(*sim, c.n);
			char name[64];
			std::snprintf(name, sizeof(name), "N=%d linear", c.n);
			bench::go(name, c.iters, [&] { sim->update(10); });
		}
		// BVH broad-phase.
		{
			auto sim = std::make_shared<simulator<T>>();
			bvh_manager<T> mgr;
			sim->set_manager(&mgr);
			setup_stress_scene(*sim, c.n);
			// Register only the walls as static; dynamic spheres stay in the flat
			// list. The first 6 solids added are the walls — this matches the
			// order setup_stress_scene uses.
			for (int i = 0; i < sim->get_num_solids(); ++i)
				mgr.add_solid(sim->get_solid(i), i < 6);
			char name[64];
			std::snprintf(name, sizeof(name), "N=%d bvh", c.n);
			bench::go(name, c.iters, [&] { sim->update(10); });
		}
	}
}

// ----------------------------------------------------------------------------
// Scenario 4: compound narrow-phase.
// Two solids each with N sphere subshapes, swept toward each other. Every
// test_solid call iterates N×N shape pairs — a direct measurement of the
// shape-iteration hot path under load. If shape size (168 B) is cache-bound,
// more shapes per solid should hurt disproportionately.
// ----------------------------------------------------------------------------

template <typename T> static void bench_compound_narrow(const char * label) {
	using tr = scalar_traits<T>;
	printf("[compound_narrow %s]\n", label);

	auto sim = std::make_shared<simulator<T>>();

	auto build_compound = [&](int n_shapes, const vec3<T> & pos) {
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		s->set_position(pos);
		for (int k = 0; k < n_shapes; ++k) {
			auto sh = std::make_shared<shape<T>>(sphere<T>{ vec3<T>{}, tr::from_milli(100) });
			vec3<T> lp = { tr::from_milli(250) * tr::from_int(k), T {}, T {} };
			sh->set_local_position(lp);
			s->add_shape(sh);
		}
		sim->add_solid(s);
		return s;
	};

	segment<T> seg;
	seg.set_start_dir(vec3<T>{}, vec3<T>{ tr::from_int(5), T {}, T {} });

	for (int n : { 1, 4, 8, 16, 32 }) {
		auto s1 = build_compound(n, vec3<T>{});
		auto s2 = build_compound(n, vec3<T>{ tr::from_int(4), T {}, T {} });
		char name[64];
		std::snprintf(name, sizeof(name), "compound vs compound, %d×%d shapes", n, n);
		bench::go(name, 10000, [&] {
			collision<T> r;
			sim->test_solid(r, s1.get(), seg, s2.get());
		});
	}
}

// ----------------------------------------------------------------------------

int main() {
	printf("hop bench\n");
	printf("---------\n");
	printf("sizeof(shape<float>) = %zu bytes\n", sizeof(shape<float>));
	printf("sizeof(solid<float>) = %zu bytes\n", sizeof(solid<float>));
	printf("sizeof(simulator<float>) = %zu bytes\n", sizeof(simulator<float>));
	printf("\n");

	bench_narrow_phase<float>("float");
	bench_narrow_phase<fixed16>("fixed16");

	bench_full_tick<float>("float");
	bench_full_tick<fixed16>("fixed16");

	bench_stress<float>("float");
	bench_stress<fixed16>("fixed16");

	bench_compound_narrow<float>("float");
	bench_compound_narrow<fixed16>("fixed16");

	printf("\ndone\n");
	return 0;
}
