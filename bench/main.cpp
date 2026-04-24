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
	auto cs_uncached = make_unit_cube_convex<T>();
	auto cs_cached = make_unit_cube_convex<T>();
	rebuild_vertices(cs_cached);
	auto convex_un_shape = std::make_shared<shape<T>>(cs_uncached);
	auto convex_ca_shape = std::make_shared<shape<T>>(cs_cached);

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
		auto p = make_pair(convex_un_shape, convex_un_shape);
		bench::go("convex vs convex (uncached)", 10000, [&] {
			collision<T> r;
			sim->test_solid(r, p.s1.get(), seg, p.s2.get());
		});
	}
	{
		auto p = make_pair(convex_ca_shape, convex_ca_shape);
		bench::go("convex vs convex (cached vertices)", 10000, [&] {
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

int main() {
	printf("hop bench\n");
	printf("---------\n");

	bench_narrow_phase<float>("float");
	bench_narrow_phase<fixed16>("fixed16");

	bench_full_tick<float>("float");
	bench_full_tick<fixed16>("fixed16");

	printf("\ndone\n");
	return 0;
}
