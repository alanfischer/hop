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
	bench::go("sim->update(10ms)", 5000, [&] { sim->update(tr::from_milli(10)); });
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
	using tr = scalar_traits<T>;
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
			bench::go(name, c.iters, [&] { sim->update(tr::from_milli(10)); });
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
			const auto & solids = sim->get_solids();
			for (size_t i = 0; i < solids.size(); ++i)
				mgr.add_solid(solids[i].get(), i < 6);
			char name[64];
			std::snprintf(name, sizeof(name), "N=%d bvh", c.n);
			bench::go(name, c.iters, [&] { sim->update(tr::from_milli(10)); });
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
// Scenario 5: manifold piles — boxes stacked on a floor, the only scene shape
// that fills the manifold provisioning. The stress scene above is spheres, and a
// sphere contact is a single point, so nothing there ever exercises the 12 slots
// x 4 points every solid carries (3360 of solid<float>'s 3864 bytes).
//
// It reports what the pile actually uses alongside the tick cost, because the two
// together are the answer to "is the provisioning worth slimming?". A 6x6x6 pile
// keeps 88% of its slots live, half of them at all 12, and fills about half the
// points provisioned inside them — so the worst case is real for a pile, even though
// a ragdoll never approaches it (its bones sit at one slot with one point 96% of the
// time, and 168 of them walk ~650 KB to say so).
//
// The footprint does not convert into time, though, which is the finding this
// scenario exists to keep honest. Measured on an M1 Max (12 MB L2): padding
// touch::point by 60 bytes (solid<float> 3864 -> 6744) and contact_pair by 64
// (192 -> 256) — the duplication a per-pair header plus slim per-point rows would
// remove — moved nothing. This pile, the 200-sphere stress and demo_ragdoll's 168
// bones all landed inside run noise, and so did a 3136-box pile whose 11.6 MB of
// solids do not fit in L2 at all (padded to 20.2 MB it was, if anything, marginally
// faster). These passes are bound by narrow-phase math, not by bandwidth. Re-run
// that padding control on a cache-poorer target before spending a restructure on it.
// ----------------------------------------------------------------------------

template <typename T> static void setup_pile_scene(simulator<T> & sim, int per_side, int layers) {
	using tr = scalar_traits<T>;
	sim.set_gravity({ T {}, T {}, -tr::from_int(20) });
	sim.set_default_contact_mode(contact_mode::speculative);

	auto floor_solid = std::make_shared<solid<T>>();
	floor_solid->set_infinite_mass();
	floor_solid->set_coefficient_of_gravity(T {});
	floor_solid->set_position({ T {}, T {}, -tr::half() });
	floor_solid->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(-tr::from_int(40), -tr::from_int(40), -tr::half(),
	              tr::from_int(40), tr::from_int(40), tr::half())));
	sim.add_solid(floor_solid);

	// 0.4 m boxes on a 0.41 m lattice: they land face to face, which is what makes
	// four-point manifolds instead of the single point a sphere would give.
	const T half = tr::from_milli(200);
	for (int L = 0; L < layers; ++L)
	for (int i = 0; i < per_side; ++i)
	for (int j = 0; j < per_side; ++j) {
		auto b = std::make_shared<solid<T>>();
		b->set_mass(tr::one());
		b->set_inertia({ tr::from_milli(27), tr::from_milli(27), tr::from_milli(27) });
		b->set_coefficient_of_restitution(T {});
		b->set_position({ tr::from_milli(410 * (i - per_side / 2)),
		                  tr::from_milli(410 * (j - per_side / 2)),
		                  tr::from_milli(210 + 430 * L) });
		b->add_shape(std::make_shared<shape<T>>(aa_box<T>(half)));
		sim.add_solid(b);
	}
}

// What share of the provisioning is live, sampled over the timed run.
template <typename T> static void report_pile_occupancy(const simulator<T> & sim) {
	long long slots_live = 0, slots_provisioned = 0, points_live = 0, points_provisioned = 0;
	long long full_slots = 0, bodies = 0;
	for (const auto & sp : sim.get_solids()) {
		const solid<T> * b = sp.get();
		++bodies;
		int n = b->get_touch_count();
		slots_live += n;
		slots_provisioned += solid<T>::max_touches;
		if (n == solid<T>::max_touches)
			++full_slots;
		for (int i = 0; i < n; ++i) {
			points_live += b->get_touch(i).point_count;
			points_provisioned += max_manifold_points;
		}
	}
	printf("    %lld bodies: %lld/%lld slots live (%.0f%%, %lld at all %d), "
	       "%lld/%lld points in those slots (%.0f%%)\n",
	       bodies, slots_live, slots_provisioned,
	       100.0 * double(slots_live) / double(slots_provisioned ? slots_provisioned : 1),
	       full_slots, solid<T>::max_touches, points_live, points_provisioned,
	       100.0 * double(points_live) / double(points_provisioned ? points_provisioned : 1));
}

template <typename T> static void bench_manifold_pile(const char * label) {
	using tr = scalar_traits<T>;
	printf("[manifold_pile %s]\n", label);

	struct config { int per_side; int layers; int iters; };
	config configs[] = { { 4, 4, 400 }, { 6, 6, 150 } };

	for (auto c : configs) {
		auto sim = std::make_shared<simulator<T>>();
		setup_pile_scene(*sim, c.per_side, c.layers);
		// Settle first: a pile in free fall has no manifolds yet, and the resting
		// pile is the state the solver actually spends its time in.
		for (int i = 0; i < 100; ++i)
			sim->update(tr::from_milli(16));
		char name[64];
		std::snprintf(name, sizeof(name), "%dx%d x %d layers (%d boxes)",
		              c.per_side, c.per_side, c.layers, c.per_side * c.per_side * c.layers);
		bench::go(name, c.iters, [&] { sim->update(tr::from_milli(16)); });
		report_pile_occupancy(*sim);
	}
}

// ----------------------------------------------------------------------------

int main() {
	printf("hop bench\n");
	printf("---------\n");
	printf("sizeof(shape<float>) = %zu bytes\n", sizeof(shape<float>));
	printf("sizeof(solid<float>) = %zu bytes\n", sizeof(solid<float>));
	printf("sizeof(solid<float>::touch) = %zu bytes  (%d slots x %d points = %zu bytes of the solid)\n",
	       sizeof(solid<float>::touch), solid<float>::max_touches, max_manifold_points,
	       sizeof(solid<float>::touch) * solid<float>::max_touches);
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

	bench_manifold_pile<float>("float");
	bench_manifold_pile<fixed16>("fixed16");

	printf("\ndone\n");
	return 0;
}
