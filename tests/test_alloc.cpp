// A warm tick allocates nothing. Solids, shapes and constraints are shared_ptr-owned
// and allocated when the scene is built; after the first ticks have grown the
// simulator's and the narrowphase's scratch to fit, update() reuses it. A scratch
// buffer still grows when a pile reaches a new high-water mark, which is a handful of
// allocations over a run; anything that allocates per tick or per contact shows up as
// hundreds.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <new>
#include <vector>

#include <hop/hop.h>

static long g_allocs = 0;
static bool g_counting = false;

void * operator new(std::size_t n) {
	if (g_counting)
		++g_allocs;
	if (void * p = std::malloc(n ? n : 1))
		return p;
	throw std::bad_alloc();
}
void operator delete(void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }

using namespace hop;
using T = double;
using V = vec3<T>;

static V v(T x, T y, T z) { V r; r.set(x, y, z); return r; }

static std::shared_ptr<shape<T>> make_shape(int i) {
	switch (i % 4) {
	case 0: return std::make_shared<shape<T>>(aa_box<T>(v(-.3, -.3, -.3), v(.3, .3, .3)));
	case 1: return std::make_shared<shape<T>>(sphere<T>(V {}, .3));
	case 2: return std::make_shared<shape<T>>(capsule<T>(v(0, -.2, 0), v(0, .4, 0), .2));
	default: {
		convex_solid<T> cs;
		const T k = 1 / std::sqrt(T(3));
		for (int s = 0; s < 8; ++s)
			cs.planes.push_back(plane<T>(v(s & 1 ? k : -k, s & 2 ? k : -k, s & 4 ? k : -k), .3));
		return std::make_shared<shape<T>>(cs);
	}
	}
}

// Allocations over `ticks` warm ticks of a tumbling, jointed pile: every shape type, or
// spheres alone.
static long warm_allocations(contact_mode mode, int count, bool spheres, int ticks) {
	simulator_config<T> config;
	config.default_contact_mode = mode;
	simulator<T> sim(v(0, -20, 0), config);
	bvh_manager<T> bvh;
	sim.set_manager(&bvh);

	auto floor = std::make_shared<solid<T>>();
	floor->add_shape(std::make_shared<shape<T>>(aa_box<T>(v(-50, -1, -50), v(50, 0, 50))));
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	sim.add_solid(floor);
	bvh.add_solid(floor.get(), true);

	std::vector<std::shared_ptr<solid<T>>> bodies;
	for (int i = 0; i < count; ++i) {
		auto s = std::make_shared<solid<T>>();
		s->add_shape(make_shape(spheres ? 1 : i));
		s->set_mass(1);
		s->set_inertia(v(.05, .05, .05));
		s->set_position(v((i % 8) * .8 - 2.8, .5 + (i / 64) * .8, ((i / 8) % 8) * .8 - 2.8));
		s->set_angular_velocity(v(i % 3, 1, i % 5));
		sim.add_solid(s);
		bvh.add_solid(s.get(), false);
		if (i % 4) {
			auto c = std::make_shared<constraint<T>>(bodies.back(), s);
			c->set_type(constraint<T>::type::rigid);
			sim.add_constraint(c);
		}
		bodies.push_back(s);
	}

	for (int t = 0; t < 60; ++t)
		sim.update(T(1) / 60);
	g_allocs = 0;
	g_counting = true;
	for (int t = 0; t < ticks; ++t)
		sim.update(T(1) / 60);
	g_counting = false;
	return g_allocs;
}

int main() {
	long n = warm_allocations(contact_mode::sweep_slide, 32, false, 60);
	printf("  sweep_slide, 32 mixed bodies: %ld allocations\n", n);
	assert(n <= 4 && "a warm tick allocates nothing");
	n = warm_allocations(contact_mode::speculative, 32, false, 60);
	printf("  speculative, 32 mixed bodies: %ld allocations\n", n);
	assert(n <= 4 && "a warm tick allocates nothing");
	// Enough single-point contacts to put the shock phase's sort past the size where
	// a stable sort reaches for a buffer.
	n = warm_allocations(contact_mode::speculative, 192, true, 30);
	printf("  speculative, 192 spheres: %ld allocations\n", n);
	assert(n <= 4 && "a warm tick allocates nothing");
	printf("test_alloc: PASS\n");
	return 0;
}
