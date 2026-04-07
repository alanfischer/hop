#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#include <set>
#include <hop/hop.h>

using namespace hop;

// ============================================================
// BVH tests
// ============================================================

template <typename T>
static void test_bvh_empty() {
	bvh<T, int> tree;
	assert(tree.empty());
	assert(tree.size() == 0);

	// Queries on empty tree should not crash
	int count = 0;
	tree.query_aabb(aa_box<T>(vec3<T>{}, vec3<T>(scalar_traits<T>::one(), scalar_traits<T>::one(), scalar_traits<T>::one())),
		[&](int) { count++; });
	assert(count == 0);

	tree.query_ray(vec3<T>{}, vec3<T>(scalar_traits<T>::one(), T{}, T{}),
		[&](int, T &) { count++; });
	assert(count == 0);

	printf("  bvh empty: OK\n");
}

template <typename T>
static void test_bvh_single_item() {
	using tr = scalar_traits<T>;
	bvh<T, int> tree;

	std::vector<std::pair<aa_box<T>, int>> entries;
	entries.push_back({aa_box<T>(vec3<T>(T{}, T{}, T{}), vec3<T>(tr::one(), tr::one(), tr::one())), 42});
	tree.build(entries);

	assert(!tree.empty());
	assert(tree.size() == 1);

	// Query that overlaps
	int found = -1;
	tree.query_aabb(aa_box<T>(vec3<T>(tr::half(), tr::half(), tr::half()),
	                          vec3<T>(tr::two(), tr::two(), tr::two())),
		[&](int item) { found = item; });
	assert(found == 42);

	// Query that misses
	found = -1;
	tree.query_aabb(aa_box<T>(vec3<T>(tr::from_int(5), tr::from_int(5), tr::from_int(5)),
	                          vec3<T>(tr::from_int(6), tr::from_int(6), tr::from_int(6))),
		[&](int item) { found = item; });
	assert(found == -1);

	printf("  bvh single item: OK\n");
}

template <typename T>
static void test_bvh_many_items() {
	using tr = scalar_traits<T>;
	bvh<T, int> tree;

	// Create a 10x10x1 grid of boxes
	std::vector<std::pair<aa_box<T>, int>> entries;
	for (int x = 0; x < 10; x++) {
		for (int y = 0; y < 10; y++) {
			T fx = tr::from_int(x);
			T fy = tr::from_int(y);
			T one = tr::one();
			aa_box<T> box(vec3<T>(fx, fy, T{}), vec3<T>(fx + one, fy + one, one));
			entries.push_back({box, x * 10 + y});
		}
	}
	tree.build(entries);

	assert(!tree.empty());

	// Query a small region — should find only nearby items
	std::set<int> found;
	aa_box<T> query(vec3<T>(tr::half(), tr::half(), T{}),
	                vec3<T>(tr::one() + tr::half(), tr::one() + tr::half(), tr::one()));
	tree.query_aabb(query, [&](int item) { found.insert(item); });

	// Should find items at (0,0), (0,1), (1,0), (1,1) = indices 0, 1, 10, 11
	assert(found.count(0) == 1);   // (0,0)
	assert(found.count(1) == 1);   // (0,1)
	assert(found.count(10) == 1);  // (1,0)
	assert(found.count(11) == 1);  // (1,1)
	assert(found.size() == 4);

	// Query far away — should find nothing
	found.clear();
	aa_box<T> far_query(vec3<T>(tr::from_int(20), tr::from_int(20), T{}),
	                    vec3<T>(tr::from_int(21), tr::from_int(21), tr::one()));
	tree.query_aabb(far_query, [&](int item) { found.insert(item); });
	assert(found.empty());

	printf("  bvh many items: OK\n");
}

template <typename T>
static void test_bvh_ray_query() {
	using tr = scalar_traits<T>;
	bvh<T, int> tree;

	// Three boxes along the X axis at x=1, x=5, x=10
	std::vector<std::pair<aa_box<T>, int>> entries;
	T half = tr::half();
	for (int i : {1, 5, 10}) {
		T cx = tr::from_int(i);
		aa_box<T> box(vec3<T>(cx - half, -half, -half), vec3<T>(cx + half, half, half));
		entries.push_back({box, i});
	}
	tree.build(entries);

	// Ray along +X from origin
	int first_hit = -1;
	tree.query_ray(vec3<T>{}, vec3<T>(tr::from_int(20), T{}, T{}),
		[&](int item, T & best_t) {
			// Simple: the closest box center gives earliest hit
			// We just record the first item the BVH visits that could be hit
			T t = tr::from_int(item) / tr::from_int(20); // approximate parametric time
			if (t < best_t) {
				best_t = t;
				first_hit = item;
			}
		});
	assert(first_hit == 1); // closest box at x=1

	printf("  bvh ray query: OK\n");
}

// ============================================================
// BVH Manager tests
// ============================================================

template <typename T>
static void test_bvh_manager_basic() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity(vec3<T>{});

	bvh_manager<T> mgr;
	sim->set_manager(&mgr);

	// Create 3 static solids spread along X
	std::vector<std::shared_ptr<solid<T>>> solids;
	for (int i = 0; i < 3; i++) {
		auto s = std::make_shared<solid<T>>();
		s->set_infinite_mass();
		T x = tr::from_int(i * 10);
		s->set_position(vec3<T>(x, T{}, T{}));
		auto sh = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
		s->add_shape(sh);
		sim->add_solid(s);
		mgr.add_solid(s.get(), true);
		solids.push_back(s);
	}

	assert(mgr.get_static_count() == 3);
	assert(mgr.get_dynamic_count() == 0);

	// Query around first solid
	solid<T> * found[10];
	int count = sim->find_solids_in_aa_box(
		aa_box<T>(vec3<T>(-tr::two(), -tr::two(), -tr::two()),
		          vec3<T>(tr::two(), tr::two(), tr::two())),
		found, 10);
	assert(count == 1);
	assert(found[0] == solids[0].get());

	// Query that encompasses all
	count = sim->find_solids_in_aa_box(
		aa_box<T>(vec3<T>(-tr::two(), -tr::two(), -tr::two()),
		          vec3<T>(tr::from_int(22), tr::two(), tr::two())),
		found, 10);
	assert(count == 3);

	printf("  bvh_manager basic: OK\n");
}

template <typename T>
static void test_bvh_manager_mixed() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity(vec3<T>{});

	bvh_manager<T> mgr;
	sim->set_manager(&mgr);

	// One static solid at origin
	auto stat = std::make_shared<solid<T>>();
	stat->set_infinite_mass();
	stat->set_position(vec3<T>{});
	auto sh1 = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
	stat->add_shape(sh1);
	sim->add_solid(stat);
	mgr.add_solid(stat.get(), true);

	// One dynamic solid at x=5
	auto dyn = std::make_shared<solid<T>>();
	dyn->set_mass(tr::one());
	dyn->set_position(vec3<T>(tr::from_int(5), T{}, T{}));
	auto sh2 = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
	dyn->add_shape(sh2);
	sim->add_solid(dyn);
	mgr.add_solid(dyn.get(), false);

	assert(mgr.get_static_count() == 1);
	assert(mgr.get_dynamic_count() == 1);

	// Query that finds both
	solid<T> * found[10];
	int count = sim->find_solids_in_aa_box(
		aa_box<T>(vec3<T>(-tr::two(), -tr::two(), -tr::two()),
		          vec3<T>(tr::from_int(7), tr::two(), tr::two())),
		found, 10);
	assert(count == 2);

	// Query that finds only the static
	count = sim->find_solids_in_aa_box(
		aa_box<T>(vec3<T>(-tr::two(), -tr::two(), -tr::two()),
		          vec3<T>(tr::two(), tr::two(), tr::two())),
		found, 10);
	assert(count == 1);
	assert(found[0] == stat.get());

	// Query that finds only the dynamic
	count = sim->find_solids_in_aa_box(
		aa_box<T>(vec3<T>(tr::from_int(4), -tr::two(), -tr::two()),
		          vec3<T>(tr::from_int(7), tr::two(), tr::two())),
		found, 10);
	assert(count == 1);
	assert(found[0] == dyn.get());

	printf("  bvh_manager mixed static+dynamic: OK\n");
}

template <typename T>
static void test_bvh_manager_remove() {
	using tr = scalar_traits<T>;

	bvh_manager<T> mgr;

	auto s1 = std::make_shared<solid<T>>();
	s1->set_infinite_mass();
	s1->set_position(vec3<T>{});
	auto sh = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
	s1->add_shape(sh);

	mgr.add_solid(s1.get(), true);
	assert(mgr.get_static_count() == 1);

	mgr.remove_solid(s1.get());
	assert(mgr.get_static_count() == 0);

	// Query should find nothing after removal
	mgr.rebuild();
	solid<T> * found[10];
	int count = mgr.find_solids_in_aa_box(
		aa_box<T>(vec3<T>(-tr::from_int(100), -tr::from_int(100), -tr::from_int(100)),
		          vec3<T>(tr::from_int(100), tr::from_int(100), tr::from_int(100))),
		found, 10);
	assert(count == 0);

	printf("  bvh_manager remove: OK\n");
}

template <typename T>
static void test_bvh_manager_trace_segment() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity(vec3<T>{});

	bvh_manager<T> mgr;
	sim->set_manager(&mgr);

	// Place a static box at x=5
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_position(vec3<T>(tr::from_int(5), T{}, T{}));
	wall->set_collision_scope(1);
	auto sh = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
	wall->add_shape(sh);
	sim->add_solid(wall);
	mgr.add_solid(wall.get(), true);
	wall->deactivate();

	// Trace a segment from origin toward +X
	collision<T> result;
	segment<T> seg;
	seg.set_start_end(vec3<T>{}, vec3<T>(tr::from_int(10), T{}, T{}));
	sim->trace_segment(result, seg, 1);

	// Should hit the box at approximately x=4 (box extends from 4 to 6)
	float hit_x = tr::to_float(result.point.x);
	printf("  trace_segment hit at x=%.3f (expected ~4.0)\n", hit_x);
	assert(result.time < tr::one());
	assert(hit_x > 3.5f && hit_x < 4.5f);

	printf("  bvh_manager trace_segment: OK\n");
}

template <typename T>
static void test_bvh_manager_trace_solid() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity(vec3<T>{});

	bvh_manager<T> mgr;
	sim->set_manager(&mgr);

	// Static wall at x=10
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->set_position(vec3<T>(tr::from_int(10), T{}, T{}));
	wall->set_collision_scope(1);
	auto sh_wall = std::make_shared<shape<T>>(aa_box<T>(tr::one()));
	wall->add_shape(sh_wall);
	sim->add_solid(wall);
	mgr.add_solid(wall.get(), true);
	wall->deactivate();

	// Moving sphere starting at origin
	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position(vec3<T>{});
	ball->set_collide_with_scope(1);
	auto sh_ball = std::make_shared<shape<T>>(hop::sphere<T>(vec3<T>{}, tr::one()));
	ball->add_shape(sh_ball);
	sim->add_solid(ball);
	mgr.add_solid(ball.get(), false);

	// Trace the ball from origin to x=20
	collision<T> result;
	segment<T> seg;
	seg.set_start_end(vec3<T>{}, vec3<T>(tr::from_int(20), T{}, T{}));
	sim->trace_solid(result, ball.get(), seg, 1);

	// Should hit: ball radius 1 + box extends from 9 to 11
	// So contact at x ≈ 8 (ball center when sphere surface touches box at x=9)
	float hit_t = tr::to_float(result.time);
	printf("  trace_solid hit at t=%.3f (expected ~0.4)\n", hit_t);
	assert(result.time < tr::one());
	assert(hit_t > 0.3f && hit_t < 0.5f);

	printf("  bvh_manager trace_solid: OK\n");
}

// ============================================================
// Main
// ============================================================

int main() {
	printf("test_bvh (float):\n");
	test_bvh_empty<float>();
	test_bvh_single_item<float>();
	test_bvh_many_items<float>();
	test_bvh_ray_query<float>();

	printf("test_bvh (fixed16):\n");
	test_bvh_empty<fixed16>();
	test_bvh_single_item<fixed16>();
	test_bvh_many_items<fixed16>();
	test_bvh_ray_query<fixed16>();

	printf("test_bvh_manager (float):\n");
	test_bvh_manager_basic<float>();
	test_bvh_manager_mixed<float>();
	test_bvh_manager_remove<float>();
	test_bvh_manager_trace_segment<float>();
	test_bvh_manager_trace_solid<float>();

	printf("test_bvh_manager (fixed16):\n");
	test_bvh_manager_basic<fixed16>();
	test_bvh_manager_mixed<fixed16>();
	test_bvh_manager_remove<fixed16>();
	test_bvh_manager_trace_segment<fixed16>();
	test_bvh_manager_trace_solid<fixed16>();

	printf("ALL PASSED\n");
	return 0;
}
