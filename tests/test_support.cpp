#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

static bool approx(float a, float b, float tol = 0.01f) { return std::fabs(a - b) < tol; }

// Build a unit cube convex_solid: 6 planes, centered at origin, half-extent 1.
template <typename T> static convex_solid<T> make_unit_cube() {
	using tr = scalar_traits<T>;
	convex_solid<T> cs;
	cs.planes.push_back({ { tr::one(), T {}, T {} }, tr::one() });   // +x
	cs.planes.push_back({ { -tr::one(), T {}, T {} }, tr::one() });  // -x
	cs.planes.push_back({ { T {}, tr::one(), T {} }, tr::one() });   // +y
	cs.planes.push_back({ { T {}, -tr::one(), T {} }, tr::one() });  // -y
	cs.planes.push_back({ { T {}, T {}, tr::one() }, tr::one() });   // +z
	cs.planes.push_back({ { T {}, T {}, -tr::one() }, tr::one() });  // -z
	return cs;
}

// Test: convex_solid support returns correct vertices for axis-aligned directions
template <typename T> static void test_convex_solid_support_axis(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_support_axis[%s]: ", label);
	auto cs = make_unit_cube<T>();

	vec3<T> result;

	// +x direction should give a corner with x=1
	support(result, cs, vec3<T> { tr::one(), T {}, T {} });
	assert(approx(tr::to_float(result.x), 1.0f));

	// -x direction should give a corner with x=-1
	support(result, cs, vec3<T> { -tr::one(), T {}, T {} });
	assert(approx(tr::to_float(result.x), -1.0f));

	// +z direction should give a corner with z=1
	support(result, cs, vec3<T> { T {}, T {}, tr::one() });
	assert(approx(tr::to_float(result.z), 1.0f));

	printf("OK\n");
}

// Test: convex_solid support for diagonal direction returns corner (1,1,1)
template <typename T> static void test_convex_solid_support_diagonal(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_support_diagonal[%s]: ", label);
	auto cs = make_unit_cube<T>();

	vec3<T> dir { tr::one(), tr::one(), tr::one() };
	vec3<T> result;
	support(result, cs, dir);
	assert(approx(tr::to_float(result.x), 1.0f));
	assert(approx(tr::to_float(result.y), 1.0f));
	assert(approx(tr::to_float(result.z), 1.0f));

	// Opposite diagonal
	vec3<T> neg_dir { -tr::one(), -tr::one(), -tr::one() };
	support(result, cs, neg_dir);
	assert(approx(tr::to_float(result.x), -1.0f));
	assert(approx(tr::to_float(result.y), -1.0f));
	assert(approx(tr::to_float(result.z), -1.0f));

	printf("OK\n");
}

// Test: shape-level support dispatches correctly for all primitive types
template <typename T> static void test_shape_dispatch(const char * label) {
	using tr = scalar_traits<T>;
	printf("  shape_dispatch[%s]: ", label);

	vec3<T> dir { tr::one(), T {}, T {} };
	vec3<T> shape_result, prim_result;

	// aa_box
	aa_box<T> box;
	box.mins = { -tr::one(), -tr::one(), -tr::one() };
	box.maxs = { tr::one(), tr::one(), tr::one() };
	shape<T> box_shape(box);
	support(shape_result, box_shape, dir);
	support(prim_result, box, dir);
	assert(approx(tr::to_float(shape_result.x), tr::to_float(prim_result.x)));
	assert(approx(tr::to_float(shape_result.y), tr::to_float(prim_result.y)));
	assert(approx(tr::to_float(shape_result.z), tr::to_float(prim_result.z)));

	// sphere
	hop::sphere<T> sph;
	sph.origin = { T {}, T {}, T {} };
	sph.radius = tr::one();
	shape<T> sph_shape(sph);
	support(shape_result, sph_shape, dir);
	support(prim_result, sph, dir);
	assert(approx(tr::to_float(shape_result.x), tr::to_float(prim_result.x)));

	// capsule
	hop::capsule<T> cap;
	cap.origin = { T {}, T {}, -tr::one() };
	cap.direction = { T {}, T {}, tr::from_int(2) };
	cap.radius = tr::half();
	shape<T> cap_shape(cap);
	support(shape_result, cap_shape, dir);
	support(prim_result, cap, dir);
	assert(approx(tr::to_float(shape_result.x), tr::to_float(prim_result.x)));

	// convex_solid
	auto cs = make_unit_cube<T>();
	shape<T> cs_shape(cs);
	support(shape_result, cs_shape, dir);
	support(prim_result, cs, dir);
	assert(approx(tr::to_float(shape_result.x), tr::to_float(prim_result.x)));
	assert(approx(tr::to_float(shape_result.y), tr::to_float(prim_result.y)));
	assert(approx(tr::to_float(shape_result.z), tr::to_float(prim_result.z)));

	printf("OK\n");
}

// Cached-vertex path: after rebuild_vertices(), support() must give the same
// answer as the on-the-fly enumeration, on the same set of probe directions.
template <typename T> static void test_convex_solid_cached_vertices(const char * label) {
	using tr = scalar_traits<T>;
	printf("  convex_solid_cached_vertices[%s]: ", label);
	auto uncached = make_unit_cube<T>();
	auto cached = make_unit_cube<T>();
	rebuild_vertices(cached);
	assert(!cached.vertices.empty());
	// A unit cube has 8 corners.
	assert(cached.vertices.size() == 8);

	const vec3<T> probes[] = {
		{ tr::one(), T {}, T {} },
		{ -tr::one(), T {}, T {} },
		{ T {}, tr::one(), T {} },
		{ T {}, T {}, tr::one() },
		{ tr::one(), tr::one(), tr::one() },
		{ -tr::one(), tr::one(), -tr::one() },
		{ tr::half(), -tr::one(), tr::half() },
	};
	for (const auto & d : probes) {
		vec3<T> r_uncached, r_cached;
		support(r_uncached, uncached, d);
		support(r_cached, cached, d);
		assert(r_uncached.x == r_cached.x);
		assert(r_uncached.y == r_cached.y);
		assert(r_uncached.z == r_cached.z);
	}
	printf("OK\n");
}

template <typename T> static void run_all_tests(const char * label) {
	test_convex_solid_support_axis<T>(label);
	test_convex_solid_support_diagonal<T>(label);
	test_shape_dispatch<T>(label);
	test_convex_solid_cached_vertices<T>(label);
}

int main() {
	printf("test_support:\n");
	run_all_tests<float>("float");
	run_all_tests<fixed16>("fixed16");
	printf("all tests passed\n");
	return 0;
}
