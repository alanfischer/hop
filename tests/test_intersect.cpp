#include <hop/hop.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace hop;

template<typename T>
static void test_point_in_box() {
	using tr = scalar_traits<T>;
	aa_box<T> box{
		vec3<T>{-tr::one(), -tr::one(), -tr::one()},
		vec3<T>{tr::one(), tr::one(), tr::one()}
	};
	vec3<T> inside{T{}, T{}, T{}};
	vec3<T> outside{tr::two(), T{}, T{}};

	assert(test_inside(box, inside));
	assert(!test_inside(box, outside));
	printf("  point in box: OK\n");
}

template<typename T>
static void test_box_box_intersection() {
	using tr = scalar_traits<T>;
	aa_box<T> a{vec3<T>{-tr::one(), -tr::one(), -tr::one()},
	            vec3<T>{tr::one(), tr::one(), tr::one()}};
	aa_box<T> b{vec3<T>{T{}, T{}, T{}},
	            vec3<T>{tr::two(), tr::two(), tr::two()}};
	aa_box<T> c{vec3<T>{tr::three(), tr::three(), tr::three()},
	            vec3<T>{tr::four(), tr::four(), tr::four()}};

	assert(test_intersection(a, b));
	assert(!test_intersection(a, c));
	printf("  box-box intersection: OK\n");
}

template<typename T>
static void test_ray_box() {
	using tr = scalar_traits<T>;
	aa_box<T> box{vec3<T>{-tr::one(), -tr::one(), -tr::one()},
	              vec3<T>{tr::one(), tr::one(), tr::one()}};

	// Ray from -5,0,0 toward +x should hit at x=-1
	segment<T> seg;
	T neg5 = -tr::from_int(5);
	seg.set_start_dir(vec3<T>{neg5, T{}, T{}}, vec3<T>{tr::from_int(10), T{}, T{}});

	vec3<T> point, normal;
	T time = find_intersection(seg, box, point, normal);
	float ft = tr::to_float(time);
	assert(ft > 0.3f && ft < 0.5f); // Should be ~0.4 (4 out of 10)
	assert(normal.x < T{});  // Should be -1 (hit left face)

	printf("  ray-box: OK\n");
}

template<typename T>
static void test_ray_sphere() {
	using tr = scalar_traits<T>;
	hop::sphere<T> sph{vec3<T>{T{}, T{}, T{}}, tr::one()};

	segment<T> seg;
	T neg5 = -tr::from_int(5);
	seg.set_start_dir(vec3<T>{neg5, T{}, T{}}, vec3<T>{tr::from_int(10), T{}, T{}});

	vec3<T> point, normal;
	T time = find_intersection(seg, sph, point, normal);
	float ft = tr::to_float(time);
	assert(ft > 0.3f && ft < 0.5f); // Should be ~0.4

	printf("  ray-sphere: OK\n");
}

int main() {
	printf("test_intersect (float):\n");
	test_point_in_box<float>();
	test_box_box_intersection<float>();
	test_ray_box<float>();
	test_ray_sphere<float>();

	printf("test_intersect (fixed16):\n");
	test_point_in_box<fixed16>();
	test_box_box_intersection<fixed16>();
	test_ray_box<fixed16>();
	test_ray_sphere<fixed16>();

	printf("ALL PASSED\n");
	return 0;
}
