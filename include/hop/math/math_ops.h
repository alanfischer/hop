#pragma once

#include <hop/math/vec3.h>
#include <hop/math/aa_box.h>
#include <hop/math/sphere.h>
#include <hop/math/capsule.h>
#include <hop/math/segment.h>
#include <hop/math/plane.h>

namespace hop {

// Constants
template<typename T>
struct constants {
	static vec3<T> zero_vec3() { return {}; }
	static vec3<T> one_vec3() {
		using tr = scalar_traits<T>;
		return {tr::one(), tr::one(), tr::one()};
	}
	static vec3<T> x_unit_vec3() { using tr = scalar_traits<T>; return {tr::one(), T{}, T{}}; }
	static vec3<T> neg_x_unit_vec3() { using tr = scalar_traits<T>; return {-tr::one(), T{}, T{}}; }
	static vec3<T> y_unit_vec3() { using tr = scalar_traits<T>; return {T{}, tr::one(), T{}}; }
	static vec3<T> neg_y_unit_vec3() { using tr = scalar_traits<T>; return {T{}, -tr::one(), T{}}; }
	static vec3<T> z_unit_vec3() { using tr = scalar_traits<T>; return {T{}, T{}, tr::one()}; }
	static vec3<T> neg_z_unit_vec3() { using tr = scalar_traits<T>; return {T{}, T{}, -tr::one()}; }
};

// vec3 free functions
template<typename T>
inline void neg(vec3<T>& r, const vec3<T>& v) { r.x = -v.x; r.y = -v.y; r.z = -v.z; }

template<typename T>
inline void neg(vec3<T>& v) { v.x = -v.x; v.y = -v.y; v.z = -v.z; }

template<typename T>
inline void add(vec3<T>& r, const vec3<T>& a, const vec3<T>& b) { r.x = a.x + b.x; r.y = a.y + b.y; r.z = a.z + b.z; }

template<typename T>
inline void add(vec3<T>& r, const vec3<T>& v) { r.x += v.x; r.y += v.y; r.z += v.z; }

template<typename T>
inline void sub(vec3<T>& r, const vec3<T>& a, const vec3<T>& b) { r.x = a.x - b.x; r.y = a.y - b.y; r.z = a.z - b.z; }

template<typename T>
inline void sub(vec3<T>& r, const vec3<T>& v) { r.x -= v.x; r.y -= v.y; r.z -= v.z; }

template<typename T>
inline void mul(vec3<T>& r, const vec3<T>& v, T f) { r.x = v.x * f; r.y = v.y * f; r.z = v.z * f; }

template<typename T>
inline void mul(vec3<T>& r, T f) { r.x *= f; r.y *= f; r.z *= f; }

template<typename T>
inline void mul(vec3<T>& r, const vec3<T>& a, const vec3<T>& b) { r.x = a.x * b.x; r.y = a.y * b.y; r.z = a.z * b.z; }

template<typename T>
inline void mul(vec3<T>& r, const vec3<T>& v) { r.x *= v.x; r.y *= v.y; r.z *= v.z; }

template<typename T>
inline void div(vec3<T>& r, const vec3<T>& v, T f) { r.x = v.x / f; r.y = v.y / f; r.z = v.z / f; }

template<typename T>
inline void div(vec3<T>& r, T f) { r.x /= f; r.y /= f; r.z /= f; }

template<typename T>
inline void div(vec3<T>& r, const vec3<T>& a, const vec3<T>& b) { r.x = a.x / b.x; r.y = a.y / b.y; r.z = a.z / b.z; }

template<typename T>
inline void madd(vec3<T>& r, const vec3<T>& b, T c, const vec3<T>& a) {
	r.x = b.x * c + a.x; r.y = b.y * c + a.y; r.z = b.z * c + a.z;
}

template<typename T>
inline void madd(vec3<T>& r, T c, const vec3<T>& a) {
	r.x = r.x * c + a.x; r.y = r.y * c + a.y; r.z = r.z * c + a.z;
}

template<typename T>
inline T length_squared(const vec3<T>& v) { return v.x * v.x + v.y * v.y + v.z * v.z; }

template<typename T>
inline T length_squared(const vec3<T>& a, const vec3<T>& b) {
	T dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
	return dx * dx + dy * dy + dz * dz;
}

template<typename T>
inline T length(const vec3<T>& v) { return scalar_traits<T>::sqrt(length_squared(v)); }

template<typename T>
inline T length(const vec3<T>& a, const vec3<T>& b) { return scalar_traits<T>::sqrt(length_squared(a, b)); }

template<typename T>
inline T dot(const vec3<T>& a, const vec3<T>& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

template<typename T>
inline void cross(vec3<T>& r, const vec3<T>& a, const vec3<T>& b) {
	r.x = a.y * b.z - a.z * b.y;
	r.y = a.z * b.x - a.x * b.z;
	r.z = a.x * b.y - a.y * b.x;
}

template<typename T>
inline void normalize(vec3<T>& v) {
	T l = length(v);
	if (l > T{}) {
		l = scalar_traits<T>::one() / l;
		v.x *= l; v.y *= l; v.z *= l;
	}
}

template<typename T>
inline void normalize(vec3<T>& r, const vec3<T>& v) {
	T l = length(v);
	if (l > T{}) {
		l = scalar_traits<T>::one() / l;
		r.x = v.x * l; r.y = v.y * l; r.z = v.z * l;
	} else {
		r.reset();
	}
}

template<typename T>
inline bool normalize_carefully(vec3<T>& v, T epsilon) {
	T l = length(v);
	if (l > epsilon) {
		l = scalar_traits<T>::one() / l;
		v.x *= l; v.y *= l; v.z *= l;
		return true;
	}
	return false;
}

template<typename T>
inline bool normalize_carefully(vec3<T>& r, const vec3<T>& v, T epsilon) {
	T l = length(v);
	if (l > epsilon) {
		l = scalar_traits<T>::one() / l;
		r.x = v.x * l; r.y = v.y * l; r.z = v.z * l;
		return true;
	}
	return false;
}

template<typename T>
inline void lerp(vec3<T>& r, const vec3<T>& a, const vec3<T>& b, T t) {
	sub(r, b, a);
	mul(r, t);
	add(r, a);
}

template<typename T>
inline T square(T v) { return v * v; }

// AA box translate overloads
template<typename T>
inline void add(aa_box<T>& r, const aa_box<T>& b, const vec3<T>& p) {
	r.mins.x = b.mins.x + p.x; r.mins.y = b.mins.y + p.y; r.mins.z = b.mins.z + p.z;
	r.maxs.x = b.maxs.x + p.x; r.maxs.y = b.maxs.y + p.y; r.maxs.z = b.maxs.z + p.z;
}

template<typename T>
inline void add(aa_box<T>& b, const vec3<T>& p) {
	b.mins.x += p.x; b.mins.y += p.y; b.mins.z += p.z;
	b.maxs.x += p.x; b.maxs.y += p.y; b.maxs.z += p.z;
}

template<typename T>
inline void sub(aa_box<T>& r, const aa_box<T>& b, const vec3<T>& p) {
	r.mins.x = b.mins.x - p.x; r.mins.y = b.mins.y - p.y; r.mins.z = b.mins.z - p.z;
	r.maxs.x = b.maxs.x - p.x; r.maxs.y = b.maxs.y - p.y; r.maxs.z = b.maxs.z - p.z;
}

template<typename T>
inline void sub(aa_box<T>& b, const vec3<T>& p) {
	b.mins.x -= p.x; b.mins.y -= p.y; b.mins.z -= p.z;
	b.maxs.x -= p.x; b.maxs.y -= p.y; b.maxs.z -= p.z;
}

// Sphere translate overloads
template<typename T>
inline void add(sphere<T>& r, const sphere<T>& s, const vec3<T>& p) {
	r.origin.x = s.origin.x + p.x; r.origin.y = s.origin.y + p.y; r.origin.z = s.origin.z + p.z;
}

template<typename T>
inline void add(sphere<T>& s, const vec3<T>& p) {
	s.origin.x += p.x; s.origin.y += p.y; s.origin.z += p.z;
}

template<typename T>
inline void sub(sphere<T>& r, const sphere<T>& s, const vec3<T>& p) {
	r.origin.x = s.origin.x - p.x; r.origin.y = s.origin.y - p.y; r.origin.z = s.origin.z - p.z;
}

template<typename T>
inline void sub(sphere<T>& s, const vec3<T>& p) {
	s.origin.x -= p.x; s.origin.y -= p.y; s.origin.z -= p.z;
}

// Capsule translate overloads
template<typename T>
inline void add(capsule<T>& r, const capsule<T>& c, const vec3<T>& p) {
	r.origin.x = c.origin.x + p.x; r.origin.y = c.origin.y + p.y; r.origin.z = c.origin.z + p.z;
}

template<typename T>
inline void add(capsule<T>& c, const vec3<T>& p) {
	c.origin.x += p.x; c.origin.y += p.y; c.origin.z += p.z;
}

template<typename T>
inline void sub(capsule<T>& r, const capsule<T>& c, const vec3<T>& p) {
	r.origin.x = c.origin.x - p.x; r.origin.y = c.origin.y - p.y; r.origin.z = c.origin.z - p.z;
}

template<typename T>
inline void sub(capsule<T>& c, const vec3<T>& p) {
	c.origin.x -= p.x; c.origin.y -= p.y; c.origin.z -= p.z;
}

// Three-plane intersection (Cramer's rule)
template<typename T>
inline bool get_intersection_of_three_planes(vec3<T>& result, const plane<T>& p1, const plane<T>& p2, const plane<T>& p3, T epsilon) {
	vec3<T> p2xp3;
	cross(p2xp3, p2.normal, p3.normal);
	T den = dot(p1.normal, p2xp3);
	if (den < epsilon && den > -epsilon) return false;

	vec3<T> p3xp1, p1xp2;
	cross(p3xp1, p3.normal, p1.normal);
	cross(p1xp2, p1.normal, p2.normal);
	mul(p1xp2, p3.distance);
	mul(p2xp3, p1.distance);
	mul(p3xp1, p2.distance);
	add(result, p1xp2, p2xp3);
	add(result, p3xp1);
	div(result, den);
	return true;
}

} // namespace hop
