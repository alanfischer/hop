#pragma once

#include <hop/scalar_traits.h>

namespace hop {

template<typename T>
struct vec3 {
	T x{}, y{}, z{};

	constexpr vec3() = default;
	constexpr vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

	vec3& set(const vec3& v) { x = v.x; y = v.y; z = v.z; return *this; }
	vec3& set(T x_, T y_, T z_) { x = x_; y = y_; z = z_; return *this; }
	vec3& reset() { x = T{}; y = T{}; z = T{}; return *this; }

	T* data() { return &x; }
	const T* data() const { return &x; }

	bool operator==(const vec3& v) const { return x == v.x && y == v.y && z == v.z; }
	bool operator!=(const vec3& v) const { return !(*this == v); }

	vec3 operator+(const vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
	vec3 operator-(const vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
	vec3 operator*(T f) const { return {x * f, y * f, z * f}; }
	vec3 operator*(const vec3& v) const { return {x * v.x, y * v.y, z * v.z}; }
	vec3 operator-() const { return {-x, -y, -z}; }

	void operator+=(const vec3& v) { x += v.x; y += v.y; z += v.z; }
	void operator-=(const vec3& v) { x -= v.x; y -= v.y; z -= v.z; }
	void operator*=(T f) { x *= f; y *= f; z *= f; }

	T& operator[](int i) { return *((&x) + i); }
	T operator[](int i) const { return *((&x) + i); }
};

template<typename T>
vec3<T> operator*(T f, const vec3<T>& v) { return {v.x * f, v.y * f, v.z * f}; }

} // namespace hop
