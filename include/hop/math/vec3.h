#pragma once

#include <hop/scalar_traits.h>

namespace hop {

template <typename T> struct vec3 {
	T x {}, y {}, z {};

	constexpr vec3() = default;
	constexpr vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

	vec3 & set(const vec3 & v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}
	vec3 & set(T x_, T y_, T z_) {
		x = x_;
		y = y_;
		z = z_;
		return *this;
	}
	vec3 & reset() {
		x = T {};
		y = T {};
		z = T {};
		return *this;
	}

	T * data() { return &x; }
	const T * data() const { return &x; }

	bool operator==(const vec3 & v) const { return x == v.x && y == v.y && z == v.z; }
	bool operator!=(const vec3 & v) const { return !(*this == v); }

	vec3 operator+(const vec3 & v) const { return { x + v.x, y + v.y, z + v.z }; }
	vec3 operator-(const vec3 & v) const { return { x - v.x, y - v.y, z - v.z }; }
	vec3 operator*(T f) const { return { x * f, y * f, z * f }; }
	vec3 operator*(const vec3 & v) const { return { x * v.x, y * v.y, z * v.z }; }
	vec3 operator-() const { return { -x, -y, -z }; }

	void operator+=(const vec3 & v) {
		x += v.x;
		y += v.y;
		z += v.z;
	}
	void operator-=(const vec3 & v) {
		x -= v.x;
		y -= v.y;
		z -= v.z;
	}
	void operator*=(T f) {
		x *= f;
		y *= f;
		z *= f;
	}

	T & operator[](int i) { return *((&x) + i); }
	T operator[](int i) const { return *((&x) + i); }
};

template <typename T> vec3<T> operator*(T f, const vec3<T> & v) { return { v.x * f, v.y * f, v.z * f }; }

// Constants
template <typename T> struct constants {
	static vec3<T> zero_vec3() { return {}; }
	static vec3<T> one_vec3() {
		using tr = scalar_traits<T>;
		return { tr::one(), tr::one(), tr::one() };
	}
	static vec3<T> x_unit_vec3() {
		using tr = scalar_traits<T>;
		return { tr::one(), T {}, T {} };
	}
	static vec3<T> neg_x_unit_vec3() {
		using tr = scalar_traits<T>;
		return { -tr::one(), T {}, T {} };
	}
	static vec3<T> y_unit_vec3() {
		using tr = scalar_traits<T>;
		return { T {}, tr::one(), T {} };
	}
	static vec3<T> neg_y_unit_vec3() {
		using tr = scalar_traits<T>;
		return { T {}, -tr::one(), T {} };
	}
	static vec3<T> z_unit_vec3() {
		using tr = scalar_traits<T>;
		return { T {}, T {}, tr::one() };
	}
	static vec3<T> neg_z_unit_vec3() {
		using tr = scalar_traits<T>;
		return { T {}, T {}, -tr::one() };
	}
};

// Free functions
template <typename T> inline void neg(vec3<T> & r, const vec3<T> & v) {
	r.x = -v.x;
	r.y = -v.y;
	r.z = -v.z;
}

template <typename T> inline void neg(vec3<T> & v) {
	v.x = -v.x;
	v.y = -v.y;
	v.z = -v.z;
}

template <typename T> inline void add(vec3<T> & r, const vec3<T> & a, const vec3<T> & b) {
	r.x = a.x + b.x;
	r.y = a.y + b.y;
	r.z = a.z + b.z;
}

template <typename T> inline void add(vec3<T> & r, const vec3<T> & v) {
	r.x += v.x;
	r.y += v.y;
	r.z += v.z;
}

template <typename T> inline void sub(vec3<T> & r, const vec3<T> & a, const vec3<T> & b) {
	r.x = a.x - b.x;
	r.y = a.y - b.y;
	r.z = a.z - b.z;
}

template <typename T> inline void sub(vec3<T> & r, const vec3<T> & v) {
	r.x -= v.x;
	r.y -= v.y;
	r.z -= v.z;
}

template <typename T> inline void mul(vec3<T> & r, const vec3<T> & v, T f) {
	r.x = v.x * f;
	r.y = v.y * f;
	r.z = v.z * f;
}

template <typename T> inline void mul(vec3<T> & r, T f) {
	r.x *= f;
	r.y *= f;
	r.z *= f;
}

template <typename T> inline void mul(vec3<T> & r, const vec3<T> & a, const vec3<T> & b) {
	r.x = a.x * b.x;
	r.y = a.y * b.y;
	r.z = a.z * b.z;
}

template <typename T> inline void mul(vec3<T> & r, const vec3<T> & v) {
	r.x *= v.x;
	r.y *= v.y;
	r.z *= v.z;
}

template <typename T> inline void div(vec3<T> & r, const vec3<T> & v, T f) {
	r.x = v.x / f;
	r.y = v.y / f;
	r.z = v.z / f;
}

template <typename T> inline void div(vec3<T> & r, T f) {
	r.x /= f;
	r.y /= f;
	r.z /= f;
}

template <typename T> inline void div(vec3<T> & r, const vec3<T> & a, const vec3<T> & b) {
	r.x = a.x / b.x;
	r.y = a.y / b.y;
	r.z = a.z / b.z;
}

template <typename T> inline void madd(vec3<T> & r, const vec3<T> & b, T c, const vec3<T> & a) {
	r.x = b.x * c + a.x;
	r.y = b.y * c + a.y;
	r.z = b.z * c + a.z;
}

template <typename T> inline void madd(vec3<T> & r, T c, const vec3<T> & a) {
	r.x = r.x * c + a.x;
	r.y = r.y * c + a.y;
	r.z = r.z * c + a.z;
}

template <typename T> inline T length_squared(const vec3<T> & v) { return v.x * v.x + v.y * v.y + v.z * v.z; }

template <typename T> inline T length_squared(const vec3<T> & a, const vec3<T> & b) {
	T dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
	return dx * dx + dy * dy + dz * dz;
}

template <typename T> inline T length(const vec3<T> & v) { return scalar_traits<T>::sqrt(length_squared(v)); }

template <typename T> inline T length(const vec3<T> & a, const vec3<T> & b) {
	return scalar_traits<T>::sqrt(length_squared(a, b));
}

template <typename T> inline T dot(const vec3<T> & a, const vec3<T> & b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

template <typename T> inline void cross(vec3<T> & r, const vec3<T> & a, const vec3<T> & b) {
	r.x = a.y * b.z - a.z * b.y;
	r.y = a.z * b.x - a.x * b.z;
	r.z = a.x * b.y - a.y * b.x;
}

template <typename T> inline void normalize(vec3<T> & v) {
	T l = length(v);
	if (l > T {}) {
		l = scalar_traits<T>::one() / l;
		v.x *= l;
		v.y *= l;
		v.z *= l;
	}
}

template <typename T> inline void normalize(vec3<T> & r, const vec3<T> & v) {
	T l = length(v);
	if (l > T {}) {
		l = scalar_traits<T>::one() / l;
		r.x = v.x * l;
		r.y = v.y * l;
		r.z = v.z * l;
	} else {
		r.reset();
	}
}

template <typename T> inline bool normalize_carefully(vec3<T> & v, T epsilon) {
	T l = length(v);
	if (l > epsilon) {
		l = scalar_traits<T>::one() / l;
		v.x *= l;
		v.y *= l;
		v.z *= l;
		return true;
	}
	return false;
}

template <typename T> inline bool normalize_carefully(vec3<T> & r, const vec3<T> & v, T epsilon) {
	T l = length(v);
	if (l > epsilon) {
		l = scalar_traits<T>::one() / l;
		r.x = v.x * l;
		r.y = v.y * l;
		r.z = v.z * l;
		return true;
	}
	return false;
}

template <typename T> inline void lerp(vec3<T> & r, const vec3<T> & a, const vec3<T> & b, T t) {
	sub(r, b, a);
	mul(r, t);
	add(r, a);
}

} // namespace hop
