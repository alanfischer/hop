#pragma once

#include <hop/math/vec3.h>

namespace hop {

template <typename T> struct sphere {
	vec3<T> origin;
	T radius {};

	constexpr sphere() = default;
	constexpr sphere(const vec3<T> & o, T r) : origin(o), radius(r) {}
	explicit constexpr sphere(T r) : radius(r) {}

	sphere & set(const sphere & s) {
		origin = s.origin;
		radius = s.radius;
		return *this;
	}
	sphere & set(const vec3<T> & o, T r) {
		origin = o;
		radius = r;
		return *this;
	}
	sphere & reset() {
		origin.reset();
		radius = T {};
		return *this;
	}

	bool operator==(const sphere & s) const { return origin == s.origin && radius == s.radius; }
	bool operator!=(const sphere & s) const { return !(*this == s); }

	sphere operator+(const vec3<T> & v) const { return { origin + v, radius }; }
	void operator+=(const vec3<T> & v) { origin += v; }
	sphere operator-(const vec3<T> & v) const { return { origin - v, radius }; }
	void operator-=(const vec3<T> & v) { origin -= v; }
};

} // namespace hop
