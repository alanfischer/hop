#pragma once

#include <hop/math/vec3.h>

namespace hop {

template<typename T>
struct aa_box {
	vec3<T> mins;
	vec3<T> maxs;

	constexpr aa_box() = default;
	constexpr aa_box(const vec3<T>& mn, const vec3<T>& mx) : mins(mn), maxs(mx) {}
	constexpr aa_box(T minx, T miny, T minz, T maxx, T maxy, T maxz)
		: mins(minx, miny, minz), maxs(maxx, maxy, maxz) {}

	explicit aa_box(T radius)
		: mins(-radius, -radius, -radius), maxs(radius, radius, radius) {}

	aa_box& set(const aa_box& b) { mins = b.mins; maxs = b.maxs; return *this; }
	aa_box& set(const vec3<T>& mn, const vec3<T>& mx) { mins = mn; maxs = mx; return *this; }
	aa_box& set(T radius) {
		mins.set(-radius, -radius, -radius);
		maxs.set(radius, radius, radius);
		return *this;
	}
	aa_box& reset() { mins.reset(); maxs.reset(); return *this; }

	void merge(const aa_box& b) {
		using traits = scalar_traits<T>;
		mins.x = traits::min_val(mins.x, b.mins.x);
		mins.y = traits::min_val(mins.y, b.mins.y);
		mins.z = traits::min_val(mins.z, b.mins.z);
		maxs.x = traits::max_val(maxs.x, b.maxs.x);
		maxs.y = traits::max_val(maxs.y, b.maxs.y);
		maxs.z = traits::max_val(maxs.z, b.maxs.z);
	}

	void merge(const vec3<T>& v) {
		using traits = scalar_traits<T>;
		mins.x = traits::min_val(mins.x, v.x);
		mins.y = traits::min_val(mins.y, v.y);
		mins.z = traits::min_val(mins.z, v.z);
		maxs.x = traits::max_val(maxs.x, v.x);
		maxs.y = traits::max_val(maxs.y, v.y);
		maxs.z = traits::max_val(maxs.z, v.z);
	}

	bool operator==(const aa_box& b) const { return mins == b.mins && maxs == b.maxs; }
	bool operator!=(const aa_box& b) const { return !(*this == b); }

	aa_box operator+(const vec3<T>& v) const { return {mins + v, maxs + v}; }
	void operator+=(const vec3<T>& v) { mins += v; maxs += v; }
	aa_box operator-(const vec3<T>& v) const { return {mins - v, maxs - v}; }
	void operator-=(const vec3<T>& v) { mins -= v; maxs -= v; }
};

} // namespace hop
