#pragma once

#include <hop/math/mat3.h>
#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

namespace hop {

// Unit quat for orientation. Default is identity (0,0,0,1).
//
// Layout: (x, y, z) is the vector part; w is the scalar part.
// Multiplication is right-handed and applies the right-hand operand first
// when rotating vectors: (q1 * q2) * v == q1 * (q2 * v).
template <typename T> struct quat {
	using tr = scalar_traits<T>;

	T x {}, y {}, z {}, w;

	quat() : w(tr::one()) {}
	quat(T x_, T y_, T z_, T w_) : x(x_), y(y_), z(z_), w(w_) {}

	quat & set(const quat & q) {
		x = q.x; y = q.y; z = q.z; w = q.w;
		return *this;
	}
	quat & set(T x_, T y_, T z_, T w_) {
		x = x_; y = y_; z = z_; w = w_;
		return *this;
	}
	quat & reset() {
		x = T {}; y = T {}; z = T {}; w = tr::one();
		return *this;
	}

	bool operator==(const quat & q) const { return x == q.x && y == q.y && z == q.z && w == q.w; }
	bool operator!=(const quat & q) const { return !(*this == q); }

	T & operator[](int i) { return *((&x) + i); }
	T operator[](int i) const { return *((&x) + i); }

	// Element-wise arithmetic — used for time integration (q + dq*dt)
	quat operator+(const quat & q) const { return { x + q.x, y + q.y, z + q.z, w + q.w }; }
	quat operator-(const quat & q) const { return { x - q.x, y - q.y, z - q.z, w - q.w }; }
	quat operator-() const { return { -x, -y, -z, -w }; }
	quat operator*(T s) const { return { x * s, y * s, z * s, w * s }; }

	void operator+=(const quat & q) { x += q.x; y += q.y; z += q.z; w += q.w; }
	void operator-=(const quat & q) { x -= q.x; y -= q.y; z -= q.z; w -= q.w; }
	void operator*=(T s) { x *= s; y *= s; z *= s; w *= s; }
};

template <typename T> quat<T> operator*(T s, const quat<T> & q) { return { q.x * s, q.y * s, q.z * s, q.w * s }; }

// Function-form element-wise arithmetic — mirrors vec3/aa_box/sphere/capsule
// so callers aren't forced to switch styles when working with quats.
// Hamilton product is `mul(r, q1, q2)` below (NOT element-wise).

template <typename T> inline void neg(quat<T> & q) {
	q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
}
template <typename T> inline void neg(quat<T> & r, const quat<T> & q) {
	r.x = -q.x; r.y = -q.y; r.z = -q.z; r.w = -q.w;
}

template <typename T> inline void add(quat<T> & r, const quat<T> & a, const quat<T> & b) {
	r.x = a.x + b.x; r.y = a.y + b.y; r.z = a.z + b.z; r.w = a.w + b.w;
}
template <typename T> inline void add(quat<T> & r, const quat<T> & q) {
	r.x += q.x; r.y += q.y; r.z += q.z; r.w += q.w;
}

template <typename T> inline void sub(quat<T> & r, const quat<T> & a, const quat<T> & b) {
	r.x = a.x - b.x; r.y = a.y - b.y; r.z = a.z - b.z; r.w = a.w - b.w;
}
template <typename T> inline void sub(quat<T> & r, const quat<T> & q) {
	r.x -= q.x; r.y -= q.y; r.z -= q.z; r.w -= q.w;
}

template <typename T> inline void mul(quat<T> & r, const quat<T> & q, T s) {
	r.x = q.x * s; r.y = q.y * s; r.z = q.z * s; r.w = q.w * s;
}
template <typename T> inline void mul(quat<T> & r, T s) {
	r.x *= s; r.y *= s; r.z *= s; r.w *= s;
}

// r = q1 * q2  (Hamilton product, right-to-left rotation composition).
// Pre: &r != &q1 && &r != &q2. Use post_mul for in-place (r = r * q2).
template <typename T> inline void mul(quat<T> & r, const quat<T> & q1, const quat<T> & q2) {
	r.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	r.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	r.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	r.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
}

// In-place Hamilton product: q1 := q1 * q2.
template <typename T> inline void post_mul(quat<T> & q1, const quat<T> & q2) {
	T x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
	T y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
	T z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
	T w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
	q1.x = x; q1.y = y; q1.z = z; q1.w = w;
}

// r = q rotating v. Assumes q is a unit quat — for non-unit q the result is
// scaled by |q|^2. Safe to call with &r == &v.
template <typename T> inline void mul(vec3<T> & r, const quat<T> & q, const vec3<T> & v) {
	T x =  q.y * v.z - q.z * v.y + q.w * v.x;
	T y = -q.x * v.z + q.z * v.x + q.w * v.y;
	T z =  q.x * v.y - q.y * v.x + q.w * v.z;
	T w = -q.x * v.x - q.y * v.y - q.z * v.z;

	r.x =  x * q.w + y * -q.z - z * -q.y + w * -q.x;
	r.y = -x * -q.z + y * q.w + z * -q.x + w * -q.y;
	r.z =  x * -q.y - y * -q.x + z * q.w + w * -q.z;
}

template <typename T> inline T length_squared(const quat<T> & q) {
	return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

template <typename T> inline T length(const quat<T> & q) {
	using tr = scalar_traits<T>;
	return tr::sqrt(length_squared(q));
}

template <typename T> inline T dot(const quat<T> & q1, const quat<T> & q2) {
	return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

template <typename T> inline void normalize(quat<T> & q) {
	using tr = scalar_traits<T>;
	T inv_l = tr::one() / length(q);
	q.x *= inv_l; q.y *= inv_l; q.z *= inv_l; q.w *= inv_l;
}

template <typename T> inline bool normalize_carefully(quat<T> & q, T epsilon) {
	using tr = scalar_traits<T>;
	T l = length(q);
	if (l > epsilon) {
		T inv_l = tr::one() / l;
		q.x *= inv_l; q.y *= inv_l; q.z *= inv_l; q.w *= inv_l;
		return true;
	}
	return false;
}

template <typename T> inline void conjugate(quat<T> & q) {
	q.x = -q.x; q.y = -q.y; q.z = -q.z;
}

template <typename T> inline void conjugate(quat<T> & r, const quat<T> & q) {
	r.x = -q.x; r.y = -q.y; r.z = -q.z; r.w = q.w;
}

template <typename T> inline void invert(quat<T> & r, const quat<T> & q) {
	using tr = scalar_traits<T>;
	T inv_l2 = tr::one() / length_squared(q);
	r.x = -q.x * inv_l2;
	r.y = -q.y * inv_l2;
	r.z = -q.z * inv_l2;
	r.w =  q.w * inv_l2;
}

// Build a unit quat from axis (unit vector) + angle (radians).
template <typename T>
inline void set_quat_from_axis_angle(quat<T> & r, const vec3<T> & axis, T angle) {
	using tr = scalar_traits<T>;
	T half_angle = angle * tr::half();
	T sin_half = tr::sin(half_angle);
	r.x = axis.x * sin_half;
	r.y = axis.y * sin_half;
	r.z = axis.z * sin_half;
	r.w = tr::cos(half_angle);
}

// Extract axis + angle from a unit quat. Returns angle in radians ∈ [0, π].
// `axis` is written with the unit axis of rotation (or (1,0,0) if undefined).
//
// Uses angle = 2 * asin(|q.xyz|) rather than 2 * acos(q.w) so the small-rotation
// case (q.w ≈ 1) stays accurate: float acos near 1 has poor conditioning,
// whereas asin(x) ≈ x for small x. |q.xyz| doubles as sin(angle/2), so no
// second trig call is needed for the axis normalization.
//
// Canonicalizes q.w ≥ 0 internally (since -q represents the same rotation).
template <typename T>
inline T get_axis_angle_from_quat(vec3<T> & axis, const quat<T> & q, T epsilon) {
	using tr = scalar_traits<T>;
	T qx = q.x, qy = q.y, qz = q.z, qw = q.w;
	if (qw < T {}) {
		qx = -qx; qy = -qy; qz = -qz; qw = -qw;
	}
	T sin_half = tr::sqrt(qx * qx + qy * qy + qz * qz);
	// Clamp against rounding: a unit quat has sin_half ≤ 1, but fixed16
	// accumulation can nudge it slightly over.
	if (sin_half > tr::one())
		sin_half = tr::one();
	T angle = tr::asin(sin_half) * tr::two();
	if (sin_half > epsilon) {
		T inv = tr::one() / sin_half;
		axis.x = qx * inv;
		axis.y = qy * inv;
		axis.z = qz * inv;
	} else {
		axis.x = tr::one(); axis.y = T {}; axis.z = T {};
	}
	return angle;
}

// Fill rotation matrix r from unit quat q. r is column-major.
template <typename T> inline void set_mat3_from_quat(mat3<T> & r, const quat<T> & q) {
	using tr = scalar_traits<T>;
	T two = tr::two();
	T tx  = q.x * two;
	T ty  = q.y * two;
	T tz  = q.z * two;
	T twx = tx * q.w;
	T twy = ty * q.w;
	T twz = tz * q.w;
	T txx = tx * q.x;
	T txy = ty * q.x;
	T txz = tz * q.x;
	T tyy = ty * q.y;
	T tyz = tz * q.y;
	T tzz = tz * q.z;

	r.set_at(0, 0, tr::one() - (tyy + tzz));  r.set_at(0, 1, txy - twz);               r.set_at(0, 2, txz + twy);
	r.set_at(1, 0, txy + twz);                r.set_at(1, 1, tr::one() - (txx + tzz)); r.set_at(1, 2, tyz - twx);
	r.set_at(2, 0, txz - twy);                r.set_at(2, 1, tyz + twx);               r.set_at(2, 2, tr::one() - (txx + tyy));
}

// Extract a unit quat from a pure-rotation matrix. Ken Shoemake's algorithm.
// Assumes m is orthonormal (pure rotation). If m contains scale, normalize m first.
template <typename T> inline void set_quat_from_mat3(quat<T> & r, const mat3<T> & m) {
	using tr = scalar_traits<T>;
	T trace = m.at(0, 0) + m.at(1, 1) + m.at(2, 2);
	T root;

	if (trace > T {}) {
		// |w| > 1/2 — choose w > 1/2 path
		root = tr::sqrt(trace + tr::one());  // = 2w
		r.w = root * tr::half();
		root = tr::half() / root;  // = 1/(4w)
		r.x = (m.at(2, 1) - m.at(1, 2)) * root;
		r.y = (m.at(0, 2) - m.at(2, 0)) * root;
		r.z = (m.at(1, 0) - m.at(0, 1)) * root;
	} else {
		static const int next[3] = { 1, 2, 0 };
		int i = 0;
		if (m.at(1, 1) > m.at(0, 0)) i = 1;
		if (m.at(2, 2) > m.at(i, i)) i = 2;
		int j = next[i];
		int k = next[j];

		root = tr::sqrt(m.at(i, i) - m.at(j, j) - m.at(k, k) + tr::one());
		r[i] = root * tr::half();
		root = tr::half() / root;
		r.w = (m.at(k, j) - m.at(j, k)) * root;
		r[j] = (m.at(j, i) + m.at(i, j)) * root;
		r[k] = (m.at(k, i) + m.at(i, k)) * root;
	}
}

template <typename T>
inline void lerp(quat<T> & r, const quat<T> & q1, const quat<T> & q2, T t) {
	using tr = scalar_traits<T>;
	T cosom = dot(q1, q2);
	T scl1 = tr::one() - t;
	T scl2 = (cosom < T {}) ? -t : t;
	r.x = scl1 * q1.x + scl2 * q2.x;
	r.y = scl1 * q1.y + scl2 * q2.y;
	r.z = scl1 * q1.z + scl2 * q2.z;
	r.w = scl1 * q1.w + scl2 * q2.w;
}

// Spherical linear interpolation between q1 and q2 at parameter t ∈ [0, 1].
// Falls back to lerp when quats are nearly parallel, to avoid division by ~0.
template <typename T>
inline void slerp(quat<T> & r, const quat<T> & q1, const quat<T> & q2, T t) {
	using tr = scalar_traits<T>;
	T cosom = dot(q1, q2);
	T scl2_sign = tr::one();
	if (cosom < T {}) {
		cosom = -cosom;
		scl2_sign = -tr::one();
	}

	T scl1, scl2;
	// Threshold chosen to match toadlet — in fixed16 raw 6/65536 ≈ 9e-5. For float,
	// this is effectively "always do slerp."
	T threshold;
	if constexpr (std::is_same_v<T, fixed16>)
		threshold = fixed16::from_raw(6);
	else
		threshold = T(1e-4);

	if ((tr::one() - cosom) > threshold) {
		T omega = tr::acos(cosom);
		T sinom = tr::sin(omega);
		scl1 = tr::sin((tr::one() - t) * omega) / sinom;
		scl2 = tr::sin(t * omega) / sinom;
	} else {
		scl1 = tr::one() - t;
		scl2 = t;
	}
	scl2 *= scl2_sign;

	r.x = scl1 * q1.x + scl2 * q2.x;
	r.y = scl1 * q1.y + scl2 * q2.y;
	r.z = scl1 * q1.z + scl2 * q2.z;
	r.w = scl1 * q1.w + scl2 * q2.w;
}

} // namespace hop
