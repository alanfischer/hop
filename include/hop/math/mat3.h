#pragma once

#include <hop/math/vec3.h>
#include <hop/scalar_traits.h>

namespace hop {

// Column-major 3x3 matrix. data[row + col*3] — matches toadlet's layout.
// Defaults to identity.
template <typename T> struct mat3 {
	using tr = scalar_traits<T>;

	T data[9];

	mat3() { reset(); }

	mat3(T x1, T x2, T x3, T y1, T y2, T y3, T z1, T z2, T z3) {
		data[0] = x1; data[3] = x2; data[6] = x3;
		data[1] = y1; data[4] = y2; data[7] = y3;
		data[2] = z1; data[5] = z2; data[8] = z3;
	}

	mat3 & set(const mat3 & m) {
		for (int i = 0; i < 9; ++i)
			data[i] = m.data[i];
		return *this;
	}

	mat3 & reset() {
		data[0] = tr::one();  data[3] = T {};       data[6] = T {};
		data[1] = T {};       data[4] = tr::one();  data[7] = T {};
		data[2] = T {};       data[5] = T {};       data[8] = tr::one();
		return *this;
	}

	T at(int row, int col) const { return data[row + col * 3]; }
	T & at(int row, int col) { return data[row + col * 3]; }
	void set_at(int row, int col, T v) { data[row + col * 3] = v; }

	bool operator==(const mat3 & m) const {
		for (int i = 0; i < 9; ++i)
			if (data[i] != m.data[i]) return false;
		return true;
	}
	bool operator!=(const mat3 & m) const { return !(*this == m); }
};

// r = m1 * m2
template <typename T> inline void mul(mat3<T> & r, const mat3<T> & m1, const mat3<T> & m2) {
	// Assumes &r != &m1 and &r != &m2 — callers must use a temp if needed.
	for (int row = 0; row < 3; ++row) {
		for (int col = 0; col < 3; ++col) {
			r.data[row + col * 3] =
			    m1.data[row + 0 * 3] * m2.data[0 + col * 3] +
			    m1.data[row + 1 * 3] * m2.data[1 + col * 3] +
			    m1.data[row + 2 * 3] * m2.data[2 + col * 3];
		}
	}
}

// r = m * v. Safe to call with &r == &v.
template <typename T> inline void mul(vec3<T> & r, const mat3<T> & m, const vec3<T> & v) {
	T vx = v.x, vy = v.y, vz = v.z;  // snapshot — r and v may alias
	r.x = m.data[0 + 0 * 3] * vx + m.data[0 + 1 * 3] * vy + m.data[0 + 2 * 3] * vz;
	r.y = m.data[1 + 0 * 3] * vx + m.data[1 + 1 * 3] * vy + m.data[1 + 2 * 3] * vz;
	r.z = m.data[2 + 0 * 3] * vx + m.data[2 + 1 * 3] * vy + m.data[2 + 2 * 3] * vz;
}

// r = transpose(m). Safe to call with &r == &m.
template <typename T> inline void transpose(mat3<T> & r, const mat3<T> & m) {
	// Swap via a local since r and m may alias.
	T d0 = m.data[0], d1 = m.data[1], d2 = m.data[2];
	T d3 = m.data[3], d4 = m.data[4], d5 = m.data[5];
	T d6 = m.data[6], d7 = m.data[7], d8 = m.data[8];
	r.data[0] = d0; r.data[3] = d1; r.data[6] = d2;
	r.data[1] = d3; r.data[4] = d4; r.data[7] = d5;
	r.data[2] = d6; r.data[5] = d7; r.data[8] = d8;
}

template <typename T> inline T determinant(const mat3<T> & m) {
	return  -m.data[0] * m.data[4] * m.data[8] + m.data[0] * m.data[7] * m.data[5]
	      +  m.data[3] * m.data[1] * m.data[8] - m.data[3] * m.data[2] * m.data[7]
	      -  m.data[6] * m.data[1] * m.data[5] + m.data[6] * m.data[2] * m.data[4];
}

// r = inverse(m). Returns false if singular. Safe to call with &r == &m.
template <typename T> inline bool invert(mat3<T> & r, const mat3<T> & m) {
	using tr = scalar_traits<T>;
	T det = determinant(m);
	if (det == T {})
		return false;
	T inv_det = tr::one() / det;
	// Snapshot m into locals — r and m may alias, and each output cell reads
	// multiple input cells.
	T m0 = m.data[0], m1 = m.data[1], m2 = m.data[2];
	T m3 = m.data[3], m4 = m.data[4], m5 = m.data[5];
	T m6 = m.data[6], m7 = m.data[7], m8 = m.data[8];
	r.data[0] = -(m4 * m8 - m7 * m5) * inv_det;
	r.data[1] =  (m1 * m8 - m7 * m2) * inv_det;
	r.data[2] = -(m1 * m5 - m4 * m2) * inv_det;
	r.data[3] =  (m3 * m8 - m6 * m5) * inv_det;
	r.data[4] = -(m0 * m8 - m6 * m2) * inv_det;
	r.data[5] =  (m0 * m5 - m3 * m2) * inv_det;
	r.data[6] = -(m3 * m7 - m6 * m4) * inv_det;
	r.data[7] =  (m0 * m7 - m6 * m1) * inv_det;
	r.data[8] = -(m0 * m4 - m3 * m1) * inv_det;
	return true;
}

// Build a rotation matrix from axis (unit vector) + angle (radians).
template <typename T> inline void set_mat3_from_axis_angle(mat3<T> & r, const vec3<T> & axis, T angle) {
	using tr = scalar_traits<T>;
	T c = tr::cos(angle);
	T s = tr::sin(angle);
	T t = tr::one() - c;

	r.set_at(0, 0, c + axis.x * axis.x * t);
	r.set_at(1, 1, c + axis.y * axis.y * t);
	r.set_at(2, 2, c + axis.z * axis.z * t);

	T tmp1 = axis.x * axis.y * t;
	T tmp2 = axis.z * s;
	r.set_at(1, 0, tmp1 + tmp2);
	r.set_at(0, 1, tmp1 - tmp2);

	tmp1 = axis.x * axis.z * t;
	tmp2 = axis.y * s;
	r.set_at(2, 0, tmp1 - tmp2);
	r.set_at(0, 2, tmp1 + tmp2);

	tmp1 = axis.y * axis.z * t;
	tmp2 = axis.x * s;
	r.set_at(2, 1, tmp1 + tmp2);
	r.set_at(1, 2, tmp1 - tmp2);
}

} // namespace hop
