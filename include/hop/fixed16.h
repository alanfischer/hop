#pragma once

#include <cstdint>

namespace hop {

struct fixed16 {
	struct raw_tag {};

	int32_t raw = 0;

	static constexpr int bits = 16;
	static constexpr int32_t one_raw = 1 << bits;

	constexpr fixed16() = default;
	constexpr explicit fixed16(int32_t r, raw_tag) : raw(r) {}

	static constexpr fixed16 from_raw(int32_t r) { return fixed16(r, raw_tag {}); }
	static constexpr fixed16 from_int(int i) { return from_raw(i << bits); }
	static constexpr fixed16 from_float(float f) { return from_raw(static_cast<int32_t>(f * one_raw)); }
	static constexpr fixed16 from_milli(int m) {
		return from_raw(static_cast<int32_t>(((static_cast<int64_t>(m) << 32) / 1000) >> bits));
	}

	constexpr int to_int() const { return raw >> bits; }
	constexpr float to_float() const { return static_cast<float>(raw) / static_cast<float>(one_raw); }

	// Arithmetic operators
	constexpr fixed16 operator+(fixed16 b) const { return from_raw(raw + b.raw); }
	constexpr fixed16 operator-(fixed16 b) const { return from_raw(raw - b.raw); }
	constexpr fixed16 operator-() const { return from_raw(-raw); }

	constexpr fixed16 operator*(fixed16 b) const {
		return from_raw(static_cast<int32_t>((static_cast<int64_t>(raw) * b.raw) >> bits));
	}

	constexpr fixed16 operator/(fixed16 b) const {
		if (b.raw == 0)
			return from_raw(0);
		return from_raw(static_cast<int32_t>((((static_cast<int64_t>(raw)) << 32) / b.raw) >> bits));
	}

	constexpr fixed16 operator%(fixed16 b) const { return from_raw(raw % b.raw); }

	constexpr fixed16 & operator+=(fixed16 b) {
		raw += b.raw;
		return *this;
	}
	constexpr fixed16 & operator-=(fixed16 b) {
		raw -= b.raw;
		return *this;
	}
	constexpr fixed16 & operator*=(fixed16 b) {
		*this = *this * b;
		return *this;
	}
	constexpr fixed16 & operator/=(fixed16 b) {
		*this = *this / b;
		return *this;
	}

	// Comparison operators
	constexpr bool operator==(fixed16 b) const { return raw == b.raw; }
	constexpr bool operator!=(fixed16 b) const { return raw != b.raw; }
	constexpr bool operator<(fixed16 b) const { return raw < b.raw; }
	constexpr bool operator<=(fixed16 b) const { return raw <= b.raw; }
	constexpr bool operator>(fixed16 b) const { return raw > b.raw; }
	constexpr bool operator>=(fixed16 b) const { return raw >= b.raw; }

	// Integer literal support
	constexpr fixed16 operator+(int b) const { return *this + from_int(b); }
	constexpr fixed16 operator-(int b) const { return *this - from_int(b); }
	constexpr fixed16 operator*(int b) const { return from_raw(raw * b); }
	constexpr fixed16 operator/(int b) const {
		if (b == 0)
			return from_raw(0);
		return from_raw(raw / b);
	}

	constexpr bool operator==(int b) const { return raw == (b << bits); }
	constexpr bool operator!=(int b) const { return raw != (b << bits); }
	constexpr bool operator<(int b) const { return raw < (b << bits); }
	constexpr bool operator<=(int b) const { return raw <= (b << bits); }
	constexpr bool operator>(int b) const { return raw > (b << bits); }
	constexpr bool operator>=(int b) const { return raw >= (b << bits); }
};

// Commutative int ops
constexpr fixed16 operator+(int a, fixed16 b) { return fixed16::from_int(a) + b; }
constexpr fixed16 operator-(int a, fixed16 b) { return fixed16::from_int(a) - b; }
constexpr fixed16 operator*(int a, fixed16 b) { return fixed16::from_raw(a * b.raw); }
constexpr fixed16 operator/(int a, fixed16 b) { return fixed16::from_int(a) / b; }

} // namespace hop
