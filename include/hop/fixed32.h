#pragma once

#include <cstdint>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace hop {
namespace detail {

// Platform-specific 128-bit helpers for Q32.32 arithmetic.
// All other fixed32 code goes through these two functions instead of spelling
// out __int128 or MSVC intrinsics directly.

#if defined(__SIZEOF_INT128__)
	using i128 = __int128;

	inline constexpr int64_t mul_shr32(int64_t a, int64_t b) noexcept {
		return static_cast<int64_t>((i128(a) * b) >> 32);
	}
	inline constexpr int64_t shl32_div(int64_t a, int64_t b) noexcept {
		return static_cast<int64_t>((i128(a) << 32) / b);
	}

#elif defined(_MSC_VER)
	// _mul128: VS2005+  _div128: VS2019+ (both x64/ARM64 only)
	inline int64_t mul_shr32(int64_t a, int64_t b) noexcept {
		int64_t h;
		uint64_t l = static_cast<uint64_t>(_mul128(a, b, &h));
		return static_cast<int64_t>((static_cast<uint64_t>(h) << 32) | (l >> 32));
	}
	inline int64_t shl32_div(int64_t a, int64_t b) noexcept {
		int64_t rem;
		return _div128(a >> 32, static_cast<uint64_t>(a) << 32, b, &rem);
	}

#else
	#error "fixed32 requires __int128 (GCC/Clang) or MSVC x64/ARM64 intrinsics (_mul128/_div128)"
#endif

} // namespace detail

struct fixed32 {
	struct raw_tag {};

	int64_t raw = 0;

	static constexpr int bits = 32;
	static constexpr int64_t one_raw = 1LL << bits;

	constexpr fixed32() = default;
	constexpr explicit fixed32(int64_t r, raw_tag) : raw(r) {}

	static constexpr fixed32 from_raw(int64_t r) { return fixed32(r, raw_tag {}); }
	static constexpr fixed32 from_int(int i) { return from_raw(static_cast<int64_t>(i) << bits); }
	static constexpr fixed32 from_float(float f) { return from_raw(static_cast<int64_t>(f * one_raw)); }
	// INT_MAX * 2^32 < INT64_MAX, so plain int64 arithmetic suffices here.
	static constexpr fixed32 from_milli(int m) {
		return from_raw(static_cast<int64_t>(m) * (1LL << 32) / 1000);
	}

	constexpr int to_int() const { return static_cast<int>(raw >> bits); }
	constexpr float to_float() const { return static_cast<float>(raw) / static_cast<float>(one_raw); }

	// Arithmetic operators
	constexpr fixed32 operator+(fixed32 b) const { return from_raw(raw + b.raw); }
	constexpr fixed32 operator-(fixed32 b) const { return from_raw(raw - b.raw); }
	constexpr fixed32 operator-() const { return from_raw(-raw); }

	constexpr fixed32 operator*(fixed32 b) const { return from_raw(detail::mul_shr32(raw, b.raw)); }

	constexpr fixed32 operator/(fixed32 b) const {
		if (b.raw == 0)
			return from_raw(0);
		return from_raw(detail::shl32_div(raw, b.raw));
	}

	constexpr fixed32 operator%(fixed32 b) const { return from_raw(raw % b.raw); }

	constexpr fixed32 & operator+=(fixed32 b) {
		raw += b.raw;
		return *this;
	}
	constexpr fixed32 & operator-=(fixed32 b) {
		raw -= b.raw;
		return *this;
	}
	constexpr fixed32 & operator*=(fixed32 b) {
		*this = *this * b;
		return *this;
	}
	constexpr fixed32 & operator/=(fixed32 b) {
		*this = *this / b;
		return *this;
	}

	// Comparison operators
	constexpr bool operator==(fixed32 b) const { return raw == b.raw; }
	constexpr bool operator!=(fixed32 b) const { return raw != b.raw; }
	constexpr bool operator<(fixed32 b) const { return raw < b.raw; }
	constexpr bool operator<=(fixed32 b) const { return raw <= b.raw; }
	constexpr bool operator>(fixed32 b) const { return raw > b.raw; }
	constexpr bool operator>=(fixed32 b) const { return raw >= b.raw; }

	// Integer literal support
	constexpr fixed32 operator+(int b) const { return *this + from_int(b); }
	constexpr fixed32 operator-(int b) const { return *this - from_int(b); }
	constexpr fixed32 operator*(int b) const { return from_raw(raw * b); }
	constexpr fixed32 operator/(int b) const {
		if (b == 0)
			return from_raw(0);
		return from_raw(raw / b);
	}

	constexpr bool operator==(int b) const { return raw == (static_cast<int64_t>(b) << bits); }
	constexpr bool operator!=(int b) const { return raw != (static_cast<int64_t>(b) << bits); }
	constexpr bool operator<(int b) const { return raw < (static_cast<int64_t>(b) << bits); }
	constexpr bool operator<=(int b) const { return raw <= (static_cast<int64_t>(b) << bits); }
	constexpr bool operator>(int b) const { return raw > (static_cast<int64_t>(b) << bits); }
	constexpr bool operator>=(int b) const { return raw >= (static_cast<int64_t>(b) << bits); }
};

// Commutative int ops
constexpr fixed32 operator+(int a, fixed32 b) { return fixed32::from_int(a) + b; }
constexpr fixed32 operator-(int a, fixed32 b) { return fixed32::from_int(a) - b; }
constexpr fixed32 operator*(int a, fixed32 b) { return fixed32::from_raw(a * b.raw); }
constexpr fixed32 operator/(int a, fixed32 b) { return fixed32::from_int(a) / b; }

} // namespace hop
