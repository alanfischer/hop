#pragma once

#include <cmath>
#include <hop/fixed16.h>

namespace hop {

// Epsilon state — differs between float (stores 1/epsilon) and fixed16 (stores bit count)
template<typename T> struct epsilon_state;

template<>
struct epsilon_state<float> {
	float epsilon = 0;
	float half_epsilon = 0;
	float quarter_epsilon = 0;
	float one_over_epsilon = 0;
};

template<>
struct epsilon_state<fixed16> {
	int epsilon_bits = 0;
	fixed16 epsilon = {};
	fixed16 half_epsilon = {};
	fixed16 quarter_epsilon = {};
};

// scalar_traits<float>
template<typename T> struct scalar_traits;

template<>
struct scalar_traits<float> {
	using type = float;

	static constexpr float one() { return 1.0f; }
	static constexpr float zero() { return 0.0f; }
	static constexpr float half() { return 0.5f; }
	static constexpr float two() { return 2.0f; }
	static constexpr float three() { return 3.0f; }
	static constexpr float four() { return 4.0f; }
	static constexpr float quarter() { return 0.25f; }
	static constexpr float third() { return 1.0f / 3.0f; }
	static constexpr float two_thirds() { return 2.0f / 3.0f; }
	static constexpr float pi() { return 3.14159265358979323846f; }
	static constexpr float two_pi() { return pi() * 2.0f; }
	static constexpr float half_pi() { return pi() / 2.0f; }

	static constexpr float from_milli(int m) { return static_cast<float>(m) / 1000.0f; }
	static constexpr float from_int(int i) { return static_cast<float>(i); }
	static constexpr int to_int(float v) { return static_cast<int>(v); }
	static constexpr float to_float(float v) { return v; }

	static float abs(float v) { return std::fabs(v); }
	static float sqrt(float v) { return std::sqrt(v); }
	static float sin(float v) { return std::sin(v); }
	static float cos(float v) { return std::cos(v); }
	static float atan2(float y, float x) { return std::atan2(y, x); }

	static bool is_real(float v) { return !std::isnan(v) && !std::isinf(v); }

	static float min_val(float a, float b) { return a < b ? a : b; }
	static float max_val(float a, float b) { return a > b ? a : b; }
	static float clamp(float low, float high, float v) {
		return min_val(high, max_val(low, v));
	}

	static float mul(float a, float b) { return a * b; }
	static float div(float a, float b) { return a / b; }
	static float madd(float a, float b, float c) { return a * b + c; }

	// Epsilon
	static void make_epsilon(epsilon_state<float>& s, float epsilon) {
		s.epsilon = epsilon;
		s.one_over_epsilon = 1.0f / epsilon;
		s.half_epsilon = epsilon * 0.5f;
		s.quarter_epsilon = epsilon * 0.25f;
	}

	static float default_epsilon() { return 0.001f; }
	static float default_max_position_component() { return 100000.0f; }
	static float default_max_velocity_component() { return 1000.0f; }
	static float default_max_force_component() { return 1000.0f; }
	static float default_deactivate_speed(const epsilon_state<float>& s) { return s.epsilon * 2.0f; }

	// Cap — clamp + NaN guard
	static float cap(float v, float limit) {
		v = max_val(-limit, v);
		v = min_val(limit, v);
		return is_real(v) ? v : 0.0f;
	}

	// Snap to grid (float version)
	static void snap_to_grid(float& v, const epsilon_state<float>& s) {
		v = static_cast<int>((v + (s.half_epsilon * -(v < 0))) * s.one_over_epsilon) * s.epsilon;
	}
};

// scalar_traits<fixed16>
template<>
struct scalar_traits<fixed16> {
	using type = fixed16;

	static constexpr fixed16 one() { return fixed16::from_raw(65536); }
	static constexpr fixed16 zero() { return fixed16::from_raw(0); }
	static constexpr fixed16 half() { return fixed16::from_raw(32768); }
	static constexpr fixed16 two() { return fixed16::from_raw(131072); }
	static constexpr fixed16 three() { return fixed16::from_raw(196608); }
	static constexpr fixed16 four() { return fixed16::from_raw(262144); }
	static constexpr fixed16 quarter() { return fixed16::from_raw(16384); }
	static constexpr fixed16 third() { return fixed16::from_raw(21845); }
	static constexpr fixed16 two_thirds() { return fixed16::from_raw(43690); }
	static constexpr fixed16 pi() { return fixed16::from_raw(205887); }
	static constexpr fixed16 two_pi() { return fixed16::from_raw(411774); }
	static constexpr fixed16 half_pi() { return fixed16::from_raw(102943); }

	static constexpr fixed16 from_milli(int m) { return fixed16::from_milli(m); }
	static constexpr fixed16 from_int(int i) { return fixed16::from_int(i); }
	static constexpr int to_int(fixed16 v) { return v.to_int(); }
	static constexpr float to_float(fixed16 v) { return v.to_float(); }

	// Bit-hack abs
	static constexpr fixed16 abs(fixed16 v) {
		return fixed16::from_raw((v.raw ^ (v.raw >> 31)) - (v.raw >> 31));
	}

	// Newton-Raphson sqrt (8 iterations)
	static fixed16 sqrt(fixed16 v) {
		if (v.raw <= 0) return zero();
		int32_t s = (v.raw + fixed16::one_raw) >> 1;
		for (int i = 0; i < 8; ++i) {
			s = (s + static_cast<int32_t>(((static_cast<int64_t>(v.raw) << 32) / s) >> 16)) >> 1;
		}
		return fixed16::from_raw(s);
	}

	// Polynomial sin (from toadlet mathfixed)
	static fixed16 sin(fixed16 f) {
		constexpr int32_t two_pi_raw = 411774;
		constexpr int32_t pi_raw = 205887;
		constexpr int32_t half_pi_raw = 102943;
		constexpr int32_t one_raw = 65536;

		if (f.raw < 0) f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw) f.raw = f.raw % two_pi_raw;

		int sign = 1;
		if (f.raw > half_pi_raw && f.raw <= pi_raw) {
			f.raw = pi_raw - f.raw;
		} else if (f.raw > pi_raw && f.raw <= pi_raw + half_pi_raw) {
			f.raw = f.raw - pi_raw;
			sign = -1;
		} else if (f.raw > pi_raw + half_pi_raw) {
			f.raw = two_pi_raw - f.raw;
			sign = -1;
		}

		int32_t sqr = static_cast<int32_t>((static_cast<int64_t>(f.raw) * f.raw) >> 16);
		int32_t result = 498;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * sqr) >> 16);
		result -= 10882;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * sqr) >> 16);
		result += one_raw;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		return fixed16::from_raw(sign * result);
	}

	// Polynomial cos (from toadlet mathfixed)
	static fixed16 cos(fixed16 f) {
		constexpr int32_t two_pi_raw = 411774;
		constexpr int32_t pi_raw = 205887;
		constexpr int32_t half_pi_raw = 102943;
		constexpr int32_t one_raw = 65536;

		if (f.raw < 0) f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw) f.raw = f.raw % two_pi_raw;

		int sign = 1;
		if (f.raw > half_pi_raw && f.raw <= pi_raw) {
			f.raw = pi_raw - f.raw;
			sign = -1;
		} else if (f.raw > half_pi_raw && f.raw <= pi_raw + half_pi_raw) {
			f.raw = f.raw - pi_raw;
			sign = -1;
		} else if (f.raw > pi_raw + half_pi_raw) {
			f.raw = two_pi_raw - f.raw;
		}

		int32_t sqr = static_cast<int32_t>((static_cast<int64_t>(f.raw) * f.raw) >> 16);
		int32_t result = 2328;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * sqr) >> 16);
		result -= 32551;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * sqr) >> 16);
		result += one_raw;
		return fixed16::from_raw(result * sign);
	}

	// Polynomial atan2 (from toadlet mathfixed)
	static fixed16 atan2(fixed16 y, fixed16 x) {
		constexpr int32_t quarter_pi_raw = 51471;
		constexpr int32_t three_quarter_pi_raw = 154414;

		int32_t absy = ((y.raw ^ (y.raw >> 31)) - (y.raw >> 31)) + 1;
		int32_t angle;
		if (x.raw >= 0) {
			int32_t r = static_cast<int32_t>((((static_cast<int64_t>(x.raw - absy)) << 32) / (x.raw + absy)) >> 16);
			angle = quarter_pi_raw - static_cast<int32_t>((static_cast<int64_t>(quarter_pi_raw) * r) >> 16);
		} else {
			int32_t r = static_cast<int32_t>((((static_cast<int64_t>(x.raw + absy)) << 32) / (y.raw - absy)) >> 16);
			angle = three_quarter_pi_raw - static_cast<int32_t>((static_cast<int64_t>(quarter_pi_raw) * r) >> 16);
		}
		return fixed16::from_raw(y.raw < 0 ? -angle : angle);
	}

	static constexpr bool is_real(fixed16) { return true; }

	static constexpr fixed16 min_val(fixed16 a, fixed16 b) {
		return fixed16::from_raw(b.raw + ((a.raw - b.raw) & -(a.raw < b.raw)));
	}

	static constexpr fixed16 max_val(fixed16 a, fixed16 b) {
		return fixed16::from_raw(a.raw - ((a.raw - b.raw) & -(a.raw < b.raw)));
	}

	static constexpr fixed16 clamp(fixed16 low, fixed16 high, fixed16 v) {
		return min_val(high, max_val(low, v));
	}

	static constexpr fixed16 mul(fixed16 a, fixed16 b) { return a * b; }
	static constexpr fixed16 div(fixed16 a, fixed16 b) { return a / b; }
	static constexpr fixed16 madd(fixed16 a, fixed16 b, fixed16 c) { return a * b + c; }

	// Epsilon (fixed uses bit-shift)
	static void make_epsilon(epsilon_state<fixed16>& s, int epsilon_bits) {
		s.epsilon_bits = epsilon_bits;
		s.epsilon = fixed16::from_raw(1 << epsilon_bits);
		s.half_epsilon = fixed16::from_raw(s.epsilon.raw >> 1);
		s.quarter_epsilon = fixed16::from_raw(s.epsilon.raw >> 2);
	}

	static int default_epsilon_bits() { return 4; }
	static fixed16 default_max_position_component() { return fixed16::from_raw(0x7FFF0000); } // ~32767
	static fixed16 default_max_velocity_component() { return fixed16::from_int(104); }
	static fixed16 default_max_force_component() { return fixed16::from_int(104); }
	static fixed16 default_deactivate_speed(const epsilon_state<fixed16>&) { return fixed16::from_raw(1 << 8); }

	// Cap — branchless clamp (no NaN possible for fixed)
	static constexpr fixed16 cap(fixed16 v, fixed16 limit) {
		return min_val(limit, max_val(fixed16::from_raw(-limit.raw), v));
	}

	// Snap to grid (fixed version — bit-shift)
	static void snap_to_grid(fixed16& v, const epsilon_state<fixed16>& s) {
		v.raw = (((v.raw + (s.half_epsilon.raw * -(v.raw < 0))) >> s.epsilon_bits) << s.epsilon_bits);
	}
};

} // namespace hop
