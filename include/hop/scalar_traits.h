#pragma once

#include <cmath>
#include <hop/fixed16.h>
#include <hop/fixed32.h>

namespace hop {

template <typename T> struct is_fixed_scalar : std::false_type {};
template <> struct is_fixed_scalar<fixed16> : std::true_type {};
template <> struct is_fixed_scalar<fixed32> : std::true_type {};
template <typename T> inline constexpr bool is_fixed_scalar_v = is_fixed_scalar<T>::value;

// Epsilon state — differs between floating-point (stores 1/epsilon) and fixed types (stores bit count)
template <typename T> struct epsilon_state {
	T epsilon = 0;
	T half_epsilon = 0;
	T quarter_epsilon = 0;
	T one_over_epsilon = 0;
};

template <> struct epsilon_state<fixed16> {
	int epsilon_bits = 0;
	fixed16 epsilon = {};
	fixed16 half_epsilon = {};
	fixed16 quarter_epsilon = {};
};

template <> struct epsilon_state<fixed32> {
	int epsilon_bits = 0;
	fixed32 epsilon = {};
	fixed32 half_epsilon = {};
	fixed32 quarter_epsilon = {};
};

// scalar_traits<float>
template <typename T> struct scalar_traits;

template <> struct scalar_traits<float> {
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
	static float asin(float v) { return std::asin(v); }
	static float acos(float v) { return std::acos(v); }
	static float atan2(float y, float x) { return std::atan2(y, x); }

	static bool is_real(float v) { return !std::isnan(v) && !std::isinf(v); }

	static float min_val(float a, float b) { return a < b ? a : b; }
	static float max_val(float a, float b) { return a > b ? a : b; }
	static float clamp(float low, float high, float v) { return min_val(high, max_val(low, v)); }

	static float mul(float a, float b) { return a * b; }
	static float div(float a, float b) { return a / b; }
	static float madd(float a, float b, float c) { return a * b + c; }
	static float square(float v) { return v * v; }

	// Epsilon
	static void make_epsilon(epsilon_state<float> & s, float epsilon) {
		s.epsilon = epsilon;
		s.one_over_epsilon = 1.0f / epsilon;
		s.half_epsilon = epsilon * 0.5f;
		s.quarter_epsilon = epsilon * 0.25f;
	}

	static float default_epsilon() { return 0.001f; }
	static float default_max_position_component() { return 100000.0f; }
	static float default_max_velocity_component() { return 1000.0f; }
	static float default_max_force_component() { return 1000.0f; }
	static float default_deactivate_speed(const epsilon_state<float> & s) { return s.epsilon * 2.0f; }

	// Cap — clamp + NaN guard
	static float cap(float v, float limit) {
		v = max_val(-limit, v);
		v = min_val(limit, v);
		return is_real(v) ? v : 0.0f;
	}
};

// scalar_traits<double>
template <> struct scalar_traits<double> {
	using type = double;

	static constexpr double one() { return 1.0; }
	static constexpr double zero() { return 0.0; }
	static constexpr double half() { return 0.5; }
	static constexpr double two() { return 2.0; }
	static constexpr double three() { return 3.0; }
	static constexpr double four() { return 4.0; }
	static constexpr double quarter() { return 0.25; }
	static constexpr double third() { return 1.0 / 3.0; }
	static constexpr double two_thirds() { return 2.0 / 3.0; }
	static constexpr double pi() { return 3.14159265358979323846; }
	static constexpr double two_pi() { return pi() * 2.0; }
	static constexpr double half_pi() { return pi() / 2.0; }

	static constexpr double from_milli(int m) { return static_cast<double>(m) / 1000.0; }
	static constexpr double from_int(int i) { return static_cast<double>(i); }
	static constexpr int to_int(double v) { return static_cast<int>(v); }
	static constexpr float to_float(double v) { return static_cast<float>(v); }

	static double abs(double v) { return std::fabs(v); }
	static double sqrt(double v) { return std::sqrt(v); }
	static double sin(double v) { return std::sin(v); }
	static double cos(double v) { return std::cos(v); }
	static double asin(double v) { return std::asin(v); }
	static double acos(double v) { return std::acos(v); }
	static double atan2(double y, double x) { return std::atan2(y, x); }

	static bool is_real(double v) { return !std::isnan(v) && !std::isinf(v); }

	static double min_val(double a, double b) { return a < b ? a : b; }
	static double max_val(double a, double b) { return a > b ? a : b; }
	static double clamp(double low, double high, double v) { return min_val(high, max_val(low, v)); }

	static double mul(double a, double b) { return a * b; }
	static double div(double a, double b) { return a / b; }
	static double madd(double a, double b, double c) { return a * b + c; }
	static double square(double v) { return v * v; }

	// Epsilon
	static void make_epsilon(epsilon_state<double> & s, double epsilon) {
		s.epsilon = epsilon;
		s.one_over_epsilon = 1.0 / epsilon;
		s.half_epsilon = epsilon * 0.5;
		s.quarter_epsilon = epsilon * 0.25;
	}

	static double default_epsilon() { return 0.001; }
	static double default_max_position_component() { return 100000.0; }
	static double default_max_velocity_component() { return 1000.0; }
	static double default_max_force_component() { return 1000.0; }
	static double default_deactivate_speed(const epsilon_state<double> & s) { return s.epsilon * 2.0; }

	// Cap — clamp + NaN guard
	static double cap(double v, double limit) {
		v = max_val(-limit, v);
		v = min_val(limit, v);
		return is_real(v) ? v : 0.0;
	}
};

// scalar_traits<fixed16>
template <> struct scalar_traits<fixed16> {
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
	static constexpr fixed16 abs(fixed16 v) { return fixed16::from_raw((v.raw ^ (v.raw >> 31)) - (v.raw >> 31)); }

	// Newton-Raphson sqrt with bit-scan initial guess.
	//
	// For Q16.16 raw r (representing r / 65536), sqrt value = sqrt(r) / 256,
	// and the raw of that is sqrt(r) * 256. When r ≈ 2^n (n = highest set bit),
	// sqrt(r) ≈ 2^(n/2), so sqrt_raw ≈ 2^(n/2 + 8). That initial guess is
	// within a factor of √2 of the true answer, so Newton converges to
	// <1e-6 relative error in 4 iterations — versus the previous 8 iterations
	// from the flat (v + 1.0) / 2 guess.
	static fixed16 sqrt(fixed16 v) {
		if (v.raw <= 0)
			return zero();
		int32_t r = v.raw;
		int n = 0;
		if (r & 0xFFFF0000) { n |= 16; r >>= 16; }
		if (r & 0x0000FF00) { n |=  8; r >>=  8; }
		if (r & 0x000000F0) { n |=  4; r >>=  4; }
		if (r & 0x0000000C) { n |=  2; r >>=  2; }
		if (r & 0x00000002) { n |=  1; }
		int32_t s = 1 << ((n >> 1) + 8);
		for (int i = 0; i < 3; ++i) {
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

		if (f.raw < 0)
			f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw)
			f.raw = f.raw % two_pi_raw;

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

		if (f.raw < 0)
			f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw)
			f.raw = f.raw % two_pi_raw;

		int sign = 1;
		if (f.raw > half_pi_raw && f.raw <= pi_raw) {
			f.raw = pi_raw - f.raw;
			sign = -1;
		} else if (f.raw > pi_raw && f.raw <= pi_raw + half_pi_raw) {
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

	// Polynomial asin (from toadlet mathfixed)
	static fixed16 asin(fixed16 f) {
		constexpr int32_t one_raw = 65536;
		constexpr int32_t half_pi_raw = 102943;
		int32_t f_root = sqrt(fixed16::from_raw(one_raw - f.raw)).raw;
		int32_t result = -1228;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result += 4866;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result -= 13901;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result += 102939;
		result = half_pi_raw - static_cast<int32_t>((static_cast<int64_t>(f_root) * result) >> 16);
		return fixed16::from_raw(result);
	}

	// Polynomial acos (from toadlet mathfixed)
	static fixed16 acos(fixed16 f) {
		constexpr int32_t one_raw = 65536;
		int32_t f_root = sqrt(fixed16::from_raw(one_raw - f.raw)).raw;
		int32_t result = -1228;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result += 4866;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result -= 13901;
		result = static_cast<int32_t>((static_cast<int64_t>(result) * f.raw) >> 16);
		result += 102939;
		result = static_cast<int32_t>((static_cast<int64_t>(f_root) * result) >> 16);
		return fixed16::from_raw(result);
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

	static constexpr fixed16 clamp(fixed16 low, fixed16 high, fixed16 v) { return min_val(high, max_val(low, v)); }

	static constexpr fixed16 mul(fixed16 a, fixed16 b) { return a * b; }
	static constexpr fixed16 div(fixed16 a, fixed16 b) { return a / b; }
	static constexpr fixed16 madd(fixed16 a, fixed16 b, fixed16 c) { return a * b + c; }
	static constexpr fixed16 square(fixed16 v) { return v * v; }

	// Epsilon (fixed uses bit-shift)
	static void make_epsilon(epsilon_state<fixed16> & s, int epsilon_bits) {
		s.epsilon_bits = epsilon_bits;
		s.epsilon = fixed16::from_raw(1 << epsilon_bits);
		s.half_epsilon = fixed16::from_raw(s.epsilon.raw >> 1);
		s.quarter_epsilon = fixed16::from_raw(s.epsilon.raw >> 2);
	}

	static int default_epsilon_bits() { return 4; }
	static fixed16 default_max_position_component() { return fixed16::from_raw(0x7FFF0000); } // ~32767
	static fixed16 default_max_velocity_component() { return fixed16::from_int(104); }
	static fixed16 default_max_force_component() { return fixed16::from_int(104); }
	static fixed16 default_deactivate_speed(const epsilon_state<fixed16> &) { return fixed16::from_raw(1 << 8); }

	// Cap — branchless clamp (no NaN possible for fixed)
	static constexpr fixed16 cap(fixed16 v, fixed16 limit) {
		return min_val(limit, max_val(fixed16::from_raw(-limit.raw), v));
	}
};

// scalar_traits<fixed32>
template <> struct scalar_traits<fixed32> {
	using type = fixed32;

	static constexpr fixed32 one() { return fixed32::from_raw(4294967296LL); }
	static constexpr fixed32 zero() { return fixed32::from_raw(0); }
	static constexpr fixed32 half() { return fixed32::from_raw(2147483648LL); }
	static constexpr fixed32 two() { return fixed32::from_raw(8589934592LL); }
	static constexpr fixed32 three() { return fixed32::from_raw(12884901888LL); }
	static constexpr fixed32 four() { return fixed32::from_raw(17179869184LL); }
	static constexpr fixed32 quarter() { return fixed32::from_raw(1073741824LL); }
	static constexpr fixed32 third() { return fixed32::from_raw(1431655765LL); }
	static constexpr fixed32 two_thirds() { return fixed32::from_raw(2863311530LL); }
	static constexpr fixed32 pi() { return fixed32::from_raw(13493037705LL); }
	static constexpr fixed32 two_pi() { return fixed32::from_raw(26986075410LL); }
	static constexpr fixed32 half_pi() { return fixed32::from_raw(6746518852LL); }

	static constexpr fixed32 from_milli(int m) { return fixed32::from_milli(m); }
	static constexpr fixed32 from_int(int i) { return fixed32::from_int(i); }
	static constexpr int to_int(fixed32 v) { return v.to_int(); }
	static constexpr float to_float(fixed32 v) { return v.to_float(); }

	// Bit-hack abs
	static constexpr fixed32 abs(fixed32 v) { return fixed32::from_raw((v.raw ^ (v.raw >> 63)) - (v.raw >> 63)); }

	// Newton-Raphson sqrt with bit-scan initial guess.
	//
	// For Q32.32 raw r (representing r / 2^32), sqrt value = sqrt(r) / 2^16,
	// and the raw of that is sqrt(r) * 2^16. When r ≈ 2^n (n = highest set bit),
	// sqrt(r) ≈ 2^(n/2), so sqrt_raw ≈ 2^(n/2 + 16). 4 iterations of Newton
	// converge to <1e-9 relative error from the initial within-√2 guess.
	static fixed32 sqrt(fixed32 v) {
		if (v.raw <= 0)
			return zero();
		int64_t r = v.raw;
		int n = 0;
		if (r & 0xFFFFFFFF00000000LL) { n |= 32; r >>= 32; }
		if (r & 0x00000000FFFF0000LL) { n |= 16; r >>= 16; }
		if (r & 0x000000000000FF00LL) { n |=  8; r >>=  8; }
		if (r & 0x00000000000000F0LL) { n |=  4; r >>=  4; }
		if (r & 0x000000000000000CLL) { n |=  2; r >>=  2; }
		if (r & 0x0000000000000002LL) { n |=  1; }
		int64_t s = 1LL << ((n >> 1) + 16);
		for (int i = 0; i < 4; ++i) {
			s = (s + detail::shl32_div(v.raw, s)) >> 1;
		}
		return fixed32::from_raw(s);
	}

	// Polynomial sin — same minimax approximation as fixed16, rescaled to Q32.32.
	static fixed32 sin(fixed32 f) {
		constexpr int64_t two_pi_raw = 26986075410LL;
		constexpr int64_t pi_raw = 13493037705LL;
		constexpr int64_t half_pi_raw = 6746518852LL;
		constexpr int64_t one_raw = 4294967296LL;

		if (f.raw < 0)
			f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw)
			f.raw = f.raw % two_pi_raw;

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

		int64_t sqr = detail::mul_shr32(f.raw, f.raw);
		int64_t result = 498LL << 16;
		result = detail::mul_shr32(result, sqr);
		result -= 10882LL << 16;
		result = detail::mul_shr32(result, sqr);
		result += one_raw;
		result = detail::mul_shr32(result, f.raw);
		return fixed32::from_raw(sign * result);
	}

	// Polynomial cos — same minimax approximation as fixed16, rescaled to Q32.32.
	static fixed32 cos(fixed32 f) {
		constexpr int64_t two_pi_raw = 26986075410LL;
		constexpr int64_t pi_raw = 13493037705LL;
		constexpr int64_t half_pi_raw = 6746518852LL;
		constexpr int64_t one_raw = 4294967296LL;

		if (f.raw < 0)
			f.raw = ((f.raw % two_pi_raw) + two_pi_raw);
		else if (f.raw >= two_pi_raw)
			f.raw = f.raw % two_pi_raw;

		int sign = 1;
		if (f.raw > half_pi_raw && f.raw <= pi_raw) {
			f.raw = pi_raw - f.raw;
			sign = -1;
		} else if (f.raw > pi_raw && f.raw <= pi_raw + half_pi_raw) {
			f.raw = f.raw - pi_raw;
			sign = -1;
		} else if (f.raw > pi_raw + half_pi_raw) {
			f.raw = two_pi_raw - f.raw;
		}

		int64_t sqr = detail::mul_shr32(f.raw, f.raw);
		int64_t result = 2328LL << 16;
		result = detail::mul_shr32(result, sqr);
		result -= 32551LL << 16;
		result = detail::mul_shr32(result, sqr);
		result += one_raw;
		return fixed32::from_raw(result * sign);
	}

	// Polynomial asin — same approximation as fixed16, rescaled to Q32.32.
	static fixed32 asin(fixed32 f) {
		constexpr int64_t one_raw = 4294967296LL;
		constexpr int64_t half_pi_raw = 6746518852LL;
		int64_t f_root = sqrt(fixed32::from_raw(one_raw - f.raw)).raw;
		int64_t result = -(1228LL << 16);
		result = detail::mul_shr32(result, f.raw);
		result += 4866LL << 16;
		result = detail::mul_shr32(result, f.raw);
		result -= 13901LL << 16;
		result = detail::mul_shr32(result, f.raw);
		result += 102939LL << 16;
		result = half_pi_raw - detail::mul_shr32(f_root, result);
		return fixed32::from_raw(result);
	}

	// Polynomial acos — same approximation as fixed16, rescaled to Q32.32.
	static fixed32 acos(fixed32 f) {
		constexpr int64_t one_raw = 4294967296LL;
		int64_t f_root = sqrt(fixed32::from_raw(one_raw - f.raw)).raw;
		int64_t result = -(1228LL << 16);
		result = detail::mul_shr32(result, f.raw);
		result += 4866LL << 16;
		result = detail::mul_shr32(result, f.raw);
		result -= 13901LL << 16;
		result = detail::mul_shr32(result, f.raw);
		result += 102939LL << 16;
		result = detail::mul_shr32(f_root, result);
		return fixed32::from_raw(result);
	}

	// Polynomial atan2 — same approximation as fixed16, rescaled to Q32.32.
	static fixed32 atan2(fixed32 y, fixed32 x) {
		constexpr int64_t quarter_pi_raw = 3373259426LL;
		constexpr int64_t three_quarter_pi_raw = 10119778279LL;

		int64_t absy = ((y.raw ^ (y.raw >> 63)) - (y.raw >> 63)) + 1;
		int64_t angle;
		if (x.raw >= 0) {
			int64_t r = detail::shl32_div(x.raw - absy, x.raw + absy);
			angle = quarter_pi_raw - detail::mul_shr32(quarter_pi_raw, r);
		} else {
			int64_t r = detail::shl32_div(x.raw + absy, y.raw - absy);
			angle = three_quarter_pi_raw - detail::mul_shr32(quarter_pi_raw, r);
		}
		return fixed32::from_raw(y.raw < 0 ? -angle : angle);
	}

	static constexpr bool is_real(fixed32) { return true; }

	static constexpr fixed32 min_val(fixed32 a, fixed32 b) {
		return fixed32::from_raw(b.raw + ((a.raw - b.raw) & -(int64_t)(a.raw < b.raw)));
	}

	static constexpr fixed32 max_val(fixed32 a, fixed32 b) {
		return fixed32::from_raw(a.raw - ((a.raw - b.raw) & -(int64_t)(a.raw < b.raw)));
	}

	static constexpr fixed32 clamp(fixed32 low, fixed32 high, fixed32 v) { return min_val(high, max_val(low, v)); }

	static constexpr fixed32 mul(fixed32 a, fixed32 b) { return a * b; }
	static constexpr fixed32 div(fixed32 a, fixed32 b) { return a / b; }
	static constexpr fixed32 madd(fixed32 a, fixed32 b, fixed32 c) { return a * b + c; }
	static constexpr fixed32 square(fixed32 v) { return v * v; }

	// Epsilon (fixed uses bit-shift; default gives same float epsilon as fixed16's default of 4 bits)
	static void make_epsilon(epsilon_state<fixed32> & s, int epsilon_bits) {
		s.epsilon_bits = epsilon_bits;
		s.epsilon = fixed32::from_raw(1LL << epsilon_bits);
		s.half_epsilon = fixed32::from_raw(s.epsilon.raw >> 1);
		s.quarter_epsilon = fixed32::from_raw(s.epsilon.raw >> 2);
	}

	static int default_epsilon_bits() { return 20; }
	static fixed32 default_max_position_component() { return fixed32::from_int(100000); }
	static fixed32 default_max_velocity_component() { return fixed32::from_int(1000); }
	static fixed32 default_max_force_component() { return fixed32::from_int(1000); }
	// 2^-8 ≈ 0.0039, matching fixed16's default_deactivate_speed of from_raw(1 << 8) / 2^16
	static fixed32 default_deactivate_speed(const epsilon_state<fixed32> &) { return fixed32::from_raw(1LL << 24); }

	// Cap — branchless clamp (no NaN possible for fixed)
	static constexpr fixed32 cap(fixed32 v, fixed32 limit) {
		return min_val(limit, max_val(fixed32::from_raw(-limit.raw), v));
	}
};

} // namespace hop
