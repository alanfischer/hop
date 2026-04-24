#pragma once

// Minimal benchmark harness. Hand-rolled on purpose — hop has no third-party
// deps and a bench harness shouldn't be the exception. If we ever want
// statistical rigor (median-of-N, noise gating), swap this for Nanobench
// (single-header, BSD) at that point.
//
// Usage:
//   bench::go("label", 10000, [&] { sim->update(10); });

#include <chrono>
#include <cstdio>

namespace hop::bench {

struct result {
	const char * name;
	double ns_per_op;
	long long total_ns;
	long long iterations;
};

template <typename F> inline result run(const char * name, long long iterations, F && body) {
	// Warm-up: ~10% of the run, minimum one iteration.
	long long warmup = iterations / 10;
	if (warmup < 1) warmup = 1;
	for (long long i = 0; i < warmup; ++i) body();

	auto t0 = std::chrono::steady_clock::now();
	for (long long i = 0; i < iterations; ++i) body();
	auto t1 = std::chrono::steady_clock::now();

	long long elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
	return { name, double(elapsed) / iterations, elapsed, iterations };
}

inline void print(const result & r) {
	printf("  %-50s %10.1f ns/op   (%lld iters, %7.2f ms total)\n",
	       r.name, r.ns_per_op, r.iterations, r.total_ns / 1e6);
}

template <typename F> inline void go(const char * name, long long iterations, F && body) {
	print(run(name, iterations, static_cast<F &&>(body)));
}

} // namespace hop::bench
