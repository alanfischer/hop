// profile_stress.cpp — Headless mirror of demo_stress.cpp for profiling.
// Builds the same closed-room scene with N spheres and runs a fixed number
// of 16ms ticks, printing total wall time and ms/tick. Designed to be run
// under `sample` / Instruments — no rendering, no input.
//
// Args: [N=10000] [steps=600]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <hop/bvh_manager.h>
#include <hop/hop.h>
#include <memory>
#include <vector>

template <typename T>
static std::shared_ptr<hop::solid<T>> make_wall(hop::simulator<T> & sim,
                                                hop::bvh_manager<T> & bvh,
                                                hop::aa_box<T> box,
                                                hop::vec3<T> pos) {
	using tr = hop::scalar_traits<T>;
	auto w = std::make_shared<hop::solid<T>>();
	w->set_infinite_mass();
	w->set_coefficient_of_gravity(T{});
	w->set_coefficient_of_restitution(tr::one());
	w->add_shape(std::make_shared<hop::shape<T>>(box));
	w->set_position(pos);
	sim.add_solid(w);
	bvh.add_solid(w.get(), true);
	return w;
}

template <typename T> static void run(int n_spheres, int steps) {
	using tr = hop::scalar_traits<T>;

	constexpr int   ROOM_HALF   = 6;
	constexpr int   ROOM_HEIGHT = 12;
	constexpr float SPHERE_R    = 0.28f;
	constexpr float SPACING     = SPHERE_R * 2.2f;
	constexpr int   COLS        = (int)((ROOM_HALF * 2.0f) / SPACING);
	constexpr int   Z_LAYERS    = (int)((ROOM_HEIGHT - SPHERE_R * 2.0f) / SPACING);

	auto fi = [](int n) { return tr::from_int(n); };
	auto ff = [](float f) -> T { return tr::from_milli((int)(f * 1000.0f)); };

	hop::simulator<T> sim;
	hop::bvh_manager<T> bvh;
	sim.set_manager(&bvh);

	T half  = fi(ROOM_HALF);
	T hgt   = fi(ROOM_HEIGHT);
	T thick = fi(1);
	T zero  = T{};
	T outer = half + thick;

	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-outer, -outer, -thick), hop::vec3<T>(outer, outer, zero)),
	    hop::vec3<T>());
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-outer, -outer, zero), hop::vec3<T>(outer, outer, thick)),
	    hop::vec3<T>(zero, zero, hgt));
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-thick, -half, zero), hop::vec3<T>(zero, half, hgt)),
	    hop::vec3<T>(-half, zero, zero));
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(zero, -half, zero), hop::vec3<T>(thick, half, hgt)),
	    hop::vec3<T>(half, zero, zero));
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-half, -thick, zero), hop::vec3<T>(half, zero, hgt)),
	    hop::vec3<T>(zero, -half, zero));
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-half, zero, zero), hop::vec3<T>(half, thick, hgt)),
	    hop::vec3<T>(zero, half, zero));
	bvh.rebuild();

	float cor  = 0.75f;
	float fric = 0.0f;

	std::vector<std::shared_ptr<hop::solid<T>>> spheres;
	spheres.reserve(n_spheres);
	for (int i = 0; i < n_spheres; ++i) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_coefficient_of_restitution_override(true);
		s->set_coefficient_of_restitution(ff(cor));
		s->set_coefficient_of_static_friction(ff(fric));
		s->set_coefficient_of_dynamic_friction(ff(fric));
		s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));
		spheres.push_back(std::move(s));
	}

	for (int i = 0; i < n_spheres; ++i) {
		int   layer = i / (COLS * COLS);
		int   rem   = i % (COLS * COLS);
		int   col   = rem % COLS;
		int   row   = rem / COLS;
		float x     = -(ROOM_HALF - SPACING * 0.5f) + col * SPACING;
		float y     = -(ROOM_HALF - SPACING * 0.5f) + row * SPACING;
		float z     = ROOM_HEIGHT - SPHERE_R - (layer % Z_LAYERS) * SPACING - SPACING * 0.5f;
		float vx    = ((i * 7  + 3) % 21 - 10) * 0.5f;
		float vy    = ((i * 13 + 5) % 21 - 10) * 0.5f;
		float vz    = ((i * 17 + 9) % 21 - 10) * 0.5f;
		spheres[i]->set_position(hop::vec3<T>(ff(x), ff(y), ff(z)));
		spheres[i]->set_velocity(hop::vec3<T>(ff(vx), ff(vy), ff(vz)));
		spheres[i]->activate();
		sim.add_solid(spheres[i]);
		bvh.add_solid(spheres[i].get(), false);
	}

	std::printf("scene: N=%d, steps=%d\n", n_spheres, steps);

	int bucket = 100;
	int active_count = n_spheres;
	int max_partners_ever = 0;
	int partner_hist[20] = {0};  // count of (solid, tick) pairs with N partners; idx 17+ = "17 or more"
	auto t0 = std::chrono::steady_clock::now();
	auto bucket_start = t0;
	for (int s = 0; s < steps; ++s) {
		sim.update(16);
		// Sample max partner count across all spheres this tick.
		for (int i = 0; i < n_spheres; ++i) {
			int pc = spheres[i]->get_impulse_partner_count();
			if (pc > max_partners_ever) max_partners_ever = pc;
			int idx = pc < 19 ? pc : 19;
			partner_hist[idx]++;
		}
		if ((s + 1) % bucket == 0) {
			auto now = std::chrono::steady_clock::now();
			double bucket_ms = std::chrono::duration<double, std::milli>(now - bucket_start).count();
			int sleeping = 0;
			double ke_sum = 0.0;
			float vmax = 0.0f;
			for (int i = 0; i < n_spheres; ++i) {
				if (!spheres[i]->active()) ++sleeping;
				float v = tr::to_float(hop::length(spheres[i]->get_velocity()));
				ke_sum += double(v) * double(v);  // mass=1 so KE ∝ v²
				if (v > vmax) vmax = v;
			}
			float v_avg = std::sqrt(float(ke_sum / n_spheres));
			std::printf("  ticks %4d-%-4d  per-tick: %.2f ms  sleep: %d  v_rms: %.2f  v_max: %.2f\n",
			            s + 1 - bucket + 1, s + 1, bucket_ms / bucket, sleeping, v_avg, vmax);
			bucket_start = now;
		}
	}
	auto t1 = std::chrono::steady_clock::now();
	double total_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
	std::printf("total: %.1f ms   per-tick: %.2f ms\n", total_ms, total_ms / steps);
	std::printf("max impulse partners observed (any solid, any tick): %d\n", max_partners_ever);
	std::printf("partner-count histogram (count of solid-ticks with N partners):\n");
	long long total_solid_ticks = (long long)n_spheres * steps;
	for (int i = 0; i < 20; ++i) {
		if (partner_hist[i] == 0) continue;
		std::printf("  %s%2d : %10d  (%.3f%%)\n",
		            i == 19 ? ">=" : "  ", i,
		            partner_hist[i], 100.0 * partner_hist[i] / double(total_solid_ticks));
	}
}

int main(int argc, char ** argv) {
	int n     = argc > 1 ? std::atoi(argv[1]) : 10000;
	int steps = argc > 2 ? std::atoi(argv[2]) : 600;
	run<float>(n, steps);
	return 0;
}
