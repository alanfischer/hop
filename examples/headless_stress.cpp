// headless_stress.cpp — headless diagnostics for the demo_stress settling scene.
// Mirrors demo_stress.cpp's setup (dissipative COR-0.5 walls, contact damping
// 0.8) but runs without a window and prints stats.
//
// Usage: headless_stress [ticks] [room_half] [room_height] [iters] [COR]
//                        [friction] [avg_normals] [deactivate_speed]
//                        [deactivate_count]
//
// Per sample (every 300 ticks, plus tick 1 and any KE spike) prints asleep
// count, total KE, max speed, and mean ball height. A final DONE line reports
// wall-clock time, asleep/total, final KE, mean height, and how many ticks had
// a floor breach. Used to sweep the sleep (deactivation) thresholds against
// settling cost — see the "Contact Solver, Settling & Sleep" README section.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <hop/bvh_manager.h>
#include <hop/hop.h>
#include <vector>

using T = float;
using tr = hop::scalar_traits<T>;
static float WALL_FRICTION = 0.0f;

template <typename T2>
static std::shared_ptr<hop::solid<T2>> make_wall(hop::simulator<T2> & sim,
                                                 hop::bvh_manager<T2> & bvh,
                                                 hop::aa_box<T2> box,
                                                 hop::vec3<T2> pos) {
	auto w = std::make_shared<hop::solid<T2>>();
	w->set_infinite_mass();
	w->set_coefficient_of_gravity(T2{});
	w->set_coefficient_of_restitution(hop::scalar_traits<T2>::from_milli(500));  // COR 0.5, match demo_stress
	w->set_coefficient_of_static_friction(WALL_FRICTION);
	w->set_coefficient_of_dynamic_friction(WALL_FRICTION);
	w->add_shape(std::make_shared<hop::shape<T2>>(box));
	w->set_position(pos);
	sim.add_solid(w);
	bvh.add_solid(w.get(), true);
	return w;
}

int main(int argc, char ** argv) {
	setvbuf(stdout, nullptr, _IOLBF, 0);  // line-buffer so streamed/redirected output is live
	int TICKS = argc > 1 ? atoi(argv[1]) : 600;
	int ROOM_HALF = argc > 2 ? atoi(argv[2]) : 6;
	int ROOM_HEIGHT = argc > 3 ? atoi(argv[3]) : 12;
	int ITERS = argc > 4 ? atoi(argv[4]) : 16;
	float COR = argc > 5 ? atof(argv[5]) : 0.75f;
	float FR  = argc > 6 ? atof(argv[6]) : 0.0f;
	int   AN  = argc > 7 ? atoi(argv[7]) : 1;
	float DS  = argc > 8 ? atof(argv[8]) : 0.2f;
	int   DC  = argc > 9 ? atoi(argv[9]) : 32;
	WALL_FRICTION = FR;

	const float SPHERE_R = 0.28f;
	const float SPACING = SPHERE_R * 2.2f;
	const int COLS = (int)((ROOM_HALF * 2.0f) / SPACING);
	const int Z_LAYERS = (int)(ROOM_HEIGHT / SPACING);
	const int COUNT = COLS * COLS * Z_LAYERS;

	auto fi = [](int n) { return tr::from_int(n); };
	auto ff = [](float f) -> T { return tr::from_milli((int)(f * 1000.0f)); };

	hop::simulator<T> sim;
	hop::bvh_manager<T> bvh;
	sim.set_manager(&bvh);
	sim.set_deactivate_count(DC);
	sim.set_deactivate_speed(tr::from_milli((int)(DS*1000)));
	sim.set_solver_iterations(ITERS);
	sim.set_average_normals(AN != 0);
	sim.set_contact_damping(ff(0.8f));  // match demo_stress

	T half = fi(ROOM_HALF), hgt = fi(ROOM_HEIGHT), thick = fi(1), zero = T{};
	T outer = half + thick;

	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(-outer, -outer, -thick), hop::vec3<T>(outer, outer, zero)), hop::vec3<T>());
	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(-outer, -outer, zero), hop::vec3<T>(outer, outer, thick)), hop::vec3<T>(zero, zero, hgt));
	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(-thick, -half, zero), hop::vec3<T>(zero, half, hgt)), hop::vec3<T>(-half, zero, zero));
	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(zero, -half, zero), hop::vec3<T>(thick, half, hgt)), hop::vec3<T>(half, zero, zero));
	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(-half, -thick, zero), hop::vec3<T>(half, zero, hgt)), hop::vec3<T>(zero, -half, zero));
	make_wall(sim, bvh, hop::aa_box<T>(hop::vec3<T>(-half, zero, zero), hop::vec3<T>(half, thick, hgt)), hop::vec3<T>(zero, half, zero));
	bvh.rebuild();

	std::vector<std::shared_ptr<hop::solid<T>>> spheres;
	spheres.reserve(COUNT);
	for (int i = 0; i < COUNT; ++i) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_restitution_combine(hop::restitution_combine::maximum);  // match demo_stress / rp3d
		s->set_coefficient_of_restitution(ff(COR));
		s->set_coefficient_of_static_friction(ff(FR));
		s->set_coefficient_of_dynamic_friction(ff(FR));
		s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));
		int layer = i / (COLS * COLS), rem = i % (COLS * COLS), col = rem % COLS, row = rem / COLS;
		float x = (col - (COLS - 1) * 0.5f) * SPACING;
		float y = (row - (COLS - 1) * 0.5f) * SPACING;
		float z = SPACING * 0.5f + layer * SPACING;
		float vx = ((i * 7 + 3) % 21 - 10) * 0.5f;
		float vy = ((i * 13 + 5) % 21 - 10) * 0.5f;
		float vz = ((i * 17 + 9) % 21 - 10) * 0.5f;
		s->set_position(hop::vec3<T>(ff(x), ff(y), ff(z)));
		s->set_velocity(hop::vec3<T>(ff(vx), ff(vy), ff(vz)));
		s->activate();
		sim.add_solid(s);
		bvh.add_solid(s.get(), false);
		spheres.push_back(std::move(s));
	}

	printf("scene: COUNT=%d  COLS=%d  Z_LAYERS=%d  room=%dx%dx%d  iters=%d COR=%.2f\n",
	       COUNT, COLS, Z_LAYERS, ROOM_HALF*2, ROOM_HALF*2, ROOM_HEIGHT, ITERS, COR);
	printf("%7s %7s %7s %12s %8s %8s %8s %8s %6s\n",
	       "tick", "active", "asleep", "KE", "maxV", "minZ", "maxZ", "E", "inj/dis");

	printf("%7s %7s %12s %8s %8s %6s\n", "tick", "asleep", "KE", "maxV", "meanZ", "");
	double prevKE = 0, prevE = 0, injPos = 0, injNeg = 0;
	int finalAsleep = 0, totalBreach = 0; float finalKE = 0, finalMeanZ = 0;
	auto t0 = std::chrono::high_resolution_clock::now();
	for (int t = 1; t <= TICKS; ++t) {
		sim.update(tr::from_milli(16));

		// Cheap O(n) stats every tick so a transient is never missed between samples.
		float ke = 0, maxv = 0, maxz = -1e9f, minz = 1e9f, sumz = 0, sumx = 0, sumy = 0;
		double pe = 0;
		int below = 0, asleep = 0;
		for (auto & s : spheres) {
			auto p = s->get_position();
			auto v = s->get_velocity();
			float sp2 = v.x*v.x + v.y*v.y + v.z*v.z;
			ke += 0.5f * sp2;
			pe += 9.81 * tr::to_float(p.z);
			maxv = std::max(maxv, std::sqrt(sp2));
			maxz = std::max(maxz, p.z);
			minz = std::min(minz, p.z);
			sumz += p.z; sumx += tr::to_float(p.x); sumy += tr::to_float(p.y);
			if (p.z < SPHERE_R - 0.05f) ++below;   // a sphere center this low has breached the floor
			if (!s->active()) ++asleep;
		}
		float comx = sumx/spheres.size(), comy = sumy/spheres.size();
		double E = ke + pe;
		if (t > 1) { double d = E - prevE; if (d > 0) injPos += d; else injNeg += d; }
		prevE = E;
		if (below > 0) ++totalBreach;

		bool spike  = t > 120 && ke > prevKE * 1.5f && ke - prevKE > 100.0f;  // the "hotspot"
		if (t % 300 == 0 || t == 1 || spike) {
			printf("%7d  %6d  KE=%9.1f maxV=%6.2f meanZ=%.3f %s\n",
			       t, asleep, ke, maxv, sumz/spheres.size(),
			       spike ? "  <-- KE SPIKE" : "");
		}
		prevKE = ke;
		finalAsleep = asleep; finalKE = ke; finalMeanZ = sumz/spheres.size();
	}
	auto t1 = std::chrono::high_resolution_clock::now();
	double secs = std::chrono::duration<double>(t1 - t0).count();
	printf("DONE  time=%.2fs  asleep=%d/%d  finalKE=%.2f  meanZ=%.3f  breachTicks=%d\n",
	       secs, finalAsleep, COUNT, finalKE, finalMeanZ, totalBreach);
	return 0;
}
