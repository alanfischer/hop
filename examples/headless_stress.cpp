// headless_stress.cpp — headless diagnostics for the demo_stress settling scene.
// Mirrors demo_stress.cpp's setup but runs without a window and prints stats.
//
// Usage: headless_stress [ticks] [room_half] [room_height] [iters] [COR]
//                        [friction] [avg_normals] [deactivate_speed]
//
// Reports per-second: active/asleep counts, total KE, max speed, max/mean ball
// height, max inter-sphere penetration, "hanging" count (balls at rest with
// empty space directly below), total mechanical energy E=KE+PE, and the running
// injected(+)/dissipated(-) energy split. E must never trend up for this scene.

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
	w->set_coefficient_of_restitution(hop::scalar_traits<T2>::one());
	w->set_coefficient_of_static_friction(WALL_FRICTION);
	w->set_coefficient_of_dynamic_friction(WALL_FRICTION);
	w->add_shape(std::make_shared<hop::shape<T2>>(box));
	w->set_position(pos);
	sim.add_solid(w);
	bvh.add_solid(w.get(), true);
	return w;
}

int main(int argc, char ** argv) {
	int TICKS = argc > 1 ? atoi(argv[1]) : 600;
	int ROOM_HALF = argc > 2 ? atoi(argv[2]) : 6;
	int ROOM_HEIGHT = argc > 3 ? atoi(argv[3]) : 12;
	int ITERS = argc > 4 ? atoi(argv[4]) : 16;
	float COR = argc > 5 ? atof(argv[5]) : 0.75f;
	float FR  = argc > 6 ? atof(argv[6]) : 0.0f;
	int   AN  = argc > 7 ? atoi(argv[7]) : 1;
	float DS  = argc > 8 ? atof(argv[8]) : 0.2f;
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
	sim.set_deactivate_count(32);
	sim.set_deactivate_speed(tr::from_milli((int)(DS*1000)));
	sim.set_solver_iterations(ITERS);
	sim.set_average_normals(AN != 0);

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
		s->set_coefficient_of_restitution_override(true);
		s->set_coefficient_of_restitution(ff(COR));
		s->set_coefficient_of_static_friction(ff(FR));
		s->set_coefficient_of_dynamic_friction(ff(FR));
		s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));
		int layer = i / (COLS * COLS), rem = i % (COLS * COLS), col = rem % COLS, row = rem / COLS;
		float x = -(ROOM_HALF - SPACING * 0.5f) + col * SPACING;
		float y = -(ROOM_HALF - SPACING * 0.5f) + row * SPACING;
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
	printf("%5s %7s %7s %10s %8s %8s %8s %8s %6s\n",
	       "tick", "active", "asleep", "KE", "maxV", "maxZ", "meanZ", "maxPen", "hang");

	const float diam = SPHERE_R * 2.0f;
	double prevE = 0; double injPos = 0, injNeg = 0;
	for (int t = 1; t <= TICKS; ++t) {
		sim.update(tr::from_milli(16));
		// per-tick total mechanical energy to track injection
		double E = 0;
		for (auto & s : spheres) { auto v=s->get_velocity(); auto p=s->get_position();
			E += 0.5*(v.x*v.x+v.y*v.y+v.z*v.z) + 9.81*p.z; }
		if (t > 1) { double d = E - prevE; if (d>0) injPos += d; else injNeg += d; }
		prevE = E;
		if (t % 60 == 0 || t == 1) {
			float ke = 0, maxz = 0, sumz = 0, maxv = 0, maxpen = 0;
			int hang = 0;
			for (auto & s : spheres) {
				auto p = s->get_position();
				auto v = s->get_velocity();
				float sp2 = v.x*v.x + v.y*v.y + v.z*v.z;
				ke += 0.5f * sp2;
				maxv = std::max(maxv, std::sqrt(sp2));
				maxz = std::max(maxz, p.z);
				sumz += p.z;
				// nearest-neighbour overlap + support check
				bool supported = p.z <= SPHERE_R + 0.08f;
				for (auto & o : spheres) {
					if (o.get() == s.get()) continue;
					auto q = o->get_position();
					float dx = q.x-p.x, dy = q.y-p.y, dz = q.z-p.z;
					float d2 = dx*dx+dy*dy+dz*dz;
					if (d2 < diam*diam) {
						float pen = diam - std::sqrt(d2);
						maxpen = std::max(maxpen, pen);
					}
					if (q.z < p.z && d2 <= (diam+0.05f)*(diam+0.05f)) supported = true;
				}
				if (sp2 <= 0.04f && !supported) hang++;
			}
			int active = sim.count_active_solids();
			printf("%5d %7d %7d %10.2f %8.3f %8.3f %8.3f %8.4f %6d  E=%.0f  +inj=%.0f -dis=%.0f\n",
			       t, active, COUNT - active, ke, maxv, maxz, sumz/spheres.size(), maxpen, hang,
			       prevE, injPos, injNeg);
		}
	}
	return 0;
}
