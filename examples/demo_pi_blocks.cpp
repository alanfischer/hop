// Galperin's billiard: a big block slides into a small one resting against a wall.
// With a mass ratio of 100^(N-1), the collisions number the first N digits of pi.
//
// Headless. Each ratio runs at 60 Hz in both contact modes, then with the micro-collision
// threshold and speed cap lifted, at a tick rate where the small block never moves more
// than a fraction of its width per tick.

#include <cmath>
#include <cstdio>
#include <hop/hop.h>
#include <string>

using T = double;

static std::shared_ptr<hop::solid<T>> block(hop::simulator<T> & sim, T mass, T x, T half) {
	auto s = std::make_shared<hop::solid<T>>();
	if (mass > 0)
		s->set_mass(mass);
	else
		s->set_infinite_mass();
	s->set_coefficient_of_gravity(0);
	s->set_coefficient_of_restitution(1);
	s->set_coefficient_of_static_friction(0);
	s->set_coefficient_of_dynamic_friction(0);
	s->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>({ -half, -half, -half }, { half, half, half })));
	s->set_position({ x, 0, 0 });
	sim.add_solid(s);
	return s;
}

static int exact_count(T ratio) {
	T v = 0, V = -1;
	int n = 0;
	for (;;) {
		if (V < v) {
			T nv = ((1 - ratio) * v + 2 * ratio * V) / (1 + ratio);
			V = ((ratio - 1) * V + 2 * v) / (1 + ratio);
			v = nv;
		} else if (v < 0)
			v = -v;
		else
			return n;
		++n;
	}
}

static const T v0 = 5;

static std::string run(T ratio, T hz, hop::contact_mode mode, bool tuned) {
	hop::simulator<T> sim;
	sim.set_gravity({ 0, 0, 0 });
	sim.set_default_contact_mode(mode);
	if (tuned) {
		sim.set_micro_collision_threshold(0);
		sim.set_max_velocity_component(1e6); // the small block peaks at v0 * sqrt(ratio)
	}
	auto wall = block(sim, 0, -1, 1);
	auto small = block(sim, 1, 1, 0.25);
	auto big = block(sim, ratio, 3, 0.5);
	big->set_velocity({ -v0, 0, 0 });
	small->set_stay_active(true);
	big->set_stay_active(true);

	int collisions = 0;
	small->set_collision_callback([&](const hop::collision<T> &) { ++collisions; });
	const T dt = 1 / hz;
	int separating = 0;
	for (long tick = 0; tick < hz * 20 && separating < hz; ++tick) {
		sim.update(dt);
		T vs = small->get_velocity().x, vb = big->get_velocity().x;
		separating = (vs >= 0 && vb >= vs) ? separating + 1 : 0;
	}
	T vs = small->get_velocity().x, vb = big->get_velocity().x;
	T energy = (vs * vs + ratio * vb * vb) / (ratio * v0 * v0);
	std::string out = std::to_string(collisions);
	if (small->get_position().x > big->get_position().x)
		out += ", passed through";
	else if (separating < hz)
		out += ", never separates";
	else if (energy < 0.99)
		out += ", stuck (" + std::to_string((int)std::lround(energy * 100)) + "% energy left)";
	return out;
}

int main() {
	const T ratios[] = { 1, 100, 10000, 1000000 };
	const T tuned_hz[] = { 60, 240, 10000, 100000 };

	printf("%10s %6s | %-24s | %-34s | %s\n", "mass ratio", "exact", "60 Hz speculative", "60 Hz sweep_slide",
	       "speculative, no micro threshold or speed cap");
	for (int i = 0; i < 4; ++i) {
		const T r = ratios[i];
		printf("%10.0f %6d | %-24s | %-34s | %s at %.0f Hz\n", r, exact_count(r),
		       run(r, 60, hop::contact_mode::speculative, false).c_str(),
		       run(r, 60, hop::contact_mode::sweep_slide, false).c_str(),
		       run(r, tuned_hz[i], hop::contact_mode::speculative, true).c_str(), tuned_hz[i]);
	}
	return 0;
}
