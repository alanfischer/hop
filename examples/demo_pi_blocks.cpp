// Galperin's billiard: a big block slides into a small one resting against a wall.
// With a mass ratio of 100^(N-1), the collisions number the first N digits of pi.
//
// It opens a raylib window and counts the collisions as they happen; --headless runs the
// tables instead. Each ratio runs at 60 Hz in both contact modes, then with the
// micro-collision threshold and speed cap lifted, at a tick rate where the small block never
// moves more than a fraction of its width per tick. A second table repeats that last run in
// each scalar type.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <hop/fixed16.h>
#include <hop/fixed32.h>
#include <hop/hop.h>
#include <memory>
#include <raylib.h>
#include <string>
#include <type_traits>

namespace {

template <typename T> T num(double d) {
	if constexpr (hop::is_fixed_scalar_v<T>)
		return T::from_float(static_cast<float>(d));
	else
		return static_cast<T>(d);
}

template <typename T> double dbl(T v) { return hop::scalar_traits<T>::to_float(v); }

const double ratios[] = { 1, 100, 10000, 1000000 };
const double tuned_hz[] = { 60, 240, 10000, 100000 };
const double v0 = 5;
const double small_half = 0.25, big_half = 0.5;

int exact_count(double ratio) {
	double v = 0, V = -1;
	int n = 0;
	for (;;) {
		if (V < v) {
			double nv = ((1 - ratio) * v + 2 * ratio * V) / (1 + ratio);
			V = ((ratio - 1) * V + 2 * v) / (1 + ratio);
			v = nv;
		} else if (v < 0)
			v = -v;
		else
			return n;
		++n;
	}
}

template <typename T> struct blocks {
	hop::simulator<T> sim;
	std::shared_ptr<hop::solid<T>> wall, small, big;

	blocks(double ratio, hop::contact_mode mode, bool tuned, double small_mass = 1) {
		sim.set_gravity({ T {}, T {}, T {} });
		sim.set_default_contact_mode(mode);
		if (tuned) {
			sim.set_micro_collision_threshold(T {});
			// The small block peaks at v0 * sqrt(ratio); fixed16 tops out near 32768.
			sim.set_max_velocity_component(num<T>(std::is_same_v<T, hop::fixed16> ? 30000 : 1e6));
		}
		wall = add(0, -1, 1);
		small = add(small_mass, 1, small_half);
		big = add(ratio * small_mass, 3, big_half);
		big->set_velocity({ num<T>(-v0), T {}, T {} });
		small->set_stay_active(true);
		big->set_stay_active(true);
	}

	std::shared_ptr<hop::solid<T>> add(double mass, double x, double half) {
		auto s = std::make_shared<hop::solid<T>>();
		if (mass > 0)
			s->set_mass(num<T>(mass));
		else
			s->set_infinite_mass();
		s->set_coefficient_of_gravity(T {});
		s->set_coefficient_of_restitution(hop::scalar_traits<T>::one());
		s->set_coefficient_of_static_friction(T {});
		s->set_coefficient_of_dynamic_friction(T {});
		const T h = num<T>(half);
		s->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>({ -h, -h, -h }, { h, h, h })));
		s->set_position({ num<T>(x), T {}, T {} });
		sim.add_solid(s);
		return s;
	}

	// Both moving right, the small one no faster: nothing is left to hit.
	bool separating() const {
		double vs = dbl(small->get_velocity().x), vb = dbl(big->get_velocity().x);
		return vs >= 0 && vb >= vs;
	}
};

template <typename T>
std::string run(double ratio, double hz, hop::contact_mode mode, bool tuned, double small_mass = 1) {
	blocks<T> b(ratio, mode, tuned, small_mass);
	int collisions = 0;
	b.small->set_collision_callback([&](const hop::collision<T> &) { ++collisions; });
	const T dt = num<T>(1 / hz);
	int separating = 0;
	for (long tick = 0; tick < hz * 20 && separating < hz; ++tick) {
		b.sim.update(dt);
		separating = b.separating() ? separating + 1 : 0;
	}
	double vs = dbl(b.small->get_velocity().x), vb = dbl(b.big->get_velocity().x);
	double energy = (vs * vs + ratio * vb * vb) / (ratio * v0 * v0);
	std::string out = std::to_string(collisions);
	if (b.small->get_position().x > b.big->get_position().x)
		out += ", passed through";
	else if (separating < hz)
		out += ", never separates";
	else if (energy < 0.99)
		out += ", stuck (" + std::to_string((int)std::lround(energy * 100)) + "% energy left)";
	return out;
}

template <typename T> void scalar_row(const char * name) {
	const auto spec = hop::contact_mode::speculative;
	printf("%8s", name);
	for (int i = 0; i < 4; ++i) {
		const double r = ratios[i];
		std::string cell;
		if constexpr (std::is_same_v<T, hop::fixed16>) {
			// 16 fraction bits: an inverse mass of 1e-4 is 6 raw units, and dt can be no
			// finer than one. Splitting the ratio as 1/sqrt(r) : sqrt(r) rounds both inverse
			// masses least, and the tick rate stops at one raw unit of dt.
			const double hz = std::min(tuned_hz[i], 65536.0), small_mass = 1 / std::sqrt(r);
			cell = run<T>(r, hz, spec, true, small_mass);
			const T one = hop::scalar_traits<T>::one();
			const double held = dbl(one / num<T>(small_mass)) / dbl(one / num<T>(r * small_mass));
			if (std::fabs(held / r - 1) > 1e-4) {
				char buf[64];
				snprintf(buf, sizeof buf, " (holds %.4g, exact %d)", held, exact_count(held));
				cell += buf;
			}
			if (hz != tuned_hz[i])
				cell += " @" + std::to_string((int)hz) + " Hz";
		} else {
			cell = run<T>(r, tuned_hz[i], spec, true);
		}
		printf(" | %-24s", cell.c_str());
	}
	printf("\n");
}

void run_tables() {
	printf("%10s %6s | %-24s | %-34s | %s\n", "mass ratio", "exact", "60 Hz speculative", "60 Hz sweep_slide",
	       "speculative, no micro threshold or speed cap");
	for (int i = 0; i < 4; ++i) {
		const double r = ratios[i];
		printf("%10.0f %6d | %-24s | %-34s | %s at %.0f Hz\n", r, exact_count(r),
		       run<double>(r, 60, hop::contact_mode::speculative, false).c_str(),
		       run<double>(r, 60, hop::contact_mode::sweep_slide, false).c_str(),
		       run<double>(r, tuned_hz[i], hop::contact_mode::speculative, true).c_str(), tuned_hz[i]);
	}

	printf("\nThe last column in each scalar type:\n%8s", "");
	for (int i = 0; i < 4; ++i)
		printf(" | %-24s", ("ratio " + std::to_string((long)ratios[i]) + ", exact " + std::to_string(exact_count(ratios[i]))).c_str());
	printf("\n");
	scalar_row<double>("double");
	scalar_row<float>("float");
	scalar_row<hop::fixed32>("fixed32");
	scalar_row<hop::fixed16>("fixed16");
}

// The tuned speculative run, drawn: 1-4 pick the mass ratio, Up/Down change the time scale,
// Space pauses, R restarts.
struct view_scene {
	blocks<double> b;
	int wall_hits = 0, block_hits = 0;
	int separating = 0;
	double accum = 0;
	double wall_flash = 0, block_flash = 0;

	explicit view_scene(int r) : b(ratios[r], hop::contact_mode::speculative, true) {
		b.small->set_collision_callback([this](const hop::collision<double> & c) {
			if (c.collider == b.wall.get()) {
				++wall_hits;
				wall_flash = 1;
			} else {
				++block_hits;
				block_flash = 1;
			}
		});
	}
	int collisions() const { return wall_hits + block_hits; }
};

void run_view() {
	const int width = 1280, height = 600;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
	InitWindow(width, height, "Hop Physics - Pi Blocks");
	SetTargetFPS(60);

	int r = 3;
	int exact = exact_count(ratios[r]);
	double time_scale = 1.0 / 16;
	bool paused = false;
	auto s = std::make_unique<view_scene>(r);

	const float px = 160; // pixels per unit
	const float wall_x = 120;
	const float floor_y = 420;
	auto sx = [&](double x) { return wall_x + (float)x * px; };

	while (!WindowShouldClose()) {
		for (int k = 0; k < 4; ++k) {
			if (IsKeyPressed(KEY_ONE + k)) {
				r = k;
				exact = exact_count(ratios[r]);
				s = std::make_unique<view_scene>(r);
			}
		}
		if (IsKeyPressed(KEY_R))
			s = std::make_unique<view_scene>(r);
		if (IsKeyPressed(KEY_SPACE))
			paused = !paused;
		if (IsKeyPressed(KEY_UP))
			time_scale = std::fmin(time_scale * 2, 1);
		if (IsKeyPressed(KEY_DOWN))
			time_scale = std::fmax(time_scale / 2, 1.0 / 1024);

		const double dt = 1 / tuned_hz[r];
		const double frame = GetFrameTime();
		if (!paused) {
			s->accum += std::fmin(frame, 0.1) * time_scale;
			while (s->accum >= dt) {
				s->accum -= dt;
				s->b.sim.update(dt);
				s->separating = s->b.separating() ? s->separating + 1 : 0;
			}
		}
		s->wall_flash = std::fmax(0, s->wall_flash - frame * 6);
		s->block_flash = std::fmax(0, s->block_flash - frame * 6);

		BeginDrawing();
		ClearBackground(RAYWHITE);

		DrawRectangle(0, 0, (int)wall_x, (int)floor_y, ColorLerp(DARKGRAY, ORANGE, (float)s->wall_flash));
		DrawLine(0, (int)floor_y, width, (int)floor_y, DARKGRAY);

		auto draw_block = [&](const std::shared_ptr<hop::solid<double>> & b, double half, Color c) {
			double x = b->get_position().x;
			float w = (float)(2 * half) * px;
			Rectangle rect = { sx(x - half), floor_y - w, w, w };
			DrawRectangleRec(rect, ColorLerp(c, ORANGE, (float)s->block_flash));
			DrawRectangleLinesEx(rect, 2, BLACK);
		};
		draw_block(s->b.small, small_half, SKYBLUE);
		draw_block(s->b.big, big_half, DARKBLUE);

		float big_x = sx(s->b.big->get_position().x);
		const char * mass = TextFormat("%.0f kg", ratios[r]);
		DrawText(mass, (int)big_x - MeasureText(mass, 20) / 2, (int)(floor_y + 10), 20, DARKBLUE);
		float small_x = sx(s->b.small->get_position().x);
		DrawText("1 kg", (int)small_x - MeasureText("1 kg", 20) / 2, (int)(floor_y + 34), 20, SKYBLUE);

		DrawText(TextFormat("%d", s->collisions()), 40, 470, 80, BLACK);
		DrawText(TextFormat("collisions  (%d wall, %d block; exact %d)", s->wall_hits, s->block_hits, exact), 40, 550, 20,
		         GRAY);
		if (s->separating > 10) {
			int digits = r + 1;
			const char * pi = TextFormat("pi ~ %.*f", digits - 1, s->collisions() / std::pow(10.0, digits - 1));
			DrawText(pi, 420, 490, 50, s->collisions() == exact ? DARKGREEN : RED);
		}

		DrawText(TextFormat("mass ratio %.0f   %.0f Hz   time x%g%s", ratios[r], tuned_hz[r], time_scale,
		                    paused ? "   PAUSED" : ""),
		         20, 20, 20, DARKGRAY);
		DrawText(TextFormat("small v %+.2f   big v %+.4f", s->b.small->get_velocity().x, s->b.big->get_velocity().x),
		         20, 46, 20, DARKGRAY);
		DrawText("1-4: mass ratio   Up/Down: time scale   Space: pause   R: restart   (--headless runs the tables)",
		         20, 72, 20, GRAY);

		EndDrawing();
	}

	CloseWindow();
}

} // namespace

int main(int argc, char ** argv) {
	bool headless = false;
	for (int i = 1; i < argc; ++i)
		if (std::strcmp(argv[i], "--headless") == 0)
			headless = true;
	if (headless)
		run_tables();
	else
		run_view();
	return 0;
}
