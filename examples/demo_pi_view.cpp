// demo_pi_blocks, drawn: watch the big block push in and count the collisions as they happen.
// 1-4 pick the mass ratio, Up/Down change the time scale, Space pauses, R restarts.

#include <cmath>
#include <hop/hop.h>
#include <raylib.h>

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

static const T ratios[] = { 1, 100, 10000, 1000000 };
static const T tick_hz[] = { 60, 240, 10000, 100000 };
static const int exact[] = { 3, 31, 314, 3141 };
static const T v0 = 5;
static const T small_half = 0.25, big_half = 0.5;

struct scene {
	hop::simulator<T> sim;
	std::shared_ptr<hop::solid<T>> wall, small, big;
	int wall_hits = 0, block_hits = 0;
	int separating = 0;
	double accum = 0;
	double wall_flash = 0, block_flash = 0;

	explicit scene(int r) {
		sim.set_gravity({ 0, 0, 0 });
		sim.set_default_contact_mode(hop::contact_mode::speculative);
		sim.set_micro_collision_threshold(0);
		sim.set_max_velocity_component(1e6);
		wall = block(sim, 0, -1, 1);
		small = block(sim, 1, 1, small_half);
		big = block(sim, ratios[r], 3, big_half);
		big->set_velocity({ -v0, 0, 0 });
		small->set_stay_active(true);
		big->set_stay_active(true);
		small->set_collision_callback([this](const hop::collision<T> & c) {
			if (c.collider == wall.get()) {
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

int main() {
	const int width = 1280, height = 600;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
	InitWindow(width, height, "Hop Physics - Pi Blocks");
	SetTargetFPS(60);

	int r = 3;
	double time_scale = 1.0 / 16;
	bool paused = false;
	auto s = std::make_unique<scene>(r);

	const float px = 160; // pixels per unit
	const float wall_x = 120;
	const float floor_y = 420;
	auto sx = [&](T x) { return wall_x + (float)x * px; };

	while (!WindowShouldClose()) {
		for (int k = 0; k < 4; ++k) {
			if (IsKeyPressed(KEY_ONE + k)) {
				r = k;
				s = std::make_unique<scene>(r);
			}
		}
		if (IsKeyPressed(KEY_R))
			s = std::make_unique<scene>(r);
		if (IsKeyPressed(KEY_SPACE))
			paused = !paused;
		if (IsKeyPressed(KEY_UP))
			time_scale = std::fmin(time_scale * 2, 1);
		if (IsKeyPressed(KEY_DOWN))
			time_scale = std::fmax(time_scale / 2, 1.0 / 1024);

		const T dt = 1 / tick_hz[r];
		const double frame = GetFrameTime();
		if (!paused) {
			s->accum += std::fmin(frame, 0.1) * time_scale;
			while (s->accum >= dt) {
				s->accum -= dt;
				s->sim.update(dt);
				T vs = s->small->get_velocity().x, vb = s->big->get_velocity().x;
				s->separating = (vs >= 0 && vb >= vs) ? s->separating + 1 : 0;
			}
		}
		s->wall_flash = std::fmax(0, s->wall_flash - frame * 6);
		s->block_flash = std::fmax(0, s->block_flash - frame * 6);
		const bool done = s->separating > 10; // both moving right, small slower: nothing left to hit

		BeginDrawing();
		ClearBackground(RAYWHITE);

		DrawRectangle(0, 0, (int)wall_x, (int)floor_y, ColorLerp(DARKGRAY, ORANGE, (float)s->wall_flash));
		DrawLine(0, (int)floor_y, width, (int)floor_y, DARKGRAY);

		auto draw_block = [&](const std::shared_ptr<hop::solid<T>> & b, T half, Color c, double flash) {
			T x = b->get_position().x;
			float w = (float)(2 * half) * px;
			Rectangle rect = { sx(x - half), floor_y - w, w, w };
			DrawRectangleRec(rect, ColorLerp(c, ORANGE, (float)flash));
			DrawRectangleLinesEx(rect, 2, BLACK);
		};
		draw_block(s->small, small_half, SKYBLUE, s->block_flash);
		draw_block(s->big, big_half, DARKBLUE, s->block_flash);

		float big_x = sx(s->big->get_position().x);
		const char * mass = TextFormat("%.0f kg", ratios[r]);
		DrawText(mass, (int)big_x - MeasureText(mass, 20) / 2, (int)(floor_y + 10), 20, DARKBLUE);
		float small_x = sx(s->small->get_position().x);
		DrawText("1 kg", (int)small_x - MeasureText("1 kg", 20) / 2, (int)(floor_y + 34), 20, SKYBLUE);

		DrawText(TextFormat("%d", s->collisions()), 40, 470, 80, BLACK);
		DrawText(TextFormat("collisions  (%d wall, %d block; exact %d)", s->wall_hits, s->block_hits, exact[r]), 40,
		         550, 20, GRAY);
		if (done) {
			int digits = r + 1;
			const char * pi = TextFormat("pi ~ %.*f", digits - 1, s->collisions() / std::pow(10.0, digits - 1));
			DrawText(pi, 420, 490, 50, s->collisions() == exact[r] ? DARKGREEN : RED);
		}

		DrawText(TextFormat("mass ratio %.0f   %.0f Hz   time x%g%s", ratios[r], tick_hz[r], time_scale,
		                    paused ? "   PAUSED" : ""),
		         20, 20, 20, DARKGRAY);
		DrawText(TextFormat("small v %+.2f   big v %+.4f", s->small->get_velocity().x, s->big->get_velocity().x), 20,
		         46, 20, DARKGRAY);
		DrawText("1-4: mass ratio   Up/Down: time scale   Space: pause   R: restart", 20, 72, 20, GRAY);

		EndDrawing();
	}

	CloseWindow();
	return 0;
}
