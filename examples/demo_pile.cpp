// demo_pile.cpp — Gravity pile: a column of spheres dropped onto a floor.
// Exercises the post-integration contact solver (pass B) — without it, the
// stack jitters and doesn't transmit load through to the floor in one tick.
// Space : pause   R : reset   I/K : solver iterations -/+   G : toggle gravity

#include <chrono>
#include <cmath>
#include <hop/bvh_manager.h>
#include <hop/hop.h>
#include <raylib.h>
#include <vector>

template <typename T> static Vector3 to_rl(const hop::vec3<T> & v) {
	using tr = hop::scalar_traits<T>;
	return { tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y) };
}

static Color heat_color(float speed) {
	float t = std::min(speed / 6.0f, 1.0f);
	if (t < 0.5f) {
		float s = t * 2.0f;
		return { 0, (unsigned char)(80 + 175 * s), (unsigned char)(255 - 200 * s), 255 };
	}
	float s = (t - 0.5f) * 2.0f;
	return { (unsigned char)(255 * s), (unsigned char)(255 * (1.0f - s)), 0, 255 };
}

template <typename T>
static std::shared_ptr<hop::solid<T>> make_wall(hop::simulator<T> & sim,
                                                 hop::bvh_manager<T> & bvh,
                                                 hop::aa_box<T> box,
                                                 hop::vec3<T> pos) {
	auto w = std::make_shared<hop::solid<T>>();
	w->set_infinite_mass();
	w->set_coefficient_of_gravity(T{});
	w->set_coefficient_of_restitution(T{});
	w->set_coefficient_of_static_friction(hop::scalar_traits<T>::half());
	w->set_coefficient_of_dynamic_friction(hop::scalar_traits<T>::half());
	w->add_shape(std::make_shared<hop::shape<T>>(box));
	w->set_position(pos);
	sim.add_solid(w);
	bvh.add_solid(w.get(), true);
	return w;
}

template <typename T> static void run() {
	using tr = hop::scalar_traits<T>;

	constexpr int   ROOM_HALF  = 5;
	constexpr int   ROOM_HEIGHT = 18;
	constexpr float SPHERE_R   = 0.30f;
	// Drop spheres in a loose grid above the floor so they pile naturally.
	constexpr int   GRID       = 8;
	constexpr int   LAYERS     = 20;
	constexpr int   COUNT      = GRID * GRID * LAYERS;

	auto fi = [](int n) { return tr::from_int(n); };
	auto ff = [](float f) -> T { return tr::from_milli((int)(f * 1000.0f)); };

	hop::simulator<T> sim;
	hop::bvh_manager<T> bvh;
	sim.set_manager(&bvh);
	sim.set_deactivate_count(32);

	T half  = fi(ROOM_HALF);
	T hgt   = fi(ROOM_HEIGHT);
	T thick = fi(1);
	T zero  = T{};
	T outer = half + thick;

	// Floor + four side walls form an open-top box.
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-outer, -outer, -thick), hop::vec3<T>(outer, outer, zero)),
	    hop::vec3<T>());
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

	std::vector<std::shared_ptr<hop::solid<T>>> spheres;
	spheres.reserve(COUNT);

	auto spawn_pile = [&]() {
		for (auto & s : spheres) {
			sim.remove_solid(s);
			bvh.remove_solid(s.get());
		}
		spheres.clear();
		float pitch = SPHERE_R * 2.4f;
		float base  = -(GRID * pitch * 0.5f) + pitch * 0.5f;
		for (int i = 0; i < COUNT; ++i) {
			auto s = std::make_shared<hop::solid<T>>();
			s->set_mass(tr::one());
			s->set_restitution_combine(hop::restitution_combine::minimum);
			s->set_coefficient_of_restitution(ff(0.05f));
			s->set_coefficient_of_static_friction(ff(0.4f));
			s->set_coefficient_of_dynamic_friction(ff(0.4f));
			s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));

			int layer = i / (GRID * GRID);
			int rem   = i % (GRID * GRID);
			int col   = rem % GRID;
			int row   = rem / GRID;
			// Small per-layer jitter so the column doesn't perfectly column-align.
			float jx = ((i * 7) % 5 - 2) * 0.02f;
			float jy = ((i * 13) % 5 - 2) * 0.02f;
			float x  = base + col * pitch + jx;
			float y  = base + row * pitch + jy;
			float z  = SPHERE_R + 2.0f + layer * pitch;
			s->set_position(hop::vec3<T>(ff(x), ff(y), ff(z)));
			s->activate();
			sim.add_solid(s);
			bvh.add_solid(s.get(), false);
			spheres.push_back(std::move(s));
		}
	};
	spawn_pile();

	InitWindow(1280, 720, "hop — pile test");
	SetTargetFPS(120);

	bool  paused    = false;
	bool  gravity_on = true;
	float cam_angle = 0.4f;
	float phys_ms   = 0.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		if (dt > 0.1f)
			dt = 0.1f;

		if (IsKeyPressed(KEY_SPACE)) paused = !paused;
		if (IsKeyPressed(KEY_R))     spawn_pile();
		if (IsKeyPressed(KEY_I))     sim.set_solver_iterations(std::max(1, sim.get_solver_iterations() - 1));
		if (IsKeyPressed(KEY_K))     sim.set_solver_iterations(sim.get_solver_iterations() + 1);
		if (IsKeyPressed(KEY_G)) {
			gravity_on = !gravity_on;
			if (gravity_on) {
				sim.set_gravity({ T{}, T{}, -tr::from_milli(9810) });
			} else {
				sim.set_gravity({ T{}, T{}, T{} });
			}
		}

		if (!paused) {
			auto t0 = std::chrono::high_resolution_clock::now();
			sim.update(tr::from_milli(16));
			auto t1 = std::chrono::high_resolution_clock::now();
			phys_ms   = std::chrono::duration<float, std::milli>(t1 - t0).count();
			cam_angle += 0.10f * dt;
		}

		Camera3D cam = {};
		cam.position = { 22.0f * cosf(cam_angle), 12.0f, 22.0f * sinf(cam_angle) };
		cam.target   = { 0.0f, (float)ROOM_HEIGHT * 0.25f, 0.0f };
		cam.up       = { 0.0f, 1.0f, 0.0f };
		cam.fovy     = 55.0f;
		cam.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 12, 12, 20, 255 });
		BeginMode3D(cam);

		const float W = (float)(ROOM_HALF * 2);
		const float H = (float)ROOM_HEIGHT;
		DrawCubeWires({ 0.0f, H * 0.5f, 0.0f }, W, H, W, { 50, 50, 80, 255 });
		DrawCube(     { 0.0f, -0.02f,   0.0f }, W, 0.04f, W, { 30, 30, 60, 220 });

		for (auto & s : spheres) {
			Vector3 p     = to_rl(s->get_position());
			float   speed = tr::to_float(hop::length(s->get_velocity()));
			Color   color = s->active() ? heat_color(speed) : GRAY;
			DrawSphereEx(p, SPHERE_R, 4, 6, color);
		}

		EndMode3D();

		int   fps       = GetFPS();
		Color fps_color = fps >= 50 ? GREEN : fps >= 30 ? YELLOW : RED;
		DrawText(TextFormat("FPS:        %d", fps),                       12, 12, 22, fps_color);
		int sleeping = 0;
		for (auto & s : spheres) if (!s->active()) ++sleeping;
		DrawText(TextFormat("Objects:    %d  (sleep: %d)", COUNT, sleeping), 12, 38, 22, WHITE);
		Color pc = phys_ms < 8.0f ? GREEN : phys_ms < 16.0f ? YELLOW : RED;
		DrawText(TextFormat("Physics:    %.1f ms", (double)phys_ms),      12, 64, 22, pc);
		DrawText(TextFormat("Iterations: %d", sim.get_solver_iterations()), 12, 90, 22, WHITE);
		DrawText(TextFormat("Gravity:    %s", gravity_on ? "ON" : "OFF"),  12, 116, 22, WHITE);

		if (paused)
			DrawText("PAUSED", 560, 330, 48, { 255, 220, 60, 220 });

		DrawText("Space : pause   R : reset   I/K : iterations -/+   G : gravity", 12, 690, 16, { 90, 90, 120, 255 });

		EndDrawing();
	}

	CloseWindow();
}

int main() {
	run<float>();
	return 0;
}
