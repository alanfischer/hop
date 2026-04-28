// demo_stress.cpp — Stress test: many spheres bouncing in a closed room.
// Demonstrates BVH broad-phase performance under high object count.
// +/- : add/remove 25 spheres   R : reset   Space : pause

#include <chrono>
#include <cmath>
#include <hop/bvh_manager.h>
#include <hop/hop.h>
#include <raylib.h>
#include <vector>

// Hop is Z-up, raylib is Y-up — swap Y and Z
template <typename T> static Vector3 to_rl(const hop::vec3<T> & v) {
	using tr = hop::scalar_traits<T>;
	return { tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y) };
}

// Speed → color: blue (still) → green → yellow → red (fast)
static Color heat_color(float speed) {
	float t = std::min(speed / 12.0f, 1.0f);
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

template <typename T> static void run() {
	using tr = hop::scalar_traits<T>;

	constexpr int   ROOM_HALF   = 6;
	constexpr int   ROOM_HEIGHT = 12;
	constexpr float SPHERE_R    = 0.28f;
	constexpr int   MAX_SPHERES = 10000;
	constexpr int   STEP        = 100;
	constexpr int   INIT_COUNT  = 100;
	// Grid spacing between centers: diameter + small gap
	constexpr float SPACING     = SPHERE_R * 2.2f;
	constexpr int   COLS        = (int)((ROOM_HALF * 2.0f) / SPACING);

	// float -> T
	auto fi = [](int n) { return tr::from_int(n); };
	auto ff = [](float f) -> T { return tr::from_milli((int)(f * 1000.0f)); };

	hop::simulator<T> sim;
	hop::bvh_manager<T> bvh;
	sim.set_manager(&bvh);

	T half  = fi(ROOM_HALF);
	T hgt   = fi(ROOM_HEIGHT);
	T thick = fi(1);
	T zero  = T{};

	// Floor and ceiling extend beyond the side walls so that at every wall-floor
	// edge the floor's top face is always the nearest Minkowski face for any
	// sphere in the corner. Without this, when two Minkowski boxes overlap at a
	// corner the nearest-face tie-break can return the wrong normal, giving the
	// sphere a sideways impulse that doesn't resolve the floor overlap.
	T outer = half + thick;

	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-outer, -outer, -thick), hop::vec3<T>(outer, outer, zero)),
	    hop::vec3<T>());                                                         // floor
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-outer, -outer, zero), hop::vec3<T>(outer, outer, thick)),
	    hop::vec3<T>(zero, zero, hgt));                                          // ceiling
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-thick, -half, zero), hop::vec3<T>(zero, half, hgt)),
	    hop::vec3<T>(-half, zero, zero));                                        // -X
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(zero, -half, zero), hop::vec3<T>(thick, half, hgt)),
	    hop::vec3<T>(half, zero, zero));                                         // +X
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-half, -thick, zero), hop::vec3<T>(half, zero, hgt)),
	    hop::vec3<T>(zero, -half, zero));                                        // -Y
	make_wall(sim, bvh,
	    hop::aa_box<T>(hop::vec3<T>(-half, zero, zero), hop::vec3<T>(half, thick, hgt)),
	    hop::vec3<T>(zero, half, zero));                                         // +Y
	bvh.rebuild();

	// Tweakable physics values — changed at runtime, applied to all active spheres
	float cor  = 0.75f;
	float fric = 0.0f;

	// Pre-create all sphere solids (none added to sim yet)
	std::vector<std::shared_ptr<hop::solid<T>>> spheres;
	spheres.reserve(MAX_SPHERES);
	for (int i = 0; i < MAX_SPHERES; ++i) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_coefficient_of_restitution_override(true);
		s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));
		spheres.push_back(std::move(s));
	}

	int active = 0;

	auto apply_physics = [&](int i) {
		spheres[i]->set_coefficient_of_restitution(ff(cor));
		spheres[i]->set_coefficient_of_static_friction(ff(fric));
		spheres[i]->set_coefficient_of_dynamic_friction(ff(fric));
	};

	auto apply_to_all = [&]() {
		for (int i = 0; i < active; ++i)
			apply_physics(i);
	};

	// 3D grid placement throughout the room interior with random velocities in
	// all directions. No gravity means there's no floor pile-up to worry about.
	constexpr int Z_LAYERS = (int)((ROOM_HEIGHT - SPHERE_R * 2.0f) / SPACING);
	auto place = [&](int i) {
		int   layer = i / (COLS * COLS);
		int   rem   = i % (COLS * COLS);
		int   col   = rem % COLS;
		int   row   = rem / COLS;
		float x     = -(ROOM_HALF - SPACING * 0.5f) + col * SPACING;
		float y     = -(ROOM_HALF - SPACING * 0.5f) + row * SPACING;
		float z     = ROOM_HALF + (layer % Z_LAYERS) * SPACING - SPACING * 0.5f;
		// Pseudo-random velocity in all three axes
		float vx    = ((i * 7  + 3) % 21 - 10) * 0.5f;
		float vy    = ((i * 13 + 5) % 21 - 10) * 0.5f;
		float vz    = ((i * 17 + 9) % 21 - 10) * 0.5f;
		spheres[i]->set_position(hop::vec3<T>(ff(x), ff(y), ff(z)));
		spheres[i]->set_velocity(hop::vec3<T>(ff(vx), ff(vy), ff(vz)));
		apply_physics(i);
		spheres[i]->activate();
	};

	auto add_batch = [&](int n) {
		for (int i = 0; i < n && active < MAX_SPHERES; ++i, ++active) {
			place(active);
			sim.add_solid(spheres[active]);
			bvh.add_solid(spheres[active].get(), false);
		}
	};

	auto remove_batch = [&](int n) {
		for (int i = 0; i < n && active > 0; ++i, --active) {
			bvh.remove_solid(spheres[active - 1].get());
			sim.remove_solid(spheres[active - 1]);
		}
	};

	add_batch(INIT_COUNT);

	InitWindow(1280, 720, "hop — stress test");
	SetTargetFPS(120);

	bool  paused    = false;
	float cam_angle = 0.2f;
	float phys_ms   = 0.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		if (dt > 0.1f)
			dt = 0.1f;

		if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))      add_batch(STEP);
		if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) remove_batch(STEP);
		if (IsKeyPressed(KEY_R)) { remove_batch(active); add_batch(INIT_COUNT); }
		if (IsKeyPressed(KEY_SPACE)) paused = !paused;

		// Restitution: [ / ]   Friction: , / .
		if (IsKeyPressed(KEY_RIGHT_BRACKET)) { cor  = std::min(1.0f, cor  + 0.05f); apply_to_all(); }
		if (IsKeyPressed(KEY_LEFT_BRACKET))  { cor  = std::max(0.0f, cor  - 0.05f); apply_to_all(); }
		if (IsKeyPressed(KEY_PERIOD))        { fric = std::min(1.0f, fric + 0.05f); apply_to_all(); }
		if (IsKeyPressed(KEY_COMMA))         { fric = std::max(0.0f, fric - 0.05f); apply_to_all(); }

		if (!paused) {
			auto t0 = std::chrono::high_resolution_clock::now();
			sim.update(16);
			auto t1 = std::chrono::high_resolution_clock::now();
			phys_ms   = std::chrono::duration<float, std::milli>(t1 - t0).count();
			cam_angle += 0.15f * dt;
		}

		// Orbiting camera — room center in raylib Y-up is (0, ROOM_HEIGHT/2, 0)
		Camera3D cam  = {};
		cam.position  = { 30.0f * cosf(cam_angle), 10.0f, 30.0f * sinf(cam_angle) };
		cam.target    = { 0.0f, (float)ROOM_HEIGHT * 0.5f, 0.0f };
		cam.up        = { 0.0f, 1.0f, 0.0f };
		cam.fovy      = 55.0f;
		cam.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 12, 12, 20, 255 });
		BeginMode3D(cam);

		// Room wireframe (raylib Y-up: X∈[-6,6], Y∈[0,12], Z∈[-6,6])
		const float W = (float)(ROOM_HALF * 2);
		const float H = (float)ROOM_HEIGHT;
		DrawCubeWires({ 0.0f, H * 0.5f, 0.0f }, W, H, W, { 50, 50, 80, 255 });
		DrawCube(     { 0.0f, -0.02f,   0.0f }, W, 0.04f, W, { 30, 30, 60, 220 });

		for (int i = 0; i < active; ++i) {
			Vector3 p     = to_rl(spheres[i]->get_position());
			float   speed = tr::to_float(hop::length(spheres[i]->get_velocity()));
			DrawSphereEx(p, SPHERE_R, 4, 6, heat_color(speed));
		}

		EndMode3D();

		// HUD
		int   fps       = GetFPS();
		Color fps_color = fps >= 50 ? GREEN : fps >= 30 ? YELLOW : RED;
		DrawText(TextFormat("FPS:     %d", fps),                       12, 12, 22, fps_color);
		int sleeping = 0;
		for (int i = 0; i < active; ++i) if (!spheres[i]->active()) ++sleeping;
		DrawText(TextFormat("Objects: %d / %d  (sleep: %d)", active, MAX_SPHERES, sleeping), 12, 38, 22, WHITE);
		Color pc = phys_ms < 8.0f ? GREEN : phys_ms < 16.0f ? YELLOW : RED;
		DrawText(TextFormat("Physics: %.1f ms", (double)phys_ms),      12, 64, 22, pc);
		DrawText(TextFormat("Restitution: %.2f  [/]", (double)cor),    12, 92, 20, LIGHTGRAY);
		DrawText(TextFormat("Friction:    %.2f  ,/.", (double)fric),   12, 116, 20, LIGHTGRAY);

		if (paused)
			DrawText("PAUSED", 560, 330, 48, { 255, 220, 60, 220 });

		DrawText("+/- : spheres   R : reset   Space : pause   [/] : restitution   ,/. : friction",
		         12, 700, 16, { 90, 90, 120, 255 });

		EndDrawing();
	}

	CloseWindow();
}

int main() {
	run<float>();
	return 0;
}
