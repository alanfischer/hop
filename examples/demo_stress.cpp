// demo_stress.cpp — Stress test: a few thousand spheres dropped into a box.
// Demonstrates BVH broad-phase performance under high object count, and the
// contact solver settling a frictional heap (compare demo_stress_rp3d). The
// spheres are given friction and modest restitution so the pile bounces off the
// initial drop and then comes to rest instead of sloshing forever; see
// docs/settling_plan.md for why the initial fill is kept shallow.
// Space : pause

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
	w->set_coefficient_of_static_friction(tr::from_milli(600));
	w->set_coefficient_of_dynamic_friction(tr::from_milli(600));
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
	// Grid spacing between centers: diameter + small gap
	constexpr float SPACING     = SPHERE_R * 2.2f;
	constexpr int   COLS        = (int)((ROOM_HALF * 2.0f) / SPACING);
	// Spawn only the lower layers rather than packing the box solid to the
	// ceiling. A pile dozens of spheres deep does not fully quiesce in a
	// translation-only swept solver (resting load propagates one contact layer
	// per solver iteration, and the velocity-only contact response injects a
	// little energy per layer each tick — see docs/settling_plan.md). Keeping the
	// initial pile shallow lets it settle into a stable heap with headroom above
	// for the bounce, while still exercising the broad phase with a few thousand
	// bodies.
	constexpr int   FILL_LAYERS = 7;
	constexpr int   COUNT       = COLS * COLS * FILL_LAYERS;

	// float -> T
	auto fi = [](int n) { return tr::from_int(n); };
	auto ff = [](float f) -> T { return tr::from_milli((int)(f * 1000.0f)); };

	hop::simulator<T> sim;
	hop::bvh_manager<T> bvh;
	sim.set_manager(&bvh);
	sim.set_deactivate_count(32);
	// Friction is what dissipates a frictionless sphere's only un-damped degree
	// of freedom — tangential sliding — so a heap can actually come to rest
	// instead of sloshing indefinitely. More solver iterations let resting load
	// propagate through the pile in a single tick.
	sim.set_solver_iterations(32);

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

	// 3D grid filling the room interior with random velocities in all directions.
	// Gravity is active; the ceiling keeps balls from escaping upward.
	std::vector<std::shared_ptr<hop::solid<T>>> spheres;
	spheres.reserve(COUNT);
	for (int i = 0; i < COUNT; ++i) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_coefficient_of_restitution_override(true);
		s->set_coefficient_of_restitution(ff(0.6f));   // bouncy enough to rebound off the initial drop
		s->set_coefficient_of_static_friction(ff(0.6f));
		s->set_coefficient_of_dynamic_friction(ff(0.6f));
		s->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(ff(SPHERE_R))));

		int   layer = i / (COLS * COLS);
		int   rem   = i % (COLS * COLS);
		int   col   = rem % COLS;
		int   row   = rem / COLS;
		float x     = -(ROOM_HALF - SPACING * 0.5f) + col * SPACING;
		float y     = -(ROOM_HALF - SPACING * 0.5f) + row * SPACING;
		float z     = SPACING * 0.5f + layer * SPACING;
		// Pseudo-random velocity in all three axes
		float vx    = ((i * 7  + 3) % 21 - 10) * 0.5f;
		float vy    = ((i * 13 + 5) % 21 - 10) * 0.5f;
		float vz    = ((i * 17 + 9) % 21 - 10) * 0.5f;
		s->set_position(hop::vec3<T>(ff(x), ff(y), ff(z)));
		s->set_velocity(hop::vec3<T>(ff(vx), ff(vy), ff(vz)));
		s->activate();
		sim.add_solid(s);
		bvh.add_solid(s.get(), false);
		spheres.push_back(std::move(s));
	}

	InitWindow(1280, 720, "hop — stress test");
	SetTargetFPS(120);

	bool  paused    = false;
	float cam_angle = 0.2f;
	float phys_ms   = 0.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		if (dt > 0.1f)
			dt = 0.1f;

		if (IsKeyPressed(KEY_SPACE)) paused = !paused;

		if (!paused) {
			auto t0 = std::chrono::high_resolution_clock::now();
			sim.update(tr::from_milli(16));
			auto t1 = std::chrono::high_resolution_clock::now();
			phys_ms   = std::chrono::duration<float, std::milli>(t1 - t0).count();
			cam_angle += 0.15f * dt;
		}

		// Orbiting camera — room center in raylib Y-up is (0, ROOM_HEIGHT/2, 0)
		Camera3D cam  = {};
		cam.position  = { 30.0f * cosf(cam_angle), 10.0f, 30.0f * sinf(cam_angle) };
		cam.target    = { 0.0f, (float)ROOM_HEIGHT * 0.25f, 0.0f };  // pile sits low in the tall box
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

		for (auto & s : spheres) {
			Vector3 p     = to_rl(s->get_position());
			float   speed = tr::to_float(hop::length(s->get_velocity()));
			Color   color = s->active() ? heat_color(speed) : GRAY;
			DrawSphereEx(p, SPHERE_R, 4, 6, color);
		}

		EndMode3D();

		// HUD
		int   fps       = GetFPS();
		Color fps_color = fps >= 50 ? GREEN : fps >= 30 ? YELLOW : RED;
		DrawText(TextFormat("FPS:     %d", fps),                       12, 12, 22, fps_color);
		int sleeping = 0;
		for (auto & s : spheres) if (!s->active()) ++sleeping;
		DrawText(TextFormat("Objects: %d  (sleep: %d)", COUNT, sleeping), 12, 38, 22, WHITE);
		Color pc = phys_ms < 8.0f ? GREEN : phys_ms < 16.0f ? YELLOW : RED;
		DrawText(TextFormat("Physics: %.1f ms", (double)phys_ms),      12, 64, 22, pc);

		if (paused)
			DrawText("PAUSED", 560, 330, 48, { 255, 220, 60, 220 });

		DrawText("Space : pause", 12, 700, 16, { 90, 90, 120, 255 });

		EndDrawing();
	}

	CloseWindow();
}

int main() {
	run<float>();
	return 0;
}
