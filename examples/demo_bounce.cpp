// demo_bounce.cpp — Raylib visualization of hop physics
// A box, sphere, and capsule bounce around inside a closed room.
// Pass --fixed to use fixed16 arithmetic instead of float.

#include <hop/hop.h>
#include <raylib.h>
#include <rlgl.h>
#include <cstring>
#include <string>
#include <cmath>

// Hop is Z-up, raylib is Y-up — swap Y and Z
template<typename T>
Vector3 to_raylib(const hop::vec3<T>& v) {
	using tr = hop::scalar_traits<T>;
	return {tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y)};
}

// Helper: create a wall solid (infinite mass, no gravity, positioned at `pos`)
template<typename T>
std::shared_ptr<hop::solid<T>> make_wall(hop::simulator<T>& sim, const hop::aa_box<T>& box, const hop::vec3<T>& pos) {
	using tr = hop::scalar_traits<T>;
	auto wall = std::make_shared<hop::solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T{});
	wall->set_coefficient_of_restitution(tr::from_milli(900));
	auto sh = std::make_shared<hop::shape<T>>(box);
	wall->add_shape(sh);
	wall->set_position(pos);
	sim.add_solid(wall);
	return wall;
}

static const char* capture_dir = nullptr;

template<typename T>
void run() {
	using tr = hop::scalar_traits<T>;

	hop::simulator<T> sim;

	// Room dimensions: 10x10x10, centered at origin, floor at z=0, ceiling at z=10
	const T half_size = tr::from_int(3);
	const T size = tr::from_int(6);
	const T wall_thick = tr::one();
	const T zero = T{};

	// Floor: z = -wall_thick to 0
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>(-half_size, -half_size, -wall_thick),
		hop::vec3<T>( half_size,  half_size, zero)
	), hop::vec3<T>());

	// Ceiling: z = 10 to 10+wall_thick
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>(-half_size, -half_size, zero),
		hop::vec3<T>( half_size,  half_size, wall_thick)
	), hop::vec3<T>(zero, zero, size));

	// Wall -X: x = -5-thick to -5
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>(-wall_thick, -half_size, zero),
		hop::vec3<T>( zero,        half_size, size)
	), hop::vec3<T>(-half_size, zero, zero));

	// Wall +X: x = 5 to 5+thick
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>( zero,       -half_size, zero),
		hop::vec3<T>( wall_thick,  half_size, size)
	), hop::vec3<T>(half_size, zero, zero));

	// Wall -Y: y = -5-thick to -5
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>(-half_size, -wall_thick, zero),
		hop::vec3<T>( half_size,  zero,       size)
	), hop::vec3<T>(zero, -half_size, zero));

	// Wall +Y: y = 5 to 5+thick
	make_wall(sim, hop::aa_box<T>(
		hop::vec3<T>(-half_size,  zero,       zero),
		hop::vec3<T>( half_size,  wall_thick, size)
	), hop::vec3<T>(zero, half_size, zero));

	T cor = tr::from_milli(800);
	T fric_zero = T{};

	// Dynamic box: 1x1x1, starts at (2, 0, 7)
	auto box_solid = std::make_shared<hop::solid<T>>();
	box_solid->set_mass(tr::one());
	box_solid->set_coefficient_of_restitution(cor);
	box_solid->set_coefficient_of_restitution_override(true);
	box_solid->set_coefficient_of_static_friction(fric_zero);
	box_solid->set_coefficient_of_dynamic_friction(fric_zero);
	box_solid->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(
		hop::vec3<T>(-tr::half(), -tr::half(), -tr::half()),
		hop::vec3<T>( tr::half(),  tr::half(),  tr::half())
	)));
	box_solid->set_position(hop::vec3<T>(tr::from_int(1), zero, tr::from_int(4)));
	box_solid->set_velocity(hop::vec3<T>(tr::from_int(3), tr::from_int(-2), zero));
	sim.add_solid(box_solid);

	// Dynamic sphere: radius 0.5, starts at (-2, 1, 8)
	auto sphere_solid = std::make_shared<hop::solid<T>>();
	sphere_solid->set_mass(tr::one());
	sphere_solid->set_coefficient_of_restitution(cor);
	sphere_solid->set_coefficient_of_restitution_override(true);
	sphere_solid->set_coefficient_of_static_friction(fric_zero);
	sphere_solid->set_coefficient_of_dynamic_friction(fric_zero);
	sphere_solid->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(tr::half())));
	sphere_solid->set_position(hop::vec3<T>(tr::from_int(-1), tr::from_int(1), tr::from_int(5)));
	sphere_solid->set_velocity(hop::vec3<T>(tr::from_int(-1), tr::from_int(3), tr::from_int(2)));
	sim.add_solid(sphere_solid);

	// Dynamic capsule: radius 0.4, length 1.5 along Z, starts at (0, -2, 6)
	auto capsule_solid = std::make_shared<hop::solid<T>>();
	capsule_solid->set_mass(tr::one());
	capsule_solid->set_coefficient_of_restitution(cor);
	capsule_solid->set_coefficient_of_restitution_override(true);
	capsule_solid->set_coefficient_of_static_friction(fric_zero);
	capsule_solid->set_coefficient_of_dynamic_friction(fric_zero);
	hop::capsule<T> cap_shape(
		hop::vec3<T>(),
		hop::vec3<T>(zero, zero, tr::from_milli(1500)),
		tr::from_milli(400)
	);
	capsule_solid->add_shape(std::make_shared<hop::shape<T>>(cap_shape));
	capsule_solid->set_position(hop::vec3<T>(zero, tr::from_int(-1), tr::from_int(3)));
	capsule_solid->set_velocity(hop::vec3<T>(tr::from_int(2), tr::from_int(1), tr::from_int(-3)));
	sim.add_solid(capsule_solid);

	// Raylib window
	int win_w = capture_dir ? 400 : 800;
	int win_h = capture_dir ? 300 : 600;
	float duration = capture_dir ? 6.0f : 10.0f;
	InitWindow(win_w, win_h, "hop physics — bounce room");
	SetTargetFPS(60);

	float cam_angle = 0.0f;
	const char* mode_label = std::is_same_v<T, hop::fixed16> ? "fixed16" : "float";
	int frame_num = 0;

	float elapsed = 0.0f;
	while (!WindowShouldClose() && elapsed < duration) {
		elapsed += GetFrameTime();
		sim.update(16);

		// Orbiting camera looking at room center
		cam_angle += 0.3f * GetFrameTime();
		float cam_dist = 18.0f;
		float cam_height = 8.0f;
		Camera3D camera = {};
		camera.position = {
			cam_dist * cosf(cam_angle),
			cam_height,
			cam_dist * sinf(cam_angle)
		};
		camera.target = {0.0f, 3.0f, 0.0f};  // room center in raylib coords (Y-up)
		camera.up = {0.0f, 1.0f, 0.0f};
		camera.fovy = 50.0f;
		camera.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({30, 30, 40, 255});

		BeginMode3D(camera);

		// Draw room wireframe (raylib Y-up: room is 6x6x6, Y from 0 to 6)
		DrawCubeWires({0.0f, 3.0f, 0.0f}, 6.0f, 6.0f, 6.0f, DARKGRAY);
		// Semi-transparent floor
		DrawCube({0.0f, -0.05f, 0.0f}, 6.0f, 0.1f, 6.0f, {60, 60, 80, 100});

		// Box
		Vector3 bp = to_raylib(box_solid->get_position());
		DrawCube(bp, 1.0f, 1.0f, 1.0f, RED);
		DrawCubeWires(bp, 1.0f, 1.0f, 1.0f, MAROON);

		// Sphere
		Vector3 sp = to_raylib(sphere_solid->get_position());
		DrawSphere(sp, 0.5f, BLUE);
		DrawSphereWires(sp, 0.5f, 8, 8, DARKBLUE);

		// Capsule — origin + direction in hop Z-up, convert both endpoints
		auto& cap_pos = capsule_solid->get_position();
		hop::vec3<T> cap_top = cap_pos;
		cap_top.z = cap_top.z + tr::from_milli(1500);
		Vector3 cp_bot = to_raylib(cap_pos);
		Vector3 cp_top = to_raylib(cap_top);
		DrawCapsule(cp_bot, cp_top, 0.4f, 8, 8, GREEN);
		DrawCapsuleWires(cp_bot, cp_top, 0.4f, 8, 8, DARKGREEN);

		EndMode3D();

		// HUD
		DrawText(mode_label, 10, 10, 20, LIGHTGRAY);
		DrawFPS(10, 40);

		auto& bpos = box_solid->get_position();
		std::string txt = "box:     " + std::to_string(tr::to_float(bpos.z));
		if constexpr (std::is_same_v<T, hop::fixed16>) txt += "  (raw: " + std::to_string(bpos.z.raw) + ")";
		DrawText(txt.c_str(), 10, 70, 16, RED);

		auto& spos = sphere_solid->get_position();
		txt = "sphere:  " + std::to_string(tr::to_float(spos.z));
		if constexpr (std::is_same_v<T, hop::fixed16>) txt += "  (raw: " + std::to_string(spos.z.raw) + ")";
		DrawText(txt.c_str(), 10, 90, 16, BLUE);

		auto& cpos = capsule_solid->get_position();
		txt = "capsule: " + std::to_string(tr::to_float(cpos.z));
		if constexpr (std::is_same_v<T, hop::fixed16>) txt += "  (raw: " + std::to_string(cpos.z.raw) + ")";
		DrawText(txt.c_str(), 10, 110, 16, GREEN);

		EndDrawing();

		if (capture_dir) {
			char path[512];
			snprintf(path, sizeof(path), "%s/frame_%04d.png", capture_dir, frame_num);
			Image img = LoadImageFromScreen();
			ExportImage(img, path);
			UnloadImage(img);
		}
		frame_num++;
	}

	CloseWindow();
}

int main(int argc, char* argv[]) {
	bool use_fixed = false;
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--fixed") == 0) {
			use_fixed = true;
		} else if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc) {
			capture_dir = argv[++i];
		}
	}

	if (use_fixed) {
		run<hop::fixed16>();
	} else {
		run<float>();
	}
	return 0;
}
