// demo_rotating_platform.cpp — Raylib visualization of hop's KINEMATIC ANGULAR
// CARRY (Phase 6): a spinning platform drags the bodies resting on it, the way a
// GoldSrc func_rotating carries its riders.
//
// A large infinite-mass platform spins about the vertical (Z) axis. Its orientation
// is advanced by hand each frame (a scripted kinematic mover — hop does not integrate
// orientation for infinite-mass bodies) and its angular_velocity is published so the
// contact solver biases each rider's relative velocity by ω×r at the contact point.
// The dynamic riders sitting on top are therefore carried in a circle by friction —
// no rider-side rotation code, just the surface-velocity bias. The riders also have
// finite inertia, so the same friction that drags them also spins them up (Phase 9),
// and an off-center stack tumbles (a fun secondary effect).
//
// Pass --fixed to use fixed16 arithmetic instead of float.

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>
#include <hop/hop.h>
#include <raylib.h>

// Hop is Z-up, raylib is Y-up — swap Y and Z.
template <typename T> static Vector3 to_raylib(const hop::vec3<T> & v) {
	using tr = hop::scalar_traits<T>;
	return { tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y) };
}

// Draw a box by its 8 orientation-transformed world corners (translucent faces both
// windings + bright wire edges) — reads the tilt off the real transformed geometry.
template <typename T>
static void draw_oriented_box(const hop::vec3<T> & pos, const hop::mat3<T> & R,
                              const hop::vec3<T> & half, Color face, Color wire) {
	Vector3 c[8];
	for (int i = 0; i < 8; ++i) {
		hop::vec3<T> local((i & 1) ? half.x : -half.x,
		                   (i & 2) ? half.y : -half.y,
		                   (i & 4) ? half.z : -half.z);
		hop::vec3<T> w;
		hop::mul(w, R, local);
		hop::add(w, pos);
		c[i] = to_raylib(w);
	}
	static const int faces[6][4] = {
		{ 0, 1, 3, 2 }, { 4, 6, 7, 5 },
		{ 0, 4, 5, 1 }, { 2, 3, 7, 6 },
		{ 0, 2, 6, 4 }, { 1, 5, 7, 3 },
	};
	for (auto & f : faces) {
		DrawTriangle3D(c[f[0]], c[f[1]], c[f[2]], face);
		DrawTriangle3D(c[f[0]], c[f[2]], c[f[3]], face);
		DrawTriangle3D(c[f[0]], c[f[2]], c[f[1]], face);
		DrawTriangle3D(c[f[0]], c[f[3]], c[f[2]], face);
	}
	for (int i = 0; i < 8; ++i)
		for (int b = 1; b < 8; b <<= 1)
			if (i < (i ^ b))
				DrawLine3D(c[i], c[i ^ b], wire);
}

template <typename T> static void run(bool fixed_label) {
	using tr = hop::scalar_traits<T>;
	const T zero = T {};

	hop::simulator<T> sim; // default gravity -Z

	// The spinning platform: a wide, shallow infinite-mass box, top face at z=0.
	// High friction so it grips its riders and carries them around.
	const hop::vec3<T> plat_half(tr::from_int(5), tr::from_int(5), tr::half());
	auto platform = std::make_shared<hop::solid<T>>();
	platform->set_infinite_mass();
	platform->set_coefficient_of_gravity(zero);
	platform->set_coefficient_of_static_friction(tr::one());
	platform->set_coefficient_of_dynamic_friction(tr::one());
	platform->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(
	    hop::vec3<T>(-plat_half.x, -plat_half.y, -plat_half.z),
	    hop::vec3<T>(plat_half.x, plat_half.y, plat_half.z))));
	platform->set_position(hop::vec3<T>(zero, zero, -tr::half())); // top face at z=0
	sim.add_solid(platform);

	const T spin_rate = tr::from_milli(700); // rad/s about +Z (~40°/s)

	const hop::vec3<T> rider_half(tr::half(), tr::half(), tr::half());
	auto unit_box = [&]() {
		return std::make_shared<hop::shape<T>>(hop::aa_box<T>(
		    hop::vec3<T>(-rider_half.x, -rider_half.y, -rider_half.z),
		    hop::vec3<T>(rider_half.x, rider_half.y, rider_half.z)));
	};
	// A dynamic rider resting on the platform at world (x,y). Finite inertia → it
	// also spins as friction drags it; high friction so the carry actually grips.
	auto add_rider = [&](T x, T y) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_inertia(hop::vec3<T>(tr::from_milli(167), tr::from_milli(167), tr::from_milli(167)));
		s->set_coefficient_of_restitution(tr::from_milli(50));
		s->set_coefficient_of_static_friction(tr::one());
		s->set_coefficient_of_dynamic_friction(tr::one());
		s->add_shape(unit_box());
		s->set_position(hop::vec3<T>(x, y, tr::half() + tr::from_milli(20)));
		sim.add_solid(s);
		return s;
	};

	// Riders at a few radii — the outer ones orbit faster (ω×r grows with r).
	std::vector<std::shared_ptr<hop::solid<T>>> riders;
	riders.push_back(add_rider(tr::from_int(3), zero));
	riders.push_back(add_rider(-tr::from_int(3), zero));
	riders.push_back(add_rider(zero, tr::from_int(2)));
	riders.push_back(add_rider(-tr::from_milli(1500), -tr::from_milli(1500)));

	InitWindow(900, 650, "hop physics — Phase 6 kinematic angular carry");
	SetTargetFPS(60);
	const T dt = tr::from_milli(16);
	float cam_angle = 0.6f;
	T plat_angle = zero; // accumulated platform yaw (advanced at the sim timestep)

	while (!WindowShouldClose()) {
		// Drive the platform like a scripted kinematic mover: advance its orientation
		// by ω·dt and publish the matching angular velocity so the solver carries the
		// riders (the ω×r surface-velocity bias). Stepping the angle at the same dt the
		// solver sees keeps the published ω consistent with the orientation delta.
		plat_angle = plat_angle + spin_rate * dt;
		hop::mat3<T> R;
		hop::set_mat3_from_axis_angle(R, hop::vec3<T>(zero, zero, tr::one()), plat_angle);
		platform->set_orientation(R);
		platform->set_angular_velocity(hop::vec3<T>(zero, zero, spin_rate));

		sim.update(dt);

		cam_angle += 0.12f * GetFrameTime();
		Camera3D camera = {};
		camera.position = { 12.0f * cosf(cam_angle), 9.0f, 12.0f * sinf(cam_angle) };
		camera.target = { 0.0f, 0.0f, 0.0f };
		camera.up = { 0.0f, 1.0f, 0.0f };
		camera.fovy = 50.0f;
		camera.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 28, 28, 38, 255 });
		BeginMode3D(camera);

		draw_oriented_box(platform->get_position(), platform->get_orientation(), plat_half,
		                  { 70, 80, 110, 70 }, (Color){ 120, 140, 200, 255 });
		Color rider_face[4] = {
			{ 220, 70, 70, 150 }, { 70, 200, 90, 150 }, { 230, 200, 70, 150 }, { 200, 110, 220, 150 }
		};
		Color rider_wire[4] = { RED, GREEN, GOLD, PURPLE };
		for (size_t i = 0; i < riders.size(); ++i)
			draw_oriented_box(riders[i]->get_position(), riders[i]->get_orientation(), rider_half,
			                  rider_face[i % 4], rider_wire[i % 4]);

		EndMode3D();

		DrawText("Phase 6 — kinematic angular carry (a spinning platform carries its riders)", 14, 12, 20, RAYWHITE);
		DrawText(TextFormat("scalar: %s   |   platform spins about +Z; riders dragged around by friction (omega x r)",
		                    fixed_label ? "fixed16" : "float"),
		         14, 38, 16, LIGHTGRAY);
		DrawText("riders also have inertia, so the same friction spins them up (Phase 9)", 14, 58, 16,
		         (Color){ 180, 200, 180, 255 });
		EndDrawing();
	}
	CloseWindow();
}

int main(int argc, char ** argv) {
	bool fixed = false;
	for (int i = 1; i < argc; ++i)
		if (std::strcmp(argv[i], "--fixed") == 0)
			fixed = true;
	if (fixed)
		run<hop::fixed16>(true);
	else
		run<float>(false);
	return 0;
}
