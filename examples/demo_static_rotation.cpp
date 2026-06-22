// demo_static_rotation.cpp — Raylib visualization of hop's STATIC orientation
// (Phase 5: oriented polytope narrowphase).
//
// Three 1x1x1 boxes are dropped onto a flat floor:
//   * GRAY  — axis-aligned control. Rests flat, lowest face at z=0, center z=0.5.
//   * RED   — rotated 45° about the up axis-in-plane (its X-Z cross-section is a
//             diamond). With Phase 5 it collides on its true rotated faces and
//             comes to rest BALANCED ON ITS EDGE, center at z=√2/2≈0.707 — NOT
//             sunk to the axis-aligned 0.5 it would hit if the box collided as
//             its world AABB (the pre-Phase-5 behavior).
//   * BLUE  — rotated about a tilted axis so it settles on a single corner/edge.
//
// What this demo does NOT show: dynamic tumbling. The orientation is STATIC — set
// once, never integrated — because angular integration (Phase 8) and angular
// impulse response (Phase 9) are not implemented yet. The diamond therefore rests
// perfectly balanced on its edge instead of toppling: there is no torque to tip
// it. Phase 5 makes the *contact* correct, not the spin.
//
// Pass --fixed to use fixed16 arithmetic instead of float.

#include <cmath>
#include <cstring>
#include <hop/hop.h>
#include <raylib.h>

// Hop is Z-up, raylib is Y-up — swap Y and Z.
template <typename T> static Vector3 to_raylib(const hop::vec3<T> & v) {
	using tr = hop::scalar_traits<T>;
	return { tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y) };
}

// Draw a box by its 8 orientation-transformed world corners: faces (translucent,
// both windings so nothing culls) + bright wire edges. Reading the tilt off the
// real transformed geometry avoids any matrix-stack / axis-swap ambiguity.
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
	// 6 faces as corner-index quads (consistent winding); drawn both ways so the
	// translucent fill shows from either side.
	static const int faces[6][4] = {
		{ 0, 1, 3, 2 }, { 4, 6, 7, 5 }, // -z, +z
		{ 0, 4, 5, 1 }, { 2, 3, 7, 6 }, // -y, +y
		{ 0, 2, 6, 4 }, { 1, 5, 7, 3 }, // -x, +x
	};
	for (auto & f : faces) {
		DrawTriangle3D(c[f[0]], c[f[1]], c[f[2]], face);
		DrawTriangle3D(c[f[0]], c[f[2]], c[f[3]], face);
		DrawTriangle3D(c[f[0]], c[f[2]], c[f[1]], face);
		DrawTriangle3D(c[f[0]], c[f[3]], c[f[2]], face);
	}
	// 12 edges: corner pairs differing in exactly one bit.
	for (int i = 0; i < 8; ++i)
		for (int b = 1; b < 8; b <<= 1)
			if (i < (i ^ b))
				DrawLine3D(c[i], c[i ^ b], wire);
}

template <typename T> static void run(bool fixed_label) {
	using tr = hop::scalar_traits<T>;
	const T zero = T {};

	hop::simulator<T> sim; // default gravity -Z

	// Floor: an infinite-mass box, top face at z=0.
	auto floor = std::make_shared<hop::solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(zero);
	floor->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(
	    hop::vec3<T>(-tr::from_int(6), -tr::from_int(6), -tr::one()),
	    hop::vec3<T>(tr::from_int(6), tr::from_int(6), zero))));
	floor->set_position(hop::vec3<T>());
	sim.add_solid(floor);

	const hop::vec3<T> half(tr::half(), tr::half(), tr::half());
	auto unit_box = [&]() {
		return std::make_shared<hop::shape<T>>(hop::aa_box<T>(
		    hop::vec3<T>(-tr::half(), -tr::half(), -tr::half()),
		    hop::vec3<T>(tr::half(), tr::half(), tr::half())));
	};
	auto drop = [&](const hop::vec3<T> & pos, const hop::mat3<T> & R) {
		auto s = std::make_shared<hop::solid<T>>();
		s->set_mass(tr::one());
		s->set_coefficient_of_restitution(tr::from_milli(50)); // settle, don't bounce
		s->set_coefficient_of_static_friction(tr::from_milli(800));
		s->set_coefficient_of_dynamic_friction(tr::from_milli(800));
		s->add_shape(unit_box());
		s->set_orientation(R);
		s->set_position(pos);
		sim.add_solid(s);
		return s;
	};

	hop::mat3<T> identity;
	hop::mat3<T> diamond, corner;
	// 45° about Y (hop up-in-plane is Z; Y rotation tilts the X-Z cross-section
	// into a diamond resting on its edge).
	hop::set_mat3_from_axis_angle(diamond, hop::vec3<T>(zero, tr::one(), zero),
	                              tr::from_milli(785)); // ~π/4
	// Tilt about a diagonal axis so it lands on a corner/edge.
	hop::vec3<T> diag(tr::one(), tr::one(), zero);
	hop::normalize_carefully(diag, tr::from_milli(1));
	hop::set_mat3_from_axis_angle(corner, diag, tr::from_milli(700));

	auto ctrl = drop(hop::vec3<T>(-tr::from_int(2), zero, tr::from_int(3)), identity);
	auto dia = drop(hop::vec3<T>(zero, zero, tr::from_int(3)), diamond);
	auto cor = drop(hop::vec3<T>(tr::from_int(2), zero, tr::from_int(3)), corner);

	// Phase 8: a freely-spinning box. Finite (asymmetric) inertia + an initial ω
	// about a tilted axis → hop integrates its orientation and it tumbles in place.
	// Gravity off and no collision (Phase 9 owns response), so it just spins.
	auto spinner = std::make_shared<hop::solid<T>>();
	spinner->set_mass(tr::one());
	spinner->set_inertia(hop::vec3<T>(tr::one(), tr::two(), tr::from_int(3))); // asymmetric → visible wobble
	spinner->set_coefficient_of_gravity(zero);
	spinner->set_collide_with_scope(0);
	spinner->add_shape(unit_box());
	spinner->set_position(hop::vec3<T>(zero, zero, tr::from_int(4))); // floats above the settled boxes
	spinner->set_angular_velocity(hop::vec3<T>(tr::one(), tr::two(), tr::half())); // tilted axis
	sim.add_solid(spinner);

	InitWindow(900, 650, "hop physics — rotation (Phase 5 static + Phase 8 dynamic)");
	SetTargetFPS(60);
	float cam_angle = 0.5f;

	while (!WindowShouldClose()) {
		sim.update(tr::from_milli(16));

		cam_angle += 0.25f * GetFrameTime();
		Camera3D camera = {};
		camera.position = { 10.0f * cosf(cam_angle), 5.5f, 10.0f * sinf(cam_angle) };
		camera.target = { 0.0f, 0.7f, 0.0f };
		camera.up = { 0.0f, 1.0f, 0.0f };
		camera.fovy = 50.0f;
		camera.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 28, 28, 38, 255 });
		BeginMode3D(camera);

		// Floor tile + a faint grid.
		DrawCube({ 0.0f, -0.025f, 0.0f }, 12.0f, 0.05f, 12.0f, { 55, 55, 75, 255 });
		DrawGrid(12, 1.0f);

		draw_oriented_box(ctrl->get_position(), ctrl->get_orientation(), half,
		                  { 130, 130, 140, 90 }, GRAY);
		draw_oriented_box(dia->get_position(), dia->get_orientation(), half,
		                  { 220, 70, 70, 90 }, RED);
		draw_oriented_box(cor->get_position(), cor->get_orientation(), half,
		                  { 70, 120, 220, 90 }, BLUE);
		// Phase 8: the freely-spinning box — its orientation is integrated each step.
		draw_oriented_box(spinner->get_position(), spinner->get_orientation(), half,
		                  { 230, 200, 70, 110 }, GOLD);

		EndMode3D();

		DrawText("Phase 5 (static orientation in collision) + Phase 8 (dynamic spin)", 14, 12, 20, RAYWHITE);
		DrawText(TextFormat("scalar: %s   |   gray=axis-aligned (rests z=0.50)", fixed_label ? "fixed16" : "float"),
		         14, 38, 16, LIGHTGRAY);
		DrawText(TextFormat("red 45 deg balances on edge: center z=%.3f (AABB would be 0.500)",
		                    (double)hop::scalar_traits<T>::to_float(dia->get_position().z)),
		         14, 58, 16, (Color){ 235, 120, 120, 255 });
		DrawText("gold box: free spin under angular integration (no collision response yet - Phase 9)",
		         14, 78, 16, (Color){ 220, 190, 90, 255 });
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
