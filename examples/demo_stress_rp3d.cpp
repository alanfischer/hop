// demo_stress_rp3d.cpp — ReactPhysics3D port of demo_stress.cpp
//
// Same scene: a 12 x 6 x 6 room packed with ~700+ spheres bouncing with no
// gravity. RP3D handles broad-phase + narrow-phase + integration internally.
// Press Space to pause.
//
// HUD shows the physics-step wall time so you can compare against hop directly.
// Both demos run a fixed 16 ms world step at 60 Hz by default.

// reactphysics3d MUST come before raylib — raylib's RED/GREEN/etc. color macros
// otherwise collide with rp::DebugRenderer::DebugColor enumerators.
#include <reactphysics3d/reactphysics3d.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <raylib.h>
#include <vector>

namespace rp = reactphysics3d;

static Vector3 to_rl(const rp::Vector3 & v) { return { v.x, v.y, v.z }; }

static Color heat_color(float speed) {
	float t = std::min(speed / 12.0f, 1.0f);
	if (t < 0.5f) {
		float s = t * 2.0f;
		return { 0, (unsigned char)(80 + 175 * s), (unsigned char)(255 - 200 * s), 255 };
	}
	float s = (t - 0.5f) * 2.0f;
	return { (unsigned char)(255 * s), (unsigned char)(255 * (1.0f - s)), 0, 255 };
}

static rp::RigidBody * make_wall(rp::PhysicsCommon & common, rp::PhysicsWorld * world,
                                 const rp::Vector3 & half_extents, const rp::Vector3 & position) {
	rp::Transform t(position, rp::Quaternion::identity());
	rp::RigidBody * body = world->createRigidBody(t);
	body->setType(rp::BodyType::STATIC);
	auto * shape = common.createBoxShape(half_extents);
	auto * col = body->addCollider(shape, rp::Transform::identity());
	col->getMaterial().setBounciness(1.0f);
	col->getMaterial().setFrictionCoefficient(0.0f);
	return body;
}

static void run() {
	constexpr int ROOM_HALF = 6;       // half-extent in X and Z (raylib Y-up)
	constexpr int ROOM_HEIGHT = 12;    // along Y
	constexpr float SPHERE_R = 0.28f;
	constexpr float SPACING = SPHERE_R * 2.2f;
	constexpr int COLS = (int)((ROOM_HALF * 2.0f) / SPACING);
	constexpr int Y_LAYERS = (int)(ROOM_HEIGHT / SPACING);
	constexpr int COUNT = COLS * COLS * Y_LAYERS;

	rp::PhysicsCommon common;
	rp::PhysicsWorld::WorldSettings settings;
	settings.isSleepingEnabled = true;
	rp::PhysicsWorld * world = common.createPhysicsWorld(settings);

	// Floor / ceiling along Y (Y-up world). Outer extents larger than the side
	// walls — same trick the hop demo uses to avoid corner-edge ambiguity.
	const float thick = 1.0f;
	const float outer = ROOM_HALF + thick;
	make_wall(common, world,
	          rp::Vector3(outer, thick * 0.5f, outer),
	          rp::Vector3(0, -thick * 0.5f, 0)); // floor
	make_wall(common, world,
	          rp::Vector3(outer, thick * 0.5f, outer),
	          rp::Vector3(0, ROOM_HEIGHT + thick * 0.5f, 0)); // ceiling
	make_wall(common, world,
	          rp::Vector3(thick * 0.5f, ROOM_HEIGHT * 0.5f, ROOM_HALF),
	          rp::Vector3(-ROOM_HALF - thick * 0.5f, ROOM_HEIGHT * 0.5f, 0)); // -X
	make_wall(common, world,
	          rp::Vector3(thick * 0.5f, ROOM_HEIGHT * 0.5f, ROOM_HALF),
	          rp::Vector3(ROOM_HALF + thick * 0.5f, ROOM_HEIGHT * 0.5f, 0)); // +X
	make_wall(common, world,
	          rp::Vector3(ROOM_HALF, ROOM_HEIGHT * 0.5f, thick * 0.5f),
	          rp::Vector3(0, ROOM_HEIGHT * 0.5f, -ROOM_HALF - thick * 0.5f)); // -Z
	make_wall(common, world,
	          rp::Vector3(ROOM_HALF, ROOM_HEIGHT * 0.5f, thick * 0.5f),
	          rp::Vector3(0, ROOM_HEIGHT * 0.5f, ROOM_HALF + thick * 0.5f)); // +Z

	// Shared sphere shape.
	rp::SphereShape * sphere_shape = common.createSphereShape(SPHERE_R);

	std::vector<rp::RigidBody *> spheres;
	spheres.reserve(COUNT);
	for (int i = 0; i < COUNT; ++i) {
		int layer = i / (COLS * COLS);  // along Y (up)
		int rem = i % (COLS * COLS);
		int col = rem % COLS;
		int row = rem / COLS;
		float x = -(ROOM_HALF - SPACING * 0.5f) + col * SPACING;
		float z = -(ROOM_HALF - SPACING * 0.5f) + row * SPACING;
		float y = SPACING * 0.5f + layer * SPACING;
		float vx = ((i * 7 + 3) % 21 - 10) * 0.5f;
		float vz = ((i * 13 + 5) % 21 - 10) * 0.5f;
		float vy = ((i * 17 + 9) % 21 - 10) * 0.5f;

		rp::Transform t(rp::Vector3(x, y, z), rp::Quaternion::identity());
		auto * body = world->createRigidBody(t);
		body->setMass(1.0f);
		body->setLinearDamping(0.0f);
		body->setAngularDamping(0.0f);
		auto * collider = body->addCollider(sphere_shape, rp::Transform::identity());
		collider->getMaterial().setBounciness(0.75f);
		collider->getMaterial().setFrictionCoefficient(0.0f);
		body->setLinearVelocity(rp::Vector3(vx, vy, vz));
		spheres.push_back(body);
	}

	InitWindow(1280, 720, "reactphysics3d — stress test");
	SetTargetFPS(120);

	bool paused = false;
	float cam_angle = 0.2f;
	float phys_ms = 0.0f;
	const float fixed_dt = 1.0f / 60.0f;

	while (!WindowShouldClose()) {
		float dt = GetFrameTime();
		if (dt > 0.1f) dt = 0.1f;

		if (IsKeyPressed(KEY_SPACE)) paused = !paused;

		if (!paused) {
			auto t0 = std::chrono::high_resolution_clock::now();
			world->update(fixed_dt);
			auto t1 = std::chrono::high_resolution_clock::now();
			phys_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
			cam_angle += 0.15f * dt;
		}

		Camera3D cam = {};
		cam.position = { 30.0f * cosf(cam_angle), 10.0f, 30.0f * sinf(cam_angle) };
		cam.target = { 0.0f, (float)ROOM_HEIGHT * 0.5f, 0.0f };
		cam.up = { 0.0f, 1.0f, 0.0f };
		cam.fovy = 55.0f;
		cam.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 12, 12, 20, 255 });
		BeginMode3D(cam);

		const float W = (float)(ROOM_HALF * 2);
		const float H = (float)ROOM_HEIGHT;
		DrawCubeWires({ 0.0f, H * 0.5f, 0.0f }, W, H, W, { 50, 50, 80, 255 });
		DrawCube({ 0.0f, -0.02f, 0.0f }, W, 0.04f, W, { 30, 30, 60, 220 });

		int sleeping = 0;
		for (auto * s : spheres) {
			Vector3 p = to_rl(s->getTransform().getPosition());
			float speed = s->getLinearVelocity().length();
			bool asleep = s->isSleeping();
			if (asleep) ++sleeping;
			Color color = asleep ? GRAY : heat_color(speed);
			DrawSphereEx(p, SPHERE_R, 4, 6, color);
		}

		EndMode3D();

		int fps = GetFPS();
		Color fps_color = fps >= 50 ? GREEN : fps >= 30 ? YELLOW : RED;
		DrawText(TextFormat("FPS:     %d", fps), 12, 12, 22, fps_color);
		DrawText(TextFormat("Objects: %d  (sleep: %d)", COUNT, sleeping), 12, 38, 22, WHITE);
		Color pc = phys_ms < 8.0f ? GREEN : phys_ms < 16.0f ? YELLOW : RED;
		DrawText(TextFormat("Physics: %.1f ms", (double)phys_ms), 12, 64, 22, pc);
		DrawText("reactphysics3d", 12, 90, 18, { 180, 180, 220, 255 });

		if (paused) DrawText("PAUSED", 560, 330, 48, { 255, 220, 60, 220 });
		DrawText("Space : pause", 12, 700, 16, { 90, 90, 120, 255 });

		EndDrawing();
	}

	CloseWindow();
	common.destroyPhysicsWorld(world);
}

int main() {
	run();
	return 0;
}
