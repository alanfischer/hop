// demo_bounce_rp3d.cpp — ReactPhysics3D port of demo_bounce.cpp
//
// Same scene, same visualization — only the physics engine changes. Useful for
// a side-by-side feel/perf comparison against hop's demo_bounce.
//
// Differences vs hop's demo:
//   - ReactPhysics3D is Y-up; the original hop demo is Z-up. We swap axes when
//     placing bodies so the rendered scene is identical (raylib is also Y-up,
//     so to_rl is now the identity).
//   - RP3D has no native spring or rope/distance constraint. We apply Hooke's
//     law forces each frame for springs, and a strong one-sided spring for
//     ropes (force only when stretched past rest length). That's the idiomatic
//     way to add these in RP3D.
//   - RP3D's trigger callback (EventListener::onTrigger) replaces hop's
//     trace_solid trigger probe.

// reactphysics3d MUST be included before raylib — raylib defines RED/GREEN/etc.
// as macros, which collide with rp::DebugRenderer::DebugColor enumerators.
#include <reactphysics3d/reactphysics3d.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <raylib.h>
#include <rlgl.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace rp = reactphysics3d;

static Vector3 to_rl(const rp::Vector3 & v) { return { v.x, v.y, v.z }; }

// Hop scene is Z-up; RP3D scene is Y-up. Mirror a hop (x, y, z) position
// onto RP3D world (x, z, y) so the rendered layout matches the original.
static rp::Vector3 zup(float x, float y, float z) { return rp::Vector3(x, z, y); }

// Spark particle (identical to demo_bounce.cpp).
struct spark {
	Vector3 pos;
	Vector3 vel;
	float life;
	float max_life;
};

static std::vector<spark> sparks;

static void spawn_sparks(Vector3 pos, int count = 8) {
	for (int i = 0; i < count; ++i) {
		float angle = (float)i / count * 2.0f * PI + (float)GetRandomValue(0, 100) / 100.0f * 0.5f;
		float pitch = (float)GetRandomValue(-60, 60) * DEG2RAD;
		float speed = 1.5f + (float)GetRandomValue(0, 100) / 50.0f;
		spark s;
		s.pos = pos;
		s.vel = {
			cosf(pitch) * cosf(angle) * speed,
			sinf(pitch) * speed,
			cosf(pitch) * sinf(angle) * speed,
		};
		s.life = 0.3f + (float)GetRandomValue(0, 100) / 400.0f;
		s.max_life = s.life;
		sparks.push_back(s);
	}
}

static void update_and_draw_sparks(float dt) {
	for (int i = (int)sparks.size() - 1; i >= 0; --i) {
		auto & s = sparks[i];
		s.pos.x += s.vel.x * dt;
		s.pos.y += s.vel.y * dt;
		s.pos.z += s.vel.z * dt;
		s.vel.y -= 9.8f * dt;
		s.life -= dt;
		if (s.life <= 0.0f) {
			sparks[i] = sparks.back();
			sparks.pop_back();
			continue;
		}
		float t = s.life / s.max_life;
		unsigned char alpha = (unsigned char)(255 * t);
		unsigned char r = 255;
		unsigned char g = (unsigned char)(200 * t + 55);
		unsigned char b = (unsigned char)(50 * t);
		float radius = 0.04f * t;
		DrawSphere(s.pos, radius, { r, g, b, alpha });
	}
}

// Octahedron draw — identical to demo_bounce.cpp.
static void draw_octahedron(Vector3 p, float r, Color fill, Color wire) {
	Vector3 v[6] = {
		{ p.x + r, p.y,     p.z     },
		{ p.x - r, p.y,     p.z     },
		{ p.x,     p.y + r, p.z     },
		{ p.x,     p.y - r, p.z     },
		{ p.x,     p.y,     p.z + r },
		{ p.x,     p.y,     p.z - r },
	};
	rlDisableBackfaceCulling();
	DrawTriangle3D(v[2], v[0], v[4], fill);
	DrawTriangle3D(v[2], v[4], v[1], fill);
	DrawTriangle3D(v[2], v[1], v[5], fill);
	DrawTriangle3D(v[2], v[5], v[0], fill);
	DrawTriangle3D(v[3], v[4], v[0], fill);
	DrawTriangle3D(v[3], v[1], v[4], fill);
	DrawTriangle3D(v[3], v[5], v[1], fill);
	DrawTriangle3D(v[3], v[0], v[5], fill);
	rlEnableBackfaceCulling();
	DrawLine3D(v[2], v[0], wire); DrawLine3D(v[2], v[1], wire);
	DrawLine3D(v[2], v[4], wire); DrawLine3D(v[2], v[5], wire);
	DrawLine3D(v[3], v[0], wire); DrawLine3D(v[3], v[1], wire);
	DrawLine3D(v[3], v[4], wire); DrawLine3D(v[3], v[5], wire);
	DrawLine3D(v[0], v[4], wire); DrawLine3D(v[4], v[1], wire);
	DrawLine3D(v[1], v[5], wire); DrawLine3D(v[5], v[0], wire);
}

static void draw_constraint_line(Vector3 a, Vector3 b, float rest_length, bool is_spring) {
	float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
	float len = sqrtf(dx * dx + dy * dy + dz * dz);
	float stretch = (rest_length > 0.0f) ? (len / rest_length) : 1.0f;
	float tension = stretch <= 1.0f ? 0.0f : fminf((stretch - 1.0f) * 1.5f, 1.0f);
	Color c = {
		(unsigned char)(180 + 75 * tension),
		(unsigned char)(180 * (1.0f - tension)),
		(unsigned char)(180 * (1.0f - tension)),
		220,
	};
	if (!is_spring) {
		DrawLine3D(a, b, c);
		return;
	}
	const int segs = 14;
	Vector3 axis = { dx / fmaxf(len, 1e-4f), dy / fmaxf(len, 1e-4f), dz / fmaxf(len, 1e-4f) };
	Vector3 up = fabsf(axis.y) < 0.9f ? Vector3{ 0, 1, 0 } : Vector3{ 1, 0, 0 };
	Vector3 perp = {
		axis.y * up.z - axis.z * up.y,
		axis.z * up.x - axis.x * up.z,
		axis.x * up.y - axis.y * up.x,
	};
	float pl = sqrtf(perp.x * perp.x + perp.y * perp.y + perp.z * perp.z);
	if (pl > 1e-4f) {
		perp.x /= pl; perp.y /= pl; perp.z /= pl;
	}
	float amp = 0.08f;
	Vector3 prev = a;
	for (int i = 1; i <= segs; ++i) {
		float t = (float)i / segs;
		float side = ((i & 1) == 0) ? -amp : amp;
		float pinch = (i == segs) ? 0.0f : 4.0f * t * (1.0f - t);
		Vector3 pt = {
			a.x + dx * t + perp.x * side * pinch,
			a.y + dy * t + perp.y * side * pinch,
			a.z + dz * t + perp.z * side * pinch,
		};
		DrawLine3D(prev, pt, c);
		prev = pt;
	}
}

static const char * capture_dir = nullptr;

// Spring/rope spec applied each step by user code.
struct spring_spec {
	rp::RigidBody * a;            // body A (always set)
	rp::RigidBody * b;            // body B; nullptr → anchored to world
	rp::Vector3 anchor;           // world anchor used when b == nullptr
	rp::Vector3 local_a;          // attachment point in A's local frame
	float rest_length;
	float k;
	bool is_rope;                 // rope: force applied only when stretched past rest_length
};

static rp::Vector3 body_world_point(rp::RigidBody * body, const rp::Vector3 & local) {
	return body->getTransform() * local;
}

static void apply_springs(const std::vector<spring_spec> & springs) {
	for (const auto & s : springs) {
		rp::Vector3 pa = body_world_point(s.a, s.local_a);
		rp::Vector3 pb = s.b ? s.b->getTransform().getPosition() : s.anchor;
		rp::Vector3 d = pb - pa;
		float len = d.length();
		if (len < 1e-6f) continue;
		float displacement = len - s.rest_length;
		if (s.is_rope && displacement <= 0.0f) continue;
		rp::Vector3 force = (d / len) * (s.k * displacement);
		s.a->applyWorldForceAtWorldPosition(force, pa);
		if (s.b) {
			s.b->applyWorldForceAtWorldPosition(-force, s.b->getTransform().getPosition());
		}
	}
}

// Event listener: tracks which bodies are inside the trigger zone and emits
// sparks on contact start.
class scene_events : public rp::EventListener {
public:
	std::unordered_set<uint32_t> in_zone;
	rp::Entity zone_entity{ rp::Entity(0, 0) };

	bool is_in_zone(rp::RigidBody * body) const {
		return in_zone.find(static_cast<uint32_t>(body->getEntity().id)) != in_zone.end();
	}

	void onTrigger(const rp::OverlapCallback::CallbackData & data) override {
		for (uint32_t i = 0; i < data.getNbOverlappingPairs(); ++i) {
			auto pair = data.getOverlappingPair(i);
			auto * b1 = pair.getBody1();
			auto * b2 = pair.getBody2();
			rp::RigidBody * other = nullptr;
			if (b1->getEntity() == zone_entity) other = static_cast<rp::RigidBody *>(b2);
			else if (b2->getEntity() == zone_entity) other = static_cast<rp::RigidBody *>(b1);
			else continue;
			auto type = pair.getEventType();
			if (type == rp::OverlapCallback::OverlapPair::EventType::OverlapStart ||
			    type == rp::OverlapCallback::OverlapPair::EventType::OverlapStay) {
				in_zone.insert(static_cast<uint32_t>(other->getEntity().id));
			} else if (type == rp::OverlapCallback::OverlapPair::EventType::OverlapExit) {
				in_zone.erase(static_cast<uint32_t>(other->getEntity().id));
			}
		}
	}

	void onContact(const rp::CollisionCallback::CallbackData & data) override {
		for (uint32_t p = 0; p < data.getNbContactPairs(); ++p) {
			auto pair = data.getContactPair(p);
			if (pair.getEventType() != rp::CollisionCallback::ContactPair::EventType::ContactStart) continue;
			for (uint32_t c = 0; c < pair.getNbContactPoints(); ++c) {
				auto cp = pair.getContactPoint(c);
				// Approximate world impact point: transform local point on collider 1.
				auto * collider = pair.getCollider1();
				rp::Vector3 world_pt = collider->getLocalToWorldTransform() * cp.getLocalPointOnCollider1();
				// Speed-based spark count: use combined linear-velocity magnitude.
				// 0.10.2 only has RigidBody; safe to static_cast unconditionally.
				auto * rb1 = static_cast<rp::RigidBody *>(pair.getBody1());
				auto * rb2 = static_cast<rp::RigidBody *>(pair.getBody2());
				rp::Vector3 v {};
				if (rb1->getType() == rp::BodyType::DYNAMIC) v = v + rb1->getLinearVelocity();
				if (rb2->getType() == rp::BodyType::DYNAMIC) v = v - rb2->getLinearVelocity();
				float speed = v.length();
				int count = 4 + (int)(speed * 1.5f);
				if (count > 16) count = 16;
				spawn_sparks(to_rl(world_pt), count);
			}
		}
	}
};

// Octahedron convex mesh, vertex distance r from center.
static rp::ConvexMesh * make_octahedron_mesh(rp::PhysicsCommon & common, float r) {
	static float verts[6 * 3];
	verts[0] = r;  verts[1] = 0;  verts[2] = 0;     // +x
	verts[3] = -r; verts[4] = 0;  verts[5] = 0;     // -x
	verts[6] = 0;  verts[7] = r;  verts[8] = 0;     // +y
	verts[9] = 0;  verts[10] = -r; verts[11] = 0;   // -y
	verts[12] = 0; verts[13] = 0; verts[14] = r;    // +z
	verts[15] = 0; verts[16] = 0; verts[17] = -r;   // -z

	// 8 triangular faces, CCW outward winding (viewed from outside).
	static int indices[8 * 3] = {
		2, 4, 0,
		2, 1, 4,
		2, 5, 1,
		2, 0, 5,
		3, 0, 4,
		3, 4, 1,
		3, 1, 5,
		3, 5, 0,
	};
	static rp::PolygonVertexArray::PolygonFace faces[8];
	for (int i = 0; i < 8; ++i) {
		faces[i].nbVertices = 3;
		faces[i].indexBase = i * 3;
	}
	static rp::PolygonVertexArray polygon_array(
	    6, verts, sizeof(float) * 3,
	    indices, sizeof(int),
	    8, faces,
	    rp::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
	    rp::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
	std::vector<rp::Message> messages;
	rp::ConvexMesh * mesh = common.createConvexMesh(polygon_array, messages);
	for (const auto & m : messages) {
		const char * label = m.type == rp::Message::Type::Error ? "error"
		                   : m.type == rp::Message::Type::Warning ? "warning" : "info";
		std::fprintf(stderr, "rp3d convex mesh %s: %s\n", label, m.text.c_str());
	}
	return mesh;
}

static rp::RigidBody * make_wall(rp::PhysicsCommon & common, rp::PhysicsWorld * world,
                                 const rp::Vector3 & half_extents, const rp::Vector3 & position) {
	rp::Transform t(position, rp::Quaternion::identity());
	rp::RigidBody * body = world->createRigidBody(t);
	body->setType(rp::BodyType::STATIC);
	rp::BoxShape * shape = common.createBoxShape(half_extents);
	auto * collider = body->addCollider(shape, rp::Transform::identity());
	collider->getMaterial().setBounciness(1.0f);
	collider->getMaterial().setFrictionCoefficient(0.0f);
	return body;
}

static void set_dynamic_defaults(rp::RigidBody * body) {
	body->setLinearDamping(0.0f);
	body->setAngularDamping(0.0f);
}

static void set_collider_material(rp::Collider * collider) {
	collider->getMaterial().setBounciness(1.0f);
	collider->getMaterial().setFrictionCoefficient(0.0f);
}

static void run() {
	rp::PhysicsCommon common;
	rp::PhysicsWorld::WorldSettings settings;
	settings.gravity = rp::Vector3(0, -9.8f, 0);
	rp::PhysicsWorld * world = common.createPhysicsWorld(settings);

	scene_events events;
	world->setEventListener(&events);

	const float half_size = 3.0f;
	const float size = 6.0f;
	const float wall_thick = 1.0f;

	// Six walls — half-extents and position (RP3D positions a box's local origin
	// at the body center, so wall positions are the *center* of the slab).
	make_wall(common, world,
	          rp::Vector3(half_size, wall_thick * 0.5f, half_size),
	          rp::Vector3(0, -wall_thick * 0.5f, 0)); // floor
	make_wall(common, world,
	          rp::Vector3(half_size, wall_thick * 0.5f, half_size),
	          rp::Vector3(0, size + wall_thick * 0.5f, 0)); // ceiling
	make_wall(common, world,
	          rp::Vector3(wall_thick * 0.5f, size * 0.5f, half_size),
	          rp::Vector3(-half_size - wall_thick * 0.5f, size * 0.5f, 0)); // -x
	make_wall(common, world,
	          rp::Vector3(wall_thick * 0.5f, size * 0.5f, half_size),
	          rp::Vector3(half_size + wall_thick * 0.5f, size * 0.5f, 0)); // +x
	make_wall(common, world,
	          rp::Vector3(half_size, size * 0.5f, wall_thick * 0.5f),
	          rp::Vector3(0, size * 0.5f, -half_size - wall_thick * 0.5f)); // -z (hop -y)
	make_wall(common, world,
	          rp::Vector3(half_size, size * 0.5f, wall_thick * 0.5f),
	          rp::Vector3(0, size * 0.5f, half_size + wall_thick * 0.5f)); // +z (hop +y)

	// Trigger pad — same footprint as the hop version, in the (-x, -z) corner.
	const float pad_half = 1.1f;
	const float pad_height = 0.2f;
	const float pad_corner = -1.7f;
	rp::Transform pad_xform(rp::Vector3(pad_corner, pad_height * 0.5f, pad_corner), rp::Quaternion::identity());
	rp::RigidBody * zone = world->createRigidBody(pad_xform);
	zone->setType(rp::BodyType::STATIC);
	{
		rp::BoxShape * pad_shape = common.createBoxShape(rp::Vector3(pad_half, pad_height * 0.5f, pad_half));
		auto * pad_collider = zone->addCollider(pad_shape, rp::Transform::identity());
		pad_collider->setIsTrigger(true);
	}
	events.zone_entity = zone->getEntity();

	std::vector<spring_spec> springs;

	// ── Box (1x1x1) free-bouncing ────────────────────────────────────────────
	rp::RigidBody * box_body;
	{
		rp::Transform t(zup(1.0f, 0.0f, 4.0f), rp::Quaternion::identity());
		box_body = world->createRigidBody(t);
		box_body->setMass(1.0f);
		set_dynamic_defaults(box_body);
		auto * shape = common.createBoxShape(rp::Vector3(0.5f, 0.5f, 0.5f));
		auto * col = box_body->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		// hop velocity (3, -2, 0) is Z-up → RP3D (3, 0, -2) Y-up
		box_body->setLinearVelocity(rp::Vector3(3, 0, -2));
	}

	// ── Vertical capsule on a spring from the ceiling ────────────────────────
	// hop solid pos (2, 0, 3); capsule extends 0 → (0,0,1.5) along hop +Z (i.e. up).
	// Capsule centroid offset in solid-local = (0, 0, 0.75).
	// World centroid (hop) = (2, 0, 3.75) → RP3D Y-up (2, 3.75, 0).
	const float pend_radius = 0.4f;
	const float pend_len = 1.5f;
	rp::RigidBody * pendulum_body;
	{
		rp::Transform t(rp::Vector3(2.0f, 3.75f, 0.0f), rp::Quaternion::identity());
		pendulum_body = world->createRigidBody(t);
		pendulum_body->setMass(1.0f);
		set_dynamic_defaults(pendulum_body);
		// RP3D capsule's local axis is Y. We want world Y, so no rotation needed.
		auto * shape = common.createCapsuleShape(pend_radius, pend_len);
		auto * col = pendulum_body->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		// hop velocity (-2, 1, 0) → RP3D (-2, 0, 1)
		pendulum_body->setLinearVelocity(rp::Vector3(-2, 0, 1));
	}
	// Spring attaches to the capsule's centroid; the RP3D body origin already
	// sits at the centroid, so the local anchor is (0, 0, 0).
	const rp::Vector3 pendulum_anchor = zup(1.5f, 0.0f, size);
	{
		spring_spec s{};
		s.a = pendulum_body;
		s.b = nullptr;
		s.anchor = pendulum_anchor;
		s.local_a = rp::Vector3(0, 0, 0);
		s.rest_length = 2.0f;
		s.k = 6.0f;
		s.is_rope = false;
		springs.push_back(s);
	}

	// ── Horizontal capsule on a rope ─────────────────────────────────────────
	// hop solid pos (-2, -1, 2); capsule extends 0 → (1.8, 0, 0) along hop +X.
	// Capsule centroid in solid-local = (0.9, 0, 0).
	// World centroid (hop) = (-1.1, -1, 2) → RP3D (-1.1, 2, -1).
	// RP3D capsule local axis is Y — rotate −90° about Z so local Y → world X.
	const float leash_radius = 0.3f;
	const float leash_len = 1.8f;
	rp::RigidBody * leash_body;
	{
		// Quaternion rotating local Y onto world X = rotation by −90° about Z.
		rp::Quaternion q = rp::Quaternion::fromEulerAngles(0, 0, -3.14159265358979f * 0.5f);
		rp::Transform t(rp::Vector3(-1.1f, 2.0f, -1.0f), q);
		leash_body = world->createRigidBody(t);
		leash_body->setMass(1.0f);
		set_dynamic_defaults(leash_body);
		auto * shape = common.createCapsuleShape(leash_radius, leash_len);
		auto * col = leash_body->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		// hop velocity (1, 2, 3) → RP3D (1, 3, 2)
		leash_body->setLinearVelocity(rp::Vector3(1, 3, 2));
	}
	const rp::Vector3 leash_anchor = zup(-1.5f, -1.0f, size);
	{
		// hop's local_anchor_a was (0.9, 0, 0) = the capsule centroid in solid-local.
		// In RP3D we placed the body origin at the centroid, so local anchor is (0,0,0).
		spring_spec s{};
		s.a = leash_body;
		s.b = nullptr;
		s.anchor = leash_anchor;
		s.local_a = rp::Vector3(0, 0, 0);
		s.rest_length = 3.0f;
		s.k = 20.0f;
		s.is_rope = true;
		springs.push_back(s);
	}

	// ── Octahedron + sphere, spring-coupled ──────────────────────────────────
	rp::ConvexMesh * octa_mesh = make_octahedron_mesh(common, 0.5f);
	rp::RigidBody * octa_body;
	{
		rp::Transform t(zup(-1.0f, 1.0f, 5.0f), rp::Quaternion::identity());
		octa_body = world->createRigidBody(t);
		octa_body->setMass(1.0f);
		set_dynamic_defaults(octa_body);
		auto * shape = common.createConvexMeshShape(octa_mesh);
		auto * col = octa_body->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		octa_body->setLinearVelocity(rp::Vector3(2, 0, -1));
	}
	rp::RigidBody * partner_body;
	{
		rp::Transform t(zup(0.5f, 1.0f, 5.0f), rp::Quaternion::identity());
		partner_body = world->createRigidBody(t);
		partner_body->setMass(1.0f);
		set_dynamic_defaults(partner_body);
		auto * shape = common.createSphereShape(0.4f);
		auto * col = partner_body->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		partner_body->setLinearVelocity(rp::Vector3(-1, 1, 2));
	}
	{
		spring_spec s{};
		s.a = octa_body;
		s.b = partner_body;
		s.local_a = rp::Vector3(0, 0, 0);
		s.rest_length = 1.5f;
		s.k = 8.0f;
		s.is_rope = false;
		springs.push_back(s);
	}

	// ── Two roped spheres (bola) ─────────────────────────────────────────────
	rp::RigidBody * bola_a;
	{
		rp::Transform t(zup(1.0f, 2.0f, 2.0f), rp::Quaternion::identity());
		bola_a = world->createRigidBody(t);
		bola_a->setMass(1.0f);
		set_dynamic_defaults(bola_a);
		auto * shape = common.createSphereShape(0.35f);
		auto * col = bola_a->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		bola_a->setLinearVelocity(rp::Vector3(3, 2, -2));
	}
	rp::RigidBody * bola_b;
	{
		rp::Transform t(zup(2.0f, 1.5f, 2.0f), rp::Quaternion::identity());
		bola_b = world->createRigidBody(t);
		bola_b->setMass(1.0f);
		set_dynamic_defaults(bola_b);
		auto * shape = common.createSphereShape(0.35f);
		auto * col = bola_b->addCollider(shape, rp::Transform::identity());
		set_collider_material(col);
		bola_b->setLinearVelocity(rp::Vector3(-2, 1, 3));
	}
	{
		spring_spec s{};
		s.a = bola_a;
		s.b = bola_b;
		s.local_a = rp::Vector3(0, 0, 0);
		s.rest_length = 1.4f;
		s.k = 25.0f;
		s.is_rope = true;
		springs.push_back(s);
	}

	// Raylib window
	int win_w = capture_dir ? 400 : 800;
	int win_h = capture_dir ? 300 : 600;
	float duration = capture_dir ? 6.0f : 0.0f;
	InitWindow(win_w, win_h, "reactphysics3d — bounce room");
	SetTargetFPS(60);

	float cam_angle = 0.0f;
	int frame_num = 0;
	const float fixed_dt = 1.0f / 60.0f;
	float elapsed = 0.0f;

	while (!WindowShouldClose() && (duration <= 0.0f || elapsed < duration)) {
		float frame_dt = GetFrameTime();
		elapsed += frame_dt;

		apply_springs(springs);
		world->update(fixed_dt);

		bool box_in_zone = events.is_in_zone(box_body);
		bool pendulum_in_zone = events.is_in_zone(pendulum_body);
		bool leash_in_zone = events.is_in_zone(leash_body);
		bool octa_in_zone = events.is_in_zone(octa_body);
		bool partner_in_zone = events.is_in_zone(partner_body);
		bool bola_a_in_zone = events.is_in_zone(bola_a);
		bool bola_b_in_zone = events.is_in_zone(bola_b);

		cam_angle += 0.3f * GetFrameTime();
		float cam_dist = 18.0f;
		float cam_height = 8.0f;
		Camera3D camera = {};
		camera.position = { cam_dist * cosf(cam_angle), cam_height, cam_dist * sinf(cam_angle) };
		camera.target = { 0.0f, 3.0f, 0.0f };
		camera.up = { 0.0f, 1.0f, 0.0f };
		camera.fovy = 50.0f;
		camera.projection = CAMERA_PERSPECTIVE;

		BeginDrawing();
		ClearBackground({ 30, 30, 40, 255 });
		BeginMode3D(camera);

		DrawCubeWires({ 0.0f, 3.0f, 0.0f }, 6.0f, 6.0f, 6.0f, DARKGRAY);
		DrawCube({ 0.0f, -0.05f, 0.0f }, 6.0f, 0.1f, 6.0f, { 60, 60, 80, 100 });

		{
			float pad_w = 2.0f * pad_half;
			Vector3 zp = to_rl(zone->getTransform().getPosition());
			DrawCube(zp, pad_w, pad_height, pad_w, { 255, 220, 80, 50 });
			DrawCubeWires(zp, pad_w, pad_height, pad_w, { 255, 220, 80, 200 });
		}

		Color tint_in = YELLOW;

		// Box
		{
			rp::Vector3 p = box_body->getTransform().getPosition();
			rp::Quaternion q = box_body->getTransform().getOrientation();
			Vector3 axis; float angle;
			q.getRotationAngleAxis(angle, *(rp::Vector3 *)&axis);
			rlPushMatrix();
			rlTranslatef(p.x, p.y, p.z);
			rlRotatef(angle * 180.0f / 3.14159265358979f, axis.x, axis.y, axis.z);
			DrawCube({ 0, 0, 0 }, 1.0f, 1.0f, 1.0f, box_in_zone ? tint_in : RED);
			DrawCubeWires({ 0, 0, 0 }, 1.0f, 1.0f, 1.0f, MAROON);
			rlPopMatrix();
		}

		// Pendulum capsule (vertical, body-local Y is world up)
		{
			rp::Vector3 top_local(0, pend_len * 0.5f, 0);
			rp::Vector3 bot_local(0, -pend_len * 0.5f, 0);
			Vector3 top_rl = to_rl(pendulum_body->getTransform() * top_local);
			Vector3 bot_rl = to_rl(pendulum_body->getTransform() * bot_local);
			Vector3 mid_rl = to_rl(pendulum_body->getTransform().getPosition());
			DrawCapsule(bot_rl, top_rl, pend_radius, 8, 8, pendulum_in_zone ? tint_in : GREEN);
			DrawCapsuleWires(bot_rl, top_rl, pend_radius, 8, 8, DARKGREEN);
			// Spring from ceiling anchor → capsule centroid.
			draw_constraint_line(to_rl(pendulum_anchor), mid_rl, 2.0f, true);
		}

		// Leash capsule (horizontal — rotated so body-local Y points along world X)
		{
			rp::Vector3 end_local(0, leash_len * 0.5f, 0);
			rp::Vector3 start_local(0, -leash_len * 0.5f, 0);
			Vector3 a = to_rl(leash_body->getTransform() * start_local);
			Vector3 b = to_rl(leash_body->getTransform() * end_local);
			Vector3 m = to_rl(leash_body->getTransform().getPosition());
			DrawCapsule(a, b, leash_radius, 8, 8, leash_in_zone ? tint_in : ORANGE);
			DrawCapsuleWires(a, b, leash_radius, 8, 8, { 200, 100, 0, 255 });
			draw_constraint_line(to_rl(leash_anchor), m, 3.0f, false);
		}

		// Octahedron + sphere
		Vector3 octa_pos = to_rl(octa_body->getTransform().getPosition());
		Vector3 sphere_pos = to_rl(partner_body->getTransform().getPosition());
		{
			Color octa_color = octa_in_zone ? tint_in : SKYBLUE;
			draw_octahedron(octa_pos, 0.5f, octa_color, DARKBLUE);
			DrawSphere(sphere_pos, 0.4f, partner_in_zone ? tint_in : VIOLET);
			DrawSphereWires(sphere_pos, 0.4f, 8, 8, DARKPURPLE);
			draw_constraint_line(octa_pos, sphere_pos, 1.5f, true);
		}

		// Bola
		Vector3 ba = to_rl(bola_a->getTransform().getPosition());
		Vector3 bb = to_rl(bola_b->getTransform().getPosition());
		{
			DrawSphere(ba, 0.35f, bola_a_in_zone ? tint_in : PINK);
			DrawSphereWires(ba, 0.35f, 8, 8, MAROON);
			DrawSphere(bb, 0.35f, bola_b_in_zone ? tint_in : PINK);
			DrawSphereWires(bb, 0.35f, 8, 8, MAROON);
			draw_constraint_line(ba, bb, 1.4f, false);
		}

		update_and_draw_sparks(frame_dt);

		EndMode3D();

		DrawText("reactphysics3d", 10, 10, 20, LIGHTGRAY);
		DrawFPS(10, 40);

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
	common.destroyPhysicsWorld(world);
}

int main(int argc, char * argv[]) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc) {
			capture_dir = argv[++i];
		}
	}
	run();
	return 0;
}
