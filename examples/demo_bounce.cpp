// demo_bounce.cpp — Raylib visualization of hop physics
//
// Showcases every primitive collision shape (box, sphere, capsule, convex_solid)
// and every constraint variant:
//   spring + solid→anchor  : vertical capsule pendulum hanging from the ceiling
//   rope   + solid→anchor  : horizontal capsule leashed to a ceiling anchor
//   spring + solid→solid   : octahedron and a sphere coupled together
//   rope   + solid→solid   : two spheres tethered like a bola
//
// A thin "pressure pad" trigger zone sits in one floor corner — bodies tint
// yellow while overlapping it.
//
// Pass --fixed to use fixed16 arithmetic instead of float.

#include <cmath>
#include <cstring>
#include <hop/hop.h>
#include <raylib.h>
#include <rlgl.h>
#include <string>
#include <vector>

// Hop is Z-up, raylib is Y-up — swap Y and Z
template <typename T> Vector3 to_raylib(const hop::vec3<T> & v) {
	using tr = hop::scalar_traits<T>;
	return { tr::to_float(v.x), tr::to_float(v.z), tr::to_float(v.y) };
}

// Draw a cube by its 8 orientation-transformed world corners (Phase 8/9: the free
// box now spins, so DrawCube — which ignores rotation — would hide the tumble).
template <typename T>
static void draw_oriented_cube(const hop::vec3<T> & pos, const hop::mat3<T> & R, T half, Color face, Color wire) {
	Vector3 c[8];
	for (int i = 0; i < 8; ++i) {
		hop::vec3<T> local((i & 1) ? half : -half, (i & 2) ? half : -half, (i & 4) ? half : -half);
		hop::vec3<T> w;
		hop::mul(w, R, local);
		hop::add(w, pos);
		c[i] = to_raylib(w);
	}
	static const int faces[6][4] = { { 0, 1, 3, 2 }, { 4, 6, 7, 5 }, { 0, 4, 5, 1 }, { 2, 3, 7, 6 }, { 0, 2, 6, 4 }, { 1, 5, 7, 3 } };
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

// Spark particle
struct spark {
	Vector3 pos;
	Vector3 vel;
	float life; // remaining lifetime in seconds
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
		s.vel.y -= 9.8f * dt; // gravity
		s.life -= dt;
		if (s.life <= 0.0f) {
			sparks[i] = sparks.back();
			sparks.pop_back();
			continue;
		}
		float t = s.life / s.max_life; // 1 = fresh, 0 = dead
		unsigned char alpha = (unsigned char)(255 * t);
		unsigned char r = 255;
		unsigned char g = (unsigned char)(200 * t + 55); // yellow -> orange
		unsigned char b = (unsigned char)(50 * t);
		float radius = 0.04f * t;
		DrawSphere(s.pos, radius, { r, g, b, alpha });
	}
}

template <typename T> void spark_on_collision(const hop::collision<T> & c) {
	Vector3 p = to_raylib(c.impact);
	// Scale spark count by collision speed
	float speed = hop::scalar_traits<T>::to_float(hop::length(c.velocity));
	int count = 4 + (int)(speed * 1.5f);
	if (count > 16)
		count = 16;
	spawn_sparks(p, count);
}

// Helper: create a wall solid (infinite mass, no gravity, positioned at `pos`)
template <typename T>
std::shared_ptr<hop::solid<T>> make_wall(hop::simulator<T> & sim,
                                         const hop::aa_box<T> & box,
                                         const hop::vec3<T> & pos) {
	using tr = hop::scalar_traits<T>;
	auto wall = std::make_shared<hop::solid<T>>();
	wall->set_infinite_mass();
	wall->set_coefficient_of_gravity(T {});
	wall->set_coefficient_of_restitution(tr::one());
	auto sh = std::make_shared<hop::shape<T>>(box);
	wall->add_shape(sh);
	wall->set_position(pos);
	sim.add_solid(wall);
	return wall;
}

// Build a regular octahedron centered at the origin, vertex distance `r` from
// the center. 8 face planes — the dual of an axis-aligned cube.
template <typename T> static hop::convex_solid<T> make_octahedron(T r) {
	using tr = hop::scalar_traits<T>;
	hop::convex_solid<T> cs;
	// Face normals point at the 8 octants. Each face plane satisfies
	// dot(n, vertex) = r/√3 for the axis-vertices, so distance = r/√3.
	T inv_sqrt3 = tr::one() / tr::sqrt(tr::from_int(3));
	T n = inv_sqrt3;
	T d = r * inv_sqrt3;
	cs.planes.push_back({ {  n,  n,  n }, d });
	cs.planes.push_back({ {  n,  n, -n }, d });
	cs.planes.push_back({ {  n, -n,  n }, d });
	cs.planes.push_back({ {  n, -n, -n }, d });
	cs.planes.push_back({ { -n,  n,  n }, d });
	cs.planes.push_back({ { -n,  n, -n }, d });
	cs.planes.push_back({ { -n, -n,  n }, d });
	cs.planes.push_back({ { -n, -n, -n }, d });
	return cs;
}

// Draw a regular octahedron of radius r, oriented by R at hop position pos. The 6
// vertices are built in hop space (pos + R·(±r·axis)) and mapped to raylib, so the
// body's Phase 8/9 spin is visible.
template <typename T>
static void draw_octahedron(const hop::vec3<T> & pos, const hop::mat3<T> & R, T r, Color fill, Color wire) {
	auto vert = [&](T x, T y, T z) {
		hop::vec3<T> w;
		hop::mul(w, R, hop::vec3<T>(x, y, z));
		hop::add(w, pos);
		return to_raylib(w);
	};
	const T nr = -r;
	Vector3 v[6] = {
		vert(r, T {}, T {}),   // +x
		vert(nr, T {}, T {}),  // -x
		vert(T {}, r, T {}),   // +y
		vert(T {}, nr, T {}),  // -y
		vert(T {}, T {}, r),   // +z
		vert(T {}, T {}, nr),  // -z
	};
	// 8 triangle faces, wound so the outward normal points away from p.
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
	// Edges: 4 from top vertex (v2), 4 from bottom vertex (v3), 4 equator.
	DrawLine3D(v[2], v[0], wire); DrawLine3D(v[2], v[1], wire);
	DrawLine3D(v[2], v[4], wire); DrawLine3D(v[2], v[5], wire);
	DrawLine3D(v[3], v[0], wire); DrawLine3D(v[3], v[1], wire);
	DrawLine3D(v[3], v[4], wire); DrawLine3D(v[3], v[5], wire);
	DrawLine3D(v[0], v[4], wire); DrawLine3D(v[4], v[1], wire);
	DrawLine3D(v[1], v[5], wire); DrawLine3D(v[5], v[0], wire);
}

// Draw a constraint as a line between two world points. Springs zigzag, ropes
// are straight; both fade to red when stretched well past their rest length.
static void draw_constraint_line(Vector3 a, Vector3 b, float rest_length, bool is_spring) {
	float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
	float len = sqrtf(dx * dx + dy * dy + dz * dz);
	float stretch = (rest_length > 0.0f) ? (len / rest_length) : 1.0f;
	// Slack rope: gray. Spring at rest: gray. Stretched: blend to red.
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
	// Zigzag for springs: 12 segments offset perpendicular to the spring axis.
	const int segs = 14;
	// Build an arbitrary unit vector perpendicular to (dx,dy,dz).
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
		// Pinch amplitude near endpoints so the spring meets each anchor cleanly.
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

// Trigger-zone channel. Dynamic bodies clear this bit from their
// collide_with_scope so the simulator's normal sweep ignores the zone (no
// impulse push). Each frame we run a separate probe trace with this bit set
// to detect which bodies are currently inside the zone.
static constexpr int TRIGGER_BIT = 0x100;

template <typename T> void run() {
	using tr = hop::scalar_traits<T>;

	hop::simulator<T> sim;

	// Room dimensions: 6x6x6, centered at origin, floor at z=0, ceiling at z=6
	const T half_size = tr::from_int(3);
	const T size = tr::from_int(6);
	const T wall_thick = tr::one();
	const T zero = T {};

	// Floor: z = -wall_thick to 0
	make_wall(
	    sim,
	    hop::aa_box<T>(hop::vec3<T>(-half_size, -half_size, -wall_thick), hop::vec3<T>(half_size, half_size, zero)),
	    hop::vec3<T>());

	// Ceiling: z = 6 to 6+wall_thick
	make_wall(
	    sim,
	    hop::aa_box<T>(hop::vec3<T>(-half_size, -half_size, zero), hop::vec3<T>(half_size, half_size, wall_thick)),
	    hop::vec3<T>(zero, zero, size));

	// Wall -X
	make_wall(sim,
	          hop::aa_box<T>(hop::vec3<T>(-wall_thick, -half_size, zero), hop::vec3<T>(zero, half_size, size)),
	          hop::vec3<T>(-half_size, zero, zero));

	// Wall +X
	make_wall(sim,
	          hop::aa_box<T>(hop::vec3<T>(zero, -half_size, zero), hop::vec3<T>(wall_thick, half_size, size)),
	          hop::vec3<T>(half_size, zero, zero));

	// Wall -Y
	make_wall(sim,
	          hop::aa_box<T>(hop::vec3<T>(-half_size, -wall_thick, zero), hop::vec3<T>(half_size, zero, size)),
	          hop::vec3<T>(zero, -half_size, zero));

	// Wall +Y
	make_wall(sim,
	          hop::aa_box<T>(hop::vec3<T>(-half_size, zero, zero), hop::vec3<T>(half_size, wall_thick, size)),
	          hop::vec3<T>(zero, half_size, zero));

	// Trigger zone: a thin pad in the -X/-Y floor corner. Bodies pass straight
	// through it; the zone's TRIGGER_BIT bit shows up in trace_solid results
	// whenever a body is overlapping. Marked with TRIGGER_BIT on both broadcast
	// (collision_scope) and trigger_scope; collide_with_scope = 0 so the zone
	// never sweeps. Dynamic bodies below clear TRIGGER_BIT from their
	// collide_with_scope, so the simulator's normal trace filter discards the
	// zone — no impulse, no bounce.
	const T pad_half = tr::from_milli(1100);   // 1.1 m half-extent in XY
	const T pad_height = tr::from_milli(200);  // 20 cm tall
	const T pad_corner = -tr::from_milli(1700); // center at (-1.7, -1.7)
	auto zone = std::make_shared<hop::solid<T>>();
	zone->set_infinite_mass();
	zone->set_coefficient_of_gravity(T {});
	zone->set_collision_scope(TRIGGER_BIT);
	zone->set_collide_with_scope(0);
	zone->set_trigger_scope(TRIGGER_BIT);
	zone->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(
	    hop::vec3<T>(-pad_half, -pad_half, zero),
	    hop::vec3<T>(pad_half, pad_half, pad_height))));
	zone->set_position(hop::vec3<T>(pad_corner, pad_corner, zero));
	sim.add_solid(zone);

	// Dynamic bodies pass through the zone but still bounce off everything
	// else (walls, each other) — the inverse of TRIGGER_BIT keeps every
	// other channel.
	const int normal_channels = ~TRIGGER_BIT;

	const T cor = tr::one();
	const T fric_zero = T {};

	auto set_common = [&](const std::shared_ptr<hop::solid<T>> & s) {
		s->set_mass(tr::one());
		s->set_coefficient_of_restitution(cor);
		s->set_restitution_combine(hop::restitution_combine::minimum);
		s->set_coefficient_of_static_friction(fric_zero);
		s->set_coefficient_of_dynamic_friction(fric_zero);
		s->set_collide_with_scope(normal_channels);
	};

	// ── Shape: aa_box ────────────────────────────────────────────────────────
	// Free-bouncing 1x1x1 box. No constraint.
	auto box_solid = std::make_shared<hop::solid<T>>();
	set_common(box_solid);
	box_solid->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(
	    hop::vec3<T>(-tr::half(), -tr::half(), -tr::half()),
	    hop::vec3<T>(tr::half(), tr::half(), tr::half()))));
	// Phase 8/9: finite inertia (unit box, m=1 → I = m/12·(1²+1²) = 1/6 per axis) so
	// off-center wall/floor hits torque it and it tumbles via angular impulse response.
	box_solid->set_inertia(hop::vec3<T>(tr::from_milli(167), tr::from_milli(167), tr::from_milli(167)));
	box_solid->set_position(hop::vec3<T>(tr::from_int(1), zero, tr::from_int(4)));
	box_solid->set_velocity(hop::vec3<T>(tr::from_int(3), tr::from_int(-2), zero));
	sim.add_solid(box_solid);

	// ── Shape: capsule + Constraint: spring → anchor ─────────────────────────
	// Vertical capsule on a spring "pendulum" from the ceiling — bilateral
	// force pulls when stretched, pushes when compressed.
	auto pendulum_solid = std::make_shared<hop::solid<T>>();
	set_common(pendulum_solid);
	// Spine centered on the solid origin (= COM / rotation pivot), so it spins about
	// its middle rather than one end: from local z=−0.75 to +0.75.
	hop::capsule<T> pendulum_shape(hop::vec3<T>(zero, zero, -tr::from_milli(750)), hop::vec3<T>(zero, zero, tr::from_milli(1500)), tr::from_milli(400));
	pendulum_solid->add_shape(std::make_shared<hop::shape<T>>(pendulum_shape));
	// Phase 9: finite inertia (anisotropic — easy spin about the spine = local z).
	pendulum_solid->set_inertia(hop::vec3<T>(tr::half(), tr::half(), tr::from_milli(100)));
	pendulum_solid->set_position(hop::vec3<T>(tr::from_int(2), zero, tr::from_int(3)));
	pendulum_solid->set_velocity(hop::vec3<T>(-tr::from_int(2), tr::from_int(1), zero));
	sim.add_solid(pendulum_solid);

	hop::vec3<T> pendulum_anchor(tr::from_milli(1500), zero, size);
	auto pendulum_spring = std::make_shared<hop::constraint<T>>(pendulum_solid, pendulum_anchor);
	pendulum_spring->set_type(hop::constraint<T>::type::spring);
	pendulum_spring->set_rest_length(tr::from_int(2));
	pendulum_spring->set_spring_constant(tr::from_int(6));
	pendulum_spring->set_damping_constant(T {});
	// Anchor the spring at the capsule's centroid (now the solid origin).
	pendulum_spring->set_local_anchor_a(hop::vec3<T>(zero, zero, zero));
	sim.add_constraint(pendulum_spring);

	// ── Shape: capsule + Constraint: rope → anchor ───────────────────────────
	// Horizontal capsule leashed to a ceiling anchor — falls and bounces freely
	// until the rope hits its max length, then snaps back.
	auto leash_capsule = std::make_shared<hop::solid<T>>();
	set_common(leash_capsule);
	// Spine centered on the solid origin (= COM), so it spins about its middle.
	hop::capsule<T> leash_shape(hop::vec3<T>(-tr::from_milli(900), zero, zero), hop::vec3<T>(tr::from_milli(1800), zero, zero), tr::from_milli(300));
	leash_capsule->add_shape(std::make_shared<hop::shape<T>>(leash_shape));
	// Phase 9: finite inertia (anisotropic — easy spin about the spine = local x).
	leash_capsule->set_inertia(hop::vec3<T>(tr::from_milli(100), tr::half(), tr::half()));
	leash_capsule->set_position(hop::vec3<T>(-tr::from_int(2), -tr::from_int(1), tr::from_int(2)));
	leash_capsule->set_velocity(hop::vec3<T>(tr::one(), tr::from_int(2), tr::from_int(3)));
	sim.add_solid(leash_capsule);

	hop::vec3<T> leash_anchor(-tr::from_milli(1500), -tr::from_milli(1000), size);
	auto leash_rope = std::make_shared<hop::constraint<T>>(leash_capsule, leash_anchor);
	leash_rope->set_type(hop::constraint<T>::type::rope);
	leash_rope->set_rest_length(tr::from_int(3));
	leash_rope->set_spring_constant(tr::from_int(20));
	leash_rope->set_damping_constant(T {});
	// Anchor the rope at the capsule's centroid (now the solid origin).
	leash_rope->set_local_anchor_a(hop::vec3<T>(zero, zero, zero));
	sim.add_constraint(leash_rope);

	// ── Shape: convex_solid + sphere + Constraint: spring → solid ────────────
	// Octahedron coupled to a sphere by a spring — the pair oscillates as
	// they tumble around the room.
	auto octa_solid = std::make_shared<hop::solid<T>>();
	set_common(octa_solid);
	octa_solid->add_shape(std::make_shared<hop::shape<T>>(make_octahedron<T>(tr::from_milli(500))));
	octa_solid->set_inertia(hop::vec3<T>(tr::from_milli(100), tr::from_milli(100), tr::from_milli(100))); // Phase 9: tumbles off hits
	octa_solid->set_position(hop::vec3<T>(-tr::from_int(1), tr::from_int(1), tr::from_int(5)));
	octa_solid->set_velocity(hop::vec3<T>(tr::from_int(2), -tr::one(), zero));
	sim.add_solid(octa_solid);

	auto partner_sphere = std::make_shared<hop::solid<T>>();
	set_common(partner_sphere);
	partner_sphere->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(tr::from_milli(400))));
	partner_sphere->set_inertia(hop::vec3<T>(tr::from_milli(64), tr::from_milli(64), tr::from_milli(64))); // 2/5 m r², r=0.4: rolls under friction
	partner_sphere->set_position(hop::vec3<T>(tr::from_milli(500), tr::from_int(1), tr::from_int(5)));
	partner_sphere->set_velocity(hop::vec3<T>(-tr::one(), tr::from_int(2), tr::from_int(1)));
	sim.add_solid(partner_sphere);

	auto pair_spring = std::make_shared<hop::constraint<T>>(octa_solid, partner_sphere);
	pair_spring->set_type(hop::constraint<T>::type::spring);
	pair_spring->set_rest_length(tr::from_milli(1500));
	pair_spring->set_spring_constant(tr::from_int(8));
	pair_spring->set_damping_constant(T {});
	sim.add_constraint(pair_spring);

	// ── Shape: sphere + Constraint: rope → solid ─────────────────────────────
	// Two spheres tethered by a rope — a "bola" that swings around itself.
	auto bola_a = std::make_shared<hop::solid<T>>();
	set_common(bola_a);
	bola_a->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(tr::from_milli(350))));
	bola_a->set_inertia(hop::vec3<T>(tr::from_milli(49), tr::from_milli(49), tr::from_milli(49)));
	bola_a->set_position(hop::vec3<T>(tr::from_int(1), tr::from_int(2), tr::from_int(2)));
	bola_a->set_velocity(hop::vec3<T>(tr::from_int(3), -tr::from_int(2), tr::from_int(2)));
	sim.add_solid(bola_a);

	auto bola_b = std::make_shared<hop::solid<T>>();
	set_common(bola_b);
	bola_b->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(tr::from_milli(350))));
	bola_b->set_inertia(hop::vec3<T>(tr::from_milli(49), tr::from_milli(49), tr::from_milli(49)));
	bola_b->set_position(hop::vec3<T>(tr::from_int(2), tr::from_milli(1500), tr::from_int(2)));
	bola_b->set_velocity(hop::vec3<T>(-tr::from_int(2), tr::from_int(3), tr::one()));
	sim.add_solid(bola_b);

	auto bola_rope = std::make_shared<hop::constraint<T>>(bola_a, bola_b);
	bola_rope->set_type(hop::constraint<T>::type::rope);
	bola_rope->set_rest_length(tr::from_milli(1400));
	bola_rope->set_spring_constant(tr::from_int(25));
	bola_rope->set_damping_constant(T {});
	sim.add_constraint(bola_rope);

	// Collision sparks — everything dynamic.
	box_solid->set_collision_callback(spark_on_collision<T>);
	pendulum_solid->set_collision_callback(spark_on_collision<T>);
	leash_capsule->set_collision_callback(spark_on_collision<T>);
	octa_solid->set_collision_callback(spark_on_collision<T>);
	partner_sphere->set_collision_callback(spark_on_collision<T>);
	bola_a->set_collision_callback(spark_on_collision<T>);
	bola_b->set_collision_callback(spark_on_collision<T>);

	// Raylib window
	int win_w = capture_dir ? 400 : 800;
	int win_h = capture_dir ? 300 : 600;
	float duration = capture_dir ? 6.0f : 0.0f;
	InitWindow(win_w, win_h, "hop physics — bounce room");
	SetTargetFPS(60);

	float cam_angle = 0.0f;
	const char * mode_label = std::is_same_v<T, hop::fixed16> ? "fixed16" : "float";
	int frame_num = 0;

	// Static-overlap probe: ask "is this body currently inside any solid that
	// broadcasts on TRIGGER_BIT?" by tracing a zero-direction segment with the
	// trigger channel as the only listened-for bit. The trace ignores walls
	// and other dynamic bodies because their trigger_scope is 0.
	auto in_trigger_zone = [&](const std::shared_ptr<hop::solid<T>> & body) -> bool {
		hop::collision<T> r;
		hop::segment<T> probe;
		probe.set_start_dir(body->get_position(), hop::vec3<T>());
		sim.trace_solid(r, body.get(), probe, TRIGGER_BIT);
		return (r.trigger_scope & TRIGGER_BIT) != 0;
	};

	float elapsed = 0.0f;
	while (!WindowShouldClose() && (duration <= 0.0f || elapsed < duration)) {
		float frame_dt = GetFrameTime();
		elapsed += frame_dt;
		sim.update(tr::from_milli(16));

		bool box_in_zone = in_trigger_zone(box_solid);
		bool pendulum_in_zone = in_trigger_zone(pendulum_solid);
		bool leash_in_zone = in_trigger_zone(leash_capsule);
		bool octa_in_zone = in_trigger_zone(octa_solid);
		bool partner_in_zone = in_trigger_zone(partner_sphere);
		bool bola_a_in_zone = in_trigger_zone(bola_a);
		bool bola_b_in_zone = in_trigger_zone(bola_b);

		// Orbiting camera looking at room center
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

		// Room wireframe + semi-transparent floor
		DrawCubeWires({ 0.0f, 3.0f, 0.0f }, 6.0f, 6.0f, 6.0f, DARKGRAY);
		DrawCube({ 0.0f, -0.05f, 0.0f }, 6.0f, 0.1f, 6.0f, { 60, 60, 80, 100 });

		// Trigger pad: a low, faintly glowing tile on the floor.
		{
			float pad_w = 2.0f * hop::scalar_traits<T>::to_float(pad_half);
			float pad_h = hop::scalar_traits<T>::to_float(pad_height);
			Vector3 zp = to_raylib(zone->get_position());
			Vector3 center = { zp.x, pad_h * 0.5f, zp.z };
			DrawCube(center, pad_w, pad_h, pad_w, { 255, 220, 80, 50 });
			DrawCubeWires(center, pad_w, pad_h, pad_w, { 255, 220, 80, 200 });
		}

		Color tint_in = YELLOW;

		// Box — oriented draw so its Phase 8/9 tumble is visible.
		{
			Color box_face = box_in_zone ? tint_in : (Color){ 230, 41, 55, 160 };
			draw_oriented_cube(box_solid->get_position(), box_solid->get_orientation(), tr::half(), box_face, MAROON);
		}

		// Spring pendulum capsule — spine endpoints rotated by the solid orientation.
		{
			auto & p = pendulum_solid->get_position();
			const hop::mat3<T> & R = pendulum_solid->get_orientation();
			auto along = [&](T s) { hop::vec3<T> o, w; hop::mul(o, R, hop::vec3<T>(zero, zero, s)); hop::add(w, p, o); return w; };
			Vector3 bot_rl = to_raylib(along(-tr::from_milli(750))); // spine ends, centered on the COM
			Vector3 top_rl = to_raylib(along(tr::from_milli(750)));
			Vector3 mid_rl = to_raylib(p);                           // centroid = origin
			DrawCapsule(bot_rl, top_rl, 0.4f, 8, 8, pendulum_in_zone ? tint_in : GREEN);
			DrawCapsuleWires(bot_rl, top_rl, 0.4f, 8, 8, DARKGREEN);
			// Spring line: ceiling anchor to capsule centroid.
			draw_constraint_line(to_raylib(pendulum_anchor), mid_rl, 2.0f, /*spring*/ true);
		}

		// Rope-leashed horizontal capsule — spine endpoints rotated by the orientation.
		{
			auto & p = leash_capsule->get_position();
			const hop::mat3<T> & R = leash_capsule->get_orientation();
			auto along = [&](T s) { hop::vec3<T> o, w; hop::mul(o, R, hop::vec3<T>(s, zero, zero)); hop::add(w, p, o); return w; };
			Vector3 a = to_raylib(along(-tr::from_milli(900))); // spine ends, centered on the COM
			Vector3 b = to_raylib(along(tr::from_milli(900)));
			Vector3 m = to_raylib(p);                           // centroid = origin
			DrawCapsule(a, b, 0.3f, 8, 8, leash_in_zone ? tint_in : ORANGE);
			DrawCapsuleWires(a, b, 0.3f, 8, 8, { 200, 100, 0, 255 });
			// Rope from anchor to capsule centroid
			draw_constraint_line(to_raylib(leash_anchor), m, 3.0f, /*spring*/ false);
		}

		// Octahedron + sphere spring pair
		Vector3 octa_pos = to_raylib(octa_solid->get_position());
		Vector3 sphere_pos = to_raylib(partner_sphere->get_position());
		{
			Color octa_color = octa_in_zone ? tint_in : SKYBLUE;
			draw_octahedron(octa_solid->get_position(), octa_solid->get_orientation(), tr::from_milli(500), octa_color, DARKBLUE);
			DrawSphere(sphere_pos, 0.4f, partner_in_zone ? tint_in : VIOLET);
			DrawSphereWires(sphere_pos, 0.4f, 8, 8, DARKPURPLE);
			draw_constraint_line(octa_pos, sphere_pos, 1.5f, /*spring*/ true);
		}

		// Bola: two roped spheres
		Vector3 ba = to_raylib(bola_a->get_position());
		Vector3 bb = to_raylib(bola_b->get_position());
		{
			DrawSphere(ba, 0.35f, bola_a_in_zone ? tint_in : PINK);
			DrawSphereWires(ba, 0.35f, 8, 8, MAROON);
			DrawSphere(bb, 0.35f, bola_b_in_zone ? tint_in : PINK);
			DrawSphereWires(bb, 0.35f, 8, 8, MAROON);
			draw_constraint_line(ba, bb, 1.4f, /*spring*/ false);
		}

		// Sparks
		update_and_draw_sparks(frame_dt);

		EndMode3D();

		// HUD
		DrawText(mode_label, 10, 10, 20, LIGHTGRAY);

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

int main(int argc, char * argv[]) {
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
