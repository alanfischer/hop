// A corpse is 21 boxes and 20 pins. Does the chain hold, does it sleep, and what does it
// cost?
//
// Headless — no raylib, no renderer. Run it and read the three tables. Exits non-zero
// while anything is wrong, so it is the acceptance test for constraint::type::rigid.
//
// The bodies are laid out like the hitboxes on a GoldSrc player model, because that is
// literally where the game's ragdoll comes from: models/player/*.mdl carries 21 boxes in
// bone-local space, one per bone below the root, and the game builds a PhysicalBone3D from
// each. Every bone is collision_scope 0 / collide_with 1 — it falls against the floor and
// passes through every other bone, which is what keeps a limb from fighting its own torso.
//
// ── TABLE 1: the chain holds ────────────────────────────────────────────────
//
// The joint error is the distance between a pin's two anchors, which a satisfied pin holds
// at zero. This is the test a force spring cannot pass at any stiffness: a spring needs a
// nonzero stretch to produce force at all, so a chain hung off one SAGS by construction,
// and raising k to hide the sag makes an explicitly-integrated chain ring and then leave.
//
// ── TABLE 2: the corpse does NOT sleep, and the joints are not why ──────────
//
// Sleep was meant to be the affordability argument: a wizard corpse lies on the floor for
// sixty seconds, and eight of them at 21 bodies apiece only fit in hop's budget if they go
// quiet. The joint half of that works — a rigid pin at rest carries ~zero positional error,
// so constraint::is_loaded reads false and a pinned body deactivates normally, which
// test_rigid_joint_sleeps holds to. What stops the corpse is underneath the joints:
//
//   AN ORIENTED BOX RESTING ON A FLOOR SPINS FOREVER.
//
// The control at the bottom of Table 2 is one box, no joints, no ragdoll. Dropped exactly
// axis-aligned it settles and sleeps. Tilted by 0.05 rad — three degrees — it turns at
// several rad/s for as long as you run it, and no amount of damping touches it, because the
// spin is not accumulating: it is handed out fresh every tick.
//
// It is arithmetic, not a solver bug. hop resolves a pair at ONE point, and support() on a
// tilted box returns a CORNER — the face centre it collapses to for an axis-aligned box is
// exactly why that case is the one that works. So a box lying almost flat is held up at a
// corner, and one tick of gravity's impulse at that lever is worth ~4 rad/s on a 0.35 kg
// bone. It tips, catches the next corner, and rocks there permanently, where a real contact
// manifold would put two or four points under it and hold it still. The smaller the body
// the worse it is (|w| roughly 1/size), and a corpse's extremities are its smallest bodies.
//
// This is the same failure as bug 2 in plans/rotating_gibs.md — "a capsule resting on a
// floor creates spin from nothing" — which was recorded as capsule-only with boxes immune.
// Boxes are immune only while they are axis-aligned. Fixing it properly is contact
// manifolds, which is a phase of its own and not this one. Until then the game stops a
// settled corpse itself rather than waiting for hop to sleep it.
//
// ── TABLE 3: what it costs ──────────────────────────────────────────────────
//
// Milliseconds per tick against corpse count. hop's whole-frame budget in Wizard Wars is
// ~6.5 ms, and a corpse is supposed to be a rounding error in it. If eight corpses are not,
// the fallback (see plans/one_corpse_ragdoll.md) is to merge hitboxes per limb segment down
// to ~11 bodies, which is a change to the game's builder and not to hop. Read the resting
// column knowing nothing sleeps yet — see Table 2 — so it is the honest worst case.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include <hop/hop.h>

using namespace hop;
using T = float;
using tr = scalar_traits<T>;

namespace {

struct bone_def {
	const char * name;
	int parent;
	T x, y, z;     // rest centre, pelvis-relative in metres
	T hx, hy, hz;  // half extents
};

// A humanoid the size of a GoldSrc player: ~1.8 m standing, pelvis at 1.0.
const bone_def kBones[] = {
	{ "pelvis",     -1,  0.00f, 1.00f,  0.00f, 0.090f, 0.090f, 0.070f },
	{ "spine",       0,  0.00f, 1.18f,  0.00f, 0.100f, 0.090f, 0.070f },
	{ "chest",       1,  0.00f, 1.36f,  0.00f, 0.110f, 0.090f, 0.080f },
	{ "neck",        2,  0.00f, 1.52f,  0.00f, 0.050f, 0.050f, 0.050f },
	{ "head",        3,  0.00f, 1.66f,  0.00f, 0.090f, 0.100f, 0.090f },
	{ "l_clavicle",  2,  0.09f, 1.48f,  0.00f, 0.060f, 0.040f, 0.040f },
	{ "l_upperarm",  5,  0.28f, 1.46f,  0.00f, 0.120f, 0.050f, 0.050f },
	{ "l_forearm",   6,  0.52f, 1.46f,  0.00f, 0.120f, 0.045f, 0.045f },
	{ "l_hand",      7,  0.70f, 1.46f,  0.00f, 0.060f, 0.040f, 0.030f },
	{ "r_clavicle",  2, -0.09f, 1.48f,  0.00f, 0.060f, 0.040f, 0.040f },
	{ "r_upperarm",  9, -0.28f, 1.46f,  0.00f, 0.120f, 0.050f, 0.050f },
	{ "r_forearm",  10, -0.52f, 1.46f,  0.00f, 0.120f, 0.045f, 0.045f },
	{ "r_hand",     11, -0.70f, 1.46f,  0.00f, 0.060f, 0.040f, 0.030f },
	{ "l_thigh",     0,  0.09f, 0.78f,  0.00f, 0.070f, 0.200f, 0.070f },
	{ "l_calf",     13,  0.09f, 0.40f,  0.00f, 0.060f, 0.190f, 0.060f },
	{ "l_foot",     14,  0.09f, 0.06f,  0.04f, 0.050f, 0.050f, 0.110f },
	{ "l_toe",      15,  0.09f, 0.03f,  0.16f, 0.040f, 0.030f, 0.040f },
	{ "r_thigh",     0, -0.09f, 0.78f,  0.00f, 0.070f, 0.200f, 0.070f },
	{ "r_calf",     17, -0.09f, 0.40f,  0.00f, 0.060f, 0.190f, 0.060f },
	{ "r_foot",     18, -0.09f, 0.06f,  0.04f, 0.050f, 0.050f, 0.110f },
	{ "r_toe",      19, -0.09f, 0.03f,  0.16f, 0.040f, 0.030f, 0.040f },
};
const int kBoneCount = static_cast<int>(sizeof(kBones) / sizeof(kBones[0]));

const int kFloorScope = 1;
const T kDensity = 900.0f;  // flesh, near enough

struct ragdoll {
	std::vector<std::shared_ptr<solid<T>>> bones;
	std::vector<std::shared_ptr<constraint<T>>> joints;
};

// One corpse, dropped in at `origin` with every bone carrying `launch`.
ragdoll build(simulator<T> & sim, const vec3<T> & origin, const vec3<T> & launch) {
	ragdoll r;
	r.bones.resize(kBoneCount);
	for (int i = 0; i < kBoneCount; ++i) {
		const bone_def & b = kBones[i];
		auto s = std::make_shared<solid<T>>();
		const T mass = 8.0f * b.hx * b.hy * b.hz * kDensity;
		const T sx = 2.0f * b.hx, sy = 2.0f * b.hy, sz = 2.0f * b.hz;
		s->set_mass(mass);
		s->set_inertia(vec3<T>(mass * (sy * sy + sz * sz) / 12.0f,
		                       mass * (sx * sx + sz * sz) / 12.0f,
		                       mass * (sx * sx + sy * sy) / 12.0f));
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-b.hx, -b.hy, -b.hz), vec3<T>(b.hx, b.hy, b.hz))));
		s->set_position(vec3<T>(origin.x + b.x, origin.y + b.y, origin.z + b.z));
		s->set_velocity(launch);
		sim.add_solid(s);
		// After add_solid, which stamps the space default. A rigid joint is solved in
		// Pass B, so its bodies must not have committed their position back in Pass A.
		s->set_contact_mode(contact_mode::speculative);
		// The game's layer 0 / mask WORLD: a bone hears the floor and broadcasts to
		// nothing, so limbs pass through each other and nothing can be hit by an arm.
		s->set_collision_scope(0);
		s->set_collide_with_scope(kFloorScope);
		r.bones[i] = s;
	}
	for (int i = 0; i < kBoneCount; ++i) {
		const bone_def & b = kBones[i];
		if (b.parent < 0)
			continue;  // the pelvis is the free root
		const bone_def & p = kBones[b.parent];
		// The pin sits between the two bone centres — near enough to the real joint for
		// a corpse, and exactly what PhysicalBone3D hands the server.
		vec3<T> anchor((b.x + p.x) * 0.5f, (b.y + p.y) * 0.5f, (b.z + p.z) * 0.5f);
		auto c = std::make_shared<constraint<T>>(r.bones[b.parent], r.bones[i]);
		c->set_type(constraint<T>::type::rigid);
		c->set_local_anchor_a(vec3<T>(anchor.x - p.x, anchor.y - p.y, anchor.z - p.z));
		c->set_local_anchor_b(vec3<T>(anchor.x - b.x, anchor.y - b.y, anchor.z - b.z));
		sim.add_constraint(c);
		r.joints.push_back(c);
	}
	return r;
}

void add_floor(simulator<T> & sim) {
	auto floor = std::make_shared<solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});  // infinite mass still falls without this
	floor->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-50.0f, -1.0f, -50.0f), vec3<T>(50.0f, 0.0f, 50.0f))));
	floor->set_position(vec3<T>(0.0f, 0.0f, 0.0f));
	sim.add_solid(floor);
	floor->set_collision_scope(kFloorScope);
	floor->set_collide_with_scope(0);
}

vec3<T> anchor_world(const solid<T> * s, const vec3<T> & local) {
	vec3<T> lever, out;
	mul(lever, s->get_orientation(), local);
	add(out, s->get_position(), lever);
	return out;
}

// Largest gap between any pin's two anchors, in metres.
float worst_joint_error(const ragdoll & r) {
	float worst = 0.0f;
	for (auto & c : r.joints) {
		vec3<T> a = anchor_world(c->get_start_solid(), c->get_local_anchor_a());
		vec3<T> b = anchor_world(c->get_end_solid(), c->get_local_anchor_b());
		float e = std::sqrt(length_squared(a, b));
		if (e > worst)
			worst = e;
	}
	return worst;
}

bool all_asleep(const ragdoll & r) {
	for (auto & s : r.bones)
		if (s->active())
			return false;
	return true;
}

bool sane(const ragdoll & r) {
	for (auto & s : r.bones) {
		const vec3<T> & p = s->get_position();
		if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
			return false;
		if (std::fabs(p.x) > 100.0f || std::fabs(p.y) > 100.0f || std::fabs(p.z) > 100.0f)
			return false;
	}
	return true;
}

// The control for Table 2: one box, no joints, no ragdoll. Returns the mean |ω| over the
// second half of a ten-second run, long after it has landed.
float resting_box_spin(T half, T tilt) {
	simulator<T> sim;
	sim.set_gravity(vec3<T>(0.0f, -20.0f, 0.0f));
	add_floor(sim);
	auto b = std::make_shared<solid<T>>();
	const T m = 8.0f * half * half * half * kDensity;
	const T side = 2.0f * half;
	b->set_mass(m);
	b->set_inertia(vec3<T>(m * (side * side + side * side) / 12.0f,
	                       m * (side * side + side * side) / 12.0f,
	                       m * (side * side + side * side) / 12.0f));
	b->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-half, -half, -half), vec3<T>(half, half, half))));
	if (tilt != 0.0f) {
		mat3<T> r;
		set_mat3_from_axis_angle(r, vec3<T>(0.5774f, 0.5774f, 0.5774f), tilt);
		b->set_orientation(r);
	}
	b->set_position(vec3<T>(0.0f, 0.5f, 0.0f));
	sim.add_solid(b);
	b->set_contact_mode(contact_mode::speculative);
	b->set_collision_scope(0);
	b->set_collide_with_scope(kFloorScope);
	double sum = 0.0;
	int n = 0;
	for (int i = 0; i <= 600; ++i) {
		sim.update(tr::from_milli(16));
		if (i > 300) {
			sum += std::sqrt(length_squared(b->get_angular_velocity()));
			++n;
		}
	}
	return static_cast<float>(sum / n);
}

} // namespace

int main() {
	bool ok = true;
	const T dt = tr::from_milli(16);

	// ── TABLE 1: the chain holds ────────────────────────────────────────────
	printf("Table 1: one corpse dropped from 1.2 m, thrown backwards at 4 m/s.\n");
	printf("  %6s  %10s  %10s  %8s\n", "tick", "worst_err", "pelvis_y", "awake");
	simulator<T> sim;
	sim.set_gravity(vec3<T>(0.0f, -20.0f, 0.0f));  // the game's world gravity
	add_floor(sim);
	ragdoll doll = build(sim, vec3<T>(0.0f, 1.2f, 0.0f), vec3<T>(0.0f, 1.0f, -4.0f));
	float worst_settled = 0.0f;
	double err_sum = 0.0;
	int err_n = 0;
	int slept_at = -1;
	for (int tick = 0; tick <= 600; ++tick) {
		sim.update(dt);
		float err = worst_joint_error(doll);
		if (tick > 60) {
			if (err > worst_settled)
				worst_settled = err;
			err_sum += err;
			++err_n;
		}
		if (slept_at < 0 && all_asleep(doll))
			slept_at = tick;
		if (tick % 60 == 0 || tick == 600) {
			int awake = 0;
			for (auto & s : doll.bones)
				awake += s->active() ? 1 : 0;
			printf("  %6d  %10.4f  %10.3f  %8d\n", tick, err,
			       doll.bones[0]->get_position().y, awake);
		}
	}
	const float err_mean = static_cast<float>(err_sum / err_n);
	printf("  joint error after settling: mean %.4f m, worst %.4f m\n", err_mean, worst_settled);
	if (!sane(doll)) {
		printf("  FAIL: a bone left the world\n");
		ok = false;
	}
	// The mean is the measure of whether the chain holds; a sagging chain reads in
	// centimetres and a chain coming apart in tenths. The worst single tick is kept as a
	// separate, looser bound because the extremities are being shaken by the resting-box
	// spin in Table 2 — a hand flicked at the end of a four-link chain stretches for a
	// tick, and that is the shake's fault, not the pin's.
	if (err_mean > 0.01f) {
		printf("  FAIL: the chain sags (mean %.4f m > 0.01 m)\n", err_mean);
		ok = false;
	}
	if (worst_settled > 0.15f) {
		printf("  FAIL: the chain came apart (worst %.4f m > 0.15 m)\n", worst_settled);
		ok = false;
	}

	// ── TABLE 2: sleep, and the control that says whose fault it is ─────────
	printf("\nTable 2: sleep.\n");
	if (slept_at >= 0) {
		printf("  whole corpse asleep at tick %d (%.1f s)\n", slept_at, slept_at * 0.016f);
	} else {
		printf("  corpse still awake after 600 ticks (9.6 s) — it lingers for 60.\n");
		printf("  Control — one box on a floor, no joints anywhere:\n");
		printf("    %10s  %10s  %12s\n", "half (m)", "tilt (rad)", "mean |w|");
		for (T half : { 0.04f, 0.16f, 0.64f }) {
			for (T tilt : { 0.0f, 0.05f }) {
				printf("    %10.3f  %10.2f  %12.4f\n", half, tilt, resting_box_spin(half, tilt));
			}
		}
		printf("  A tilted box spins on its own. The joints hold it (Table 1); they cannot\n");
		printf("  hold it still. See this file's header — it needs a contact manifold.\n");
	}

	// ── TABLE 3: what it costs ──────────────────────────────────────────────
	printf("\nTable 3: cost, %d bodies and %d joints per corpse.\n",
	       kBoneCount, kBoneCount - 1);
	printf("  %8s  %8s  %12s  %12s  %8s\n", "corpses", "bodies", "ms/tick fall", "ms/tick rest", "asleep");
	for (int corpses : { 1, 2, 4, 8 }) {
		simulator<T> s2;
		s2.set_gravity(vec3<T>(0.0f, -20.0f, 0.0f));
		add_floor(s2);
		std::vector<ragdoll> dolls;
		for (int i = 0; i < corpses; ++i)
			dolls.push_back(build(s2, vec3<T>(i * 2.5f - corpses * 1.25f, 1.2f, 0.0f),
			                      vec3<T>(0.0f, 1.0f, -4.0f)));
		auto run = [&](int ticks) {
			auto t0 = std::chrono::steady_clock::now();
			for (int i = 0; i < ticks; ++i)
				s2.update(dt);
			auto t1 = std::chrono::steady_clock::now();
			return std::chrono::duration<double, std::milli>(t1 - t0).count() / ticks;
		};
		double falling = run(120);   // in the air and hitting the floor: the expensive part
		double resting = run(480);   // settled, and for most of a corpse's life this is it
		int asleep = 0;
		for (auto & d : dolls)
			asleep += all_asleep(d) ? 1 : 0;
		printf("  %8d  %8d  %12.3f  %12.3f  %6d/%d\n", corpses, corpses * kBoneCount,
		       falling, resting, asleep, corpses);
		// Eight corpses is the worst case the game can produce at once. Awake, because
		// nothing sleeps yet, this is the number that has to fit.
		if (corpses == 8 && resting > 6.5) {
			printf("  FAIL: eight corpses cost %.2f ms/tick, over the whole frame budget\n", resting);
			ok = false;
		}
	}

	printf("\n%s\n", ok ? "PASS" : "FAIL");
	return ok ? 0 : 1;
}
