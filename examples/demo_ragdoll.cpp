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
// ── TABLE 4: the corpse keeps its shape ─────────────────────────────────────
//
// Phase 13, and the reason there is a Phase 13. A pin is a ball-socket: it constrains
// POSITION and says nothing whatever about relative orientation, so a pin-only corpse's
// twenty joints are free 360-degree swivels and a neck folded to the knees satisfies every
// constraint in the system. Table 1 can read 6 mm of joint error while the body holding to
// it collapses into a heap.
//
// So the table reports the same drop twice, pins against cones, and three numbers: how far
// past its span the worst joint sits (a limit declared but not enforced reads in tens of
// degrees), how far the pins were pried open doing it (a limit is pure torque at the
// velocity level and a turn about the joint at the position level, so this must not move),
// and head-minus-pelvis — which is the one that actually answers "does it look like a body".
//
// ── TABLE 3: what it costs ──────────────────────────────────────────────────
//
// Milliseconds per tick against corpse count, now with and without the limits, because up
// to two extra angular rows per joint is not free until it has been measured. hop's whole-frame budget in Wizard Wars is
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
	// The joint limits, and the frame they are measured in. `dir` is the way the bone
	// runs at rest, which becomes the cone's axis — the game gets this for free because
	// a GoldSrc bone runs down its own local +X, but the boxes here are laid out in
	// world, so the demo has to say so. `swing`/`twist` are the spans in degrees, and
	// negative means no limit. `centre` aims the cone that many degrees OFF the rest
	// pose, about `bend`, which is what turns a symmetric cone into a one-sided hinge —
	// an elbow at centre 60 with a span of 60 travels 0 to 120 and never backwards.
	T dx, dy, dz;
	float swing, twist, centre;
	T bx, by, bz;
};

// A humanoid the size of a GoldSrc player: ~1.8 m standing, pelvis at 1.0.
//
// The spans are the starting table from plans/ragdoll_joint_limits.md, which is a table to
// start an argument with and not one to trust. The game tunes its own against the BAKED
// pose of a real player model; these only have to be plausible enough that the corpse below
// keeps a shape.
const bone_def kBones[] = {
	//  name         parent    centre x/y/z          half extents              bone dir       swing twist centre   bend axis
	{ "pelvis",     -1,  0.00f, 1.00f,  0.00f, 0.090f, 0.090f, 0.070f,  0, 1, 0,    -1,   -1,    0,   0, 0, 0 },
	{ "spine",       0,  0.00f, 1.18f,  0.00f, 0.100f, 0.090f, 0.070f,  0, 1, 0,    25,   25,    0,   0, 0, 0 },
	{ "chest",       1,  0.00f, 1.36f,  0.00f, 0.110f, 0.090f, 0.080f,  0, 1, 0,    25,   25,    0,   0, 0, 0 },
	{ "neck",        2,  0.00f, 1.52f,  0.00f, 0.050f, 0.050f, 0.050f,  0, 1, 0,    30,   30,    0,   0, 0, 0 },
	{ "head",        3,  0.00f, 1.66f,  0.00f, 0.090f, 0.100f, 0.090f,  0, 1, 0,    45,   40,    0,   0, 0, 0 },
	{ "l_clavicle",  2,  0.09f, 1.48f,  0.00f, 0.060f, 0.040f, 0.040f,  1, 0, 0,    20,   15,    0,   0, 0, 0 },
	{ "l_upperarm",  5,  0.28f, 1.46f,  0.00f, 0.120f, 0.050f, 0.050f,  1, 0, 0,    80,   60,    0,   0, 0, 0 },
	{ "l_forearm",   6,  0.52f, 1.46f,  0.00f, 0.120f, 0.045f, 0.045f,  1, 0, 0,    60,   10,   60,   0, 0, -1 },
	{ "l_hand",      7,  0.70f, 1.46f,  0.00f, 0.060f, 0.040f, 0.030f,  1, 0, 0,    45,   30,    0,   0, 0, 0 },
	{ "r_clavicle",  2, -0.09f, 1.48f,  0.00f, 0.060f, 0.040f, 0.040f, -1, 0, 0,    20,   15,    0,   0, 0, 0 },
	{ "r_upperarm",  9, -0.28f, 1.46f,  0.00f, 0.120f, 0.050f, 0.050f, -1, 0, 0,    80,   60,    0,   0, 0, 0 },
	{ "r_forearm",  10, -0.52f, 1.46f,  0.00f, 0.120f, 0.045f, 0.045f, -1, 0, 0,    60,   10,   60,   0, 0, 1 },
	{ "r_hand",     11, -0.70f, 1.46f,  0.00f, 0.060f, 0.040f, 0.030f, -1, 0, 0,    45,   30,    0,   0, 0, 0 },
	{ "l_thigh",     0,  0.09f, 0.78f,  0.00f, 0.070f, 0.200f, 0.070f,  0, -1, 0,   60,   30,    0,   0, 0, 0 },
	{ "l_calf",     13,  0.09f, 0.40f,  0.00f, 0.060f, 0.190f, 0.060f,  0, -1, 0,   60,    5,   60,   1, 0, 0 },
	{ "l_foot",     14,  0.09f, 0.06f,  0.04f, 0.050f, 0.050f, 0.110f,  0, 0, 1,    35,   15,    0,   0, 0, 0 },
	{ "l_toe",      15,  0.09f, 0.03f,  0.16f, 0.040f, 0.030f, 0.040f,  0, 0, 1,    20,   10,    0,   0, 0, 0 },
	{ "r_thigh",     0, -0.09f, 0.78f,  0.00f, 0.070f, 0.200f, 0.070f,  0, -1, 0,   60,   30,    0,   0, 0, 0 },
	{ "r_calf",     17, -0.09f, 0.40f,  0.00f, 0.060f, 0.190f, 0.060f,  0, -1, 0,   60,    5,   60,   1, 0, 0 },
	{ "r_foot",     18, -0.09f, 0.06f,  0.04f, 0.050f, 0.050f, 0.110f,  0, 0, 1,    35,   15,    0,   0, 0, 0 },
	{ "r_toe",      19, -0.09f, 0.03f,  0.16f, 0.040f, 0.030f, 0.040f,  0, 0, 1,    20,   10,    0,   0, 0, 0 },
};
const int kBoneCount = static_cast<int>(sizeof(kBones) / sizeof(kBones[0]));
const int kHeadBone = 4;
const int kDrops = 32;  // one drop is chaos; only the mean over many is a measurement

const float kDeg = 3.14159265f / 180.0f;

// The rotation that takes +X — hop's twist axis, and Bullet's — onto `d`.
quat<T> align_x_to(const vec3<T> & d) {
	const vec3<T> x(1.0f, 0.0f, 0.0f);
	vec3<T> axis;
	cross(axis, x, d);
	const T s = std::sqrt(length_squared(axis));
	const T c = dot(x, d);
	quat<T> q;
	if (s < 1e-6f) {
		if (c < 0.0f)  // antiparallel: any perpendicular axis will do
			set_quat_from_axis_angle(q, vec3<T>(0.0f, 0.0f, 1.0f), 3.14159265f);
		return q;
	}
	mul(axis, 1.0f / s);
	set_quat_from_axis_angle(q, axis, std::atan2(s, c));
	return q;
}

const int kFloorScope = 1;
const T kDensity = 900.0f;  // flesh, near enough

struct ragdoll {
	std::vector<std::shared_ptr<solid<T>>> bones;
	std::vector<std::shared_ptr<constraint<T>>> joints;
};

// One corpse, dropped in at `origin` with every bone carrying `launch`. `limited` builds
// the Phase 13 corpse: the same twenty pins, each with a cone and a twist span on it.
ragdoll build(simulator<T> & sim, const vec3<T> & origin, const vec3<T> & launch,
              bool limited = false) {
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
		if (limited) {
			// Every bone starts identity-oriented, so a frame written in world is a frame
			// written in either body's local space, and the rest pose is zero swing.
			const quat<T> aligned = align_x_to(vec3<T>(b.dx, b.dy, b.dz));
			c->set_frame_b(aligned);
			if (b.centre != 0.0f) {
				// Aim the parent's frame off the rest pose, so the rest pose sits at the
				// EDGE of the cone and the joint can only travel one way out of it.
				quat<T> off, centred;
				set_quat_from_axis_angle(off, vec3<T>(b.bx, b.by, b.bz), b.centre * kDeg);
				mul(centred, off, aligned);
				c->set_frame_a(centred);
			} else {
				c->set_frame_a(aligned);
			}
			c->set_swing_span(b.swing < 0.0f ? -1.0f : b.swing * kDeg);
			c->set_twist_span(b.twist < 0.0f ? -1.0f : b.twist * kDeg);
		}
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

// How far past its spans the worst joint in the corpse is sitting, in degrees. This is the
// number that says whether the limits are being ENFORCED, as opposed to merely declared.
void worst_limit_violation(const ragdoll & r, float & swing_out, float & twist_out) {
	swing_out = 0.0f;
	twist_out = 0.0f;
	for (auto & c : r.joints) {
		T swing {}, twist {};
		if (!c->measure_limits(swing, twist, 0.001f))
			continue;
		const T ss = c->get_swing_span();
		const T ts = c->get_twist_span();
		if (ss >= 0.0f && swing > ss && (swing - ss) / kDeg > swing_out)
			swing_out = (swing - ss) / kDeg;
		if (ts >= 0.0f && std::fabs(twist) > ts && (std::fabs(twist) - ts) / kDeg > twist_out)
			twist_out = (std::fabs(twist) - ts) / kDeg;
	}
}

// How far the corpse has actually TRAVELLED over a window, as the worst bone's displacement
// from where it was `window` ticks ago. Not speed: a bone balanced on a box corner is handed
// a few rad/s every tick for as long as it lies there (see the header), so an instantaneous
// speed never reads still and never will until hop grows contact manifolds. Displacement
// sees through that — a shivering bone stays where it is, and a snaking one does not, which
// is also the difference the eye makes.
float worst_travel(const ragdoll & r, const std::vector<vec3<T>> & then) {
	float worst = 0.0f;
	for (size_t i = 0; i < r.bones.size(); ++i) {
		const float d = std::sqrt(length_squared(r.bones[i]->get_position(), then[i]));
		if (d > worst)
			worst = d;
	}
	return worst;
}

void snapshot(const ragdoll & r, std::vector<vec3<T>> & out) {
	out.resize(r.bones.size());
	for (size_t i = 0; i < r.bones.size(); ++i)
		out[i] = r.bones[i]->get_position();
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

	// ── TABLE 4: the corpse keeps its shape, and stops ──────────────────────
	printf("\nTable 4: limits. %d corpses, each thrown differently, 300 ticks each.\n", kDrops);
	printf("  %10s  %11s  %11s  %9s  %11s  %11s  %9s\n",
	       "joints", "head-pelvis", "settled", "crawl", "swing rest", "twist rest", "worst pin");
	float limited_span = 0.0f;
	float pins_span = 0.0f;
	float limited_rest_swing = 0.0f;
	float limited_rest_twist = 0.0f;
	float limited_settle = 0.0f;
	float limited_crawl = 0.0f;
	float pins_settle = 0.0f;
	float pins_crawl = 0.0f;
	float limited_pin = 0.0f;
	for (int pass = 0; pass < 2; ++pass) {
		const bool limited = pass == 1;
		double span_sum = 0.0, swing_sum = 0.0, twist_sum = 0.0, settle_sum = 0.0, crawl_sum = 0.0;
		float worst_pin = 0.0f;
		int never = 0;
		for (int drop = 0; drop < kDrops; ++drop) {
			simulator<T> s3;
			s3.set_gravity(vec3<T>(0.0f, -20.0f, 0.0f));
			add_floor(s3);
			// A spread of deaths: dropped on his feet, shoved, and flung. One drop is not a
			// measurement — a ragdoll on a floor is chaos, and a single corpse's settle time
			// swings by seconds on a change that means nothing.
			const float turn = drop * 1.7f;
			ragdoll d = build(s3, vec3<T>(0.0f, 0.05f + 0.35f * (drop % 3), 0.0f),
			                  vec3<T>(1.6f * std::cos(turn) * (drop % 4),
			                          0.4f * (drop % 2),
			                          1.6f * std::sin(turn) * (drop % 4)), limited);
			std::vector<vec3<T>> mark;
			snapshot(d, mark);
			int settled_at = -1;
			double late_travel = 0.0;
			int late_windows = 0;
			float rest_swing = 0.0f, rest_twist = 0.0f;
			for (int tick = 0; tick <= 300; ++tick) {
				s3.update(dt);
				const float pin = worst_joint_error(d);
				if (tick > 60 && pin > worst_pin)
					worst_pin = pin;
				if (tick % 30 == 29) {
					const float travel = worst_travel(d, mark);
					if (travel > 0.05f)
						settled_at = -1;
					else if (settled_at < 0)
						settled_at = tick - 29;
					if (tick > 200) {
						late_travel += travel;
						++late_windows;
					}
					snapshot(d, mark);
				}
			}
			// The settled pose is the one that matters, so it is measured once, at the end,
			// rather than 301 times and thrown away 300 of them — every call decomposes
			// every joint.
			worst_limit_violation(d, rest_swing, rest_twist);
			crawl_sum += late_travel / late_windows;
			span_sum += std::sqrt(length_squared(d.bones[kHeadBone]->get_position(),
			                                     d.bones[0]->get_position()));
			swing_sum += rest_swing;
			twist_sum += rest_twist;
			if (settled_at >= 0)
				settle_sum += settled_at * 0.016;
			else
				++never;
		}
		const float span = static_cast<float>(span_sum / kDrops);
		const float settle = (never < kDrops)
		    ? static_cast<float>(settle_sum / (kDrops - never)) : 0.0f;
		char when[24];
		if (never == kDrops)
			snprintf(when, sizeof(when), "never");
		else if (never)
			snprintf(when, sizeof(when), "%.2f s (%d never)", settle, never);
		else
			snprintf(when, sizeof(when), "%.2f s", settle);
		const float crawl = static_cast<float>(crawl_sum / kDrops);
		printf("  %10s  %11.3f  %11s  %7.3f m  %8.1f deg  %8.1f deg  %9.4f\n",
		       limited ? "limited" : "pins only", span, when, crawl,
		       static_cast<float>(swing_sum / kDrops), static_cast<float>(twist_sum / kDrops),
		       worst_pin);
		if (limited) {
			limited_crawl = crawl;
			limited_span = span;
			limited_rest_swing = static_cast<float>(swing_sum / kDrops);
			limited_rest_twist = static_cast<float>(twist_sum / kDrops);
			limited_settle = (never == kDrops) ? 99.0f : settle;
			limited_pin = worst_pin;
		} else {
			pins_span = span;
			pins_crawl = crawl;
			pins_settle = (never == kDrops) ? 99.0f : settle;
		}
	}
	// The limits have to be ENFORCED, not merely declared. The bar is loose on purpose: a
	// forearm pinned under a torso against the floor is outside its cone and physically
	// cannot get back in, so a settled corpse carries real residual violation and always
	// will. What this catches is the failure that matters — a limit that is not being
	// solved at all reads in three figures, not two.
	if (limited_rest_swing > 30.0f || limited_rest_twist > 35.0f) {
		printf("  FAIL: joints settle %.1f/%.1f deg past their spans\n",
		       limited_rest_swing, limited_rest_twist);
		ok = false;
	}
	// And they must not do it by prying the pins open. A limit is a pure torque, so the
	// pin underneath it should read much as it does in Table 1.
	if (limited_pin > 0.15f) {
		printf("  FAIL: the limits opened the pins (%.4f m)\n", limited_pin);
		ok = false;
	}
	printf("  A corpse keeps its spine: %.3f m head to pelvis, against %.3f m on pins\n",
	       limited_span, pins_span);
	printf("  alone and %.3f m standing.\n", 0.66f);
	if (limited_span < 0.50f) {
		printf("  FAIL: the limited corpse folded into a heap (%.3f m)\n", limited_span);
		ok = false;
	}
	// And it has to STOP. A corpse still crawling seconds after it lands reads as alive,
	// and it is also 21 bodies still on the physics bill. Nothing ever comes fully to rest
	// — every bone balances on a box corner that hands it a few rad/s a tick, which is not
	// this phase's to fix — so the bar is the pin-only corpse beside it: adding limits must
	// not add motion. It did once. A limit with a position pass drove a corpse across the
	// floor at 0.20 m per half-second against this 0.035 m, and two corpses in twelve ever
	// came to rest; that pass is gone and this is the test that keeps it gone.
	printf("  And it stops: %.3f m of crawl per half-second against %.3f m on pins, settling\n",
	       limited_crawl, pins_crawl);
	printf("  in %.2f s against %.2f s.\n", limited_settle, pins_settle);
	if (limited_crawl > pins_crawl * 1.5f + 0.01f) {
		printf("  FAIL: the limits drive the corpse (%.3f m per half-second, pins %.3f m)\n",
		       limited_crawl, pins_crawl);
		ok = false;
	}
	if (limited_settle > 3.0f) {
		printf("  FAIL: the corpse writhes for %.2f s before it settles\n", limited_settle);
		ok = false;
	}

	// ── TABLE 3: what it costs ──────────────────────────────────────────────
	printf("\nTable 3: cost, %d bodies and %d joints per corpse.\n",
	       kBoneCount, kBoneCount - 1);
	printf("  %8s  %8s  %9s  %12s  %12s  %8s\n",
	       "corpses", "bodies", "joints", "ms/tick fall", "ms/tick rest", "asleep");
	for (int pass = 0; pass < 2; ++pass) {
		const bool limited = pass == 1;
		for (int corpses : { 1, 2, 4, 8 }) {
			simulator<T> s2;
			s2.set_gravity(vec3<T>(0.0f, -20.0f, 0.0f));
			add_floor(s2);
			std::vector<ragdoll> dolls;
			for (int i = 0; i < corpses; ++i)
				dolls.push_back(build(s2, vec3<T>(i * 2.5f - corpses * 1.25f, 1.2f, 0.0f),
				                      vec3<T>(0.0f, 1.0f, -4.0f), limited));
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
			printf("  %8d  %8d  %9s  %12.3f  %12.3f  %6d/%d\n", corpses, corpses * kBoneCount,
			       limited ? "limited" : "pins", falling, resting, asleep, corpses);
			// Eight corpses is the worst case the game can produce at once. Awake, because
			// nothing sleeps yet, this is the number that has to fit.
			if (corpses == 8 && resting > 6.5) {
				printf("  FAIL: eight corpses cost %.2f ms/tick, over the whole frame budget\n", resting);
				ok = false;
			}
		}
	}

	printf("\n%s\n", ok ? "PASS" : "FAIL");
	return ok ? 0 : 1;
}
