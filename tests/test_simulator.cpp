#include <cassert>
#include <cmath>
#include <cstdio>
#include <hop/hop.h>

using namespace hop;

template <typename T> static void test_gravity_drop() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });

	// Create a solid with a sphere shape
	auto s = std::make_shared<solid<T>>();
	s->set_mass(tr::one());
	s->set_position({ T {}, T {}, tr::from_int(10) });
	s->set_collide_with_scope(0); // No collision, pure freefall

	auto sh = std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() });
	s->add_shape(sh);

	sim->add_solid(s);

	// Update for 1 second (1000ms) in 10ms steps
	for (int i = 0; i < 100; ++i) {
		sim->update(tr::from_milli(10));
	}

	// Expected: z ≈ 10 - 0.5 * 9.81 * 1^2 = 10 - 4.905 = 5.095
	float z = tr::to_float(s->get_position().z);
	printf("  gravity drop: z = %.3f (expected ~5.1)\n", z);
	assert(z > 4.0f && z < 6.5f);
	printf("  gravity drop: OK\n");
}

template <typename T> static void test_trigger_scope() {
	using tr = scalar_traits<T>;

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, T {} });

	// Damage zone: a sphere tagged with trigger bit 0x4. Has zero collide_with
	// so it's a pass-through volume — physics objects don't bounce off it.
	auto zone = std::make_shared<solid<T>>();
	zone->set_infinite_mass();
	zone->set_position({ T {}, T {}, T {} });
	zone->set_trigger_scope(0x4);
	zone->set_collide_with_scope(0); // pure trigger
	zone->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::from_int(2) }));
	sim->add_solid(zone);

	// Player: a small sphere we'll move into the zone.
	auto player = std::make_shared<solid<T>>();
	player->set_mass(tr::one());
	player->set_position({ tr::from_int(5), T {}, T {} });
	player->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::half() }));
	sim->add_solid(player);

	// Static overlap query: player at (5,0,0), zone at origin r=2 — no overlap.
	collision<T> r;
	segment<T> probe;
	probe.set_start_dir(player->get_position(), { T {}, T {}, T {} });
	sim->trace_solid(r, player.get(), probe, -1);
	assert(r.trigger_scope == 0);
	printf("  trigger_scope outside: trigger_scope=0x%x (expected 0)\n", r.trigger_scope);

	// Move player to origin — now overlapping the zone.
	player->set_position({ T {}, T {}, T {} });
	r.reset();
	probe.set_start_dir(player->get_position(), { T {}, T {}, T {} });
	sim->trace_solid(r, player.get(), probe, -1);
	assert(r.trigger_scope == 0x4);
	printf("  trigger_scope inside: trigger_scope=0x%x (expected 0x4)\n", r.trigger_scope);
	printf("  trigger_scope: OK\n");
}

// A manager that injects a static floor plane at world z=0 as EXTERNAL geometry
// (NOT a solid in the simulator's list). Exercises the speculative pipeline's
// manager-query discovery path: the discovered contact has no owning solid, so
// it can only be resolved against the simulator's immovable world anchor.
template <typename T> class manager_floor : public hop::manager<T> {
	using tr = scalar_traits<T>;

public:
	int find_solids_in_aa_box(const aa_box<T> &, solid<T> *[], int) override { return -1; }
	void trace_segment(collision<T> &, const segment<T> &, int) override {}
	void trace_solid(collision<T> & result, solid<T> * s, const segment<T> & seg, int, T margin) override {
		// Floor plane at z=0, inflated upward by the speculative margin so a body
		// resting within the margin registers as an overlap. Mirrors the recipe in
		// test_collision's traceable floor, but reports through the manager path.
		T surface_z = margin;
		T lowest_z = T {};
		for (auto & shape : s->get_shapes()) {
			aa_box<T> bound;
			shape->get_bound(bound);
			if (bound.mins.z < lowest_z)
				lowest_z = bound.mins.z;
		}
		T start_z = seg.origin.z + lowest_z;
		if (start_z <= surface_z) {
			if (result.time > T {}) {
				result.time = T {};
				result.point.set(seg.origin);
				result.normal = { T {}, T {}, tr::one() };
				result.depth = surface_z - start_z;
			}
			return;
		}
		T dz = seg.direction.z;
		if (dz >= T {})
			return;
		T t = (surface_z - start_z) / dz;
		if (t >= T {} && t <= tr::one() && t < result.time) {
			result.time = t;
			mul(result.point, seg.direction, t);
			add(result.point, seg.origin);
			result.normal = { T {}, T {}, tr::one() };
		}
	}
	void pre_update(T) override {}
	void post_update(T) override {}
	void pre_update(solid<T> *, T) override {}
	void intra_update(solid<T> *, T) override {}
	bool collision_response(solid<T> *, vec3<T> &, vec3<T> &, collision<T> &) override { return false; }
	void post_update(solid<T> *, T) override {}
};

// Drop a sphere onto manager-injected (non-solid) floor geometry with the
// speculative pipeline on. The contact is discovered only via the manager query
// in integrate_and_discover and resolved against the world anchor — verifies the
// body rests on the surface (no tunnelling, no sink) and settles to sleep, which
// can only happen if the world-anchor touch counts as load-bearing support.
template <typename T> static void test_speculative_manager_floor(const char * label) {
	using tr = scalar_traits<T>;
	printf("  speculative_manager_floor[%s]: ", label);

	manager_floor<T> floor;
	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	sim->set_manager(&floor);

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::from_int(5) });
	ball->set_coefficient_of_restitution(T {});  // no bounce: settle quickly
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	for (int i = 0; i < 300; ++i)
		sim->update(tr::from_milli(16));

	float z = tr::to_float(ball->get_position().z);
	float vz = tr::to_float(ball->get_velocity().z);
	bool asleep = !ball->active();
	printf("z=%.3f vz=%.4f asleep=%d (expected z~1.0)\n", z, vz, asleep ? 1 : 0);
	// Rests on the radius-1 sphere's contact with the z=0 plane: center near z=1.
	// Generous band to stay robust across float/fixed and the margin shell.
	assert(z > 0.9f && z < 1.15f);
	assert(std::fabs(vz) < 0.1f);
	assert(asleep);
	printf("  speculative_manager_floor[%s]: OK\n", label);
}

// Injects the same z=0 floor but CLAIMS every contact via collision_response.
// Verifies the speculative pipeline calls the hook and that a claimed contact is
// excluded from the solver: with no impulse applied, nothing stops the body.
template <typename T> class claiming_floor : public manager_floor<T> {
public:
	int response_calls = 0;
	bool collision_response(solid<T> *, vec3<T> &, vec3<T> &, collision<T> &) override {
		++response_calls;
		return true;  // claim it; do nothing, so the solver must not resolve it
	}
};

template <typename T> static void test_speculative_manager_response(const char * label) {
	using tr = scalar_traits<T>;
	printf("  speculative_manager_response[%s]: ", label);

	claiming_floor<T> floor;
	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	sim->set_manager(&floor);

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::from_int(3) });
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	for (int i = 0; i < 120; ++i)
		sim->update(tr::from_milli(16));

	float z = tr::to_float(ball->get_position().z);
	printf("response_calls=%d z=%.2f (expected hook fired, body fell through z<0)\n", floor.response_calls, z);
	assert(floor.response_calls > 0);  // the hook fires under the speculative pipeline
	assert(z < 0.0f);                  // claimed contact => no solver impulse => not stopped
	printf("  speculative_manager_response[%s]: OK\n", label);
}

// Mixed contact modes in one simulator: a finite-mass sweep_slide "character" is
// pushed by a speculative ball through the shared velocity solve. Verifies (a) the
// character is influenced by physics — it is shoved in +x from rest, the behavior a
// kinematic (inv_mass=0) character controller would NOT give — and (b) the ball
// does not tunnel through it. Exercises the per-body dispatch and the mixed-pair
// seam (sweep_slide owns its own position via the snap/slide; the impulse exchange
// still uses its real finite mass).
template <typename T> static void test_mixed_modes_push(const char * label) {
	using tr = scalar_traits<T>;
	printf("  mixed_modes_push[%s]: ", label);

	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, T {} });  // zero-g: isolate the push

	// Character: sweep_slide, finite mass, at rest at the origin.
	auto character = std::make_shared<solid<T>>();
	character->set_mass(tr::one());
	character->set_position({ T {}, T {}, T {} });
	character->set_coefficient_of_restitution(T {});  // inelastic: clean momentum transfer
	character->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(character);
	character->set_contact_mode(contact_mode::sweep_slide);

	// Ball: speculative, finite mass, approaching from -x.
	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ tr::from_int(-4), T {}, T {} });
	ball->set_velocity({ tr::from_int(5), T {}, T {} });
	ball->set_coefficient_of_restitution(T {});
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);
	ball->set_contact_mode(contact_mode::speculative);

	for (int i = 0; i < 120; ++i)
		sim->update(tr::from_milli(16));

	float cx  = tr::to_float(character->get_position().x);
	float cvx = tr::to_float(character->get_velocity().x);
	float bx  = tr::to_float(ball->get_position().x);
	printf("char x=%.2f vx=%.3f  ball x=%.2f (char shoved +x, ball stays behind)\n", cx, cvx, bx);
	assert(cx > 0.1f);    // the sweep_slide character was pushed by the speculative ball
	assert(cvx > 0.0f);   // ... and is still carrying that motion
	assert(bx < cx);      // the ball never tunnelled past the character
	printf("  mixed_modes_push[%s]: OK\n", label);
}

// Phase 6 (kinematic angular carry): a spinning infinite-mass platform must
// carry the rider resting on it. The platform spins about the vertical (z) axis;
// at the off-axis rider the surface velocity ω×r is tangential, so friction
// should drag the rider around the axis (ω×(r,0,·) = (0,ω·r,0) → +y first). The
// platform's spin is scripted carry only — Phase 6 does not integrate orientation
// from ω, so the geometry stays put and the term is isolated.
//
// The platform here is a large sphere (its near-flat cap is the floor); the
// box-top version of this carry lives in test_angular_carry_box. Both stay seated
// under fixed16 now — the sphere×box top-face tunnel that once forced the sphere
// platform here has been fixed (analytic sphere×box closest point, see collide.h).
template <typename T> static void test_angular_carry(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_carry[%s]: ", label);

	auto run = [](const vec3<T> & omega) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity({ T {}, T {}, -tr::from_int(10) });  // press the rider down for friction load

		// Platform: infinite mass, big sphere whose cap sits at z≈0, spun about z.
		// Inelastic + frictional so the rider settles and the surface can grip it.
		auto platform = std::make_shared<solid<T>>();
		platform->set_infinite_mass();
		platform->set_position({ T {}, T {}, -tr::from_int(40) });
		platform->set_coefficient_of_gravity(T {});
		platform->set_coefficient_of_restitution(T {});
		platform->set_coefficient_of_static_friction(tr::half());
		platform->set_coefficient_of_dynamic_friction(tr::half());
		platform->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::from_int(40) }));
		platform->set_angular_velocity(omega);
		sim->add_solid(platform);

		// Rider: finite-mass sphere resting on the cap, off-axis (lever arm ≈ +x).
		// Restitution 0 so it stays seated rather than bouncing off the spin.
		auto rider = std::make_shared<solid<T>>();
		rider->set_mass(tr::one());
		rider->set_position({ tr::from_int(3), T {}, tr::from_milli(600) });
		rider->set_coefficient_of_restitution(T {});
		rider->set_coefficient_of_static_friction(tr::half());
		rider->set_coefficient_of_dynamic_friction(tr::half());
		rider->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::half() }));
		sim->add_solid(rider);

		for (int i = 0; i < 100; ++i)
			sim->update(tr::from_milli(10));
		return rider->get_position();
	};

	// Spin about +z: ω×r at (3,0,·) points +y, so the rider is carried +y first.
	vec3<T> spun = run({ T {}, T {}, tr::one() });
	// Control: no spin → the rider stays put (no tangential drift).
	vec3<T> still = run({ T {}, T {}, T {} });

	float sx = tr::to_float(spun.x), sy = tr::to_float(spun.y), sz = tr::to_float(spun.z);
	float ty = tr::to_float(still.y), tz = tr::to_float(still.z);
	float r_spun = std::sqrt(sx * sx + sy * sy);
	printf("spun=(%.2f,%.2f,%.2f) r=%.2f  still_y=%.2f still_z=%.2f (carried +y, stays seated)\n",
	       sx, sy, sz, r_spun, ty, tz);

	assert(sy > 0.5f);             // the spinning platform dragged the rider tangentially (+y)
	assert(std::fabs(ty) < 0.2f);  // without spin it does not drift
	assert(sz > 0.2f);             // the rider stayed seated on the cap (did not tunnel through)
	assert(r_spun > 2.0f && r_spun < 4.0f);  // carried around the axis, still on the cap
	printf("  angular_carry[%s]: OK\n", label);
}

// Box-platform variant of the angular carry, and the regression for the fixed16
// sphere×box narrowphase bug: a finite-mass sphere rests on the flat top of a
// large, spinning, infinite-mass box and must be carried tangentially while
// staying seated. Pre-fix this tunnelled under fixed16 — the spin walks the
// contact point off the box's symmetry axes, where GJK's closest-point
// reconstruction lost the contact vector to fixed-point cancellation, so the
// resting contact vanished and the rider fell through. With the analytic
// sphere×box closest point it stays on the cap exactly as on a sphere platform.
template <typename T> static void test_angular_carry_box(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_carry_box[%s]: ", label);

	auto run = [](const vec3<T> & omega) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity({ T {}, T {}, -tr::from_int(10) });

		// Platform: large box, top face at z=0 (centre z=-10, half-extent 10 thick,
		// 40 wide). Infinite mass, spun about z, inelastic + frictional.
		auto platform = std::make_shared<solid<T>>();
		platform->set_infinite_mass();
		platform->set_position({ T {}, T {}, -tr::from_int(10) });
		platform->set_coefficient_of_gravity(T {});
		platform->set_coefficient_of_restitution(T {});
		platform->set_coefficient_of_static_friction(tr::half());
		platform->set_coefficient_of_dynamic_friction(tr::half());
		platform->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(-tr::from_int(40), -tr::from_int(40), -tr::from_int(10),
		              tr::from_int(40), tr::from_int(40), tr::from_int(10))));
		platform->set_angular_velocity(omega);
		sim->add_solid(platform);

		auto rider = std::make_shared<solid<T>>();
		rider->set_mass(tr::one());
		rider->set_position({ tr::from_int(3), T {}, tr::from_milli(600) });
		rider->set_coefficient_of_restitution(T {});
		rider->set_coefficient_of_static_friction(tr::half());
		rider->set_coefficient_of_dynamic_friction(tr::half());
		rider->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::half() }));
		sim->add_solid(rider);

		for (int i = 0; i < 100; ++i)
			sim->update(tr::from_milli(10));
		return rider->get_position();
	};

	vec3<T> spun = run({ T {}, T {}, tr::one() });
	vec3<T> still = run({ T {}, T {}, T {} });

	float sx = tr::to_float(spun.x), sy = tr::to_float(spun.y), sz = tr::to_float(spun.z);
	float ty = tr::to_float(still.y), tz = tr::to_float(still.z);
	float r_spun = std::sqrt(sx * sx + sy * sy);
	printf("spun=(%.2f,%.2f,%.2f) r=%.2f still_y=%.2f still_z=%.2f\n", sx, sy, sz, r_spun, ty, tz);

	assert(sy > 0.5f);            // dragged tangentially (+y) by the spinning box top
	assert(std::fabs(ty) < 0.2f); // no drift without spin
	assert(sz > 0.4f);            // stayed seated on the top face (did NOT tunnel)
	assert(std::fabs(tz - 0.5f) < 0.1f);     // rests at sphere radius above z=0
	assert(r_spun > 2.0f && r_spun < 4.0f);  // carried around the axis, still on top
	printf("  angular_carry_box[%s]: OK\n", label);
}

// Capsule-rider variant: an upright capsule on the spinning large box top. Exercises
// the analytic capsule×box (segment-vs-box) path end-to-end under fixed16 — it must
// keep the rider seated and carry it tangentially, just like the sphere rider.
template <typename T> static void test_angular_carry_box_capsule(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_carry_box_capsule[%s]: ", label);

	auto run = [](const vec3<T> & omega) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity({ T {}, T {}, -tr::from_int(10) });

		auto platform = std::make_shared<solid<T>>();
		platform->set_infinite_mass();
		platform->set_position({ T {}, T {}, -tr::from_int(10) });
		platform->set_coefficient_of_gravity(T {});
		platform->set_coefficient_of_restitution(T {});
		platform->set_coefficient_of_static_friction(tr::half());
		platform->set_coefficient_of_dynamic_friction(tr::half());
		platform->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(-tr::from_int(40), -tr::from_int(40), -tr::from_int(10),
		              tr::from_int(40), tr::from_int(40), tr::from_int(10))));
		platform->set_angular_velocity(omega);
		sim->add_solid(platform);

		// Upright capsule: spine 0..0.5 in z, radius 0.4 — bottom cap rests on z=0.
		auto rider = std::make_shared<solid<T>>();
		rider->set_mass(tr::one());
		rider->set_position({ tr::from_int(3), T {}, tr::from_milli(600) });
		rider->set_coefficient_of_restitution(T {});
		rider->set_coefficient_of_static_friction(tr::half());
		rider->set_coefficient_of_dynamic_friction(tr::half());
		capsule<T> c;
		c.set({ T {}, T {}, T {} }, { T {}, T {}, tr::half() }, tr::from_milli(400));
		rider->add_shape(std::make_shared<shape<T>>(c));
		sim->add_solid(rider);

		for (int i = 0; i < 100; ++i)
			sim->update(tr::from_milli(10));
		return rider->get_position();
	};

	vec3<T> spun = run({ T {}, T {}, tr::one() });
	vec3<T> still = run({ T {}, T {}, T {} });
	float sx = tr::to_float(spun.x), sy = tr::to_float(spun.y), sz = tr::to_float(spun.z);
	float ty = tr::to_float(still.y), tz = tr::to_float(still.z);
	float r_spun = std::sqrt(sx * sx + sy * sy);
	printf("spun=(%.2f,%.2f,%.2f) r=%.2f still_y=%.2f still_z=%.2f\n", sx, sy, sz, r_spun, ty, tz);

	assert(sy > 0.5f);            // carried tangentially (+y) by the spinning box top
	assert(std::fabs(ty) < 0.2f); // no drift without spin
	assert(sz > 0.3f);            // bottom cap stayed on the top face (did NOT tunnel)
	assert(r_spun > 2.0f && r_spun < 4.0f);
	printf("  angular_carry_box_capsule[%s]: OK\n", label);
}

// Statically-rotated boxes dropped on a flat floor must settle on their true
// rotated geometry (Phase 5 oriented polytope×polytope), not their world AABB, and
// the infinite-mass floor must not move. TWO boxes are used deliberately: with a
// single oriented box the broad-phase bug below is masked, because the floor's own
// per-tick recovery keeps the lone box up; with two boxes the floor can only
// recover the closest one each tick, exposing the real defects. Regression for the
// coupled bugs the oriented narrowphase exposed:
//   (a) the per-step broad-phase query box was built from the *un-rotated*
//       local_bound_, so an oriented box (reaching √2·half past its AABB) queried a
//       box too small to reach the floor and tunnelled until it sank deep enough —
//       then snapped back with injected energy. Fixed by querying the cached
//       orientation-aware world_bound_.
//   (b) the oriented sweep used trace_convex_solid's per-face entry, which misses
//       an edge/vertex contact; rewritten onto conservative_advance with the CSO
//       deepest-face distance.
//   (c) update_solid's penetration recovery moved an infinite-mass MOVER (the floor
//       pushing itself out of a resting box); it now never relocates infinite mass.
// A box rotated 45° about Y rests balanced on its lower edge with its center at
// √2/2 ≈ 0.707 (vs 0.5 if it collided as an AABB).
template <typename T> static void test_oriented_box_rest(const char * label) {
	using tr = scalar_traits<T>;
	printf("  oriented_box_rest[%s]: ", label);
	simulator<T> sim;

	auto floor = std::make_shared<solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	floor->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_int(6), -tr::from_int(6), -tr::one()),
	              vec3<T>(tr::from_int(6), tr::from_int(6), T {}))));
	sim.add_solid(floor);

	mat3<T> r;
	set_mat3_from_axis_angle(r, vec3<T>(T {}, tr::one(), T {}), tr::from_milli(785)); // ~45° about Y
	auto make_box = [&](T x, bool rotated) {
		auto b = std::make_shared<solid<T>>();
		b->set_mass(tr::one());
		b->set_coefficient_of_restitution(T {});
		b->set_coefficient_of_static_friction(tr::from_int(2));
		b->set_coefficient_of_dynamic_friction(tr::from_int(2));
		b->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
		if (rotated)
			b->set_orientation(r);
		b->set_position(vec3<T>(x, T {}, tr::from_int(2)));
		sim.add_solid(b);
		return b;
	};
	// A second body 4 m away (never touching) removes the single-box floor-recovery
	// crutch; the rotated box must self-support.
	make_box(-tr::from_int(2), false);
	auto box = make_box(tr::from_int(2), true);

	// Run long enough to exercise deactivation; track steady-state amplitude. The
	// jitter bug rested near the right height but the box periodically lost the (edge)
	// contact, free-fell several frames, and snapped back (>10 cm swings) — invisible
	// to a final-position check, so assert a tight steady-state band.
	float zmin = 1e9f, zmax = -1e9f, fmin = 1e9f, fmax = -1e9f;
	for (int i = 0; i < 800; ++i) {
		sim.update(tr::from_milli(16));
		if (i >= 300) {
			float z = tr::to_float(box->get_position().z);
			float f = tr::to_float(floor->get_position().z);
			zmin = std::fmin(zmin, z); zmax = std::fmax(zmax, z);
			fmin = std::fmin(fmin, f); fmax = std::fmax(fmax, f);
		}
	}
	printf("box.z=[%.3f,%.3f] floor.z=[%.3f,%.3f] ", zmin, zmax, fmin, fmax);
	assert(std::fabs(zmax - 0.7071f) < 0.03f); // rests on the rotated edge, not the AABB 0.5
	assert((zmax - zmin) < 0.01f);             // no fall/snap jitter (was >0.16)
	assert(std::fabs(fmin) < 0.01f && std::fabs(fmax) < 0.01f); // infinite-mass floor never moves
	printf("OK\n");
}

// Phase 8: a finite-inertia body integrates orientation under torque and free spin,
// while a body with no inertia (inv_inertia == 0, the default) never rotates
// dynamically. No collision response yet — these bodies don't collide
// (collide_with_scope 0), they just spin.
template <typename T> static void test_dynamic_spin(const char * label) {
	using tr = scalar_traits<T>;
	printf("  dynamic_spin[%s]: ", label);
	const T z {};
	auto make = [&](const vec3<T> & inertia, const vec3<T> & w0) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity(vec3<T>(z, z, z));
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		if (inertia.x > z) s->set_inertia(inertia);
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
		s->set_collide_with_scope(0);
		s->set_angular_velocity(w0);
		sim->add_solid(s);
		return std::make_pair(sim, s);
	};
	auto spin_z = [](const std::shared_ptr<solid<T>> & s) {
		const auto & R = s->get_orientation();
		return std::atan2(tr::to_float(R.at(1, 0)), tr::to_float(R.at(0, 0)));
	};

	// (1) Free spin: 2 rad/s about Z, no torque → ω constant, angle advances ω·t.
	{
		auto [sim, s] = make(vec3<T>(tr::one(), tr::one(), tr::one()), vec3<T>(z, z, tr::two()));
		for (int i = 0; i < 100; ++i) sim->update(tr::from_milli(16)); // 1.6 s → 3.2 rad ≡ -3.083
		float wz = tr::to_float(s->get_angular_velocity().z);
		printf("free wz=%.2f ang=%.2f ", wz, spin_z(s));
		assert(std::fabs(wz - 2.0f) < 0.1f);                 // ω unchanged (no torque)
		assert(std::fabs(spin_z(s) - (-3.083f)) < 0.2f);     // 3.2 rad wrapped
	}
	// (2) Spin-up: τ=4 about Z, I=2 → dω=2 rad/s² → ω≈3.2 after 1.6 s.
	{
		auto [sim, s] = make(vec3<T>(tr::two(), tr::two(), tr::two()), vec3<T>(z, z, z));
		for (int i = 0; i < 100; ++i) { s->add_torque(vec3<T>(z, z, tr::from_int(4))); sim->update(tr::from_milli(16)); }
		float wz = tr::to_float(s->get_angular_velocity().z);
		printf("spinup wz=%.2f ", wz);
		assert(std::fabs(wz - 3.2f) < 0.2f);
	}
	// (3) Opt-out: no inertia (inv_inertia 0) → never spins despite torque + set ω.
	{
		auto [sim, s] = make(vec3<T>(z, z, z), vec3<T>(z, z, tr::from_int(5)));
		for (int i = 0; i < 50; ++i) { s->add_torque(vec3<T>(z, z, tr::from_int(10))); s->set_angular_velocity(vec3<T>(z, z, tr::from_int(5))); sim->update(tr::from_milli(16)); }
		printf("optout ang=%.4f ", spin_z(s));
		assert(std::fabs(spin_z(s)) < 0.01f); // orientation never changed
	}
	// (4) Cap: runaway torque → |ω| clamped to default_max_angular_velocity_component.
	{
		auto [sim, s] = make(vec3<T>(tr::one(), tr::one(), tr::one()), vec3<T>(z, z, z));
		for (int i = 0; i < 100; ++i) { s->add_torque(vec3<T>(z, z, tr::from_int(1000))); sim->update(tr::from_milli(16)); }
		float wz = tr::to_float(s->get_angular_velocity().z);
		float cap = tr::to_float(tr::default_max_angular_velocity_component());
		printf("cap wz=%.1f ", wz);
		assert(std::fabs(wz - cap) < 1.0f);
	}
	printf("OK\n");
}

// Phase 9: an off-center impact transfers linear momentum into spin (lever arm),
// while a centered impact produces ~none. A projectile (no inertia, so it can't
// spin) strikes a free finite-inertia box; the +y-offset hit pushing +x torques the
// box clockwise about Z (ω.z < 0), and the box's linear speed is lower than the
// centered case because energy went into rotation.
template <typename T> static void test_angular_impulse(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_impulse[%s]: ", label);
	const T z {};
	auto run = [&](T yoff) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity(vec3<T>(z, z, z));
		auto A = std::make_shared<solid<T>>();
		A->set_mass(tr::one());
		A->set_inertia(vec3<T>(tr::one(), tr::one(), tr::one()));
		A->set_coefficient_of_restitution(tr::half());
		A->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
		sim->add_solid(A);
		auto B = std::make_shared<solid<T>>(); // no inertia → B never spins
		B->set_mass(tr::one());
		B->set_coefficient_of_restitution(tr::half());
		B->add_shape(std::make_shared<shape<T>>(sphere<T>(vec3<T>(z, z, z), tr::from_milli(300))));
		B->set_position(vec3<T>(-tr::two(), yoff, z));
		B->set_velocity(vec3<T>(tr::from_int(8), z, z));
		sim->add_solid(B);
		for (int i = 0; i < 60; ++i) sim->update(tr::from_milli(16));
		return std::make_pair(tr::to_float(A->get_velocity().x), tr::to_float(A->get_angular_velocity().z));
	};
	auto c = run(z);                 // centered
	auto o = run(tr::from_milli(400)); // +0.4 in y
	printf("centered(vx=%.2f wz=%.3f) offcenter(vx=%.2f wz=%.3f) ", c.first, c.second, o.first, o.second);
	assert(c.first > 1.0f);              // box was pushed
	assert(std::fabs(c.second) < 0.15f); // centered → ~no spin
	assert(o.second < -0.5f);            // off-center → real clockwise spin about z
	assert(o.first < c.first);           // energy went into rotation → less linear speed
	printf("OK\n");
}

// Phase 9: friction at a contact below the center of mass torques the body — a box
// sliding along the floor decelerates AND tips forward (acquires ω about the axis
// perpendicular to motion), the start of rolling. Exercises the angular friction
// (tangent effective-mass) path.
template <typename T> static void test_friction_rolling(const char * label) {
	using tr = scalar_traits<T>;
	printf("  friction_rolling[%s]: ", label);
	simulator<T> sim; // default gravity −Z
	auto floor = std::make_shared<solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	floor->set_coefficient_of_static_friction(tr::one());
	floor->set_coefficient_of_dynamic_friction(tr::one());
	floor->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_int(6), -tr::from_int(6), -tr::one()), vec3<T>(tr::from_int(6), tr::from_int(6), T {}))));
	sim.add_solid(floor);
	auto s = std::make_shared<solid<T>>();
	s->set_mass(tr::one());
	s->set_inertia(vec3<T>(tr::one(), tr::one(), tr::one()));
	s->set_coefficient_of_restitution(T {});
	s->set_coefficient_of_static_friction(tr::one());
	s->set_coefficient_of_dynamic_friction(tr::one());
	s->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()), vec3<T>(tr::half(), tr::half(), tr::half()))));
	s->set_position(vec3<T>(T {}, T {}, tr::half()));
	s->set_velocity(vec3<T>(tr::from_int(5), T {}, T {})); // slide +x
	sim.add_solid(s);
	for (int i = 0; i < 30; ++i) sim.update(tr::from_milli(16));
	float vx = tr::to_float(s->get_velocity().x);
	float wy = tr::to_float(s->get_angular_velocity().y);
	printf("vx=%.2f wy=%.3f ", vx, wy);
	assert(vx < 4.0f);  // friction decelerated it
	assert(wy > 0.25f); // and torqued it into a forward roll about +y
	printf("OK\n");
}

// Phase 10: a spring whose anchor sits off the body's center torques the body via
// its lever arm (τ = r × F). An off-center pull spins the body about +z; a centered
// pull (lever = 0) produces pure translation and no spin. Exercises rotated anchors
// + accumulate_constraint_torque, and the bit-identical center-anchor fast path.
template <typename T> static void test_constraint_anchor_torque(const char * label) {
	using tr = scalar_traits<T>;
	printf("  constraint_anchor_torque[%s]: ", label);
	const T z {};
	auto run = [&](T anchor_x) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity(vec3<T>(z, z, z));
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		s->set_inertia(vec3<T>(tr::one(), tr::one(), tr::one()));
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::one(), -tr::one(), -tr::half()), vec3<T>(tr::one(), tr::one(), tr::half()))));
		s->set_position(vec3<T>(z, z, z));
		sim->add_solid(s);
		// Spring pulls the anchor toward a point 1 unit in +y; rest length 0.5 keeps
		// it stretched (force ≈ +y at the anchor). end_point shares anchor_x so the
		// pull is purely +y in both cases — only the lever arm differs.
		auto c = std::make_shared<constraint<T>>(s, vec3<T>(anchor_x, tr::one(), z));
		c->set_type(constraint<T>::type::spring);
		c->set_local_anchor_a(vec3<T>(anchor_x, z, z));
		c->set_rest_length(tr::half());
		c->set_spring_constant(tr::from_int(20));
		c->set_damping_constant(z);
		sim->add_constraint(c);
		for (int i = 0; i < 12; ++i) sim->update(tr::from_milli(16));
		return tr::to_float(s->get_angular_velocity().z);
	};
	float centered = run(z);
	float offcenter = run(tr::from_milli(700)); // anchor at +0.7 x
	printf("centered(wz=%.3f) offcenter(wz=%.3f) ", centered, offcenter);
	assert(std::fabs(centered) < 0.05f); // centered pull through COM → no spin
	assert(offcenter > 0.2f);            // off-center pull → real spin about +z
	printf("OK\n");
}

// Phase 9 hardening: a fast/thin spinner must not tunnel through a thin wall between
// orientation snapshots. A blade (±2 long) spinning at 40 rad/s sweeps a tip 1.28
// units/step against a 0.2-thick slab — classic angular-tunnel setup. The broad-phase
// inflation (|ω|·dt·r) + Phase 5 oriented narrowphase + Phase 9 angular response catch
// it: pinned at center so it can't recoil, the tip is still stopped at the near face
// and the spin is arrested. Guards against regressing any of those three mechanisms
// (the deferred end-of-step SAT recovery proved redundant against this case).
template <typename T> static void test_fast_spinner_no_tunnel(const char * label) {
	using tr = scalar_traits<T>;
	printf("  fast_spinner_no_tunnel[%s]: ", label);
	const T z {};
	simulator<T> sim;
	sim.set_gravity(vec3<T>(z, z, z));
	auto wall = std::make_shared<solid<T>>();
	wall->set_infinite_mass();
	wall->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(tr::one(), -tr::from_int(3), -tr::from_int(3)),
	              vec3<T>(tr::from_milli(1200), tr::from_int(3), tr::from_int(3)))));
	sim.add_solid(wall);
	auto blade = std::make_shared<solid<T>>();
	blade->set_mass(tr::one());
	blade->set_inertia(vec3<T>(tr::one(), tr::one(), tr::from_milli(200)));
	blade->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_int(2), -tr::from_milli(50), -tr::half()),
	              vec3<T>(tr::from_int(2), tr::from_milli(50), tr::half()))));
	blade->set_angular_velocity(vec3<T>(z, z, tr::from_int(40)));
	sim.add_solid(blade);
	auto pin = std::make_shared<constraint<T>>(blade, vec3<T>(z, z, z)); // center pinned
	pin->set_type(constraint<T>::type::spring);
	pin->set_rest_length(z);
	pin->set_spring_constant(tr::from_int(200));
	pin->set_damping_constant(tr::from_int(5));
	sim.add_constraint(pin);
	float max_tipx = 0.0f;
	for (int i = 0; i < 200; ++i) {
		sim.update(tr::from_milli(16));
		vec3<T> tip;
		mul(tip, blade->get_orientation(), vec3<T>(tr::from_int(2), z, z));
		float tipx = tr::to_float(tip.x) + tr::to_float(blade->get_position().x);
		if (tipx > max_tipx) max_tipx = tipx;
	}
	float wz = tr::to_float(blade->get_angular_velocity().z);
	printf("max_tip_x=%.3f final_wz=%.2f ", max_tipx, wz);
	assert(max_tipx < 1.1f);        // tip stopped at the wall, never swept past it
	assert(std::fabs(wz) < 5.0f);   // spin arrested from 40 rad/s
	printf("OK\n");
}

template <typename T> static void test_dual_instantiation() {
	// Just verify both can be instantiated in the same TU
	simulator<T> sim;
	solid<T> s;
	shape<T> sh;
	constraint<T> c;
	collision<T> col;
	printf("  dual instantiation: OK\n");
}

int main() {
	printf("test_simulator (float):\n");
	test_gravity_drop<float>();
	test_trigger_scope<float>();
	test_speculative_manager_floor<float>("float");
	test_speculative_manager_response<float>("float");
	test_mixed_modes_push<float>("float");
	test_angular_carry<float>("float");
	test_angular_carry_box<float>("float");
	test_angular_carry_box_capsule<float>("float");
	test_oriented_box_rest<float>("float");
	test_dynamic_spin<float>("float");
	test_angular_impulse<float>("float");
	test_friction_rolling<float>("float");
	test_constraint_anchor_torque<float>("float");
	test_fast_spinner_no_tunnel<float>("float");
	test_dual_instantiation<float>();

	printf("test_simulator (fixed16):\n");
	test_dual_instantiation<fixed16>();
	// Note: fixed16 gravity drop is more sensitive to overflow
	// with large position values, so we test it carefully
	{
		using tr = scalar_traits<fixed16>;
		auto sim = std::make_shared<simulator<fixed16>>();
		auto s = std::make_shared<solid<fixed16>>();
		s->set_mass(tr::one());
		s->set_position({ fixed16 {}, fixed16 {}, tr::from_int(10) });
		s->set_collide_with_scope(0);

		auto sh = std::make_shared<shape<fixed16>>(hop::sphere<fixed16> { vec3<fixed16> {}, tr::one() });
		s->add_shape(sh);
		sim->add_solid(s);

		for (int i = 0; i < 100; ++i) {
			sim->update(tr::from_milli(10));
		}
		float z = tr::to_float(s->get_position().z);
		printf("  fixed16 gravity drop: z = %.3f (expected ~5.1)\n", z);
		assert(z > 3.0f && z < 7.0f); // More relaxed bounds for fixed16
	}
	test_speculative_manager_floor<fixed16>("fixed16");
	test_speculative_manager_response<fixed16>("fixed16");
	test_mixed_modes_push<fixed16>("fixed16");
	test_angular_carry<fixed16>("fixed16");
	test_angular_carry_box<fixed16>("fixed16");
	test_angular_carry_box_capsule<fixed16>("fixed16");
	test_oriented_box_rest<fixed16>("fixed16");
	test_dynamic_spin<fixed16>("fixed16");
	test_angular_impulse<fixed16>("fixed16");
	test_friction_rolling<fixed16>("fixed16");
	test_constraint_anchor_torque<fixed16>("fixed16");
	test_fast_spinner_no_tunnel<fixed16>("fixed16");

	printf("ALL PASSED\n");
	return 0;
}
