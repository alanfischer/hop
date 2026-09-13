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
	int find_solids_in_aa_box(const aa_box<T> &, solid<T> *[], int, int) override { return -1; }
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

// A bouncy ball must run out of bounces. Gravity is integrated before the contact
// solve, so the tick's own g*dt rides inside the relative normal velocity the
// restitution target is built from — restitution then hands back cor*g*dt of velocity
// the ball never had, every bounce, forever. That converges on a fixed point at
// v = cor*g*dt/(1-cor): below cor ~0.86 (at 60Hz) it lands under the micro-collision
// threshold and the ball settles anyway, which is why this only ever showed up on the
// bounciest bodies. At cor 0.9 the fixed point is above the threshold and the ball
// hovers there forever, bouncing a couple of centimetres and never sleeping.
template <typename T> static void test_bouncy_ball_settles(const char * label) {
	using tr = scalar_traits<T>;
	printf("  bouncy_ball_settles[%s]: ", label);

	manager_floor<T> floor;
	auto sim = std::make_shared<simulator<T>>();
	sim->set_gravity({ T {}, T {}, -tr::from_milli(9810) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);
	sim->set_manager(&floor);

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::from_int(5) });
	ball->set_coefficient_of_restitution(tr::from_milli(900));
	// max, not the default average: the manager floor has no owning solid, so the
	// partner is the world anchor and its stock cor 0.5 would average this down to 0.7
	// — under the fixed point, where the ball settles either way and proves nothing.
	ball->set_restitution_combine(hop::restitution_combine::maximum);
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	// Long enough for a cor 0.9 ball dropped 4m to bounce itself out. Each bounce
	// keeps 90% of its speed, so the ~21 bounces down to the micro-collision
	// threshold take ~19s of flight; the tail after that must be flat.
	// Amplitude, not height: the resting centre sits a slop gap above z=1, and it is
	// the OSCILLATION that says the ball never ran out of bounces.
	float hi = -1e9f, lo = 1e9f;
	for (int i = 0; i < 2400; ++i) {
		sim->update(tr::from_milli(16));
		if (i >= 1800) {
			float z_i = tr::to_float(ball->get_position().z);
			if (z_i > hi) hi = z_i;
			if (z_i < lo) lo = z_i;
		}
	}
	float peak_late = hi - lo;

	float z = tr::to_float(ball->get_position().z);
	bool asleep = !ball->active();
	printf("z=%.3f late_swing=%.5f asleep=%d\n", z, peak_late, asleep ? 1 : 0);
	assert(z > 0.9f && z < 1.15f);
	assert(peak_late < 0.002f);  // no perpetual hop
	assert(asleep);
	printf("  bouncy_ball_settles[%s]: OK\n", label);
}

// A body that comes to rest ON a real static solid must sleep, wherever in the
// resting band it stopped. Two things used to keep it awake, both traceable to the
// same g·dt: the wake rule read the closing speed gravity re-manufactures every tick
// (0.16 m/s here, over deactivate_speed_) as an impact and had the resting body and
// its floor wake each other in turn; and a body settling at exactly the slop gap
// reads a rounding-hair above it, so the gap-only support test called a contact
// bearing its full weight "thin air". Started at rest inside the band so the landing
// transient can't paper over either — a ball that bounces in lands deeper.
template <typename T> static void test_resting_body_sleeps(const char * label, float start_gap) {
	using tr = scalar_traits<T>;
	printf("  resting_body_sleeps[%s gap=%.4f]: ", label, start_gap);

	auto sim = std::make_shared<simulator<T>>();
	// 20 m/s2 (a game's gravity, and WizardWars' own) on purpose: g*dt is then 0.33 m/s
	// at this 16ms step, above deactivate_speed_, which is what turned the resting
	// body's re-manufactured closing speed into a wake-up. Under ~12.5 m/s2 the tick's
	// increment stays below the threshold and nothing here has anything to prove.
	sim->set_gravity({ T {}, T {}, -tr::from_int(20) });
	sim->set_default_contact_mode(hop::contact_mode::speculative);

	// Static floor, top face at z=0, asleep as a static body is.
	auto floor_solid = std::make_shared<solid<T>>();
	floor_solid->set_infinite_mass();
	floor_solid->set_coefficient_of_gravity(T {});
	floor_solid->set_position({ T {}, T {}, -tr::half() });
	floor_solid->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(-tr::from_int(20), -tr::from_int(20), -tr::half(),
	              tr::from_int(20), tr::from_int(20), tr::half())));
	sim->add_solid(floor_solid);
	floor_solid->deactivate();

	auto ball = std::make_shared<solid<T>>();
	ball->set_mass(tr::one());
	ball->set_position({ T {}, T {}, tr::one() + tr::from_milli((int)(start_gap * 1000.0f)) });
	ball->set_coefficient_of_restitution(T {});
	ball->add_shape(std::make_shared<shape<T>>(hop::sphere<T> { vec3<T> {}, tr::one() }));
	sim->add_solid(ball);

	int slept_at = -1;
	for (int i = 0; i < 400; ++i) {
		sim->update(tr::from_milli(16));
		if (slept_at < 0 && !ball->active())
			slept_at = i;
	}

	float z = tr::to_float(ball->get_position().z);
	printf("z=%.4f slept_at=%d\n", z, slept_at);
	assert(z > 0.95f && z < 1.05f);  // still sitting on the floor, not sunk or launched
	assert(slept_at >= 0);
	printf("  resting_body_sleeps[%s gap=%.4f]: OK\n", label, start_gap);
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

// Regression: a kinematic spinner must carry a rider that ALSO spins dynamically.
// The riders above are inertia-less (inv_inertia==0), so the pair takes the linear
// v_bias carry path. Give the rider finite inertia and it starts rotating
// dynamically — flipping the pair onto the angular vrel path, where the platform's
// ω×r was being dropped because it was gated on rotates_dynamically() (false for an
// infinite-mass platform) instead of "has angular velocity". The carry then
// vanished. This is exactly the demo_rotating_platform scenario (its riders have
// inertia so friction spins them up). The rider must still be carried +y AND, as
// the Phase 9 secondary effect, spin up about +z from that same friction.
template <typename T> static void test_angular_carry_finite_inertia_rider(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_carry_finite_inertia_rider[%s]: ", label);

	auto run = [](const vec3<T> & omega) {
		auto sim = std::make_shared<simulator<T>>();
		sim->set_gravity({ T {}, T {}, -tr::from_int(10) });

		// Platform: wide, shallow box, top face at z=0, infinite mass, spun about z.
		// Kept shallow (centre z=-0.5) so the platform's lever arm to the contact has
		// no large z-component — a deep centre injects fixed16 rounding into ω×r.
		auto platform = std::make_shared<solid<T>>();
		platform->set_infinite_mass();
		platform->set_position({ T {}, T {}, -tr::half() });
		platform->set_coefficient_of_gravity(T {});
		platform->set_coefficient_of_restitution(T {});
		platform->set_coefficient_of_static_friction(tr::one());
		platform->set_coefficient_of_dynamic_friction(tr::one());
		platform->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(-tr::from_int(20), -tr::from_int(20), -tr::half(),
		              tr::from_int(20), tr::from_int(20), tr::half())));
		platform->set_angular_velocity(omega);
		sim->add_solid(platform);

		// Rider: finite mass AND finite inertia → rotates_dynamically() is true, so the
		// pair uses the angular vrel path. High friction so the carry grips.
		auto rider = std::make_shared<solid<T>>();
		rider->set_mass(tr::one());
		rider->set_inertia({ tr::from_milli(167), tr::from_milli(167), tr::from_milli(167) });
		rider->set_position({ tr::from_int(3), T {}, tr::half() + tr::from_milli(20) });
		rider->set_coefficient_of_restitution(T {});
		rider->set_coefficient_of_static_friction(tr::one());
		rider->set_coefficient_of_dynamic_friction(tr::one());
		rider->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(-tr::half(), -tr::half(), -tr::half(), tr::half(), tr::half(), tr::half())));
		sim->add_solid(rider);

		// dt=16ms matches demo_rotating_platform. (fixed16 carry of a box rider WITH
		// inertia rounds the friction impulse to zero at dt=10ms — a precision edge of
		// the small step, not of this carry path; the demo's step is well-behaved.)
		for (int i = 0; i < 100; ++i)
			sim->update(tr::from_milli(16));
		return std::make_pair(rider->get_position(), rider->get_angular_velocity());
	};

	auto spun = run({ T {}, T {}, tr::one() });
	auto still = run({ T {}, T {}, T {} });

	const vec3<T> & sp = spun.first;
	float sx = tr::to_float(sp.x), sy = tr::to_float(sp.y), sz = tr::to_float(sp.z);
	float ty = tr::to_float(still.first.y);
	float r_spun = std::sqrt(sx * sx + sy * sy);
	float wz_spun = tr::to_float(spun.second.z);
	float wz_still = tr::to_float(still.second.z);
	printf("spun=(%.2f,%.2f,%.2f) r=%.2f wz=%.3f  still_y=%.2f still_wz=%.3f\n",
	       sx, sy, sz, r_spun, wz_spun, ty, wz_still);

	assert(sy > 0.5f);                       // carried tangentially (+y) despite dynamic spin
	assert(sz > 0.3f);                       // stayed seated on the top face
	assert(r_spun > 2.0f && r_spun < 4.0f);  // carried around the axis, still on top
	assert(wz_spun > 0.05f);                 // Phase 9: friction spun the rider up about +z
	assert(std::fabs(ty) < 0.2f);            // no drift without platform spin
	assert(std::fabs(wz_still) < 0.05f);     // and no spin-up without it
	printf("  angular_carry_finite_inertia_rider[%s]: OK\n", label);
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
// Shock propagation must solve a contact at the same effective mass the main
// Gauss-Seidel sweep does. It used to pass the bare inverse-mass sum, which ignores
// the lever-arm term a rotating body contributes — for a flat slab contacted under a
// corner that is ~6x too small, so every shock pass over-relaxed the normal
// constraint by that factor and the residual error grew instead of shrinking. A
// landing then ran away by orders of magnitude (E30/E5 above 700x here) until the
// velocity cap caught it. Speculative only (shock propagation runs nowhere else) and
// rotating-body only (with inv_inertia == 0 the two masses are equal, so this is a
// no-op for every non-spinning body).
template <typename T> static void test_shock_angular_mass(const char * label) {
	using tr = scalar_traits<T>;
	printf("  shock_angular_mass[%s]: ", label);
	const T z {};
	const T grav = tr::from_int(20);
	// 12 x 3 x 12 cm, m = 0.2: a gib-sized slab, thin on Y.
	const T hx = tr::from_milli(60), hy = tr::from_milli(15), hz = tr::from_milli(60);
	const T mass = tr::from_milli(200);
	const T k = tr::from_int(1000);
	const vec3<T> inertia(tr::from_milli(255) / k, tr::from_milli(480) / k, tr::from_milli(255) / k);
	auto energy = [&](const std::shared_ptr<solid<T>> & b) {
		const vec3<T> v = b->get_velocity();
		mat3<T> Rt;
		transpose(Rt, b->get_orientation());
		vec3<T> wb;
		mul(wb, Rt, b->get_angular_velocity());   // body frame, where the inertia is diagonal
		const double m = tr::to_float(mass);
		return 0.5 * m * tr::to_float(dot(v, v)) +
		       0.5 * (tr::to_float(wb.x) * tr::to_float(wb.x) * tr::to_float(inertia.x) +
		              tr::to_float(wb.y) * tr::to_float(wb.y) * tr::to_float(inertia.y) +
		              tr::to_float(wb.z) * tr::to_float(wb.z) * tr::to_float(inertia.z)) +
		       m * tr::to_float(grav) * tr::to_float(b->get_position().y);
	};
	// Judge energy, never |w|: a small body rolling legitimately spins fast, so a |w|
	// threshold flags honest rolling and misses a body quietly doubling its energy.
	const vec3<T> spins[3] = { vec3<T>(tr::from_int(9), tr::from_int(3), tr::from_int(5)),
	                           vec3<T>(tr::from_int(2), tr::from_int(11), tr::from_int(4)),
	                           vec3<T>(tr::from_int(6), tr::from_int(6), tr::from_int(12)) };
	double worst = 0;
	for (int s = 0; s < 3; ++s) {
		simulator<T> sim;
		sim.set_gravity(vec3<T>(z, -grav, z));
		sim.set_default_contact_mode(contact_mode::speculative);
		auto floor = std::make_shared<solid<T>>();
		floor->set_infinite_mass();
		floor->set_coefficient_of_gravity(z);
		floor->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_int(60), -tr::one(), -tr::from_int(60)),
		              vec3<T>(tr::from_int(60), z, tr::from_int(60)))));
		sim.add_solid(floor);
		auto b = std::make_shared<solid<T>>();
		b->set_mass(mass);
		b->add_shape(std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-hx, -hy, -hz), vec3<T>(hx, hy, hz))));
		b->set_inertia(inertia);   // finite inertia IS rotation unlocked
		b->set_position(vec3<T>(z, tr::from_milli(900), z));
		b->set_velocity(vec3<T>(tr::from_milli(1200), tr::from_int(2), -tr::from_milli(800)));
		b->set_angular_velocity(spins[s]);
		sim.add_solid(b);
		// Energy at 30 s over energy at 5 s, so growth AFTER the landing transient.
		double landed = 0;
		for (int i = 0; i < 1800; ++i) {
			sim.update(tr::one() / tr::from_int(60));
			if (i == 300) landed = energy(b);
		}
		if (landed > 1e-6) worst = std::fmax(worst, energy(b) / landed);
	}
	printf("worst E30/E5 = %.2fx ", worst);
	assert(worst < 10.0);   // was 710x (double) / 2160x (float)
	printf("OK\n");
}

// A lever arm has to belong to the body it turns. When only one side of a pair
// discovers a contact, the other side's arm is fabricated from the discovering
// side's contact point — and support() cannot recover the tangential position of a
// FACE contact, so it collapses to the face CENTRE. For a floor that is the floor
// box's own centre, which is wherever the level author put it, and the arm it
// implies torques the partner about a point metres outside itself.
//
// The invariant: two floors that present the SAME plane under the body, differing
// only in where the box's centre sits, must produce the same landing. They used to
// produce different ones, which is how you can tell a fabricated arm is being used
// as if it were real.
template <typename T> static void test_contact_arm_not_face_centre(const char * label) {
	using tr = scalar_traits<T>;
	printf("  contact_arm_not_face_centre[%s]: ", label);
	const T z {};
	// `extra` grows the floor box in +x/+z only, moving its centre away from the
	// landing without changing the surface the body actually meets.
	auto land = [&](T extra, int spin_idx, vec3<T> & end_pos, T & end_speed) {
		simulator<T> sim;
		sim.set_gravity(vec3<T>(z, -tr::from_int(20), z));
		sim.set_default_contact_mode(contact_mode::speculative);
		auto floor = std::make_shared<solid<T>>();
		floor->set_infinite_mass();
		floor->set_coefficient_of_gravity(z);
		floor->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_int(60), -tr::one(), -tr::from_int(60)),
		              vec3<T>(tr::from_int(60) + extra, z, tr::from_int(60) + extra))));
		sim.add_solid(floor);
		auto b = std::make_shared<solid<T>>();
		const T hx = tr::from_milli(60), hy = tr::from_milli(15), hz = tr::from_milli(60);
		const T k = tr::from_int(1000);
		b->set_mass(tr::from_milli(200));
		b->add_shape(std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-hx, -hy, -hz), vec3<T>(hx, hy, hz))));
		b->set_inertia(vec3<T>(tr::from_milli(255) / k, tr::from_milli(480) / k, tr::from_milli(255) / k));
		b->set_position(vec3<T>(z, tr::from_milli(900), z));
		b->set_velocity(vec3<T>(tr::from_milli(1200), tr::from_int(2), -tr::from_milli(800)));
		const vec3<T> spins[3] = { vec3<T>(tr::from_int(9), tr::from_int(3), tr::from_int(5)),
		                           vec3<T>(tr::from_int(2), tr::from_int(11), tr::from_int(4)),
		                           vec3<T>(tr::from_int(6), tr::from_int(6), tr::from_int(12)) };
		b->set_angular_velocity(spins[spin_idx]);
		sim.add_solid(b);
		for (int i = 0; i < 1800; ++i)
			sim.update(tr::one() / tr::from_int(60));
		end_pos.set(b->get_position());
		end_speed = length(b->get_velocity());
	};
	float worst = 0;
	for (int s = 0; s < 3; ++s) {
		vec3<T> centred, offset;
		T v_centred, v_offset;
		land(z, s, centred, v_centred);                    // 120 m floor, centre under the landing
		land(tr::from_int(2000), s, offset, v_offset);     // same plane, centre 1 km away
		vec3<T> delta;
		sub(delta, offset, centred);
		worst = std::fmax(worst, tr::to_float(length(delta)));
		worst = std::fmax(worst, std::fabs(tr::to_float(v_offset) - tr::to_float(v_centred)));
	}
	printf("worst divergence = %.4f m ", worst);
	assert(worst < 0.01f);   // was 0.5-2 m: the landing followed the floor box's centre
	printf("OK\n");
}

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
	assert(wy > 0.12f); // and torqued it into a forward roll about +y
	// Threshold lowered from 0.25 when the contact lever arm was made sweep-free:
	// the box slides at 5 m/s, so the previous "impact − current_position" arm
	// carried a per-tick sweep offset that inflated the roll to wy≈0.55. The
	// corrected (sweep-free, current-position) arm gives wy≈0.18 here and the exact
	// textbook v0/1.4 rolling-without-slipping result for a sphere (see lever fix).
	printf("OK\n");
}

// Phase 10: a spring whose anchor sits off the body's center torques the body via
// its lever arm (τ = r × F). An off-center pull spins the body about +z; a centered
// pull (lever = 0) produces pure translation and no spin. Exercises rotated anchors
// + accumulate_constraint_torque, and the bit-identical center-anchor fast path.
// Friction's tangent effective mass is DIRECTION-dependent once a lever arm is in
// play, and the slip direction rotates during the solve. It used to be derived once
// from the pre-solve slip and reused for every iteration, which over-relaxes by the
// ratio between the two masses -- measured at 2.47x on a landing gib, and anything
// past 2 makes Gauss-Seidel diverge. The slip then grew ~1.47x per iteration and one
// tick handed a 5 cm chunk of debris several joules.
//
// The metric is the worst SINGLE-tick energy injection, not an end-to-end ratio: no
// physical contact adds joules in one frame, and a ratio averages a kick away against
// a chaotic tumble. A cube is the control -- its tangent mass is the same in every
// direction, so the frozen scalar was always right for it and it does not move.
template <typename T> static void test_friction_tangent_mass(const char * label) {
	using tr = scalar_traits<T>;
	printf("  friction_tangent_mass[%s]: ", label);
	const T z {};
	const T grav = tr::from_int(20);
	// 5 x 4 x 3 cm, m = 0.2: a rock gib, and the worst offender at 10.4 J a tick.
	const T hx = tr::from_milli(25), hy = tr::from_milli(20), hz = tr::from_milli(15);
	const T mass = tr::from_milli(200);
	// Godot's AABB inertia for that box at m = 0.2: 4.17e-5, 5.67e-5, 6.83e-5.
	const T k = tr::from_int(10000000);
	const vec3<T> inertia(tr::from_int(417) / k, tr::from_int(567) / k, tr::from_int(683) / k);
	auto energy = [&](const std::shared_ptr<solid<T>> & b) {
		const vec3<T> v = b->get_velocity();
		mat3<T> Rt;
		transpose(Rt, b->get_orientation());
		vec3<T> wb;
		mul(wb, Rt, b->get_angular_velocity());
		const double m = tr::to_float(mass);
		return 0.5 * m * tr::to_float(dot(v, v)) +
		       0.5 * (tr::to_float(wb.x) * tr::to_float(wb.x) * tr::to_float(inertia.x) +
		              tr::to_float(wb.y) * tr::to_float(wb.y) * tr::to_float(inertia.y) +
		              tr::to_float(wb.z) * tr::to_float(wb.z) * tr::to_float(inertia.z)) +
		       m * tr::to_float(grav) * tr::to_float(b->get_position().y);
	};
	const vec3<T> spins[3] = { vec3<T>(tr::from_int(9), tr::from_int(3), tr::from_int(5)),
	                           vec3<T>(tr::from_int(2), tr::from_int(11), tr::from_int(4)),
	                           vec3<T>(tr::from_int(6), tr::from_int(6), tr::from_int(12)) };
	double worst = 0;
	for (int s = 0; s < 3; ++s) {
		simulator<T> sim;
		sim.set_gravity(vec3<T>(z, -grav, z));
		sim.set_default_contact_mode(contact_mode::speculative);
		auto floor = std::make_shared<solid<T>>();
		floor->set_infinite_mass();
		floor->set_coefficient_of_gravity(z);
		floor->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_int(60), -tr::one(), -tr::from_int(60)),
		              vec3<T>(tr::from_int(60), z, tr::from_int(60)))));
		sim.add_solid(floor);
		auto b = std::make_shared<solid<T>>();
		b->set_mass(mass);
		b->add_shape(std::make_shared<shape<T>>(aa_box<T>(vec3<T>(-hx, -hy, -hz), vec3<T>(hx, hy, hz))));
		b->set_inertia(inertia);
		b->set_position(vec3<T>(z, tr::from_milli(900), z));
		b->set_velocity(vec3<T>(tr::from_milli(1200), tr::from_int(2), -tr::from_milli(800)));
		b->set_angular_velocity(spins[s]);
		sim.add_solid(b);
		double prev = energy(b);
		for (int i = 0; i < 3600; ++i) {   // 60 s: the worst kick is not always early
			sim.update(tr::one() / tr::from_int(60));
			const double e = energy(b);
			if (i > 60)   // past the landing transient
				worst = std::fmax(worst, e - prev);
			prev = e;
		}
	}
	printf("worst single-tick injection = %.4f J ", worst);
	assert(worst < 0.5);   // was 10.4 J
	printf("OK\n");
}

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

// Angular substepping (opt-in CCD): a thin obstacle the single per-frame snapshot
// steps angularly *over* (clear before and after, swept through between) is the one
// case the snapshot model misses. A blade spinning at 80 rad/s (~73°/step) sweeps its
// tip past a small peg most frames without a snapshot landing on it.
// set_angular_substeps_max subdivides the frame at the spinner's tip speed, so every
// pass is traced.
//
// Measured as PASSES TRACED, not as how far the peg ends up. Displacement conflates
// detection with response, and a snapshot that happens to end a frame overlapping the
// peg still delivers a full strike — so the two configurations can knock the peg
// comparably far while differing entirely in how many passes they actually saw, which
// is the thing substepping exists to change. The peg is pinned immovable for that
// count so the measurement cannot drift as it gets knocked out of the blade's path.
// A second, moving run keeps the end-to-end check that a traced pass still transfers
// the blade's surface speed.
template <typename T> static void test_angular_substep_ccd(const char * label) {
	using tr = scalar_traits<T>;
	printf("  angular_substep_ccd[%s]: ", label);
	const T z {};
	auto run = [&](int substeps) {
		simulator<T> sim;
		sim.set_gravity(vec3<T>(z, z, z));
		sim.set_angular_substeps_max(substeps);
		auto blade = std::make_shared<solid<T>>(); // thin ±2 blade, pinned, spun fast about z
		blade->set_mass(tr::one());
		blade->set_inertia(vec3<T>(tr::one(), tr::one(), tr::from_milli(200)));
		blade->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_int(2), -tr::from_milli(40), -tr::from_milli(40)),
		              vec3<T>(tr::from_int(2), tr::from_milli(40), tr::from_milli(40)))));
		blade->set_angular_velocity(vec3<T>(z, z, tr::from_int(80)));
		sim.add_solid(blade);
		auto pin = std::make_shared<constraint<T>>(blade, vec3<T>(z, z, z));
		pin->set_type(constraint<T>::type::spring);
		pin->set_rest_length(z);
		pin->set_spring_constant(tr::from_int(400));
		pin->set_damping_constant(tr::from_int(8));
		sim.add_constraint(pin);
		auto peg = std::make_shared<solid<T>>(); // small free sphere at the blade-tip radius
		peg->set_mass(tr::from_milli(200));
		peg->add_shape(std::make_shared<shape<T>>(sphere<T>(vec3<T>(z, z, z), tr::from_milli(120))));
		peg->set_position(vec3<T>(tr::from_milli(1700), z, z));
		sim.add_solid(peg);
		const T dt = tr::from_milli(16);
		T ang = z;
		const vec3<T> p0 = peg->get_position();
		float maxd = 0.0f;
		for (int i = 0; i < 200; ++i) {
			ang = ang + tr::from_int(80) * dt;
			mat3<T> R;
			set_mat3_from_axis_angle(R, vec3<T>(z, z, tr::one()), ang);
			blade->set_orientation(R);
			blade->set_angular_velocity(vec3<T>(z, z, tr::from_int(80)));
			sim.update(dt);
			vec3<T> d;
			sub(d, peg->get_position(), p0);
			float dist = std::sqrt(tr::to_float(d.x) * tr::to_float(d.x) + tr::to_float(d.y) * tr::to_float(d.y));
			if (dist > maxd) maxd = dist;
		}
		return maxd;
	};

	// Passes traced, with the peg held still so only detection varies. Identical to the
	// run above except the peg is immovable, so the count cannot drift as it is knocked
	// out of the blade's path.
	auto passes = [&](int substeps) {
		simulator<T> sim;
		sim.set_gravity(vec3<T>(z, z, z));
		sim.set_angular_substeps_max(substeps);
		auto blade = std::make_shared<solid<T>>();
		blade->set_mass(tr::one());
		blade->set_inertia(vec3<T>(tr::one(), tr::one(), tr::from_milli(200)));
		blade->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_int(2), -tr::from_milli(40), -tr::from_milli(40)),
		              vec3<T>(tr::from_int(2), tr::from_milli(40), tr::from_milli(40)))));
		blade->set_angular_velocity(vec3<T>(z, z, tr::from_int(80)));
		sim.add_solid(blade);
		auto pin = std::make_shared<constraint<T>>(blade, vec3<T>(z, z, z));
		pin->set_type(constraint<T>::type::spring);
		pin->set_rest_length(z);
		pin->set_spring_constant(tr::from_int(400));
		pin->set_damping_constant(tr::from_int(8));
		sim.add_constraint(pin);
		auto peg = std::make_shared<solid<T>>();
		peg->set_infinite_mass(); // immovable: the count must not depend on being knocked clear
		peg->add_shape(std::make_shared<shape<T>>(sphere<T>(vec3<T>(z, z, z), tr::from_milli(120))));
		peg->set_position(vec3<T>(tr::from_milli(1700), z, z));
		sim.add_solid(peg);
		int seen = 0;
		peg->set_collision_callback([&](const collision<T> &) { ++seen; });
		const T dt = tr::from_milli(16);
		T ang = z;
		for (int i = 0; i < 200; ++i) {
			ang = ang + tr::from_int(80) * dt;
			mat3<T> R;
			set_mat3_from_axis_angle(R, vec3<T>(z, z, tr::one()), ang);
			blade->set_orientation(R);
			blade->set_angular_velocity(vec3<T>(z, z, tr::from_int(80)));
			sim.update(dt);
		}
		return seen;
	};

	int seen_off = passes(1);
	int seen_on = passes(8);
	float on = run(8);
	printf("passes traced off=%d on=%d | peg displacement on=%.2f ", seen_off, seen_on, on);
	// The blade crosses the peg ~81 times in 200 frames (two ends, ~73°/frame). A single
	// snapshot only sees the crossings a frame boundary happens to land inside, which is
	// the narrow angular window the peg subtends; subdividing traces the sweep itself.
	assert(seen_on > seen_off * 2);
	assert(on > 50.0f); // and a traced pass still knocks the peg well away
	printf("OK\n");
}

// Phase 12: a rigid pin HOLDS. Two links hang off a world point under gravity; a force
// spring at any stiffness sags (it needs a stretch to produce force at all), while the
// rigid solve drives the anchor pair together at both the velocity and position level.
// The assertion is on the joint error, not the position: a chain is allowed to swing.
template <typename T> static void test_rigid_joint_chain(const char * label) {
	using tr = scalar_traits<T>;
	printf("  rigid_joint_chain[%s]: ", label);
	const T z {};
	const T half = tr::half();
	simulator<T> sim;
	sim.set_gravity(vec3<T>(z, -tr::from_int(20), z));  // hang along -y, not hop's default -z
	auto link = [&](T y) {
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		s->set_inertia(vec3<T>(tr::one(), tr::one(), tr::one()));
		s->set_collide_with_scope(0);  // a hanging chain, nothing to hit
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::from_milli(100), -half, -tr::from_milli(100)),
		              vec3<T>(tr::from_milli(100), half, tr::from_milli(100)))));
		s->set_position(vec3<T>(z, y, z));
		sim.add_solid(s);
		// After add_solid, which stamps the space default: a rigid joint solves in Pass B,
		// so its bodies must not have committed their position already in Pass A.
		s->set_contact_mode(contact_mode::speculative);
		return s;
	};
	// Anchors at (0,0,0) and (0,-1,0): link 1 hangs off the world, link 2 off link 1.
	auto s1 = link(-half);
	auto s2 = link(-half - tr::one());
	auto top = std::make_shared<constraint<T>>(s1, vec3<T>(z, z, z));
	top->set_type(constraint<T>::type::rigid);
	top->set_local_anchor_a(vec3<T>(z, half, z));
	sim.add_constraint(top);
	auto mid = std::make_shared<constraint<T>>(s1, s2);
	mid->set_type(constraint<T>::type::rigid);
	mid->set_local_anchor_a(vec3<T>(z, -half, z));
	mid->set_local_anchor_b(vec3<T>(z, half, z));
	sim.add_constraint(mid);

	auto anchor_of = [](const std::shared_ptr<solid<T>> & s, const vec3<T> & local) {
		vec3<T> lever, out;
		mul(lever, s->get_orientation(), local);
		add(out, s->get_position(), lever);
		return out;
	};
	float worst_top = 0.0f;
	float worst_mid = 0.0f;
	for (int i = 0; i < 300; ++i) {
		sim.update(tr::from_milli(16));
		vec3<T> a = anchor_of(s1, vec3<T>(z, half, z));
		float e_top = std::sqrt(tr::to_float(length_squared(a, vec3<T>(z, z, z))));
		vec3<T> b = anchor_of(s1, vec3<T>(z, -half, z));
		vec3<T> c = anchor_of(s2, vec3<T>(z, half, z));
		float e_mid = std::sqrt(tr::to_float(length_squared(b, c)));
		if (i > 30) {  // the first few ticks are the chain taking up its own weight
			if (e_top > worst_top) worst_top = e_top;
			if (e_mid > worst_mid) worst_mid = e_mid;
		}
	}
	float span = tr::to_float(s2->get_position().y);
	printf("top_err=%.4f mid_err=%.4f tail_y=%.3f ", worst_top, worst_mid, span);
	assert(worst_top < 0.02f);   // the chain hangs where it is pinned
	assert(worst_mid < 0.02f);
	assert(span > -2.2f);        // and did not stretch or fall away
	printf("OK\n");
}

// A satisfied rigid pin reads UNLOADED, so the body it holds can sleep. This is not a
// nicety: a soft spring holding a limb up against gravity is loaded by definition — it
// needs a nonzero stretch to produce any force — so a spring ragdoll never deactivates,
// and 21 bodies per corpse stay awake for the corpse's whole lifetime.
template <typename T> static void test_rigid_joint_sleeps(const char * label) {
	using tr = scalar_traits<T>;
	printf("  rigid_joint_sleeps[%s]: ", label);
	const T z {};
	auto hang = [&](typename constraint<T>::type kind) {
		simulator<T> sim;
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		s->set_collide_with_scope(0);
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::half(), -tr::half(), -tr::half()),
		              vec3<T>(tr::half(), tr::half(), tr::half()))));
		s->set_position(vec3<T>(z, z, z));
		sim.add_solid(s);
		s->set_contact_mode(contact_mode::speculative);  // after add_solid; see above
		auto c = std::make_shared<constraint<T>>(s, vec3<T>(z, z, z));
		c->set_type(kind);
		c->set_rest_length(z);
		c->set_spring_constant(tr::from_int(200));
		c->set_damping_constant(kind == constraint<T>::type::rigid ? tr::one() : tr::from_int(20));
		sim.add_constraint(c);
		for (int i = 0; i < 400; ++i)
			sim.update(tr::from_milli(16));
		return s->active();
	};
	bool spring_awake = hang(constraint<T>::type::spring);
	bool rigid_awake = hang(constraint<T>::type::rigid);
	printf("spring_awake=%d rigid_awake=%d ", spring_awake ? 1 : 0, rigid_awake ? 1 : 0);
	assert(spring_awake);   // a loaded spring can never go quiet — the contrast is the point
	assert(!rigid_awake);   // the pin holds it exactly, so it has nothing left to do
	printf("OK\n");
}

// ── Phase 13: angular limits ────────────────────────────────────────────────
//
// A horizontal arm pinned to a fixed post at its inboard end. Gravity folds it down, and
// the whole question is where it stops: a plain pin lets it hang straight down (90 degrees
// of swing, which on a corpse is a neck folded to the knees), and a cone limit must catch
// it at its span. `span` negative builds the unlimited pin for comparison.
//
// Reports the settled swing, the worst swing seen, and the worst PIN error — that last one
// is the assertion that matters most: a limit's impulse is pure torque, so hanging a limit
// off a pin must not open the pin.
template <typename T> struct limit_arm_result {
	float settled_swing = 0.0f;
	float worst_swing = 0.0f;
	float worst_pin = 0.0f;
	float final_x = 0.0f;
	float final_y = 0.0f;
};

template <typename T>
static limit_arm_result<T> run_limit_arm(T swing_span, T twist_span, int ticks) {
	using tr = scalar_traits<T>;
	const T z {};
	simulator<T> sim;
	sim.set_gravity(vec3<T>(z, -tr::from_int(20), z));
	// The post: immovable, and the frame every angle below is measured against.
	auto post = std::make_shared<solid<T>>();
	post->set_infinite_mass();
	post->set_coefficient_of_gravity(z);
	post->set_collide_with_scope(0);
	post->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_milli(50), -tr::from_milli(50), -tr::from_milli(50)),
	              vec3<T>(tr::from_milli(50), tr::from_milli(50), tr::from_milli(50)))));
	post->set_position(vec3<T>(z, z, z));
	sim.add_solid(post);
	post->set_contact_mode(contact_mode::speculative);
	// The arm: 1 m long down its own +X, which is the twist axis, so the cone points
	// along the arm exactly as it does down a GoldSrc bone.
	const T half_len = tr::half();
	const T half_thick = tr::from_milli(100);
	auto arm = std::make_shared<solid<T>>();
	arm->set_mass(tr::one());
	arm->set_inertia(vec3<T>(tr::from_milli(7), tr::from_milli(87), tr::from_milli(87)));
	arm->set_collide_with_scope(0);
	arm->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-half_len, -half_thick, -half_thick),
	              vec3<T>(half_len, half_thick, half_thick))));
	arm->set_position(vec3<T>(half_len, z, z));
	sim.add_solid(arm);
	arm->set_contact_mode(contact_mode::speculative);

	auto c = std::make_shared<constraint<T>>(post, arm);
	c->set_type(constraint<T>::type::rigid);
	c->set_local_anchor_a(vec3<T>(z, z, z));
	c->set_local_anchor_b(vec3<T>(-half_len, z, z));
	c->set_swing_span(swing_span);
	c->set_twist_span(twist_span);
	sim.add_constraint(c);

	const T eps = tr::from_milli(1);
	limit_arm_result<T> out;
	for (int i = 0; i < ticks; ++i) {
		sim.update(tr::from_milli(16));
		T swing {}, twist {};
		c->measure_limits(swing, twist, eps);
		vec3<T> lever, anchor;
		mul(lever, arm->get_orientation(), vec3<T>(-half_len, z, z));
		add(anchor, arm->get_position(), lever);
		float pin = std::sqrt(tr::to_float(length_squared(anchor, vec3<T>(z, z, z))));
		if (i > 30) {  // the first few ticks are the arm taking up its own weight
			float sw = tr::to_float(swing);
			if (sw > out.worst_swing) out.worst_swing = sw;
			if (pin > out.worst_pin) out.worst_pin = pin;
		}
		out.settled_swing = tr::to_float(swing);
	}
	out.final_x = tr::to_float(arm->get_position().x);
	out.final_y = tr::to_float(arm->get_position().y);
	return out;
}

// The cone catches the arm where it says it will, and the pin underneath it still holds.
template <typename T> static void test_cone_limit_holds(const char * label) {
	using tr = scalar_traits<T>;
	printf("  cone_limit_holds[%s]: ", label);
	const T span = tr::from_milli(524);  // 30 degrees
	auto limited = run_limit_arm<T>(span, -tr::one(), 300);
	auto free_pin = run_limit_arm<T>(-tr::one(), -tr::one(), 300);
	printf("limited=%.1f deg (worst %.1f, pin_err %.4f) free=%.1f deg (pin_err %.4f) ",
	       limited.settled_swing * 57.2958f, limited.worst_swing * 57.2958f, limited.worst_pin,
	       free_pin.settled_swing * 57.2958f, free_pin.worst_pin);
	// The unlimited arm hangs straight down. Without this the test would pass on a limit
	// that does nothing because nothing ever pushed on it.
	assert(free_pin.settled_swing > 1.4f);
	// The limited one stops at its span. The slack is the overshoot one tick of gravity
	// buys before the velocity sweep sees it, which the position pass then unwinds.
	assert(limited.worst_swing < tr::to_float(span) + 0.10f);
	assert(limited.settled_swing < tr::to_float(span) + 0.05f);
	assert(limited.settled_swing > tr::to_float(span) - 0.15f);  // it did reach its stop
	// The point of item 1: a limit is pure torque, so carrying load on one must not open
	// the pin it rides on. Same bound as the plain pin, not a looser one.
	assert(limited.worst_pin < 0.02f);
	assert(limited.worst_pin < free_pin.worst_pin + 0.005f);
	printf("OK\n");
}

// Spin the arm about its own length and the twist limit stops it there.
template <typename T> static void test_twist_limit_holds(const char * label) {
	using tr = scalar_traits<T>;
	printf("  twist_limit_holds[%s]: ", label);
	const T z {};
	const T span = tr::from_milli(524);  // 30 degrees
	simulator<T> sim;
	sim.set_gravity(vec3<T>(z, z, z));  // gravity is the swing's business, not the twist's
	auto post = std::make_shared<solid<T>>();
	post->set_infinite_mass();
	post->set_coefficient_of_gravity(z);
	post->set_collide_with_scope(0);
	post->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::from_milli(50), -tr::from_milli(50), -tr::from_milli(50)),
	              vec3<T>(tr::from_milli(50), tr::from_milli(50), tr::from_milli(50)))));
	sim.add_solid(post);
	post->set_contact_mode(contact_mode::speculative);
	auto arm = std::make_shared<solid<T>>();
	arm->set_mass(tr::one());
	arm->set_inertia(vec3<T>(tr::from_milli(7), tr::from_milli(87), tr::from_milli(87)));
	arm->set_collide_with_scope(0);
	arm->add_shape(std::make_shared<shape<T>>(
	    aa_box<T>(vec3<T>(-tr::half(), -tr::from_milli(100), -tr::from_milli(100)),
	              vec3<T>(tr::half(), tr::from_milli(100), tr::from_milli(100)))));
	arm->set_position(vec3<T>(tr::half(), z, z));
	sim.add_solid(arm);
	arm->set_contact_mode(contact_mode::speculative);
	// About +X, which is the twist axis: the pin sees none of this, so whatever stops it
	// is the twist limit and nothing else.
	arm->set_angular_velocity(vec3<T>(tr::from_int(5), z, z));

	auto c = std::make_shared<constraint<T>>(post, arm);
	c->set_type(constraint<T>::type::rigid);
	c->set_local_anchor_a(vec3<T>(z, z, z));
	c->set_local_anchor_b(vec3<T>(-tr::half(), z, z));
	c->set_swing_span(-tr::one());  // cone free; only the twist is under test
	c->set_twist_span(span);
	sim.add_constraint(c);

	const T eps = tr::from_milli(1);
	float worst = 0.0f;
	T swing {}, twist {};
	for (int i = 0; i < 200; ++i) {
		sim.update(tr::from_milli(16));
		c->measure_limits(swing, twist, eps);
		float t = std::fabs(tr::to_float(twist));
		if (t > worst)
			worst = t;
	}
	printf("twist=%.1f deg worst=%.1f deg swing=%.2f deg ",
	       tr::to_float(twist) * 57.2958f, worst * 57.2958f, tr::to_float(swing) * 57.2958f);
	assert(worst < tr::to_float(span) + 0.15f);          // it stopped at its span
	assert(std::fabs(tr::to_float(twist)) > 0.2f);       // and it did wind up to it
	assert(tr::to_float(swing) < 0.05f);                 // the twist row is not a cone row
	printf("OK\n");
}

// A limit that is not engaged must cost nothing and CHANGE nothing. Bit-identical is the
// bar, because anything less means a cone-twist is a different joint from a pin even in
// the middle of its range, and every tuned number in the game's table would then be
// covering for a solver that moved.
template <typename T> static void test_limit_is_unilateral(const char * label) {
	using tr = scalar_traits<T>;
	printf("  limit_is_unilateral[%s]: ", label);
	// A swing angle is at most pi by construction and a twist at most pi, so a span of
	// 229 degrees cannot be engaged even at the far end of its soft band. The arm below
	// swings all the way through vertical and up the other side, which is most of that
	// range, and still never touches these.
	const T wide = tr::from_milli(4000);  // 229 degrees
	auto plain = run_limit_arm<T>(-tr::one(), -tr::one(), 200);
	auto wide_cone = run_limit_arm<T>(wide, wide, 200);
	printf("plain=(%.6f,%.6f) wide=(%.6f,%.6f) swing=%.1f deg ",
	       plain.final_x, plain.final_y, wide_cone.final_x, wide_cone.final_y,
	       wide_cone.settled_swing * 57.2958f);
	assert(wide_cone.final_x == plain.final_x);
	assert(wide_cone.final_y == plain.final_y);
	assert(wide_cone.settled_swing == plain.settled_swing);
	printf("OK\n");
}

// Item 6, and the one most likely to be got wrong. A joint RESTING on its limit is a body
// resting on a floor: held, but not working, and it has to be allowed to sleep. The
// counter-test is in the same run — while the limit is still being violated the joint IS
// loaded and nothing may sleep, or the pair freezes in a pose the limit forbids.
template <typename T> static void test_joint_on_its_limit_sleeps(const char * label) {
	using tr = scalar_traits<T>;
	printf("  joint_on_its_limit_sleeps[%s]: ", label);
	const T z {};
	const T span = tr::from_milli(524);  // 30 degrees
	simulator<T> sim;
	sim.set_gravity(vec3<T>(z, z, z));
	auto make = [&](T x) {
		auto s = std::make_shared<solid<T>>();
		s->set_mass(tr::one());
		s->set_inertia(vec3<T>(tr::from_milli(7), tr::from_milli(87), tr::from_milli(87)));
		s->set_collide_with_scope(0);
		s->add_shape(std::make_shared<shape<T>>(
		    aa_box<T>(vec3<T>(-tr::half(), -tr::from_milli(100), -tr::from_milli(100)),
		              vec3<T>(tr::half(), tr::from_milli(100), tr::from_milli(100)))));
		s->set_position(vec3<T>(x, z, z));
		sim.add_solid(s);
		s->set_contact_mode(contact_mode::speculative);
		return s;
	};
	auto parent = make(-tr::half());
	auto child = make(tr::half());
	// Start the child folded 60 degrees into a 30-degree cone, at rest. Nothing but the
	// limit's own recovery can unwind this, and until it has, nothing may sleep.
	mat3<T> folded;
	set_mat3_from_axis_angle(folded, vec3<T>(z, z, tr::one()), tr::from_milli(1047));
	child->set_orientation(folded);

	auto c = std::make_shared<constraint<T>>(parent, child);
	c->set_type(constraint<T>::type::rigid);
	c->set_local_anchor_a(vec3<T>(tr::half(), z, z));
	c->set_local_anchor_b(vec3<T>(-tr::half(), z, z));
	c->set_swing_span(span);
	c->set_twist_span(-tr::one());
	// BIAS sets the recovery rate, so a small one is a deliberately slow unwind: "still
	// violating" and "resting on the limit" land dozens of ticks apart and the test can
	// look at both.
	c->set_limit_bias(tr::from_milli(20));
	sim.add_constraint(c);

	const T eps = tr::from_milli(1);
	T swing {}, twist {};
	bool awake_while_violating = false;
	for (int i = 0; i < 600; ++i) {
		sim.update(tr::from_milli(16));
		if (i == 5) {
			c->measure_limits(swing, twist, eps);
			awake_while_violating = child->active() && swing > span + tr::from_milli(17);
		}
	}
	c->measure_limits(swing, twist, eps);
	printf("swing=%.1f deg awake_at_5=%d asleep=%d ", tr::to_float(swing) * 57.2958f,
	       awake_while_violating ? 1 : 0, (!child->active() && !parent->active()) ? 1 : 0);
	assert(awake_while_violating);        // a violated limit is load, and load stays awake
	assert(tr::to_float(swing) < tr::to_float(span) + 0.03f);  // it unwound to its stop
	assert(!child->active());             // and then, resting on it, went quiet
	assert(!parent->active());
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
	test_bouncy_ball_settles<float>("float");
	test_resting_body_sleeps<float>("float", 0.001f);
	test_resting_body_sleeps<float>("float", 0.005f);
	test_speculative_manager_response<float>("float");
	test_mixed_modes_push<float>("float");
	test_angular_carry<float>("float");
	test_angular_carry_box<float>("float");
	test_angular_carry_box_capsule<float>("float");
	test_angular_carry_finite_inertia_rider<float>("float");
	test_oriented_box_rest<float>("float");
	test_dynamic_spin<float>("float");
	test_angular_impulse<float>("float");
	test_friction_rolling<float>("float");
	test_shock_angular_mass<float>("float");
	test_contact_arm_not_face_centre<float>("float");
	test_friction_tangent_mass<float>("float");
	test_constraint_anchor_torque<float>("float");
	test_rigid_joint_chain<float>("float");
	test_rigid_joint_sleeps<float>("float");
	test_cone_limit_holds<float>("float");
	test_twist_limit_holds<float>("float");
	test_limit_is_unilateral<float>("float");
	test_joint_on_its_limit_sleeps<float>("float");
	test_fast_spinner_no_tunnel<float>("float");
	test_angular_substep_ccd<float>("float");
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
	test_bouncy_ball_settles<fixed16>("fixed16");
	test_resting_body_sleeps<fixed16>("fixed16", 0.005f);
	test_speculative_manager_response<fixed16>("fixed16");
	test_mixed_modes_push<fixed16>("fixed16");
	test_angular_carry<fixed16>("fixed16");
	test_angular_carry_box<fixed16>("fixed16");
	test_angular_carry_box_capsule<fixed16>("fixed16");
	test_angular_carry_finite_inertia_rider<fixed16>("fixed16");
	test_oriented_box_rest<fixed16>("fixed16");
	test_dynamic_spin<fixed16>("fixed16");
	test_angular_impulse<fixed16>("fixed16");
	test_friction_rolling<fixed16>("fixed16");
	test_constraint_anchor_torque<fixed16>("fixed16");
	// The decomposition is more trig than hop does anywhere else, and asin/acos/atan2 are
	// polynomials in fixed point. The game's space is float, so this is a
	// correctness-of-the-port question, not a shipping one — instantiated, not tuned on.
	test_cone_limit_holds<fixed16>("fixed16");
	test_twist_limit_holds<fixed16>("fixed16");
	test_fast_spinner_no_tunnel<fixed16>("fixed16");
	test_angular_substep_ccd<fixed16>("fixed16");

	printf("ALL PASSED\n");
	return 0;
}
