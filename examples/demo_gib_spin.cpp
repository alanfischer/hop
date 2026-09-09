// A spinning body invents energy. Two separate bugs, and the first one needs no
// contacts at all.
//
// Headless — no raylib, no renderer. Run it and read the two tables.
// Exits non-zero while anything gains energy, so it is the acceptance test.
//
// THE TEST is always energy, never angular velocity. Nothing physical makes
// energy, whereas a small body ROLLING has a perfectly legitimate large |w| — a
// 1 cm sphere rolling at 0.7 m/s turns at 70 rad/s. Judging |w| flags honest
// rolling and misses a body that quietly doubles its energy while barely turning.
//
// ── BUG 1: free spin in a vacuum ────────────────────────────────────────────
//
// Table 1 gives a body no gravity, no floor, and no collision scope whatsoever,
// spins it, and leaves it alone. Rotational energy is a conserved quantity for a
// free rigid body, so every row must read 1.000x.
//
// FIXED. It used to read 65x on a thin rod. Keep this table as the regression
// guard — the failure is invisible on any isotropic body, so nothing else catches
// it. What follows is what was wrong.
//
// The cause was in simulator::integrate_angular. Euler's equation was stepped
// with FORWARD EULER:
//
//     cross(gyro, wb, Iw);          // w x (I.w)
//     sub(net, tb, gyro);
//     mul(dwb, inv_inertia_, net);
//     mul(dwb, dt);
//     add(wb, dwb);                 // <- explicit step
//
// The free top conserves 0.5*w.I.w, which confines w to an ellipsoid in the body
// frame; the gyroscopic term is the rotation that carries it around that surface.
// A forward step goes along the TANGENT, which always lands outside — so the
// explicit step gains energy every single step, without bound. The comment in
// the source calls the gyroscopic term "cheap and stabilizing"; it is cheap, and
// it is the opposite of stabilizing.
//
// It shows up only on ANISOTROPIC bodies, which is why nothing caught it: for a
// sphere or a cube I is a scalar multiple of the identity, so w x (I.w) is
// identically zero and the whole term vanishes. Every hop demo body that spins is
// either isotropic or does not spin long enough to notice.
//
// The fix applied is the second of the two that were measured: the gyroscopic and
// torque halves are stepped apart, and w is rescaled back onto the energy ellipsoid
// after the gyroscopic half. That half does exactly zero work, so pinning the
// energy across it is the physical invariant rather than a fudge. Precession
// survives — an intermediate-axis spin still flips (tennis-racket effect). Energy
// comes back exact; |L| is not pinned and drifts up to 8% over 60 s, bounded and
// independent of spin rate. Bullet pins both with an implicit gyroscopic solve,
// which needs a 3x3 inverse hop cannot do across its fixed-point scalar types.
//
// The rejected alternative was dropping the gyroscopic term: also 1.000x, but it
// leaves w constant in the BODY frame, so asymmetric bodies never precess.
//
// ── BUG 2: a thin box landing on a floor ────────────────────────────────────
//
// Table 2 throws the same bodies at a floor, tumbling, and measures energy after
// they are down. With bug 1 fixed every SPHERE-collider row is clean, which is how
// we know the two are separate — and leaves the thin-box-under-speculative rows
// running away by five orders of magnitude. That one is NOT diagnosed. What is
// known: it is specific to the speculative contact mode (hop's default
// sweep_slide is nearly clean), it tracks flatness (a cube and a chunk are fine),
// it is absent when the body is placed at rest rather than landing, and it
// arrives as a single frame injecting several joules on first contact.

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <hop/hop.h>

using T = double;
using V = hop::vec3<T>;
using tr = hop::scalar_traits<T>;

static V vec(T x, T y, T z) {
	V v;
	v.set(x, y, z);
	return v;
}

static const double GRAVITY = 20.0;
static const double MASS = 0.2;

// What hop-godot computes when the game leaves inertia at zero: the collision
// AABB against the mass, exactly as Godot does.
static V aabb_inertia(double hx, double hy, double hz, double m) {
	const double x = 2 * hx, y = 2 * hy, z = 2 * hz;
	return vec((T)(m * (y * y + z * z) / 12), (T)(m * (x * x + z * z) / 12),
	           (T)(m * (x * x + y * y) / 12));
}

// Rotational energy in the BODY frame, where the inertia is diagonal.
static double spin_energy(const std::shared_ptr<hop::solid<T>> & b, const V & I) {
	hop::mat3<T> Rt;
	hop::transpose(Rt, b->get_orientation());
	V wb;
	hop::mul(wb, Rt, b->get_angular_velocity());
	return 0.5 * ((double)wb.x * (double)wb.x * (double)I.x +
	              (double)wb.y * (double)wb.y * (double)I.y +
	              (double)wb.z * (double)wb.z * (double)I.z);
}

static double total_energy(const std::shared_ptr<hop::solid<T>> & b, const V & I) {
	const V v = b->get_velocity();
	const double m = (double)b->get_mass();
	return 0.5 * m * ((double)v.x * (double)v.x + (double)v.y * (double)v.y +
	                  (double)v.z * (double)v.z) +
	       spin_energy(b, I) + m * GRAVITY * (double)b->get_position().y;
}

struct body_spec {
	const char * name;
	double hx, hy, hz;
};

static const body_spec BODIES[] = {
    { "cube   (cinder)  3 x 3 x 3 cm", 0.015, 0.015, 0.015 },
    { "chunk  (rock)    5 x 4 x 3 cm", 0.025, 0.020, 0.015 },
    { "plate  (metal)   9 x 2 x 7 cm", 0.045, 0.010, 0.035 },
    { "shard  (glass)  10 x 1 x 6 cm", 0.050, 0.005, 0.030 },
    { "slab   (today's collider)     ", 0.060, 0.015, 0.060 },
    { "rod    (bone)   18 x 2 x 2 cm", 0.090, 0.010, 0.010 },
};

static int failures = 0;

// ── Table 1: no gravity, no floor, collides with nothing ────────────────────

static double free_spin(const body_spec & s, V w0) {
	hop::simulator<T> sim;
	sim.set_gravity(vec(0, 0, 0));
	auto b = std::make_shared<hop::solid<T>>();
	b->add_shape(std::make_shared<hop::shape<T>>(
	    hop::aa_box<T>(vec(-(T)s.hx, -(T)s.hy, -(T)s.hz), vec((T)s.hx, (T)s.hy, (T)s.hz))));
	const V I = aabb_inertia(s.hx, s.hy, s.hz, MASS);
	b->set_mass((T)MASS);
	b->set_inertia(I);
	b->set_collide_with_scope(0);   // nothing to hit, in a world with nothing in it
	b->set_angular_velocity(w0);
	sim.add_solid(b);

	const double E0 = spin_energy(b, I);
	double worst = 1.0;
	for (int i = 0; i < 1800; ++i) {   // 30 s
		sim.update((T)(1.0 / 60.0));
		if (E0 > 1e-12) worst = std::fmax(worst, spin_energy(b, I) / E0);
	}
	return worst;
}

static void table_one() {
	printf("\n  TABLE 1 — free spin in a vacuum: no gravity, no floor, collides with\n");
	printf("  nothing. Rotational energy is CONSERVED here, so every row must be 1.00x.\n\n");
	printf("  %-34s %9s %12s %12s\n", "", "Ix : Iy", "|w0| = 10", "|w0| = 30");
	printf("  %s\n", "----------------------------------------------------------------------------");
	for (const auto & s : BODIES) {
		const V I = aabb_inertia(s.hx, s.hy, s.hz, MASS);
		const double slow = free_spin(s, vec(6, 6, 5));
		const double fast = free_spin(s, vec(18, 18, 15));
		if (slow > 1.02) ++failures;
		if (fast > 1.02) ++failures;
		printf("  %-34s  1 : %-5.2f %10.2fx %s %8.2fx %s\n", s.name,
		       (double)I.y / (double)I.x, slow, slow <= 1.02 ? "ok  " : "FAIL",
		       fast, fast <= 1.02 ? "ok  " : "FAIL");
	}
	printf("\n  Only a body whose inertia is isotropic survives, because there w x (I.w)\n");
	printf("  is identically zero and the unstable term never fires.\n");
}

// ── Table 2: thrown at a floor ──────────────────────────────────────────────

// Energy at 30 s over energy at 5 s: growth AFTER the body is down and the
// landing transient is over.
static double land(const body_spec & s, V spin, hop::contact_mode mode, double sphere_r) {
	hop::simulator<T> sim;
	sim.set_gravity(vec(0, -GRAVITY, 0));
	sim.set_default_contact_mode(mode);

	auto floor = std::make_shared<hop::solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	floor->add_shape(std::make_shared<hop::shape<T>>(
	    hop::aa_box<T>(vec(-60, -1, -60), vec(60, 0, 60))));
	sim.add_solid(floor);

	auto b = std::make_shared<hop::solid<T>>();
	if (sphere_r > 0)
		b->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(vec(0, 0, 0), (T)sphere_r)));
	else
		b->add_shape(std::make_shared<hop::shape<T>>(
		    hop::aa_box<T>(vec(-(T)s.hx, -(T)s.hy, -(T)s.hz), vec((T)s.hx, (T)s.hy, (T)s.hz))));
	// Finite inertia IS rotation unlocked. A Godot body with lock_rotation leaves
	// this zero, which gates hop's angular path off entirely — which is why gibs
	// are stable today, and why unlocking them is what exposes all of this.
	const V I = aabb_inertia(s.hx, s.hy, s.hz, MASS);
	b->set_mass((T)MASS);
	b->set_inertia(I);
	b->set_position(vec(0, 0.9, 0));
	b->set_angular_velocity(spin);
	b->set_velocity(vec(1.2, 2.0, -0.8));
	sim.add_solid(b);

	double landed = 0;
	for (int i = 0; i < 1800; ++i) {
		sim.update((T)(1.0 / 60.0));
		if (i == 300) landed = total_energy(b, I);
	}
	const double end = total_energy(b, I);
	if (landed <= 1e-9) return end > 1e-6 ? 999.0 : 1.0;
	return end / landed;
}

static void table_two() {
	const V spins[3] = { vec(9, 3, 5), vec(2, 11, 4), vec(6, 6, 12) };
	printf("\n\n  TABLE 2 — thrown tumbling at a floor. Energy at 30 s over energy at 5 s,\n");
	printf("  so growth AFTER landing. Worst of three launch spins.\n\n");
	printf("  %-34s %-22s %-22s\n", "", "    model AABB collider", "    sphere collider");
	printf("  %-34s %10s %10s %10s %10s\n", "", "slide", "specul.", "slide", "specul.");
	printf("  %s\n", "------------------------------------------------------------------------------------");
	for (const auto & s : BODIES) {
		printf("  %-34s", s.name);
		for (int kind = 0; kind < 2; ++kind) {
			// A sphere big enough to clear hop's speculative margin (8 mm default);
			// below that the collider is smaller than the tolerance around it.
			const double r = kind ? std::fmax(std::fmin(std::fmin(s.hx, s.hy), s.hz), 0.015) : 0.0;
			for (int m = 0; m < 2; ++m) {
				const auto mode = m ? hop::contact_mode::speculative : hop::contact_mode::sweep_slide;
				double worst = 0;
				for (int k = 0; k < 3; ++k) worst = std::fmax(worst, land(s, spins[k], mode, r));
				if (worst > 1.05) ++failures;
				printf(" %9.2fx", worst);
			}
		}
		printf("\n");
	}
	printf("\n  With bug 1 fixed the sphere columns go clean and these do not, which is\n");
	printf("  how we know they are two bugs and not one. Note sweep_slide (hop's own\n");
	printf("  default, and what every demo runs) is nearly clean — the runaway is in\n");
	printf("  the speculative path, which is what hop-godot gives dynamic bodies.\n");
}

int main(int argc, char ** argv) {
	const std::string arg = argc > 1 ? argv[1] : "";
	printf("\nA spinning body invents energy. 30 s at 60 Hz.\n");
	if (arg != "--land") table_one();
	if (arg != "--spin") table_two();
	printf("\n  %d checks FAILED\n\n", failures);
	return failures == 0 ? 0 : 1;
}
