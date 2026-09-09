// A body resting on a floor spins itself up out of nothing.
//
// Headless — no raylib, no renderer. Run it and read the table.
//
// THE BUG. A narrowphase returns ONE witness point. That is the whole contact for
// a sphere, whose support along any direction is a single point. It is not, for a
// capsule lying down (which touches along a LINE) or a box resting on its face
// (which touches over a QUAD). There the support function has a tie, it breaks it
// arbitrarily, and the winner changes frame to frame:
//
//     frame 3   lever = (-0.5120, -0.1000,  0.0000)
//     frame 4   lever = ( 0.5119, -0.1114, -0.0021)
//     frame 5   lever = (-0.5118, -0.1143, -0.0006)
//
// The solver then applies its normal impulse at a lever arm that swings the full
// width of the contact and flips sign with it. Push up the left end, then the
// right, then the left. Nothing cancels. Case A below drops a capsule on a floor
// with NO spin at all and it acquires angular velocity it never loses.
//
// WHY demo_bounce LOOKS FINE. It is a fair question and the answer is not "nobody
// measured" — case E replicates demo_bounce's own free box and it really is well
// behaved. Everything about it avoids the tie:
//
//   * It is a unit CUBE with isotropic inertia, so what lever arms it has are
//     short relative to the inertia they act on.
//   * It has restitution 1 and zero friction, so it never stops bouncing and
//     never forms the sustained resting contact the see-saw needs.
//   * It runs hop's DEFAULT contact mode, sweep_slide.
//
// A gib is the opposite on every count: a thin, elongated, very anisotropic slab
// that comes to rest lying flat — and hop-godot puts dynamic bodies on
// SPECULATIVE, not sweep_slide. Case D is that body. Compare its two columns:
// the amplification is mostly, though not entirely, in the speculative path.
//
// Each row checks itself: a body left alone on a floor must end still and must
// never end with more energy than it began with. Six of the twelve checks fail,
// and the program exits non-zero, so it doubles as the acceptance test for a fix.
//
//     demo_gib_spin              the table
//     demo_gib_spin --levers     the alternating contact point, frame by frame

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

struct result {
	double w_end;
	double energy_start;
	double energy_end;
	double peak_height;
};

static double energy(const std::shared_ptr<hop::solid<T>> & b, const V & inertia) {
	const V v = b->get_velocity();
	const V w = b->get_angular_velocity();
	// Inertia is diagonal in the BODY frame, so rotate w into it before using it.
	hop::mat3<T> Rt;
	hop::transpose(Rt, b->get_orientation());
	V wb;
	hop::mul(wb, Rt, w);
	const double lin = 0.5 * (double)b->get_mass() *
	                   ((double)v.x * (double)v.x + (double)v.y * (double)v.y + (double)v.z * (double)v.z);
	const double rot = 0.5 * ((double)wb.x * (double)wb.x * (double)inertia.x +
	                          (double)wb.y * (double)wb.y * (double)inertia.y +
	                          (double)wb.z * (double)wb.z * (double)inertia.z);
	const double pot = (double)b->get_mass() * GRAVITY * (double)b->get_position().y;
	return lin + rot + pot;
}

enum class body_shape { sphere, capsule, slab, cube };

// One body dropped on a floor. `spin` is its launch angular velocity; a gib gets
// one from the burst it came out of, and cases A/E deliberately get none.
static result drop(body_shape shape, V spin, hop::contact_mode mode, bool elastic) {
	hop::simulator<T> sim;
	sim.set_gravity(vec(0, -GRAVITY, 0));
	sim.set_default_contact_mode(mode);

	auto floor = std::make_shared<hop::solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	if (elastic) floor->set_coefficient_of_restitution(tr::one());
	floor->add_shape(std::make_shared<hop::shape<T>>(
	    hop::aa_box<T>(vec(-20, -1, -20), vec(20, 0, 20))));
	sim.add_solid(floor);

	// A gib-sized slab: 12 cm across, 3 cm thick. The capsule is the same body
	// with its corners rounded off; the sphere is the control, and the cube is
	// demo_bounce's.
	const double half = 0.06, thick = 0.015;
	auto body = std::make_shared<hop::solid<T>>();
	V inertia;
	double start_y = 0;
	switch (shape) {
	case body_shape::sphere:
		body->add_shape(std::make_shared<hop::shape<T>>(hop::sphere<T>(vec(0, 0, 0), (T)thick)));
		inertia = vec((T)(0.4 * 0.2 * thick * thick), (T)(0.4 * 0.2 * thick * thick), (T)(0.4 * 0.2 * thick * thick));
		start_y = thick + 0.05;
		break;
	case body_shape::capsule: {
		body->add_shape(std::make_shared<hop::shape<T>>(
		    hop::capsule<T>(vec(-(T)half, 0, 0), vec((T)(2 * half), 0, 0), (T)thick)));
		const double len = 2 * half + 2 * thick;
		inertia = vec((T)(0.5 * 0.2 * thick * thick),
		              (T)(0.2 * (3 * thick * thick + len * len) / 12),
		              (T)(0.2 * (3 * thick * thick + len * len) / 12));
		start_y = thick + 0.05;
		break;
	}
	case body_shape::slab: {
		body->add_shape(std::make_shared<hop::shape<T>>(
		    hop::aa_box<T>(vec(-(T)half, -(T)thick, -(T)half), vec((T)half, (T)thick, (T)half))));
		const double x = 2 * half, y = 2 * thick, z = 2 * half;
		inertia = vec((T)(0.2 * (y * y + z * z) / 12), (T)(0.2 * (x * x + z * z) / 12),
		              (T)(0.2 * (x * x + y * y) / 12));
		start_y = thick + 0.05;
		break;
	}
	case body_shape::cube:
		body->add_shape(std::make_shared<hop::shape<T>>(
		    hop::aa_box<T>(vec(-0.5, -0.5, -0.5), vec(0.5, 0.5, 0.5))));
		inertia = vec((T)0.167, (T)0.167, (T)0.167);
		start_y = 1.5;
		break;
	}
	body->set_mass(shape == body_shape::cube ? tr::one() : (T)0.2);
	body->set_inertia(inertia);
	if (elastic) {
		// demo_bounce's own settings: perfectly elastic and frictionless, so it
		// never settles and never forms a sustained resting contact.
		body->set_coefficient_of_restitution(tr::one());
		body->set_restitution_combine(hop::restitution_combine::minimum);
		body->set_coefficient_of_static_friction(T {});
		body->set_coefficient_of_dynamic_friction(T {});
	}
	body->set_position(vec(0, (T)start_y, 0));
	body->set_angular_velocity(spin);
	sim.add_solid(body);

	result r {};
	r.energy_start = energy(body, inertia);
	r.peak_height = -1e9;
	for (int i = 0; i < 1800; ++i) {   // 30 seconds
		sim.update((T)(1.0 / 60.0));
		r.peak_height = std::fmax(r.peak_height, (double)body->get_position().y);
	}
	r.w_end = (double)hop::length(body->get_angular_velocity());
	r.energy_end = energy(body, inertia);
	return r;
}

// A body left alone on a floor must end still, and must never end with more
// energy than it started with. Losing energy is fine — that is what friction and
// restitution below 1 are for.
static bool quiet(const result & r) {
	return r.w_end < 0.05 && r.energy_end <= r.energy_start * 1.05;
}

static int failures = 0;

static void row(const char * label, body_shape shape, V spin, bool elastic) {
	const result slide = drop(shape, spin, hop::contact_mode::sweep_slide, elastic);
	const result spec = drop(shape, spin, hop::contact_mode::speculative, elastic);
	const bool ok_slide = quiet(slide), ok_spec = quiet(spec);
	if (!ok_slide) ++failures;
	if (!ok_spec) ++failures;
	printf("  %-42s %8.2f %5.1fx %-5s %8.2f %5.1fx %-5s\n", label,
	       slide.w_end, slide.energy_end / slide.energy_start, ok_slide ? "ok" : "FAIL",
	       spec.w_end, spec.energy_end / spec.energy_start, ok_spec ? "ok" : "FAIL");
}

// The mechanism, frame by frame: the contact witness flipping end to end while
// the body sits there. This is the whole bug in one column.
static void dump_levers() {
	hop::simulator<T> sim;
	sim.set_gravity(vec(0, -GRAVITY, 0));
	sim.set_default_contact_mode(hop::contact_mode::speculative);
	auto floor = std::make_shared<hop::solid<T>>();
	floor->set_infinite_mass();
	floor->set_coefficient_of_gravity(T {});
	floor->add_shape(std::make_shared<hop::shape<T>>(hop::aa_box<T>(vec(-20, -1, -20), vec(20, 0, 20))));
	sim.add_solid(floor);
	auto rod = std::make_shared<hop::solid<T>>();
	rod->add_shape(std::make_shared<hop::shape<T>>(
	    hop::capsule<T>(vec(-0.512, 0, 0), vec(1.024, 0, 0), 0.1)));
	rod->set_mass((T)1);
	rod->set_inertia(vec(0.005, 0.09, 0.09));
	// Friction off, so what you see below is the NORMAL impulse alone.
	rod->set_coefficient_of_static_friction(T {});
	rod->set_coefficient_of_dynamic_friction(T {});
	rod->set_position(vec(0, 0.15, 0));
	rod->set_angular_velocity(vec(10, 0, 0));
	sim.add_solid(rod);

	printf("\n  A capsule lying on a floor, friction OFF, spun about its own spine.\n");
	printf("  The contact is a LINE and the witness is one point, so it alternates:\n\n");
	printf("  %-6s %-34s %8s\n", "frame", "contact lever arm", "|w|");
	for (int i = 0; i < 16; ++i) {
		sim.update((T)(1.0 / 60.0));
		printf("  %-6d ", i);
		if (rod->get_touch_count() > 0) {
			const auto & t = rod->get_touch(0);
			printf("(%7.4f, %7.4f, %7.4f)%9s", (double)t.lever.x, (double)t.lever.y,
			       (double)t.lever.z, "");
		} else {
			printf("%-34s", "  (airborne)");
		}
		printf("%8.2f\n", (double)hop::length(rod->get_angular_velocity()));
	}
	printf("\n  The x component swings the full half-length and changes sign nearly\n");
	printf("  every frame. The solver pushes up at one end, then the other, then the\n");
	printf("  first. Nothing cancels, so |w| ratchets.\n\n");
}

int main(int argc, char ** argv) {
	if (argc > 1 && std::string(argv[1]) == "--levers") {
		dump_levers();
		return 0;
	}
	printf("\nA body resting on a floor spins itself up. 30 seconds of simulation.\n");
	printf("Run with --levers to watch the contact point that causes it.\n\n");
	printf("  %-42s %-20s %-20s\n", "", "     sweep_slide", "    speculative");
	printf("  %-42s %8s %6s %-5s %8s %6s %-5s\n", "", "|w| end", "energy", "", "|w| end", "energy", "");
	printf("  %s\n", "---------------------------------------------------------------------------------------");

	// The control. A sphere's support along any direction is ONE point, so there
	// is no tie to break and nothing to see-saw on. It must stay quiet.
	row("sphere, dropped, no spin", body_shape::sphere, vec(0, 0, 0), false);

	// A. The headline. No spin, no push, nothing but gravity and a floor.
	row("capsule, dropped, NO SPIN AT ALL", body_shape::capsule, vec(0, 0, 0), false);

	// B. The same body spun about its own spine — a symmetry of the shape, which
	//    ought to be invisible to a contact.
	row("capsule, spun about its own spine", body_shape::capsule, vec(10, 0, 0), false);

	// C. A box resting on its face: a quad contact, the same tie one dimension up.
	row("slab, dropped, no spin", body_shape::slab, vec(0, 0, 0), false);

	// D. The gib. Thin, elongated, anisotropic, tumbling out of a burst, landing
	//    flat. hop-godot puts dynamic bodies on the SPECULATIVE column.
	row("slab, tumbling (a real gib)", body_shape::slab, vec(9, 3, 5), false);

	// E. demo_bounce's own free box, on demo_bounce's own settings. This is the
	//    control for "but the demo looks fine" — it does, and here is why.
	row("demo_bounce's cube (elastic, frictionless)", body_shape::cube, vec(0, 0, 0), true);

	printf("\n  Every row should read ok: a body left alone on a floor ends still, and\n");
	printf("  never ends with more energy than it began with. Losing energy is fine.\n");
	printf("\n  The first and last rows are the CONTROLS, and both pass. A sphere's\n");
	printf("  support is a single point, so it has no tie to break. demo_bounce's cube\n");
	printf("  is isotropic, perfectly elastic and frictionless, so it never settles into\n");
	printf("  the sustained resting contact the see-saw needs. That is why the demos\n");
	printf("  look fine while a gib does not.\n");
	printf("\n  %d of 12 checks FAILED\n\n", failures);
	return failures == 0 ? 0 : 1;
}
