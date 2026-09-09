// demo_gib_floor.cpp — the BSP trace has no idea what shape it is holding.
//
// Toggle the floor between a plain aa_box and a GoldSrc BSP hull, throw gib-sized
// debris at it, and watch what changes. Boxes and capsules, because the two are
// supposed to rest differently and against the BSP hull they do not.
//
//   F / click        floor: BOX <-> BSP
//   1 / click        spawn 6 boxes
//   2 / click        spawn 6 capsules
//   C / click        clear
//   G                show/hide what the BSP trace actually sees
//   SPACE            pause
//
// WHAT TO LOOK FOR, with the BSP floor selected and G on:
//
// Every gib is drawn twice. In colour, the shape it really is; in grey wireframe, the
// AXIS-ALIGNED box the trace reads off it — `s->get_local_bound()`, which never turns
// with the body. That grey box is the entire shape the solver gets. Watch a long bone
// tumble onto its end: the grey box does not follow it, so the gib rests at the same
// height it did lying flat, and settles at whatever attitude it happened to stop in.
// There is no flat side to fall onto, because the solver cannot tell there is one.
//
// The red dot under each resting gib is the contact witness the trace reported. It sits
// straight below the centre, always, so the support impulse has no lever arm and cannot
// tip anything over. Switch to the BOX floor and the same debris beds down on a face.
//
// Space note: this demo is Y-UP, unlike hop's other raylib demos, because the BSP
// traceable converts between the host's space and GoldSrc's Z-up, and the host space it
// was written against is Y-up metres. Running it Y-up is what makes it reproduce the
// game rather than something adjacent to it.

#include <hop/hop.h>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "bsp/bsp_blob.h"
#include "bsp/hop_bsp_traceable.h"

using T = double;
using V = hop::vec3<T>;

// GoldSrc units to metres, as WizardWars scales its maps.
static const double SCALE = 0.025;
static const double GRAVITY = 20.0;
static const double DT = 1.0 / 60.0;
static const double GIB_LIFETIME = 6.0;
static const double GIB_MASS = 0.2;
// Matches the shipped game: hop-godot applies BODY angular damp only, and skips the
// write once a body is turning slower than the sleep threshold (otherwise the write
// re-activates it every step and it can never sleep).
static const double ANGULAR_DAMP = 3.0;

static const double FLOOR_HALF = 6.0;  // metres, both floors

static V vec(T x, T y, T z) { V v; v.set(x, y, z); return v; }

// Real gib proportions, metres, taken from the shipped GoldSrc gib models.
struct GibSize { const char *name; double x, y, z; };
static const GibSize GIB_SIZES[6] = {
	{ "bone",   0.120, 0.024, 0.070 },
	{ "shard",  0.100, 0.013, 0.060 },
	{ "rod",    0.180, 0.020, 0.020 },
	{ "plate",  0.090, 0.020, 0.070 },
	{ "chunk",  0.060, 0.050, 0.040 },
	{ "cinder", 0.050, 0.030, 0.050 },
};

struct Gib {
	std::shared_ptr<hop::solid<T>> solid;
	V half;            // half-extents of the box the collider spans
	double radius = 0; // capsule only
	bool capsule = false;
	double age = 0;
	Color color {};
};

static Vector3 rl(const V &v) { return (Vector3){ (float)v.x, (float)v.y, (float)v.z }; }

// Principal-axis inertia of a solid box.
static V box_inertia(const V &half, double m) {
	const double x = 2 * half.x, y = 2 * half.y, z = 2 * half.z;
	return vec((T)(m * (y * y + z * z) / 12), (T)(m * (x * x + z * z) / 12),
	           (T)(m * (x * x + y * y) / 12));
}

// Capsule about its own axis (x) and across it, treating it as a cylinder plus caps.
static V capsule_inertia(double half_len, double r, double m) {
	const double L = 2 * half_len + 2 * r;
	return vec((T)(0.5 * m * r * r), (T)(m * (3 * r * r + L * L) / 12),
	           (T)(m * (3 * r * r + L * L) / 12));
}

// Own RNG rather than raylib's, so --selftest runs with no window open.
static unsigned g_seed = 12345;
static float frand(float a, float b) {
	g_seed = g_seed * 1664525u + 1013904223u;
	return a + (b - a) * (float)((g_seed >> 8) & 0xFFFF) / 65535.0f;
}

// raylib names its Matrix fields by ROW (m0,m4,m8,m12 is the first row) while storing
// them column-major, which is what rlMultMatrixf wants. hop's mat3 is column-major too
// and is addressed at(row, col).
static Matrix to_matrix(const hop::mat3<T> &R, const V &p) {
	Matrix m = MatrixIdentity();
	m.m0 = (float)R.at(0, 0); m.m4 = (float)R.at(0, 1); m.m8  = (float)R.at(0, 2); m.m12 = (float)p.x;
	m.m1 = (float)R.at(1, 0); m.m5 = (float)R.at(1, 1); m.m9  = (float)R.at(1, 2); m.m13 = (float)p.y;
	m.m2 = (float)R.at(2, 0); m.m6 = (float)R.at(2, 1); m.m10 = (float)R.at(2, 2); m.m14 = (float)p.z;
	return m;
}

// ---------------------------------------------------------------------------

struct World {
	hop::simulator<T> sim;
	std::shared_ptr<hop::solid<T>> floor;
	std::vector<uint8_t> blob;                       // kept alive for the traceable
	std::unique_ptr<HopBspTraceable<T>> traceable;   // owned by the shape once moved
	std::vector<Gib> gibs;
	bool bsp = true;

	void build(bool use_bsp) {
		gibs.clear();
		sim = hop::simulator<T>();
		sim.set_gravity(vec(0, -GRAVITY, 0));
		// hop-godot gives every dynamic body speculative contacts, so the demo must too
		// or it is not reproducing the game's path.
		sim.set_default_contact_mode(hop::contact_mode::speculative);

		bsp = use_bsp;
		floor = std::make_shared<hop::solid<T>>();
		floor->set_infinite_mass();
		floor->set_coefficient_of_gravity(T {});
		floor->set_coefficient_of_static_friction(1);
		floor->set_coefficient_of_dynamic_friction(1);

		if (bsp) {
			// A slab whose top face is exactly y = 0, authored in GoldSrc units.
			const double gs = FLOOR_HALF / SCALE;
			const double mins[3] = { -gs, -gs, -64 }, maxs[3] = { gs, gs, 0 };
			blob = bsp_blob::make_box_map(mins, maxs);
			auto tr = std::make_unique<HopBspTraceable<T>>();
			tr->build(blob.data(), blob.size(), 0, (T)SCALE, hopbsp::BLOCK_SOLID);
			floor->add_shape(std::make_shared<hop::shape<T>>(std::move(tr)));
		} else {
			hop::aa_box<T> box;
			box.mins.set(-FLOOR_HALF, -1, -FLOOR_HALF);
			box.maxs.set(FLOOR_HALF, 0, FLOOR_HALF);
			floor->add_shape(std::make_shared<hop::shape<T>>(box));
		}
		sim.add_solid(floor);
	}

	void spawn(bool capsules) {
		for (int i = 0; i < 6; ++i) {
			const GibSize &g = GIB_SIZES[i];
			Gib gib;
			gib.capsule = capsules;
			gib.half = vec(g.x * 0.5, g.y * 0.5, g.z * 0.5);

			auto s = std::make_shared<hop::solid<T>>();
			if (capsules) {
				// Same length, rounded off: radius from the thinner cross-section, so a
				// bone stays a bone rather than becoming a sausage.
				const double r = std::fmax(0.5 * std::fmin(g.y, g.z), 0.006);
				const double hl = std::fmax(0.5 * g.x - r, 0.001);
				gib.radius = r;
				gib.half = vec(hl + r, r, r);
				s->add_shape(std::make_shared<hop::shape<T>>(
					hop::capsule<T>(vec(-hl, 0, 0), vec(2 * hl, 0, 0), (T)r)));
				s->set_inertia(capsule_inertia(hl, r, GIB_MASS));
			} else {
				hop::aa_box<T> b;
				b.mins.set(-gib.half.x, -gib.half.y, -gib.half.z);
				b.maxs.set(gib.half.x, gib.half.y, gib.half.z);
				s->add_shape(std::make_shared<hop::shape<T>>(b));
				s->set_inertia(box_inertia(gib.half, GIB_MASS));
			}
			s->set_mass((T)GIB_MASS);
			s->set_coefficient_of_static_friction(1);
			s->set_coefficient_of_dynamic_friction(1);
			s->set_coefficient_of_restitution(0);
			s->set_position(vec(frand(-0.7f, 0.7f), 1.4 + 0.12 * i, frand(-0.7f, 0.7f)));
			s->set_velocity(vec(frand(-1.2f, 1.2f), 0, frand(-1.2f, 1.2f)));

			// A random attitude, and a tumble about an axis ACROSS the long axis — a
			// thrown stick turns end over end.
			hop::quat<T> q;
			V axis = vec(frand(-1, 1), frand(-1, 1), frand(-1, 1));
			if (hop::length(axis) < 1e-3) axis = vec(0, 1, 0);
			hop::mul(axis, (T)(1.0 / hop::length(axis)));
			hop::set_quat_from_axis_angle(q, axis, (T)frand(0, 6.28f));
			s->set_orientation_from_quat(q);
			s->set_angular_velocity(vec(frand(-4, 4), frand(6, 14), frand(-4, 4)));

			sim.add_solid(s);
			gib.solid = s;
			gib.color = capsules ? (Color){ 90, 150, 220, 255 } : (Color){ 220, 140, 70, 255 };
			gibs.push_back(gib);
		}
	}

	void step() {
		// hop-godot's angular-damp loop, guard included: skip the write once the body is
		// slower than the sleep threshold, or set_angular_velocity's activate() resets
		// the deactivation counter every step and nothing ever sleeps.
		for (auto &g : gibs) {
			if (!g.solid->rotates_dynamically()) continue;
			if (hop::length(g.solid->get_angular_velocity()) < sim.get_deactivate_speed())
				continue;
			V w = g.solid->get_angular_velocity();
			hop::mul(w, (T)std::fmax(0.0, 1.0 - ANGULAR_DAMP * DT));
			g.solid->set_angular_velocity(w);
		}

		sim.update((T)DT);

		for (size_t i = 0; i < gibs.size();) {
			gibs[i].age += DT;
			// Gone below the floor's edge counts as gone, so a gib that rolls off does
			// not stay in the list falling forever.
			if (gibs[i].age >= GIB_LIFETIME || gibs[i].solid->get_position().y < -3.0) {
				sim.remove_solid(gibs[i].solid);
				gibs.erase(gibs.begin() + (long)i);
			} else {
				++i;
			}
		}
	}
};

// ---------------------------------------------------------------------------

struct Button {
	Rectangle rect;
	const char *label;
	bool hot = false;
};

static bool button(Button &b) {
	const Vector2 m = GetMousePosition();
	b.hot = CheckCollisionPointRec(m, b.rect);
	DrawRectangleRec(b.rect, b.hot ? (Color){ 70, 70, 78, 255 } : (Color){ 48, 48, 54, 255 });
	DrawRectangleLinesEx(b.rect, 1, (Color){ 110, 110, 120, 255 });
	const int tw = MeasureText(b.label, 16);
	DrawText(b.label, (int)(b.rect.x + (b.rect.width - tw) / 2), (int)(b.rect.y + 9), 16, RAYWHITE);
	return b.hot && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
}

// Headless proof that this demo reproduces the bug, for when you want the number
// rather than the picture. Drops one identical box from four attitudes on each floor
// and prints where it comes to rest.
static int selftest() {
	struct Case { const char *name; V axis; double angle; };
	const Case cases[] = {
		{ "flat", vec(1, 0, 0), 0.0 },
		{ "on end  (Z by 90)", vec(0, 0, 1), M_PI / 2 },
		{ "on edge (X by 90)", vec(1, 0, 0), M_PI / 2 },
		{ "corner  (X by 45)", vec(1, 0, 0), M_PI / 4 },
	};
	const V half = vec(0.060, 0.012, 0.035);  // the bone, half-extents

	printf("A box resting on each floor, from four attitudes.\n");
	printf("A real floor gives four different heights; half-Y is %.4f, half-X %.4f.\n\n",
	       (double)half.y, (double)half.x);
	printf("%-20s %-22s %-22s\n", "dropped", "BSP hull", "aa_box");
	printf("%-20s %-22s %-22s\n", "", "rest y   tilt  slept", "rest y   tilt  slept");

	double rest[4][2] = {};
	double tilt[4][2] = {};
	bool slept[4][2] = {};
	for (int floor_kind = 0; floor_kind < 2; ++floor_kind) {
		for (int c = 0; c < 4; ++c) {
			World w;
			w.build(/*use_bsp=*/floor_kind == 0);
			auto s = std::make_shared<hop::solid<T>>();
			hop::aa_box<T> b;
			b.mins.set(-half.x, -half.y, -half.z);
			b.maxs.set(half.x, half.y, half.z);
			s->add_shape(std::make_shared<hop::shape<T>>(b));
			s->set_mass((T)GIB_MASS);
			s->set_inertia(box_inertia(half, GIB_MASS));
			s->set_coefficient_of_static_friction(1);
			s->set_coefficient_of_dynamic_friction(1);
			s->set_coefficient_of_restitution(0);
			s->set_position(vec(0, 0.35, 0));
			hop::quat<T> q;
			hop::set_quat_from_axis_angle(q, cases[c].axis, (T)cases[c].angle);
			s->set_orientation_from_quat(q);
			w.sim.add_solid(s);
			for (int i = 0; i < 600; ++i) w.sim.update((T)DT);
			rest[c][floor_kind] = (double)s->get_position().y;
			slept[c][floor_kind] = !s->active();
			// How far the body's own Y axis ended up from world up: 0 deg means it is
			// lying flat, 90 means it is standing on a side.
			const hop::mat3<T> &R = s->get_orientation();
			const double uy = (double)R.at(1, 1);
			tilt[c][floor_kind] = std::acos(std::fmax(-1.0, std::fmin(1.0, std::fabs(uy)))) * 180.0 / M_PI;
		}
	}
	for (int c = 0; c < 4; ++c)
		printf("%-20s %-8.4f %-5.0f %-7s %-8.4f %-5.0f %-7s\n", cases[c].name,
		       rest[c][0], tilt[c][0], slept[c][0] ? "yes" : "no",
		       rest[c][1], tilt[c][1], slept[c][1] ? "yes" : "no");

	double spread_bsp = 0, spread_box = 0, worst_tilt_bsp = 0, worst_tilt_box = 0;
	for (int c = 1; c < 4; ++c) {
		spread_bsp = std::fmax(spread_bsp, std::fabs(rest[c][0] - rest[0][0]));
		spread_box = std::fmax(spread_box, std::fabs(rest[c][1] - rest[0][1]));
		worst_tilt_bsp = std::fmax(worst_tilt_bsp, tilt[c][0]);
		worst_tilt_box = std::fmax(worst_tilt_box, tilt[c][1]);
	}
	printf("\nheight spread across attitudes:  BSP %.4f   box %.4f\n", spread_bsp, spread_box);
	printf("worst resting tilt:              BSP %.0f deg   box %.0f deg\n",
	       worst_tilt_bsp, worst_tilt_box);
	printf("\n%s\n", spread_bsp < 1e-4
		? "BSP gives ONE height for every attitude, and the box comes to rest still standing\n"
		  "on its end or its corner: the trace never saw the shape turn, so there is no flat\n"
		  "side for it to fall onto. On the aa_box floor the same box tips onto its face."
		: "BSP spread is non-zero: the trace has become orientation-aware.");
	printf("\nThe aa_box column is orientation-AWARE, which is the point of the comparison.\n"
	       "Its absolute numbers are honest now that the two speculative box-vs-box defects\n"
	       "are fixed (#85, the shock pass solving at the wrong effective mass, and #86, the\n"
	       "lever arm fabricated from a face centre), but the signal here is still the\n"
	       "spread rather than the value: it is the BSP column this demo is about.\n");
	return 0;
}

int main(int argc, char **argv) {
	for (int i = 1; i < argc; ++i)
		if (strcmp(argv[i], "--selftest") == 0) return selftest();

	const int W = 1280, H = 720;
	InitWindow(W, H, "hop — gibs on a BSP floor vs a box floor");
	SetTargetFPS(60);

	World world;
	world.build(/*use_bsp=*/true);

	bool paused = false;
	bool show_trace_view = true;

	Camera3D cam = { 0 };
	cam.position = (Vector3){ 2.6f, 1.5f, 2.6f };
	cam.target = (Vector3){ 0.0f, 0.15f, 0.0f };
	cam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	cam.fovy = 45.0f;
	cam.projection = CAMERA_PERSPECTIVE;

	while (!WindowShouldClose()) {
		Button b_floor { { 16, 52, 190, 34 }, world.bsp ? "Floor: BSP HULL" : "Floor: AA BOX" };
		Button b_boxes { { 16, 94, 190, 34 }, "1  Spawn 6 boxes" };
		Button b_caps  { { 16, 132, 190, 34 }, "2  Spawn 6 capsules" };
		Button b_clear { { 16, 170, 190, 34 }, "C  Clear" };

		if (IsKeyPressed(KEY_SPACE)) paused = !paused;
		if (IsKeyPressed(KEY_G)) show_trace_view = !show_trace_view;

		// The third-person camera steals the mouse to look with, so leave it alone while
		// the pointer is over the button panel.
		const bool over_ui = GetMouseX() < 230 && GetMouseY() < 230;
		if (!over_ui) UpdateCamera(&cam, CAMERA_THIRD_PERSON);

		if (!paused) world.step();

		BeginDrawing();
		ClearBackground((Color){ 24, 26, 30, 255 });

		BeginMode3D(cam);
		// The floor, drawn the same either way — the difference is what it collides as.
		DrawCubeV((Vector3){ 0, -0.5f, 0 },
		          (Vector3){ (float)(2 * FLOOR_HALF), 1.0f, (float)(2 * FLOOR_HALF) },
		          world.bsp ? (Color){ 46, 58, 46, 255 } : (Color){ 44, 48, 62, 255 });
		DrawGrid(24, 0.5f);

		for (const Gib &g : world.gibs) {
			const V p = g.solid->get_position();
			const hop::mat3<T> &R = g.solid->get_orientation();
			const Color c = g.solid->active() ? g.color
			                                  : (Color){ 130, 130, 130, 255 };

			if (g.capsule) {
				// Endpoints of the spine, turned with the body.
				V ax = vec(R.at(0, 0), R.at(1, 0), R.at(2, 0));  // body +x in world
				const double hl = g.half.x - g.radius;
				V a = p, bb = p;
				V d = ax; hop::mul(d, (T)hl);
				hop::sub(a, d);
				hop::add(bb, d);
				DrawCapsule(rl(a), rl(bb), (float)g.radius, 12, 6, c);
				DrawCapsuleWires(rl(a), rl(bb), (float)g.radius, 12, 6, Fade(BLACK, 0.35f));
			} else {
				rlPushMatrix();
				const Matrix m = to_matrix(R, p);
				rlMultMatrixf(MatrixToFloat(m));
				DrawCubeV((Vector3){ 0, 0, 0 },
				          (Vector3){ (float)(2 * g.half.x), (float)(2 * g.half.y), (float)(2 * g.half.z) }, c);
				DrawCubeWiresV((Vector3){ 0, 0, 0 },
				               (Vector3){ (float)(2 * g.half.x), (float)(2 * g.half.y), (float)(2 * g.half.z) },
				               Fade(BLACK, 0.35f));
				rlPopMatrix();
			}

			if (show_trace_view && world.bsp) {
				// What the trace actually reads: the LOCAL bound, unrotated, at the
				// body's position. It never turns with the body — that is the bug.
				const hop::aa_box<T> &lb = g.solid->get_local_bound();
				const Vector3 size = { (float)(lb.maxs.x - lb.mins.x), (float)(lb.maxs.y - lb.mins.y),
				                       (float)(lb.maxs.z - lb.mins.z) };
				DrawCubeWiresV(rl(p), size, (Color){ 200, 200, 205, 190 });

				// And the contact witness it reported, straight below the centre.
				for (int t = 0; t < g.solid->get_touch_count(); ++t) {
					const auto &tc = g.solid->get_touch(t);
					DrawSphere(rl(tc.impact), 0.012f, (Color){ 230, 60, 60, 255 });
				}
			}
		}
		EndMode3D();

		// --- HUD ---
		DrawRectangle(0, 0, 420, 232, Fade(BLACK, 0.55f));
		DrawText("gibs on a BSP floor vs a box floor", 16, 16, 20, RAYWHITE);

		if (button(b_floor)) world.build(!world.bsp);
		if (button(b_boxes)) world.spawn(false);
		if (button(b_caps)) world.spawn(true);
		if (button(b_clear)) world.build(world.bsp);
		if (IsKeyPressed(KEY_F)) world.build(!world.bsp);
		if (IsKeyPressed(KEY_ONE)) world.spawn(false);
		if (IsKeyPressed(KEY_TWO)) world.spawn(true);
		if (IsKeyPressed(KEY_C)) world.build(world.bsp);

		char line[192];
		snprintf(line, sizeof line, "gibs: %d      %s", (int)world.gibs.size(),
		         paused ? "PAUSED (space)" : "");
		DrawText(line, 220, 60, 16, (Color){ 190, 190, 200, 255 });
		DrawText(show_trace_view ? "G: trace view ON" : "G: trace view OFF", 220, 82, 16,
		         (Color){ 190, 190, 200, 255 });
		DrawText("drag to look, WASD to move", 220, 104, 16, (Color){ 130, 130, 140, 255 });

		const int y0 = 244;
		if (world.bsp) {
			DrawRectangle(0, y0 - 8, 420, 108, Fade(BLACK, 0.45f));
			DrawText("Grey wire = the shape the trace sees.", 16, y0, 16, (Color){ 225, 225, 230, 255 });
			DrawText("It is the UNROTATED bound and never turns,", 16, y0 + 20, 16, (Color){ 190, 190, 200, 255 });
			DrawText("so on-end and flat are the same state.", 16, y0 + 40, 16, (Color){ 190, 190, 200, 255 });
			DrawText("Red dot = contact, always dead below centre:", 16, y0 + 60, 16, (Color){ 190, 190, 200, 255 });
			DrawText("no lever arm, so nothing can tip over.", 16, y0 + 80, 16, (Color){ 190, 190, 200, 255 });
		} else {
			DrawRectangle(0, y0 - 8, 420, 48, Fade(BLACK, 0.45f));
			DrawText("Box floor: real shape, real contacts.", 16, y0, 16, (Color){ 225, 225, 230, 255 });
			DrawText("Debris beds down on a face. Compare (F).", 16, y0 + 20, 16, (Color){ 190, 190, 200, 255 });
		}

		// Resting height is the tell: against the BSP hull every attitude reports the
		// same number, because the trace only ever reads half the unrotated Y extent.
		int shown = 0;
		for (const Gib &g : world.gibs) {
			if (g.solid->active() || shown >= 6) continue;
			const hop::aa_box<T> &lb = g.solid->get_local_bound();
			const double expect = 0.5 * (double)(lb.maxs.y - lb.mins.y);
			snprintf(line, sizeof line, "%-6s rest y %6.4f   half-Y %6.4f",
			         GIB_SIZES[shown].name, (double)g.solid->get_position().y, expect);
			DrawText(line, 16, H - 130 + shown * 18, 15, (Color){ 170, 200, 170, 255 });
			++shown;
		}
		if (shown > 0)
			DrawText("settled gibs:", 16, H - 152, 15, (Color){ 225, 225, 230, 255 });

		DrawFPS(W - 90, 12);
		EndDrawing();
	}

	CloseWindow();
	return 0;
}
