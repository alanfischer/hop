# hop

A standalone, header-only C++17 physics library for swept-collision simulation. Extracted from the [toadlet](https://github.com/alanfischer/toadlet) game engine.

Supports `float`, `double`, `fixed16`, and `fixed32` scalar types via template parameterization, allowing deterministic fixed-point physics alongside standard floating-point in the same binary.

![demo_bounce — box, sphere, and capsules bouncing inside a room with collision sparks and a spring-constraint pendulum](docs/demo_bounce.gif)

## Authors

- Alan Fischer (me@alan.fish)
- Andrew Fischer (contact@apastron.co)

## Design Philosophy

Hop began as a **translation-only** physics engine — and that fast, predictable core is still its heart: continuous swept-collision detection, deterministic fixed-point support, and zero-allocation simulation. It was designed for the [toadlet](https://github.com/alanfischer/toadlet) game engine, where gameplay objects (characters, projectiles, pickups, triggers, vehicles on terrain) need solid collision response without the cost of a full rigid-body solver on every body.

**Rotation is now supported, and it is opt-in by construction.** A solid carries an orientation, angular velocity, and a principal-axis inertia, but every rotational term is gated behind a runtime *identity fast path*: a solid with identity orientation and zero inertia takes exactly the original translation-only code path, bit-for-bit (including fixed-point replay). There is **no template flag and no global switch** — you pay for rotation only on the bodies that use it:

- Leave a solid alone and it never rotates — orientation stays identity, `inv_inertia` stays zero, and the angular math multiplies out to nothing.
- Call `set_orientation(...)` for a statically-tilted brush (honored by the narrowphase and traceables) without making it spin.
- Call `set_inertia(...)` to opt a body into **dynamic rotation**: it then spins under torque, tumbles off off-center collisions, rolls via friction, and is torqued by off-center constraint anchors.
- Kinematic platforms carry their riders' velocity through `set_angular_velocity(...)` (a spinning `func_rotating`-style mover) with no inertia required.

See `docs/rotation_plan.md` for the full roadmap and the per-phase design notes.

## Features

- **Swept collision detection** (continuous collision detection) for sphere, capsule, box, and convex solid shapes
- **GJK closest-point narrowphase** for rounded-vs-polytope pairs — correct edge/vertex contact normals (a capsule rides up a thin ledge instead of catching on a fabricated wall) and vertex-bounded; switchable to a cheaper plane-inflation path via `set_accurate_narrowphase`
- **Multiple numerical integrators**: Euler, Improved Euler, Heun (default), Runge-Kutta
- **Collision response** with coefficient of restitution, conservation of momentum, and friction
- **Opt-in rigid-body rotation** — static orientation honored by the narrowphase and traceables, dynamic spin under torque (drift-free exponential quaternion integration), lever-arm angular impulse response (off-center hits tumble, friction rolls), kinematic angular carry for spinning platforms, and torque from off-center constraint anchors. Gated behind an identity fast path so non-rotating bodies stay bit-identical, fixed-point included
- **Stacking contact solver** — a post-integration Gauss–Seidel pass over the touched-pair graph (warm-started, with restitution targets and Coulomb-cone friction at the velocity level) lets resting piles transmit load and settle; iteration count is tunable via `set_solver_iterations`
- **Constraint system** with spring constants, damping, and distance thresholds
- **Deactivation/sleeping** for inactive solids
- **BVH spatial acceleration** — bounding volume hierarchy for broad-phase collision queries via `bvh_manager`
- **Collision scopes** — bitmask filtering for selective collision groups, plus `trigger_scope` for damage-zone / sensor-volume tagging
- **Per-solid collision filters** — custom `std::function` callback for fine-grained collision filtering
- **Fixed-point arithmetic** — `fixed16` & `fixed32` types with polynomial sin/cos/atan2, Newton-Raphson sqrt, and branchless min/max/abs
- **JavaScript bindings** — WebAssembly build via Emscripten/embind for browser-based physics
- **Zero external dependencies** — only the C++ standard library

## Overlap / Intersection Queries

Hop's fundamental primitive is the **swept segment** — every collision test moves a shape along a line and finds the earliest contact time `t ∈ [0, 1]`. There is no separate static overlap API; instead, use a **zero-direction segment** (`seg.origin == seg.end`) to ask "is this solid already overlapping something?"

```cpp
hop::segment<float> seg;
seg.set_start_end(pos, pos);   // zero direction = static overlap query

hop::collision<float> result;
sim->trace_solid(result, my_solid, seg, collision_mask);

bool overlapping = result.time < 1.0f;  // t == 0 means solid is inside
```

### Implementing `traceable::trace_solid` for zero direction

The sweep loop inside a traceable typically guards each face with:

```cpp
T denom = dot(face_n, seg.direction);
if (denom >= T{}) continue;   // moving away or parallel — skip
```

When `seg.direction` is zero, `denom` is always zero and **every face is skipped**, making the test silently report "clear" even when the solid is fully inside the mesh.

Traceable implementations **must** add an explicit zero-direction branch.  The correct algorithm is a Minkowski-sum static overlap check:

1. Compute `expanded_d = dot(face_n, v0) + dot(support(shape, -face_n), -face_n)` — the triangle plane expanded outward by the solid's extent.
2. If `dot(face_n, local_origin) <= expanded_d`, the solid's support surface has crossed the plane (penetrating from this face's side).
3. Project `local_origin` onto the original triangle plane and run a barycentric in-triangle test.
4. On a hit, set `result.time = T{}` and return.

See `HopTrimeshTraceable::trace_solid` in `hop-godot` for a reference implementation.

### Migrating a custom `traceable` for rotation

Adding rotation kept the public `solid`/`simulator` API source-compatible — an
existing program that never touches orientation behaves identically (the identity
fast path). The one interface that changed is `traceable::trace_solid`, so **custom
traceable implementors** (anyone with their own trimesh / heightfield / voxel target)
need two updates:

1. **Honor the solid's orientation.** `trace_solid` receives the mover's orientation;
   trace it the way GoldSrc traces a rotating brush — transform the mover into the
   target's local frame by `Rᵀ`, run the existing unrotated sweep, and map the
   resulting normal/point back out by `R`. Identity orientation reduces to the old
   path exactly, so a target that ignores the argument still works for non-rotated
   movers.
2. **Fill the real contact point (`col.impact`).** Implementations must set
   `col.impact` to the world-space witness point on the target surface (not the mover
   origin). This is the lever-arm input the angular response needs (`r = impact −
   solid.position`). It is inert until a consumer reads it and never feeds back into
   `time`/`normal`/`depth`, so fixed-point replay is unaffected — but a custom target
   that leaves it unset will produce no angular response at its contacts. The built-in
   trimesh / heightfield / plane traceables already fill it.

## Collision Pairs

Traceable-vs-Traceable is not supported. Segment traces (raycasting) work against all shape types.

**No cylinder primitive**: a cylinder's flat cap meets its curved side at a sharp circular edge. Sweeping any shape against that edge requires finding the intersection with a quarter-torus, which is a degree-4 polynomial — inconsistent with the quadratic math used everywhere else in hop, and impractical in fixed-point. Use a **capsule** instead: it is geometrically equivalent for collision purposes, and its hemispherical caps turn the hard rim problem into a smooth sphere test. If flat caps are required, a `convex_solid` approximation is available.

### Narrowphase: accurate (GJK) vs cheap (inflation)

For pairs where one shape is **rounded** (sphere/capsule) and the other a **polytope** (box/convex_solid), hop offers two narrowphase methods, selected per-simulator with `set_accurate_narrowphase(bool)` (default `true`):

- **Accurate** — GJK closest-point distance + conservative advancement, working from the polytope's **support function** (its vertex hull). The contact normal is the true direction between the closest features, so it is correct at **edges and vertices**: a capsule sweeping a ledge gets an up-and-out normal and rides over it, rather than the fabricated vertical-wall normal a face-based method reports. It is also inherently **bounded** — an ill-formed or unbounded plane set still has a finite vertex hull, so it can't phantom-block from afar.
- **Cheap** — the legacy path: inflate the convex's planes (or expand the box's AABB) by the rounded shape's support, then ray-trace. Exact on faces and slightly faster, but **face-normals-only** at edges (no ride-up). `set_accurate_narrowphase(false)` forces this globally.

`gjk_eligible_pair()` in `collide.h` names the eligible set. Two classes of pair are intentionally **not** routed through GJK:

- **rounded × rounded** (sphere/capsule pairs) — these have exact closed-form swept tests (`trace_sphere`, `trace_capsule`, `trace_capsule_capsule`); GJK would only approximate them.
- **polytope × polytope** (box×box, box/convex × convex) — their combined rounded radius is zero, so GJK would have to resolve contact at distance ≈ 0, where the divisions lose precision in fixed-point. The non-zero radius of a rounded shape is exactly what keeps GJK well-conditioned, so it is the defining property of eligibility. These keep the plane-inflation path, which is exact on faces anyway.

Two semantics worth knowing about the swept GJK path:

- A swept query is blocked **only by a contact it is moving into**. A body resting on a convex surface slides, walks, and steps off the edge freely (a tangential or separating contact does not stop the sweep); resting/ground contact is reported separately by the caller's down-probe, not by the motion cast.
- On **deep core penetration** the GJK path declines and the pair falls back to the inflation path, which recovers the overlap depth for de-penetration.

## Contact Solver, Settling & Sleep

Hop's tick runs in the **reverse of the textbook sequential-impulse order**. A standard rigid-body engine solves velocity constraints *first* and then integrates positions; hop integrates positions *first* (in `update_solid`, including gravity and the swept push-out that snaps a body to its earliest time-of-impact), and *then* runs the contact solver (`solve_contacts`, "Pass B") to fix up velocities. This ordering is what makes the swept/continuous-collision and fixed-point-deterministic design fall out cleanly — position is always advanced by an exact TOI snap rather than a predicted-then-corrected step.

The cost of that ordering is a small, structural **energy injection at contacts**. Every tick a resting body first sinks slightly into its neighbor under gravity, the push-out lifts it back out, and only afterward does the velocity solver run — so a little energy leaks back in, worst at angled contacts and low coefficients of restitution. For one or a few contacts this is invisible. For a deep pile it accumulates into a permanent low-grade churn: the stack transmits load and *looks* settled, but its kinetic energy plateaus instead of decaying to zero.

The visible symptom is in `examples/demo_stress.cpp` (~6859 spheres). It never fully goes to sleep. With the headless harness (`examples/headless_stress.cpp`), KE plateaus around ~55k and only a handful of bodies ever fall below the deactivation threshold — and switching the walls between elastic and dissipative barely moves it, confirming the residual is **endogenous** (sphere-to-sphere), not wall bounces.

**Knobs for the default pipeline** (symptomatic — they hide the churn, they don't remove its source):

- `set_deactivate_speed(s)` / `set_deactivate_count(n)` — how slow, and for how many consecutive ticks, before a body sleeps. Raising the speed / lowering the count sleeps far more of the pile (a sweep took the stress demo from ~55s to ~40s at `speed≈1.0, count=16` with no visible change, and to ~5s if pushed hard) — but past a point it *freezes bodies mid-motion*, leaving the pile puffed up with frozen-in voids. It buys speed by trading away settling accuracy.
- `set_solver_iterations(n)` — Pass B Gauss–Seidel sweeps (default 16). Lower is correct for shallow/single contacts; deep stacks need the full count. Note this is **not** a performance fast path: the bottleneck is the swept narrow-phase, not the solver, and fewer iterations leave the pile churning so *more* bodies stay awake — in practice lowering it makes the stress scene slower, not faster.

**What an actual fix would require — the pipeline reorder.** To let a large pile dissipate to true rest (and then sleep at the default threshold, for free), hop would need to move to the standard order: integrate to a *predicted* position, build contacts against that prediction, solve the velocity constraints, and only then commit the position — i.e. **speculative/swept-reconciled contacts** rather than the current integrate-then-snap-then-solve. This is the reconciliation that mainstream engines use to combine continuous collision with a sequential-impulse solver.

This is feasible for hop precisely because it already owns the expensive half. The swept query that answers "where would I go and what would I hit on the way" is exactly what speculative contacts need; most engines have to bolt continuous collision *on* to get it, whereas here it is the core primitive. Replacing the per-tick TOI *snap* with a velocity *clamp* (remove only enough approach speed to close the remaining gap) is still pure arithmetic, so it does **not** threaten the fixed-point determinism.

**Both strategies now ship as a per-body choice — `contact_mode`, not a global switch.** Each `solid` selects how its position is resolved (`solid::set_contact_mode`, defaulting to `simulator::set_default_contact_mode`):

- **`contact_mode::sweep_slide`** (the default) — the integrate → snap → slide path described above: exact TOI placement, the distinctive never-penetrate behavior, and crisp collide-and-slide movement. Ideal for a player/character and the single- or few-body game-object case hop targets.
- **`contact_mode::speculative`** — the reorder: discover → solve → integrate, with margin-shell contact discovery and an iterative NGS position solver. On `demo_stress` it removes the energy injection (~50× less residual KE), keeps a deep pile off the floor, and — with real Coulomb friction — settles a centered, drift-free pile. Ideal for dynamic debris/balls that need to pile and sleep.

The two are resolved in a single unified tick and **interoperate**: the velocity solve is shared (its target reduces to the legacy restitution response at a closed contact, so a `sweep_slide` body gets exactly the legacy behavior), and impulses are exchanged at each body's real mass. So a finite-mass `sweep_slide` character is genuinely *pushed* by `speculative` balls while still sliding crisply along walls — `tests/test_simulator.cpp::test_mixed_modes_push` guards this. Position de-penetration stays per-strategy (a `sweep_slide` body owns its position via the snap/slide; the NGS treats it as an immovable support, so a speculative partner takes the whole correction).

## Scope Bitmasks

Each `solid` carries four independent `int` bitmasks. They serve different purposes — keeping them straight is easier with one sentence per role:

| Mask | Default | Role |
|---|---|---|
| `scope` | `-1` | Per-tick activation. The solid is updated only when `(scope & update_arg) != 0`. |
| `collision_scope` | `-1` | Channels this solid **broadcasts on**. A trace against me must intersect this. |
| `collide_with_scope` | `-1` | Channels this solid **listens to**. Set to `0` to make a sensor that does broad-phase but never gets pushed. |
| `trigger_scope` | `0` | Tag bits OR'd into `collision::trigger_scope` when something is **statically overlapping** this solid (`t == 0`). |

`trigger_scope` lets you mark a solid as a damage zone, water volume, or quest area. After a swept trace, inspect `result.trigger_scope` to see which zones the moving solid ended up inside:

```cpp
auto zone = std::make_shared<solid<float>>();
zone->set_infinite_mass();
zone->set_trigger_scope(0x4);          // "this is damage zone bit 4"
zone->set_collide_with_scope(0);       // pass-through — physics ignores the volume
zone->add_shape(std::make_shared<shape<float>>(sphere<float>{{}, 2.0f}));
sim->add_solid(zone);

// Each tick, ask "which trigger zones is the player inside?"
collision<float> r;
segment<float> probe;
probe.set_start_dir(player->get_position(), {});  // zero direction = static query
sim->trace_solid(r, player.get(), probe, -1);
if (r.trigger_scope & 0x4) {
    // player is inside a damage zone
}
```

Trigger bits work for primitive shapes and traceable meshes alike, so a `convex_solid` or trimesh trigger volume reports the same way as a sphere.

## User Data

Each `solid` carries an opaque `void *` user data pointer for associating game-engine objects with physics bodies without any allocation:

```cpp
struct GameObject { /* ... */ };
GameObject obj;

auto s = std::make_shared<solid<float>>();
s->set_user_data(&obj);

// Later, in a collision callback or query loop:
auto * go = static_cast<GameObject *>(s->get_user_data());
```

The pointer is `nullptr` by default and is cleared on `solid::reset()`. It is not interpreted or dereferenced by the simulator.

## Usage

```cpp
#include <hop/hop.h>

using namespace hop;
using tr = scalar_traits<float>;

auto sim = std::make_shared<simulator<float>>();
sim->set_gravity({0.0f, 0.0f, -9.81f});

auto s = std::make_shared<solid<float>>();
s->set_mass(1.0f);
s->set_position({0.0f, 0.0f, 10.0f});

auto sh = std::make_shared<shape<float>>(sphere<float>{{}, 1.0f});
s->add_shape(sh);
sim->add_solid(s);

// Simulate 1 second in 10ms steps. dt is in seconds — match the gravity units.
for (int i = 0; i < 100; ++i) {
	sim->update(0.01f);
}
// s->get_position().z is now ~5.1 (freefall: 10 - 0.5 * 9.81 * 1^2)
```

For deterministic fixed-point simulation, substitute `fixed16` for `float`:

```cpp
auto sim = std::make_shared<simulator<fixed16>>();
sim->set_gravity({fixed16{}, fixed16{}, scalar_traits<fixed16>::from_milli(-9810)});
```

## Web Demo

A browser-based [demo](web/) using WebAssembly + Three.js is included — precompiled, no build step needed. Just serve the `web/` directory:

```sh
cd web && python3 -m http.server 8080
# Open http: // localhost:8080
```

To rebuild the WASM bindings from source (requires [Emscripten](https://emscripten.org)):

```sh
source emsdk/emsdk_env.sh
mkdir -p build-web && cd build-web
emcmake cmake .. -DHOP_BUILD_WEB=ON -DHOP_BUILD_TESTS=OFF -DHOP_BUILD_EXAMPLES=OFF
emmake make
```

## Building

```sh
mkdir build && cd build
cmake ..
make
ctest
```

Requires a C++17 compiler. Header-only — link against the `hop` CMake interface library to get the include path.

## Structure

```
include/hop/
  hop.h                  # umbrella header
  fixed16.h              # 16.16 fixed-point type
  fixed32.h              # 32.32 fixed-point type
  scalar_traits.h        # scalar_traits<float>, scalar_traits<double>, scalar_traits<fixed16>, scalar_traits<fixed32>
  simulator.h            # physics simulation loop
  solid.h                # dynamic/static physics body
  shape.h                # collision geometry (box, sphere, capsule, convex)
  constraint.h           # spring/damper constraints
  collision.h            # collision result data
  collide.h              # swept-collision routines (shape-vs-segment + solid-pair dispatch)
  manager.h              # spatial partitioning interface
  bvh.h                  # bounding volume hierarchy
  bvh_manager.h          # BVH-based manager implementation
  traceable.h            # custom shape interface
  fwd.h                  # forward declarations
  math/
    vec3.h               # 3D vector
    aa_box.h             # axis-aligned bounding box
    sphere.h             # sphere
    capsule.h            # capsule (swept sphere)
    segment.h            # line segment
    plane.h              # half-space plane
    convex_solid.h       # convex polyhedron
    mat3.h               # 3×3 matrix
    quat.h               # quaternion
    support.h            # support functions + convex-solid vertex enumeration
    gjk.h                # GJK closest-point distance + conservative-advancement sweep
    intersect.h          # intersection tests
    bounding.h           # bounding box computation
    project.h            # point/segment projection + segment-segment closest points
    triangle.h           # point/segment-vs-triangle closest points
```

## License

MIT License. See [LICENSE](LICENSE) for details.
