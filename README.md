# hop

A standalone, header-only C++17 physics library for swept-collision simulation. Extracted from the [toadlet](https://github.com/alanfischer/toadlet) game engine.

Supports `float`, `double`, `fixed16`, and `fixed32` scalar types via template parameterization, allowing deterministic fixed-point physics alongside standard floating-point in the same binary.

![demo_bounce — box, sphere, and capsules bouncing inside a room with collision sparks and a spring-constraint pendulum](docs/demo_bounce.gif)

## Authors

- Alan Fischer (me@alan.fish)
- Andrew Fischer (contact@apastron.co)

## Design Philosophy

Hop is currently a **translation-only** physics engine — solids have position and linear velocity but no rotation. There is no angular velocity, no torque, no inertia tensor, and no orientation state.

This is intentional. Hop was designed for the [toadlet](https://github.com/alanfischer/toadlet) game engine where gameplay objects needed fast, predictable collision response without the complexity of a full rigid-body solver. The tradeoff is straightforward: you get continuous swept-collision detection, deterministic fixed-point support, and zero-allocation simulation in exchange for not handling rotational dynamics. For many game scenarios — characters, projectiles, pickups, triggers, vehicles on terrain — translation-only physics is sufficient and far simpler to reason about.

Rotational support is a work in progress — see `docs/rotation_plan.md` for the roadmap. The `mat3` and `quat` math primitives have landed; the simulator itself has not yet adopted them.

## Features

- **Swept collision detection** (continuous collision detection) for sphere, capsule, box, and convex solid shapes
- **Multiple numerical integrators**: Euler, Improved Euler, Heun (default), Runge-Kutta
- **Collision response** with coefficient of restitution, conservation of momentum, and friction
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

## Collision Pairs

Traceable-vs-Traceable is not supported. Segment traces (raycasting) work against all shape types.

**No cylinder primitive**: a cylinder's flat cap meets its curved side at a sharp circular edge. Sweeping any shape against that edge requires finding the intersection with a quarter-torus, which is a degree-4 polynomial — inconsistent with the quadratic math used everywhere else in hop, and impractical in fixed-point. Use a **capsule** instead: it is geometrically equivalent for collision purposes, and its hemispherical caps turn the hard rim problem into a smooth sphere test. If flat caps are required, a `convex_solid` approximation is available.

## Contact Solver, Settling & Sleep

Hop's tick runs in the **reverse of the textbook sequential-impulse order**. A standard rigid-body engine solves velocity constraints *first* and then integrates positions; hop integrates positions *first* (in `update_solid`, including gravity and the swept push-out that snaps a body to its earliest time-of-impact), and *then* runs the contact solver (`solve_contacts`, "Pass B") to fix up velocities. This ordering is what makes the swept/continuous-collision and fixed-point-deterministic design fall out cleanly — position is always advanced by an exact TOI snap rather than a predicted-then-corrected step.

The cost of that ordering is a small, structural **energy injection at contacts**. Every tick a resting body first sinks slightly into its neighbor under gravity, the push-out lifts it back out, and only afterward does the velocity solver run — so a little energy leaks back in, worst at angled contacts and low coefficients of restitution. For one or a few contacts this is invisible. For a deep pile it accumulates into a permanent low-grade churn: the stack transmits load and *looks* settled, but its kinetic energy plateaus instead of decaying to zero.

The visible symptom is in `examples/demo_stress.cpp` (~6859 spheres). It never fully goes to sleep. With the headless harness (`examples/headless_stress.cpp`), KE plateaus around ~55k and only a handful of bodies ever fall below the deactivation threshold — and switching the walls between elastic and dissipative barely moves it, confirming the residual is **endogenous** (sphere-to-sphere), not wall bounces.

**Knobs for the default pipeline** (symptomatic — they hide the churn, they don't remove its source):

- `set_deactivate_speed(s)` / `set_deactivate_count(n)` — how slow, and for how many consecutive ticks, before a body sleeps. Raising the speed / lowering the count sleeps far more of the pile (a sweep took the stress demo from ~55s to ~40s at `speed≈1.0, count=16` with no visible change, and to ~5s if pushed hard) — but past a point it *freezes bodies mid-motion*, leaving the pile puffed up with frozen-in voids. It buys speed by trading away settling accuracy.
- `set_solver_iterations(n)` — Pass B Gauss–Seidel sweeps (default 16). Lower is correct for shallow/single contacts; deep stacks need the full count. Note this is **not** a performance fast path: the bottleneck is the swept narrow-phase, not the solver, and fewer iterations leave the pile churning so *more* bodies stay awake — in practice lowering it makes the stress scene slower, not faster.

**What an actual fix would require — the pipeline reorder.** To let a large pile dissipate to true rest (and then sleep at the default threshold, for free), hop would need to move to the standard order: integrate to a *predicted* position, build contacts against that prediction, solve the velocity constraints, and only then commit the position — i.e. **speculative/swept-reconciled contacts** rather than the current integrate-then-snap-then-solve. This is the reconciliation that mainstream engines use to combine continuous collision with a sequential-impulse solver.

This is feasible for hop precisely because it already owns the expensive half. The swept query that answers "where would I go and what would I hit on the way" is exactly what speculative contacts need; most engines have to bolt continuous collision *on* to get it, whereas here it is the core primitive. Replacing the per-tick TOI *snap* with a velocity *clamp* (remove only enough approach speed to close the remaining gap) is still pure arithmetic, so it does **not** threaten the fixed-point determinism.

**This reorder is now implemented as an opt-in pipeline:** `set_speculative_contacts(true)` switches the tick to discover → solve → integrate, with margin-shell contact discovery and an iterative NGS position solver. On `demo_stress` it removes the energy injection (~50× less residual KE), keeps a deep pile from penetrating the floor, and — with real Coulomb friction rather than the old viscous-damping crutch — settles a centered, drift-free pile. It is **experimental and off by default**: the default pipeline (exact TOI snap) is unchanged, which keeps hop's distinctive never-penetrate behavior for the single-body game-object case it targets. Remaining work before it could become the default (full settle-to-sleep, exposed tuning knobs, a settling/tunneling test suite) is tracked in `docs/pipeline_reorder_plan.md`.

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
    intersect.h          # intersection tests
    bounding.h           # bounding box computation
    project.h            # point/segment projection
```

## License

MIT License. See [LICENSE](LICENSE) for details.
