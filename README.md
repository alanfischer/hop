# hop

A standalone, header-only C++17 physics library for swept-collision simulation. Extracted from the [Toadlet](https://github.com/alanfischer/toadlet) game engine.

Supports both `float` and 16.16 `fixed16` scalar types via template parameterization, allowing deterministic fixed-point physics alongside standard floating-point in the same binary.

![demo_bounce — box, sphere, and capsules bouncing inside a room with collision sparks and a spring-constraint pendulum](docs/demo_bounce.gif)

## Authors

- Alan Fischer (me@alan.fish)
- Andrew Fischer (contact@apastron.co)

## Design Philosophy

Hop is a **translation-only** physics engine — solids have position and linear velocity but no rotation. There is no angular velocity, no torque, no inertia tensor, and no orientation state. All collision shapes (boxes, spheres, capsules) remain axis-aligned at all times.

This is intentional. Hop was designed for the [Toadlet](https://github.com/alanfischer/toadlet) game engine where gameplay objects needed fast, predictable collision response without the complexity of a full rigid-body solver. The tradeoff is straightforward: you get continuous swept-collision detection, deterministic fixed-point support, and zero-allocation simulation in exchange for not handling rotational dynamics. For many game scenarios — characters, projectiles, pickups, triggers, vehicles on terrain — translation-only physics is sufficient and far simpler to reason about.

## Features

- **Swept collision detection** (continuous collision detection) for sphere, capsule, box, and convex solid shapes
- **Multiple numerical integrators**: Euler, Improved Euler, Heun (default), Runge-Kutta
- **Collision response** with coefficient of restitution, conservation of momentum, and friction
- **Constraint system** with spring constants, damping, and distance thresholds
- **Deactivation/sleeping** for inactive solids
- **BVH spatial acceleration** — bounding volume hierarchy for broad-phase collision queries via `bvh_manager`
- **Collision scopes** — bitmask filtering for selective collision groups
- **Per-solid collision filters** — custom `std::function` callback for fine-grained collision filtering
- **Fixed-point arithmetic** — `fixed16` type with polynomial sin/cos/atan2, Newton-Raphson sqrt, and branchless min/max/abs
- **JavaScript bindings** — WebAssembly build via Emscripten/embind for browser-based physics
- **Zero external dependencies** — only the C++ standard library
- **Zero-allocation hot paths** — all temporaries are pre-allocated as cache members on the simulator

## Overlap / Intersection Queries

Hop's fundamental primitive is the **swept segment** — every collision test moves a shape along a line and finds the earliest contact time `t ∈ [0, 1]`. There is no separate static overlap API; instead, use a **zero-direction segment** (`seg.origin == seg.end`) to ask "is this solid already overlapping something?"

```cpp
hop::segment<float> seg;
seg.set_start_end(pos, pos);   // zero direction = static overlap query

hop::collision<float> result;
sim->trace_solid(result, my_solid, seg, collision_mask);

bool overlapping = result.time < 1.0f;  // t == 0 means solid is inside
```

### Zero-direction behaviour by shape type

| Shape pair | Zero-direction result |
|---|---|
| Solid vs box | `t = 0` when inside (via `test_inside` path) |
| Solid vs sphere | `t = 0` when inside |
| Solid vs capsule | `t = 0` when inside |
| Solid vs traceable | Depends on the `traceable` implementation — **see below** |

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

Not all shape combinations are supported for solid-vs-solid collision. The matrix below shows which pairs have swept collision detection:

|               |  Box   | Sphere | Capsule | Convex | Traceable |
|---------------|:------:|:------:|:-------:|:------:|:---------:|
| **Box**       |   ✓    |   ✓    |    ✓    |   ✓    |     ✓     |
| **Sphere**    |   ✓    |   ✓    |    ✓    |   ✓    |     ✓     |
| **Capsule**   |   ✓    |   ✓    |    ✓    |   ✓    |     ✓     |
| **Convex**    |   ✓    |   ✓    |    ✓    |   ✓    |     ✓     |
| **Traceable** |   ✓    |   ✓    |    ✓    |   ✓    |           |

Traceable-vs-Traceable is not supported. Segment traces (raycasting) work against all shape types.

**No cylinder primitive**: a cylinder's flat cap meets its curved side at a sharp circular edge. Sweeping any shape against that edge requires finding the intersection with a quarter-torus, which is a degree-4 polynomial — inconsistent with the quadratic math used everywhere else in hop, and impractical in fixed-point. Use a **capsule** instead: it is geometrically equivalent for collision purposes, and its hemispherical caps turn the hard rim problem into a smooth sphere test. If flat caps are required, a `convex_solid` approximation is available.

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

// Simulate 1 second in 10ms steps
for (int i = 0; i < 100; ++i) {
	sim->update(10);
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
  scalar_traits.h        # scalar_traits<float> and scalar_traits<fixed16>
  simulator.h            # physics simulation loop
  solid.h                # dynamic/static physics body
  shape.h                # collision geometry (box, sphere, capsule, convex)
  constraint.h           # spring/damper constraints
  collision.h            # collision result data
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
    math_ops.h           # vector operations (dot, cross, normalize, ...)
    intersect.h          # intersection tests
    bounding.h           # bounding box computation
    project.h            # point/segment projection
```

## License

MIT License. See [LICENSE](LICENSE) for details.
