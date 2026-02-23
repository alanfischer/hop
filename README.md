# hop

A standalone, header-only C++17 physics library for swept-collision simulation. Extracted from the [Toadlet](https://github.com/alanfischer/toadlet) game engine.

Supports both `float` and 16.16 `fixed16` scalar types via template parameterization, allowing deterministic fixed-point physics alongside standard floating-point in the same binary.

![demo_bounce — box, sphere, and capsule bouncing inside a room](docs/demo_bounce.gif)

## Authors

- Alan Fischer (me@alan.fish)
- Andrew Fischer (contact@apastron.co)

## Design Philosophy

Hop is a **translation-only** physics engine — solids have position and linear velocity but no rotation. There is no angular velocity, no torque, no inertia tensor, and no orientation state. All collision shapes (boxes, spheres, capsules) remain axis-aligned at all times.

This is intentional. Hop was designed for the [Toadlet](https://github.com/alanfischer/toadlet) game engine where gameplay objects needed fast, predictable collision response without the complexity of a full rigid-body solver. The tradeoff is straightforward: you get continuous swept-collision detection, deterministic fixed-point support, and zero-allocation simulation in exchange for not handling rotational dynamics. For many game scenarios — characters, projectiles, pickups, triggers, vehicles on terrain — translation-only physics is sufficient and far simpler to reason about.

## Features

- **Swept collision detection** (continuous collision detection) for sphere, capsule, AA-box, and convex solid shapes
- **Multiple numerical integrators**: Euler, Improved Euler, Heun (default), Runge-Kutta
- **Collision response** with coefficient of restitution, conservation of momentum, and friction
- **Constraint system** with spring constants, damping, and distance thresholds
- **Deactivation/sleeping** for inactive solids
- **Collision scopes** — bitmask filtering for selective collision groups
- **Fixed-point arithmetic** — `fixed16` type with polynomial sin/cos/atan2, Newton-Raphson sqrt, and branchless min/max/abs
- **Zero external dependencies** — only the C++ standard library
- **Zero-allocation hot paths** — all temporaries are pre-allocated as cache members on the simulator

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

A browser-based demo using WebAssembly + Three.js is in the `web/` directory. To build:

```sh
# Install emscripten (one-time)
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest && cd ..

# Build WASM
source emsdk/emsdk_env.sh
mkdir -p build-web && cd build-web
emcmake cmake .. -DHOP_BUILD_WEB=ON -DHOP_BUILD_TESTS=OFF -DHOP_BUILD_EXAMPLES=OFF
emmake make
cd ..
```

Then serve and open:

```sh
cd web
python3 -m http.server 8080
# Open http://localhost:8080
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
  collision_listener.h   # collision event callback
  manager.h              # spatial partitioning interface
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
