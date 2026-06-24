# Rotation support — plan

Roadmap for adding rotation to hop. Current as of June 2026: **static
orientation** (Phase 4), the **real traceable contact point** (the Phase 6/9
prerequisite), **kinematic angular carry** (Phase 6 — spinning platforms carry
their riders, verified), the **complete static narrowphase** (Phase 5 — every
collision pair honors a static orientation), **dynamic free spin under torque**
(Phase 8 — solids integrate their own orientation), **angular impulse response**
(Phase 9 — bodies bounce/tumble off what they hit), **angular constraints**
(Phase 10 — off-center anchors torque their bodies), and the **docs/examples/bindings**
(Phase 11) and the **kinematic door blocking + rider yaw carry** data surface
(Phase 7) have all shipped. **The rotation roadmap is complete** — every phase is done.

The roadmap builds up to full dynamic rotation in graded milestones, each
shippable on its own:

  static orientation (done) → **kinematic angular carry** (done) → finish the
  static narrowphase (done) → dynamic spin under torque (done) → angular impulse
  response (done) → constraints/friction use angular (done) → docs/examples/bindings
  (done) → kinematic door blocking + rider yaw carry (done, game-side policy).

Phases were not done strictly in order: 6 before 5 (capsule carry needed nothing
from the polytope narrowphase), and 8/9 before 7 (the dynamic arc is the real library
work; Phase 7 is game-side). The full dynamic-rotation feel is now in — off-center
hits induce spin, spinning bodies tumble and roll, and off-center constraint anchors
torque their bodies (Phase 10), the docs/examples/bindings are updated (Phase 11), and
the rotating-door data (platform ω, blocked depth, rider carry) is complete (Phase 7).
**The rotation roadmap is complete.** The only remaining items are non-phase polish:
in-engine Godot smoke for the newer paths, and an optional perf project on the
long-standing narrowphase/broad-phase hotspots.
Phase 9's deferred end-of-step SAT recovery was **investigated and found redundant** —
the broad-phase inflation + Phase 5 oriented narrowphase + Phase 9 angular response
already stop a fast/thin spinner from tunnelling (measured; see
`test_fast_spinner_no_tunnel`), so a parallel SAT pass would only duplicate that
discovery and damping.

## Status at a glance

| Phase | State | Notes |
|---|---|---|
| 1. Contact points on `collision<T>` | done | `c.impact` is the real witness point on **every** pair, incl. traceables (hop #49 / hop-godot #5) |
| 2. Compound colliders via `shape::local_position` | done | |
| 3. Rotation math primitives (`quat`, `mat3`, `asin`/`acos`) | done | |
| 4. **Static orientation** — `solid.orientation` + `shape.local_rotation`, honored by GJK + traceables | done | |
| 5. Close the static narrowphase gap (rotated polytope×polytope + box-mover-vs-traceable, OBB) | done | oriented polytope×polytope via the world-space Minkowski-CSO sweep (`trace_pair_oriented_polytope` in `collide.h`); box-mover-vs-rotated-traceable via OBB-vs-triangle (`obb_local_vs_triangle` in hop-godot). Both halves shipped |
| 6. Kinematic angular carry (`ω×r` surface velocity; spinning platforms carry riders) | done | `solid.angular_velocity`; solver biases relative velocity by `ω×r`; hop-godot derives ω from the per-frame orientation delta. **Verified in-engine.** Box/convex riders now carried too (Phase 5) |
| 7. Kinematic blocking/crush + rider yaw carry (rotating-door parity) | done | data surface complete: platform `ω` via `body_get_state(ANGULAR_VELOCITY)`, blocked partner+depth via `_body_test_motion` (`collision_depth`/`collider`), rider carry `v+ω×r` via `velocity_at_local` (now consistent across the contact + motion paths). Door/crush policy is a small game-side controller (recipe in Phase 7 detail) |
| 8. Dynamic orientation state (angular integration under torque) | done | `solid.inertia_/inv_inertia_/torque_/orientation_q_`; `integrate_angular` (body-frame Euler eq + gyroscopic + exponential quat step) in `simulator.h`; `inv_inertia==0` opt-out (default); angular cap + deactivation. hop-godot: inertia auto-compute, torque, ω state, orientation writeback |
| 9. Angular impulse response | done | lever-arm impulse in the GS velocity solver (`solve_contacts`): off-center hits transfer linear↔angular, friction induces rolling, Phase 6 carry is the infinite-inertia limit. Gated on `rotates_dynamically()` so non-rotating pairs stay bit-identical. + broad-phase inflation for spinners. (End-of-step SAT recovery deferred) |
| 10. Constraints and friction use angular | done | `constraint<T>` anchors rotate with their solid (`R·local_anchor`) and exert torque via lever arm (`accumulate_constraint_torque`, τ = r×F); anchor velocity carries `ω×r` for damping. Center anchors (the default) stay bit-identical. Friction already shipped in Phase 9 |
| 11. Docs, examples, bindings | done | README de-"translation-only" + opt-in/identity-fast-path framing + traceable migration note; `demo_rotating_platform` (Phase 6 carry); web bindings expose orientation/angular-velocity/inertia/torque; hop-godot wrapper already wired |

---

## Current capabilities (Phases 1–4 + contact points)

What collides correctly under a static orientation today:

- **Traceable targets** (trimesh / heightfield / plane), via the local-frame
  query transform (`Rᵀ` in, `R` out) — the same way GoldSrc traces a rotating
  brush (player into the brush's unrotated frame, trace the hull, map back).
- **GJK rounded×polytope** pairs (sphere/capsule vs box/convex), either side and
  either level (solid/shape) oriented. The mover against a traceable is reduced
  to an explicit capsule/sphere spine.

~~What is not yet honored (→ Phase 5): polytope×polytope pairs … and a box mover
vs a traceable …~~ — **closed in Phase 5.** Oriented polytope×polytope now routes
through the Minkowski-CSO sweep (`trace_pair_oriented_polytope`), and a box mover
is honored against a rotated traceable via OBB-vs-triangle. Identity-orientation
pairs still take the cheaper axis-aligned fast paths unchanged.

**Contact point (`col.impact`).** Every collision pair — including all traceables
— now reports the real world contact point on the collidee surface, not the mover
origin. This is the lever-arm input Phases 6 and 9 need: `r = col.impact −
solid.position`. The `traceable::trace_solid` contract requires implementations
to fill it (`collide.h` no longer fabricates one by copying `col.point`); the
built-in trimesh / heightfield / plane traceables do. It is inert until a
consumer reads it and never feeds back into `time`/`normal`/`depth`, so
fixed-point replay is unaffected.

---

## Key decisions (still valid; don't re-litigate unless someone has new info)

### Swept-OBB strategy for Phase 5: approach (a) + Minkowski reduction

For the remaining static polytope×polytope pairs, OBB-vs-OBB and
OBB-vs-convex_solid collision reduces to a **point-vs-convex_solid sweep** where
the convex_solid is the Minkowski zonotope of the two OBBs (up to 30 planes in
general position). Hop already has swept point-vs-convex_solid, so most of this
is plumbing and Minkowski construction. For the box-mover-vs-traceable case, the
analogous move is an **oriented-box-vs-triangle** test: transform the OBB into
the triangle's local frame (the traceable is already there) and reuse the
existing per-triangle routines.

Approach (a) snapshots orientation at the start of each sub-step. It gives up
hop's "zero overlap, ever" guarantee in exchange for tractability. (Note: the
shipped Phase 4 already snapshots a fixed orientation per frame — same trade,
and the same one GoldSrc makes by rotating brushes discretely per tick.)
Mitigations land in Phase 9:
- Angular velocity cap (like the existing linear cap). **Shipped.**
- Broad-phase AABB inflation for spinning solids: `+= |ω|·dt·r`. **Shipped.**
- ~~End-of-step SAT-based penetration recovery.~~ **Not needed** — the inflation +
  oriented narrowphase + angular response already cover it (see Phase 9).

### Sub-frame rotation: snapshot → substep → conservative advancement

The snapshot holds orientation fixed for a frame's trace and sweeps only
*translation* in the rotated frame, so it loses the continuous guarantee for
rotation: a contact that exists only at an intermediate angle (a thin rod
spinning past a small obstacle — clear at both frame endpoints, sweeping through
it at 90° between) is missed. The improvement spectrum, cheapest first — all
deterministic for fixed-point replay:

1. **Snapshot (default).** One orientation per frame. Fine when per-frame rotation
   is small relative to clearance — which covers rotating-door/platform gameplay and
   most cases. The only thing it misses is a contact present *only* between two frame
   orientations (clear at both endpoints); for that, escalate to (2).

2. **Angular substepping — SHIPPED (opt-in).** Subdivide a fast spinner's frame into
   N fixed-orientation sub-traces, interpolating `R(t)` (the exponential-quat step
   gives it for free) and sweeping translation in each: `N ≈ ceil(|ω|·dt·r_max /
   clearance)`. Implemented exactly as the plan predicted — "reuses the existing
   snapshot trace entirely, just a loop": Pass A calls the *unchanged* `update_solid`
   N times at `dt/N` for a sweep_slide body whose `angular_substeps(...)` returns
   N > 1, so each call rotates a fraction and traces at that interpolated orientation.
   It is the **same lever** as the angular-velocity cap (cap `|ω|` to force N=1, or
   substep to keep the speed). **Opt-in:** `set_angular_substeps_max(n)` (default 1 =
   single snapshot, bit-identical; the whole common path is one `int` compare and the
   `nsub == 1` branch calls `update_solid(s, dt)` exactly as before). `N` grows with
   spin speed up to the cap, gated on `rotates_dynamically()`, so non-fast-spinners
   pay nothing. Deterministic for fixed16 (the count uses a fixed-point reciprocal +
   `to_int`; `dt/N` is a fixed-point divide). The frame's external force/torque are
   re-applied per sub-step so they still sum to one frame's impulse. Scope:
   **sweep_slide bodies only** — speculative-mode bodies use a different
   discover/commit model and are not substepped. Test: `test_angular_substep_ccd`
   (an 80 rad/s blade that tunnels a peg at N=1 strikes it reliably at N=8; float +
   fixed16).

3. **Rotational conservative advancement.** Extend hop's existing translation
   `conservative_advance` with the rotational term — bound the max approach speed
   of any point by `|ω|·r` and advance under the combined screw motion for a true
   TOI. Principled but heavy: each iteration re-evaluates the closest point at the
   interpolated `R(t)` (a rotated per-triangle query), and the bound math needs
   fixed-point care. Reserve for a game with fast, thin, gameplay-critical
   spinners (a blade trap you must not clip through). Note: Jolt's CCD is
   linear-only and PhysX largely substeps — true angular CCD is nowhere
   cheap-and-standard, so this is an escape hatch, not a planned phase.

What does **not** work as a shortcut: tracing the two frame endpoints and
interpolating the contact point as a *replacement* for the above. It catches
contacts present at either endpoint (strictly better than one snapshot) but
cannot reconstruct one that exists only between them — interpolating two misses
is still a miss. That is exactly the N=2 case of substepping; use it as such, not
as a substitute.

### Inertia model: principal-axis diagonal only

Store inertia as a `vec3<T>` (Ix, Iy, Iz), not a full 3×3 tensor. Simpler
world-space transform and avoids fixed-point matrix inverse. Works correctly
for sphere/capsule/box (natural principal axes). If someone later needs a
non-aligned inertia tensor, that's a separate feature request.

### Integration: exponential, not forward Euler

Forward-Euler integration accumulates ~0.9°/revolution in fixed16 (1.2°/rev in
float) — and no amount of renormalization helps, because the error is in the
step direction, not the magnitude. Use exponential integration instead:

```cpp
T speed = length(omega);
if (speed > epsilon) {
    vec3<T> axis = omega * (T(1) / speed);
    quat<T> dq;
    set_quat_from_axis_angle(dq, axis, speed * dt);
    quat<T> q_new;
    mul(q_new, dq, q);
    q = q_new;
}
```

Exact for constant ω, ~same cost as forward-Euler, and reduces drift to
quantization-floor only (~0.1–0.2°/rev in fixed16). `set_quat_from_axis_angle`
is essential for this — it's **not** user-facing gravy.

Note: Phase 4 stores orientation as `mat3<T>` (matches Godot `Basis`, avoids a
per-query quat→matrix conversion). The dynamic phases integrate a `quat<T>`
(for drift-free exponential stepping) and derive the `mat3` from it each step.

---

## Known hard parts

1. **Fixed-point quaternion drift** — baseline ~0.1°/rev with exponential
   integration. Fine for most gameplay; visible only if a solid spins
   continuously for minutes. Mitigation: clamp angular velocity, or audit where
   precision is lost (sqrt in normalize, polynomial sin/cos). Re-run
   `test_rotation_drift` after Phase 8 lands; its bounds are padded for this.

2. **Minkowski zonotope of two OBBs** — up to 30 planes in general position
   (15 axis pairs × 2 sides): 3 face normals from each OBB (6 planes) + 9
   edge-edge cross products (18 planes, paired). Degenerate cases (parallel
   edges) collapse to fewer planes. Needs careful fixed-point handling — cross
   products of unit-ish axes can produce noise that looks like false planes.

3. **Contact point → angular impulse** — the response formula gains lever-arm
   cross products: `J = -(1+e) · v_rel·n / (1/m_a + 1/m_b +
   ((I_a⁻¹·(r_a×n))×r_a + (I_b⁻¹·(r_b×n))×r_b)·n)`. With principal-axis diagonal
   inertia, `I⁻¹·v` is component-wise `(v.x/Ix, v.y/Iy, v.z/Iz)` — but only in
   the solid's local frame. Rotate into local, divide, rotate back. (The
   contact point `r` is now available — see Current capabilities.)

4. **Broad-phase refit cost** — rotating solids change world AABB every frame.
   Phase 4 already recomputes the oriented world AABB (`rotate_aabb`) on every
   `set_orientation`, so static spinners refit each step. Probably fine for
   hop-scale scenes but measure on worst case.

5. ~~**End-of-step penetration recovery**~~ — **resolved without new code.** The
   intended SAT-over-OBB-pairs pass turned out to duplicate work already done: the
   Phase 5 oriented narrowphase discovers the overlap, the existing position recovery
   pushes out along the contact normal, and the Phase 9 angular solver damps the
   spin at the lever arm. Verified against a pinned fast spinner
   (`test_fast_spinner_no_tunnel`). See Phase 9 for the full reasoning.

---

## Phase-by-phase detail

### Phase 5 — close the static narrowphase gap (oriented polytope pairs) — **SHIPPED**

Finished what Phase 4 scoped out: **every** pair now honors a static orientation
(prerequisite for both static completeness and dynamic OBBs tumbling).

**Shipped:**
- Oriented polytope×polytope (box×box, box×convex, convex×convex) via
  `trace_pair_oriented_polytope` (`collide.h`). The Minkowski configuration-space
  obstacle `N = S_B ⊖ S_A` is built directly in **world space** (mover-relative,
  so fixed16 support dot products stay in shape-local magnitudes) as a
  `convex_solid<T>`, then swept via `conservative_advance` with the CSO's
  plane-exact deepest-face distance (see *Resting-contact robustness* below for why
  this rather than `trace_convex_solid`'s per-face entry). The
  plane set is the safe tangent-plane construction noted under *Key decisions*:
  both shapes' face normals (bounds `N`) plus the `edge_A × edge_B` cross products
  (tightens OBB×OBB to exact). Implementation realities not in the original
  sketch: (a) **coincident planes are deduplicated** — a box edge crossed with the
  other box's axis reproduces a face normal, and an exact duplicate makes the
  plane trace reject its own entry under fixed-point rounding (the recomputed entry
  point lands an ulp outside its twin); (b) convex edge directions are the
  deduplicated pairwise cross products of the shape's face normals (a safe
  superset), capped. Dispatch is gated on `pair_is_oriented` (any non-identity
  orientation/local-rotation); identity pairs keep the cheaper axis-aligned paths.
- Oriented-box-vs-triangle so a **box mover** is honored against a rotated
  traceable (trimesh + heightfield share `hoptri::trace_solid_rotated`). The box
  is reduced to an OBB in the target's local frame and run through
  `obb_local_vs_triangle` — the same winding-agnostic support-plane test the AA
  box path uses, expressed locally and filling `col.impact` (the projected point
  on the triangle), mapped back to world by the driver.
- `col.impact` is now **orientation-aware** for every primitive pair (`collide.h`
  rotates sh1's support by its world rotation) — this also corrected the latent
  un-rotated impact on the existing oriented GJK pairs.
- **Resting-contact robustness.** Honoring rotation in the narrowphase also requires
  it in the **broad phase** — the real root cause of a dynamic oriented box falling
  through the floor: the per-step broad-phase query box (`update_solid` and
  `integrate_and_discover`) was built from the **un-rotated `local_bound_`**, so an
  oriented box (reaching √2·half past its AABB) queried a box too small to reach the
  floor and tunnelled until it sank deep enough — then snapped back with injected
  energy. Masked with a single body (the floor's own recovery keeps a lone box up),
  visible the moment a second body exists (the floor recovers only the closest per
  tick). Fixed by querying the cached orientation-aware **`world_bound_`**
  (`rotate_aabb(local_bound_, orientation_) + position_`, already maintained on every
  move/reorient). Two supporting changes: the oriented sweep drives
  `conservative_advance` with the CSO's plane-exact deepest-face distance
  `max_i(n_i·xA − d_i)` (the same swept-contact machinery the GJK pairs use — clean
  resting, no GJK simplex / no fixed-point loss; `trace_convex_solid`'s per-face entry
  misses edge/vertex contacts on approach); and `update_solid`'s penetration recovery
  no longer relocates an **infinite-mass mover** (the old split checked only the
  partner's mass — a correct-invariant fix for a latent bug, though the broad-phase
  fix alone keeps the floor still in practice). Regression: `test_oriented_box_rest`
  in `tests/test_simulator.cpp` drops a rotated box *with a second body present* and
  asserts it settles on its edge at z≈0.707 with a tight steady-state band (no
  fall/snap) and the floor never moves (float + fixed16).
- Broad phase: the per-solid `world_bound_` was already orientation-rotated, but the
  per-*step* query box was not (the *Resting-contact robustness* bug above). Now both
  the cached bound and the per-step query honor orientation.

**Tests:** `test_oriented_polytope` in `tests/test_gjk.cpp` (float/fixed16/fixed32:
rotated box×box stops earlier than unrotated, box/convex agreement, rotated-mover
impact); `test_box_mover_vs_rotated_mesh` in hop-godot's
`tests/test_trimesh_traceable.cpp` (swept + static box vs a rotated mesh).

**Preserves:** hop's per-frame "snapshot" model from Phase 4. Solid API
unchanged. Identity orientation stays an exact no-op.

**Verification:** the raylib demo `examples/demo_static_rotation.cpp` validates it
visually — tilted boxes drop and rest on their true rotated faces (and exposed the
broad-phase bug). **Still owed:** the hop-godot test-project smoke (a rotated
box/convex prop in Godot) — the runtime wiring feeds orientation
(`hop_body_data.cpp`, `hop_shape_data.cpp`) but has not been eyeballed in-engine.

### Phase 6 — kinematic angular carry (GoldSrc `func_rotating` parity) — **SHIPPED**

A spinning kinematic platform **carries** the bodies resting on / touching it,
the way the existing kinematic *linear* movers carry riders. No inertia, torque,
or angular impulse — the platform's spin is scripted; physics just transports
riders.

**Shipped:**
- `vec3<T> angular_velocity_` on `solid` (axis·rate; pivot = `position`), with
  set/get. Always-present, zero when unset, reset in `reset()`.
- The contact solver biases the relative velocity by the **surface velocity at
  the contact point**: it precomputes per-pair `v_bias = ω_b×r_b − ω_a×r_a`
  (`r = touch.impact − position`) once at build, then adds it everywhere the GS
  solver and restitution snapshot read `v_b − v_a` (`vn0`, the normal solve, and
  the friction sweep). An infinite-mass kinematic pusher therefore drags the
  rider tangentially through the existing non-penetration / friction path — no
  new resolution code. The real contact point is now persisted into the `touch`
  slot (`add_or_refresh_touch` takes `col.impact`), correct for trimesh brushes
  too. Zero `ω` makes `v_bias` exactly zero (skipped) — an exact translation-only
  no-op. Test: `test_angular_carry` in `tests/test_simulator.cpp` (float +
  fixed16; a rider orbits a spinning platform, a no-spin control stays put).
- **hop-godot:** the kinematic pre-step loop (alongside the linear
  `velocity = delta/dt` block) derives `ω` from the per-frame orientation delta
  (`ΔR = R_new·R_oldᵀ` → `set_quat_from_mat3` → `get_axis_angle_from_quat` →
  `ω = axis·angle/dt`), commits the new orientation, and calls
  `set_angular_velocity`; the post-step snap-back zeroes `ω` (orientation is
  already the frame target). A per-frame angle past ~0.5 rad is treated as a
  placement snap (discontinuous), mirroring the positional teleport guard.
  `_body_set_state(ANGULAR_VELOCITY)` still only stores (kinematic carry is
  transform-derived, like kinematic linear velocity; a directly-set `avelocity`
  on a *dynamic* body awaits Phase 8 integration).

**Scope:** capsule/sphere riders are carried correctly now; box/convex riders
(crates on a platform) wait on Phase 5's polytope narrowphase. wizardwars riders
are capsules, so this is fully usable.

**End-of-phase state:** stand on a `func_rotating`, get carried around. The
single biggest gameplay payoff in the roadmap.

**Verified:** in-engine smoke test (a `func_rotating` platform in the test-project
carrying the player) confirmed. Box/convex riders are now also carried correctly
(Phase 5 closed the polytope narrowphase they depended on).

### Phase 7 — kinematic blocking/crush + rider yaw carry — **SHIPPED**

Rotating-door parity (rotating-door style): (a) yaw the rider's view and (b) let the
mover detect when an entity pins its rotation (damage/reverse). As the plan predicted,
hop's surface here is small and the behavior is game-side — and it turned out the data
surface was **almost entirely already in place** from Phases 6/8/9 and the existing
hop-godot body-state work. The open "how much lives in hop vs the game controller"
decision resolves to: **the data lives in hop/hop-godot (now complete); the door/crush
policy lives in the game.** What was verified/added:

- **Rider yaw carry — data already exposed.** A spinning kinematic platform/door
  publishes its derived `ω` (from the per-frame orientation delta, Phase 6) into the
  Godot body state, so the game reads it with
  `PhysicsServer3D.body_get_state(floor_rid, BODY_STATE_ANGULAR_VELOCITY)` and adds the
  up-axis component to the player's facing each frame. The *tangential* (positional)
  carry was already correct via the contact/​motion surface velocity `v + ω×r`.
- **Blocked / crush — already reported.** A kinematic door driven by `move_and_collide`
  gets the blocking partner and penetration depth straight from the motion result
  (`_body_test_motion` fills `collision_depth` and `collider` / `collider_id`), so the
  game reads `KinematicCollision3D.get_collider()` / `get_depth()` and decides to
  damage or reverse. No new hop API needed — the recovery path already computed the depth.
- **The one real gap fixed:** the `_integrate_forces` contact path (`HopBodyData::on_collision`)
  reported the collider's **linear-only** velocity, while the `_body_test_motion` path
  correctly returned `v + ω×r` (`velocity_at_local`). A dynamic `RigidBody3D` rider on a
  rotating platform therefore read an incomplete carry velocity from its contacts. Now
  both paths use `velocity_at_local`, so the rotational carry term is consistent
  regardless of how a rider queries it.

**GDScript recipe (game-side):**

```gdscript
# Rider yaw carry — turn the player with the platform it stands on.
if is_on_floor():
    var floor_rid := get_last_slide_collision().get_collider_rid()
    var w := PhysicsServer3D.body_get_state(floor_rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY) as Vector3
    rotate_y(w.y * delta)   # carry facing with the platform's yaw

# Blocked / crush — a rotating door damages or reverses when it pins a body.
var col := move_and_collide(rotation_step_motion)
if col and col.get_depth() > crush_threshold:
    var victim := col.get_collider()
    if victim.has_method("take_damage"): victim.take_damage(crush_dps * delta)
    reverse_or_pause()       # back the door off the pinned entity
```

**End-of-phase state:** rotating-door parity — riders turn with the door and get
crushed/reverse it when pinned. Core/binding data surface complete; the door behavior
is a small game-side controller (the recipe above).

### Phase 8 — dynamic orientation state (free spin under torque) — **SHIPPED**

Orientation now *evolves* from physics rather than script. Reuses the
`angular_velocity` state from Phase 6.

**Shipped (always-present runtime state):** `quat<T> orientation_q_` (integrated;
the queried `mat3 orientation_` is derived from it each step and stays in sync with
`set_orientation`), plus `torque_`, `inertia_`, `inv_inertia_` on `solid` (`solid.h`),
with `set_inertia` / `add_torque` / `rotates_dynamically` accessors.

**Dynamic rotation integration** — `simulator::integrate_angular` (`simulator.h`),
called in Pass A by both contact modes after the linear step:
- Euler's equation in the **body frame** (inertia diagonal there): rotate ω/τ in by
  `Rᵀ`, `ω̇_b = I⁻¹·(τ_b − ω_b × (I·ω_b))` with the gyroscopic term kept, rotate ω
  back to world. (ω is stored world-frame, matching the Phase 6 `ω×r` carry.)
- Exponential quat step `q ← dq·q` (drift-free for constant ω), renormalized;
  `solid::set_orientation_from_quat` refreshes the queried mat3 (`set_mat3_from_quat`)
  and recomputes the world bound as one unit, keeping the Phase 5 oriented broad
  phase tracking the spin.
- Angular velocity cap: runtime-configurable via `simulator::set_max_angular_velocity_component`
  (0 = off), defaulted from `default_max_angular_velocity_component` on `scalar_traits`
  (all four scalar types). Applied both at integration and post-contact-solve.
- Angular deactivation term (a dynamically-spinning body won't sleep mid-rotation).
- Global `set_angular_integration(bool)` toggle (A/B + replay-bisection lever).

**hop-godot wiring** (so a Godot `RigidBody3D` spins): `BODY_PARAM_INERTIA` set/get
+ auto-compute from the collision AABB×mass when the game leaves it zero;
`apply_torque` / `apply_torque_impulse` / `constant_torque` → `add_torque` / Δω; a
dynamic `BODY_STATE_ANGULAR_VELOCITY` seeds the integrated spin; `sync_from_hop`
writes the integrated orientation (+ω) back to the body transform (scale preserved),
gated on `rotates_dynamically()` so non-spinning bodies stay Godot-authoritative.

**Disabling dynamic rotation (the documented toggle).** Through Phase 7,
"ignore rotation" needs *no* switch: orientation and `angular_velocity` are
zero/identity by default and take the exact identity fast path (`collide.h`
gates on `Ra != identity || Rb != identity`; a zero `ω` makes the Phase 6
`ω × r` surface-velocity term vanish). Dynamic spin is the only kind that
appears *uninvited* — a finite-inertia body hit off-center acquires `ω` whether
or not the game wanted it — so it is the one kind that needs an explicit
off-switch. The contract:

- **Per-solid:** `inv_inertia == 0` means "this body never rotates dynamically."
  Store `inv_inertia` as the primary state (not derived from `inertia`), because
  zero is exact in `fixed16` whereas infinite `inertia` is not — and every
  angular term in the response formula (hard part #3) is gated by `I⁻¹`, so a
  zero `inv_inertia` makes them all vanish to a multiply-by-zero rather than a
  branch. This is the infinite-moment-of-inertia limit, stored as the reciprocal
  so it stays numerically clean. It is the canonical marker for static/kinematic
  level brushes (which must never tumble) and for any prop the game wants to
  pin upright. `inv_inertia` defaults to **zero**, so a solid spins only once a
  game explicitly gives it a finite inertia — rotation is opt-in, not opt-out.
- **Global:** angular integration lives entirely in `integrator_type`, so a
  simulator-level "skip angular integration" early-out is trivial. Keep it — it
  is the clean A/B and deterministic-replay-bisection lever for the dynamic
  phases.

**End-of-phase state:** solids spin freely under torque; no collision response
yet — they rotate through each other if hit (Phase 9).

**Tests:** `test_dynamic_spin` in `tests/test_simulator.cpp` (float + fixed16: free
spin keeps ω, torque spins up at `I⁻¹τt`, `inv_inertia==0` never rotates, runaway
torque hits the cap); existing `test_rotation_drift` (`test_quat.cpp`) covers the
exponential-integration drift bound. Demo: a freely-spinning (asymmetric-inertia)
gold box added to `examples/demo_static_rotation.cpp`. hop core suite +
hop-godot addon build green; UBSan-clean on the integration paths.

**Verification still owed:** in-engine Godot smoke — a `RigidBody3D` with an applied
torque actually tumbles (the C++ integration is tested and the addon compiles +
links, but the Godot wiring has not been run in-engine).

### Phase 9 — angular impulse response — **SHIPPED**

The GS velocity solver (`solve_contacts` in `simulator.h`) now resolves contacts at
the real contact point with a lever arm, so a body bounces/tumbles off what it hits.

**Shipped:**
- Lever arms `r_a/r_b = slot.impact − position` and an angular-aware **effective
  normal mass** `k_n = inv_m_sum + n·((Iₐ⁻¹(rₐ×n))×rₐ) + n·((I_b⁻¹(r_b×n))×r_b)`,
  precomputed per pair (orientation fixed during the solve). `apply_inv_inertia_world`
  does the body-frame `R·(invI ∘ Rᵀ·v)` round-trip; the effective mass along any
  direction (normal `k_n` and the per-sweep friction slip direction) shares one
  `angular_eff_mass(pair, dir)` helper.
- `apply_pair_impulse` now also torques each finite-inertia body:
  `ω_a −= Iₐ⁻¹(rₐ×J)`, `ω_b += I_b⁻¹(r_b×J)`. The relative velocity is the **live
  surface velocity** `v + ω×r` at the contact (recomputed each sweep), and `vn0` /
  restitution derive from it.
- **Friction** uses the tangent effective mass along the live slip direction, so a
  tangential contact torques the body (rolling).
- The **Phase 6 kinematic carry is subsumed**: for an `inv_inertia==0` body ω is
  fixed, so `ω×r` *is* the old `v_bias` and the impulse changes no ω (infinite-inertia
  limit).
- **Gating:** the whole angular path activates only when a pair has a body with
  `rotates_dynamically()` (`inv_inertia != 0`). Every other pair takes the original
  linear + `v_bias` solve **bit-identical** — Phase 6 and all prior tests unchanged.
- **Broad-phase inflation** for spinners (`spin_broadphase_reach = |ω|·dt·r_max`)
  added to both per-step query boxes (0 for non-spinners).

**Tests** (`tests/test_simulator.cpp`, float + fixed16): `test_angular_impulse`
(off-center hit induces spin of the right sign + steals linear speed; centered hit
gives ~none), `test_friction_rolling` (a sliding box tips forward via friction
torque). Phase 6 `test_angular_carry*` unchanged. Full suite green; UBSan-clean;
hop-godot addon recompiles. Demo: `examples/demo_bounce.cpp`'s free box has finite
inertia + oriented rendering — it now tumbles off the walls/floor (the headline
payoff to "why doesn't the bounce demo tumble").

**End-of-step SAT recovery — investigated, found redundant.** This was deferred out
of Phase 9 as the hardening item for fast/thin spinners that sweep through a thin
obstacle between orientation snapshots. Measuring the actual failure case showed it is
already covered by three shipped mechanisms acting together: (1) the Phase 9
broad-phase inflation `|ω|·dt·r_max` widens the query so the swept trace still finds
the contact; (2) the Phase 5 oriented narrowphase discovers the OBB overlap at the
endpoint orientation; (3) the existing position recovery pushes the bodies apart and
feeds the contact (with `separation = −depth`) into the Phase 9 angular solver, which
damps the spin-driven closing velocity at the lever arm. A blade spinning at 40 rad/s
(tip sweeping 1.28 units/step against a 0.2-thick wall), **pinned at its center so it
cannot recoil**, is still stopped at the wall's near face with its spin arrested
(`test_fast_spinner_no_tunnel`, float + fixed16). A standalone SAT pass would only
re-derive that same oriented overlap and angular damping, so it was not built. The one
case the snapshot model genuinely missed — true mid-rotation tunnelling, a contact
present only *between* two snapshots (clear at both endpoints) — is now closed by the
**opt-in angular substepping** shipped under *Sub-frame rotation* above
(`set_angular_substeps_max`, `test_angular_substep_ccd`). Default is the single
snapshot (bit-identical); a game with fast/thin spinners raises the cap. Only the
heaviest level (rotational conservative advancement) remains unshipped, reserved as an
escape hatch.

**Owed:** in-engine Godot smoke (a `RigidBody3D` tumbling off a wall).

### Phase 10 — constraints and friction use angular — **SHIPPED**

**Local attach points on `constraint<T>`** now rotate with their solid and torque it:

- The anchor's world lever arm is `R·local_anchor` (`anchor_lever`), so the anchor
  tracks the body's orientation. A zero anchor — the default — gives a zero lever, so
  every existing center-anchored constraint is **bit-identical** (`test_constraint_anchor_torque`'s
  centered case spins at exactly 0, float and fixed16).
- The spring/damping force at that lever exerts torque `τ = r × F`, summed once per
  step into `torque_` by `accumulate_constraint_torque` (called inside
  `integrate_angular`, alongside game-applied torque, only for `rotates_dynamically()`
  bodies). The per-constraint force is factored into `constraint_force_on`, shared by
  the linear `constraint_link` and the angular torque pass (one force function, two
  consumers — no duplicated spring math).
- The anchor's **velocity carries `ω×r`** (`anchor_velocity`) so the damping term
  resists spin too; zero lever ⇒ plain body velocity ⇒ bit-identical.

**Nothing new for friction** — Phase 9 already did the tangential-impulse work; a
sliding/rolling body torques itself through the contact. This phase adds the
constraint-anchor half.

**Tests:** `test_constraint_anchor_torque` in `tests/test_simulator.cpp` (float +
fixed16): an off-center spring pull spins the body about +z while a centered pull
gives pure translation (exactly zero spin). Full suite green; UBSan+ASan clean.

**Owed:** in-engine Godot smoke (a joint/spring anchored off-center actually torquing
a `RigidBody3D`) — the hop-godot binding does not yet plumb per-joint local anchors.

### Phase 11 — docs, examples, bindings — **SHIPPED**

- **README:** the "translation-only" framing is retired. The *Design Philosophy*
  section now documents rotation as **opt-in by construction** via the runtime
  identity fast path (no template flag, no global switch — you pay only on bodies
  that set an orientation/inertia/angular velocity), and the *Features* list calls out
  the rotation capabilities.
- **Examples:** `examples/demo_rotating_platform.cpp` — a Phase 6 showcase: a spinning
  infinite-mass platform carries its dynamic riders around by friction (`ω×r` bias),
  and the riders' own inertia spins them up (Phase 9). Verified headlessly (a rider
  orbits a half-circle at radius ≈3, stays on the platform, self-spins to the surface
  rate). Dynamic tumbling is already covered by `demo_bounce` (free box, octahedron,
  rolling sphere, constraint-linked bola) and `demo_static_rotation` (static tilt +
  free spin), so no separate ramp demo was added.
- **Web bindings:** `HopSolid` now exposes `setOrientation`/`getOrientation`
  (quaternion `{x,y,z,w}`), `setAngularVelocity`/`getAngularVelocity`, `setInertia`,
  and `addTorque` (`web/hop_bindings.cpp`). The WASM module rebuilds cleanly via the
  canonical `HOP_BUILD_WEB` CMake path.
- **hop-godot wrapper:** already adopted the Phase 4 API, fills `col.impact`, and
  carries angular velocity (Phase 6) + dynamic spin/impulse (Phase 8/9) — shipped in
  the earlier hop-godot PRs.
- **Migration guide:** the README's *Migrating a custom `traceable` for rotation*
  subsection documents the two interface changes for custom traceable implementors
  (honor the mover's `orientation` via the `Rᵀ`/`R` local-frame trace; fill the real
  `col.impact` witness point). The public `solid`/`simulator` API stayed
  source-compatible (identity no-op).

---

## Open decisions

- [x] Fixed-point drift: ~0.1–0.2°/rev with exponential integration. **Accepted.**
      (`test_rotation_drift` bounds are already padded for this.)
- [x] Phase 5 box-mover-vs-traceable: full OBB-vs-triangle, or capsule/sphere
      only? **Resolved: full OBB-vs-triangle shipped** (both halves of Phase 5).
- [x] Phase 7 yaw carry: how much lives in hop vs. the game controller? **Resolved:
      the data surface lives in hop/hop-godot (platform `ω` via `body_get_state`, blocked
      depth via `_body_test_motion`, rider carry `v+ω×r`); the door/crush policy is a
      small game-side controller. See Phase 7 detail for the GDScript recipe.**

---

## Resolved decisions (kept for the record)

- ~~Traceables stay world-axis-aligned~~ — **reversed in Phase 4**; traceables
  now rotate with their solid via the local-frame query transform.
- ~~`WithRotation` template parameter on `solid`/`simulator`/`collision`~~ —
  **dropped**; orientation (and later angular state) is always-present runtime
  state with an exact identity fast path. No API break, no opt-in.
- **Rotation opt-out** — through Phase 7, nothing is needed (identity/zero
  defaults take the no-op fast path). For the dynamic phases (8+), the toggle is
  per-solid `inv_inertia == 0` (default), plus a global integrator early-out for
  A/B and replay bisection. Rotation is opt-in. See Phase 8's "Disabling dynamic
  rotation" note.
- **Traceable contact point** — **shipped** (hop #49 / hop-godot #5). Was the
  old "hard part #6": `col.impact` fell back to `col.point` for traceables.
  Now `trace_solid` fills the real surface witness point (contract in
  `traceable.h`) and `collide.h` no longer clobbers it. Required by Phases 6/9;
  no longer a blocker.

---

## References in this repo

- Phase 2: `shape::local_position` (compound colliders); tests in
  `tests/test_compound.cpp`.
- Phase 3: `quat<T>`, `mat3<T>`, scalar-traits extensions; tests in
  `tests/test_quat.cpp`, `tests/test_mat3.cpp`, plus a drift-measurement test
  relevant again at Phase 8.
- Phase 4 (static orientation): `solid.orientation` / `shape.local_rotation`,
  GJK oriented support, rotated traceable interface; tests in `tests/test_gjk.cpp`
  (`test_gjk_solid_orientation`) and `tests/test_collision.cpp`
  (`test_traceable_orientation`).
- Contact point (hop #49 / hop-godot #5): the `traceable::trace_solid` impact
  contract in `traceable.h`, the no-clobber in `collide.h`, and tests
  `test_impact_traceable_floor` (hop) / `test_contact_point_on_surface`
  (hop-godot).
- Phase 5 (oriented polytope narrowphase): `trace_pair_oriented_polytope` +
  `build_world_polytope` / `build_polytope_cso` / `pair_is_oriented` and the
  orientation-aware `col.impact` block in `collide.h`; tests
  `test_oriented_polytope` in `tests/test_gjk.cpp`. Box-mover-vs-traceable:
  `obb_support` / `obb_local_vs_triangle` in hop-godot's
  `src/hop_triangle_collision.h` (driven by `trace_solid_rotated`, which gained a
  `seam_tol` arg); test `test_box_mover_vs_rotated_mesh` in
  `tests/test_trimesh_traceable.cpp`. Resting-contact fixes: the orientation-aware
  broad-phase query (cached `world_bound_`) in `update_solid` / `integrate_and_discover`
  (`simulator.h`, the root cause), the oriented sweep on `conservative_advance` with
  the CSO deepest-face distance (`collide.h`), and the infinite-mass recovery guard in
  `update_solid` (`simulator.h`); regression `test_oriented_box_rest` (two-body) in
  `tests/test_simulator.cpp`. Visual demo: `examples/demo_static_rotation.cpp`
  (raylib; tilted boxes settle on their rotated faces, built under
  `-DHOP_BUILD_EXAMPLES=ON`).
- Phase 6 (kinematic angular carry): `solid.angular_velocity_` +
  `set/get_angular_velocity` and `touch.impact` in `solid.h`; the per-pair
  `v_bias` build and its use in the GS normal/friction sweeps in `simulator.h`
  (`solve_contacts`); `add_or_refresh_touch` now carries `col.impact`. hop-godot:
  the ω-from-orientation-delta derivation in `HopPhysicsServer::_step`
  (kinematic pre-step loop). Test: `test_angular_carry` in
  `tests/test_simulator.cpp`.
- Phase 8 (dynamic free spin): `solid.inertia_/inv_inertia_/torque_/orientation_q_`
  + `set_inertia`/`add_torque`/`rotates_dynamically` in `solid.h`;
  `simulator::integrate_angular` + the `angular_integration_` toggle + angular
  deactivation in `simulator.h`; `default_max_angular_velocity_component` in
  `scalar_traits.h`. hop-godot: `update_body_inertia`/`aabb_box_inertia`,
  `BODY_PARAM_INERTIA`, torque methods, dynamic `BODY_STATE_ANGULAR_VELOCITY` in
  `hop_physics_server.cpp`; orientation writeback in `HopBodyData::sync_from_hop`;
  `to_godot_basis` in `hop_conversions.h`. Test: `test_dynamic_spin` in
  `tests/test_simulator.cpp`; demo `examples/demo_static_rotation.cpp` (gold box).
- Phase 9 (angular impulse response): `apply_inv_inertia_world`,
  `spin_broadphase_reach`, and the `contact_pair` `r_a/r_b/has_angular/eff_n` +
  angular `apply_pair_impulse` / `contact_point_vrel` / friction tangent-mass in
  `solve_contacts` (`simulator.h`). Tests `test_angular_impulse` /
  `test_friction_rolling` in `tests/test_simulator.cpp`; demo `examples/demo_bounce.cpp`
  (the free box tumbles).
- Phase 10 (constraint anchors use angular): `constraint<T>::local_anchor_a_/b_`
  rotated by the solid orientation in `is_loaded` (`constraint.h`); `constraint_force_on`
  (shared per-constraint force + lever), `accumulate_constraint_torque`, `anchor_lever`,
  `anchor_velocity`, and the torque hook in `integrate_angular` (`simulator.h`). Test:
  `test_constraint_anchor_torque` in `tests/test_simulator.cpp`.
- Phase 9 SAT-recovery investigation (found redundant): no production code; regression
  `test_fast_spinner_no_tunnel` in `tests/test_simulator.cpp` (a pinned blade spinning
  at 40 rad/s is stopped at a thin wall, spin arrested) guards the broad-phase
  inflation + oriented narrowphase + angular response that subsume it.
- Angular substepping (opt-in CCD): `set_angular_substeps_max` / `angular_substeps()` /
  `angular_substep_clearance_` and the Pass-A substep loop (`update_solid` called N×
  at `dt/N`) in `simulator.h`. Test: `test_angular_substep_ccd` in
  `tests/test_simulator.cpp` (an 80 rad/s blade tunnels a peg at N=1, strikes it at N=8;
  float + fixed16).
- Phase 11 (docs/examples/bindings): the *Design Philosophy* + *Features* +
  *Migrating a custom `traceable` for rotation* sections in `README.md`; the
  `setOrientation`/`getOrientation`/`setAngularVelocity`/`getAngularVelocity`/`setInertia`/`addTorque`
  bindings in `web/hop_bindings.cpp`; the `demo_rotating_platform` target in
  `CMakeLists.txt` + `examples/demo_rotating_platform.cpp`.
- Toadlet port reference:
  `/Users/afischer/personal/toadlet/source/cpp/toadlet/egg/mathfixed/` —
  original quaternion/matrix3x3 ops and fixed-point polynomial asin/acos.
