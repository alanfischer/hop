# Rotation support — plan

Roadmap for adding rotation to hop. Current as of June 2026: **static
orientation** (Phase 4), the **real traceable contact point** (the Phase 6/9
prerequisite), **kinematic angular carry** (Phase 6 — spinning platforms carry
their riders, verified), and the **complete static narrowphase** (Phase 5 — every
collision pair now honors a static orientation) have shipped.

The roadmap builds up to full dynamic rotation in graded milestones, each
shippable on its own:

  static orientation (done) → **kinematic angular carry** (done) → finish the
  static narrowphase (done) → kinematic blocking → dynamic spin under torque →
  angular impulse response → constraints/friction.

The kinematic-carry milestones (6–7) are deliberately ahead of the dynamic ones:
they deliver the GoldSrc rotating-platform feel with no inertia/torque math, and
the dynamic phases reuse the same `angular_velocity` state and contact-point
plumbing. Phase 6 shipped before Phase 5, exactly as anticipated below — the
capsule-rider carry needed nothing from the polytope narrowphase. With Phase 5
now in, the next milestone is **Phase 7** (kinematic blocking + rider yaw carry).

## Status at a glance

| Phase | State | Notes |
|---|---|---|
| 1. Contact points on `collision<T>` | done | `c.impact` is the real witness point on **every** pair, incl. traceables (hop #49 / hop-godot #5) |
| 2. Compound colliders via `shape::local_position` | done | |
| 3. Rotation math primitives (`quat`, `mat3`, `asin`/`acos`) | done | |
| 4. **Static orientation** — `solid.orientation` + `shape.local_rotation`, honored by GJK + traceables | done | |
| 5. Close the static narrowphase gap (rotated polytope×polytope + box-mover-vs-traceable, OBB) | done | oriented polytope×polytope via the world-space Minkowski-CSO sweep (`trace_pair_oriented_polytope` in `collide.h`); box-mover-vs-rotated-traceable via OBB-vs-triangle (`obb_local_vs_triangle` in hop-godot). Both halves shipped |
| 6. Kinematic angular carry (`ω×r` surface velocity; spinning platforms carry riders) | done | `solid.angular_velocity`; solver biases relative velocity by `ω×r`; hop-godot derives ω from the per-frame orientation delta. **Verified in-engine.** Box/convex riders now carried too (Phase 5) |
| 7. Kinematic blocking/crush + rider yaw carry (rotating-door parity) | **next** | not started |
| 8. Dynamic orientation state (angular integration under torque) | pending | |
| 9. Angular impulse response | pending | contact point ready |
| 10. Constraints and friction use angular | pending | |
| 11. Docs, examples, bindings | pending | |

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
- Angular velocity cap (like the existing linear cap).
- Broad-phase AABB inflation for spinning solids: `+= |ω|·dt·r`.
- End-of-step SAT-based penetration recovery.

### Sub-frame rotation: snapshot → substep → conservative advancement

The snapshot holds orientation fixed for a frame's trace and sweeps only
*translation* in the rotated frame, so it loses the continuous guarantee for
rotation: a contact that exists only at an intermediate angle (a thin rod
spinning past a small obstacle — clear at both frame endpoints, sweeping through
it at 90° between) is missed. The improvement spectrum, cheapest first — all
deterministic for fixed-point replay:

1. **Snapshot (today).** One orientation per frame. Fine when per-frame rotation
   is small relative to clearance — which covers GoldSrc doors/platforms and most
   gameplay. Rare overshoot is caught by the end-of-step SAT recovery above.

2. **Angular substepping.** Subdivide the frame into N fixed-orientation
   snapshots, interpolating `R(t)` between `R_old` and `R_new` (slerp / the
   exponential-quat step from the integration decision) and sweeping translation
   in each: `N ≈ ceil(|ω|·dt·r_max / clearance)`. Reuses the existing snapshot
   trace entirely — just a loop — and is the **same lever** as the angular-velocity
   cap: cap `|ω|` to force N=1, or substep to keep the speed. N=1 is today; N=2 is
   the "trace the old and new frame, reconcile the contact" idea; N=k for fast
   spinners. Recommended upgrade; lives in the same Phase 9 slot as the cap.

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

5. **End-of-step penetration recovery** — new code path. SAT over OBB pairs +
   push-out along least-penetrating axis + dampen angular velocity along that
   axis. Needs to preserve determinism for fixed16 replay.

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

### Phase 7 — kinematic blocking/crush + rider yaw carry

GoldSrc rotating doors (a) yaw the rider's view and (b) fire a `blocked`
function (damage/reverse) when an entity pins the rotation.

**Add:**
- **Blocked reporting:** when a kinematic mover can't complete its rotation
  because a rider can't be pushed clear, surface a "blocked by solid X, depth D"
  result (the recovery path already computes penetration depth) so the game
  layer can damage/reverse the mover.
- **Rider yaw carry (mostly game-side):** expose the supporting body's `ω` and
  the existing ground-contact report so the player controller can add the yaw
  delta to facing. hop's surface here is small — just make "what am I standing
  on" + its `ω` queryable.

**End-of-phase state:** rotating-door parity — riders turn with the door and get
crushed/reverse it when pinned.

**Estimated effort:** 2–4 days.

### Phase 8 — dynamic orientation state (free spin under torque)

Now make orientation *evolve* from physics rather than script. Reuses the
`angular_velocity` state from Phase 6.

**Add (always-present runtime state, per the no-template decision):**
`quat<T> orientation` (integrated; `mat3` derived per step), reuse
`angular_velocity`, plus `torque`, `inertia`, `inv_inertia`.

**Dynamic rotation integration** (in each `integrator_type`):
- Exponential quat step: `set_quat_from_axis_angle(dq, ω_hat, |ω|*dt)`,
  `q ← dq * q`.
- Angular velocity step: `ω ← ω + I⁻¹·(τ - ω × (I·ω)) · dt` (gyroscopic
  term kept because it's cheap and stabilizing).
- Angular velocity cap (new `default_max_angular_velocity_component` on
  `scalar_traits`).
- Angular component of deactivation threshold.

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
yet — they rotate through each other if hit.

**Estimated effort:** 1–2 days.

### Phase 9 — angular impulse response

**Rework collision response** to use contact point + lever arm:
- `r_a = col.impact - solid_a.position` (and for b). `col.impact` is now the
  real surface point on every pair (Phase 1 follow-up) — no remaining prereq.
- Effective mass includes angular terms via principal-axis inertia.
- Linear impulse → Δv for both solids.
- Angular impulse → Δω for both solids.
- Friction at contact produces tangential impulse through the same machinery
  (this subsumes existing friction paths *and* the Phase 6 kinematic carry,
  which becomes the infinite-mass limit of this formula).

**Mitigate approach (a) artifacts:**
- Inflate broad-phase AABB for spinning solids by `|ω|·dt·max_radius`.
- End-of-step SAT penetration check for rotating pairs; push out along
  least-penetrating axis; dampen angular velocity along that axis.

**Estimated effort:** 3–5 days.

### Phase 10 — constraints and friction use angular

**Local attach points on `constraint<T>`:** anchor in solid's local frame,
rotates with the solid. Spring force at offset produces torque via lever
arm. Naturally subsumes existing spring behavior when attach point is at
origin.

**Nothing new for friction** — Phase 9 already did the tangential-impulse
work. This phase is mostly documenting that rolling now works.

**Estimated effort:** 2–3 days.

### Phase 11 — docs, examples, bindings

- **README:** retire "translation-only" framing. Document the runtime
  identity-fast-path (zero cost when unrotated) — there is no opt-in flag.
- **Examples:** rotating-platform demo (Phase 6) and a cube tumbling on a ramp
  (Phase 9).
- **Web bindings:** expose `setOrientation`/`getOrientation`,
  `setAngularVelocity`/`getAngularVelocity`, `setInertia` on `HopSolid`.
- **hop-godot wrapper:** already adopted the Phase 4 API and fills `col.impact`;
  extend for angular-velocity carry (Phase 6).
- **Migration guide:** Phase 4 shipped with no API break for existing users
  (identity no-op); document the traceable interface change (`orientation` arg +
  the `impact` contract) for custom `traceable` implementors.

---

## Open decisions

- [x] Fixed-point drift: ~0.1–0.2°/rev with exponential integration. **Accepted.**
      (`test_rotation_drift` bounds are already padded for this.)
- [x] Phase 5 box-mover-vs-traceable: full OBB-vs-triangle, or capsule/sphere
      only? **Resolved: full OBB-vs-triangle shipped** (both halves of Phase 5).
- [ ] Phase 7 yaw carry: how much lives in hop vs. the game controller?

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
- Toadlet port reference:
  `/Users/afischer/personal/toadlet/source/cpp/toadlet/egg/mathfixed/` —
  original quaternion/matrix3x3 ops and fixed-point polynomial asin/acos.
