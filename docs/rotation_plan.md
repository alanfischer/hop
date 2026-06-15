# Rotation support — plan

Roadmap for adding rotation to hop. Drafted April 2026; revised June 2026 after
**static orientation shipped** on the `static-rotation` branch.

The roadmap now builds up to full dynamic rotation in graded milestones, each
shippable on its own:

  static orientation (done) → finish the static narrowphase → **kinematic
  angular carry / blocking** (GoldSrc `func_rotating` parity) → dynamic spin
  under torque → angular impulse response → constraints/friction.

The kinematic-carry milestones (6–7) are deliberately ahead of the dynamic ones:
they deliver the GoldSrc rotating-platform feel with no inertia/torque math, and
the dynamic phases reuse the same `angular_velocity` state and contact-point
plumbing they introduce.

## Status at a glance

| Phase | State | Branch / notes |
|---|---|---|
| 1. Contact points on `collision<T>` | done | `c.impact` already exists |
| 2. Compound colliders via `shape::local_position` | done | `rotation/phase-2-local-offset` |
| 3. Rotation math primitives (`quat`, `mat3`, `asin`/`acos`) | done | `rotation/phase-3-rotation-math` |
| 4. **Static orientation** — `solid.orientation` + `shape.local_rotation`, honored by GJK + traceables | **done** | `static-rotation` (June 2026) |
| 5. Close the static narrowphase gap (rotated polytope×polytope + box-mover-vs-traceable, OBB) | **next** | not started |
| 6. Kinematic angular carry (`ω×r` surface velocity; spinning platforms carry riders) | pending | not started |
| 7. Kinematic blocking/crush + rider yaw carry (rotating-door parity) | pending | not started |
| 8. Dynamic orientation state (angular integration under torque) | pending | not started |
| 9. Angular impulse response | pending | not started |
| 10. Constraints and friction use angular | pending | not started |
| 11. Docs, examples, bindings | pending | not started |

---

## What Phase 4 actually shipped (and how it diverged from the original plan)

Static orientation landed as a **runtime** feature, not the originally-planned
Minkowski-OBB + `WithRotation` template. Two original "key decisions" were
reversed by it — recorded here so they aren't accidentally re-applied:

- **Orientation is always-present runtime state, not a template parameter.**
  `solid` carries `mat3<T> orientation_` and `shape` carries `mat3<T>
  local_rotation_`, both identity by default. The narrowphase takes an identity
  fast path (`if (R != identity)`) that is an exact, byte-identical no-op, so
  existing translation-only content is unchanged with no compile-time opt-in.
  The dynamic phases (8–10) extend this with always-present `angular_velocity` /
  `inertia` state on the same principle — **the `WithRotation` template idea is
  dropped.**

- **Traceables now rotate with their solid.** The `traceable<T>` interface gained
  an `orientation` argument; trimesh/heightfield/plane honor it by transforming
  the query into the geometry's local frame by `Rᵀ` and mapping the contact back
  by `R` (the grid/triangles never move). This is exactly how GoldSrc traces a
  rotating brush — player into the brush's unrotated frame, trace the hull, map
  back — so the collision **query** side already has GoldSrc rotation parity. The
  old "traceables stay world-aligned" decision is **void.**

What Phase 4 honors today: traceable targets (trimesh/heightfield/plane) and
GJK-eligible **rounded×polytope** pairs (sphere/capsule vs box/convex), with
either side and either level (solid/shape) oriented. The mover against a
traceable is reduced to an explicit capsule/sphere spine.

What it does **not** yet honor (→ Phase 5): **polytope×polytope** pairs
(box×box, box×convex, convex×convex) collide as if unrotated (they route through
`trace_aa_box` / Minkowski-AABB, not GJK), and a **box mover vs a traceable** is
skipped (no rotation-invariant spine). A rotated box/convex therefore only
collides correctly today when it meets a *rounded* shape via the GJK path.

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

### Inertia model: principal-axis diagonal only

Store inertia as a `vec3<T>` (Ix, Iy, Iz), not a full 3×3 tensor. Simpler
world-space transform and avoids fixed-point matrix inverse. Works correctly
for sphere/capsule/box (natural principal axes). If someone later needs a
non-aligned inertia tensor, that's a separate feature request.

### Integration: exponential, not forward Euler

Phase 3's drift test revealed forward-Euler accumulates **0.9°/revolution** in
fixed16 (1.2°/rev in float) — and no amount of renormalization helps, because
the error is in the step direction, not the magnitude. Use exponential
integration instead:

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
quantization-floor only (~0.1-0.2°/rev in fixed16). `set_quat_from_axis_angle`
is essential for this — it's **not** user-facing gravy.

Note: Phase 4 stores orientation as `mat3<T>` (matches Godot `Basis`, avoids a
per-query quat→matrix conversion). The dynamic phases integrate a `quat<T>`
(for drift-free exponential stepping) and derive the `mat3` from it each step.

---

## Known hard parts

1. **Fixed-point quaternion drift** — baseline ~0.1°/rev with exponential
   integration. Fine for most gameplay. If a game scenario spins a solid
   continuously for minutes, visible. Mitigation: clamp angular velocity,
   or if real need arises, audit where the precision is actually lost (sqrt
   in normalize, or polynomial sin/cos).

2. **Minkowski zonotope of two OBBs** — up to 30 planes in general position
   (15 axis pairs × 2 sides). Computing the plane set: 3 face normals from
   each OBB (6 planes) + 9 edge-edge cross products (18 planes, paired).
   Degenerate cases (parallel edges) collapse to fewer planes. Needs careful
   fixed-point handling — cross products of unit-ish axes can produce noise
   that looks like false planes.

3. **Contact point → angular impulse** — collision response formula gains
   lever-arm cross products: `J = -(1+e) · v_rel·n / (1/m_a + 1/m_b +
   ((I_a⁻¹·(r_a×n))×r_a + (I_b⁻¹·(r_b×n))×r_b)·n)`. With principal-axis
   diagonal inertia, `I⁻¹·v` is component-wise `(v.x/Ix, v.y/Iy, v.z/Iz)` —
   but only in the solid's local frame. Need to rotate into local, divide,
   rotate back.

4. **Broad-phase refit cost** — rotating solids change world AABB every frame.
   Phase 4 already recomputes the oriented world AABB (`rotate_aabb`) on every
   `set_orientation`, so static spinners refit each step. Probably fine for
   hop-scale scenes but measure on worst case.

5. **End-of-step penetration recovery** — new code path. SAT over OBB pairs +
   push-out along least-penetrating axis + dampen angular velocity along
   that axis. Needs to preserve determinism for fixed16 replay.

6. **Traceable impact point** — for `shape_type::traceable` collisions,
   `col.impact` currently falls back to `col.point` (simulator.h ~line 1215).
   Phases 6 (carry) and 9 (angular response) need a real contact point. The
   Phase 4 interface change already added `orientation` to `traceable::trace_*`;
   filling a real `impact` is the remaining traceable-API extension. Breaking
   change for `hop-godot::HopTrimeshTraceable` — coordinate rollout.

---

## Phase-by-phase detail

### Phase 5 — close the static narrowphase gap (oriented polytope pairs)

Finish what Phase 4 scoped out, so **every** pair honors a static orientation
(prerequisite for both static completeness and dynamic OBBs tumbling).

**Add:**
- `oriented_box`-style handling for the polytope×polytope pairs (box×box,
  box×convex, convex×convex) via the Minkowski-zonotope reduction above:
  enumerate the plane set, fill a `convex_solid<T>` in one OBB's local frame,
  transform the sweep segment in, call existing swept point-vs-convex_solid,
  map the contact back.
- Oriented-box-vs-triangle path so a **box mover** is honored against a
  rotated traceable (lift the Phase-4 capsule/sphere-only restriction in
  `hoptri::mover_world_*`).
- Broad-phase AABB already encloses oriented shapes (`rotate_aabb`, shipped in
  Phase 4) — verify it covers OBB corners for the new pairs.

**Preserves:** hop's per-frame "snapshot" model from Phase 4. Solid API
unchanged. Identity orientation stays an exact no-op.

**End-of-phase state:** full *static* rotation for every collision pair, swept.
Many users (and all of GoldSrc's non-moving angled brushes) can stop here.

**Estimated effort:** 1–1.5 weeks.

### Phase 6 — kinematic angular carry (GoldSrc `func_rotating` parity)

Make a spinning kinematic platform **carry** the bodies resting on / touching it,
the way the existing kinematic *linear* movers carry riders (the lift work). No
inertia, torque, or angular impulse — the platform's spin is scripted; physics
just transports riders.

**Add:**
- `vec3<T> angular_velocity_` on `solid` (axis·rate; pivot = `position`),
  with set/get. Always-present, zero when unset.
- In the contact solver, use the **surface velocity at the contact point** for
  the collidee: `v_surface = v_linear + ω × (contact − position)`. Today it
  reads linear velocity only; this one change makes an infinite-mass kinematic
  pusher drag the rider tangentially through the existing non-penetration /
  friction path — no new resolution code.
- **hop-godot:** in the pre-step loop (mirrors the linear `velocity = delta/dt`
  block), derive `ω` from the per-frame orientation delta
  (`ΔR = R_new·R_oldᵀ` → axis-angle → `ω = axis·angle/dt`) and call
  `set_angular_velocity`; snap orientation back post-step like position. Wire
  `_body_set_state(ANGULAR_VELOCITY)` (currently a stored no-op) for scripts
  that set `avelocity` directly.

**Note:** the player-capsule-on-rotating-brush case already has correct contacts
from Phase 4 (GJK + traceable), so this phase can ship *before* Phase 5; Phase 5
only matters for box/convex riders (crates on a platform).

**End-of-phase state:** stand on a `func_rotating`, get carried around. The
single biggest gameplay payoff in the roadmap.

**Estimated effort:** 3–5 days.

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

**End-of-phase state:** solids spin freely under torque; no collision response
yet — they rotate through each other if hit.

**Estimated effort:** 1–2 days.

### Phase 9 — angular impulse response

**Rework collision response** to use contact point + lever arm:
- `r_a = col.impact - solid_a.position` (and for b).
- Effective mass includes angular terms via principal-axis inertia.
- Linear impulse → Δv for both solids.
- Angular impulse → Δω for both solids.
- Friction at contact produces tangential impulse through the same
  machinery (this subsumes existing friction paths *and* the Phase 6 kinematic
  carry, which becomes the infinite-mass limit of this formula).

**Mitigate approach (a) artifacts:**
- Inflate broad-phase AABB for spinning solids by `|ω|·dt·max_radius`.
- End-of-step SAT penetration check for rotating pairs; push out along
  least-penetrating axis; dampen angular velocity along that axis.

**Requires:** real `col.impact` for traceable collisions (hard part #6).

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
- **hop-godot wrapper:** already adopted the Phase 4 API; extend for
  angular-velocity carry (Phase 6) and the real traceable impact point (Phase 9).
- **Migration guide:** Phase 4 already shipped with no API break for existing
  users (identity no-op); document the traceable interface change for custom
  `traceable` implementors.

**Estimated effort:** 1 day.

---

## Open decisions to confirm before starting Phase 5+

- [ ] Fixed-point drift: ~0.1-0.2°/rev with exponential integration. OK?
      (Phase 3 drift test's regression bounds are already padded for this.)
- [ ] Phase 5 box-mover-vs-traceable: full OBB-vs-triangle, or is the
      capsule/sphere rider set (Phase 4) enough for the games in practice?
      (wizardwars riders are capsules; crates would be the box case.)
- [ ] Phase 7 yaw carry: how much lives in hop vs. the game controller?
- [ ] Contact-point on traceable (hard part #6): Phases 6/9 need real
      `col.impact` for traceable collisions. Coordinate the
      `traceable::trace_*` extension with the `hop-godot` owner.

---

## Resolved decisions (kept for the record)

- ~~Traceables stay world-axis-aligned~~ — **reversed in Phase 4**; traceables
  now rotate with their solid via local-frame query transform.
- ~~`WithRotation` template parameter on `solid`/`simulator`/`collision`~~ —
  **dropped**; orientation (and later angular state) is always-present runtime
  state with an exact identity fast path. No API break, no opt-in.

---

## References in this repo

- Phase 2 branch: `rotation/phase-2-local-offset` — `shape::local_position`
  added, 6 tests in `tests/test_compound.cpp`, web demo has a compound
  dumbbell.
- Phase 3 branch: `rotation/phase-3-rotation-math` — `quat<T>`, `mat3<T>`,
  scalar traits extensions, 10 + 6 tests in `tests/test_quat.cpp` and
  `tests/test_mat3.cpp`. Includes a drift-measurement test that will be
  relevant again in Phase 8.
- Phase 4 branch: `static-rotation` — `solid.orientation` / `shape.local_rotation`,
  GJK oriented support, rotated traceable interface; tests in
  `tests/test_gjk.cpp` (`test_gjk_solid_orientation`) and
  `tests/test_collision.cpp` (`test_traceable_orientation`). hop-godot wrapper
  on its matching `static-rotation` branch.
- Toadlet port reference: `/Users/afischer/personal/toadlet/source/cpp/toadlet/egg/mathfixed/`
  — original implementations of quaternion/matrix3x3 ops and fixed-point
  polynomial asin/acos.

---

## Fixed-point drift observations from Phase 3

Forward-Euler integration at ω=2π rad/s, 100 steps/rev, 10 revs:

| | `float` | `fixed16` |
|---|---|---|
| `\|q\|` drift | 6×10⁻⁸ | 1.5×10⁻⁵ |
| Residual angle (peak) | 1.2° | 8.6° |
| Rotated-vector error | 2.1% | 15% |

Most of that drift is forward-Euler scheme error, not precision. With
exponential integration the fixed16 residual should drop to ~0.1-0.2°/rev.
Re-run `test_rotation_drift` after Phase 8 is implemented; the current
bounds are padded conservatively.

---

## Gravy in Phase 3 that's not strictly needed for physics

These exist as user-facing utilities; the rotation work itself doesn't call
them. Safe to remove if someone wants lean, but they're header-only and zero
binary cost when unused:

- `get_axis_angle_from_quat` — debug / inspection
- `slerp`, `lerp` on quat — animation, not physics
- `set_quat_from_mat3` (Shoemake) — not needed by the rotation path
- `mat3::invert`, `mat3::determinant` — rotation inverse is transpose
- `scalar_traits::asin`, `acos` — only used by the three above

Axis-angle *encoding* (`set_quat_from_axis_angle`) is **core**, since
exponential integration needs it. Everything else in the list above is
optional.
