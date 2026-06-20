# Rotation support — plan

Roadmap for adding rotation to hop. Current as of June 2026: **static
orientation** (Phase 4) and the **real traceable contact point** (the Phase 6/9
prerequisite) have shipped.

The roadmap builds up to full dynamic rotation in graded milestones, each
shippable on its own:

  static orientation (done) → finish the static narrowphase → **kinematic
  angular carry / blocking** (GoldSrc `func_rotating` parity) → dynamic spin
  under torque → angular impulse response → constraints/friction.

The kinematic-carry milestones (6–7) are deliberately ahead of the dynamic ones:
they deliver the GoldSrc rotating-platform feel with no inertia/torque math, and
the dynamic phases reuse the same `angular_velocity` state and contact-point
plumbing.

## Status at a glance

| Phase | State | Notes |
|---|---|---|
| 1. Contact points on `collision<T>` | done | `c.impact` is the real witness point on **every** pair, incl. traceables (hop #49 / hop-godot #5) |
| 2. Compound colliders via `shape::local_position` | done | |
| 3. Rotation math primitives (`quat`, `mat3`, `asin`/`acos`) | done | |
| 4. **Static orientation** — `solid.orientation` + `shape.local_rotation`, honored by GJK + traceables | done | |
| 5. Close the static narrowphase gap (rotated polytope×polytope + box-mover-vs-traceable, OBB) | **next** | not started |
| 6. Kinematic angular carry (`ω×r` surface velocity; spinning platforms carry riders) | pending — **unblocked** | contact point ready |
| 7. Kinematic blocking/crush + rider yaw carry (rotating-door parity) | pending | |
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

What is **not** yet honored (→ Phase 5): **polytope×polytope** pairs (box×box,
box×convex, convex×convex) collide as if unrotated (they route through
`trace_aa_box` / Minkowski-AABB, not GJK), and a **box mover vs a traceable** is
skipped (no rotation-invariant spine). A rotated box/convex only collides
correctly today when it meets a *rounded* shape via the GJK path.

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
  `hoptri::mover_world_*`). When added, fill `col.impact` for it too — the
  capsule/sphere/box support-plane paths already do.
- Broad-phase AABB already encloses oriented shapes (`rotate_aabb`, shipped in
  Phase 4) — verify it covers OBB corners for the new pairs.

**Preserves:** hop's per-frame "snapshot" model from Phase 4. Solid API
unchanged. Identity orientation stays an exact no-op.

**End-of-phase state:** full *static* rotation for every collision pair, swept.
Many users (and all of GoldSrc's non-moving angled brushes) can stop here.

**Estimated effort:** 1–1.5 weeks.

### Phase 6 — kinematic angular carry (GoldSrc `func_rotating` parity)

Make a spinning kinematic platform **carry** the bodies resting on / touching it,
the way the existing kinematic *linear* movers carry riders. No inertia, torque,
or angular impulse — the platform's spin is scripted; physics just transports
riders.

**Add:**
- `vec3<T> angular_velocity_` on `solid` (axis·rate; pivot = `position`),
  with set/get. Always-present, zero when unset.
- In the contact solver, use the **surface velocity at the contact point** for
  the collidee: `v_surface = v_linear + ω × (col.impact − position)`. Today it
  reads linear velocity only; this one change makes an infinite-mass kinematic
  pusher drag the rider tangentially through the existing non-penetration /
  friction path — no new resolution code. `col.impact` is now the real surface
  point on every pair, so the lever arm is correct for trimesh brushes too.
- **hop-godot:** in the pre-step loop (mirrors the linear `velocity = delta/dt`
  block), derive `ω` from the per-frame orientation delta
  (`ΔR = R_new·R_oldᵀ` → axis-angle → `ω = axis·angle/dt`) and call
  `set_angular_velocity`; snap orientation back post-step like position. Wire
  `_body_set_state(ANGULAR_VELOCITY)` (currently a stored no-op) for scripts
  that set `avelocity` directly.

**Note:** unblocked — the player-capsule-on-rotating-brush case has correct
contacts from Phase 4 and a real contact point from the Phase 1 follow-up, so
this can ship *before* Phase 5; Phase 5 only matters for box/convex riders
(crates on a platform).

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

## Open decisions to confirm before starting Phase 5+

- [ ] Fixed-point drift: ~0.1–0.2°/rev with exponential integration. OK?
      (`test_rotation_drift` bounds are already padded for this.)
- [ ] Phase 5 box-mover-vs-traceable: full OBB-vs-triangle, or is the
      capsule/sphere rider set (Phase 4) enough for the games in practice?
      (wizardwars riders are capsules; crates would be the box case.)
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
- Toadlet port reference:
  `/Users/afischer/personal/toadlet/source/cpp/toadlet/egg/mathfixed/` —
  original quaternion/matrix3x3 ops and fixed-point polynomial asin/acos.
