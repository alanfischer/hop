# Rotation support — plan

Roadmap for adding dynamic rotation to hop, drafted April 2026.
Companion to the Phase 2 (`rotation/phase-2-local-offset`) and
Phase 3 (`rotation/phase-3-rotation-math`) branches.

## Status at a glance

| Phase | State | Branch / notes |
|---|---|---|
| 1. Contact points on `collision<T>` | done — shipped before plan started | `c.impact` already exists |
| 2. Compound colliders via `shape::local_position` | done | `rotation/phase-2-local-offset` |
| 3. Rotation math primitives (`quat`, `mat3`, `asin`/`acos`) | done | `rotation/phase-3-rotation-math` |
| 4. Oriented shapes + Minkowski swept collision | **next** | not started |
| 5. `WithRotation` template param + dynamic state | pending | not started |
| 6. Angular impulse response | pending | not started |
| 7. Constraints and friction use angular | pending | not started |
| 8. Docs, examples, bindings | pending | not started |

---

## Key decisions (already made, don't re-litigate unless someone has new info)

### Swept-OBB strategy: approach (a) + Minkowski reduction

For static-orientation sub-steps, OBB-vs-OBB and OBB-vs-convex-solid collision
reduces to a **point-vs-convex_solid sweep** where the convex_solid is the
Minkowski zonotope of the two OBBs (up to 30 planes in general position).
Hop already has swept point-vs-convex_solid, so most of Phase 4 is
plumbing and Minkowski construction.

Approach (a) snapshots orientation at the start of each sub-step. It gives
up hop's "zero overlap, ever" guarantee in exchange for tractability.
Mitigations in Phase 6:
- Angular velocity cap (like the existing linear cap).
- Broad-phase AABB inflation for spinning solids: `+= |ω|·dt·r`.
- End-of-step SAT-based penetration recovery.

### Inertia model: principal-axis diagonal only

Store inertia as a `vec3<T>` (Ix, Iy, Iz), not a full 3×3 tensor. Simpler
world-space transform and avoids fixed-point matrix inverse. Works correctly
for sphere/capsule/box (natural principal axes). If someone later needs a
non-aligned inertia tensor, that's a separate feature request.

### Traceables stay world-axis-aligned

User-implemented `traceable<T>` (trimeshes in `hop-godot`) do **not** rotate
with their owning solid. Too breaking an API change for too little gain.
All rotation happens on convex shapes.

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

### `WithRotation` template parameter arrives in Phase 5

`template <typename T, bool WithRotation = false>` added to `solid`,
`simulator`, `collision`. EBO-backed rotation state only exists when `true`.
`if constexpr` gates all rotation paths. Default preserves existing API and
has zero overhead. This is the one deliberate breaking change in the plan.

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
   Currently aa_box only changes on position change. With rotation, every
   rotating solid refits BVH every step. Probably fine for hop-scale scenes
   but measure on worst case.

5. **End-of-step penetration recovery** — new code path. SAT over OBB pairs +
   push-out along least-penetrating axis + dampen angular velocity along
   that axis. Needs to preserve determinism for fixed16 replay.

6. **Traceable impact point** — for `shape_type::traceable` collisions,
   `col.impact` currently falls back to `col.point` (simulator.h ~line 1215).
   In Phase 6, angular response needs a real contact point. Requires extending
   the `traceable::trace_solid` API so user implementations fill it in.
   Breaking change for `hop-godot::HopTrimeshTraceable` — coordinate rollout.

---

## Phase-by-phase detail

### Phase 4 — oriented shapes + Minkowski swept collision

**Add:**
- `quat<T> local_orientation` on `shape<T>` (default identity).
- `shape_type::oriented_box`, new `oriented_box<T>` in `math/`.
- Sphere-vs-OBB, capsule-vs-OBB swept — transform sweep segment into OBB's
  local frame, use existing AABB sweep, transform result back.
- OBB-vs-OBB / OBB-vs-convex / convex-vs-convex swept via Minkowski zonotope:
  1. Enumerate plane set (up to 30 planes: 6 from each OBB face, up to 18
     from edge-edge cross products).
  2. Fill a `convex_solid<T>` with these planes, in one OBB's local frame.
  3. Transform sweep segment into that frame.
  4. Call existing `find_intersection(segment, convex_solid, ...)`.
  5. Map contact point back to features on A and B.
- Extend broad-phase AABB computation to enclose oriented shapes (corners
  of rotated OBB, endpoints of rotated capsule).

**Preserves:** hop's "zero overlap ever" guarantee. Solid API unchanged.

**End-of-phase state:** full static rotation works for every collision
pair, with swept CCD. Many users can stop here.

**Estimated effort:** 1–1.5 weeks.

### Phase 5 — WithRotation template + dynamic orientation state

**Add the template param:**
```cpp
template <typename T, bool WithRotation = false>
class solid : private rotation_state<T, WithRotation> { ... };
```
EBO-backed `rotation_state<T, false>` is empty; `<T, true>` carries
`quat<T> orientation`, `vec3<T> angular_velocity`, `vec3<T> torque`,
`vec3<T> inertia`, `vec3<T> inv_inertia`.

**Dynamic rotation integration** (in each `integrator_type`):
- Exponential quat step: `set_quat_from_axis_angle(dq, ω_hat, |ω|*dt)`,
  `q ← dq * q`.
- Angular velocity step: `ω ← ω + I⁻¹·(τ - ω × (I·ω)) · dt` (gyroscopic
  term kept because it's cheap and stabilizing).
- Angular velocity cap (new `default_max_angular_velocity_component` on
  `scalar_traits`).
- Angular component of deactivation threshold.

**End-of-phase state:** solids can spin freely under torque; no collision
response yet — they just rotate through each other if hit.

**Estimated effort:** 1–2 days.

### Phase 6 — angular impulse response (WithRotation=true)

**Rework collision response** to use contact point + lever arm:
- `r_a = col.impact - solid_a.position` (and for b).
- Effective mass includes angular terms via principal-axis inertia.
- Linear impulse → Δv for both solids.
- Angular impulse → Δω for both solids.
- Friction at contact produces tangential impulse through the same
  machinery (this subsumes existing friction paths).

**Extend `collision<T>`** to carry whatever additional fields Phase 6 needs
(probably none — `impact` and `normal` are sufficient).

**Mitigate approach (a) artifacts:**
- Inflate broad-phase AABB for spinning solids by `|ω|·dt·max_radius`.
- End-of-step SAT penetration check for rotating pairs; push out along
  least-penetrating axis; dampen angular velocity along that axis.

**Estimated effort:** 3–5 days.

### Phase 7 — constraints and friction use angular (WithRotation=true)

**Local attach points on `constraint<T>`:** anchor in solid's local frame,
rotates with the solid. Spring force at offset produces torque via lever
arm. Naturally subsumes existing spring behavior when attach point is at
origin.

**Nothing new for friction** — Phase 6 already did the tangential-impulse
work. This phase is mostly documenting that rolling now works.

**Estimated effort:** 2–3 days.

### Phase 8 — docs, examples, bindings

- **README:** retire "translation-only" framing. Document that
  `WithRotation=false` is still zero-cost opt-out.
- **Examples:** add a rotation demo (cube tumbling on ramp).
- **Web bindings:** expose `setOrientation`/`getOrientation`,
  `setAngularVelocity`/`getAngularVelocity`, `setInertia` on `HopSolid`
  when we want to expose the rotating variant. Or — add
  `HopRotatingSimulator` / `HopRotatingSolid` as a parallel class surface.
- **hop-godot wrapper:** update to use new API. Coordinate traceable
  breaking change (Phase 6).
- **Migration guide:** for existing users, nothing changes unless they
  opt into `WithRotation=true`.

**Estimated effort:** 1 day.

---

## Open decisions to confirm before starting Phase 4

- [ ] Fixed-point drift: ~0.1-0.2°/rev with exponential integration. OK?
      (Phase 3 drift test's regression bounds are already padded for this.)
- [ ] API break timing: template param on `solid`/`simulator`/`collision`
      lands in Phase 5. OK to break in a 2.0 major version bump?
- [ ] Traceable rotation: confirm we're freezing them as world-aligned,
      meaning `hop-godot` doesn't need trimesh rotation support.
- [ ] Contact-point on traceable: Phase 6 needs real `col.impact` for
      traceable collisions. Coordinate with `hop-godot` owner on the
      `traceable::trace_solid` API extension.

---

## References in this repo

- Phase 2 branch: `rotation/phase-2-local-offset` — `shape::local_position`
  added, 6 tests in `tests/test_compound.cpp`, web demo has a compound
  dumbbell.
- Phase 3 branch: `rotation/phase-3-rotation-math` — `quat<T>`, `mat3<T>`,
  scalar traits extensions, 10 + 6 tests in `tests/test_quat.cpp` and
  `tests/test_mat3.cpp`. Includes a drift-measurement test that will be
  relevant again in Phase 5.
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
Re-run `test_rotation_drift` after Phase 5 is implemented; the current
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
