# Pile settling — diagnosis & status

Why a deep pile of spheres in `demo_stress` misbehaves, what has been fixed,
and what remains. Compare against ReactPhysics3D (`demo_stress_rp3d`). Updated
June 2026 after a round of measurement that overturned the original plan's
premise (see "Course correction" below).

## Status at a glance

| Item | State |
|---|---|
| Deactivation velocity-threshold bug (frozen mid-air) | **fixed** (`simulator.h`) |
| Phantom velocity frozen on sleep | **fixed** (`solid.h`) |
| `is_loaded` distance-vs-speed bug | **fixed** (`simulator.h`/`constraint.h`) |
| Restitution energy runaway (KE→1e7, floor tunnelling) | **fixed** (`simulator.h`) |
| Sticky-contact warm-start on separating pairs | **fixed** (`simulator.h`) |
| Velocity cap on solver output + configurable sub-step budget | **done** (backstops) |
| Per-tick iteration flip + centered spawn (directional drift) | **done** — ~halves the drift |
| Order-independent multi-contact corner resolution | **next** — the real remaining work |

## Harnesses

```
clang++ -std=c++17 -O2 -Iinclude examples/headless_stress.cpp -o build/headless_stress
clang++ -std=c++17 -O2 -Iinclude examples/headless_micro.cpp  -o build/headless_micro
# headless_stress <ticks> <room_half> <room_height> <iters> <COR> <friction> <avg_normals> <deactivate_speed>
```
`headless_stress` reproduces the demo scene and prints per-sample KE, maxV,
min/max Z, horizontal centre-of-mass (COM, for the drift), and total mechanical
energy E=KE+PE. `headless_micro` runs isolated cases (drop / stack / box / gas /
wedge / sphere-on-sphere). Re-check any change here before touching the demo.

## Course correction — the original premise was wrong

The first draft of this plan blamed a *position-teleport energy injection* and
proposed split-impulse position correction as the fix. Direct measurement on the
current code disproved that: **disabling the start-of-frame push-out entirely
left the residual KE unchanged.** So the teleport is not the injector, and
split-impulse is not the lever. What the measurements actually showed:

- **The restitution energy runaway was the catastrophic bug** (the "hotspot" and
  "floor breaking" reports). The target was `cor · impact_speed`, where
  `impact_speed` is a latched max-over-sub-steps TOI quantity measured along the
  per-contact normal — it can exceed the live approach `|vn0|`, so contacts
  separated faster than they closed (effective COR > 1). In a deep pile the many
  contacts compounded it until KE hit ~1e7 and balls tunnelled the floor. Fixed
  by restituting off the live pre-solve approach: `target = -cor · vn0`
  (guarantees separation ≤ approach for cor ≤ 1).
- **Floor "tunnelling" was a downstream symptom**, not a CCD weakness or a moving
  floor (the floor's z was measured constant). Once the runaway drove balls past
  a few hundred m/s, a single 16 ms step crossed metres through the pile and the
  per-body sub-step loop ran out of budget. Backstops added: the velocity cap is
  now enforced on solver output too, and the sub-step budget is configurable
  (`set_max_collision_iterations`, default raised 5→16).
- **The residual ~70–110k KE is undissipated frictionless tangential sloshing**,
  not injection. Total mechanical energy *does* trend down; KE plateaus because
  the normal-only solver removes approach velocity but nothing removes the
  sliding DOF, and gravity keeps stirring. Friction is the physical dissipation
  lever — the demo runs frictionless to match rp3d, so the pile stays lively.
  The sticky-contact fix also lowered this floor (~108k→~71k) by no longer
  holding energy in spuriously-stuck contacts.

## The directional drift (pile leans into +X+Y corner)

Measured: COM drifts monotonically to ≈(+0.15, +0.15) and plateaus — a bounded
lean, not unbounded migration. It is an **order-dependence** in hop's
incremental swept update: bodies update in a fixed order and commit positions
immediately, so a body collides against earlier bodies' already-advanced
positions while they saw its stale one. Mitigations applied:

- **Per-tick iteration flip** (body update loop, contact-pair build, GS sweeps
  all alternate direction by tick parity). Cancels the order-induced component;
  drift ~0.15 → ~0.085. This only became safe after the sticky-contact fix —
  before it, flipping tipped a fixed-point case into a stuck contact and broke
  `test_sphere_capsule_collision`.
- **Centered spawn grid** in the demo/harness (it was spawning at x∈[−5.69,+5.40],
  centre −0.145, so the scene literally started biased).

The remaining ~0.085 lean is **not** removable by reordering: it lives in the
within-ball corner resolution, where a body's simultaneous contacts are collapsed
into one merged normal+depth per sub-step. The merge takes the first contact's
depth (order-dependent); making it order-independent naively (max depth along the
averaged corner normal) over-pushes and explodes. Flipping the within-ball order
also explodes. So the single-merged-contact model can't be made symmetric with a
local rule.

## Remaining work: order-independent multi-contact resolution

The one item that would remove the residual drift (and is the honest "real fix"):
resolve a body's *simultaneous* contacts together rather than one merged contact
per sub-step. Two shapes it could take:

1. **Per-contact push-out.** In `update_solid`, when a sweep starts already
   penetrating multiple colliders, push out against each contact's own
   normal/depth (a small projected-Gauss–Seidel over that body's live contacts)
   instead of collapsing them via `merge_collision`. Order-independent and
   geometrically correct in corners; the current single-merge is the lossy step.
2. **Double-buffered positions.** Read all bodies' start-of-tick positions,
   compute new positions, commit at the end — so update order can't affect
   outcomes at all. Cleaner in principle but changes the swept loop's contract
   (later bodies currently *rely* on seeing earlier bodies' updated positions),
   and is the larger change.

Either is a careful change to the core swept loop and must be validated in
fixed-point as well as float (the sticky-contact bug is a reminder that
precision interacts with contact persistence). Until then the drift is a bounded,
documented limitation.

Friction remains the only real dissipation path for the frictionless slosh; if a
genuinely-at-rest pile is wanted without it, that needs the above plus a sleeping
scheme — but per the team that should not just mask an underlying injection, and
with the runaway fixed there is no longer a net injection to mask.
