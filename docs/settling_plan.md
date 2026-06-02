# Pile settling — diagnosis & status

Why a deep pile of spheres in `demo_stress` misbehaves, what has been fixed,
and what remains. Compare against ReactPhysics3D (`demo_stress_rp3d`). Updated
2026-06-02 after a round of measurement that overturned the original plan's
premise (see "Course correction" below) and after the symmetric-push-out work
that closed out the original "real remaining work" item.

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
| Symmetric (COM-preserving) frame-start push-out | **done** — drift ~0.085 → ~0.03 |
| Contact pairs canonicalized by stable id (fixed16 determinism) | **done** (`simulator.h`) |
| Static-friction coefficient honored; velocity cap on solver output | **done** (`simulator.h`) |
| `restitution_combine` precedence contract documented | **done** (`solid.h`) |
| Residual slow COM creep (~0.03–0.05 over 1500 ticks) | **bounded, documented** — see drift section |

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
- **The residual KE is undissipated frictionless tangential sloshing**, not
  injection. Total mechanical energy *does* settle (E≈277–279k, roughly flat
  across 1500 ticks); KE plateaus because the normal-only solver removes approach
  velocity but nothing removes the sliding DOF, and gravity keeps stirring.
  Friction is the physical dissipation lever — the demo runs frictionless to
  match rp3d, so the pile stays lively. Successive fixes have walked this floor
  down: the sticky-contact fix (~108k→~71k) by no longer holding energy in
  spuriously-stuck contacts, and the symmetric push-out plus solver-output
  velocity cap further to ~30–35k in the current default scene.

## The directional drift (pile leans into +X+Y corner)

Originally measured: COM drifted monotonically to ≈(+0.15, +0.15) and plateaued
— a bounded lean, not unbounded migration. It is an **order-dependence** in
hop's incremental swept update: bodies update in a fixed order and commit
positions immediately, so a body collides against earlier bodies'
already-advanced positions while they saw its stale one. Three fixes, in order
of impact:

- **Symmetric frame-start push-out** (`update_solid`, commit `1f27130`). The
  largest source. When a sweep starts already penetrating a dynamic partner, the
  overlap is now resolved by moving *both* bodies ±depth/2 in one shot, rather
  than each body self-pushing half during its own update. The old self-only
  split was order-dependent — the body updated second saw an already-reduced
  overlap and pushed less, so each pair's center of mass crept toward whichever
  body updated first, and over a fixed-order grid of contacts that summed into
  the lean. The ±depth/2 correction is center-of-mass preserving and
  order-independent, removing the drift at its source. (See also the
  `hop_pushout_symmetry` note: the asymmetric self-only version also injected
  energy.)
- **Contact pairs canonicalized by stable id, not pointer** (`solve_contacts`,
  commit `0af69be`). The a<b side of each pair was chosen by raw pointer compare;
  ASLR varied heap addresses run-to-run, so the order impulses applied within a
  pair changed between runs. Float absorbed it but fixed16 amplified the
  different operation order into divergent trajectories. Now keyed on stable id,
  so identical builds reproduce identical energy.
- **Per-tick iteration flip** (body update loop, contact-pair build, GS sweeps
  all alternate direction by tick parity) and a **centered spawn grid**. The flip
  cancels the residual order-induced component; it only became safe after the
  sticky-contact fix (before it, flipping tipped a fixed-point case into a stuck
  contact and broke `test_sphere_capsule_collision`). The centered grid fixes a
  scene that literally started biased (was spawning at x∈[−5.69,+5.40], centre
  −0.145).

Current measurement (`headless_stress 1500`, defaults): COM reaches only
≈(+0.03, +0.05) by tick 1500 and is still creeping very slowly rather than fully
plateaued — down from the ~0.085 that the per-tick flip alone left, and an order
of magnitude below the original ~0.15. The symmetric push-out, not reordering,
is what closed most of the gap; the old claim that the residual lived
irreducibly in a within-ball merged-contact resolution no longer holds.

## Remaining work

The original "real remaining work" — making the frame-start overlap correction
order-independent — is **done**: the symmetric ±depth/2 push-out is the
per-contact-symmetric resolution that the earlier plan proposed (option 1), and
it took the drift from ~0.085 to ~0.03 without exploding. What is left is
smaller and largely optional:

1. **Residual slow creep.** COM still inches toward the +X+Y corner (~0.03→0.05
   over 1200→1500 ticks) rather than fully plateauing. This is the *remaining*
   order-dependence: the symmetric push-out fixes overlap correction, but later
   bodies still sweep against earlier bodies' already-committed positions within
   a tick. The clean (and only fully order-free) fix is **double-buffered
   positions** — read all bodies' start-of-tick positions, compute new
   positions, commit at the end. It changes the swept loop's contract (later
   bodies currently *rely* on seeing earlier bodies' updated positions) and is
   the larger change; deferred while the creep stays this small and bounded.
2. **Frictionless slosh / genuine rest.** KE plateaus at ~30–35k because nothing
   removes the sliding DOF. Friction is the physical dissipation lever; the demo
   runs frictionless to match rp3d. A genuinely-at-rest pile without friction
   would need a sleeping scheme on top — but per the team that should not just
   mask an injection, and with the runaway fixed there is no longer a net
   injection to mask.

Any change here is a careful change to the core swept loop and must be validated
in fixed-point as well as float (the sticky-contact and pointer-canonicalization
bugs are both reminders that precision and address-stability interact with
contact persistence). Until then both items are bounded, documented limitations.
