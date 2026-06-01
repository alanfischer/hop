# Pile settling — diagnosis & plan

Why a deep pile of spheres in `demo_stress` does not fully come to rest, what
was already fixed, and the plan to close the gap with ReactPhysics3D
(`demo_stress_rp3d`). Drafted June 2026.

## Status at a glance

| Item | State |
|---|---|
| Deactivation velocity-threshold bug | **fixed** (`simulator.h`) |
| Phantom velocity frozen on sleep | **fixed** (`solid.h`) |
| `demo_stress` tuned to settle (friction + shallow fill) | **done** |
| Split-impulse position correction | **next** — not started |
| Island / aggregate sleep | pending |

---

## How this was diagnosed

Two headless harnesses reproduce the scene without a window and print energy,
sleep counts, penetration, and per-phase energy deltas each tick:

```
clang++ -std=c++17 -O2 -Iinclude examples/headless_stress.cpp -o build/headless_stress
clang++ -std=c++17 -O2 -Iinclude examples/headless_micro.cpp  -o build/headless_micro
# headless_stress <ticks> <room_half> <room_height> <iters> <COR> <friction> <avg_normals> <deactivate_speed>
# headless_micro  [drop|stack|box|gas|wedge|sos] ...
```

They are the fastest way to re-check any change here: total mechanical energy
`E = KE + PE` must never trend *up* for an isolated dissipative scene.

## What is and isn't broken

Established by isolation tests (all in `headless_micro`):

- **The collision routines are correct.** A single drop rebounds to the ideal
  COR height; sphere-on-sphere, wedge, and 4-high stacks settle exactly.
- **The contact solver conserves/dissipates energy correctly** *without
  gravity*: an elastic gas (COR 1) holds KE flat for 600 ticks; an inelastic
  gas (COR 0) decays monotonically to a coasting floor. No injection there.
- **Frictionless spheres cannot settle, by construction.** The normal-only
  impulse solver removes approach velocity but nothing removes *tangential*
  sliding, so a frictionless pile sloshes indefinitely (gravity keeps trading
  PE for KE as it rearranges). This is why `demo_stress` now gives the spheres
  friction — it is the only dissipation path for the sliding DOF.
- **Residual, depth-proportional energy injection under gravity.** Even with
  friction and COR 0, a deep pile holds a KE floor that scales with depth
  (KE/ball ≈ 0.5 at 216 balls → ~2.3 at 1728 → ~10 at 6859). This is the part
  that still needs an architectural fix (below).

## Two bugs already fixed

1. **Deactivation gated on raw displacement, not speed** (`simulator.h`,
   `update_solid`). The test compared per-tick `|Δx|` against `deactivate_speed_`
   directly; since `|Δx| = v·dt`, the effective velocity threshold was
   `deactivate_speed_ / dt` — about 12 m/s at a 16 ms tick. A body moving several
   m/s but displacing little per tick (e.g. wedged against a neighbour for one
   frame) could be put to sleep mid-flight — the literal "ball hanging in the
   air." Now gated on `deactivate_speed_ · dt`. This alone flipped
   `box_box_collision`, `sphere_sphere_collision`, and `scope_filtering` from
   failing to passing in `test_collision`.

2. **Velocity not zeroed on sleep** (`solid.h`, `deactivate`). A sleeping body
   kept whatever residual velocity the swept-snap left in `velocity_`, so a
   "resting" pile carried phantom KE that was re-injected the instant a neighbour
   woke it. `deactivate()` now zeroes velocity — sleep means rest.

## Root cause of the residual injection

The solver is velocity-only; penetration is corrected by **teleporting position**
(the TOI snap and the start-of-frame push-out in `update_solid`), and there is a
one-tick lag between integrating position and solving velocity. In a stack this
compounds per layer:

1. Body A integrates downward and snaps onto support B's *start-of-tick*
   position; A's velocity is left for the solver.
2. Later in the tick B integrates downward and sinks slightly.
3. A small gap opens between A and B. Next tick A free-falls across it, gaining
   KE the solver then has to remove — but the snap/teleport that re-closes the
   gap moves A *up* (a PE gain with no velocity cost).

The net is a small energy gain per contact layer per tick, so the KE floor grows
with pile depth. Ruled out as the primary cause (measured, no improvement):
solver iteration count (16→256), push-out fraction (1.0→0.0), normal averaging,
low-speed contact damping, and bottom-up iteration order.

## Plan: split-impulse (pseudo-velocity) position correction

The standard fix, and what RP3D uses. Resolve penetration through a *separate*
velocity channel that moves position but is discarded each tick, so it never
feeds real kinetic energy.

1. **Capture penetration depth per contact.** Extend `solid<T>::touch` (and the
   `contact_pair`) with the start-of-tick separation/penetration along the
   normal. `collision<T>` already carries `depth`; thread it into
   `add_or_refresh_touch`.
2. **Add a pseudo-velocity to the solver.** In `solve_contacts`, after the normal
   and friction sweeps, run a second Gauss–Seidel sweep on a per-body
   `pseudo_vel` (stack-local, zero-initialised) whose normal target is a
   Baumgarte bias `β · max(penetration − slop, 0) / dt` (β ≈ 0.2, slop ≈ a few ×
   epsilon). Clamp the accumulated pseudo-impulse ≥ 0 like the real normal
   impulse.
3. **Integrate position from the pseudo-velocity, then drop it.** Apply
   `position += pseudo_vel · dt` once after the sweep; do **not** write it into
   `velocity_`. Remove the start-of-frame push-out teleport in `update_solid`
   (and keep the TOI snap only for the moving body's own sweep, not for resting
   correction) so penetration is owned by this pass.
4. **Validate with the harnesses.** `E` must be flat-or-down for the inelastic
   box at every depth; the 6859-ball frictionless pile should fall below the
   restitution threshold and quiesce the way it does in `demo_stress_rp3d`.

### Follow-up: island / aggregate sleep

Even with no injection, per-body sleep lets a jittering pile keep waking its
neighbours. RP3D sleeps a whole connected contact island together once every
member is below threshold for the dwell time. Building islands from the
`contact_pair` graph each tick (union-find) and sleeping/​waking per island would
let the settled heap go fully to rest. Do this after split-impulse lands, since a
non-injecting solver makes the velocity thresholds meaningful.
