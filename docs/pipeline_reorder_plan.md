# Contact-solver pipeline — diagnosis & reorder plan

Why a frictionless pile in `demo_stress` never fully rests, what is actually
wrong (it is deeper than earlier thought), and what a real fix requires. This
supersedes the former `settling_plan.md`, whose central claim ("the residual is
undissipated slosh, *not* injection; total energy settles flat") was overturned
by later measurement — there **is** a net energy injection, and its root cause
is the update order.

## TL;DR

- A frictionless pile never sleeps because the contact solver **injects energy**.
- Root cause: hop's **update order**. `update_solid` integrates *position* (with
  gravity) and then `solve_contacts` solves *velocity*. So every tick gravity
  drives bodies into their supports (position is integrated with un-solved
  velocity), and the per-tick position correction lifts them back out — doing
  work against gravity that the frictionless velocity solver cannot reclaim,
  leaking the tangential part at angled contacts. Standard sequential-impulse
  engines avoid this by solving velocity **before** integrating position.
- The genuine fix is to **reorder the pipeline**, which is a core rewrite gated
  on a new **speculative (signed-distance) narrow-phase query** that hop lacks.
- Practical mitigation today: **friction / contact damping** drains the leaked
  tangential energy (this is why the demo "settles-ish" with friction).

## Root-cause diagnosis

### Energy injection — why the frictionless pile never rests
Measured (`headless_stress`, frictionless, no damping): total mechanical energy
E **grows**, worse as restitution falls — COR=0 → E 1.65M→1.88M; COR=0.5 →
632k→884k; COR=0.75 ≈ bounded. Key cuts:
- **Not restitution.** COR=0 (nothing bounces) injects the *most*. The earlier
  restitution-runaway fix (`target = -cor·vn0`) is intact.
- **The per-tick position correction is the proximate injector.** Disabling the
  frame-start push-out entirely stops the growth (E goes flat/declining; bodies
  just sink, losing PE). So lifting penetrating bodies out against gravity each
  tick is what adds energy; the frictionless solver removes only the *normal*
  component of what gravity reclaims, leaking the tangential part at angled
  contacts. Lower COR ⇒ deeper, more persistent overlaps ⇒ bigger correction ⇒
  more injection.
- **The deeper cause is the update order, not the correction *mechanism*.**
  Split-impulse (correct via a pseudo-velocity instead of a teleport) does *not*
  fix it: it is still a per-tick position correction against gravity, with the
  same δ and the same leak — and it destabilises piles besides. The real issue
  is that position is integrated with pre-contact-solve velocity, so bodies
  penetrate under gravity in the first place.

### Directional drift — pile leans to a corner
- **Frictionless: FIXED** (landed on `main`). Root was `average_normals`
  blending several colliders' normals into one biased `c.normal`; the fix uses
  the true per-pair normal for both the push-out and the solver cache. Drift ≈ 0
  over 10k ticks.
- **With friction: drift RETURNS** (a corner ratchet). Source is the
  **movement-pass body-update order** (immediate commit), *not* the solver:
  forcing forward-only body order drifts to −X−Y, backward-only to +X+Y — the
  drift follows the order. Friction *rectifies* that small per-tick order bias
  into one-way creep (frictionless it averages out). Symmetric Gauss–Seidel in
  the solver had no effect, confirming the bias is upstream in the movement pass.
  Removing it needs order-independent movement (double-buffering) — a dead end on
  its own (below).

## Approaches tried and ruled out
- **Double-buffered positions** (read start-of-tick, commit after the pass):
  does not reduce the drift and destabilises the dense pile over long runs; a
  naive version (stale contacts fed to the velocity solver) diverges (KE→6e7).
  The swept-snap response is tightly coupled to immediate commit.
- **Symmetric Gauss–Seidel** (cancel solver sweep-order bias within each tick):
  no effect on the friction drift — the bias lives in the movement pass.
- **Split-impulse position correction** (pseudo-velocity bias pass): explodes
  angled-contact piles (NGS over-correction in frustrated dense contacts) and,
  per the diagnosis, cannot fix the injection (same per-tick correction-against-
  gravity). Simple/vertical cases work and sleep, but those have no tangential
  leak, so they don't exercise the bug.
- **Contact damping (frictionless, viscous tangential)**: a clean, drift-free
  energy sink, but only reaches a non-zero equilibrium (E ≈ 317k) because the
  injection persists — it cannot reach sleep on its own.

## The fix: reorder the update pipeline

Target order (standard sequential impulse):
```
1. integrate velocity      v += a·dt            (gravity/drag/forces → velocity)
2. detect contacts         narrow-phase at start-of-tick positions → touch cache
3. solve velocity          existing solve_contacts (warm-started)
4. integrate position      x += v·dt            (with the SOLVED velocity)
5. position correction     split-impulse for residual penetration (now stable)
6. CCD                     for fast bodies (see tension below)
```
Solving velocity before integrating position (step 3 before 4) is the whole
point: gravity's approach velocity is removed first, so bodies never penetrate
under gravity, so step 5 corrects only tiny residuals — no lift-against-gravity,
no injection. It also makes split-impulse stable (it was exploding precisely
because it fought full gravity-driven penetration every tick).

### Gating prerequisite — speculative narrow-phase (the long pole)
Step 2 must detect contacts **before** moving — i.e. **speculative contacts**:
"within margin, about to touch," not just "already overlapping." hop's narrow
phase (`collide.h`) is swept-only: a zero-length probe reports a normal/depth
only when the configuration-space origin is already *inside* (penetrating); for
a near-but-separated pair it just returns "miss." So the reorder first needs a
**closest-feature / signed-distance query** returning a normal + separation for
non-overlapping pairs within a margin — all shape types, fixed-point-safe. This
is a real addition to `support.h`/`collide.h` and is the go/no-go for the whole
effort. (The same gap blocked the double-buffering and discovery-pass attempts.)

### Architectural tension — hop *is* a swept-CCD engine
The target order integrates position discretely (step 4), but hop's identity is
continuous swept collision (how fast bodies avoid tunnelling). Reconcile via:
- **Speculative margin** (preferred): detect within `|v_rel|·dt + margin`; the
  velocity solver's non-penetration constraint then stops a body at the contact
  this tick. Needs the distance query above.
- **Separate CCD pass** (step 6): keep a swept test only for bodies whose
  `|v|·dt` exceeds their size; snap TOI after position integration.

### Phases
- **Phase 0 — speculative narrow-phase query (prerequisite, sizeable).** Build
  and prove the distance/closest-feature query in isolation, in float and
  fixed16. If intractable, the reorder is too.
- **Phase 1 — split `update_solid`** into velocity-integrate vs position-
  integrate; restructure `update()` to the 6-step order; detection becomes a
  dedicated pass (replacing the per-body sweep's cache population).
- **Phase 2 — position integration with solved velocity + split-impulse**
  correction with Box2D-style clamping (`maxLinearCorrection`) and under-
  relaxation (the missing pieces that made the bias pass explode).
- **Phase 3 — CCD reconciliation** (speculative margin and/or fast-body swept
  pass) so tunnelling does not regress.
- **Phase 4 — re-validate everything:** frictionless injection gone (E bounded,
  pile sleeps), all 11 tests incl. fixed16, the per-pair-normal drift fix still
  holds, `headless_micro` scenarios, the demo, and **performance** (the swept
  narrow-phase is already the hotspot at ~65% of frame time; a second detection
  pass / distance queries will likely cost ~2×).

### Risks
Large, multi-session, rewrites the core loop and adds a narrow-phase capability;
every contact-driven behaviour (characters, projectiles, triggers, stacking)
must be re-validated in float and fixed16; likely perf regression on the
existing hotspot; CCD behaviour change (hop's headline feature); determinism
must hold through new detection and correction passes. Treat as a deliberate
roadmap item ("hop 2.0 solver"), with **Phase 0 as the go/no-go**.

## Practical mitigation today
Friction or a small viscous contact damping drains the leaked tangential energy.
Tradeoffs: friction reintroduces the corner ratchet (movement-pass order);
frictionless damping reaches a non-zero equilibrium (cannot fully sleep) because
the injection persists.

## History — landed fixes (still in the code)
These are accurate and explain why the contact code looks the way it does:

| Fix | State |
|---|---|
| Deactivation velocity-threshold bug (frozen mid-air) | fixed (`simulator.h`) |
| Phantom velocity frozen on sleep | fixed (`solid.h`) |
| `is_loaded` distance-vs-speed bug | fixed (`simulator.h`/`constraint.h`) |
| Restitution energy runaway (KE→1e7, floor tunnelling) → `target = -cor·vn0` | fixed (`simulator.h`) |
| Sticky-contact warm-start on separating pairs | fixed (`simulator.h`) |
| Velocity cap on solver output + configurable sub-step budget | done |
| Per-tick iteration flip + centered spawn (directional drift) | done |
| Symmetric (COM-preserving) frame-start push-out | done |
| Contact pairs canonicalized by stable id (fixed16 determinism) | done |
| `restitution_combine` precedence contract documented | done (`solid.h`) |
| Settling COM drift (frictionless) → true per-pair contact normal | fixed (`simulator.h`) |
| …its re-test optimised away by capturing the unblended normal in `merge_collision` | done (`collide.h`/`simulator.h`) |

## Harnesses
```
clang++ -std=c++17 -O2 -Iinclude examples/headless_stress.cpp -o build/headless_stress
clang++ -std=c++17 -O2 -Iinclude examples/headless_micro.cpp  -o build/headless_micro
# headless_stress <ticks> <room_half> <room_height> <iters> <COR> <friction> <avg_normals> <deactivate_speed>
```
`headless_stress` mirrors the demo scene and prints per-sample KE, maxV, min/max
Z, horizontal COM (the drift), and total mechanical energy E=KE+PE (E must not
trend up). `headless_micro` runs isolated cases (drop / stack / box / gas /
wedge / sphere-on-sphere). Validate any core-loop change in both float and
fixed16.
