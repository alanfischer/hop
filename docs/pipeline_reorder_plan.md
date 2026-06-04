# Pipeline Reorder: Speculative Contacts

A plan to move hop from its current **integrate → snap → solve** tick order to a
**discover → solve → integrate** order built on *speculative contacts*, so that
resting piles dissipate to true rest and sleep at the default threshold instead
of churning forever.

Background and the why-it-matters framing live in the README section
"Contact Solver, Settling & Sleep". This document is the implementation plan.

## The core idea in one paragraph

Hop already runs the expensive half of speculative contacts every tick: the
swept query that finds *where a body would go and what it would hit on the way*.
Today that query is used to **snap** a body to its earliest time-of-impact and
then patch up velocity afterward (`solve_contacts`, "Pass B"), which leaks a
little energy every tick (see README). The reorder reuses the same query to
instead **discover** contacts ahead of the move — recording, per contact, the
remaining gap distance — then solves velocities with a constraint that only
removes enough approach speed to close that gap, and *then* integrates. Because
velocity is made non-penetrating before the move, there is no snap and no
push-out, so there is no leak. The position update becomes a plain
`pos += v·dt`.

## Current pipeline (what we're changing)

Per `simulator<T>::update` (`include/hop/simulator.h:174`):

1. For each active solid, `update_solid` (`:423`):
   - **Integrate** position and velocity with the chosen integrator (`:445–501`),
     committing the new velocity immediately (`:504`).
   - Run the **collision loop** (`:550–697`): sweep the integrated path
     (`trace_solid_with_current_spacials`, `:560`), and for the earliest hit:
     - if penetrating at frame start, **push both bodies out** by the overlap
       depth (`:570–595`);
     - otherwise **snap** the committed position to the TOI point (`:596–599`);
     - record the contact in the persistent per-body **touch cache**
       (`add_or_refresh_touch`, `:643`);
     - project the leftover motion onto the contact tangent and sub-step
       (`slide_vel`, `:653–692`).
   - Velocity is deliberately **not** mutated here (`:538`).
2. After every body is integrated, **`solve_contacts`** (`:1043`) rebuilds the
   canonical pair list from the touch caches (`:1048–1143`), snapshots `vn0`
   and sets each pair's restitution target `target = -cor·vn0` (`:1161`),
   warm-starts from cached impulses (`:1165–1186`), and runs warm-started
   Gauss–Seidel normal+friction sweeps (`:1188–1262`), writing accumulated
   impulses back to the cache.

The leak is structural: the body is integrated into penetration under gravity,
the position is corrected by hand (snap / push-out), and only *then* is velocity
reconciled — so each tick injects a sliver of energy, worst at angled contacts
and low restitution. In a deep pile it never decays.

## Target pipeline

Per tick:

1. **Integrate velocity only** (forces, gravity, damping) → a *tentative*
   velocity per body. Compute a *predicted* displacement `Δ = v·dt` but **do not
   commit position**.
2. **Discovery sweep.** Sweep each body's current position along `Δ` (plus a
   small **speculative margin** so contacts about to happen are caught) and, for
   every contact found, record into the touch cache: the contact normal and the
   **separation gap** — the signed distance still to be closed along the normal
   (negative if already penetrating). No snap, no push-out, no position change.
3. **Solve velocities** (`solve_contacts`) with a *speculative* normal target:
   the post-solve approach speed may not exceed `gap/dt`. Resting contacts
   (`gap≈0`) target zero approach (or the restitution bounce); contacts still a
   little apart are allowed to keep closing just until they touch. Friction and
   warm-starting are unchanged.
4. **Commit positions.** `pos += v·dt` with the solved velocity. Because no
   contact can be closed by more than its gap, bodies land on surfaces instead
   of inside them.
5. **Penetration backstop** (small): a single cheap positional correction for
   any residual overlap (numerical slop, or a contact discovered along the old
   direction that the solved tangential motion slid into). This replaces the
   dominant push-out of today with a small safety net (a slop band, corrected
   fractionally — Baumgarte-style), not the primary mechanism.

The corner-sliding behavior that the current sub-step loop produces
(`slide_vel`, `:653–662`) falls out of step 3 for free: multiple
non-penetration constraints on one body, solved together, already yield motion
along their intersection. Continuous-collision (anti-tunneling) is preserved
because the discovery sweep still spans the *full* predicted motion — a fast
body's gap to a thin wall is found within `Δ`, and the velocity clamp stops it
from crossing.

## Data-structure changes

- `solid<T>::touch` (`include/hop/solid.h:42`): add `T separation {};` — the gap
  recorded at discovery. `accum_n` / `accum_t` / `normal` / `last_tick` /
  `pair_built_tick` are unchanged and keep warm-starting working as-is.
- `contact_pair` (`simulator.h:384`): add `T separation {};`, carried from the
  slot at build time alongside the existing fields.
- Likely retire `impact_speed`'s role in the restitution path (already not the
  target since `:1161`); keep it only for wake/callback logic.

## Implementation phases

**Phase 0 — Discovery without behavior change.** Refactor the body of the
collision loop (`:550–697`) into a `discover_contacts(solid, path)` that fills
the touch cache with normal **and separation**, *without* mutating position.
Keep the existing snap/push-out as the default; have discovery run alongside it
under an internal flag so the new cache field can be validated against the old
contacts before anything switches. No observable change yet.

**Phase 1 — Reorder `update`.** Split the per-body work in `update`
(`:191–211`) into two passes over the iteration list: pass 1 integrates velocity
and stores predicted `Δ`; pass 2 runs discovery. Then `solve_contacts`, then a
commit pass (`pos += v·dt`). Gate the whole new order behind a
`set_speculative_contacts(bool)` toggle (default **off**) so both pipelines ship
side by side during bring-up and A/B testing.

**Phase 2 — Speculative target.** In `solve_contacts` (`:1148–1163`), replace
`target = -cor·vn0` with the speculative form: a non-penetration velocity bound
`vn ≥ -gap/dt` combined with the restitution bounce, i.e.
`target = max(-cor·vn0_if_closing, -gap/dt)` (signs per the existing pair-normal
convention). The clamped-accumulator GS (`:1220–1240`) is otherwise unchanged.
Restitution stays dissipative for the same reason it is today (`|target| ≤
|vn0|`).

**Phase 3 — Commit + backstop, and the discovery-completeness blocker.**
Implement the `pos += v·dt` commit (done) and a position-only penetration
backstop (a COM-split, order-independent NGS pass over the contact graph;
prototyped). The backstop turned out to be **necessary but not sufficient** —
see the findings below. The real blocker surfaced here is **contact-discovery
completeness**, which must be solved before the backstop helps.

**Phase 4 — Re-tune and prune.** With the pile now dissipating naturally,
re-tune `deactivate_speed` / `deactivate_count` (the sweep harness,
`examples/headless_stress.cpp`, already measures this). Re-evaluate whether
`set_contact_damping` (`demo_stress`'s current crutch) is still needed — it
likely becomes a no-op for settling and can default off. Confirm
`solver_iterations` budgets.

**Phase 5 — Flip the default and remove the old path.** Once the test matrix
below is green with `speculative_contacts` on, make it the default, then delete
the legacy snap-driven settling path and the toggle.

## Implementation status & findings (2026-06-04)

Phases 0–2 are **implemented and committed** behind `set_speculative_contacts`
(default off): `integrate_and_discover` → `solve_contacts` (speculative target)
→ `commit_solid`, with a shared `try_deactivate`. The headline result is
confirmed — the structural **energy injection is gone** (demo_stress KE
55223 → ~1400, ~40×), single contacts are exact (a drop lands at rest), the
box stack rests at KE 0, and a 300 m/s sphere does **not** tunnel a thin wall
(CCD preserved).

**The open blocker — directional discovery under-counts resting contacts.**
The swept query is *directional*: it finds what a body moves *toward*. That is
perfect for the cases above (a falling body sweeps down onto its support; a fast
body sweeps into a wall), which is why the vertical box stack, the drop, and the
tunneling test all pass. But a dense 3D pile's contacts arrive from **all
directions** at gap ≈ 0, and a near-zero predicted Δ sweep does not enumerate the
lateral neighbours a body is merely resting against. With an incomplete contact
set, neither the velocity solve nor a position backstop can prevent
interpenetration, and demo_stress breaches the floor on most ticks
(`breachTicks ≈ 760/1200`).

A position-only backstop (Phase 3) was prototyped and **made the dense pile
worse** (meanZ collapsed 3.4 → 1.5, bodies sleeping interpenetrated) precisely
because it shuffles positions using the same incomplete contact set — it was
reverted. Conclusion: **omnidirectional contact discovery is the prerequisite**,
and the next real step, not the position pass.

**Margin-shell discovery — DONE (Option A, commit 2f84e3b).** A `margin`
parameter is threaded through the narrow phase (`hop::test_solid` + the simulator
wrapper, default 0 = exact shapes). It Minkowski-inflates every shape pair
uniformly — AABB Minkowski grows, sphere/capsule combined radii gain the margin,
convex planes shift out — so a near-resting contact registers as an overlap and
discovery recovers the true signed gap as `margin − reported_depth`. Discovery
now enumerates contacts in **all** directions, not just along Δ.

Effect: the dense pile is now **stable** — the incomplete-discovery collapse and
ejection (a sphere reaching z = −94) are gone, and the 40× KE-injection win holds
(demo_stress KE ~1300 vs 55223). Default path untouched (11/11); box stack rests,
drop lands, 300 m/s sphere doesn't tunnel.

**Iterative NGS position solver — DONE (commit 2edb1ef).** `correct_positions()`
runs after the velocity solve: it accumulates a per-body pseudo-position
(`solid::pos_correction_`, zeroed in Pass A, folded into the commit), never
touching velocity, so it removes penetration without adding energy (the velocity
Baumgarte term is dropped). It is *non-linear* — each pair's separation is
re-derived from the running correction every visit
(`sep = discovery_gap + dot(Δb − Δa, n)`), so a body's several contacts converge
instead of summing. Two ejection sources were found and fixed along the way: the
naive one-shot summed projection (over-displaces; the reason iteration +
re-derivation are required), and **stale corrections on sleeping partners** —
sleeping/static bodies are now treated as immovable supports (zeroed first, inverse
mass forced to 0), so awake bodies take the whole correction.

Result (demo_stress headless, speculative, baumgarte 0.8 × 8 iters + gated contact
damping): deep-load floor penetration essentially gone (finalBelow 316 → ~2, no
ejection), KE ~1000 vs default 55223 (~50×), ~140 asleep vs 6, floor clean.
Default path untouched (11/11).

**Still open — full sleep.** A low ~0.7 m/s churn keeps most bodies from fully
sleeping (a position-solve / velocity-solve interaction: NGS shifts positions, the
velocities don't quite reach zero against the shifted configuration). The pile is
stable and ~50× calmer than default but not fully at rest. Next tuning targets:
expose the NGS knobs (`spec_pos_baumgarte_`, `spec_pos_iters_`, `spec_margin_`,
`spec_slop_`) as setters; consider relaxing the velocity solve's restitution
gating at rest, or a small post-NGS velocity projection; then add the full-sleep +
tunneling tests to the suite and re-tune before flipping the default. With NGS in
place, modest contact damping is beneficial again (drains the residual churn) and
no longer hovers (it is gated to real contacts) — so contact_damping does NOT need
to be 0 with speculative+NGS.

## Determinism / fixed-point

No new risk. The snap (a TOI solve) is replaced by a velocity clamp and a
multiply-add — strictly *fewer* and simpler operations. The `solve_id`
canonical pair ordering (`:1078–1090`) and the per-tick traversal flip
(`:1062`, `:190`) carry over unchanged. The one new arithmetic op is `gap/dt`;
division already exists in fixed-point. Discovery must compute `separation`
deterministically (reuse the same trace math that produces `c.depth`/`c.point`),
so float and fixed stay in lockstep.

## Test matrix

- **`test_box_stack`** (float + fixed16) — must still reach true rest; this is
  the existing solver guard.
- **New settling test** — `demo_stress` headless: assert KE decays below a tight
  bound and `asleep == COUNT` within N ticks (today it never does). This is the
  feature's reason for existing.
- **Tunneling test** — a fast body fired at a thin static wall must not pass
  through (proves CCD survives the reorder). Run float + fixed16.
- **Restitution test** — a single ball dropped at known COR returns to the
  expected height (no energy gain/loss regression).
- **Determinism test** — identical fixed16 runs are bit-identical; an A↔B
  position round-trip is stable.

## Risks and mitigations

- **Tunneling regressions** from discovery missing a contact the old sub-step
  loop would have caught. *Mitigation:* the discovery sweep spans the full
  predicted `Δ` (not first-hit-only), plus the speculative margin; the tunneling
  test gates it.
- **"Hovering"** — too large a speculative margin stops bodies short of
  surfaces. *Mitigation:* margin proportional to a small fraction of `Δ`/epsilon;
  tune against the settling and stacking tests.
- **Predicted-but-unreached contacts** (body stopped by A never reaches B).
  *Mitigation:* a contact whose gap is never closed simply contributes no
  impulse (clamped accumulator ≥ 0); it is harmless, this is the standard
  speculative behavior.
- **Re-tuning churn** — damping/sleep/iteration values balanced around the old
  leak. *Mitigation:* the toggle (Phase 1) keeps the old path for A/B until the
  matrix is green; the headless sweep harness quantifies each change.

## Status

**Deferred, not rejected.** The translation-only, game-object use case (a
character, some projectiles, a few crates) rarely needs thousand-body piles to
fully sleep, so this rework is not currently scheduled. It is written down
because hop is unusually well-positioned to do it — it already owns the swept
discovery query — and the cost is control-flow rework and re-tuning, not
technical risk to the swept/fixed-point differentiators.
