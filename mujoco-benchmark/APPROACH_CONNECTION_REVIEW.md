# Approach connection: stop multiplying rejected templates

## Evidence boundary (2026-09-28)

The user's supplied server handoff reports 240 proposals, 239 IK/robot passes,
239 environment rejections, one IK-not-found, zero new physics executions, and
92 paths rejected before their four families diverged. These figures have NOT
been independently checked in the latest server artifacts in this turn.

The connected GitHub API returned 404 for `codex/server-multifamily-20260928`
and for its named report; branch enumeration also lacked that branch. The
previous `farmily_tomato` repository did not expose it either. This work is
therefore an additive implementation on the actually readable commit
`d8db76d2d3a7e62081cf15c1f26a32c538676eed`. Do not replace the server's newest
branch with this older base. Cherry-pick the additive files ON TOP OF the
server branch, preserving its cache, pair-lookup and robot-facing-mouth fixes.

Read directly: `dataset_motion.plan`, `plan_candidates.initialize/compute`,
`RobotKinematics.ik/fk`, `SelfCollisionCheck`, `MuJoCoScene`, and the current
search contact-policy implementation. The existing prefix is smooth joint
interpolation to a single IK solution; the next approach segment is Cartesian
interpolation. Collision detection rejects those paths but does not route
around obstacles. This is the concrete software gap addressed here.

## Decision

Keep the simulator, robot, scene generator, policy and evidence collection.
Do not continue the same pattern of more hand-designed family points followed
by rejection. First compare a conventional obstacle-aware approach connector
against the original prefix, on the SAME target/pose/policy.

Do not replace everything with unconstrained full-arm RL yet. The reported
new candidates never reached physics. This measures path accessibility and
proposal quality, not whether RL or physics can learn local hooking. RL also
needs an operational reward and usable reset-state distribution. Once valid
staging states exist, bounded local RL can be compared with local trajectory
search. An RL teacher may use simulator truth, but that is NOT evidence that
an initial-RGB-D student can make the same decisions.

## Added implementation

- `approach_connection.py`: OMPL RRTConnect wrapper on normalized bounded
  joints; exact-solution requirement, independent edge checks, multi-start
  existing IK, actual-FK residual check, safe trace joining and speed-limited
  edge-preserving retiming. No custom physics or new RL engine.
- `plan_approach_connection.py`: reads an immutable target snapshot and one
  explicit candidate. Replaces ALL leading preapproach/approach waypoints up
  to the handoff before insertion. It does not merely patch the first move.
- Reuses the existing FCL self-checker and native MuJoCo collider inventory
  DURING graph expansion, including exact current target/own-rachis contact
  policy. Neither source material nor masks nor robot geometry is edited.
- Considers multiple valid IK branches. Plans the unchanged local suffix from
  the REACHED branch, not from an old branch or a teleported state. When a
  suffix fails, tries another branch within the declared budget.
- Rechecks the assembled 60 Hz command path with existing environment checks.
  Writes a full manipulation plan/trace only if the full check completes and passes.
  Also exports separately checked `approach_only_run` snapshots followed by a
  one-second hold, so suffix failure does not block testing actual arrival.
  These are marked `manipulation_attempt=false`, never harvest attempts.
- Stores `prefix_found` separately from `whole_path_passed`; never calls a
  valid prefix a harvest success. `impossible=null` on timeout/IK exhaustion.
- All previous modules are unchanged. This opt-in tool must not be fed to the
  old collector without an explicit integration decision.

The source candidate's parameters are retained for provenance. The replacement
joint-space prefix is recorded separately in `plan.approach_connection`, since
it is not representable by the original Cartesian prefix or action14. For
actual execution/replay use the new `trace.json`, not a replan from the old
parameter list.

## Usage

Use an isolated Python environment with the EXISTING planning dependencies
(Torch, FCL, etc.), native MuJoCo matching the snapshot and `ompl==2.0.1`.
Do not upgrade the user's working environment in place. The new binding API
uses `allocState()` and a direct Python validity callback, not OMPL 1.x's
`ob.State(space)`/`StateValidityCheckerFn`.

```bash
# RUN is a real source run root with replay_assets and candidates.json.
# BASE is that target's existing base_policy.json (not the expanded snapshot).
# CID is an actual candidate ID selected BEFORE inspecting new results.
# OUT must not already exist and must be outside RUN.
timeout --kill-after=10s 180s "$PLAN_PY" \
  mujoco-benchmark/scripts/plan_approach_connection.py "$RUN" \
  --candidate "$CID" --base-policy "$BASE" --output "$OUT" \
  --ik-seeds 8 --max-branches 3 --solve-seconds 8 \
  --total-seconds 120 --seed 20260928
```

`connection_result.json` is the summary. `branch_XX/prefix_joint_path.json`
is geometry-only, not a physics video. On full success, `OUT/run` is the new
snapshot for the existing `target_fruit_contact_trial.py` with the SAME base
policy, `--search-contact-policy`, and existing 5N/20mm experimental bounds.
A separately checked `branch_XX/approach_only_run` can test arrival even when
its local suffix failed. Its candidate ID ends in `_approach`. Keep its results
out of manipulation-success denominators. The hold measures arrival stability,
not hooking. This tool does not launch physics. Those bounds are not real-robot safety or
fruit-damage limits. Verify actual execution with the source engine afterward.

The Python timer is advisory while a native routine is executing. Use the outer
process timeout above as well. Timeouts without an atomic result are execution
errors, not target failures.

## One bounded decision round, not another unlimited campaign

1. First publish/identify the missing server branch, then cherry-pick the
   additive connector files onto it. Run its existing regressions and these
   tests. Keep every source scene, snapshot, candidate and policy hash.
2. Select four fixed targets from previously prefix-blocked groups, covering
   at least two valid scenes and including the second truss. Keep two targets
   previously blocked only after insertion as controls. No new scenes yet.
3. For the four prefix cases, compare original interpolation with the connector
   on the SAME three handoff poses per target: 12 paired queries. Use low-tilt
   poses first, selected before scoring; vary heading separately. Do not vary
   heading, roll, pitch, height and standoff all at once.
4. Per query: at most 8 IK seeds, 3 route/suffix branches, 8 seconds OMPL per
   branch, 120 seconds overall planning. Maximum 12 new full-start physics
   attempts, including approach-only tests; only fully checked traces execute. No threshold sweeps.
5. Report initial-state validity, unique IK solutions, goal self/environment
   validity, prefix found, suffix validity, full trace validity, actual arrival
   and contact outcome separately. Compare counts per UNIQUE handoff, not four
   copied family prefixes. Report time and geometry-query counts.
6. If at least two of the four prefix targets reach a checked staging pose,
   stop prefix engineering. Compare three local path families at those states,
   then replay promising full-start paths. The two-of-four threshold is a
   practical predeclared progress gate, NOT a statistical success claim.
7. If fewer than two improve, terminate this connector round. Classify the
   blocker: invalid start, no valid staging IK, valid endpoints but no route,
   or suffix blocked. A second bounded direction decision is required; do not
   keep increasing all budgets or call those targets physically impossible.

For side-mouth link3 collisions, a colliding goal cannot be fixed by more
RRT iterations. Try the bounded alternative IK branches, then change a
predeclared standoff/low-tilt staging pose while checking the mount and link3.
Do not ignore self-collision or quietly change the final local goal. Preserve
the server's robot-facing-mouth convention; it was not available for review.

## RL and other fallback conditions

- Prefix reachable but local motion fails across distinct valid staging states:
  compare local task-space RL with parameter search using the same resets,
  observation/action availability, contact policy and physics-step budget.
  Use 3 seeds and a predeclared step/time cap; record real physics steps and
  wall time, not just policy updates. Do not train free-arm full-episode RL
  merely to replace a missing obstacle-aware transit planner.
- Start/goal self collision dominates: fix staging representation or test a
  different physically permitted lift/base setup as a separate task. No method
  can execute an actually invalid configuration; IK timeout is not a proof.
- Valid endpoints but repeated route failure: inspect collision geometry and
  narrow passage clearance; use a planning baseline or separate simplified
  diagnostic, not a hidden removal of obstacles from reported experiments.
- Suspected simulator/evaluator mismatch: a small labelled validation on a
  simple physical/fixture arrangement is preferable to ever more synthetic
  successes. A 1.4mm gap alone neither proves real failure nor real success.
- Wider data collection resumes only after the batch makes actual executions
  on several targets; then expand to 80 fruits and later independent scenes.
  Keep successful evidence, valid-path failure, search-unresolved and simulator
  error distinct. No negative training label from a failed finite search.

## Validation scope

Initial local test: 10 passed, 3 skipped because OMPL/MuJoCo were absent in the
chat container. Native OMPL and MuJoCo tests run in CI. The tests include a
blocked straight path with an available detour, a wall with no exact solution,
metric resolution for mixed slide/hinge joints, source-preserving suffix
planning, join continuity and a private MuJoCo state. They are not an RB5 or
whole-greenhouse integration run. Check the actual CI status before using.

Still unverified: the newest server code, real scene binaries, full Torch/FCL
adapter execution on those binaries, whole-arm success improvement, runtime
cost and actual approach videos. No current physical harvesting success claim.

## Primary references used in the design review

- OMPL RRTConnect and state validity: https://ompl.kavrakilab.org/core/classompl_1_1geometric_1_1RRTConnect.html
  and https://ompl.kavrakilab.org/core/stateValidation.html
- OMPL Python binding API: https://ompl.kavrakilab.org/python.html
- IndustReal: https://arxiv.org/abs/2305.17110
- Residual Learning from Demonstration: https://arxiv.org/abs/2008.07682

These support the algorithmic choice and the possible later role of RL. They
are not validation of this hook, plant, observation model or success detector.
