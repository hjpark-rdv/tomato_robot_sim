# Environment preflight: opt-in prototype

Base: `mjlab-performance` at `da04a0dbb16a2b6c661beb8f5ffff1bac355689b`.
Branch: `codex/mujoco-environment-preflight`.

## What changed

This is a **nominal-path rejection gate, not a new planner**. It leaves the
image prediction, angle, trajectory, actuator control, physics settings,
center-entry classifier and `hook_success=None` unchanged. With no policy
selected, the existing execution path is unchanged.

The existing `test_direct_angle.py` freezes predictions, calls
`plan_candidates.py`, and invokes `candidate_experiment.initialize/execute`
in the physics Python environment. The new optional check runs in `execute`
before `RobotEngine.rollout`. It uses the **actual loaded MuJoCo collision
geoms**, not a second FCL greenhouse. Existing IK/self-collision checks remain.

## First use: read-only inspection of an existing result

Use a per-target **physics run** containing `manifest.json`, `replay_assets`,
and `candidates/<id>/plan.json` and `trace.json`. Do not pass the parent training
folder. Replace RUN with one of your existing paths:

```bash
cd /root/farmily_tomato
RUN=/absolute/path/to/existing/targets/Tomato_01/physics
OUT=/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_environment_audit
DISPLAY=:0 ./mujoco-benchmark/.venv/bin/python \
  mujoco-benchmark/scripts/audit_environment_preflight.py \
  "$RUN" --candidate predicted_00000 \
  --policy mujoco-benchmark/config/environment_preflight.example.json \
  --output "$OUT" --gui
```

The output folder must be new and outside the original run. The audit does not
rewrite the original results, rerun dynamics or retrain the model.

Outputs: `environment_preflight.json` and `environment_preflight.html`, with
sampled time, phase(s), robot/environment geom names, distance, robot command,
geometry inventory, trace/model/policy hashes and processing counters.

With `--gui`, the first rejected **nominal command pose** opens in the interactive
MuJoCo viewer, with red markers at the returned closest points. The camera starts
in free mode and is not overwritten each frame. This is a frozen kinematic
preview, **not a physical replay or evidence that this exact penetration actually
occurred**. No rejected sample means no failure viewer. MP4 export is not added.

Exit codes: 0 = sampled-clear, 2 = blocked, 3 = inconclusive. Nonzero here is an
inspection result, not automatically a software crash.

## Enable the check for NEW experiments

```bash
export FARMILY_ENV_PREFLIGHT_POLICY="$PWD/mujoco-benchmark/config/environment_preflight.example.json"
# Run the existing test_direct_angle.py or candidate_experiment.py command,
# with a NEW output directory. Existing angles/controllers are unchanged.
# For a new unguarded baseline run:
# unset FARMILY_ENV_PREFLIGHT_POLICY
```

The variable propagates through existing subprocesses; no duplicate direct-angle
pipeline is needed. New manifests snapshot the policy. A recorded policy cannot
silently be changed or disabled on resume; conflicting overrides fail.
Use the standalone audit for old runs, not an execution/resume wrapper. The
execution gate refuses a candidate with existing result/state files, but old
external wrappers may have their own exception/result writers.

Outcomes:

- `environment_preflight_blocked`: observed sampled distance violates policy.
- `environment_preflight_inconclusive`: invalid input, unsupported model, timeout,
  sampling/query budget, or checker error.
- `sampled_clear`: all requested samples passed; the original commands then run.

Blocked/inconclusive candidates have `physics_executed=false`,
`training_eligible=false`, `center_entered=null`, `hook_success=null`, and no
`states.npz`. **Do not train on them as physical failures.** This patch does not
modify training loaders. A passed check leaves the original physics outcome
classifier intact.

The per-target report links to `environment_preflight_index.html`, separating
checked decisions, held plans and actually executed rollouts. The original
higher-level direct-angle report may still call all processed decisions
'attempts'; the companion counters are the correct denominator for guarded runs.

## Policy and coverage

The example deliberately has `clearance_m=0.0`: it is an **intersection/contact
diagnostic**, not a measured safe margin for a real robot. Use a separate policy
with a measured positive margin when appropriate. The sample step sizes and
budgets are diagnostic settings, not certified collision bounds.

There are no blanket target exceptions. Strict mode can reject intended hook
contact or an existing base/ground mounting contact. Inspect the pair first,
then allow only verified exact pairs in explicit stages, for example:

```json
{
  "clearance_m": 0.0,
  "allowed_contacts": [
    {
      "robot_geom": "EXACT_HOOK_GEOM",
      "environment_geom": "EXACT_TARGET_PEDICEL_GEOM",
      "phases": ["rise"]
    }
  ]
}
```

The placeholders must be replaced with actual names from the audit inventory.
Unknown names or phases are rejected. No wildcards. At a phase boundary a pair
must be allowed in both adjacent phases. Never allow the entire truss simply to
get a pass. These permissions affect only preflight: physical collisions remain
unchanged.

Implementation coverage:

- Active geoms under `Robot_*`/`Hook` bodies and descendants, versus all other
  active geoms, following `build_robot_model.py` naming.
- Zero-mask visual geoms excluded unless explicitly paired. Collision masks or
  body exclusions between active geoms do not silently remove obstacles here.
- Separate `MjData` at the engine's reset joint positions. The environment is
  frozen; the live data/model are not changed. No `mj_step` in the checker.
- The same 60 Hz piecewise-linear command interpolation as `RobotEngine.command`,
  with time and scalar-joint subdivision, including initial/final samples.
- Bounding-sphere culling then native `mj_geomDistance` queries. Environment
  planes are never culled by radius. Sphere/capsule/box/ellipsoid/cylinder/mesh
  supported; unsupported hfield/SDF/flex coverage fails closed.
- Collision meshes have native convex semantics, not the visual triangle shape.
  Legacy convex-distance mode is refused without modifying model flags. MuJoCo's
  `nativeccd` means native **convex** collision detection, not continuous-time CCD.

MuJoCo documents that distance results differ under the legacy convex pipeline:
https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-geomdistance

## What a pass does NOT prove

This is GT-based, **discrete, frozen-scene screening**, not a robot safety system.
Between-sample collisions, real tracking error, motion/deformation of plants,
and missing collision geometry remain possible. There is no online monitor,
certified stop/retreat, detour generation, hook-success detector or REAL RGB-D
obstacle reconstruction in this patch.

It does not recheck robot self-collision or environment/environment initial
intersections. Existing checks remain necessary. Visual-only neighboring fruit
and leaves remain outside coverage: a visually present object is not necessarily
a checked collider. A missing robot/environment geom set is inconclusive.

`minimum_queried_distance_m` is only the minimum of narrow-phase distances
actually queried before termination and below the query horizon. Null does not
mean infinite clearance. `first_violation` is the first sampled violating pair,
not a continuous-time first impact.

## Validation and acceptance

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./mujoco-benchmark/.venv/bin/python -m pytest -v -rs \
  mujoco-benchmark/tests/test_environment_preflight.py
```

Local implementation tests: **33 passed, 2 skipped** because MuJoCo is absent.
The passing tests cover policy integrity, phases, interpolation, analytic
geometry test doubles, fail-closed budgets/errors, report counters, and the
actual `execute` function with a mocked engine. They are not plant simulations.
Two native adapter tests run when MuJoCo is installed. The branch-specific
GitHub Actions job installs CPU MuJoCo and runs both sets; inspect its result
separately. The unchanged classifier was also checked against the base AST.

**Not run in the authoring container:** original greenhouse model binaries,
trained neural weights, GPU viewer/video, real robot, or throughput benchmark.
Before enabling large runs, audit known clear, gutter-intersecting,
elbow/stem-colliding, and rise-stage cases on the actual scene. Review any
initial mounting contact and intended target-contact permissions explicitly.
Passing toy geometry tests does not replace these acceptance checks.
