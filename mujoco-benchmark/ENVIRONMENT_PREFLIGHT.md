# Opt-in environment preflight (prototype)

Base: `mjlab-performance` at `da04a0dbb16a2b6c661beb8f5ffff1bac355689b`.
This adds a **rejection gate, not a new planner**. It does not change the neural
prediction, angle, trajectory, physics parameters, center-entry classifier, or
`hook_success=None`. With no configuration selected, execution is unchanged.

## Why this integration point

`test_direct_angle.py` already freezes image predictions, calls
`plan_candidates.py`, then calls `candidate_experiment.initialize/execute` in the
physics Python environment. The existing preflight covers IK/self-collision.
The new optional gate runs inside `execute`, **before** `RobotEngine.rollout`.
It reuses the actual loaded MuJoCo collision geoms, rather than building an
independent FCL greenhouse. No Isaac import is needed by the new checker.

## Check existing results first (read-only)

Use a per-target **physics run** containing `manifest.json`, `replay_assets`,
and `candidates/<id>/plan.json` plus `trace.json`. Do not pass the parent training
folder. Set RUN to an existing run on your own machine:

```bash
cd /root/farmily_tomato
RUN=/absolute/path/to/existing/targets/Tomato_01/physics
OUT=/root/docker_share/mujoko_debugging_data/$(date +%Y%m%d_%H%M%S)_environment_audit
./mujoco-benchmark/.venv/bin/python mujoco-benchmark/scripts/audit_environment_preflight.py \
  "$RUN" --candidate predicted_00000 \
  --policy mujoco-benchmark/config/environment_preflight.example.json \
  --output "$OUT" --gui
```

The output folder must be new and outside the source run. Original results and
states are untouched. Exit codes: 0 sampled-clear, 2 blocked, 3 inconclusive.
With `--gui`, the first rejected **nominal command pose** is displayed, with red
markers at the returned closest points. Camera drag/scroll is enabled. This is
a frozen kinematic preview, **not a physics replay or evidence of penetration
that actually occurred**. Without a rejected sample there is no failure viewer.
There is no MP4 renderer in this first change.

Outputs: `environment_preflight.json` and `environment_preflight.html`. They
include geometry inventories, time, phase, geom pair, queried distance, command,
policy hash, trace/model hashes and inspection budgets.

## Enable for NEW direct-angle / candidate runs

The environment variable deliberately propagates through the existing
subprocess pipeline, so the direct-angle script needs no copied implementation:

```bash
export FARMILY_ENV_PREFLIGHT_POLICY="$PWD/mujoco-benchmark/config/environment_preflight.example.json"
# Run your existing test_direct_angle.py or candidate_experiment.py command,
# using a NEW --output directory. No angle or controller option is changed.
# Disable for a new unguarded comparison:
# unset FARMILY_ENV_PREFLIGHT_POLICY
```

New runs snapshot the policy in their manifest. Resume cannot silently disable
or change a recorded policy. Conflicting policy hashes fail. Do not rerun
`execute` over an old result: use the read-only audit above instead.

`blocked` and `inconclusive` decisions DO NOT enter physics. Their results are
`environment_preflight_blocked` / `environment_preflight_inconclusive`, with
`physics_executed=false`, `training_eligible=false`, and no center-entry label.
They are not negative physical-outcome training examples. A `sampled_clear`
decision executes the SAME original commands and keeps the original outcome
classifier. No new `hook_success` is invented.

Each guarded candidate has a diagnostic HTML. The existing per-target dashboard
links to `environment_preflight_index.html`, which separately counts decisions,
held plans and physics executions. The legacy direct-angle top-level report may
still call all processed decisions 'attempts'; use the companion report for the
actual guarded-execution count. Do not interpret its denominator as harvest
success rate.

## Scope and policy

- All active geoms under `Robot_*` / `Hook` bodies (including descendants) versus
  all other active geoms. Names follow `build_robot_model.py`.
- Zero-mask visual geoms are excluded unless explicitly paired in MuJoCo.
  Mask/exclude filters between active geoms are NOT used to hide obstacles.
- Initial plant state is frozen in a separate `MjData`; the live engine data
  and model are not edited. No `mj_step` occurs in preflight.
- Commands are interpolated like `RobotEngine.command`, including start/end,
  with time and scalar-joint subdivision. Phase boundaries require permission
  in both phases. Robot hinge/slide joints only.
- Native `mj_geomDistance` queries follow bounding-sphere culling. Supported
  geometric primitives and collision meshes only; unsupported hfield/SDF/flex
  models are inconclusive, not silently clear. Meshes use native convex
  collision semantics, not visual triangles.
- Empty robot/environment geometry, malformed data, stale allowlist names,
  unsupported geometry, timeout or sampling/query budget: fail closed.

The example has **zero clearance**, meaning a diagnostic intersection/contact
screen, NOT a calibrated safe margin. Choose a measured positive clearance
separately; no arbitrary millimetre value is presented as real safety.

There are **no default allowed target contacts**. Strict mode may reject an
otherwise intended hook contact, or a fixed robot/base-ground contact. Inspect
the exact pair and stage before adding an exception. For example, using actual
names from YOUR inventory:

```json
{
  "clearance_m": 0.0,
  "allowed_contacts": [
    {"robot_geom": "EXACT_HOOK_GEOM", "environment_geom": "EXACT_TARGET_PEDICEL_GEOM", "phases": ["rise"]}
  ]
}
```

This example is deliberately not executable until real names are supplied.
Never allow an entire truss or all target contacts to make the test pass.
Allowlisting is a planning policy only: it DOES NOT turn physical collisions off.

## Limitations that remain

**This is GT-based, discrete, frozen-scene screening, not a safety certificate.**
A sampled-clear result can still collide due to between-sample motion, tracking
error, deformation or missing collision geoms. It does not monitor or change
commands during rollout, certify safe stopping, generate a detour, check robot
self-collision again, or check environment/environment initial overlaps.
Existing IK/self-collision and post-rollout physics audits remain necessary.
Visual-only neighboring fruit/leaves remain outside physical coverage. This is
not a real-RGB-D obstacle model and not sim-to-real validation.

`minimum_queried_distance_m` is only the minimum among queried narrow-phase
pairs/samples before termination and below the query horizon. Null is not an
infinite-clearance guarantee. `first_violation` is the first sampled time and
pair visited, not a continuous-time first impact.

## Validation

```bash
./mujoco-benchmark/.venv/bin/python -m pytest -v -rs \
  mujoco-benchmark/tests/test_environment_preflight.py
```

Local implementation environment: **30 passed, 2 skipped** (MuJoCo unavailable).
Pure tests cover command interpolation, interior/start collisions using an
analytic test double, strict phase/pair permissions, configuration integrity,
fail-closed budgets, dashboard counts, and the actual `execute` function's gate
with a mocked engine. Legacy success/classification logic is unchanged.
Two additional tests use small native MuJoCo models (sphere/box obstacles) when
installed. A branch-only GitHub Actions job runs these tests with CPU MuJoCo;
its result and exact installed version must be checked separately.

**NOT executed here:** user's full greenhouse, real model binaries, neural
weights, NVIDIA rendering, timing/throughput benchmark, or real robot. Before
larger use, audit a known clear path, gutter intersection, elbow/stem collision,
and entry-versus-rise cases using your saved scene assets. Native tests on toy
models do not substitute for this acceptance check.
