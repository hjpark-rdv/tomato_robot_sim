"""Offline, opt-in serialization control for one through eight motion clones.

Export after replication/runtime opinions and before gravity-neutral warmup.
The caller must reopen the result through omni.usd, process OPENED, reacquire
Prim/API objects, and reapply SimulationManager scene/device/dt/backend settings
before native initialization. This helper never opens an Isaac process or steps
physics. Serialization is an unvalidated composition experiment, not a fix claim.
"""
from pathlib import Path
import hashlib
import json
import time

from pxr import Usd, UsdPhysics, UsdUtils

from motion_replication import (ENVS_ROOT, SOURCE_ROOTS, _assert_joint_scope,
                                _assert_relationship_scope, _compare, _snapshot)
from motion_stage_metrics import copy_stage_metrics, stage_metrics


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(2**20), b''):
            digest.update(block)
    return digest.hexdigest()


def _scene_and_groups(stage):
    rows = {}
    for prim in stage.Traverse():
        if not (prim.IsA(UsdPhysics.Scene) or prim.IsA(UsdPhysics.CollisionGroup)):
            continue
        rows[str(prim.GetPath())] = dict(type=prim.GetTypeName(),
            schemas=sorted(prim.GetAppliedSchemas()),
            attrs={attr.GetName(): repr(attr.Get()) for attr in prim.GetAttributes()
                   if attr.GetName().startswith(('physics:', 'physx', 'collection:'))},
            rels={rel.GetName(): [str(path) for path in rel.GetTargets()]
                  for rel in prim.GetRelationships()})
    return rows


def materialize_replication_stage(stage, output, replication, *, mode='layer-stack', device='cpu'):
    """Save/verify a fresh 1..8-env USDC; the caller performs the cold reopen.

    ``layer-stack`` merges session/root/sublayer opinions while retaining asset
    references. ``composition`` uses Stage.Flatten and preserves USD instancing,
    but also emits internal prototype definitions. Both are explicit diagnostics.
    """
    output = Path(output).resolve()
    audit = output.with_suffix('.materialization.json')
    if output.exists():
        raise FileExistsError('Refusing to overwrite a stage: ' + str(output))
    if audit.exists():
        raise FileExistsError(audit)
    if output.suffix != '.usdc':
        raise ValueError('Materialized clone output must be a new .usdc file')
    metadata = replication['metadata']
    num_envs = metadata['num_envs']
    if not isinstance(device, str) or device not in {'cpu', 'cuda:0'}:
        raise ValueError("Cold reopen device must be 'cpu' or 'cuda:0'")
    if type(num_envs) is not int or not 1 <= num_envs <= 8:
        raise ValueError('Cold reopen control supports 1..8 environments')
    if metadata['native_replication'] is not False:
        raise ValueError('Cold reopen requires native_replication=False; GPU callbacks cannot cross stage IDs')
    environments = replication['environments']
    if len(environments) != num_envs or any(
            env['index'] != i or env['root'] != f'{ENVS_ROOT}/env_{i}'
            for i, env in enumerate(environments)):
        raise ValueError('Replication environment metadata does not match its bounded namespace')
    if mode not in {'layer-stack', 'composition'}:
        raise ValueError("mode must be 'layer-stack' or 'composition'")
    if not stage.GetPrimAtPath(ENVS_ROOT):
        raise ValueError('Replication namespace is missing')
    if any(stage.GetPrimAtPath(path).IsActive() for path in SOURCE_ROOTS):
        raise ValueError('Original motion roots must remain inactive')
    for env in environments:
        if not stage.GetPrimAtPath(env['root']) or not stage.GetPrimAtPath(env['root']).IsActive():
            raise ValueError('An expected environment root is missing or inactive')
        _assert_joint_scope(stage, env['root'])
        _assert_relationship_scope(stage, env['root'])
    roots = tuple(path for path in (ENVS_ROOT, '/World/Greenhouse', '/World/Ground')
                  if stage.GetPrimAtPath(path) and stage.GetPrimAtPath(path).IsActive())
    before = _snapshot(stage, roots)
    policy = _scene_and_groups(stage)
    instances = sorted(str(prim.GetPath()) for prim in stage.Traverse() if prim.IsInstance())
    prototype_count = len(stage.GetPrototypes())
    original_layers = {layer.identifier: layer.ExportToString()
                       for layer in [stage.GetRootLayer(), stage.GetSessionLayer()]}
    inputs = {layer.realPath: _sha(layer.realPath) for layer in stage.GetUsedLayers()
              if layer.realPath and Path(layer.realPath).is_file()}
    started = time.perf_counter()
    layer = UsdUtils.FlattenLayerStack(stage) if mode == 'layer-stack' else stage.Flatten()
    exported = Usd.Stage.Open(layer)
    metrics = copy_stage_metrics(stage, exported)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not layer.Export(str(output)):
        raise RuntimeError('Materialized stage export failed')
    reopened = Usd.Stage.Open(str(output))
    verification = _compare(before, _snapshot(reopened, roots), 'serialized clone + shared obstacles')
    if stage_metrics(reopened) != metrics or _scene_and_groups(reopened) != policy:
        raise AssertionError('Serialization changed stage metrics or scene/collision-group policy')
    if instances != sorted(str(prim.GetPath()) for prim in reopened.Traverse() if prim.IsInstance()):
        raise AssertionError('Serialization changed shared instance paths')
    if len(reopened.GetPrototypes()) != prototype_count:
        raise AssertionError('Serialization changed prototype sharing')
    if any(reopened.GetPrimAtPath(path) and reopened.GetPrimAtPath(path).IsActive() for path in SOURCE_ROOTS):
        raise AssertionError('Serialization reactivated original motion roots')
    if reopened.GetSessionLayer().rootPrims:
        raise AssertionError('Reopened stage unexpectedly depends on session prim opinions')
    scope_checks = [dict(index=env['index'], joints=_assert_joint_scope(reopened, env['root']),
                         **_assert_relationship_scope(reopened, env['root'])) for env in environments]
    if any(layer.ExportToString() != original_layers[layer.identifier]
           for layer in [stage.GetRootLayer(), stage.GetSessionLayer()]):
        raise AssertionError('Serialization modified the live input layers')
    if any(_sha(path) != digest for path, digest in inputs.items()):
        raise AssertionError('Serialization modified an original file')
    result = dict(schema='s300-clone-materialization-v2', file=str(output),
        file_sha256=_sha(output), mode=mode, offline_export_and_verification_s=time.perf_counter()-started,
        num_envs=num_envs, device=device, native_replication_callbacks_carried=False,
        stage_metrics=metrics, preservation=verification, instance_count=len(instances),
        prototype_count=prototype_count, original_inputs_sha256=inputs,
        original_input_layers_unchanged=True, reopened_session_prim_count=0,
        environment_paths=environments, relationship_scope_checks=scope_checks, source_assets_saved=False,
        native_validation='pending', native_failure_resolution_proven=False, training_eligible=False,
        integration=['Reopen this file through omni.usd before native initialization',
                     'Process stage OPENED; reacquire stage and all USD Prim/API handles',
                     'Reapply explicit scene/device/dt/backend/fabric settings after manager reset',
                     'Register contact and break callbacks after reopening',
                     'Apply gravity-neutral warmup only after reopening'])
    audit.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result
