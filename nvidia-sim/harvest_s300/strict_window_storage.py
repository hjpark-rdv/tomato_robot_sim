"""Opt-in whole-layer exact canonical roundtrip proof for a derived window.

This proves storage identity, not successful physics or source-to-window
equivalence. Every authored field/prototype/transform/topology is compared;
the source window still needs its existing initial/path/native checks.
"""
import hashlib
import json
import math
import numbers
from pathlib import Path
import time

import numpy as np
from pxr import Gf,Sdf,Usd

from compact_motion_assets import file_sha256
from motion_stage_metrics import copy_stage_metrics,require_motion_units,stage_metrics


def authored_numeric_validity(layer):
    """Check all authored default/time-sample numeric values, not API fallbacks.

    SDK automatic-inertia fallback sentinels (un-authored -Inf) are not model
    data. Vt numeric array buffers can be shared by many source mesh specs;
    retain the exact immutable array owner while caching its finite test.
    """
    paths=[];layer.Traverse(Sdf.Path.absoluteRootPath,lambda path:paths.append(path))
    counters=dict(authored_specs=len(paths),authored_attributes=0,numeric_values=0,
                  numeric_array_elements=0,shared_numeric_array_checks=0,time_samples=0)
    array_cache={}
    def check(value,path):
        if isinstance(value,(str,bytes,Sdf.AssetPath,Sdf.Path)) or value is None:return
        if isinstance(value,numbers.Number):
            finite=np.isfinite(value) if isinstance(value,complex) else math.isfinite(value)
            if not finite:raise ValueError('Nonfinite authored numeric value: '+str(path))
            counters['numeric_values']+=1;return
        if isinstance(value,(Gf.Quatf,Gf.Quatd,Gf.Quath)):
            check([value.GetReal(),*value.GetImaginary()],path);return
        if isinstance(value,dict):
            for item in value.values():check(item,path)
            return
        try:array=np.asarray(value)
        except (TypeError,ValueError):array=None
        if array is not None and array.dtype.kind in 'biufc':
            if array.size>256:
                key=(array.__array_interface__['data'][0],array.dtype.str,array.shape,array.strides)
                if key in array_cache:
                    counters['shared_numeric_array_checks']+=1;return
                array_cache[key]=array
            if not np.isfinite(array).all():raise ValueError('Nonfinite authored numeric array: '+str(path))
            counters['numeric_array_elements']+=int(array.size);counters['numeric_values']+=1;return
        if isinstance(value,(list,tuple)) or type(value).__module__=='pxr.Vt':
            for item in value:check(item,path)
    for path in paths:
        spec=layer.GetObjectAtPath(path)
        if not isinstance(spec,Sdf.AttributeSpec):continue
        counters['authored_attributes']+=1
        if spec.HasInfo('default'):check(spec.GetInfo('default'),path)
        for stamp in layer.ListTimeSamplesForPath(path):
            check(layer.QueryTimeSample(path,stamp),path);counters['time_samples']+=1
    counters.update(passed=True,scope='All authored attribute defaults and time samples; SDK un-authored fallbacks excluded')
    return counters


def canonical_layer_digest(layer):
    """Entire canonical Sdf representation, no text truncation or sampling."""
    value=layer.ExportToString()
    encoded=value.encode('utf-8')
    return dict(sha256=hashlib.sha256(encoded).hexdigest(),bytes=len(encoded))


def require_self_contained(layer):
    if layer.subLayerPaths or any(layer.GetExternalReferences()) or layer.GetExternalAssetDependencies():
        raise ValueError('Exact layer proof requires a self-contained layer without external asset dependencies')


def require_identity_wrapper(wrapper,materialized):
    root=wrapper.GetRootLayer();allowed={'subLayers','subLayerOffsets','defaultPrim','metersPerUnit','kilogramsPerUnit',
        'upAxis','timeCodesPerSecond','framesPerSecond','startTimeCode','endTimeCode'}
    if (list(root.rootPrims) or set(root.pseudoRoot.ListInfoKeys())-allowed or
            list(root.subLayerPaths)!=[materialized.name] or list(root.subLayerOffsets)!=[Sdf.LayerOffset()] or
            set(root.GetExternalReferences())!={materialized.name} or root.GetExternalAssetDependencies()):
        raise ValueError('Materialization wrapper must have only an identity sublayer and stage metadata')
    direct=Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(materialized)))
    identity=lambda s:(stage_metrics(s),str(s.GetDefaultPrim().GetPath()) if s.GetDefaultPrim() else None,
        s.HasAuthoredTimeCodeRange(),s.GetStartTimeCode(),s.GetEndTimeCode())
    if identity(wrapper)!=identity(direct):raise ValueError('Wrapper changed composed units/default prim/time range')
    return dict(identity_sublayer=True,no_prim_opinions=True,no_unknown_metadata=True,
                stage_metadata_matches_direct=True)


def materialize_exact_layer(stage,output,*,preserve_authored_layer=False):
    """Write NEW USDC/wrapper with exact full canonical equality and SHA."""
    output=Path(output);materialized=output/'composed.usdc';scene=output/'scene.usda'
    if materialized.exists() or scene.exists():raise FileExistsError('Refusing to overwrite materialized window files')
    output.mkdir(parents=True,exist_ok=True);require_motion_units(stage)
    start=time.perf_counter();timings={}
    def timed(name,fn):
        tick=time.perf_counter();value=fn();timings[name]=time.perf_counter()-tick;return value
    if type(preserve_authored_layer) is not bool:
        raise ValueError('Explicit boolean authored-layer preservation option required')
    if preserve_authored_layer:
        require_self_contained(stage.GetRootLayer())
        session=stage.GetSessionLayer()
        if session.rootPrims or session.subLayerPaths or session.pseudoRoot.ListInfoKeys():
            raise ValueError('Authored-layer copy requires no session-layer opinions')
        def clone_authored_layer():
            layer=Sdf.Layer.CreateAnonymous();layer.TransferContent(stage.GetRootLayer());return layer
        flat=timed('authored_layer_clone_s',clone_authored_layer)
    else:
        flat=timed('flatten_s',lambda:stage.Flatten(addSourceFileComment=False))
    require_self_contained(flat)
    finite=timed('authored_numeric_finite_s',lambda:authored_numeric_validity(flat))
    before=timed('source_canonical_digest_s',lambda:canonical_layer_digest(flat))
    timed('usdc_write_s',lambda:flat.Export(str(materialized)))
    reopened=timed('reopen_isolated_s',lambda:Sdf.Layer.OpenAsAnonymous(str(materialized)))
    if not reopened or reopened.GetFileFormat().formatId!='usdc':raise ValueError('Expected reopened USDC layer')
    require_self_contained(reopened)
    after=timed('roundtrip_canonical_digest_s',lambda:canonical_layer_digest(reopened))
    if before!=after:raise ValueError('Whole canonical authored layer changed during USDC materialization')
    wrapper=Usd.Stage.CreateNew(str(scene));wrapper.GetRootLayer().subLayerPaths=[materialized.name]
    copy_stage_metrics(stage,wrapper)
    if stage.HasAuthoredTimeCodeRange():
        wrapper.SetStartTimeCode(stage.GetStartTimeCode());wrapper.SetEndTimeCode(stage.GetEndTimeCode())
    if stage.GetDefaultPrim():wrapper.SetDefaultPrim(wrapper.GetPrimAtPath(stage.GetDefaultPrim().GetPath()))
    wrapper.GetRootLayer().Save();wrapper_proof=require_identity_wrapper(wrapper,materialized)
    report=dict(schema='s300-whole-canonical-layer-storage-identity-v1',passed=True,
        source_canonical=before,roundtrip_canonical=after,canonical_equal=True,
        source_composition_mode='exact_self_contained_authored_layer_copy' if preserve_authored_layer else 'composed_stage_flatten',
        full_authored_layer_checked=True,numeric_validity=finite,identity_wrapper=wrapper_proof,
        materialized_sha256=file_sha256(materialized),wrapper_sha256=file_sha256(scene),
        timing_s=timings,total_wall_s=time.perf_counter()-start,
        python_resolved_property_inventory_recomputed=False,world_vertices_recomputed=False,
        proof_scope='Exact full canonical authored content under identical schema/runtime and identity wrapper; not source-to-derived geometry or native physics success',
        native_validation='pending',training_eligible=False,harvest_success=None)
    (output/'checkpoint_materialization_identity.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
