"""Explicit verified native entry layer selection; never alter the source asset.

The public wrapper is the default. The opt-in selects its exact immutable
materialized USDC dependency before the application opens a native stage.
Build-time USD/property equivalence does not guarantee PhysX runtime parity;
the source-wrapper/direct-layer A/B remains separate native evidence.
"""
import argparse
import json
from pathlib import Path

from pxr import Sdf,Usd

from compact_motion_assets import file_sha256,property_inventory,verify_compact_dependencies
from motion_stage_metrics import require_motion_units,stage_metrics


def select_native_scene(assembly,*,use_materialized=False):
    """Return both immutable source/native paths and hashes, with unchanged default.

    Direct selection requires a pure single-dependency wrapper: no authored
    root prims/overrides may be silently discarded. All composed data remain in
    the verified USDC. Runtime property edits belong to the opened native stage,
    not a saved change to either input file.
    """
    assembly=Path(assembly).resolve();source=assembly/'scene.usda'
    manifest=json.loads((assembly/'motion_manifest.json').read_text());source_sha=file_sha256(source)
    if source_sha!=manifest.get('scene_sha256'):raise ValueError('Native source wrapper SHA mismatch')
    dependency=verify_compact_dependencies(assembly,manifest)
    runtime=source
    if use_materialized:
        if not dependency or dependency.get('format')!='usdc' or dependency.get('materialized') is not True:
            raise ValueError('Explicit verified materialized USDC dependency required')
        runtime=(assembly/dependency['relative_path']).resolve()
        if not runtime.is_relative_to(assembly):raise ValueError('Native materialized dependency escaped its assembly')
        wrapper=Sdf.Layer.FindOrOpen(str(source));materialized=Sdf.Layer.FindOrOpen(str(runtime))
        if (not wrapper or not materialized or list(wrapper.rootPrims) or
                list(wrapper.subLayerPaths)!=[dependency['relative_path']] or
                set(wrapper.GetExternalReferences())!={dependency['relative_path']} or materialized.subLayerPaths or
                any(materialized.GetExternalReferences())):
            raise ValueError('Direct native layer selection would discard wrapper overrides or external composition')
    return dict(schema='s300-native-stage-selection-v1',source_assembly=str(assembly),
        source_scene=str(source),source_scene_sha256=source_sha,native_scene=str(runtime),
        native_scene_sha256=file_sha256(runtime),materialized_requested=bool(use_materialized),
        mode='verified_materialized_direct' if use_materialized else 'existing_public_wrapper',
        verified_materialized_dependency=dependency,source_asset_modified=False,
        resolved_property_comparison='not performed by selector; optional offline verification available',
        native_runtime_validation='Separate native A/B required; storage/property equality is not native physics equality',
        training_eligible=False,harvest_success=None)


def verify_native_scene_composition(assembly,*,report_path=None):
    """Offline USD resolved-property/metrics proof, independent of PhysX startup."""
    result=select_native_scene(assembly,use_materialized=True)
    source=Usd.Stage.Open(result['source_scene']);native=Usd.Stage.Open(result['native_scene'])
    require_motion_units(source);require_motion_units(native)
    left=property_inventory(source);right=property_inventory(native)
    if left!=right:raise ValueError('Source wrapper and native direct layer have different resolved properties')
    if stage_metrics(source)!=stage_metrics(native):raise ValueError('Source/native stage metrics differ')
    if str(source.GetDefaultPrim().GetPath())!=str(native.GetDefaultPrim().GetPath()):
        raise ValueError('Source/native default prim differs')
    source_range=(source.HasAuthoredTimeCodeRange(),source.GetStartTimeCode(),source.GetEndTimeCode())
    native_range=(native.HasAuthoredTimeCodeRange(),native.GetStartTimeCode(),native.GetEndTimeCode())
    if source_range!=native_range:raise ValueError('Source/native authored time range differs')
    result.update(source_properties=left,native_properties=right,stage_metrics=stage_metrics(native),
        resolved_property_comparison='passed exact full resolved property inventory',
        wrapper_resolved_properties_match_native_layer=True)
    if report_path is not None:
        with Path(report_path).open('x',encoding='utf-8') as stream:
            json.dump(result,stream,ensure_ascii=False,allow_nan=False,indent=2);stream.write('\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('assembly',type=Path)
    parser.add_argument('--materialized',action='store_true')
    parser.add_argument('--verify-composition',action='store_true')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args()
    if args.verify_composition:
        result=verify_native_scene_composition(args.assembly,report_path=args.report)
    else:
        result=select_native_scene(args.assembly,use_materialized=args.materialized)
        if args.report is not None:
            with args.report.open('x',encoding='utf-8') as stream:
                json.dump(result,stream,ensure_ascii=False,allow_nan=False,indent=2);stream.write('\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
