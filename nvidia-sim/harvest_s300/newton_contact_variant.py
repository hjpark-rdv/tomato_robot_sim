"""Prepare immutable belt-only direct-solref stiffness/damping experiments.

MuJoCo's positive (time constant, damping ratio) representation couples K/D.
Direct negative solref permits changing only K or only D. A separate direct-
format control is mandatory because direct parameters change contact mixing.
DISCRETE does not use the positive-solref 2*dt clamp; continuous integrators do.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from newton_contact_policy import identify_geometries
from newton_attachment_variant import direct_reference_solref
from newton_inputs import sha256


def direct_reference(solref, reference_hz, *, discrete=False, refsafe=True):
    solref = np.asarray(solref, dtype=float)
    if reference_hz <= 0 or not np.isfinite(reference_hz):
        raise ValueError('Positive reference Hz required')
    if solref.shape != (2,) or np.any(solref <= 0):
        raise ValueError('Positive source solref required for explicit conversion control')
    return direct_reference_solref(solref[None,:],reference_hz=reference_hz,
                                  discrete=discrete or not refsafe)[0]


def invariant_arrays(source, candidate, belt_ids):
    """Check all public numeric model arrays, with only selected solref exempt."""
    checks, mismatches = {}, []
    for field in dir(source):
        if field.startswith('_'):
            continue
        try:
            left, right = getattr(source, field), getattr(candidate, field)
        except AttributeError:
            continue
        if not isinstance(left, np.ndarray):
            continue
        if field == 'geom_solref':
            keep = np.ones(source.ngeom, bool); keep[belt_ids] = False
            left, right = left[keep], right[keep]
        same = bool(np.array_equal(left, right))
        checks[field] = same
        if not same:
            mismatches.append(field)
    if mismatches:
        raise ValueError('Unexpected physical model changes: '+repr(mismatches))
    return checks


def prepare(source, output, *, mode='reference_direct', factor=1., reference_hz=400.):
    source, output = Path(source).resolve(), Path(output).resolve()
    if mode not in ('reference_direct','stiffness','damping','positive_stiffness','positive_damping'):
        raise ValueError('Unknown belt contact mode')
    if not np.isfinite(factor) or factor <= 0:
        raise ValueError('Positive finite factor required')
    if mode == 'reference_direct' and factor != 1:
        raise ValueError('Reference direct format requires factor=1')
    proof = json.loads((source/'inputs.json').read_text())
    for filename, record in proof['input_files'].items():
        if record.get('sha256') and sha256(source/filename) != record['sha256']:
            raise ValueError('Source input SHA mismatch: '+filename)
    metadata = json.loads((source/'source_export.json').read_text())
    original = mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    _, _, belt_rows = identify_geometries(original, metadata)
    belts = np.array([entry['geom_id'] for entry in belt_rows])
    if original.npair:
        raise ValueError('Explicit contact pairs require a separate material mapping')
    candidate = copy.copy(original)
    discrete=original.opt.integrator==mj.mjtIntegrator.mjINT_DISCRETE
    refsafe=not bool(original.opt.disableflags & mj.mjtDisableBit.mjDSBL_REFSAFE)
    before, control, after = [], [], []
    for g in belts:
        reference = (original.geom_solref[g].copy() if mode.startswith('positive_') else
                     direct_reference(original.geom_solref[g], reference_hz,discrete=discrete,refsafe=refsafe))
        changed = reference.copy()
        if mode == 'stiffness':
            changed[0] *= factor
        elif mode == 'damping':
            changed[1] *= factor
        elif mode == 'positive_stiffness':
            changed[1] /= np.sqrt(factor)
        elif mode == 'positive_damping':
            changed[0] /= factor
            changed[1] *= factor
        candidate.geom_solref[g] = changed
        before.append(original.geom_solref[g].tolist())
        control.append(reference.tolist())
        after.append(changed.tolist())
    checks = invariant_arrays(original,candidate,belts)
    output.mkdir(parents=True,exist_ok=False)
    tree = ET.parse(source/'input_scene.xml')
    mapping = {candidate.geom(int(g)).name:candidate.geom_solref[g] for g in belts}
    for geom in tree.getroot().iter('geom'):
        if geom.get('name') in mapping:
            geom.set('solref',' '.join(format(float(x),'.17g') for x in mapping[geom.get('name')]))
    tree.write(output/'input_scene.xml',encoding='unicode')
    # The binary retains exactly the source runtime flags/mesh preprocessing.
    mj.mj_saveModel(candidate,str(output/'input_model.mjb'))
    compiled = mj.MjModel.from_binary_path(str(output/'input_model.mjb'))
    invariant_arrays(candidate,compiled,[])
    for filename in ('commands.npz','sampled_plan.json','initial_state.npz'):
        shutil.copy2(source/filename,output/filename)
    variant = dict(schema='s300-belt-contact-variant-v1',mode=mode,factor=factor,
        source_inputs=str(source),source_mjb_sha256=sha256(source/'input_model.mjb'),
        reference_hz=reference_hz,source_integrator=int(original.opt.integrator),
        source_is_discrete=bool(discrete),source_refsafe_clamp_applies=bool(refsafe and not discrete),
        reference_min_time_constant_s=2/reference_hz if refsafe and not discrete else None,
        belt_geometries=belt_rows,source_solref=before,direct_control_solref=control,
        variant_solref=after,exact_unchanged_model_arrays=checks,
        sole_mutated_model_array='geom_solref, exact moving belt geometry rows only',
        separate_control_required=True,
        direct_parameter_note='Negative solref givesrawK/Bdirectly,but bypassesDISCRETEpositiveCONTACTrowREFSAFEcap; itisNOTaneffective-equivalentcontrol. Positive_stiffnessholdstauandchangeszeta=zeta0/sqrt(alpha) (rawKalpha,rawBfixed). Positive_dampingusestau=tau0/beta,zeta=zeta0*beta (rawKfixed,rawBbeta). ActualDISCRETErowcapcanstillcoupleeffectiveK/B; reportthatseparately.',
        mixing_note='A nonpositive solref participant selects elementwise minimum instead of the positive-format weighted average. Compare each K/D variant against reference_direct, and compare reference_direct separately against original.',
        source_state_unchanged=True,source_commands_unchanged=True,
        intended_material_coefficients_changed=mode!='reference_direct',
        physical_contact_representation_changed=not mode.startswith('positive_'),
        raw_coefficient_change_scope='RawKBtargets only; sourcecontactrowREFSAFEcanchangeeffectiveKBtogether.',
        effective_source_contact_equivalence_claimed=False,training_eligible=False,harvest_success=None)
    metadata['belt_contact_variant']=variant
    (output/'source_export.json').write_text(json.dumps(metadata,indent=2)+'\n')
    proof['input_files']={name:dict(path=str(output/name),sha256=sha256(output/name)) for name in
        ('input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json')}
    proof['belt_contact_variant']=variant
    proof.setdefault('code_sha256',{})[str(Path(__file__).resolve())]=sha256(__file__)
    proof['physical_steps']=0
    (output/'inputs.json').write_text(json.dumps(proof,indent=2)+'\n')
    (output/'contact_variant.json').write_text(json.dumps(variant,indent=2)+'\n')
    return variant


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--mode',choices=('reference_direct','stiffness','damping','positive_stiffness','positive_damping'),default='reference_direct')
    parser.add_argument('--factor',type=float,default=1.)
    parser.add_argument('--reference-hz',type=float,default=400.)
    args=parser.parse_args()
    print(json.dumps(prepare(args.inputs,args.output,mode=args.mode,factor=args.factor,reference_hz=args.reference_hz),indent=2))
