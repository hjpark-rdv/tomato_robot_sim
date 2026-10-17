"""Independent CONNECT representation experiment; preserve angular/release mechanics.

Positive solref is clamped to 2*dt for continuous integrators. The source's
DISCRETE integrator explicitly does NOT apply that clamp to CONNECT rows.
Direct negative-format solref expresses the SAME source K/B. A second variant doubles translational
damping while retaining position stiffness. Neither changes angular springs,
deadband, topology, collision geometry, masses, nor force/torque break limits.
This is an opt-in diagnostic, not calibrated real tomato attachment physics.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from newton_inputs import sha256


def direct_reference_solref(source, *, reference_hz=400., damping_factor=1., discrete=False):
    """Return direct coefficients matching source K/B at reference timestep.

    The shared dmax divisors cancel: direct k=1/(tc*zeta)^2 and b=2/tc.
    dmax and positional impedance remain unchanged in the prepared model.
    """
    source = np.asarray(source, dtype=float)
    if source.ndim != 2 or source.shape[1] != 2 or not np.isfinite(source).all():
        raise ValueError('Expected finite [neq,2] solref array')
    if not np.isfinite(reference_hz) or reference_hz <= 0 or \
            not np.isfinite(damping_factor) or damping_factor <= 0:
        raise ValueError('Positive finite reference frequency/damping factor required')
    if np.any(source <= 0):
        raise ValueError('Reference conversion requires positive timeconstant/dampratio')
    tc = source[:, 0] if discrete else np.maximum(source[:, 0], 2/reference_hz)
    stiffness = 1/(tc*source[:, 1])**2
    damping = 2/tc * damping_factor
    return np.stack([-stiffness, -damping], axis=1)


def prepare(source, output, *, reference_hz=400., damping_factor=1.):
    source, output = Path(source).resolve(strict=True), Path(output).resolve()
    original = mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    proof = json.loads((source/'inputs.json').read_text())
    for name, value in proof['input_files'].items():
        if sha256(source/name) != value['sha256']:
            raise ValueError('Source input hash mismatch: '+name)
    if original.neq != 10 or np.any(original.eq_type != mj.mjtEq.mjEQ_CONNECT):
        raise ValueError('Expected exactly ten existing fruit CONNECT constraints')
    if original.opt.disableflags & mj.mjtDisableBit.mjDSBL_REFSAFE:
        raise ValueError('Reference assumes existing REFSAFE is enabled')
    discrete = original.opt.integrator == mj.mjtIntegrator.mjINT_DISCRETE
    values = direct_reference_solref(original.eq_solref, reference_hz=reference_hz,
                                     damping_factor=damping_factor, discrete=discrete)
    meta = json.loads((source/'source_export.json').read_text())
    specs = meta['attachments']
    for spec in specs:
        if spec['force_n'] != 3. or spec['torque_nm'] != .008:
            raise ValueError('Unexpected release reference; refuse implicit threshold change')
    tree = ET.parse(source/'input_scene.xml')
    elements = {node.get('name'): node for node in tree.getroot().iter('connect')}
    for index in range(original.neq):
        name = original.equality(index).name
        if name not in elements:
            raise ValueError('Missing source CONNECT XML: '+name)
        elements[name].set('solref', ' '.join(format(float(v), '.17g') for v in values[index]))
    output.mkdir(parents=True, exist_ok=False)
    tree.write(output/'input_scene.xml', encoding='unicode')
    # Preserve original compiled model arrays; XML is retained for metadata import.
    old_solref = original.eq_solref.copy()
    original.eq_solref[:] = values
    mj.mj_saveModel(original, str(output/'input_model.mjb'))
    reloaded = mj.MjModel.from_binary_path(str(output/'input_model.mjb'))
    source_model = mj.MjModel.from_binary_path(str(source/'input_model.mjb'))
    changed = []
    for name in dir(source_model):
        a, b = getattr(source_model, name), getattr(reloaded, name)
        if isinstance(a, np.ndarray) and not np.array_equal(a, b):
            changed.append(name)
    if changed != ['eq_solref']:
        raise ValueError('Unexpected model array changes: '+repr(changed))
    for name in set(proof['input_files']) | {'sampled_plan.json'}:
        if name in ('input_scene.xml', 'input_model.mjb', 'source_export.json'):
            continue
        shutil.copy2(source/name, output/name)
    variant = dict(schema='s300-attachment-solref-variant-v1', source_inputs=str(source),
        source_mjb_sha256=sha256(source/'input_model.mjb'),
        mode='direct_reference' if damping_factor == 1 else 'direct_same_stiffness_overdamped',
        reference_hz=reference_hz, damping_factor=damping_factor,
        source_integrator=int(original.opt.integrator), source_is_discrete=bool(discrete),
        source_positive_timeconstant_refsafe_clamp_applies=not discrete,
        old_eq_solref=old_solref.tolist(), new_eq_solref=values.tolist(),
        changed_model_arrays=changed, physical_model_changed=damping_factor != 1,
        direct_format_not_refsafe_clamped=True,
        reference_equivalence_scope=('translational K/B at all dt: DISCRETE does not clamp CONNECT timeconstant' if discrete else
            'translational K/B at fixed reference dt; other dt differs because positive continuous-integrator source solref is dt-clamped'),
        position_stiffness_preserved_at_reference_hz=True,
        angular_spring_damping_deadband_preserved=True,
        release_force_n=[s['force_n'] for s in specs],
        release_torque_nm=[s['torque_nm'] for s in specs],
        release_semantics_preserved=True, topology_changed=False, geometry_changed=False,
        training_eligible=False, harvest_success=None)
    meta['attachment_variant'] = variant
    (output/'source_export.json').write_text(json.dumps(meta,indent=2)+'\n')
    proof['input_files'] = {name:dict(path=str(output/name),sha256=sha256(output/name))
        for name in proof['input_files']}
    proof['attachment_variant'] = variant
    proof.setdefault('code_sha256',{})[str(Path(__file__).resolve())] = sha256(__file__)
    (output/'inputs.json').write_text(json.dumps(proof,indent=2)+'\n')
    (output/'attachment_variant.json').write_text(json.dumps(variant,indent=2)+'\n')
    return variant


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--reference-hz',type=float,default=400.)
    parser.add_argument('--damping-factor',type=float,default=1.)
    args=parser.parse_args()
    print(json.dumps(prepare(args.source,args.output,reference_hz=args.reference_hz,
                            damping_factor=args.damping_factor),indent=2))
