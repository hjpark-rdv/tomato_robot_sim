"""Authorization is separate from physical validity and successful hooking."""
import copy
import importlib.util
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import target_fruit_contact_trial as tc
import environment_preflight as ep

S = tc.ContactScope('fruit', ('rear', 'front'), ('pedicel',), ('rear',))


def test_tcp_speed_is_poststep_displacement_over_actual_physics_dt():
    r=tc.tcp_sample([1.,2.,3.],[1.,2.,3.00001],.005)
    assert r['actual_tcp_speed_m_s']==pytest.approx(.002)
    with pytest.raises(ValueError):tc.tcp_sample([0,0,0],[0,0,0],0.)


def base():
    return dict(clearance_m=0., allowed_contacts=[dict(robot_geom='rear', environment_geom='pedicel',
                                                    phases=['seat', 'hold', 'verify'])])


def contact(a='rear', b='fruit', f=(.1, 0., 0., 0., 0., 0.), active=True):
    return dict(geoms=[a,b], force6=list(f), dist_m=.0001, active=active)


@pytest.mark.parametrize('wire', ['front', 'rear'])
@pytest.mark.parametrize('phase', ['insert', 'seat', 'hold', 'verify'])
def test_authorized_wire_and_phase(wire, phase):
    assert tc.category(contact(wire), S, [phase]) == 'target_fruit_touch'
    assert tc.category(contact('fruit', wire), S, [phase]) == 'target_fruit_touch'


@pytest.mark.parametrize('a,b,phases', [
    ('mount','fruit',['seat']), ('arm','fruit',['seat']), ('rear','other_fruit',['seat']),
    ('rear','rachis',['seat']), ('rear','gutter',['seat']), ('rear','stem',['seat']),
    ('rear','fruit',['preapproach']), ('rear','fruit',['approach','insert']),
    ('front','pedicel',['seat']), ('rear','pedicel',['insert','seat']), ('rear','fruit',[])])
def test_other_contacts_remain_forbidden(a,b,phases):
    assert tc.category(contact(a,b), S, phases) == 'forbidden_contact'


def test_target_fruit_is_never_pedicel_capture():
    assert tc.category(contact(), S, ['seat']) != 'target_pedicel_contact'
    assert tc.category(contact(b='pedicel'), S, ['seat']) == 'target_pedicel_contact'


def test_policy_is_exact_and_does_not_mutate_input():
    raw = base(); original = copy.deepcopy(raw)
    p = tc.trial_policy(raw, S, ['preapproach','approach','insert','seat','hold','verify'])
    assert raw == original and p.clearance_m == 0.
    assert ep.pair_allowed(p,'front','fruit',['insert','seat'])
    assert not ep.pair_allowed(p,'mount','fruit',['seat'])
    assert not ep.pair_allowed(p,'rear','fruit',['approach','insert'])
    assert ep.pair_allowed(p,'rear','pedicel',['seat'])
    assert not ep.pair_allowed(p,'rear','other_fruit',['seat'])


def test_policy_cannot_hide_prior_unsafe_exception():
    raw = base(); raw['allowed_contacts'].append(dict(robot_geom='mount',environment_geom='fruit',phases=['seat']))
    with pytest.raises(ValueError): tc.trial_policy(raw,S,['insert','seat'])


def test_absent_phases_are_trimmed_not_silently_added():
    p = tc.trial_policy(base(),S,['insert','seat'])
    assert all(set(r['phases']) <= {'insert','seat'} for r in p.allowed_contacts)
    with pytest.raises(ValueError): tc.trial_policy(base(),S,['ready'])
    with pytest.raises(ValueError): tc.trial_policy(dict(clearance_m=0.),S,['seat'])


def test_force_sums_do_not_cancel_and_tangent_counts():
    rows = [contact(f=(.3,.4,0,0,0,0)),contact(f=(.3,-.4,0,0,0,0))]
    stats = tc.contact_stats(rows,S,['seat'])
    assert stats['target_fruit_touch']['force_sum_N'] == pytest.approx(1.)
    assert stats['forbidden_contact']['force_sum_N'] == 0.
    assert tc.stop_reasons([stats],.001,0.,True,tc.TrialLimits(.9,.005)) == ['target_force_limit']


def test_contact_records_not_force_thresholded():
    rows = [contact(f=(0,0,0,0,0,0),active=False),contact(f=(1e-6,0,0,0,0,0))]
    stats = tc.contact_stats(rows,S,['seat'])['target_fruit_touch']
    assert stats['records']==2 and stats['active_records']==1 and stats['force_sum_N']==pytest.approx(1e-6)


@pytest.mark.parametrize('bad', [0.,-.1,float('nan'),float('inf'),True])
def test_explicit_positive_limits(bad):
    with pytest.raises(ValueError): tc.TrialLimits(bad,.005)
    with pytest.raises(ValueError): tc.TrialLimits(1.,bad)


def test_physics_penetration_not_waived_for_authorized_fruit():
    stats=tc.contact_stats([contact()],S,['seat'])
    assert tc.stop_reasons([stats],.001,.000501,True,tc.TrialLimits(1.,.005))==['physics_penetration']
    assert tc.stop_reasons([stats],.006,0.,True,tc.TrialLimits(1.,.005))==['target_displacement_limit']


def test_live_and_private_are_not_summed_as_one_time():
    stats=tc.contact_stats([contact(f=(.6,0,0,0,0,0))],S,['seat'])
    assert tc.stop_reasons([stats,stats],0.,0.,True,tc.TrialLimits(1.,.005))==[]


def test_non_target_force_blocks_even_after_authorized_touch():
    stats=tc.contact_stats([contact(),contact('mount','fruit',f=(.011,0,0,0,0,0))],S,['seat'])
    assert tc.stop_reasons([stats],0.,0.,True,tc.TrialLimits(1.,.005))==['forbidden_force']


def test_no_loosening_forbidden_floor_or_malformed_measurements():
    with pytest.raises(ValueError): tc.TrialLimits(1.,.005,.02)
    with pytest.raises(ValueError): tc.contact_stats([contact(f=(float('nan'),0,0,0,0,0))],S,['seat'])
    with pytest.raises(ValueError): tc.stop_reasons([],0.,0.,True,tc.TrialLimits(1.,.005))


def test_phase_boundary_conservative():
    trace=[{'phase':'approach'},{'phase':'insert'},{'phase':'seat'}]
    phases=tc.phase_window(trace,0.,1/60)
    assert tc.category(contact(),S,phases)=='forbidden_contact'
    assert tc.category(contact(),S,tc.phase_window(trace,1/60,2/60))=='target_fruit_touch'


@pytest.mark.parametrize('status,passed,complete,requested,expected', [
    ('blocked',False,True,True,False),('inconclusive',False,False,True,False),
    ('sampled_clear',True,False,True,False),('sampled_clear',True,True,False,False),
    ('sampled_clear',True,True,True,True),('blocked',True,True,True,False)])
def test_only_complete_conditional_audit_can_execute(status,passed,complete,requested,expected):
    assert tc.execution_permitted(dict(status=status,passed=passed,complete=complete),requested) is expected


def test_identity_not_wildcards():
    with pytest.raises(ValueError): tc.ContactScope('*',('rear',),('ped',),('rear',))
    with pytest.raises(ValueError): tc.ContactScope('fruit',('rear',),('fruit',),('rear',))


def test_cli_help_is_available_without_server():
    r=subprocess.run([sys.executable,str(Path(tc.__file__)),'--help'],capture_output=True,text=True)
    assert r.returncode==0 and '--execute' in r.stdout


def test_later_non_target_obstacle_is_not_hidden_by_fruit_permission():
    class Scene:
        inventory={};version='analytic-test-only';robot_names=['rear','front']
        environment_names=['fruit','pedicel','stem'];joint_steps=np.array([.01])
        robot_radii=np.array([.02,.02]);environment_radii=np.array([.02,.02,.02])
        environment_positions=np.array([[.3,0,0],[10,0,0],[.8,0,0]])
        def set_robot(self,q):self.robot_positions=np.array([[q[0],0,0],[q[0],1,0]])
        def distance(self,i,j,limit):
            d=np.linalg.norm(self.robot_positions[i]-self.environment_positions[j])-.04
            return min(d,limit),np.r_[self.robot_positions[i],self.environment_positions[j]]
    raw=base();p=tc.trial_policy(raw,S,['seat'])
    strict=ep.screen(Scene(),[[0.],[1.]],['seat','seat'],1/60,ep.Policy.from_dict(raw|{'allowed_contacts':[dict(raw['allowed_contacts'][0],phases=['seat'])]}))
    conditional=ep.screen(Scene(),[[0.],[1.]],['seat','seat'],1/60,p,collect_all_violations=True)
    assert strict['first_violation']['environment_geom']=='fruit'
    assert conditional['status']=='blocked' and conditional['complete']
    assert conditional['first_violation']['environment_geom']=='stem'
