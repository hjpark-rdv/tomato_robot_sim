import copy
import json
import unittest
import numpy as np
from motion_timing import retime_entry


def fixture():
    rows=[('prepare',3.,.044,(0.,0.)),('approach',4.,.096,(0.,0.)),
          ('belt_start_open',1.,.096,(-.05,.05)),('close',6.,.028,(-.05,.05)),
          ('cycle_open',1.,.096,(-.05,.05)),('cycle_reclose',6.,.028,(-.05,.05)),
          ('release_open',1.,.096,(0.,0.)),('withdraw',4.,.096,(0.,0.)),
          ('return_ready',3.,.096,(0.,0.))]
    phases=[(name,duration,np.arange(14,dtype=float).reshape(2,7)+i,gap,*belts)
            for i,(name,duration,gap,belts) in enumerate(rows)]
    metadata=dict(phases=[dict(name=name,duration_s=duration,waypoint_q=q.tolist(),
                              end_gap_m=gap,belt_local_y_m_s=[left,right])
                          for name,duration,q,gap,left,right in phases],
        belt_cycle=dict(start_after_settle_s=7.,duration_s=14.,ramp_seconds=.5,
                        full_open_gap_m=.096,phases=['belt_start_open','close','cycle_open','cycle_reclose']),
        belt_spin=dict(duration_s=2.,surface_speed_m_s=.05,ramp_seconds=.5),
        goal_pose=[[1.,0.,0.,.3]],pitch_deg=0.,yaw_offset_deg=0.)
    return phases,metadata


class MotionTimingTests(unittest.TestCase):
    def test_default_returns_exact_original_objects_and_command_arrays(self):
        phases,metadata=fixture()
        output,result=retime_entry(phases,metadata,1.)
        self.assertIs(output,phases)
        self.assertIs(result,metadata)
        for old,new in zip(phases,output):self.assertIs(new[2],old[2])
        self.assertNotIn('entry_timing',result)

    def test_only_entry_duration_changes_and_belt_window_shifts_without_stretch(self):
        phases,metadata=fixture()
        original=json.dumps(metadata,sort_keys=True)
        output,result=retime_entry(phases,metadata,4.)
        self.assertEqual(json.dumps(metadata,sort_keys=True),original)
        self.assertEqual([p[1] for p in output],[3.,16.,1.,6.,1.,6.,1.,16.,3.])
        for old,new,old_row,new_row in zip(phases,output,metadata['phases'],result['phases']):
            self.assertIs(old[2],new[2])
            self.assertEqual(old[3:],new[3:])
            self.assertEqual({k:v for k,v in old_row.items() if k!='duration_s'},
                             {k:v for k,v in new_row.items() if k!='duration_s'})
            if old[0] not in ('approach','withdraw'):
                self.assertIs(old,new)
                self.assertIs(old_row,new_row)
        self.assertEqual(result['belt_cycle']['start_after_settle_s'],19.)
        for key in metadata['belt_cycle']:
            if key!='start_after_settle_s':self.assertEqual(result['belt_cycle'][key],metadata['belt_cycle'][key])
        self.assertIs(result['belt_spin'],metadata['belt_spin'])
        self.assertIs(result['goal_pose'],metadata['goal_pose'])
        self.assertEqual(result['entry_timing']['total_duration_added_s'],24.)

    def test_phase_sampling_at_same_fraction_preserves_joint_gap_and_belt_command(self):
        phases,metadata=fixture()
        output,_=retime_entry(phases,metadata,8.)
        for old,new in zip(phases,output):
            for fraction in (0.,.1,.5,.9,1.):
                smooth=fraction*fraction*(3-2*fraction)
                np.testing.assert_array_equal(old[2][0]*(1-smooth)+old[2][-1]*smooth,
                                              new[2][0]*(1-smooth)+new[2][-1]*smooth)
                self.assertEqual(old[3:],new[3:])

    def test_invalid_scale_and_phase_mapping_fail_before_inputs_change(self):
        phases,metadata=fixture()
        for scale in (True,False,0.,.99,8.01,float('nan'),float('inf'),'4'):
            with self.subTest(scale=scale),self.assertRaisesRegex(ValueError,'scale'):
                retime_entry(phases,metadata,scale)
        for kind in ('duration','name','missing_withdraw','duplicate_approach','bad_cycle'):
            changed_phases=list(phases);changed_metadata=copy.deepcopy(metadata)
            if kind=='duration':changed_metadata['phases'][1]['duration_s']+=1.
            elif kind=='name':changed_metadata['phases'][1]['name']='insert'
            elif kind=='missing_withdraw':
                changed_phases[7]=('hold',*changed_phases[7][1:]);changed_metadata['phases'][7]['name']='hold'
            elif kind=='duplicate_approach':
                changed_phases[0]=('approach',*changed_phases[0][1:]);changed_metadata['phases'][0]['name']='approach'
            else:changed_metadata['belt_cycle']['start_after_settle_s']=float('nan')
            before=json.dumps(changed_metadata,sort_keys=True)
            with self.subTest(kind=kind),self.assertRaises(ValueError):retime_entry(changed_phases,changed_metadata,4.)
            self.assertEqual(json.dumps(changed_metadata,sort_keys=True),before)


if __name__=='__main__':unittest.main()
