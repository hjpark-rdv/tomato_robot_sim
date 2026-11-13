"""Conveyor geometry, outlet clearance and accepted-physics replay guards."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from preview_conveyor_bin import conveyor_bin_spec,load_unload_record,unload_frame


class ConveyorTests(unittest.TestCase):
    def test_flush_housing_and_entire_usable_floor(self):
        spec=conveyor_bin_spec();lo,hi=np.array(spec['bounds'])
        self.assertAlmostEqual(hi[2]-lo[2],.20)
        self.assertEqual(spec['support_feet_count'],0)
        self.assertEqual(spec['chassis_gap_m'],0.)
        parts={p['name']:p for p in spec['parts']}
        for name in ('side_0','side_1','front_drive_frame'):
            p=parts[name]
            self.assertAlmostEqual(p['position'][2]-p['dimensions'][2]/2,lo[2])
        belt=parts['belt'];inner_width=.56-2*.012
        self.assertGreater(belt['dimensions'][0]/inner_width,.96)
        self.assertGreater(belt['dimensions'][1]/1.10,.96)
        self.assertEqual(len(spec['roller_y_m']),31)
        self.assertAlmostEqual(spec['roller_z_m']+spec['roller_radius_m'],
                               spec['belt_top_m']-.002)

    def test_belt_reaches_past_fixed_outlet_lip_without_touching_closed_door(self):
        spec=conveyor_bin_spec();parts={p['name']:p for p in spec['parts']}
        frame=parts['front_drive_frame'];belt=parts['belt']
        self.assertLess(belt['position'][1]-belt['dimensions'][1]/2,
                        frame['position'][1]-frame['dimensions'][1]/2)
        for p in spec['gate_parts']:
            bottom=spec['gate_hinge_m'][2]+p['position'][2]-p['dimensions'][2]/2
            self.assertGreaterEqual(bottom,spec['belt_top_m']+.0019)

    def test_opened_gate_clears_the_floor_outlet(self):
        spec=conveyor_bin_spec();rotation=Rotation.from_euler('x',spec['gate_open_angle_rad'])
        for p in spec['gate_parts']:
            half=np.array(p['dimensions'])/2
            corners=np.array(np.meshgrid(*[[-h,h]for h in half])).reshape(3,-1).T
            opened=rotation.apply(corners+np.array(p['position']))+spec['gate_hinge_m']
            self.assertGreater(opened[:,2].min(),spec['belt_top_m']+.09)
        # No accidental filled box blocks the interior in the physics fixture.
        from run_conveyor_unload import fixture
        import mujoco
        with tempfile.TemporaryDirectory()as directory:
            path=Path(directory)/'scene.xml';names=fixture(path,spec)
            model=mujoco.MjModel.from_xml_path(str(path))
            self.assertEqual(len(names),120)
            self.assertEqual(model.nu,32)
            self.assertEqual(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'roller_envelope'),-1)

    def record(self,directory):
        directory=Path(directory);spec=conveyor_bin_spec();state_path=directory/'states.npz'
        np.savez(state_path,time=[0.,2.],tomato_positions=np.zeros((2,1,3)),gate_angle=[0.,-1.9],
            roller_angles=np.zeros((2,31)),belt_distance=[0.,.5],discharged_count=[0,1])
        report=dict(schema='s300-conveyor-unload-gpu-v1',engine=dict(backend='mujoco_warp_cuda'),
            layout=spec,trials=dict(belt_on=dict(valid=True,fruit_count=1,discharged=1),
                                   belt_off=dict(valid=True,fruit_count=1,discharged=0)),
            state_file=state_path.name,state_sha256=hashlib.sha256(state_path.read_bytes()).hexdigest())
        path=directory/'record.json';path.write_text(json.dumps(report))
        return spec,path,report

    def test_valid_record_interpolates_and_clamps(self):
        with tempfile.TemporaryDirectory()as directory:
            spec,path,_=self.record(directory);_,states=load_unload_record(path,spec)
            self.assertAlmostEqual(unload_frame(states,1.)['gate_angle'],-.95)
            self.assertAlmostEqual(unload_frame(states,-1.)['belt_distance'],0.)
            self.assertAlmostEqual(unload_frame(states,3.)['belt_distance'],.5)

    def test_invalid_or_changed_record_is_not_displayed(self):
        with tempfile.TemporaryDirectory()as directory:
            spec,path,report=self.record(directory)
            report['trials']['belt_on']['valid']=False;path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'physical validity'):load_unload_record(path,spec)
            report['trials']['belt_on']['valid']=True;path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError,'differs'):load_unload_record(path,conveyor_bin_spec(.32))
            (Path(directory)/'states.npz').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'SHA'):load_unload_record(path,spec)


if __name__=='__main__':unittest.main()
