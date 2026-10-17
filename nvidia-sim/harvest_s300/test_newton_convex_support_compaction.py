"""CPU-only tests: exact current support, transformed frames and physical fields."""
import hashlib,json,tempfile,unittest
from pathlib import Path
import mujoco as mj
import numpy as np
from newton_convex_support_compaction import prepare,support_vertex_proof,_geom_support
from newton_fruit_proxy import _write_obj
from newton_inputs import sha256


def fixture(root):
    root.mkdir();vertices=[];faces=[]
    localfaces=np.array([[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]])
    for lo,hi in [((-1,-.2,0),(-.5,.2,1)),((.5,-.2,0),(1,.2,1)),((-.5,-.2,0),(.5,.2,.3))]:
        x,y,z=lo;X,Y,Z=hi;v=np.array([[x,y,z],[X,y,z],[X,Y,z],[x,Y,z],[x,y,Z],[X,y,Z],[X,Y,Z],[x,Y,Z]])
        faces.extend((localfaces+len(vertices)).tolist());vertices.extend(v.tolist())
    _write_obj(root/'cad.obj',np.array(vertices),np.array(faces))
    xml='''<mujoco><compiler angle="radian" inertiafromgeom="false" fusestatic="false" discardvisual="true"/>
    <asset><mesh name="cad" file="cad.obj" scale="1.2 .7 1.1"/>
    <mesh name="discarded_cad_visual" file="cad.obj"/></asset>
    <worldbody><body name="robot" pos="0 0 .5"><joint name="rail" type="slide" axis="0 1 0" damping=".3"/>
    <inertial pos=".02 .01 .03" mass="2" diaginertia=".2 .3 .4"/>
    <geom name="active_cad" type="mesh" mesh="cad" pos=".12 -.04 .1" quat=".9238795325 0 0 .3826834324" friction=".6 .01 .002" margin=".0001"/>
    <site name="unchanged_site" pos=".1 .2 .3"/></body>
    <body name="visual_leaf"><geom name="hidden_cad" type="mesh" mesh="discarded_cad_visual" contype="0" conaffinity="0" margin=".0007"/></body>
    <body name="ball" pos="2 0 1"><inertial pos="0 0 0" mass=".1" diaginertia=".01 .01 .01"/>
    <geom name="ball" type="sphere" size=".03"/></body></worldbody>
    <actuator><position name="motor" joint="rail" kp="100"/></actuator></mujoco>'''
    (root/'input_scene.xml').write_text(xml);m=mj.MjModel.from_xml_path(str(root/'input_scene.xml'));d=mj.MjData(m);mj.mj_forward(m,d);mj.mj_saveModel(m,str(root/'input_model.mjb'))
    np.savez_compressed(root/'initial_state.npz',qpos=d.qpos,qvel=d.qvel,eq_active=d.eq_active,ctrl=d.ctrl)
    np.savez_compressed(root/'commands.npz',command=np.array([[.1],[.2]]))
    (root/'source_export.json').write_text(json.dumps(dict(geometry=[dict(name='active_cad')],robot=dict(geometry=[]),attachments=[],training_eligible=False)))
    names=['input_scene.xml','input_model.mjb','initial_state.npz','commands.npz','source_export.json','cad.obj']
    (root/'inputs.json').write_text(json.dumps(dict(input_files={n:dict(sha256=sha256(root/n)) for n in names})))
    return m


class CompactionTests(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.source=self.root/'source';self.old=fixture(self.source)
    def tearDown(self):self.tmp.cleanup()
    def test_all_current_vertices_and_transformed_support_are_preserved(self):
        before={p.name:sha256(p) for p in self.source.iterdir()};out=self.root/'compact';proof=prepare(self.source,out)
        self.assertTrue(proof['body_mass_inertia_joint_eq_material_invariants']['passed'])
        self.assertEqual(proof['source_counts']['ngeom'],proof['variant_counts']['ngeom'])
        self.assertLess(proof['variant_counts']['nmeshvert'],proof['source_counts']['nmeshvert'])
        self.assertLess(proof['variant_counts']['nmeshface'],proof['source_counts']['nmeshface'])
        self.assertEqual(proof['discarded_xml_visual_geom_names'],['hidden_cad'])
        new=mj.MjModel.from_binary_path(str(out/'input_model.mjb'))
        for name in ['body_mass','body_inertia','body_ipos','body_iquat','body_margin','dof_damping','actuator_gainprm','geom_friction','geom_margin']:
            np.testing.assert_array_equal(getattr(self.old,name),getattr(new,name))
        record=support_vertex_proof(_geom_support(self.old,0),_geom_support(new,0),1e-6)
        self.assertTrue(record['passed']);self.assertLess(record['all_direction_support_error_bound_m'],1e-6)
        self.assertEqual(before,{p.name:sha256(p) for p in self.source.iterdir()})
        self.assertEqual(sha256(self.source/'initial_state.npz'),sha256(out/'initial_state.npz'))
        self.assertEqual(sha256(self.source/'commands.npz'),sha256(out/'commands.npz'))
        self.assertFalse(proof['original_ray_triangle_surface_preserved'])
    def test_cpu_positive_and_negative_contact_distances_match(self):
        out=self.root/'compact';prepare(self.source,out);new=mj.MjModel.from_binary_path(str(out/'input_model.mjb'))
        source=_geom_support(self.old,self.old.geom('active_cad').id);center=source.mean(axis=0)
        for delta in [np.array([.01,.005,.002]),np.array([.1,.1,.1]),np.array([3,0,0])]:
            distances=[]
            for m in [self.old,new]:
                m.body_pos[m.body('ball').id]=center+delta+np.array([0,0,.5]);d=mj.MjData(m);mj.mj_forward(m,d)
                distances.append(mj.mj_geomDistance(m,d,m.geom('active_cad').id,m.geom('ball').id,10,None))
            self.assertAlmostEqual(distances[0],distances[1],delta=1e-6)
        self.assertGreater(distances[0],0)
    def test_symmetric_internal_cpu_distance_is_explicitly_not_equivalent(self):
        out=self.root/'compact';proof=prepare(self.source,out);new=mj.MjModel.from_binary_path(str(out/'input_model.mjb'))
        center=_geom_support(self.old,0).mean(axis=0);distances=[]
        for m in [self.old,new]:
            m.body_pos[m.body('ball').id]=center+np.array([0,0,.5]);d=mj.MjData(m);mj.mj_forward(m,d)
            distances.append(mj.mj_geomDistance(m,d,0,1,10,None))
        # Keep the counterexample: preserving the convex support does not
        # prove identical GJK/EPA initialization after mesh COM recentering.
        self.assertLess(distances[0],-.1);self.assertAlmostEqual(distances[1],0,delta=1e-8)
        self.assertFalse(proof['exact_contact_manifold_or_trajectory_equivalence_claimed'])
    def test_missing_extreme_vertex_and_mutable_source_are_rejected(self):
        cube=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)
        with self.assertRaisesRegex(ValueError,'roundtrip'):support_vertex_proof(cube,cube[:-1],1e-9)
        (self.source/'cad.obj').write_text('modified')
        with self.assertRaisesRegex(ValueError,'SHA256'):prepare(self.source,self.root/'bad')
    def test_nonempty_body_margin_change_is_not_silently_restored(self):
        self.old.body_margin[self.old.body('robot').id]=.002
        mj.mj_saveModel(self.old,str(self.source/'input_model.mjb'))
        manifest=json.loads((self.source/'inputs.json').read_text())
        manifest['input_files']['input_model.mjb']['sha256']=sha256(self.source/'input_model.mjb')
        (self.source/'inputs.json').write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError,'nonempty'):
            prepare(self.source,self.root/'nonempty_margin')

if __name__=='__main__':unittest.main()
