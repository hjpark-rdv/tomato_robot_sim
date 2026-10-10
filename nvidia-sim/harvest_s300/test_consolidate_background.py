"""Offline source/physics preservation checks; no native simulation imports."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from consolidate_background import (build, enabled, inventory, motion_spec_snapshot,
                                     physical_snapshot, sha, world)


def fixture(directory):
    source, cache = Path(directory) / 'source', Path(directory) / 'cache'
    source.mkdir(); cache.mkdir()
    piece = Usd.Stage.CreateNew(str(source / 'piece.usdc'))
    root = UsdGeom.Xform.Define(piece, '/Piece'); piece.SetDefaultPrim(root.GetPrim())
    vertices = []
    for index, label in enumerate(('MainStem', 'Leaf')):
        mesh = UsdGeom.Mesh.Define(piece, '/Piece/' + label)
        v = np.array([[0., 0., index*.25], [.25, 0., index*.25], [0., .25, index*.25]], np.float32)
        vertices.append(v.astype(float))
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v))
        mesh.CreateFaceVertexCountsAttr([3]); mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        mesh.CreateDoubleSidedAttr(True)
        mesh.GetPrim().SetCustomDataByKey('sourceName', label)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    piece.GetRootLayer().Save()
    stage = Usd.Stage.CreateNew(str(source / 'scene.usda'))
    root = UsdGeom.Xform.Define(stage, '/World'); stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, .25); UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdPhysics.SetStageKilogramsPerUnit(stage, .5)
    stage.SetTimeCodesPerSecond(48.); stage.SetFramesPerSecond(24.)
    groups = []
    for index in range(2):
        path = '/World/Greenhouse/Plants/Plant_%02d/Stem' % index
        part = UsdGeom.Xform.Define(stage, path)
        part.AddTranslateOp().Set(Gf.Vec3d(index*2, .3, .4))
        part.AddRotateZOp().Set(index*30.)
        part.GetPrim().GetReferences().AddReference(str(source / 'piece.usdc'))
        part.GetPrim().SetInstanceable(True)
        groups.append(dict(path=path, plant=index, part='Stem', mode='all', geometry='shared'))
    house = UsdGeom.Mesh.Define(stage, '/World/Greenhouse/House/Wall')
    house.CreatePointsAttr([[0,0,0], [1,0,0], [0,1,0]])
    house.CreateFaceVertexCountsAttr([3]); house.CreateFaceVertexIndicesAttr([0,1,2])
    UsdPhysics.CollisionAPI.Apply(house.GetPrim())
    plant = UsdGeom.Xform.Define(stage, '/World/ElasticTarget')
    UsdPhysics.ArticulationRootAPI.Apply(plant.GetPrim())
    for name in ('Stem', 'Fruit'):
        body = UsdGeom.Xform.Define(stage, '/World/ElasticTarget/' + name)
        body.AddTranslateOp().Set(Gf.Vec3d(.1, .2, .3))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(body.GetPrim()); mass.CreateMassAttr(.02)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.01,.02,.03))
        collider = UsdGeom.Sphere.Define(stage, str(body.GetPath()) + '/Collider')
        collider.CreateRadiusAttr(.02); UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    joint = UsdPhysics.Joint.Define(stage, '/World/ElasticTarget/Joints/Spring')
    joint.CreateBody1Rel().SetTargets(['/World/ElasticTarget/Stem'])
    for axis in ('rotX', 'rotY', 'rotZ'):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateStiffnessAttr(1.2); drive.CreateDampingAttr(.03)
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr(-45.); limit.CreateHighAttr(45.)
    harvest = UsdPhysics.FixedJoint.Define(stage, '/World/ElasticTarget/HarvestJoints/Fruit')
    harvest.CreateBody0Rel().SetTargets(['/World/ElasticTarget/Stem'])
    harvest.CreateBody1Rel().SetTargets(['/World/ElasticTarget/Fruit'])
    harvest.CreateBreakForceAttr(3.); harvest.CreateBreakTorqueAttr(.08)
    stage.GetRootLayer().Save()
    np.savez(source / 'skin.npz', dummy=np.zeros(1))
    (source/'motion_manifest.json').write_text(json.dumps(dict(scene_sha256=sha(source/'scene.usda'),
        target=dict(path='/World/ElasticTarget/Fruit'), plant=dict(bodies=[]))))
    v = np.concatenate(vertices); f = np.array([[0,1,2],[3,4,5]], np.int32)
    np.savez(cache/'geometry.npz', shared_v=v, shared_f=f)
    (cache/'geometry.json').write_text(json.dumps(dict(active_plant=22, groups=groups,
        geometry=dict(shared=dict(sha256=hashlib.sha256(v.tobytes()+f.tobytes()).hexdigest(),
            primitive_labels=['MainStem','Leaf'], face_ranges=[[0,1],[1,1]])))))
    return source, cache


class BackgroundTests(unittest.TestCase):
    def test_direct_specs_copy_exact_references_overrides_and_keep_inherited_farm_layered(self):
        with tempfile.TemporaryDirectory() as directory:
            source, cache = fixture(directory); output = Path(directory)/'direct'
            base = Usd.Stage.CreateNew(str(source/'base.usda'))
            UsdGeom.Xform.Define(base, '/World/HarvestRobot/Robot')
            inherited = UsdGeom.Xform.Define(base, '/World/HarvestRobot/Inherited')
            inherited.AddTranslateOp().Set(Gf.Vec3d(1,2,3))
            base.GetRootLayer().Save()
            stage = Usd.Stage.Open(str(source/'scene.usda'))
            stage.GetRootLayer().subLayerPaths = [str(source/'base.usda')]
            robot = UsdGeom.Xformable(stage.OverridePrim('/World/HarvestRobot/Robot'))
            robot.AddTranslateOp().Set(Gf.Vec3d(.4,.5,.6))
            tool = UsdGeom.Xform.Define(stage, '/World/ActuatedTool')
            tool.GetPrim().GetReferences().AddReference(str(source/'piece.usdc'), '/Piece')
            tool.GetPrim().SetCustomDataByKey('sourceIdentity', 'actual-tool')
            scene = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
            scene.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr(9.81)
            stage.GetRootLayer().Save()
            manifest = json.loads((source/'motion_manifest.json').read_text())
            manifest['scene_sha256'] = sha(source/'scene.usda')
            (source/'motion_manifest.json').write_text(json.dumps(manifest))
            original_bytes = (source/'scene.usda').read_bytes()
            expected = motion_spec_snapshot(stage.GetRootLayer())
            audit = build(source, cache, output, author_motion_specs_in_root=True)
            result = Usd.Stage.Open(str(output/'scene.usda'))
            self.assertEqual(expected, motion_spec_snapshot(result.GetRootLayer()))
            self.assertEqual(expected, audit['copied_motion_root_specs'])
            self.assertEqual(len(expected), 4)
            self.assertTrue(audit['author_motion_specs_in_root'])
            self.assertEqual(physical_snapshot(stage), physical_snapshot(result))
            self.assertEqual(original_bytes, (source/'scene.usda').read_bytes())
            self.assertTrue(result.GetPrimAtPath('/World/HarvestRobot/Inherited').IsDefined())
            self.assertIsNone(result.GetRootLayer().GetPrimAtPath('/World/HarvestRobot/Inherited'))
            self.assertEqual(list(result.GetRootLayer().GetPrimAtPath('/World/ActuatedTool').referenceList.prependedItems),
                             list(stage.GetRootLayer().GetPrimAtPath('/World/ActuatedTool').referenceList.prependedItems))
            self.assertEqual(result.GetRootLayer().GetPrimAtPath('/World/HarvestRobot').specifier,
                             Sdf.SpecifierOver)

    def test_exact_triangle_replacement_keeps_source_dynamics_house_and_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            source, cache = fixture(directory); output = Path(directory)/'new'
            original = Usd.Stage.Open(str(source/'scene.usda'))
            snapshot = physical_snapshot(original); source_bytes = (source/'scene.usda').read_bytes()
            audit = build(source, cache, output)
            result = Usd.Stage.Open(str(output/'scene.usda'))
            self.assertEqual(snapshot, physical_snapshot(result))
            self.assertEqual(source_bytes, (source/'scene.usda').read_bytes())
            self.assertEqual(audit['before']['background_plant_colliders'], 4)
            self.assertEqual(audit['after']['background_plant_colliders'], 2)
            self.assertEqual(audit['usd_collision_prototypes'], 1)
            self.assertEqual(inventory(original)['body_paths'], inventory(result)['body_paths'])
            self.assertEqual(UsdGeom.GetStageMetersPerUnit(result), .25)
            self.assertEqual(UsdGeom.GetStageUpAxis(result), 'Z')
            self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(result), .5)
            self.assertEqual(result.GetTimeCodesPerSecond(), 48.)
            self.assertEqual(result.GetFramesPerSecond(), 24.)
            for instance in audit['source_part_instances']:
                for row in instance['triangle_sources']:
                    self.assertFalse(enabled(result.GetPrimAtPath(row['source_collider'])))
                    self.assertTrue(enabled(original.GetPrimAtPath(row['source_collider'])))
                mesh = UsdGeom.Mesh(result.GetPrimAtPath(instance['collider']))
                np.testing.assert_array_equal(mesh.GetFaceVertexIndicesAttr().Get(), [0,1,2,3,4,5])
                self.assertTrue(mesh.GetDoubleSidedAttr().Get())
                self.assertEqual(mesh.GetOrientationAttr().Get(), 'rightHanded')
                self.assertEqual([x['first_triangle'] for x in instance['triangle_sources']], [0,1])
                self.assertEqual([x['source_name'] for x in instance['triangle_sources']], ['MainStem','Leaf'])
            library = Usd.Stage.Open(str(output/'collision_libraries/shared.usdc'))
            self.assertEqual(UsdGeom.GetStageMetersPerUnit(library), .25)
            self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(library), .5)
            self.assertTrue(enabled(result.GetPrimAtPath('/World/Greenhouse/House/Wall')))
            self.assertEqual(result.GetPrimAtPath('/World/ElasticTarget/HarvestJoints/Fruit')
                             .GetAttribute('physics:breakForce').Get(), 3.)
            self.assertFalse(audit['training_eligible'])
            self.assertNotEqual(audit['scene_sha256'], audit['source_scene_sha256'])
            self.assertTrue(any('offset' in x for x in audit['limitations']))

    def test_dynamic_background_and_negative_winding_are_rejected_before_output(self):
        for invalid in ('dynamic', 'negative', 'leftHanded'):
            with self.subTest(invalid=invalid), tempfile.TemporaryDirectory() as directory:
                source, cache = fixture(directory); output = Path(directory)/'new'
                stage = Usd.Stage.Open(str(source/'scene.usda'))
                part = stage.GetPrimAtPath('/World/Greenhouse/Plants/Plant_00/Stem')
                if invalid == 'dynamic':
                    UsdPhysics.RigidBodyAPI.Apply(part)
                elif invalid == 'negative':
                    UsdGeom.Xformable(part).AddScaleOp().Set(Gf.Vec3f(-1,1,1))
                else:
                    asset = Usd.Stage.Open(str(source/'piece.usdc'))
                    UsdGeom.Mesh(asset.GetPrimAtPath('/Piece/Leaf')).CreateOrientationAttr('leftHanded')
                    asset.GetRootLayer().Save()
                stage.GetRootLayer().Save()
                manifest = json.loads((source/'motion_manifest.json').read_text())
                manifest['scene_sha256'] = sha(source/'scene.usda')
                (source/'motion_manifest.json').write_text(json.dumps(manifest))
                message = dict(dynamic='dynamic body', negative='Negative source-part',
                               leftHanded='Non-default source winding')[invalid]
                with self.assertRaisesRegex(ValueError, message): build(source, cache, output)
                self.assertFalse(output.exists())

    def test_changed_cache_cannot_silently_replace_source_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            source, cache = fixture(directory); output = Path(directory)/'new'
            arrays = dict(np.load(cache/'geometry.npz')); arrays['shared_v'][0,0] += .001
            np.savez(cache/'geometry.npz', **arrays)
            metadata = json.loads((cache/'geometry.json').read_text())
            metadata['geometry']['shared']['sha256'] = hashlib.sha256(
                arrays['shared_v'].tobytes()+arrays['shared_f'].tobytes()).hexdigest()
            (cache/'geometry.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError, 'actual USD source triangles'):
                build(source, cache, output)
            self.assertFalse(output.exists())


REAL = Path('/root/docker_share/mujoko_debugging_data/20261010_s300_performance')
MODEL = REAL/'consolidated_background_metre_z_direct_specs'


@unittest.skipUnless((MODEL/'consolidation_audit.json').exists(),
                     'Full legal audit asset is not present')
class FullLegalTests(unittest.TestCase):
    def test_all_authored_motion_root_specs_match_source_without_farm_flattening(self):
        audit = json.loads((MODEL/'consolidation_audit.json').read_text())
        source = Usd.Stage.Open(str(Path(audit['source_assembly'])/'scene.usda'))
        result = Usd.Stage.Open(str(MODEL/'scene.usda'))
        expected = motion_spec_snapshot(source.GetRootLayer())
        self.assertEqual(expected, motion_spec_snapshot(result.GetRootLayer()))
        self.assertEqual(expected, audit['copied_motion_root_specs'])
        self.assertEqual(len(expected), 4)
        self.assertTrue(audit['author_motion_specs_in_root'])
        self.assertEqual(result.GetRootLayer().subLayerPaths,
                         [str(Path(audit['source_assembly'])/'scene.usda')])
        self.assertEqual(result.GetRootLayer().GetPrimAtPath('/World/HarvestRobot').specifier,
                         Sdf.SpecifierOver)
        # The inherited 30,067 source triangle meshes are not copied/flattened.
        for instance in audit['source_part_instances']:
            for row in instance['triangle_sources']:
                spec = result.GetRootLayer().GetPrimAtPath(row['source_collider'])
                self.assertIsNotNone(spec)
                self.assertNotIn('points', spec.properties)

    def test_reopened_overlay_preserves_physics_transforms_and_stage_metrics(self):
        output = MODEL
        audit = json.loads((output/'consolidation_audit.json').read_text())
        source = Usd.Stage.Open(str(Path(audit['source_assembly'])/'scene.usda'))
        result = Usd.Stage.Open(str(output/'scene.usda'))
        self.assertEqual(physical_snapshot(source), physical_snapshot(result))
        self.assertEqual(inventory(result), audit['after'])
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(source), UsdGeom.GetStageMetersPerUnit(result))
        self.assertEqual(UsdGeom.GetStageUpAxis(source), UsdGeom.GetStageUpAxis(result))
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(result), 1.)
        self.assertEqual(UsdGeom.GetStageUpAxis(result), 'Z')
        self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(source), UsdPhysics.GetStageKilogramsPerUnit(result))
        self.assertEqual(source.GetTimeCodesPerSecond(), result.GetTimeCodesPerSecond())
        self.assertEqual(source.GetFramesPerSecond(), result.GetFramesPerSecond())
        a, b = UsdGeom.XformCache(), UsdGeom.XformCache()
        for instance in audit['source_part_instances']:
            path = instance['source_part']
            np.testing.assert_array_equal(world(a, source.GetPrimAtPath(path)), world(b, result.GetPrimAtPath(path)))

    def test_prepared_full_model_inventory_and_every_input_hash(self):
        output = MODEL
        audit = json.loads((output/'consolidation_audit.json').read_text())
        for key, expected in [('body_count',248), ('plant_bodies',235), ('active_colliders',605),
                              ('house_colliders',1096), ('plant_rotational_dofs',612)]:
            self.assertEqual(audit['before'][key], expected)
            self.assertEqual(audit['after'][key], expected)
        self.assertEqual(audit['before']['background_plant_colliders'], 28971)
        self.assertEqual(audit['after']['background_plant_colliders'], 170)
        self.assertEqual(audit['usd_collision_prototypes'], 7)
        self.assertEqual(audit['scene_sha256'], sha(output/'scene.usda'))
        for path, digest in audit['original_inputs_sha256'].items():
            self.assertEqual(sha(path), digest)

    def test_every_library_has_exact_indices_points_and_orientation(self):
        output = MODEL
        audit = json.loads((output/'consolidation_audit.json').read_text())
        arrays = np.load(REAL/'legal_layout_v2/geometry.npz')
        for key in audit['collision_libraries']:
            stage = Usd.Stage.Open(str(output/'collision_libraries'/(key+'.usdc')))
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath('/Library/Collider'))
            np.testing.assert_array_equal(np.asarray(mesh.GetPointsAttr().Get(),np.float32),
                                          arrays[key+'_v'].astype(np.float32))
            np.testing.assert_array_equal(np.asarray(mesh.GetFaceVertexIndicesAttr().Get(),np.int32),
                                          arrays[key+'_f'].reshape(-1))
            self.assertTrue(mesh.GetDoubleSidedAttr().Get())
            self.assertEqual(mesh.GetOrientationAttr().Get(),'rightHanded')
            self.assertEqual(UsdPhysics.MeshCollisionAPI(mesh.GetPrim()).GetApproximationAttr().Get(),'none')
            self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage),1.)
            self.assertEqual(UsdGeom.GetStageUpAxis(stage),'Z')
            self.assertEqual(UsdPhysics.GetStageKilogramsPerUnit(stage),1.)


if __name__ == '__main__': unittest.main()
