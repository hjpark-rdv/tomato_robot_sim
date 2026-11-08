import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import Mock
import numpy as np
from recording_storage import load_array, save_array
from PIL import Image
from pxr import Usd,UsdGeom
from mounted_gt import (classify,authored_color,stable_id,resolve_pixels,visible_objects,
                        frame_durations,write_review,build_catalog,GroundTruthOverlay,
                        GroundTruthReadbackError,owned_gt_snapshot)


class OwnedGroundTruthReadbackTests(unittest.TestCase):
    def bundle(self):
        return dict(data=np.array([[0,7],[7,1]],np.uint32),
                    info=dict(idToLabels={'0':'BACKGROUND','1':'UNLABELLED','7':'/fruit'},nested={'a':[1]}))

    def overlay(self):
        result=object.__new__(GroundTruthOverlay)
        result.annotator=Mock();result.annotator.get_data.return_value=self.bundle()
        result.meshes={'/fruit':123}
        result.objects={123:dict(key='/fruit',class_name='fruit',class_id=1,
            plant='P',truss='T',color_stage='red',red_proxy=True)}
        return result

    def test_read_requests_owned_cpu_array_and_copies_nested_info(self):
        overlay=self.overlay();borrowed=overlay.annotator.get_data.return_value
        snapshot=overlay.read_snapshot(expected_shape=(2,2))
        overlay.annotator.get_data.assert_called_once_with(device='cpu',do_array_copy=True)
        borrowed['data'][:]=99;borrowed['info']['idToLabels']['7']='/changed';borrowed['info']['nested']['a'][0]=9
        np.testing.assert_array_equal(snapshot['data'],[[0,7],[7,1]])
        self.assertEqual(snapshot['info']['idToLabels']['7'],'/fruit')
        self.assertEqual(snapshot['info']['nested']['a'],[1])
        self.assertFalse(np.shares_memory(snapshot['data'],borrowed['data']))

    def test_missing_id_is_retryable_payload_failure_with_owned_full_evidence(self):
        bundle=self.bundle();bundle['data'][0,0]=2139095040
        with self.assertRaises(GroundTruthReadbackError) as caught:
            owned_gt_snapshot(bundle,(2,2))
        error=caught.exception
        self.assertEqual(error.details['missing_label_count'],1)
        self.assertEqual(error.details['missing_label_ids_sample'],[2139095040])
        self.assertTrue(error.details['retryable_readback'])
        json.dumps(error.details,allow_nan=False)
        bundle['data'][:]=0;bundle['info']['idToLabels'].clear()
        self.assertEqual(error.raw[0,0],2139095040)
        self.assertEqual(error.labels['7'],'/fruit')
        self.assertLess(len(str(error)),200)

    def test_log_remains_concise_for_large_invalid_texture(self):
        raw=np.arange(1000,11000,dtype=np.uint32).reshape(100,100)
        with self.assertRaises(GroundTruthReadbackError) as caught:
            owned_gt_snapshot(dict(data=raw,info=dict(idToLabels={})),raw.shape)
        self.assertEqual(caught.exception.details['missing_label_count'],10000)
        self.assertEqual(len(caught.exception.details['missing_label_ids_sample']),16)
        self.assertLess(len(str(caught.exception)),200)

    def test_bad_dtype_shape_or_missing_metadata_is_not_background(self):
        for data in (dict(data=np.zeros((2,2),np.float32),info=dict(idToLabels={})),
                     dict(data=np.zeros((1,2),np.uint32),info=dict(idToLabels={})),
                     dict(data=np.zeros((2,2),np.uint32),info={}),None):
            with self.subTest(data_type=type(data).__name__):
                with self.assertRaises(GroundTruthReadbackError):owned_gt_snapshot(data,(2,2))

    def test_single_channel_and_integer_labels_preserve_one_row(self):
        snapshot=owned_gt_snapshot(dict(data=np.array([[[7],[1]]],np.uint32),
            info=dict(idToLabels={7:'/fruit'})),(1,2))
        self.assertEqual(snapshot['data'].shape,(1,2))

    def test_save_uses_supplied_bundle_without_late_annotator_read(self):
        overlay=self.overlay();snapshot=overlay.read_snapshot((2,2));overlay.annotator.get_data.reset_mock()
        overlay.annotator.get_data.side_effect=AssertionError('Late read would mix exposure bundles')
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)
            result=overlay.save(output,np.full((2,2,3),100,np.uint8),np.ones((2,2),np.float32),
                np.eye(3),np.eye(4),.5,snapshot=snapshot)
            self.assertEqual(result['unresolved_renderer_ids'],0)
            np.testing.assert_array_equal(load_array(output/'renderer_instance.npy'),snapshot['data'])
            self.assertEqual(json.loads((output/'renderer_labels.json').read_text()),snapshot['info']['idToLabels'])
        overlay.annotator.get_data.assert_not_called()

    def test_real_catalog_mismatch_remains_strict_and_preserves_full_diagnostics(self):
        overlay=self.overlay();snapshot=overlay.read_snapshot((2,2));overlay.meshes={}
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)
            with self.assertRaisesRegex(RuntimeError,'GT catalog mapping failed') as caught:
                overlay.save(output,np.full((2,2,3),100,np.uint8),np.ones((2,2),np.float32),
                    np.eye(3),np.eye(4),.5,snapshot=snapshot)
            self.assertNotIsInstance(caught.exception,GroundTruthReadbackError)
            diagnostic=json.loads((output/'gt_mapping_error.json').read_text())
            self.assertEqual(diagnostic['unresolved'],{'7':'/fruit'})
            self.assertFalse(diagnostic['retryable_readback'])
            np.testing.assert_array_equal(load_array(output/'renderer_instance.npy'),snapshot['data'])
            self.assertFalse((output/'instance.npy').exists())


class VisibleGTTests(unittest.TestCase):
    def test_calyx_is_not_fruit_despite_fruit_body_parent(self):
        p='/World/ElasticTarget/Truss_02_Fruit_11/Visual_1'
        self.assertEqual(classify(p,'Calyx_11_Sepal_1'),'calyx')
        self.assertEqual(classify(p,'Fruit_11'),'fruit')
        self.assertEqual(classify(p,'Pedicel_distal_11'),'pedicel')

    def test_authored_color_proxy_and_unknown(self):
        self.assertEqual(authored_color([.7,.05,.02])['color_stage'],'red')
        self.assertEqual(authored_color([.7,.4,.02])['color_stage'],'turning')
        self.assertEqual(authored_color([.05,.3,.02])['color_stage'],'green')
        self.assertFalse(authored_color(None)['red_proxy'])

    def test_visibility_is_pixel_mask_not_bbox_fill(self):
        # Rendered leaf in the fruit's apparent rectangle stays leaf.
        raw=np.array([[8,8,8],[8,9,8],[8,8,8]],np.uint32)
        meshes={'/fruit':123,'/leaf':456,'/hidden':789}
        objects={123:dict(class_id=1),456:dict(class_id=7),789:dict(class_id=1)}
        ids,classes,missing=resolve_pixels(raw,{'8':'/fruit','9':'/leaf','10':'/hidden'},meshes,objects)
        self.assertEqual(int(ids[1,1]),456);self.assertEqual(int(classes[1,1]),7)
        self.assertNotIn(789,ids);self.assertFalse(missing)
        self.assertEqual(stable_id('/fruit'),stable_id('/fruit'))
        other=resolve_pixels(np.array([[18]],np.uint32),{'18':'/fruit'},meshes,objects)[0]
        self.assertEqual(other[0,0],123)  # renderer IDs can change, object ID cannot

    def test_unmapped_renderer_mesh_is_reported(self):
        _,_,bad=resolve_pixels(np.array([[3]],np.uint32),{'3':'/unexpected'},{},{})
        self.assertEqual(bad,{3:'/unexpected'})

    def test_visible_surface_depth_is_not_box_or_gt_center(self):
        ids=np.array([[0,7],[7,7]],np.uint32);depth=np.array([[99.,2.],[np.nan,2.]])
        objects={7:dict(key='/fruit',class_name='fruit',plant='P',truss='T',color_stage='red',red_proxy=True)}
        pose=np.eye(4);pose[:3,3]=[10,20,30]
        row=visible_objects(ids,depth,objects,np.eye(3),pose)[0]
        self.assertEqual(row['visible_pixels'],3);self.assertEqual(row['valid_depth_pixels'],2)
        np.testing.assert_equal(row['visible_surface_median_optical_m'],[2,1,2])
        np.testing.assert_equal(row['visible_surface_median_world_m'],[12,21,32])

    def test_rachis_xy_axis_is_from_visible_depth_projection_and_camera_rotation(self):
        ids=np.zeros((12,20),np.uint32);ids[4:6,2:18]=7
        depth=np.full(ids.shape,2.,np.float32)
        K=np.array([[100.,0.,10.],[0.,100.,6.],[0.,0.,1.]])
        theta=np.deg2rad(31.);c,s=np.cos(theta),np.sin(theta)
        pose=np.eye(4);pose[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];pose[:3,3]=[2.,3.,1.]
        obj={7:dict(key='/plant/Truss_02/Rachis',class_name='rachis',plant='P',truss='Truss_02')}
        row=visible_objects(ids,depth,obj,K,pose)[0]
        axis=np.asarray(row['visible_surface_xy_axis_world'])
        self.assertEqual(axis.shape,(3,));self.assertAlmostEqual(axis[2],0.)
        self.assertAlmostEqual(np.linalg.norm(axis),1.)
        self.assertAlmostEqual(abs(np.dot(axis,[c,s,0.])),1.,places=12)
        self.assertGreater(row['visible_surface_xy_anisotropy'],4.)
        self.assertEqual(row['visible_surface_xy_axis_points'],32)
        self.assertAlmostEqual(row['visible_surface_long_axis_horizontal_fraction'],1.)
        self.assertAlmostEqual(abs(np.dot(row['visible_surface_long_axis_world'],[c,s,0.])),1.,places=12)
        # The original visible-surface medians retain the same calculation.
        np.testing.assert_allclose(row['visible_surface_median_optical_m'],[-.01,-.03,2.],atol=1e-12)
        expected=pose[:3,:3]@np.array([-.01,-.03,2.])+pose[:3,3]
        np.testing.assert_allclose(row['visible_surface_median_world_m'],expected,atol=1e-12)
        self.assertIn('no hidden GT',row['visible_surface_xy_axis_source'])

    def test_rachis_axis_ignores_occluded_outside_and_invalid_depth_pixels(self):
        ids=np.zeros((16,20),np.uint32);ids[7:9,2:18]=7
        ids[7,8:12]=99  # Leaf occludes these branch pixels.
        depth=np.full(ids.shape,2.,np.float32);depth[8,3]=np.nan
        obj={7:dict(key='/rachis',class_name='rachis',plant='P',truss='T'),
             99:dict(key='/leaf',class_name='leaf',plant='P',truss=None)}
        K=np.array([[100.,0.,10.],[0.,100.,8.],[0.,0.,1.]])
        before=visible_objects(ids,depth,obj,K,np.eye(4))
        branch=next(row for row in before if row['instance_id']==7)
        self.assertEqual(branch['visible_surface_xy_axis_points'],27)
        changed=depth.copy();changed[ids!=7]=10000.;changed[7,8:12]=.001
        after=visible_objects(ids,changed,obj,K,np.eye(4))
        other=next(row for row in after if row['instance_id']==7)
        np.testing.assert_allclose(branch['visible_surface_xy_axis_world'],other['visible_surface_xy_axis_world'])
        self.assertEqual(branch['visible_surface_xy_anisotropy'],other['visible_surface_xy_anisotropy'])
        self.assertNotIn('visible_surface_xy_axis_world',next(row for row in before if row['instance_id']==99))

    def test_rachis_heading_is_optional_when_sparse_or_world_xy_has_no_extent(self):
        obj={7:dict(key='/rachis',class_name='rachis',plant='P',truss='T')}
        ids=np.zeros((6,6),np.uint32);ids[0,:]=7
        sparse=visible_objects(ids,np.ones(ids.shape),obj,np.eye(3),np.eye(4))[0]
        self.assertNotIn('visible_surface_xy_axis_world',sparse)
        # With one image column and a camera whose +Y is world Z, all visible
        # points share world XY. A hidden branch direction is not substituted.
        ids=np.full((14,1),7,np.uint32);pose=np.eye(4)
        pose[:3,:3]=[[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]]
        flat=visible_objects(ids,np.ones(ids.shape),obj,np.eye(3),pose)[0]
        self.assertNotIn('visible_surface_xy_axis_world',flat)
        self.assertAlmostEqual(flat['visible_surface_long_axis_horizontal_fraction'],0.)
        self.assertAlmostEqual(abs(flat['visible_surface_long_axis_world'][2]),1.)

    def test_visible_3d_long_axis_reports_near_vertical_projection_ambiguity(self):
        obj={7:dict(key='/rachis',class_name='rachis',plant='P',truss='T')}
        ids=np.full((20,1),7,np.uint32);depth=np.ones(ids.shape)
        K=np.diag([100.,100.,1.])
        # This visible image column lies along camera +Y. Rotate its measured
        # 3D points into a known world direction, with no hidden skeleton input.
        angle=np.deg2rad(77.);c,s=np.cos(angle),np.sin(angle)
        pose=np.eye(4);pose[:3,:3]=[[1.,0.,0.],[0.,c,-s],[0.,s,c]]
        row=visible_objects(ids,depth,obj,K,pose)[0]
        axis=np.asarray(row['visible_surface_long_axis_world'])
        self.assertAlmostEqual(np.linalg.norm(axis),1.)
        self.assertAlmostEqual(abs(np.dot(axis,[0.,c,s])),1.,places=12)
        self.assertAlmostEqual(row['visible_surface_long_axis_horizontal_fraction'],abs(c),places=12)
        self.assertLess(row['visible_surface_long_axis_horizontal_fraction'],np.sin(np.deg2rad(15.)))
        self.assertEqual(len(row['visible_surface_long_axis_variance_world_m2']),3)
        self.assertIn('same visible',row['visible_surface_long_axis_method'])


class FrozenRouteCatalogTests(unittest.TestCase):
    def stage(self,root):
        stage=Usd.Stage.CreateInMemory()
        for name,source in [('Truss_02_Fruit_01/Visual_1','Fruit_01'),
                            ('Truss_02_Fruit_01/Calyx_01','Calyx_01_Sepal_1'),
                            ('Truss_02_Pedicel_distal_01/Visual','Pedicel_distal_01'),
                            ('Truss_02_Rachis_0/Visual','Rachis')]:
            mesh=UsdGeom.Mesh.Define(stage,root+'/'+name)
            mesh.GetPrim().SetCustomDataByKey('sourceName',source)
            mesh.CreateDisplayColorAttr([(0.7,0.05,0.02)])
        return stage

    def catalog(self,folder,stage,active_source,history=()):
        folder=Path(folder);folder.mkdir()
        manifest=dict(plant=dict(root='/World/ElasticTarget',source_plant=active_source,skin_paths=[]),
                      route_checkpoint=dict(outgoing_history=list(history)))
        (folder/'motion_manifest.json').write_text(json.dumps(manifest));np.savez(folder/'skin.npz')
        return build_catalog(stage,folder)

    def test_frozen_fruit_preserves_original_key_class_source_and_instance_id(self):
        source='/World/Greenhouse/Plants/Plant_22';frozen='/World/RouteFrozen/Plant_22'
        before=self.stage('/World/ElasticTarget');after=self.stage(frozen)
        with tempfile.TemporaryDirectory() as folder:
            active_meshes,active_objects=self.catalog(Path(folder)/'active',before,source)
            frozen_meshes,frozen_objects=self.catalog(Path(folder)/'frozen',after,
                '/World/Greenhouse/Plants/Plant_27',[dict(frozen_root=frozen,source_plant=source)])
        for old_path,identity in active_meshes.items():
            new_path=frozen+old_path[len('/World/ElasticTarget'):]
            self.assertEqual(frozen_meshes[new_path],identity)
            old=active_objects[identity];new=frozen_objects[identity]
            for field in ('instance_id','key','class_name','class_id','plant','truss','source_name'):
                self.assertEqual(old[field],new[field],field)
        fruit=frozen_objects[stable_id(source+'/Truss_02/Fruit_01')]
        self.assertEqual(fruit['class_name'],'fruit');self.assertEqual(fruit['truss'],'Truss_02')

    def test_only_declared_frozen_roots_are_treated_as_plants(self):
        source='/World/Greenhouse/Plants/Plant_22';frozen='/World/RouteFrozen/Plant_22'
        stage=self.stage(frozen)
        unknown=UsdGeom.Mesh.Define(stage,'/World/RouteFrozen/Plant_23/Truss_01_Fruit_01/Visual')
        unknown.GetPrim().SetCustomDataByKey('sourceName','Fruit_01')
        with tempfile.TemporaryDirectory() as folder:
            meshes,objects=self.catalog(Path(folder)/'catalog',stage,'/World/Greenhouse/Plants/Plant_27',
                [dict(frozen_root=frozen,source_plant=source)])
        row=objects[meshes[str(unknown.GetPath())]]
        self.assertEqual(row['class_name'],'unknown');self.assertIsNone(row['plant'])
        self.assertEqual(classify('/World/RouteFrozen/Plant_23/Visual','Fruit_01'),'unknown')

    def test_generic_frozen_skin_keeps_unknown_truss_without_invented_mapping(self):
        source='/World/Greenhouse/Plants/Plant_22';frozen='/World/RouteFrozen/Plant_22'
        stage=self.stage(frozen)
        mesh=UsdGeom.Mesh.Define(stage,frozen+'/Visual_Rachis')
        mesh.GetPrim().SetCustomDataByKey('sourceName','Rachis')
        with tempfile.TemporaryDirectory() as folder:
            meshes,objects=self.catalog(Path(folder)/'catalog',stage,'/World/Greenhouse/Plants/Plant_27',
                [dict(frozen_root=frozen,source_plant=source)])
        row=objects[meshes[str(mesh.GetPath())]]
        self.assertEqual(row['class_name'],'rachis');self.assertEqual(row['plant'],source)
        self.assertIsNone(row['truss']);self.assertEqual(row['truss_identity_source'],'unknown')

    def test_inconsistent_frozen_source_identity_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError,'Exact frozen-root'):
                self.catalog(Path(folder)/'catalog',self.stage('/World/RouteFrozen/Plant_22'),
                    '/World/Greenhouse/Plants/Plant_27',[dict(frozen_root='/World/RouteFrozen/Plant_22',
                        source_plant='/World/Greenhouse/Plants/Plant_99')])


class CameraReviewTimingTests(unittest.TestCase):
    def test_variable_intervals_follow_jsonl_order_and_nominal_tail(self):
        rows=[dict(timestamp_s=value) for value in (5.,5.133,5.277,5.5)]
        np.testing.assert_allclose(frame_durations(rows,2),[.133,.144,.223,.5],rtol=0,atol=1e-12)
        self.assertEqual(frame_durations([dict(timestamp_s=2.)],4),[.25])
        self.assertEqual(frame_durations([],2),[])

    def test_invalid_duplicate_or_nonmonotonic_clocks_are_rejected(self):
        for stamps in ([0.,0.],[0.,1.,.5],[-.1],[np.nan],[np.inf],[None],[True]):
            with self.subTest(stamps=stamps):
                with self.assertRaises(ValueError):frame_durations([dict(timestamp_s=x) for x in stamps],2)
        for fps in (0,-1,np.nan,np.inf,True):
            with self.subTest(fps=fps):
                with self.assertRaises(ValueError):frame_durations([],fps)

    @unittest.skipUnless(shutil.which('ffmpeg'),'ffmpeg unavailable')
    def test_encoded_display_holds_previous_exposure_and_never_shows_future_frame(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);rows=[]
            # The last acquisition is only3ns past a10Hz display boundary.
            # Nearest-microsecond demuxer rounding would expose it too early.
            stamps=[5.,5.101,5.200000003]
            # Deliberately nonlexical names prove source JSONL order controls
            # the sequence; the old glob implementation sorted these wrongly.
            for name,stamp,color in zip(('frame_00002','frame_00000',"frame_00001'quoted"),
                                        stamps,((255,0,0),(0,255,0),(0,0,255))):
                directory=output/name;directory.mkdir()
                for data_name in ('instance.npy','semantic.npy','depth.npy'):
                    save_array(directory/data_name,np.zeros((64,64),np.float32))
                Image.new('RGB',(64,64),color).save(directory/'analysis.png')
                derivative=(stamp==stamps[1])
                if derivative:
                    Image.new('RGB',(64,64),color).save(directory/'analysis_region.png')
                    Image.new('RGB',(64,64),(0,0,0)).save(directory/'analysis.png')
                rows.append(dict(timestamp_s=stamp,phase='test',rgb=name+'/rgb.png',
                    gt=dict(analysis='analysis_region.png' if derivative else 'analysis.png',
                        recognition='harvest_region.png' if derivative else 'recognition.png',visible_fruits=1,
                        **(dict(region='region.json',visible_selected_count=1) if derivative else {}))))
            (output/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
            result=write_review(output,2)
            self.assertIsNone(result['video_error'])
            self.assertEqual(result['source_timing'],'variable_acquisition_timestamps')
            self.assertEqual(result['region_annotated_frames'],1)
            page=(output/'index.html').read_text()
            items=json.loads(page.split('const rows=',1)[1].split(';const $=',1)[0])
            self.assertEqual(items[1]['files']['analysis'],'analysis_region.png')
            self.assertEqual(items[1]['files']['recognition'],'harvest_region.png')
            self.assertEqual(items[1]['region'],'region.json')
            self.assertNotIn('수확 영역 선택은 아직 수행하지 않습니다',page)
            self.assertEqual(result['original_timestamps_s'],stamps)
            np.testing.assert_allclose(result['source_frame_durations_s'],[.101,.099000003,.5],rtol=0,atol=1e-12)
            data=subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-threads','1',
                '-i',result['video'],'-f','rawvideo','-pix_fmt','rgb24','-threads','1','-'],
                check=True,capture_output=True).stdout
            frames=np.frombuffer(data,np.uint8).reshape(-1,64,64,3)
            displayed=[int(np.argmax(frame[32,32])) for frame in frames]
            self.assertEqual(displayed,[0,0,1,2,2,2,2,2])
            for i,color in enumerate(displayed):
                self.assertLessEqual(stamps[color]-stamps[0],i/10+1e-12)
            self.assertTrue((output/'analysis.ffconcat').is_file())
            # Regeneration must preserve an old video rather than silently
            # linking it with fresh timing metadata from another encoder.
            with self.assertRaises(FileExistsError):write_review(output,2)


if __name__=='__main__':unittest.main()
