"""Synthetic native callback comparison; no simulator/GPU dependency."""
import math
import json
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from motion_contact_buffer import BufferedContactRecorder


def header(a,b,offset,count):
    return SimpleNamespace(collider0=a,collider1=b,contact_data_offset=offset,num_contact_data=count)


def point(separation,impulse):
    return SimpleNamespace(separation=separation,impulse=list(impulse))


def legacy(headers,data,paths,t,phase,hz):
    events=[]
    for h in headers:
        left,right=paths[h.collider0],paths[h.collider1]
        for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
            contact=data[i]
            events.append(dict(t=t,phase=phase,a=left,b=right,separation_m=float(contact.separation),
                               force_n=math.sqrt(sum(float(x)**2 for x in contact.impulse))*hz,
                               impulse_ns=list(contact.impulse)))
    return events


class BufferedContactTests(unittest.TestCase):
    def setUp(self):
        self.paths={1:'/World/ElasticTarget/fruit/collider',2:'/World/Greenhouse/Plants/neighbor',
                    3:'/World/Ground',4:'/World/HarvestRobot/arm/collider',
                    2**63+9:'/World/MotionEnvs/env_1/ActuatedTool/fixed/wire'}
        self.decoded=[]
        def decode(value):self.decoded.append(value);return self.paths[value]
        self.reader=BufferedContactRecorder(decode,240)

    def test_exact_order_values_phases_timestamps_and_zero_impulse_match_legacy(self):
        data=[point(.001,[0,0,0]),point(-.0005,[1e-6,-2e-6,3e-6]),
              point(-.001,[.003,.004,0]),point(-.002,[-.1,.01,.04])]
        headers=[header(1,2,2,2),header(4,3,0,2),header(1,4,0,0)]
        expected=legacy(headers,data,self.paths,0.,'initialization',240)
        self.reader.record(headers,data,t=0.,phase='initialization')
        self.reader.clear_step()
        self.reader.record([header(2**63+9,2,0,1)],data,t=1/240,phase='insert')
        expected+=legacy([header(2**63+9,2,0,1)],data,self.paths,1/240,'insert',240)
        self.assertEqual(self.reader.materialize(),expected)
        self.assertEqual(self.reader.raw_records[-1].collider0,2**63+9)
        self.assertEqual(self.reader.step_contact_count,1)
        self.assertEqual(self.reader.materialize(4),expected[4:])

    def test_per_step_guard_distinguishes_background_and_exact_threshold(self):
        data=[point(-.001,[0,0,0]),point(-.0005,[1,2,3]),point(-.000500001,[0,0,0])]
        headers=[header(2,3,0,1),header(1,2,1,2)]
        self.reader.record(headers,data,t=.1,phase='seat')
        expected=legacy(headers,data,self.paths,.1,'seat',240)
        invalid=[e for e in expected if e['separation_m']<-.0005 and any(
            x in e['a'] or x in e['b'] for x in ('/ActuatedTool/','/HarvestRobot/','/ElasticTarget/'))]
        self.assertEqual(self.reader.step_invalid_events(),invalid)
        self.assertTrue(self.reader.step_has_invalid_contact)
        self.assertEqual(self.reader.step_minimum_separation_m,-.001)
        self.reader.clear_step()
        self.assertEqual(self.reader.step_invalid_events(),[])
        self.assertEqual(self.reader.step_contact_count,0)
        self.assertIsNone(self.reader.step_minimum_separation_m)
        self.assertEqual(self.reader.materialize(),expected)

    def test_callback_mutable_objects_are_copied_and_paths_are_cached(self):
        data=[point(.001,[.01,.02,.03])];headers=[header(1,2,0,1)]
        self.reader.record(headers,data,t=.1,phase='hold')
        data[0].impulse[:]=[9,9,9];data[0].separation=-1
        headers[0].collider0=4
        self.assertEqual(self.reader.raw_records[0].impulse_ns,(.01,.02,.03))
        self.assertEqual(self.reader.raw_records[0].separation_m,.001)
        self.assertEqual(self.reader.raw_records[0].collider0,1)
        self.reader.record([header(1,2,0,1)],data,t=.2,phase='hold')
        self.reader.materialize()
        self.assertEqual(self.decoded,[1,2])

    def test_multiple_callbacks_in_one_step_accumulate_all_violations(self):
        data=[point(-.0006,[0,0,0])]
        self.reader.clear_step()
        self.reader.record([header(1,2,0,1)],data,t=.2,phase='insert')
        self.reader.record([header(4,2,0,1)],data,t=.2,phase='insert')
        self.assertEqual(self.reader.step_contact_count,2)
        self.assertEqual(len(self.reader.step_invalid_events()),2)
        self.assertEqual(self.reader.metadata()['raw_contact_points'],2)

    def test_normal_record_does_not_compute_force_or_build_event_dictionaries(self):
        data=[point(.001,[1,2,3])]
        def unexpected(raw):raise AssertionError('Normal callback formatted a legacy event')
        self.reader._event=unexpected
        self.reader.record([header(1,2,0,1)],data,t=.2,phase='hold')
        self.assertEqual(self.reader.step_invalid_events(),[])

    def test_configuration_and_materialization_bounds_are_explicit(self):
        for hz in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):BufferedContactRecorder(self.paths.__getitem__,hz)
        with self.assertRaises(ValueError):self.reader.materialize(1)

    def test_opt_in_native_contact_frames_copy_position_normal_without_reorientation(self):
        reader=BufferedContactRecorder(self.paths.__getitem__,240,capture_frames=True)
        data=[point(-.0006,[.01,.02,.03])]
        data[0].position=[1.2,-2.3,3.4];data[0].normal=[-.6,.8,0.]
        reader.record([header(1,2,0,1)],data,t=.2,phase='close')
        data[0].position[:]=[9,9,9];data[0].normal[:]=[1,0,0]
        raw=reader.raw_records[0]
        self.assertEqual(raw.position_world_m,(1.2,-2.3,3.4))
        self.assertEqual(raw.normal_world,(-.6,.8,0.))
        event=reader.materialize()[0]
        self.assertEqual(event['position_world_m'],[1.2,-2.3,3.4])
        self.assertEqual(event['normal_world'],[-.6,.8,0.])
        self.assertEqual(reader.step_invalid_events(),[event])
        self.assertTrue(reader.metadata()['capture_frames'])

    def test_frame_capture_off_retains_exact_original_event_schema(self):
        data=[point(.001,[0,0,0])]
        self.reader.record([header(1,2,0,1)],data,t=.2,phase='hold')
        event=self.reader.materialize()[0]
        self.assertEqual(set(event),{'t','phase','a','b','separation_m','force_n','impulse_ns'})
        self.assertEqual(len(self.reader.raw_records[0]),6)
        self.assertFalse(self.reader.metadata()['capture_frames'])

    def test_missing_native_frame_fields_are_explicit_when_opted_in(self):
        reader=BufferedContactRecorder(self.paths.__getitem__,240,capture_frames=True)
        with self.assertRaisesRegex(AttributeError,'ContactData.position/normal unavailable'):
            reader.record([header(1,2,0,1)],[point(.001,[0,0,0])],t=.2,phase='hold')
        self.assertEqual(reader.record_count,0)


class FakeNativeFloat3:
    """Installed binding protocol for CPU tests; SDK proof is separate."""
    def __init__(self,values):
        self.values=list(values);self.index_reads=0;self.slice_reads=0
    def __getitem__(self,index):
        if isinstance(index,slice):
            self.slice_reads+=1
            return [float(v)for v in self.values[index]]
        self.index_reads+=1
        return float(self.values[index])


def bit_values(value):
    """Preserve signedzero/nonfinite binary64 values during equivalence tests."""
    if isinstance(value,float):return struct.pack('>d',value).hex()
    if isinstance(value,dict):return [(key,bit_values(v))for key,v in value.items()]
    if isinstance(value,(tuple,list)):return [bit_values(v)for v in value]
    return value


class NativeTripletSliceTests(unittest.TestCase):
    def setUp(self):
        self.paths={1:'/World/ElasticTarget/fruit/Collider',2:'/World/Background/leaf',
                    3:'/World/House/structure',2**63+17:'/World/ActuatedTool/R/part'}

    def reader(self,fast=True,**options):
        with patch.dict(sys.modules,{'carb':SimpleNamespace(Float3=FakeNativeFloat3)}):
            return BufferedContactRecorder(self.paths.__getitem__,120,
                native_triplet_slice=fast,**options)

    def points(self):
        return [SimpleNamespace(separation=sep,
                    impulse=FakeNativeFloat3(impulse),
                    position=FakeNativeFloat3([1.,-.0,2**-149]),
                    normal=FakeNativeFloat3([-.6,.8,0.]))
                for sep,impulse in [
                    (-.0005,[0.,-.0,2**-149]),
                    (-.0005000000001,[.001,-.002,.003]),
                    (-.1,[-.0,.004,.002]),
                    (.005,[0.,0.,0.])]]

    def test_bit_exact_framed_events_header_order_ranges_and_per_step_guard(self):
        default=self.reader(False,capture_frames=True);fast=self.reader(capture_frames=True)
        # Deliberately nonordered and repeated offsets; uint64 IDs are exact.
        headers=[header(2,3,2,1),header(2**63+17,1,0,3),header(1,2,1,0),header(1,2,1,2)]
        default_data=self.points();fast_data=self.points()
        for reader,data in [(default,default_data),(fast,fast_data)]:
            reader.record(headers,data,t=.125,phase='initialization')
            reader.clear_step()
            reader.record([header(1,2,0,4)],data,t=.125+1/120,phase='close')
        self.assertEqual(bit_values(fast.raw_records),bit_values(default.raw_records))
        self.assertEqual(bit_values(fast.materialize()),bit_values(default.materialize()))
        self.assertEqual(json.dumps(fast.materialize()),json.dumps(default.materialize()))
        self.assertEqual(bit_values(fast.step_invalid_events()),bit_values(default.step_invalid_events()))
        self.assertEqual(fast.step_contact_count,4)
        self.assertEqual(fast.step_minimum_separation_m,default.step_minimum_separation_m)
        self.assertEqual(fast.record_count,default.record_count)
        self.assertEqual(fast.materialize(1,3),default.materialize(1,3))
        self.assertGreater(sum(p.impulse.slice_reads for p in fast_data),0)
        self.assertEqual(sum(p.impulse.index_reads for p in fast_data),0)
        self.assertEqual(sum(p.impulse.slice_reads for p in default_data),0)
        fast.clear_step();default.clear_step()
        self.assertEqual(fast.step_invalid_events(),default.step_invalid_events())
        self.assertIsNone(fast.step_minimum_separation_m)

    def test_native_buffers_and_headers_mutated_after_callback_cannot_change_evidence(self):
        fast=self.reader(capture_frames=True);data=self.points();headers=[header(1,2,0,2)]
        fast.record(headers,data,t=.1,phase='close');before=bit_values(fast.raw_records)
        for p in data:
            p.separation=-9.;p.impulse.values[:]=[9.,9.,9.]
            p.position.values[:]=[8.,8.,8.];p.normal.values[:]=[1.,0.,0.]
        headers[0].collider0=3
        self.assertEqual(bit_values(fast.raw_records),before)

    def test_pair_limits_remain_exact_and_ordered_for_multiple_callbacks(self):
        calls=[]
        def pair_limit(a,b):calls.append((a,b));return .0005 if '/ActuatedTool/'in a else .2
        default=self.reader(False,pair_limit=pair_limit)
        fast=self.reader(pair_limit=pair_limit)
        for r in (default,fast):
            r.clear_step();r.record([header(2**63+17,1,0,3)],self.points(),t=.2,phase='seat')
            r.record([header(1,2,0,3)],self.points(),t=.2,phase='seat')
            r.record([header(2**63+17,1,0,3)],self.points(),t=.2,phase='seat')
        self.assertEqual(bit_values(default.step_invalid_events()),bit_values(fast.step_invalid_events()))
        self.assertEqual(default._pair_limits,fast._pair_limits)
        self.assertEqual(len(calls),4) # exactly2ordered pairs in each recorder
        self.assertEqual(len(fast.step_invalid_events()),4)

    def test_unknown_vector_and_native_subclass_use_legacy_index_float_copy(self):
        class UnknownVector:
            def __getitem__(self,index):
                if isinstance(index,slice):raise AssertionError('Unknown vector was sliced')
                return (1,-0.,2**-149)[index]
        class NativeSubclass(FakeNativeFloat3):pass
        for vec in (UnknownVector(),NativeSubclass([1,-0.,2**-149])):
            p=point(-.001,[0,0,0]);p.impulse=vec;r=self.reader()
            r.record([header(1,2,0,1)],[p],t=.1,phase='hold')
            self.assertEqual(bit_values(r.raw_records[0].impulse_ns),bit_values((1.,-0.,2**-149)))
        self.assertEqual(vec.slice_reads,0)
        self.assertEqual(vec.index_reads,3)

    def test_default_metadata_unchanged_and_opt_in_discloses_pending_native_AB(self):
        default=BufferedContactRecorder(self.paths.__getitem__,120)
        explicit=BufferedContactRecorder(self.paths.__getitem__,120,native_triplet_slice=False)
        self.assertEqual(default.metadata(),explicit.metadata())
        fast=self.reader();md=fast.metadata()
        self.assertEqual({k:v for k,v in md.items()if k!='native_triplet_copy'},default.metadata())
        self.assertEqual(md['native_triplet_copy']['native_prefix_equivalence_validation'],'pending')
        self.assertEqual(md['native_triplet_copy']['contact_points_omitted'],0)

    def test_missing_frame_preserves_same_already_copied_prefix(self):
        reports=[]
        for enabled in (False,True):
            r=self.reader(enabled,capture_frames=True);data=self.points();del data[1].normal
            with self.assertRaisesRegex(AttributeError,'Native ContactData.position/normal unavailable'):
                r.record([header(1,2,0,3)],data,t=.1,phase='close')
            reports.append((bit_values(r.raw_records),bit_values(r.step_invalid_events()),r.step_minimum_separation_m))
        self.assertEqual(reports[0],reports[1])

    def test_nonfinite_values_preserve_original_raw_order_force_and_guard_behavior(self):
        data=[SimpleNamespace(separation=sep,impulse=FakeNativeFloat3(imp),
                position=FakeNativeFloat3([-.0,float('inf'),float('nan')]),
                normal=FakeNativeFloat3([0.,-0.,1.]))
              for sep,imp in [(float('nan'),[float('nan'),0.,1.]),
                              (-float('inf'),[float('inf'),0.,-.0]),
                              (float('inf'),[0.,0.,0.])]]
        outputs=[]
        for enabled in (False,True):
            r=self.reader(enabled,capture_frames=True);r.record([header(1,2,0,3)],data,t=.1,phase='seat')
            outputs.append((bit_values(r.raw_records),bit_values(r.materialize()),
                bit_values(r.step_invalid_events()),bit_values(r.step_minimum_separation_m)))
        self.assertEqual(outputs[0],outputs[1])


if __name__=='__main__':unittest.main()
