"""Synthetic native callback comparison; no simulator/GPU dependency."""
import math
from types import SimpleNamespace
import unittest

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


if __name__=='__main__':unittest.main()
