import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from web_rig import Rig


class WebRigTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        config=json.loads((Path(__file__).resolve().parents[1]/'config.example.json').read_text())
        config['output_dir']='runs';config['calibration_file']='cal.json'
        config['limits']['max_abs_sensor_force_N']=0.8
        (self.root/'config.json').write_text(json.dumps(config))
        (self.root/'cal.json').write_text(json.dumps({'counts_per_N':10000,'offset_counts':100000}))
        self.rig=Rig(self.root/'config.json',demo=True)
        self.values={'material':'cork','engagement_level':'in_surface','needle_angle_deg':52.4,'trial_id':'cork-01'}

    def tearDown(self):
        self.rig.shutdown();self.temp.cleanup()

    def start(self):
        self.rig.tare();self.rig.start(self.values)

    def sample(self):
        self.rig.last_sample=time.monotonic()-0.11
        self.rig.poll()

    def test_tare_required_and_insertion_is_qualitative(self):
        with self.assertRaises(ValueError):self.rig.start(self.values)
        self.start()
        self.assertEqual(self.rig.meta['config']['engagement_level'],'in_surface')
        self.assertIsNone(self.rig.meta['config']['insertion_depth_mm'])

    def test_finish_saves_force_and_all_trial_fields(self):
        self.start();self.sample()
        run=self.rig.run_dir
        self.rig.finish({'outcome':'held','notes':'first trial'})
        meta=json.loads((run/'metadata.json').read_text())
        self.assertEqual(meta['status'],'finished');self.assertEqual(meta['stop_reason'],'user_finish')
        self.assertEqual(meta['notes'],'first trial');self.assertFalse(self.rig.state['tared'])
        with (run/'samples.csv').open() as f:rows=list(csv.DictReader(f))
        self.assertEqual(rows[0]['material'],'cork');self.assertEqual(rows[0]['needle_angle_deg'],'52.4')
        self.assertEqual(rows[0]['engagement_level'],'in_surface')

    def test_browser_loss_stops_and_keeps_log(self):
        self.start();run=self.rig.run_dir
        self.rig.last_heartbeat=time.monotonic()-4;self.rig.poll()
        self.assertEqual(self.rig.state['phase'],'stopped')
        self.assertEqual(self.rig.meta['stop_reason'],'browser_connection_lost')
        self.assertTrue((run/'samples.csv').exists())
        self.rig.finish({})

    def test_force_limit_preserves_trigger_sample(self):
        self.start();self.rig.config['limits']['max_abs_sensor_force_N']=0.00001
        self.rig.start_time=time.monotonic()-0.5;self.sample()
        self.assertEqual(self.rig.meta['stop_reason'],'sensor_force_limit')
        self.assertGreater(self.rig.meta['samples'],0)

    def test_disabled_force_threshold_records_large_force(self):
        self.rig.config['limits']['max_abs_sensor_force_N'] = None
        self.start()
        self.rig.demo = False
        self.rig.device = type('Device', (), {'close': lambda self: None})()
        with patch('web_rig.get_sample', return_value=(1,1000,120000,False)):
            self.rig.poll()
        self.assertEqual(self.rig.state['phase'], 'running')
        self.assertAlmostEqual(self.rig.state['force_N'], 2.0)
        self.assertGreater(self.rig.meta['samples'], 0)
        self.rig.start_time = time.monotonic()-10
        self.rig.poll()
        self.assertEqual(self.rig.meta['stop_reason'], 'duration_complete')

    def test_time_limit_and_tare_during_run(self):
        self.start()
        with self.assertRaises(ValueError):self.rig.tare()
        self.rig.start_time=time.monotonic()-10;self.rig.poll()
        self.assertEqual(self.rig.meta['stop_reason'],'duration_complete')

    def test_speed_changes_are_bounded(self):
        self.rig.speed({'rpm':2});self.assertAlmostEqual(self.rig.state['rpm'],2.061)
        for rpm in (0,16,float('nan')):
            with self.assertRaises(ValueError):self.rig.speed({'rpm':rpm})
        self.rig.speed({'rpm':15});self.assertAlmostEqual(self.rig.state['rpm'],15.114)

    def test_disabled_travel_stop_still_stops_at_duration(self):
        self.rig.config['limits']['max_motor_path_deg'] = None
        self.start()
        class Motor:
            def telemetry(self):
                return {'motor_path_deg': 1000, 'motor_velocity_rpm': -15.114}
            def stop(self): return []
        self.rig.motor = Motor()
        self.rig.poll()
        self.assertEqual(self.rig.state['phase'], 'running')
        self.rig.start_time = time.monotonic()-10
        self.rig.poll()
        self.assertEqual(self.rig.meta['stop_reason'], 'duration_complete')

    def test_pull_validation_accepts_disabled_travel_and_15_rpm(self):
        from run_test import validate
        config = copy.deepcopy(self.rig.config)
        config['limits']['max_motor_path_deg'] = None
        config['motor']['rpm'] = 15
        config['motor']['direction_verified'] = True
        config['limits']['max_abs_sensor_force_N'] = None
        validate(config, pull=True)
        config['limits']['max_motor_path_deg'] = -1
        with self.assertRaises(ValueError): validate(config, pull=True)

    def test_live_speed_records_event_and_finish_stops_motor(self):
        self.start()
        class Motor:
            def __init__(self):self.writes=[];self.stopped=False
            def read(self,*args):return 100
            def write(self,*args):self.writes.append(args)
            def stop(self):self.stopped=True;return []
        motor=Motor();self.rig.motor=motor
        self.rig.speed({'rpm':2})
        self.assertEqual(motor.writes[0],(104,4,-9))
        self.assertEqual(self.rig.meta['events'][0]['event'],'speed_change')
        self.rig.finish({});self.assertTrue(motor.stopped)

    def test_fault_after_autostop_keeps_finish_available(self):
        self.start();self.rig.stop('motor_travel_limit')
        self.rig.failure(RuntimeError('USB lost'))
        self.assertEqual(self.rig.state['phase'],'stopped');self.rig.finish({})

    def test_bad_labels_rejected_before_motor_or_files(self):
        self.rig.tare()
        for field,value in [('material',''),('engagement_level','random'),('needle_angle_deg',float('nan'))]:
            values=dict(self.values);values[field]=value
            with self.assertRaises(ValueError):self.rig.start(values)
        self.assertIsNone(self.rig.motor);self.assertIsNone(self.rig.run_dir)

    def test_shutdown_stops_and_preserves_unfinished_run(self):
        self.start();run=self.rig.run_dir;self.rig.shutdown()
        meta=json.loads((run/'metadata.json').read_text())
        self.assertEqual(meta['stop_reason'],'server_shutdown')


class StartupFreshnessTests(unittest.TestCase):
    setUp = WebRigTests.setUp
    tearDown = WebRigTests.tearDown
    def prepare_real(self):
        self.rig.tare()
        self.rig.demo=False
        class Device:
            def reset_input_buffer(self):pass
            def close(self):pass
        self.rig.device=Device()
        rig=self.rig
        class Motor:
            def __init__(self, config):self.started=False;self.info={};self.config=config
            def open(self):
                # Simulate the sensor-read pause caused by opening USB motor.
                rig.last_sample=time.monotonic()-1
                return {}
            def start(self):self.started=True
            def stop(self):return []
        self.rig.motor_factory=Motor

    def test_slow_motor_open_gets_fresh_sensor_before_start(self):
        self.prepare_real()
        with patch('web_rig.get_sample',return_value=(100,10000,100010,False)):
            self.rig.start(self.values)
        self.assertTrue(self.rig.motor.started)
        self.assertAlmostEqual(self.rig.state['force_N'],.001)
        self.assertEqual(self.rig.sequence,100)
        self.assertTrue(self.rig.meta['startup_serial_resynchronized'])

    def test_excessive_new_force_prevents_motion(self):
        self.prepare_real()
        with patch('web_rig.get_sample',return_value=(100,10000,120000,False)):
            with self.assertRaisesRegex(RuntimeError,'Force at configured limit'):
                self.rig.start(self.values)
        self.assertFalse(self.rig.motor.started)

    def test_no_new_sensor_packet_prevents_motion(self):
        self.prepare_real()
        self.rig.config['limits']['sensor_stale_s']=.005
        with patch('web_rig.get_sample',return_value=None):
            with self.assertRaisesRegex(RuntimeError,'No fresh HX711'):
                self.rig.start(self.values)
        self.assertFalse(self.rig.motor.started)

    def test_saturated_new_packet_prevents_motion(self):
        self.prepare_real()
        with patch('web_rig.get_sample',return_value=(100,10000,8388607,True)):
            with self.assertRaisesRegex(RuntimeError,'ADC saturated'):
                self.rig.start(self.values)
        self.assertFalse(self.rig.motor.started)
