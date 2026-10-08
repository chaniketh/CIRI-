import copy
import json
import math
from pathlib import Path
import sys
import unittest
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mechanics import components, parse_sample, signed
from analyze import interpolate, fit_stiffness
import run_test
from run_test import validate
from dynamixel_pull import DynamixelPull, MODELS


class MechanicsTests(unittest.TestCase):
    def test_unknown_does_not_invent_forces(self):
        self.assertTrue(all(x is None for x in components(4, {}).values()))

    def test_horizontal_contact_at_52_degrees(self):
        result = components(1, {"force_model": "fixture_resultant", "force_direction_deg": 0,
                                "needle_angle_deg": 52.4, "contact_x_mm": 12, "contact_y_mm": 18})
        self.assertAlmostEqual(result["needle_axial_est_N"], 0.610145, places=5)
        self.assertAlmostEqual(result["needle_transverse_est_N"], -0.79229, places=4)
        self.assertAlmostEqual(result["joint_moment_est_Nmm"], -18)

    def test_projection_is_inverted_before_resolving(self):
        result = components(2, {"force_model": "ideal_projection", "sensor_axis_deg": 60,
                                "force_direction_deg": 0, "needle_angle_deg": 0})
        self.assertAlmostEqual(result["resultant_est_N"], 4)
        self.assertAlmostEqual(result["needle_transverse_est_N"], 0)

    def test_projection_singularity_rejected(self):
        with self.assertRaises(ValueError):
            components(2, {"force_model": "ideal_projection", "sensor_axis_deg": 90,
                           "force_direction_deg": 0})

    def test_oblique_vector_and_signed_moment(self):
        result = components(10, {"force_model": "fixture_resultant", "force_direction_deg": 90,
                                 "needle_angle_deg": 0, "contact_x_mm": 20, "contact_y_mm": 30})
        self.assertAlmostEqual(result["joint_moment_est_Nmm"], 200)
        self.assertAlmostEqual(result["needle_transverse_est_N"], 10)

    def test_packet_validation_and_signed_adc(self):
        self.assertEqual(parse_sample("2,100,-200,0"), (2,100,-200,False))
        self.assertTrue(parse_sample("3,101,8388607,0")[3])
        self.assertIsNone(parse_sample("# hello"))
        with self.assertRaises(ValueError):
            parse_sample("3,101,99999999,0")
        self.assertEqual(signed(0xFFFFFFFF,32), -1)

    def test_dynamic_geometry_never_extrapolates(self):
        rows = [{"t_s": t, "needle_angle_deg": 50+t*10, "joint_angle_deg": t*10,
                 "contact_x_mm": 12, "contact_y_mm":18, "phase":"loading"}
                for t in (0, 0.2)]
        self.assertAlmostEqual(interpolate(rows,0.1)["needle_angle_deg"],51)
        self.assertIsNone(interpolate(rows,-0.1))
        self.assertIsNone(interpolate(rows,0.3))
        self.assertIsNone(interpolate(rows,0.1,0.05))

    def test_joint_stiffness_fit_requires_measured_loading(self):
        rows = [{"joint_angle_deg": angle, "joint_moment_est_Nmm": 2+100*math.radians(angle),
                 "angle_phase":"loading"} for angle in (0,10,20)]
        self.assertAlmostEqual(fit_stiffness(rows)["apparent_stiffness_Nmm_per_rad"],100)
        for row in rows:
            row["angle_phase"] = "unloading"
        self.assertIsNone(fit_stiffness(rows))

    def test_limits_required_and_direction_verified(self):
        c=json.loads((Path(__file__).resolve().parents[1]/"config.demo.json").read_text())
        validate(c)
        c["limits"]["max_abs_sensor_force_N"]=None
        validate(c)
        c["limits"]["max_abs_sensor_force_N"]=-1
        with self.assertRaises(ValueError):
            validate(c)
        c["limits"]["max_abs_sensor_force_N"]=2
        with self.assertRaises(ValueError):
            validate(c, True)


class CalibrationQualityTests(unittest.TestCase):
    def test_reported_bad_calibration_is_rejected(self):
        cal={"counts_per_N":21217.09341151668,"offset_counts":22469.784964079005,
             "points":[{"force_N":f,"mean_counts":m,"stdev_counts":n} for f,m,n in
                       ((0,23432.17,17.87),(.49033,23472.17,25.75),(.980665,55453.4,27.93),(1.96133,60345.37,32077.71))]}
        q=run_test.calibration_quality(cal)
        self.assertEqual(len(q["issues"]),2)
        self.assertAlmostEqual(q["max_within_load_stdev_equivalent_N"],1.512,places=3)

    def test_stable_linear_calibration_passes_including_negative_slope(self):
        for slope in (-10000,10000):
            cal={"counts_per_N":slope,"offset_counts":20000,
                 "points":[{"force_N":f,"mean_counts":20000+slope*f,"stdev_counts":20} for f in (0,.5,1,2)]}
            self.assertEqual(run_test.calibration_quality(cal)["issues"],[])


class DirectionCheckTests(unittest.TestCase):
    def fake_motor(self, drive_mode=0, fault=False):
        class Motor:
            def __init__(self):
                self.config={}
                self.stopped=False
            def open(self):
                return {"drive_mode":drive_mode}
            def start(self):
                self.sign=self.config["cw_sign"]
            def telemetry(self):
                if fault:
                    raise RuntimeError("test fault")
                return {"motor_path_deg":5.1,"motor_displacement_deg":-5.1}
            def stop(self):
                self.stopped=True
                return []
        return Motor()

    def test_cw_jog_is_bounded_and_preserves_config(self):
        from direction_check import jog
        motor=self.fake_motor()
        config={"motor":{"direction_verified":False,"cw_sign":None}}
        result=jog(config,motor_factory=lambda c:motor)
        self.assertEqual(result,"angle_limit")
        self.assertEqual(motor.sign,-1)
        self.assertTrue(motor.stopped)
        self.assertFalse(config["motor"]["direction_verified"])
        self.assertIsNone(config["motor"]["cw_sign"])

    def test_reverse_drive_mode_changes_command_sign(self):
        from direction_check import jog
        motor=self.fake_motor(drive_mode=1)
        jog({"motor":{}},motor_factory=lambda c:motor)
        self.assertEqual(motor.sign,1)

    def test_fault_always_attempts_stop(self):
        from direction_check import jog
        motor=self.fake_motor(fault=True)
        with self.assertRaises(RuntimeError):
            jog({"motor":{}},motor_factory=lambda c:motor)
        self.assertTrue(motor.stopped)


class TrialSettingsTests(unittest.TestCase):
    def test_metadata_labels_and_angle_are_overridden_without_inventing_depth(self):
        c={"material":"pending", "geometry":{"force_model":"unknown"}, "insertion_depth_mm":None}
        args=SimpleNamespace(material="cork",engagement_level="in_surface",trial_id="cork-in-01",
                             specimen_id="sheet-01",needle_angle_deg=52.4)
        run_test.apply_trial_settings(c,args)
        self.assertEqual(c["material"],"cork")
        self.assertEqual(c["engagement_level"],"in_surface")
        self.assertEqual(c["trial_id"],"cork-in-01")
        self.assertEqual(c["geometry"]["needle_angle_deg"],52.4)
        self.assertIsNone(c["insertion_depth_mm"])
        self.assertEqual(c["geometry"]["force_model"],"unknown")

    def test_omitted_trial_settings_preserve_configuration(self):
        c={"material":"cork","geometry":{"needle_angle_deg":45}}
        before=copy.deepcopy(c)
        run_test.apply_trial_settings(c,SimpleNamespace())
        self.assertEqual(c,before)

    def test_invalid_angle_is_rejected(self):
        with self.assertRaises(ValueError):
            run_test.apply_trial_settings({},SimpleNamespace(needle_angle_deg=float("nan")))


class FakePort:
    def openPort(self):
        return True
    def setBaudRate(self, baud):
        return baud == 57600
    def closePort(self):
        self.closed=True


class MotorTests(unittest.TestCase):
    def make_motor(self):
        m=DynamixelPull.__new__(DynamixelPull)
        m.config={"rpm":1,"cw_sign":-1,"model":"XL430-W250","baudrate":57600}
        m.profile=MODELS[m.config["model"]]
        m.port=FakePort()
        m.touched=m.armed=False
        m.info={}
        m.path_deg=0
        m.initial_position=m.previous_position=None
        m.writes=[]
        m.write=lambda *a: m.writes.append(a)
        m.read=lambda *a: 1000
        return m

    def test_identified_xl430_opens_without_writes(self):
        m=self.make_motor()
        m.id=1
        m.success=0
        m.packet=SimpleNamespace(ping=lambda port, ident:(1060,0,0))
        m.read=lambda address, size: {6:45,64:0,10:0,11:3}[address]
        info=m.open()
        self.assertEqual(info["model_number"],1060)
        self.assertEqual(m.writes,[])

    def test_model_mismatch_rejected_before_writes(self):
        m=self.make_motor()
        m.id=1
        m.success=0
        m.packet=SimpleNamespace(ping=lambda port, ident:(1020,0,0))
        with self.assertRaises(RuntimeError):
            m.open()
        self.assertEqual(m.writes,[])

    def test_xl430_telemetry_is_load_percent_not_current(self):
        m=self.make_motor()
        m.id=1
        m.success=0
        m.initial_position=m.previous_position=1000
        # Signed raw effort=-250 => estimated load=-25%, velocity=-4, pos=900.
        data=list((-250 & 0xFFFF).to_bytes(2,"little") +
                  (-4 & 0xFFFFFFFF).to_bytes(4,"little") + (900).to_bytes(4,"little"))
        m.packet=SimpleNamespace(readTxRx=lambda *args:(data,0,0))
        m.read=lambda address,size: {70:0,98:10,64:1}[address]
        result=m.telemetry()
        self.assertIsNone(result["motor_current_mA"])
        self.assertEqual(result["motor_load_est_percent"],-25)
        self.assertAlmostEqual(result["motor_velocity_rpm"],-0.916)
        self.assertAlmostEqual(result["motor_path_deg"],100*360/4096)

    def test_xm430_telemetry_retains_current_units(self):
        m=self.make_motor()
        m.profile=MODELS["XM430-W350"]
        m.id=1
        m.success=0
        m.initial_position=m.previous_position=1000
        data=list((100).to_bytes(2,"little") + (0).to_bytes(4,"little") + (1000).to_bytes(4,"little"))
        m.packet=SimpleNamespace(readTxRx=lambda *args:(data,0,0))
        m.read=lambda address,size: {70:0,98:10,64:1}[address]
        result=m.telemetry()
        self.assertAlmostEqual(result["motor_current_mA"],269)
        self.assertIsNone(result["motor_load_est_percent"])

    def test_zero_goal_and_watchdog_precede_torque(self):
        m=self.make_motor()
        m.start()
        self.assertEqual(m.writes[:5], [(98,1,0),(11,1,1),(104,4,0),(98,1,10),(64,1,1)])
        self.assertEqual(m.writes[-1],(104,4,-4))
        self.assertTrue(m.armed)
        self.assertEqual(m.initial_position,1000)

    def test_torque_off_attempted_even_if_zero_goal_fails(self):
        m=self.make_motor()
        m.touched=True
        def write(*args):
            m.writes.append(args)
            if args[0]==104:
                raise RuntimeError("disconnected")
        m.write=write
        errors=m.stop()
        self.assertEqual(m.writes,[(104,4,0),(64,1,0)])
        self.assertEqual(len(errors),1)
        self.assertTrue(m.port.closed)

    def test_no_motor_writes_after_read_only_rejection(self):
        m=self.make_motor()
        m.stop()
        self.assertEqual(m.writes,[])
        self.assertTrue(m.port.closed)



class FakeSerial:
    def reset_input_buffer(self):
        pass
    def close(self):
        self.closed=True


class SerialReaderTests(unittest.TestCase):
    def fake(self, chunks):
        device=SimpleNamespace(in_waiting=0)
        device.read=lambda count: chunks.pop(0) if chunks else b""
        return device

    def test_partial_packet_survives_read_timeout(self):
        device=self.fake([b"1,100,",b"-200,0\n"])
        self.assertIsNone(run_test.get_sample(device))
        self.assertEqual(run_test.get_sample(device),(1,100,-200,False))

    def test_multiple_lines_preserve_second_packet(self):
        device=self.fake([b"1,100,200,0\n2,200,300,0\n"])
        self.assertEqual(run_test.get_sample(device),(1,100,200,False))
        self.assertEqual(run_test.get_sample(device),(2,200,300,False))

    def test_flush_during_transmission_discards_only_first_fragment(self):
        device=self.fake([b"12391,4337,0\n140,12482,4350,0\n"])
        device.reset_input_buffer=lambda: None
        run_test.reset_input(device)
        self.assertIsNone(run_test.get_sample(device))
        self.assertEqual(run_test.get_sample(device),(140,12482,4350,False))

    def test_flush_fragment_split_across_reads_resynchronizes(self):
        device=self.fake([b"12391,",b"4337,0\n",b"140,12482,4350,0\n"])
        device.reset_input_buffer=lambda: None
        run_test.reset_input(device)
        self.assertIsNone(run_test.get_sample(device))
        self.assertIsNone(run_test.get_sample(device))
        self.assertEqual(run_test.get_sample(device),(140,12482,4350,False))

    def test_repeated_hx711_error_is_reported(self):
        device=self.fake([b"# ERROR HX711_NOT_READY\n"])
        with self.assertRaisesRegex(RuntimeError,"HX711_NOT_READY"):
            run_test.get_sample(device)


class LoggerTests(unittest.TestCase):
    def run_case(self, samples, expected_exception=None):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            config=json.loads((Path(__file__).resolve().parents[1]/"config.demo.json").read_text())
            config["geometry"]={"force_model":"unknown"}
            config["output_dir"]="runs"
            config["calibration_file"]="cal.json"
            config["limits"]["duration_s"]=1
            (root/"config.json").write_text(json.dumps(config))
            (root/"cal.json").write_text(json.dumps({"counts_per_N":10000,"offset_counts":100000,"force_model":"unknown"}))
            args=SimpleNamespace(config=str(root/"config.json"),pull=False,demo=False,tare=False)
            serial=FakeSerial()
            with patch.object(run_test,"open_uno",return_value=serial), patch.object(run_test,"get_sample",side_effect=samples):
                if expected_exception:
                    with self.assertRaises(expected_exception):
                        run_test.record(args)
                else:
                    run_test.record(args)
            self.assertTrue(serial.closed)
            run=next((root/"runs").iterdir())
            return json.loads((run/"metadata.json").read_text()), (run/"samples.csv").read_text()

    def test_pull_waits_for_setup_after_tare_before_motor_start(self):
        import dynamixel_pull
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            config=json.loads((Path(__file__).resolve().parents[1]/"config.demo.json").read_text())
            config["geometry"]={"force_model":"unknown"}
            config["motor"].update(cw_sign=-1,direction_verified=True)
            config["calibration_file"]="cal.json"
            config["output_dir"]="runs"
            (root/"config.json").write_text(json.dumps(config))
            (root/"cal.json").write_text(json.dumps({"counts_per_N":10000,"offset_counts":100000,"force_model":"unknown"}))
            events=[]
            motor=SimpleNamespace(armed=False,info={})
            motor.open=lambda: events.append("motor_open_readonly") or {}
            def start():
                events.append("motor_start")
                motor.armed=True
            motor.start=start
            motor.telemetry=lambda: {"motor_path_deg":20}
            motor.stop=lambda: events.append("stop") or []
            def respond(prompt):
                self.assertFalse(motor.armed)
                events.append("setup_prompt" if "Engage material" in prompt else "tare_prompt")
                return ""
            def tare(device):
                events.append("tare_capture")
                return 100000,10
            args=SimpleNamespace(config=str(root/"config.json"),pull=True,demo=False,tare=True)
            with patch.object(run_test,"open_uno",return_value=FakeSerial()), \
                 patch.object(run_test,"capture",side_effect=tare), \
                 patch.object(run_test,"get_sample",side_effect=[(i,100*i,100000,False) for i in range(5)]), \
                 patch.object(dynamixel_pull,"DynamixelPull",return_value=motor), \
                 patch("builtins.input",side_effect=respond):
                run_test.record(args)
            self.assertEqual(events,["tare_prompt","tare_capture","motor_open_readonly","setup_prompt","motor_start","stop"])

    def test_force_limit_preserves_triggering_sample(self):
        meta,csv_text=self.run_case([(0,10,130000,False)])
        self.assertEqual(meta["stop_reason"],"sensor_force_limit")
        self.assertEqual(meta["samples"],1)
        self.assertIn("130000",csv_text)

    def test_adc_saturation_stops_and_records_reason(self):
        meta,_=self.run_case([(0,10,8388607,True)],RuntimeError)
        self.assertIn("saturation",meta["stop_reason"])

    def test_uno_reset_or_missing_packet_stops(self):
        meta,_=self.run_case([(10,100,100000,False),(0,10,100000,False)],RuntimeError)
        self.assertIn("Uno reset or sample loss",meta["stop_reason"])
        self.assertEqual(meta["samples"],1)


if __name__ == "__main__":
    unittest.main()
