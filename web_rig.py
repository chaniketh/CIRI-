"""Single-owner USB worker for the local needle-test GUI."""
import copy
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
from dynamixel_pull import DynamixelPull
from serial_ports import find_uno
from mechanics import components, finite
from run_test import FIELDS, calibration_quality, capture, get_sample, open_uno, reset_input, validate


def utc():
    return datetime.now(timezone.utc).isoformat()


class Rig:
    def __init__(self, config_path, demo=False, motor_factory=DynamixelPull):
        self.config_path = Path(config_path).resolve()
        self.config = json.loads(self.config_path.read_text())
        validate(self.config, pull=not demo)
        self.demo = demo
        self.cal = json.loads((self.config_path.parent / self.config['calibration_file']).read_text())
        if not finite(self.cal.get('counts_per_N')) or abs(self.cal['counts_per_N']) < 1 or not finite(self.cal.get('offset_counts')):
            raise ValueError('Invalid calibration constants')
        if self.cal.get('quality_status') == 'rejected' or calibration_quality(self.cal)['issues']:
            raise ValueError('Calibration rejected; recalibrate before using GUI')
        self.root = self.config_path.parent / ('gui_demo_runs' if demo else self.config['output_dir'])
        self.root.mkdir(parents=True, exist_ok=True)
        self.motor_factory = motor_factory
        self.device = self.motor = None
        self.file = self.writer = self.run_dir = self.meta = None
        self.sequence = self.uno_ms = None
        self.uno_elapsed = 0
        self.last_sample = None
        self.last_port_scan = 0
        self.last_motor = 0
        self.motor_data = {}
        self.start_time = None
        self.last_heartbeat = time.monotonic()
        self.demo_seq = 0
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=20)
        self.quit = threading.Event()
        self.state = {'phase': 'idle', 'message': 'Remove load, then tare to connect the Uno.',
                      'force_N': None, 'raw_counts': None, 'tared': False,
                      'rpm': round(self.config['motor']['rpm']/0.229)*0.229,
                      'actual_rpm': None, 'peak_N': 0, 'elapsed_s': 0,
                      'trace': [], 'last_run': None, 'demo': demo,
                      'limits': self.config['limits'], 'uno_port': None, 'uno_port_error': None}
        self.thread = None

    def snapshot(self, heartbeat=True):
        with self.lock:
            if heartbeat:
                self.last_heartbeat = time.monotonic()
            result = copy.deepcopy(self.state)
            result['sample_age_s'] = time.monotonic()-self.last_sample if self.last_sample else None
            return result

    def enqueue(self, action, payload):
        if action not in ('tare', 'start', 'finish', 'speed'):
            raise ValueError('Unknown action')
        self.commands.put_nowait((action, payload))

    def launch(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def detect_uno(self):
        self.last_port_scan = time.monotonic()
        try:
            port = find_uno(self.config.get('uno_port'), self.config.get('uno_serial_number'))
        except RuntimeError as exc:
            self.state.update(uno_port=None, uno_port_error=str(exc))
            return False
        self.config['uno_port'] = port.device
        if port.serial_number:
            self.config['uno_serial_number'] = port.serial_number
        self.state.update(uno_port=port.device, uno_port_error=None)
        return True

    def tare(self):
        if self.state['phase'] not in ('idle', 'ready', 'error'):
            raise ValueError('Finish the current test before taring')
        self.state.update(phase='taring', tared=False, message='Taring. Keep needle clear and fixture still.')
        if self.demo:
            mean, noise = 100000, 20
        else:
            if self.device is None:
                if not self.detect_uno():
                    raise RuntimeError(self.state['uno_port_error'])
                self.device = open_uno(self.config['uno_port'])
            mean, noise = capture(self.device)
            reset_input(self.device)
        self.cal['offset_counts'] = mean
        self.tare_noise = noise
        self.sequence = self.uno_ms = None
        self.last_sample = time.monotonic()
        self.state.update(phase='ready', tared=True, force_N=0, raw_counts=mean,
                          trace=[], message='Tared. Engage material, then start the test.')

    def start(self, values):
        if self.state['phase'] != 'ready' or not self.state['tared']:
            raise ValueError('Tare before starting each test')
        if not self.last_sample or time.monotonic()-self.last_sample > self.config['limits']['sensor_stale_s']:
            raise ValueError('Sensor data stale; check Uno and tare again')
        material = str(values.get('material', '')).strip()
        if not material or len(material) > 160:
            raise ValueError('Enter a material name (up to 160 characters)')
        level = values.get('engagement_level')
        if level not in ('on_surface', 'in_surface', 'deep_into_surface'):
            raise ValueError('Select insertion level')
        try:
            angle = float(values['needle_angle_deg'])
        except (ValueError, KeyError, TypeError):
            raise ValueError('Enter needle angle measured from rail') from None
        if not finite(angle) or not -180 <= angle <= 180:
            raise ValueError('Angle must be between -180 and 180 degrees')
        if self.config['limits'].get('max_abs_sensor_force_N') is not None and abs(self.state['force_N']) >= self.config['limits']['max_abs_sensor_force_N']:
            raise ValueError('Force already at configured limit; unload before starting')
        cfg = copy.deepcopy(self.config)
        cfg.update(material=material, engagement_level=level,
                   trial_id=str(values.get('trial_id', '')).strip()[:100] or datetime.now().strftime('trial-%H%M%S'),
                   notes=str(values.get('notes', ''))[:4000])
        cfg['geometry']['needle_angle_deg'] = angle
        cfg['motor']['rpm'] = self.state['rpm']
        self.state.update(phase='starting', message='Preparing motor…')
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        self.run_dir = self.root / stamp
        self.run_dir.mkdir()
        self.meta = {'config': cfg, 'calibration': copy.deepcopy(self.cal), 'demo': self.demo,
                     'pull': True, 'created_utc': utc(), 'status': 'running', 'samples': 0,
                     'tare_noise_counts': getattr(self, 'tare_noise', None), 'events': [],
                     'stop_reason': None, 'time_reference': 'host monotonic receive time; motor telemetry asynchronous'}
        self.file = (self.run_dir/'samples.csv').open('w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.file.flush()
        self.persist()
        self.motor_data = {}
        self.uno_elapsed = 0
        self.state.update(peak_N=0, elapsed_s=0, trace=[])
        if not self.demo:
            self.motor = self.motor_factory(cfg['motor'])
            self.meta['motor_info'] = self.motor.open()
            # Opening the motor pauses this single-owner worker. Discard only
            # pre-motion serial backlog and demand a newly transmitted packet;
            # do not change the tare or accept old buffered data as fresh.
            self.refresh_before_pull()
            self.start_time = time.monotonic()
            self.motor.start()
            self.meta['motor_info'] = self.motor.info
        else:
            self.start_time = time.monotonic()
        self.last_motor = 0
        self.state.update(phase='running', message='Pulling. Finish test to stop and save.')
        self.persist()

    def refresh_before_pull(self):
        reset_input(self.device)
        deadline = time.monotonic() + self.config['limits']['sensor_stale_s']
        while time.monotonic() < deadline:
            sample = get_sample(self.device)
            if sample is None:
                continue
            seq, ms, raw, saturated = sample
            if saturated:
                raise RuntimeError('ADC saturated before pull; no motion started')
            force = (raw-self.cal['offset_counts'])/self.cal['counts_per_N']
            self.state.update(force_N=force, raw_counts=raw)
            self.last_sample = time.monotonic()
            if self.config['limits'].get('max_abs_sensor_force_N') is not None and abs(force) >= self.config['limits']['max_abs_sensor_force_N']:
                raise RuntimeError('Force at configured limit before pull; no motion started')
            # A startup flush deliberately drops pre-motion packets. Sequence
            # and timestamps remain strictly checked after this new baseline.
            self.sequence, self.uno_ms, self.uno_elapsed = seq, ms, 0
            self.meta['pre_pull_sample'] = {'sequence': seq, 'uno_ms': ms,
                                           'raw_counts': raw, 'sensor_force_N': force}
            self.meta['startup_serial_resynchronized'] = True
            return
        raise RuntimeError('No fresh HX711 reading after motor connection; no motion started')

    def persist(self):
        if self.run_dir and self.meta:
            target = self.run_dir/'metadata.json'
            temp = target.with_suffix('.tmp')
            temp.write_text(json.dumps(self.meta, indent=2))
            temp.replace(target)

    def stop(self, reason):
        # Stop first, before any disk work. Independently attempts velocity zero/torque off.
        errors = self.motor.stop() if self.motor else []
        self.motor = None
        if self.file:
            self.file.flush()
            self.file.close()
        self.file = self.writer = None
        if self.meta:
            self.meta.update(status='awaiting_finish', stop_reason=reason, finished_utc=utc(),
                             peak_abs_sensor_force_N=self.state['peak_N'], stop_errors=errors,
                             motor_final_telemetry=self.motor_data)
            self.persist()
        self.state.update(phase='stopped', actual_rpm=None,
                          message='Stopped: '+reason+'. Click Finish test to add notes and finalize log.')
        if errors:
            self.state['message'] += ' Stop communication failed; check hardware.'

    def finish(self, values):
        if self.state['phase'] not in ('running', 'stopped'):
            raise ValueError('No test to finish')
        if self.state['phase'] == 'running':
            self.stop('user_finish')
        self.meta.update(status='finished', outcome=str(values.get('outcome', 'not_recorded'))[:100],
                         notes=str(values.get('notes', ''))[:4000], finalized_utc=utc())
        self.persist()
        self.state.update(phase='idle', tared=False, last_run=self.run_dir.name,
                          message='Test saved. Tare before the next test.')
        self.run_dir = self.meta = None

    def speed(self, values):
        if self.state['phase'] in ('taring', 'starting', 'stopped'):
            raise ValueError('Finish the current operation before changing speed')
        rpm = float(values['rpm'])
        if not finite(rpm) or not 0.229 <= rpm <= 15.114:
            raise ValueError('Speed must be 0.229–15.114 rpm (approximately 15 rpm)')
        raw = min(66, round(rpm/0.229))
        effective = raw*0.229
        if self.motor:
            if raw > self.motor.read(44, 4):
                raise ValueError('Speed exceeds motor velocity limit')
            self.motor.write(104, 4, raw*self.config['motor']['cw_sign'])
        if self.state['phase'] == 'running':
            self.meta['events'].append({'t_s': time.monotonic()-self.start_time,
                                        'event': 'speed_change', 'requested_rpm': rpm, 'commanded_rpm': effective})
            self.persist()
        self.state['rpm'] = effective

    def poll(self):
        now = time.monotonic()
        running = self.state['phase'] == 'running'
        if not self.demo and self.device is None and now-self.last_port_scan >= 1:
            self.detect_uno()
        if running:
            limits = self.config['limits']
            if now-self.last_heartbeat > 3:
                self.stop('browser_connection_lost'); return
            if now-self.last_sample > limits['sensor_stale_s']:
                self.stop('sensor_stale'); return
            if now-self.start_time >= limits['duration_s']:
                self.stop('duration_complete'); return
            if self.motor and now-self.last_motor >= 0.05:
                self.motor_data = self.motor.telemetry()
                self.last_motor = time.monotonic()
                self.state['actual_rpm'] = self.motor_data['motor_velocity_rpm']
                travel_limit = limits.get('max_motor_path_deg')
                if travel_limit is not None and self.motor_data['motor_path_deg'] >= travel_limit:
                    self.stop('motor_travel_limit'); return
        if not self.device and not (self.demo and self.state['tared']):
            return
        if self.demo:
            if self.last_sample and now-self.last_sample < 0.09:
                return
            force = 0.3*math.sin((now-self.start_time)*math.pi/4) if running else 0
            sample = (self.demo_seq, int(now*1000)&0xFFFFFFFF,
                      round(self.cal['offset_counts']+force*self.cal['counts_per_N']), False)
            self.demo_seq += 1
        else:
            sample = get_sample(self.device)
        if sample is None:
            if self.last_sample and time.monotonic()-self.last_sample > self.config['limits']['sensor_stale_s']:
                raise RuntimeError('Sensor data stale; check Uno connection')
            return
        seq, ms, raw, saturated = sample
        if self.sequence is not None and seq != (self.sequence+1)&0xFFFFFFFF:
            raise RuntimeError('Uno reset or missing sample')
        step = (ms-self.uno_ms)&0xFFFFFFFF if self.uno_ms is not None else 0
        if step > 10000:
            raise RuntimeError('Uno timestamp reset')
        self.sequence, self.uno_ms = seq, ms
        self.uno_elapsed += step
        rx = time.monotonic()
        self.last_sample = rx
        force = (raw-self.cal['offset_counts'])/self.cal['counts_per_N']
        self.state.update(force_N=force, raw_counts=raw)
        if running:
            t = rx-self.start_time
            self.state.update(elapsed_s=t, peak_N=max(self.state['peak_N'], abs(force)))
            self.state['trace'].append([t, force])
            self.state['trace'] = self.state['trace'][-2000:]
            cfg = self.meta['config']
            geo = cfg['geometry']
            row = {'t_s': t, 'host_utc': utc(), 'host_monotonic_s': rx, 'uno_sequence': seq,
                   'uno_ms': ms, 'uno_elapsed_s': self.uno_elapsed/1000, 'raw_counts': raw,
                   'saturated': int(saturated), 'sensor_force_N': force,
                   'force_model': geo.get('force_model', 'unknown'), 'geometry_source': 'fixed_config_assumption',
                   'phase': 'pull'}
            for key in ('material', 'engagement_level', 'trial_id', 'specimen_id', 'needle_id'):
                row[key] = cfg.get(key)
            for key in ('needle_angle_deg', 'contact_x_mm', 'contact_y_mm'):
                row[key] = geo.get(key)
            row.update(components(force, geo))
            row.update(self.motor_data)
            if self.motor_data:
                row['motor_age_s'] = rx-self.motor_data['motor_rx_monotonic_s']
            self.writer.writerow(row)
            self.file.flush()
            self.meta['samples'] += 1
        if saturated:
            raise RuntimeError('HX711 ADC saturation')
        if running and self.config['limits'].get('max_abs_sensor_force_N') is not None and abs(force) >= self.config['limits']['max_abs_sensor_force_N']:
            self.stop('sensor_force_limit')

    def failure(self, exc):
        if self.motor or self.file:
            self.stop('fault: '+str(exc))
        elif self.meta:
            self.state.update(phase='stopped', tared=False, message='Sensor fault: '+str(exc)+'. Finish test to finalize the saved log.')
        else:
            self.state.update(phase='error', tared=False, message=str(exc))
        if self.device:
            self.device.close()
            self.device = None
        self.state['tared'] = False

    def loop(self):
        while not self.quit.is_set():
            try:
                with self.lock:
                    self.poll()
                    try:
                        action, values = self.commands.get_nowait()
                    except queue.Empty:
                        action = None
                    if action:
                        try:
                            getattr(self, action)(values) if action != 'tare' else self.tare()
                        except ValueError as exc:
                            self.state['message'] = str(exc)
            except Exception as exc:
                with self.lock:
                    self.failure(exc)
            self.quit.wait(0.01)

    def shutdown(self):
        self.quit.set()
        if self.thread:
            self.thread.join(timeout=10)
        with self.lock:
            if self.motor or self.file:
                self.stop('server_shutdown')
            if self.device:
                self.device.close()
                self.device = None
