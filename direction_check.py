#!/usr/bin/env python3
"""Short UNLOADED motor jog; no Uno access and no force recording.

Needle clear of material, calibration weights removed, cart free to move with
clearance before an end stop. CW is viewed from the output shaft end.
Does not mark direction_verified: observe the actual cart movement yourself.
"""
import argparse
import json
from pathlib import Path
import time
from dynamixel_pull import DynamixelPull


def jog(config, direction='cw', motor_factory=DynamixelPull):
    motor_config = dict(config['motor'])
    motor_config['rpm'] = 1.0
    motor = motor_factory(motor_config)
    reason = 'time_limit'
    latest = None
    try:
        info = motor.open()
        cw_sign = 1 if info['drive_mode'] & 1 else -1
        motor.config['cw_sign'] = cw_sign if direction == 'cw' else -cw_sign
        print(f'UNLOADED {direction.upper()} jog: ~1 rpm, max 5 degrees or 2 seconds.', flush=True)
        motor.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            latest = motor.telemetry()
            if latest['motor_path_deg'] >= 5.0:
                reason = 'angle_limit'
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        reason = 'user_interrupt'
    finally:
        errors = motor.stop()
    if errors:
        raise RuntimeError('Stop command failed; check motor state before continuing')
    print('Stopped:', reason)
    if latest:
        print(f"Measured motor displacement: {latest['motor_displacement_deg']:.2f} degrees")
    print('Observe whether the cart moved toward the motor. Configuration was not changed.')
    return reason


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config.json')
    parser.add_argument('--direction', choices=['cw', 'ccw'], default='cw')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    jog(config, args.direction)


if __name__ == '__main__':
    main()
