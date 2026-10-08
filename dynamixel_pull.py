"""Bounded XM430-W350 / XL430-W250 velocity controller, used by run_test.py.

Checks the selected model against its ping result before writing registers.
No motor commands are sent just by importing this module.
"""
import time
from mechanics import signed

# Official ROBOTIS control tables. Common motion/watchdog addresses for these models.
MODELS = {
    "XM430-W350": {"number": 1020, "effort_kind": "current", "effort_scale": 2.69},
    "XL430-W250": {"number": 1060, "effort_kind": "load_estimate", "effort_scale": 0.1},
}


class DynamixelPull:
    def __init__(self, config):
        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
        self.config = config
        self.profile = MODELS.get(config["model"])
        self.port = PortHandler(config["port"])
        self.packet = PacketHandler(2.0)
        self.success = COMM_SUCCESS
        self.id = config["id"]
        self.armed = False
        self.touched = False
        self.path_deg = 0.0
        self.initial_position = self.previous_position = None
        self.info = {}

    def check(self, comm, error):
        if comm != self.success:
            raise RuntimeError(self.packet.getTxRxResult(comm))
        if error:
            raise RuntimeError(self.packet.getRxPacketError(error))

    def read(self, address, size):
        value, comm, error = getattr(self.packet, f"read{size}ByteTxRx")(self.port, self.id, address)
        self.check(comm, error)
        return value

    def write(self, address, size, value):
        comm, error = getattr(self.packet, f"write{size}ByteTxRx")(
            self.port, self.id, address, value & ((1 << (8 * size)) - 1))
        self.check(comm, error)

    def open(self):
        if not self.port.openPort():
            raise RuntimeError("Cannot open Dynamixel port")
        if not self.port.setBaudRate(self.config["baudrate"]):
            raise RuntimeError("Cannot set Dynamixel baud rate")
        number, comm, error = self.packet.ping(self.port, self.id)
        self.check(comm, error)
        if not self.profile or number != self.profile["number"]:
            raise RuntimeError(f"Model mismatch: configured {self.config['model']}, ping returned {number}; no motor writes made")
        firmware = self.read(6, 1)
        if firmware < 38:
            raise RuntimeError("Firmware >=38 required for bus watchdog")
        if self.read(64, 1):
            raise RuntimeError("Motor already has torque enabled; disable it before this test")
        self.info = {"model_number": number, "firmware": firmware,
                     "drive_mode": self.read(10, 1), "original_mode": self.read(11, 1)}
        if self.info["drive_mode"] & 8:
            raise RuntimeError("Disable Torque On by Goal Update in Wizard before testing")
        return self.info

    def start(self):
        # Zero old goal before enabling torque. Watchdog stops on lost host traffic.
        self.touched = True
        self.write(98, 1, 0)
        self.write(11, 1, 1)  # velocity mode (EEPROM; left in this mode after test)
        self.write(104, 4, 0)
        self.write(98, 1, 10)  # 200 ms; loop normally updates every 50 ms
        self.write(64, 1, 1)
        self.armed = True
        self.initial_position = self.previous_position = signed(self.read(132, 4), 32)
        raw_speed = round(self.config["rpm"] / 0.229) * self.config["cw_sign"]
        self.write(104, 4, raw_speed)
        self.info["commanded_rpm"] = raw_speed * 0.229

    def telemetry(self):
        # One packet: model-specific effort(126), velocity(128), position(132).
        data, comm, error = self.packet.readTxRx(self.port, self.id, 126, 10)
        self.check(comm, error)
        effort = signed(int.from_bytes(bytes(data[0:2]), "little"), 16) * self.profile["effort_scale"]
        current = effort if self.profile["effort_kind"] == "current" else None
        load = effort if self.profile["effort_kind"] == "load_estimate" else None
        velocity = signed(int.from_bytes(bytes(data[2:6]), "little"), 32) * 0.229
        position = signed(int.from_bytes(bytes(data[6:10]), "little"), 32)
        hardware_error = self.read(70, 1)
        watchdog = self.read(98, 1)
        if hardware_error or watchdog == 255 or not self.read(64, 1):
            raise RuntimeError(f"Motor fault: hardware={hardware_error}, watchdog={watchdog}")
        delta = signed((position - self.previous_position) & 0xFFFFFFFF, 32)
        self.path_deg += abs(delta) * 360 / 4096
        self.previous_position = position
        displacement_deg = (position - self.initial_position) * 360 / 4096
        mm_rev = self.config.get("mm_per_motor_rev")
        return {"motor_rx_monotonic_s": time.monotonic(), "motor_position_counts": position,
                "motor_displacement_deg": displacement_deg, "motor_path_deg": self.path_deg,
                "motor_velocity_rpm": velocity, "motor_current_mA": current, "motor_load_est_percent": load,
                "cart_displacement_est_mm": displacement_deg / 360 * mm_rev if mm_rev else None}

    def stop(self):
        # Independent attempts: torque-off is still attempted if zero-velocity fails.
        errors = []
        if self.touched:
            for address, size, value in ((104, 4, 0), (64, 1, 0)):
                try:
                    self.write(address, size, value)
                except Exception as exc:
                    errors.append(str(exc))
        self.armed = False
        self.port.closePort()
        if errors:
            print("STOP communication failed: " + "; ".join(errors) +
                  ". Watchdog should stop motion after 200 ms without traffic; check hardware.")
        return errors
