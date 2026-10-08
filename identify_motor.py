#!/usr/bin/env python3
"""Read-only Dynamixel ping; no register writes, torque changes, or motion."""
import argparse
from dynamixel_pull import MODELS
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", required=True)
parser.add_argument("--id", type=int, default=1)
parser.add_argument("--baud", type=int, default=57600)
parser.add_argument("--protocol", type=float, choices=[1.0, 2.0], default=2.0)
args = parser.parse_args()
port = PortHandler(args.port)
packet = PacketHandler(args.protocol)
try:
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise RuntimeError("Cannot open port/set baud rate")
    model, comm, error = packet.ping(port, args.id)
    if comm != COMM_SUCCESS or error:
        raise RuntimeError(f"Ping failed: {packet.getTxRxResult(comm)}; {packet.getRxPacketError(error)}")
    print(f"Model number: {model}; ID: {args.id}; protocol: {args.protocol}; baud: {args.baud}")
    name = next((name for name, profile in MODELS.items() if profile["number"] == model), None)
    if name:
        print(f"Supported model: {name}; set motor.model to this name in your config.")
    else:
        print("This model needs a matching verified motor profile before pulling.")
finally:
    port.closePort()
