#!/usr/bin/env python3
"""Calibration, logging, and optional bounded motor pulling. Paths relative to config."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time
from datetime import datetime, timezone
from mechanics import components, finite, parse_sample
from dynamixel_pull import MODELS

COMPONENT_KEYS = list(components(0, {}))
FIELDS = ["t_s", "host_utc", "host_monotonic_s", "uno_sequence", "uno_ms",
          "uno_elapsed_s", "raw_counts", "saturated", "sensor_force_N",
          "material", "engagement_level", "specimen_id", "trial_id", "needle_id",
          "force_model", "geometry_source", "needle_angle_deg", "contact_x_mm",
          "contact_y_mm"] + COMPONENT_KEYS + [
          "motor_rx_monotonic_s", "motor_age_s", "motor_position_counts",
          "motor_displacement_deg", "motor_path_deg", "motor_velocity_rpm",
          "motor_current_mA", "motor_load_est_percent", "cart_displacement_est_mm", "phase"]


def load_json(path):
    return json.loads(Path(path).read_text())


def positive(value, name):
    if not finite(value) or value <= 0:
        raise ValueError(f"Set a finite positive {name}")


def validate(config, pull=False):
    limits = config["limits"]
    for key in ("duration_s", "sensor_stale_s"):
        positive(limits.get(key), key)
    if limits.get("max_abs_sensor_force_N") is not None:
        positive(limits["max_abs_sensor_force_N"], "max_abs_sensor_force_N")
    geometry = config.get("geometry", {})
    if geometry.get("needle_angle_deg") is not None and not finite(geometry["needle_angle_deg"]):
        raise ValueError("Needle angle must be finite")
    components(1, geometry)
    model = geometry.get("force_model", "unknown")
    if model != "unknown" and not finite(geometry.get("force_direction_deg")):
        raise ValueError("Set force_direction_deg before calculating forces")
    if model == "ideal_projection" and not finite(geometry.get("sensor_axis_deg")):
        raise ValueError("Set sensor_axis_deg for ideal_projection")
    if pull:
        motor = config["motor"]
        if limits.get("max_motor_path_deg") is not None:
            positive(limits["max_motor_path_deg"], "max_motor_path_deg")
        positive(motor.get("rpm"), "rpm")
        if motor["rpm"] < 0.229 or motor["rpm"] > 15.114:
            raise ValueError("This bench controller accepts 0.229 to 15.114 rpm (approximately 15 rpm)")
        if motor.get("cw_sign") not in (-1, 1) or not motor.get("direction_verified"):
            raise ValueError("Verify CW with an unloaded jog; set cw_sign and direction_verified")
        if motor.get("model") not in MODELS:
            raise ValueError("Supported profiles: " + ", ".join(MODELS))
        if not isinstance(motor.get("id"), int) or not 0 <= motor["id"] <= 252:
            raise ValueError("Invalid motor ID")
        if limits["sensor_stale_s"] > 0.5:
            raise ValueError("Pull mode requires sensor_stale_s <= 0.5")


def open_uno(port):
    import serial
    from serial_ports import find_uno
    port = find_uno(port).device
    device = serial.Serial(port, 115200, timeout=0.03)
    time.sleep(2)  # Uno resets on serial open.
    reset_input(device)
    return device


def reset_input(device):
    device.reset_input_buffer()
    device._needle_rx_buffer = b""
    # A flush may split a row already being transmitted. Discard through the
    # first newline before interpreting packets; preserve everything after it.
    device._needle_resync = True
    device._needle_bytes_seen = 0
    device._needle_last_bytes = b""


def get_sample(device):
    # Preserve partial lines across serial timeouts rather than parsing fragments.
    buffer = getattr(device, "_needle_rx_buffer", b"")
    if b"\n" not in buffer:
        chunk = device.read(min(device.in_waiting, 512) or 1)
        device._needle_bytes_seen = getattr(device, "_needle_bytes_seen", 0) + len(chunk)
        if chunk:
            device._needle_last_bytes = chunk[-80:]
        buffer += chunk
    if b"\n" not in buffer:
        device._needle_rx_buffer = buffer
        if len(buffer) > 512:
            raise RuntimeError(f"Uno output has no line endings; received {buffer[:80]!r}. Check uploaded sketch and baud rate.")
        return None
    line, remainder = buffer.split(b"\n", 1)
    device._needle_rx_buffer = remainder
    if getattr(device, "_needle_resync", False):
        device._needle_resync = False
        return None
    line = line.decode("ascii", errors="strict").strip()
    if line.startswith("# ERROR"):
        raise RuntimeError(line)
    return parse_sample(line)


def capture(device, count=30):
    reset_input(device)
    samples = []
    deadline = time.monotonic() + 10
    while len(samples) < count:
        if time.monotonic() > deadline:
            byte_count = getattr(device, "_needle_bytes_seen", 0)
            last_bytes = getattr(device, "_needle_last_bytes", b"")
            raise RuntimeError(
                f"Received {len(samples)}/{count} valid HX711 samples in 10 s; "
                f"serial bytes={byte_count}, last bytes={last_bytes!r}. "
                "Upload uno_load_cell.ino (115200 baud), then check HX711 VCC/GND, DT=D2, SCK=D3.")
        sample = get_sample(device)
        if sample:
            if sample[3]:
                raise RuntimeError("ADC saturation during calibration")
            samples.append(sample[2])
    return statistics.mean(samples), statistics.stdev(samples)


def calibration_quality(cal):
    """Coarse rejection screen, not a certification of accuracy or applied loads.

    Reject fit residual or within-load noise >5% of known calibration span.
    """
    points = cal.get("points", [])
    if not points:
        return {"issues": [], "note": "No point data available for quality screening"}
    span = max(p["force_N"] for p in points) - min(p["force_N"] for p in points)
    if not finite(span) or span <= 0:
        return {"issues": ["No known-force calibration span"]}
    slope = cal["counts_per_N"]
    offset = cal["offset_counts"]
    residual = max(abs((p["mean_counts"] - offset) / slope - p["force_N"]) for p in points)
    noise = max(p["stdev_counts"] / abs(slope) for p in points)
    issues = []
    if not finite(residual) or residual > 0.05 * span:
        issues.append("Maximum fit residual exceeds 5% of calibration force span")
    if not finite(noise) or noise > 0.05 * span:
        issues.append("Within-load standard deviation exceeds 5% of calibration force span")
    return {"issues": issues, "span_N": span, "max_fit_residual_N": residual,
            "max_within_load_stdev_equivalent_N": noise, "screen_fraction": 0.05,
            "note": "Passing this screen does not verify actual applied forces or measurement accuracy"}


def calibration(args):
    forces = [float(x) for x in args.forces.split(",")]
    if len(forces) < 3 or any(not finite(x) for x in forces) or len(set(forces)) < 3 or 0 not in forces:
        raise ValueError("Use at least three distinct known forces including 0 N")
    points = []
    with open_uno(args.uno_port) as device:
        for force in forces:
            input(f"Apply {force:g} N at the specified calibration point/direction; wait to settle, then Enter: ")
            mean, noise = capture(device)
            points.append({"force_N": force, "mean_counts": mean, "stdev_counts": noise})
            print(f"mean={mean:.2f} counts; noise={noise:.2f} counts")
    mx = statistics.mean(forces)
    my = statistics.mean(p["mean_counts"] for p in points)
    slope = sum((p["force_N"] - mx) * (p["mean_counts"] - my) for p in points) / sum((x - mx)**2 for x in forces)
    if abs(slope) < 1 or not finite(slope):
        raise ValueError("No useful calibration response")
    offset = my - slope * mx
    residuals = [(p["mean_counts"] - offset) / slope - p["force_N"] for p in points]
    payload = {"created_utc": datetime.now(timezone.utc).isoformat(),
               "counts_per_N": slope, "offset_counts": offset, "points": points,
               "max_fit_residual_N": max(map(abs, residuals)), "setup_description": args.setup,
               "force_model": args.force_model}
    quality = calibration_quality(payload)
    payload["quality_screen"] = quality
    payload["quality_status"] = "rejected" if quality["issues"] else "screen_passed"
    output = Path(args.output)
    if quality["issues"]:
        output = output.with_name(output.stem + ".rejected_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + output.suffix)
        with output.open("x") as stream:
            json.dump(payload, stream, indent=2)
        raise ValueError("Calibration rejected: " + "; ".join(quality["issues"]) +
                         f". Measurements saved for diagnosis in {output}; no usable calibration written.")
    with output.open("x") as stream:
        json.dump(payload, stream, indent=2)
    print(f"Saved {output}; max fit residual {payload['max_fit_residual_N']:.4g} N")
    print("Validate with an independent known force, unloading, and the real fixture before use.")


def apply_trial_settings(config, args):
    # Per-run overrides are stored in the run snapshot; config.json is unchanged.
    for key in ("material", "engagement_level", "trial_id", "specimen_id"):
        value = getattr(args, key, None)
        if value is not None:
            config[key] = value
    angle = getattr(args, "needle_angle_deg", None)
    if angle is not None:
        if not finite(angle):
            raise ValueError("Needle angle must be finite")
        config.setdefault("geometry", {})["needle_angle_deg"] = angle
        config["angle_definition"] = "User-entered installed needle angle from rail/material surface; keep the axis/tangent definition consistent across trials."


def record(args):
    config_path = Path(args.config).resolve()
    config = load_json(config_path)
    apply_trial_settings(config, args)
    validate(config, args.pull)
    base = config_path.parent
    cal = ({"counts_per_N": 10000.0, "offset_counts": 100000,
            "force_model": config.get("geometry", {}).get("force_model", "unknown"),
            "setup_description": "SIMULATED"} if args.demo
           else load_json(base / config["calibration_file"]))
    if not finite(cal.get("counts_per_N")) or abs(cal["counts_per_N"]) < 1 or not finite(cal.get("offset_counts")):
        raise ValueError("Invalid calibration")
    quality = calibration_quality(cal)
    if cal.get("quality_status") == "rejected" or quality["issues"]:
        raise ValueError("Calibration rejected; do not use for force measurements: " + "; ".join(quality["issues"]))
    model = config.get("geometry", {}).get("force_model", "unknown")
    if model != "unknown" and cal.get("force_model") != model:
        raise ValueError("Calibration force model differs from geometry model")
    if args.demo and args.pull:
        raise ValueError("Demo cannot drive a motor")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output = base / config["output_dir"] / stamp
    output.mkdir(parents=True, exist_ok=False)
    metadata = {"config": config, "calibration": cal, "demo": args.demo,
                "pull": args.pull, "status": "starting", "stop_reason": None,
                "time_reference": "t_s is host monotonic receive time from first sample; Uno time is separate"}
    metadata_path = output / "metadata.json"
    def save():
        metadata_path.write_text(json.dumps(metadata, indent=2))
    save()
    device = motor = None
    reason = "duration_complete"
    first_rx = first_ms = last_rx = previous_seq = previous_ms = None
    elapsed_ms = 0
    motor_data = {}
    next_motor = 0
    sample_count = 0
    peak = 0
    run_start = time.monotonic()
    try:
        if not args.demo:
            device = open_uno(config["uno_port"])
            if args.tare:
                input("Remove contact/load from needle, leaving fixture assembled; Enter to tare: ")
                offset, noise = capture(device)
                cal["offset_counts"] = offset
                metadata["tare_noise_counts"] = noise
                save()
            if args.pull:
                from dynamixel_pull import DynamixelPull
                motor = DynamixelPull(config["motor"])
                metadata["motor_info"] = motor.open()
                metadata["status"] = "awaiting_pull_setup"
                save()
                input("Tare/zero is set. Engage material if testing, check travel clearance, "
                      "then Enter to start the pull (or Ctrl+C to cancel): ")
            reset_input(device)
        run_start = time.monotonic()
        metadata["status"] = "recording"
        save()
        with (output / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            while True:
                now = time.monotonic()
                if now - (first_rx if first_rx is not None else run_start) >= config["limits"]["duration_s"]:
                    if first_rx is None:
                        raise RuntimeError("No load-cell samples received")
                    break
                if now - (last_rx if last_rx is not None else run_start) > config["limits"]["sensor_stale_s"]:
                    raise RuntimeError("Load-cell data stale")
                if motor and motor.armed and now >= next_motor:
                    motor_data = motor.telemetry()
                    next_motor = time.monotonic() + 0.05
                    travel_limit = config["limits"].get("max_motor_path_deg")
                    if travel_limit is not None and motor_data["motor_path_deg"] >= travel_limit:
                        reason = "motor_travel_limit"
                        metadata["motor_final_telemetry"] = motor_data
                        break
                if args.demo:
                    time.sleep(0.1)
                    t = sample_count * 0.1
                    force = 0.3 * (config["limits"].get("max_abs_sensor_force_N") or 1.0) * math.sin(math.pi * t / config["limits"]["duration_s"])
                    sample = (sample_count, round(t * 1000), round(cal["offset_counts"] + force * cal["counts_per_N"]), False)
                else:
                    sample = get_sample(device)
                if sample is None:
                    continue
                rx = time.monotonic()
                seq, ms, raw, saturated = sample
                if previous_seq is not None:
                    if seq != (previous_seq + 1) & 0xFFFFFFFF:
                        raise RuntimeError("Uno reset or sample loss; stopping rather than using missing data")
                    step = (ms - previous_ms) & 0xFFFFFFFF
                    if step > 10000:
                        raise RuntimeError("Uno clock reset or invalid timestamp")
                    elapsed_ms += step
                previous_seq, previous_ms = seq, ms
                if first_rx is None:
                    first_rx, first_ms = rx, ms
                last_rx = rx
                sensor_force = (raw - cal["offset_counts"]) / cal["counts_per_N"]
                estimate = components(sensor_force, config.get("geometry", {}))
                row = {"t_s": rx - first_rx, "host_utc": datetime.now(timezone.utc).isoformat(),
                       "host_monotonic_s": rx, "uno_sequence": seq, "uno_ms": ms,
                       "uno_elapsed_s": elapsed_ms / 1000, "raw_counts": raw,
                       "saturated": int(saturated), "sensor_force_N": sensor_force,
                       "force_model": model, "geometry_source": "fixed_config_assumption",
                       "phase": "pull" if motor and motor.armed else "baseline"}
                for key in ("material", "engagement_level", "specimen_id", "trial_id", "needle_id"):
                    row[key] = config.get(key, "")
                for key in ("needle_angle_deg", "contact_x_mm", "contact_y_mm"):
                    row[key] = config.get("geometry", {}).get(key)
                row.update(estimate)
                row.update(motor_data)
                if motor_data:
                    row["motor_age_s"] = rx - motor_data["motor_rx_monotonic_s"]
                writer.writerow(row)
                stream.flush()
                sample_count += 1
                peak = max(peak, abs(sensor_force))
                if saturated:
                    raise RuntimeError("HX711 ADC saturation")
                force_limit = config["limits"].get("max_abs_sensor_force_N")
                if force_limit is not None and abs(sensor_force) >= force_limit:
                    reason = "sensor_force_limit"
                    break
                if motor and not motor.armed and sample_count >= 5:
                    motor.start()
                    metadata["motor_info"] = motor.info
                    next_motor = 0
                    save()
                if sample_count % 10 == 0:
                    print(f"{rx-first_rx:6.2f}s sensor={sensor_force:+.3f} N; peak |force|={peak:.3f} N")
    except KeyboardInterrupt:
        reason = "user_interrupt"
    except Exception as exc:
        reason = f"fault: {exc}"
        raise
    finally:
        if motor:
            metadata["stop_errors"] = motor.stop()
        if device:
            device.close()
        metadata.update(status="finished", stop_reason=reason, samples=sample_count,
                        peak_abs_sensor_force_N=peak, finished_utc=datetime.now(timezone.utc).isoformat())
        save()
        print(f"Stopped: {reason}\nSaved: {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    cal = sub.add_parser("calibrate")
    cal.add_argument("--uno-port", required=True)
    cal.add_argument("--forces", required=True, help="Known signed forces in N, comma separated, e.g. 0,0.5,1,2")
    cal.add_argument("--setup", required=True, help="Describe calibration point, direction, mounting and geometry")
    cal.add_argument("--force-model", choices=["unknown", "ideal_projection", "fixture_resultant"], default="unknown")
    cal.add_argument("--output", default="calibration.json")
    cal.set_defaults(function=calibration)
    run = sub.add_parser("record")
    run.add_argument("--config", required=True)
    run.add_argument("--material", help="Material name for this run")
    run.add_argument("--needle-angle-deg", type=float, help="Installed needle angle from rail/material surface")
    run.add_argument("--engagement-level", choices=["on_surface", "in_surface", "deep_into_surface"],
                     help="Qualitative engagement; no millimetre depth inferred")
    run.add_argument("--trial-id", help="Unique trial label, e.g. cork-on-01")
    run.add_argument("--specimen-id", help="Material sample identifier")
    run.add_argument("--pull", action="store_true", help="Enable motor; default is logging only")
    run.add_argument("--tare", action="store_true", help="Explicit unloaded fixture tare before test")
    run.add_argument("--demo", action="store_true", help="Synthetic data; no hardware access")
    run.set_defaults(function=record)
    args = parser.parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
