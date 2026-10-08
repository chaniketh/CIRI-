"""Planar, quasi-static force estimates. No resultant inference without a model."""
import math


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def components(sensor_force, geometry):
    out = {key: None for key in (
        "resultant_est_N", "Fx_est_N", "Fy_est_N", "needle_axial_est_N",
        "needle_transverse_est_N", "joint_moment_est_Nmm")}
    model = geometry.get("force_model", "unknown")
    if model == "unknown":
        return out
    direction = geometry.get("force_direction_deg")
    if not finite(direction):
        return out
    phi = math.radians(direction)
    if model == "fixture_resultant":
        # Calibration force applied at the contact point in this same direction.
        force = sensor_force
    elif model == "ideal_projection":
        axis = geometry.get("sensor_axis_deg")
        if not finite(axis):
            return out
        projection = math.cos(phi - math.radians(axis))
        if abs(projection) < 0.2:
            raise ValueError("Force nearly perpendicular to sensor axis: inversion unreliable")
        force = sensor_force / projection
    else:
        raise ValueError(f"Unknown force model: {model}")
    fx, fy = force * math.cos(phi), force * math.sin(phi)
    out.update(resultant_est_N=force, Fx_est_N=fx, Fy_est_N=fy)
    theta = geometry.get("needle_angle_deg")
    if finite(theta):
        relative = phi - math.radians(theta)
        out["needle_axial_est_N"] = force * math.cos(relative)
        out["needle_transverse_est_N"] = force * math.sin(relative)
    x, y = geometry.get("contact_x_mm"), geometry.get("contact_y_mm")
    if finite(x) and finite(y):
        out["joint_moment_est_Nmm"] = x * fy - y * fx
    return out


def parse_sample(line):
    if not line or line.startswith("#"):
        return None
    try:
        seq, ms, raw, sat = map(int, line.strip().split(","))
    except ValueError:
        raise ValueError(f"Invalid Uno packet: {line!r}") from None
    if not (0 <= seq <= 0xFFFFFFFF and 0 <= ms <= 0xFFFFFFFF
            and -8388608 <= raw <= 8388607 and sat in (0, 1)):
        raise ValueError("Uno packet outside valid range")
    return seq, ms, raw, bool(sat or abs(raw) >= 8388606)


def signed(value, bits):
    return value - (1 << bits) if value & (1 << (bits - 1)) else value
