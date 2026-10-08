#!/usr/bin/env python3
"""Combine force log with synchronized measured angles, summarize, optionally plot."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
from mechanics import components


def number(value):
    if value is None or value == "":
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Nonfinite measurement")
    return result


def read_angles(path):
    with Path(path).open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("No angle measurements")
    numeric = ("t_s", "needle_angle_deg", "joint_angle_deg", "contact_x_mm", "contact_y_mm")
    for row in rows:
        for key in numeric:
            row[key] = number(row.get(key))
        if row["t_s"] is None:
            raise ValueError("Every angle row needs t_s")
    times = [r["t_s"] for r in rows]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("Angle times must be strictly increasing")
    return rows


def interpolate(rows, t, max_gap=0.5):
    times = [row["t_s"] for row in rows]
    i = bisect.bisect_left(times, t)
    if i < len(rows) and abs(times[i] - t) < 1e-9:
        return dict(rows[i])
    if i == 0 or i == len(rows):
        return None  # Never extrapolate an unmeasured angle.
    left, right = rows[i-1], rows[i]
    if right["t_s"] - left["t_s"] > max_gap:
        return None
    f = (t - left["t_s"]) / (right["t_s"] - left["t_s"])
    out = {"t_s": t, "phase": left.get("phase", ""), "event": ""}
    for key in ("needle_angle_deg", "joint_angle_deg", "contact_x_mm", "contact_y_mm"):
        a, b = left[key], right[key]
        out[key] = a + f * (b-a) if a is not None and b is not None else None
    return out


def fit_stiffness(rows):
    # Only an explicitly labeled quasi-static loading interval is fitted.
    pairs = [(math.radians(number(r["joint_angle_deg"])), number(r["joint_moment_est_Nmm"]))
             for r in rows if r.get("angle_phase") == "loading"
             and number(r.get("joint_angle_deg")) is not None
             and number(r.get("joint_moment_est_Nmm")) is not None]
    if len(pairs) < 3:
        return None
    angles, moments = zip(*pairs)
    span = max(angles) - min(angles)
    if span < math.radians(1):
        return None
    xa, ya = sum(angles)/len(angles), sum(moments)/len(moments)
    xx = sum((x-xa)**2 for x in angles)
    stiffness = sum((x-xa)*(y-ya) for x, y in pairs)/xx
    intercept = ya-stiffness*xa
    residual = sum((y-(stiffness*x+intercept))**2 for x, y in pairs)
    total = sum((y-ya)**2 for y in moments)
    return {"apparent_stiffness_Nmm_per_rad": stiffness,
            "intercept_Nmm": intercept, "angle_min_deg": math.degrees(min(angles)),
            "angle_max_deg": math.degrees(max(angles)), "samples": len(pairs),
            "R_squared": 1-residual/total if total else None,
            "caution": "Local quasi-static fit only; contains fixture effects and TPU hysteresis. Do not extrapolate."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir")
    parser.add_argument("--angles", help="Measured angle CSV, with t_s synchronized to samples.csv")
    parser.add_argument("--max-angle-gap-s", type=float, default=0.5)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.max_angle_gap_s) or args.max_angle_gap_s <= 0:
        raise ValueError("max-angle-gap-s must be positive")
    root = Path(args.run_dir)
    meta = json.loads((root/"metadata.json").read_text())
    with (root/"samples.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("Run contains no samples")
    angle_rows = read_angles(args.angles) if args.angles else None
    geometry = meta["config"].get("geometry", {})
    for row in rows:
        row["joint_angle_deg"] = None
        row["angle_phase"] = ""
        row["angle_event"] = ""
        if angle_rows is not None:
            measured = interpolate(angle_rows, float(row["t_s"]), args.max_angle_gap_s)
            dynamic = dict(geometry)
            # Never fall back to a fixed angle/contact location when dynamic data is missing.
            for key in ("needle_angle_deg", "contact_x_mm", "contact_y_mm"):
                dynamic[key] = measured.get(key) if measured else None
                row[key] = dynamic[key]
            row.update(components(float(row["sensor_force_N"]), dynamic))
            row["geometry_source"] = "measured_interpolated" if measured else "missing_dynamic_geometry"
            if measured:
                row["joint_angle_deg"] = measured.get("joint_angle_deg")
                row["angle_phase"] = measured.get("phase", "")
                row["angle_event"] = measured.get("event", "")
    with (root/"analyzed.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    force_rows = [r for r in rows if not int(r["saturated"])]
    peak_row = max(force_rows, key=lambda r: abs(float(r["sensor_force_N"]))) if force_rows else None
    summary = {"demo": meta["demo"], "material": meta["config"].get("material"),
               "engagement_level": meta["config"].get("engagement_level"),
               "trial_id": meta["config"].get("trial_id"),
               "stop_reason": meta["stop_reason"], "samples": len(rows),
               "peak_abs_sensor_force_N": abs(float(peak_row["sensor_force_N"])) if peak_row else None,
               "peak_time_s": float(peak_row["t_s"]) if peak_row else None,
               "angle_at_peak_deg": number(peak_row.get("needle_angle_deg")) if peak_row else None,
               "angle_coverage_samples": sum(r.get("geometry_source") == "measured_interpolated" for r in rows),
               "experiment_type": meta["config"].get("experiment_type", "rigid_needle_pull"),
               "note": "Peaks are sampled peaks, not guaranteed instantaneous maxima. Forces/moments are model estimates."}
    if any(number(r.get("joint_angle_deg")) is not None for r in rows):
        summary["stiffness_fit"] = fit_stiffness(rows)
    for key in ("needle_transverse_est_N",):
        vals = [number(r.get(key)) for r in force_rows if number(r.get(key)) is not None]
        summary["peak_abs_"+key] = max(map(abs, vals)) if vals else None
    (root/"summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
        t = [float(r["t_s"]) for r in rows]
        axes[0].plot(t, [float(r["sensor_force_N"]) for r in rows], label="calibrated sensor response")
        for key, label in (("needle_axial_est_N", "axial estimate"), ("needle_transverse_est_N", "transverse estimate")):
            axes[0].plot(t, [number(r.get(key)) for r in rows], label=label)
        axes[0].set(ylabel="Force (N)")
        axes[0].legend()
        axes[1].plot(t, [number(r.get("needle_angle_deg")) for r in rows], label="needle angle")
        axes[1].set(xlabel="Receive time (s)", ylabel="Needle angle from rail (deg)")
        axes[1].legend()
        fig.suptitle(("SIMULATED DEMO — " if meta["demo"] else "") + str(meta["config"].get("material")))
        fig.savefig(root/"force_angle.png", dpi=160)
        plt.close(fig)


if __name__ == "__main__":
    main()
