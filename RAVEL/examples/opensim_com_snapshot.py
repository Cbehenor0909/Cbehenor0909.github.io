"""Read COM data from an existing OpenSim 4.x Model and compatible State.

No model/data loading, smoothing, force inference, or synthetic-model duplication.
All returned vectors are expressed in and observed relative to Ground.
The caller owns state construction and derivative consistency.
API references: OpenSim Model, Body and Frame documentation linked in the lesson.
This adapter requires an existing OpenSim Python environment for real use.
"""
import csv
import math


def collect_com_snapshot(model, state, *, level="position", acceleration_source=None):
    """level: position, velocity, or acceleration.

    For acceleration, explicitly identify the State's intended derivative source:
    'prescribed_kinematics' (consistent imposed motion) or 'forward_dynamics'
    (valid force/control state). This declaration is provenance, not verification.
    A position-only IK state does not contain a measured acceleration trajectory.
    """
    if level not in {"position", "velocity", "acceleration"}:
        raise ValueError("level must be position, velocity or acceleration")
    if level == "acceleration" and acceleration_source not in {
        "prescribed_kinematics", "forward_dynamics"
    }:
        raise ValueError("Declare a consistent acceleration_source before querying acceleration")

    def vector(value):
        values = [float(value.get(i)) for i in range(3)]
        if not all(math.isfinite(x) for x in values):
            raise ValueError("OpenSim returned a nonfinite vector")
        return values

    model.realizePosition(state)
    if level in {"velocity", "acceleration"}:
        model.realizeVelocity(state)
    if level == "acceleration":
        model.realizeAcceleration(state)

    bodies = model.getBodySet()
    rows = []
    for i in range(bodies.getSize()):
        body = bodies.get(i)
        mass = float(body.getMass())
        if not math.isfinite(mass) or mass < 0:
            raise ValueError("Body masses must be finite and nonnegative")
        local_com = body.getMassCenter()
        row = {
            "body": body.getName(), "mass_kg": mass,
            "local_com_m": vector(local_com),
            "position_m": vector(body.findStationLocationInGround(state, local_com)),
        }
        if level in {"velocity", "acceleration"}:
            row["velocity_m_s"] = vector(body.findStationVelocityInGround(state, local_com))
        if level == "acceleration":
            row["acceleration_m_s2"] = vector(body.findStationAccelerationInGround(state, local_com))
        rows.append(row)

    total = {
        "body": "whole_model", "mass_kg": float(model.getTotalMass(state)),
        "position_m": vector(model.calcMassCenterPosition(state)),
    }
    if not math.isfinite(total["mass_kg"]) or total["mass_kg"] <= 0:
        raise ValueError("Model must have positive finite total mass")
    if level in {"velocity", "acceleration"}:
        total["velocity_m_s"] = vector(model.calcMassCenterVelocity(state))
    if level == "acceleration":
        total["acceleration_m_s2"] = vector(model.calcMassCenterAcceleration(state))
    return {
        "time_s": float(state.getTime()), "frame": "Ground", "level": level,
        "acceleration_source": acceleration_source if level == "acceleration" else None,
        "bodies": rows, "whole_model": total,
    }


def write_snapshots_csv(snapshots, path):
    """Write a NEW derived file; never overwrite an existing file.

    Missing velocity/acceleration fields stay blank, rather than becoming zeros.
    Keep model version, filtering and input provenance in accompanying metadata.
    """
    fields = ["time_s", "frame", "level", "acceleration_source", "body", "mass_kg"]
    for prefix in ["local_com_m", "position_m", "velocity_m_s", "acceleration_m_s2"]:
        fields.extend(f"{prefix}_{axis}" for axis in "xyz")
    with open(path, "x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for snapshot in snapshots:
            for body in [*snapshot["bodies"], snapshot["whole_model"]]:
                row = {key: snapshot[key] for key in ["time_s", "frame", "level", "acceleration_source"]}
                row.update({key: body[key] for key in ["body", "mass_kg"]})
                for prefix in ["local_com_m", "position_m", "velocity_m_s", "acceleration_m_s2"]:
                    if prefix in body:
                        row.update({f"{prefix}_{axis}": value for axis, value in zip("xyz", body[prefix])})
                writer.writerow(row)
