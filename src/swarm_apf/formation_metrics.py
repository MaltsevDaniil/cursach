"""Offline errors against slots; never used to steer robots."""
import numpy as np
from .formation import slot_position


def formation_errors(positions, robot_ids, layout, signal):
    positions = np.asarray(positions, dtype=float)
    if positions.shape != (len(robot_ids), 2) or not np.isfinite(positions).all():
        raise ValueError("finite N-by-2 positions required")
    if len(set(robot_ids)) != len(robot_ids) or set(robot_ids) != set(layout.robot_ids):
        raise ValueError("IDs must match layout")
    errors = positions-np.array([slot_position(layout, signal, i) for i in robot_ids])
    lengths = np.linalg.norm(errors, axis=1)
    return dict(rms=float(np.sqrt(np.mean(lengths**2))), mean=float(lengths.mean()),
                maximum=float(lengths.max()),
                shape_rms=float(np.sqrt(np.mean(np.sum((errors-errors.mean(axis=0))**2, axis=1)))))
