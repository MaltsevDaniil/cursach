"""Read-only numerical metrics. Never supplies forces or corrections."""

import numpy as np
from .geometry import segment_circle_clearance


def calculate_path_length(path) -> float:
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError("Expected a finite path of shape (N, 2)")
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def minimum_path_clearance(path, obstacles, agent_radius: float = 0.0) -> float:
    """Minimum continuous clearance; +inf if there are no obstacles."""
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points):
        raise ValueError("Expected a nonempty path of shape (N, 2)")
    if not np.all(np.isfinite(points)):
        raise ValueError("Nonfinite path")
    if not obstacles:
        return float("inf")
    if len(points) == 1:
        return min(obs.distance(points[0], agent_radius) for obs in obstacles)
    return min(segment_circle_clearance(a, b, obs, agent_radius)
               for a, b in zip(points[:-1], points[1:]) for obs in obstacles)
