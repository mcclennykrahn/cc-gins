"""Time-window comparisons for sampled scenario epochs."""
import numpy as np


def _lower_boundary(boundary):
    # Grid multiplication and adding outage start to duration can round in
    # opposite directions. Allow two representable values without grid snapping.
    if np.isfinite(boundary):
        return np.nextafter(np.nextafter(boundary, -np.inf), -np.inf)
    return boundary


def at_or_after(t, boundary):
    """Select epochs at or after a boundary, allowing for roundoff."""
    return t >= _lower_boundary(boundary)


def in_window(t, start, end):
    """Select a half-open time window [start, end) with roundoff tolerance."""
    return at_or_after(t, start) & (t < _lower_boundary(end))
