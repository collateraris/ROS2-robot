"""Transparent reactive comparison policy; current geometry remains authoritative."""
from __future__ import annotations

import math
from typing import Callable

import numpy as np


def reactive_action(person_bearing_rad: float, options: np.ndarray, *, angular_scale_rps: float,
                    action_allowed: Callable[[np.ndarray], bool], lookahead_s: float = 4.0) -> np.ndarray:
    """Choose the currently allowed gentle arc closest to the visible target.

    This policy has no learned dynamics or room memory. Callers first validate
    target identity/range and feed the same admissible primitive set and current
    geometry guard to both reactive and WM controllers in paired comparisons.
    """
    if not math.isfinite(person_bearing_rad) or angular_scale_rps <= 0 or lookahead_s <= 0:
        return np.zeros(2, np.float32)
    options = np.asarray(options, dtype=np.float32)
    if options.ndim != 2 or options.shape[1] != 2 or not np.isfinite(options).all():
        raise ValueError("options must be a finite [K,2] primitive array")
    allowed = [action for action in options if action[0] > 0 and action_allowed(action.copy())]
    if not allowed:
        return np.zeros(2, np.float32)
    return min(allowed, key=lambda a: abs(person_bearing_rad - float(a[1]) * angular_scale_rps * lookahead_s)).copy()
