"""Human-reviewed real-robot evidence gate; training never creates acceptance.

This checks consistency of an operator's evidence record. It does not certify
the truth of measured numbers or convert a finite trial set into a safety proof.
The independent Pi gate and physical commissioning still apply.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import re

from .data import file_sha256


def _evidence(value, name):
    entry = value.get(name)
    if (not isinstance(entry, dict) or entry.get("domain") != "real_robot"
            or not isinstance(entry.get("evidence_id"),str) or not entry["evidence_id"].strip()):
        raise ValueError(f"{name} requires real_robot measurements and a traceable evidence_id")
    return entry


def _number(entry, field):
    x = entry.get(field)
    if type(x) not in (float, int) or not math.isfinite(x) or x < 0:
        raise ValueError(f"Acceptance {field} must be a finite nonnegative measurement")
    return float(x)


def load_trial_eligibility(path: str | Path, *, calibration_fingerprint: str,
                           checkpoint_path: str | Path | None = None,
                           checkpoint_sha256: str | None = None) -> dict:
    """Load reviewed evidence for a separately enabled supervised WM trial.

    PC supplies checkpoint_path; Pi may supply configured checkpoint_sha256.
    Real held-out action ablation is necessary. It permits only the explicit
    wm_trial mode used to COLLECT paired physical evidence; it is not accepted
    general operation. Normal WM acceptance additionally requires those trials.
    """
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("Acceptance must be a schema_version=1 JSON object")
    if value.get("domain") != "real_robot" or value.get("reviewed") is not True:
        raise ValueError("Live WM needs human-reviewed real_robot evidence; synthetic tests cannot authorize motors")
    for field in ("reviewer", "reviewed_utc"):
        if not isinstance(value.get(field), str) or not value[field].strip():
            raise ValueError(f"Acceptance requires {field}")
    expected = file_sha256(checkpoint_path) if checkpoint_path is not None else checkpoint_sha256
    if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
        raise ValueError("Acceptance requires the exact SHA256 of the checkpoint")
    if checkpoint_sha256 is not None and checkpoint_sha256 != expected:
        raise ValueError("Configured checkpoint SHA256 differs from checkpoint bytes")
    if value.get("checkpoint_sha256") != expected:
        raise ValueError("Acceptance belongs to a different checkpoint")
    if value.get("calibration_fingerprint") != calibration_fingerprint:
        raise ValueError("Acceptance belongs to a different camera/motor/depth calibration")

    ablation = _evidence(value,"action_ablation")
    if ablation.get("held_out") is not True or ablation.get("common_valid_label_mask") is not True:
        raise ValueError("Action ablation must use held-out recordings and common valid label masks")
    actual = _number(ablation, "actual_action_aux_mae")
    if any(_number(ablation, name) <= actual for name in (
            "zero_action_aux_mae", "shuffled_action_aux_mae", "persistence_aux_mae")):
        raise ValueError("Actual actions must beat zero/shuffled actions and persistence on the reviewed task metric")
    return value


def load_acceptance(path: str | Path, *, calibration_fingerprint: str,
                    checkpoint_path: str | Path | None = None,
                    checkpoint_sha256: str | None = None) -> dict:
    """Normal WM eligibility adds reviewed paired real controller comparison."""
    value=load_trial_eligibility(path,calibration_fingerprint=calibration_fingerprint,
                                 checkpoint_path=checkpoint_path,checkpoint_sha256=checkpoint_sha256)
    paired = _evidence(value,"paired_trials")
    count = paired.get("count")
    if type(count) is not int or count < 1 or paired.get("same_conditions_and_gate") is not True:
        raise ValueError("Paired trials require a positive count and matching conditions/safety gate")
    for prefix in ("wm", "reactive"):
        for field in ("successes", "interventions", "collisions"):
            x = paired.get(f"{prefix}_{field}")
            if type(x) is not int or x < 0 or (field == "successes" and x > count):
                raise ValueError("Trial counts must be nonnegative integers; successes cannot exceed count")
    if paired["wm_collisions"] != 0 or paired["wm_interventions"] > paired["reactive_interventions"]:
        raise ValueError("Reviewed WM trials must have zero collisions and no increased intervention count")
    if _number(paired, "wm_min_clearance_m") < _number(paired, "reactive_min_clearance_m"):
        raise ValueError("Reviewed WM minimum clearance must not degrade relative to reactive baseline")
    no_worse_success = paired["wm_successes"] >= paired["reactive_successes"]
    improves = paired["wm_successes"] > paired["reactive_successes"]
    # Time is meaningful only over successful paired episodes, with its count recorded.
    common_successes = paired.get("common_successful_pairs", 0)
    if type(common_successes) is not int or not 0 <= common_successes <= min(paired["wm_successes"], paired["reactive_successes"]):
        raise ValueError("Invalid common_successful_pairs")
    if common_successes:
        improves |= _number(paired, "wm_median_time_s") < _number(paired, "reactive_median_time_s")
    if not (no_worse_success and improves):
        raise ValueError("Reviewed real WM must improve success or paired completion time without reducing success")
    return value
