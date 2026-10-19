"""Passive V4L2 diagnostics and a reproducible GPIO-LED latency experiment.

This module has no actuator dependency and never enables motors. The optional
LED collector toggles one explicitly selected GPIO through gpiozero; the LED
must occupy the selected image ROI. GPIO command times are intervals, not
assumed exact optical event times. Fit/check sessions stay separate, and an
observed maximum is reported as an empirical measurement, not a guarantee.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import threading
import time
import uuid

import numpy as np

from .camera import LatestCamera
from .camera_v4l2 import V4L2Camera
from .timebase import TimingConfig, exposure_bounds


def _distribution(values):
    values = np.asarray(values, dtype=np.float64)
    if not len(values):
        return None
    result = {"count": int(len(values)), "min": float(values.min()), "median": float(np.median(values)),
              "p95": float(np.percentile(values, 95)), "p99": float(np.percentile(values, 99)),
              "max": float(values.max()), "mean": float(values.mean())}
    if len(values) >= 1000:
        result["p99_9"] = float(np.percentile(values, 99.9))
    return result


def camera_mode_identity(camera_config: dict, device_metadata: dict | None = None) -> tuple[str, dict]:
    metadata = device_metadata or {}
    descriptor = {
        "backend": camera_config.get("backend", "v4l2"),
        "width": camera_config["width"], "height": camera_config["height"],
        "requested_fps": float(camera_config["requested_fps"]),
        "reported_fps": metadata.get("reported_fps"),
        "pixel_format": camera_config.get("pixel_format", "MJPG"),
        "buffer_count": metadata.get("buffer_count", camera_config.get("buffer_count", 4)),
        "configured_controls": camera_config.get("controls", {}),
        "driver": metadata.get("driver"), "bus_info": metadata.get("bus_info"), "card": metadata.get("card"),
        "drain_policy": "continuous_depth_one_v2",
    }
    canonical = json.dumps(descriptor, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hashlib.sha256(canonical).hexdigest(), descriptor


def probe_v4l2(device="/dev/video0", *, width=640, height=480, requested_fps=15., pixel_format="MJPG",
               frames=100, warmup=5, buffer_count=4, timing_config=None, controls=None) -> dict:
    if type(frames) is not int or frames < 2 or type(warmup) is not int or warmup < 0:
        raise ValueError("Probe needs at least two frames and nonnegative warmup")
    config = timing_config if isinstance(timing_config, TimingConfig) else TimingConfig.from_dict(timing_config)
    records = []
    with V4L2Camera(device, width=width, height=height, fps=requested_fps,
                   pixel_format=pixel_format, buffer_count=buffer_count, controls=controls) as camera:
        for i in range(frames + warmup):
            started = time.monotonic_ns()
            raw = camera.dequeue_latest(timeout_s=2.)
            if raw is None:
                raise RuntimeError("No V4L2 frame during the 2 s probe timeout")
            timing = exposure_bounds(dequeue_mono_ns=raw.dequeue_mono_ns,
                v4l2_timestamp_ns=raw.timestamp_ns, v4l2_flags=raw.flags,
                hardware_sequence=raw.sequence, config=config)
            if i >= warmup:
                records.append({"probe_index": i - warmup, **timing.to_dict(),
                    "dequeue_wait_ms": (raw.dequeue_mono_ns - started) / 1e6,
                    "discarded_ready_buffers": raw.skipped_ready_buffers,
                    "age_upper_at_dequeue_ns": timing.age_upper_ns(raw.dequeue_mono_ns)})
        metadata = dict(camera.metadata)
    dequeues = np.asarray([x["dequeue_mono_ns"] for x in records], dtype=np.int64)
    stamp_delays = [(x["dequeue_mono_ns"] - x["v4l2_timestamp_ns"]) / 1e6 for x in records
                    if x["timestamp_clock"] == "MONOTONIC" and x["v4l2_timestamp_ns"]]
    return {"created_utc": datetime.now(timezone.utc).isoformat(), "kind": "v4l2_timestamp_diagnostics",
        "device": metadata, "frames": len(records),
        "timestamp_clocks": sorted({x["timestamp_clock"] for x in records}),
        "timestamp_sources": sorted({x["timestamp_source"] for x in records}),
        "raw_v4l2_flag_values": sorted({x["v4l2_flags"] for x in records}),
        "dequeue_interval_ms": _distribution(np.diff(dequeues) / 1e6),
        "dequeue_minus_native_timestamp_ms": _distribution(stamp_delays),
        "read_wait_is_not_exposure_latency": True,
        "native_stamp_delay_includes_queue_but_soe_or_eof_semantics_differ": True,
        "optical_exposure_latency_measured": False, "motor_enable_allowed": False,
        "accepted": False, "records": records}


OPTICAL_COLUMNS = (
    "session", "split", "condition", "camera_mode_id", "event_id", "target_on",
    "event_earliest_ns", "event_latest_ns", "first_seen_dequeue_ns", "first_seen_frame_seq",
    "hardware_sequence", "v4l2_timestamp_ns", "v4l2_flags", "timestamp_source", "roi_mean",
)


def collect_optical_latency(camera_config: dict, csv_path: str | Path, *, gpio_pin: int,
                            roi: tuple[int, int, int, int], split: str, condition: str,
                            events: int = 40, min_hold_s: float = .5, max_hold_s: float = 1.5,
                            event_timeout_s: float = 8., off_max: float = 60., on_min: float = 180.,
                            stable_frames: int = 2, seed: int = 2026, session: str | None = None,
                            overwrite: bool = False, camera_factory=None, led_factory=None) -> dict:
    """Record first-visible GPIO LED edges using one Pi monotonic clock.

    gpio_pin uses BCM numbering (gpiozero); random holds dephase frame timing.
    The ROI mean must be <= off_max when off and >= on_min when on. Each event
    waits for stable visual confirmation before the next change, preventing
    mismatched events under a long camera delay. Partial CSV survives a fault.
    """
    if type(gpio_pin) is not int or not 0 <= gpio_pin <= 27:
        raise ValueError("GPIO pin must be an explicitly selected BCM number 0..27")
    if split not in {"fit", "check"} or not isinstance(condition, str) or not condition.strip():
        raise ValueError("Choose split=fit/check and describe the measured lighting/CPU/USB condition")
    if type(events) is not int or events < 2 or type(stable_frames) is not int or stable_frames < 1:
        raise ValueError("At least two events and one stable frame are required")
    durations = (min_hold_s, max_hold_s, event_timeout_s)
    if not all(math.isfinite(x) and x > 0 for x in durations) or min_hold_s > max_hold_s:
        raise ValueError("Positive finite hold and timeout durations are required")
    if not 0 <= off_max < on_min <= 255:
        raise ValueError("LED ROI thresholds must satisfy 0 <= off_max < on_min <= 255")
    if len(roi) != 4 or any(type(x) is not int for x in roi):
        raise ValueError("ROI must contain four integer values: x y width height")
    x, y, width, height = roi
    if min(x, y) < 0 or min(width, height) <= 0 or x + width > camera_config["width"] or y + height > camera_config["height"]:
        raise ValueError("LED ROI lies outside the configured camera image")
    path = Path(csv_path)
    if path.exists() and not overwrite:
        raise FileExistsError(f"Refusing to overwrite {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    session = session or uuid.uuid4().hex
    if not isinstance(session, str) or not session.strip():
        raise ValueError("Session identifier cannot be empty")
    rng = random.Random(seed)
    if led_factory is None:
        try:
            from gpiozero import LED
        except ImportError as exc:
            raise RuntimeError("Optical collector requires gpiozero and a working Pi GPIO pin factory") from exc
        led_factory = LED
    led = camera = None
    records = []
    try:
        # This is an LED only; no driver, arm, watchdog or motor API is imported.
        led = led_factory(gpio_pin)
        led.off()
        camera = (camera_factory or LatestCamera)(camera_config)
        mode_id, mode = camera_mode_identity(camera_config, camera.metadata)

        def wait_visible(target_on, after_ns):
            deadline = time.monotonic() + event_timeout_s
            consecutive, first = 0, None
            while time.monotonic() < deadline:
                if camera.error:
                    raise RuntimeError(camera.error)
                sample = camera.take_latest()
                if sample is None:
                    threading.Event().wait(.005)
                    continue
                if sample.timing.dequeue_mono_ns < after_ns:
                    continue
                intensity = float(np.mean(sample.bgr[y:y + height, x:x + width]))
                seen = intensity >= on_min if target_on else intensity <= off_max
                if not seen:
                    consecutive, first = 0, None
                    continue
                if first is None:
                    first = (sample, intensity)
                consecutive += 1
                if consecutive >= stable_frames:
                    return first
            raise RuntimeError(f"LED {'ON' if target_on else 'OFF'} did not become visible; check ROI/threshold/GPIO and camera delay")

        wait_visible(False, time.monotonic_ns())
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=OPTICAL_COLUMNS)
            writer.writeheader()
            for event in range(events):
                hold_deadline = time.monotonic() + rng.uniform(min_hold_s, max_hold_s)
                # The camera continues draining during this hold.
                while time.monotonic() < hold_deadline:
                    if camera.error:
                        raise RuntimeError(camera.error)
                    threading.Event().wait(min(.05, max(0., hold_deadline - time.monotonic())))
                on = event % 2 == 0
                earliest = time.monotonic_ns()
                led.on() if on else led.off()
                latest = time.monotonic_ns()
                sample, intensity = wait_visible(on, latest)
                raw = sample.timing
                row = {"session": session, "split": split, "condition": condition.strip(), "camera_mode_id": mode_id,
                    "event_id": event, "target_on": int(on), "event_earliest_ns": earliest,
                    "event_latest_ns": latest, "first_seen_dequeue_ns": raw.dequeue_mono_ns,
                    "first_seen_frame_seq": sample.sequence, "hardware_sequence": raw.hardware_sequence,
                    "v4l2_timestamp_ns": raw.v4l2_timestamp_ns, "v4l2_flags": raw.v4l2_flags,
                    "timestamp_source": raw.timestamp_source, "roi_mean": intensity}
                writer.writerow(row)
                stream.flush()
                records.append(row)
        return {"created_utc": datetime.now(timezone.utc).isoformat(), "kind": "gpio_led_first_visible_latency",
            "session": session, "split": split, "condition": condition, "events": len(records),
            "camera_mode_id": mode_id, "camera_mode": mode, "camera_metadata": camera.metadata,
            "gpio_bcm_pin": gpio_pin, "roi_xywh": list(roi), "thresholds": {"off_max": off_max, "on_min": on_min},
            "event_interval_random_seed": seed, "csv_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "event_to_first_visible_upper_s": _distribution(
                [(r["first_seen_dequeue_ns"] - r["event_earliest_ns"]) / 1e9 for r in records]),
            "includes_frame_phase_and_gpio_command_time": True,
            "motor_enable_allowed": False, "accepted": False,
            "next_step": "Collect a distinct check session; fit with latency-fit. Repeat dark/high-CPU/USB-load conditions."}
    finally:
        if led is not None:
            try:
                led.off()
            finally:
                led.close()
        if camera is not None:
            camera.close()


def fit_optical_latency(csv_paths: list[str | Path], *, requested_fps: float,
                        extra_frame_periods: float = 2., extra_margin_s: float = .05,
                        min_per_split: int = 20) -> dict:
    """Fit a maximum-plus-margin bound on fit sessions, evaluate check sessions.

    Never uses held-out data to tighten/loosen its candidate. Input can be
    separately collected CSV files; session and camera-mode identities must
    agree, whereas measurement conditions may deliberately differ.
    """
    if (not math.isfinite(requested_fps) or requested_fps <= 0
            or not math.isfinite(extra_frame_periods) or extra_frame_periods < 1
            or not math.isfinite(extra_margin_s) or extra_margin_s < 0):
        raise ValueError("Use finite positive FPS, >=1 extra frame period, and a nonnegative engineering margin")
    if type(min_per_split) is not int or min_per_split < 2:
        raise ValueError("Need at least two observations per split")
    rows, source_hashes, seen_frames = [], [], set()
    required = {"session", "split", "camera_mode_id", "event_earliest_ns", "event_latest_ns", "first_seen_dequeue_ns"}
    for csv_path in csv_paths:
        path = Path(csv_path)
        source_hashes.append(hashlib.sha256(path.read_bytes()).hexdigest())
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if not required <= set(reader.fieldnames or []):
                raise ValueError("Optical CSV lacks the event/dequeue interval or session/mode fields")
            for line, row in enumerate(reader, 2):
                try:
                    earliest, latest, first = (int(row[key]) for key in
                        ("event_earliest_ns", "event_latest_ns", "first_seen_dequeue_ns"))
                except (ValueError, TypeError) as exc:
                    raise ValueError(f"Invalid monotonic nanoseconds at {path.name}:{line}") from exc
                if not 0 <= earliest <= latest <= first < 2**63:
                    raise ValueError("Optical event/dequeue ordering invalid; use a single Pi monotonic clock")
                if row["split"] not in {"fit", "check"} or not row["session"] or len(row["camera_mode_id"]) != 64:
                    raise ValueError("Every optical record needs fit/check, session, and a camera-mode SHA256")
                identity = (row["session"], first)
                if identity in seen_frames:
                    raise ValueError("Duplicate optical observation cannot count as independent evidence")
                seen_frames.add(identity)
                rows.append({**row, "latency_upper_s": (first - earliest) / 1e9,
                    "gpio_command_interval_s": (latest - earliest) / 1e9, "file": path.name, "line": line})
    if not rows:
        raise ValueError("No optical observations")
    modes = {row["camera_mode_id"] for row in rows}
    if len(modes) != 1:
        raise ValueError("Camera modes changed; never combine timing calibrations from different modes")
    splits = {name: [r for r in rows if r["split"] == name] for name in ("fit", "check")}
    if any(len(part) < min_per_split for part in splits.values()):
        raise ValueError(f"Need at least {min_per_split} observations in both independent fit/check splits")
    sessions = {name: {r["session"] for r in part} for name, part in splits.items()}
    if sessions["fit"] & sessions["check"]:
        raise ValueError("A session cannot be both fit and check; collect a separate held-out session")
    fit = np.asarray([r["latency_upper_s"] for r in splits["fit"]])
    check = np.asarray([r["latency_upper_s"] for r in splits["check"]])
    proposed = float(fit.max() + extra_frame_periods / requested_fps + extra_margin_s)
    violations = [r for r in splits["check"] if r["latency_upper_s"] > proposed]
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
        "kind": "empirical_optical_latency_upper_bound_not_a_certificate", "camera_mode_id": next(iter(modes)),
        "source_csv_sha256": source_hashes, "requested_fps": requested_fps,
        "extra_frame_periods": extra_frame_periods, "extra_margin_s": extra_margin_s,
        "fit_sessions": sorted(sessions["fit"]), "check_sessions": sorted(sessions["check"]),
        "conditions": sorted({r.get("condition", "unspecified") for r in rows}),
        "fit_latency_upper_s": _distribution(fit), "check_latency_upper_s": _distribution(check),
        "candidate_latency_bound_s": proposed, "empirical_holdout_pass": not violations,
        "check_violations": [{"file": r["file"], "line": r["line"], "latency_s": r["latency_upper_s"]} for r in violations],
        "gpio_command_interval_s": _distribution([r["gpio_command_interval_s"] for r in rows]),
        "timestamp_min_bound_s": 0., "motor_enable_allowed": False, "accepted": False}
    evidence_id = hashlib.sha256(json.dumps(report, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()).hexdigest()
    report["evidence_id"] = evidence_id
    report["suggested_timing_config"] = {
        "accepted": False, "timestamp_uncertainty_s": None, "exposure_readout_bound_s": None,
        "empirical_latency_bound_s": proposed, "empirical_latency_min_s": 0., "evidence_id": evidence_id,
    } if not violations else None
    report["limitations"] = (
        "Finite measured maxima can be exceeded. Repeat at operational exposure/format/FPS and dark/USB/CPU load. "
        "Event latency includes sampling phase and GPIO command interval; it does not prove native SOE/EOF accuracy. "
        "No positive lower camera-latency bound is inferred from this experiment. GPIO wiring/optical response must be checked."
    )
    return report
