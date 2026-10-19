"""Camera acquisition bounds on the Pi's CLOCK_MONOTONIC timebase.

No PC clock is compared with these timestamps.  The interval bounds acquisition
start, not when Python finished decoding an image.  SOE is a driver claim about
start of exposure; EOF additionally needs a measured exposure/readout bound.
Neither flag measures the accuracy of a camera driver.  Accepted calibration is
therefore required before either interval can authorize physical motion.

Kernel semantics: https://docs.kernel.org/userspace-api/media/v4l/buffer.html
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import math
from typing import Any

NS_PER_S = 1_000_000_000
TIMESTAMP_MASK = 0x0000E000
TIMESTAMP_MONOTONIC = 0x00002000
TIMESTAMP_COPY = 0x00004000
TSTAMP_SRC_MASK = 0x00070000
TSTAMP_SRC_EOF = 0x00000000
TSTAMP_SRC_SOE = 0x00010000


def _seconds(value: Any, name: str, *, optional: bool = False) -> float | None:
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite nonnegative seconds")
    return float(value)


def _ns(value: Any, name: str, *, optional: bool = False) -> int | None:
    if value is None and optional:
        return None
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError(f"{name} must be a nonnegative signed-64-bit integer")
    return value


@dataclass(frozen=True)
class TimingConfig:
    accepted: bool = False
    timestamp_uncertainty_s: float | None = None
    exposure_readout_bound_s: float | None = None
    empirical_latency_bound_s: float | None = None
    empirical_latency_min_s: float = 0.0
    evidence_id: str = ""

    def __post_init__(self) -> None:
        if type(self.accepted) is not bool:
            raise ValueError("timing.accepted must be a boolean")
        for name in ("timestamp_uncertainty_s", "exposure_readout_bound_s", "empirical_latency_bound_s"):
            _seconds(getattr(self, name), f"timing.{name}", optional=True)
        _seconds(self.empirical_latency_min_s, "timing.empirical_latency_min_s")
        if self.empirical_latency_bound_s is not None and self.empirical_latency_min_s > self.empirical_latency_bound_s:
            raise ValueError("Empirical latency minimum exceeds its upper bound")
        if not isinstance(self.evidence_id, str) or len(self.evidence_id) > 128 or any(ord(x) < 32 for x in self.evidence_id):
            raise ValueError("Timing evidence_id must be a short printable report identifier")
        if self.accepted and not self.evidence_id:
            raise ValueError("Accepted timing requires an evidence_id from the measured calibration report")
        if self.accepted and self.timestamp_uncertainty_s is None and self.empirical_latency_bound_s is None:
            raise ValueError("Accepted timing requires a measured timestamp uncertainty or empirical latency bound")

    @classmethod
    def from_dict(cls, value: dict | None) -> "TimingConfig":
        if value is None:
            return cls()
        if not isinstance(value, dict):
            raise ValueError("camera.timing must be a mapping")
        unknown = set(value) - {f.name for f in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown camera.timing fields: {sorted(unknown)}")
        return cls(**value)


def decode_v4l2_flags(flags: int | None) -> tuple[str, str]:
    if flags is None:
        return "UNKNOWN", "EMPIRICAL"
    if type(flags) is not int or not 0 <= flags <= 0xFFFFFFFF:
        raise ValueError("V4L2 flags must be uint32")
    clock = {0: "UNKNOWN", TIMESTAMP_MONOTONIC: "MONOTONIC", TIMESTAMP_COPY: "COPY"}.get(
        flags & TIMESTAMP_MASK, "UNKNOWN")
    source = {TSTAMP_SRC_SOE: "SOE", TSTAMP_SRC_EOF: "EOF"}.get(flags & TSTAMP_SRC_MASK, "UNKNOWN")
    return clock, source


@dataclass(frozen=True)
class CaptureTiming:
    dequeue_mono_ns: int
    v4l2_timestamp_ns: int | None
    v4l2_flags: int | None
    hardware_sequence: int | None
    timestamp_clock: str
    timestamp_source: str
    exposure_earliest_ns: int | None
    exposure_latest_ns: int | None
    timing_valid: bool
    timing_reason: str
    timing_evidence_id: str = ""

    def __post_init__(self) -> None:
        _ns(self.dequeue_mono_ns, "dequeue_mono_ns")
        _ns(self.v4l2_timestamp_ns, "v4l2_timestamp_ns", optional=True)
        for name in ("v4l2_flags", "hardware_sequence"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or not 0 <= value <= 0xFFFFFFFF):
                raise ValueError(f"{name} must be uint32 or null")
        _ns(self.exposure_earliest_ns, "exposure_earliest_ns", optional=True)
        _ns(self.exposure_latest_ns, "exposure_latest_ns", optional=True)
        if type(self.timing_valid) is not bool:
            raise ValueError("timing_valid must be boolean")
        if self.timestamp_clock not in {"MONOTONIC", "UNKNOWN", "COPY"}:
            raise ValueError("Unknown timestamp clock")
        if self.timestamp_source not in {"SOE", "EOF", "EMPIRICAL"}:
            raise ValueError("Unknown timestamp source")
        if not isinstance(self.timing_reason, str) or not 1 <= len(self.timing_reason) <= 160:
            raise ValueError("Missing/oversized timing reason")
        if not isinstance(self.timing_evidence_id, str) or len(self.timing_evidence_id) > 128:
            raise ValueError("Invalid timing evidence identifier")
        if self.timing_valid:
            if self.exposure_earliest_ns is None or self.exposure_latest_ns is None or not self.timing_evidence_id:
                raise ValueError("Valid timing requires measured bounds and evidence")
            if not 0 <= self.exposure_earliest_ns <= self.exposure_latest_ns <= self.dequeue_mono_ns:
                raise ValueError("Exposure interval must precede dequeue on the Pi timebase")
            if self.timestamp_source in {"SOE", "EOF"}:
                clock, source = decode_v4l2_flags(self.v4l2_flags)
                if (self.timestamp_clock, self.timestamp_source) != (clock, source) or clock != "MONOTONIC":
                    raise ValueError("SOE/EOF bounds require matching monotonic V4L2 flags")
                if self.v4l2_timestamp_ns is None or self.v4l2_timestamp_ns == 0:
                    raise ValueError("SOE/EOF bounds require a nonzero V4L2 timestamp")
        elif self.exposure_earliest_ns is not None or self.exposure_latest_ns is not None:
            raise ValueError("Uncalibrated timing must not carry plausible exposure bounds")

    def age_upper_ns(self, now_pi_mono_ns: int) -> int | None:
        """Only call on the Pi or with an opaque Pi timestamp returned to it."""
        _ns(now_pi_mono_ns, "now_pi_mono_ns")
        if now_pi_mono_ns < self.dequeue_mono_ns:
            raise ValueError("Age must be evaluated at/after dequeue on the same Pi monotonic clock")
        return None if not self.timing_valid else now_pi_mono_ns - self.exposure_earliest_ns

    @property
    def exposure_estimate_s(self) -> float:
        # Invalid: delivery-time placeholder for shadow visualization only.
        if not self.timing_valid:
            return self.dequeue_mono_ns / NS_PER_S
        return (self.exposure_earliest_ns + self.exposure_latest_ns) / (2 * NS_PER_S)

    @property
    def exposure_uncertainty_s(self) -> float | None:
        if not self.timing_valid:
            return None
        return (self.exposure_latest_ns - self.exposure_earliest_ns) / (2 * NS_PER_S)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "CaptureTiming":
        if not isinstance(value, dict) or set(value) != {f.name for f in fields(cls)}:
            raise ValueError("Unexpected camera timing envelope")
        return cls(**value)


def exposure_bounds(*, dequeue_mono_ns: int, v4l2_timestamp_ns: int | None = None,
                    v4l2_flags: int | None = None, hardware_sequence: int | None = None,
                    config: TimingConfig | dict | None = None) -> CaptureTiming:
    """Build an acquisition-start interval without guessing any clock offset.

    EOF: subtract the upper exposure+readout+software-timestamp delay and the
    timestamp error. SOE: timestamp +/- measured error. EMPIRICAL: dequeue minus
    the independently measured optical pipeline bounds, never read() duration.
    An implausible advertised MONOTONIC stamp is a fault, not a fallback excuse.
    """
    config = config if isinstance(config, TimingConfig) else TimingConfig.from_dict(config)
    _ns(dequeue_mono_ns, "dequeue_mono_ns")
    _ns(v4l2_timestamp_ns, "v4l2_timestamp_ns", optional=True)
    clock, native_source = decode_v4l2_flags(v4l2_flags)
    source = native_source if clock == "MONOTONIC" and native_source in {"SOE", "EOF"} else "EMPIRICAL"

    def result(earliest=None, latest=None, reason="timing_not_accepted", use_source=source):
        return CaptureTiming(dequeue_mono_ns, v4l2_timestamp_ns, v4l2_flags, hardware_sequence,
                             clock, use_source, earliest, latest, earliest is not None, reason, config.evidence_id)

    if not config.accepted:
        return result()
    epsilon_ns = None if config.timestamp_uncertainty_s is None else math.ceil(config.timestamp_uncertainty_s * NS_PER_S)
    if clock == "MONOTONIC" and v4l2_timestamp_ns not in {None, 0}:
        # Reject future stamps even when empirical fallback is configured.
        allowed_future = epsilon_ns or 0
        if v4l2_timestamp_ns > dequeue_mono_ns + allowed_future:
            return result(reason="v4l2_timestamp_in_future")
        if source == "SOE" and epsilon_ns is not None:
            low = v4l2_timestamp_ns - epsilon_ns
            high = min(dequeue_mono_ns, v4l2_timestamp_ns + epsilon_ns)
            return result(max(0, low), high, "monotonic_soe_measured_uncertainty")
        if source == "EOF" and epsilon_ns is not None and config.exposure_readout_bound_s is not None:
            low = v4l2_timestamp_ns - math.ceil(config.exposure_readout_bound_s * NS_PER_S) - epsilon_ns
            high = min(dequeue_mono_ns, v4l2_timestamp_ns + epsilon_ns)
            return result(max(0, low), high, "monotonic_eof_measured_exposure_readout")
    if config.empirical_latency_bound_s is not None:
        low = max(0, dequeue_mono_ns - math.ceil(config.empirical_latency_bound_s * NS_PER_S))
        high = dequeue_mono_ns - math.floor(config.empirical_latency_min_s * NS_PER_S)
        if high < low:
            return result(reason="empirical_interval_before_boot", use_source="EMPIRICAL")
        return result(low, high, "empirical_measured_optical_latency", use_source="EMPIRICAL")
    return result(reason="timestamp_or_required_bound_unavailable")
