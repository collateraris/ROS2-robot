"""Synthetic ray-cast toy episodes for software smoke tests ONLY.

These images, labels, dynamics and latency are invented. A checkpoint trained
on them is permanently marked synthetic and cannot be loaded for live control.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .data import ACTION_SEMANTICS, CONTROL_REPRESENTATION, DATASET_SCHEMA_VERSION


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def render_scene(position: np.ndarray, yaw: float, obstacles: np.ndarray,
                 person: np.ndarray, *, depth_scale_m: float = 3.0,
                 fov: float = math.radians(68)) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A cheap 2-D ray caster, not a photorealistic simulator."""
    width = 128
    angles = yaw + np.linspace(fov / 2, -fov / 2, width)
    directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
    ranges = np.full(width, 20.0)
    identities = np.zeros(width, dtype=np.int32)
    for axis in (0, 1):
        for wall in (-2.5, 2.5):
            component = directions[:, axis]
            distances = np.divide(wall - position[axis], component,
                                  out=np.full(width, 1e9), where=np.abs(component) > 1e-8)
            hit = (distances > 0) & (distances < ranges)
            ranges[hit] = distances[hit]
    objects = np.concatenate((obstacles, np.array([[person[0], person[1], 0.20]])), axis=0)
    for object_i, (x, y, radius) in enumerate(objects, start=1):
        relative = position - np.array([x, y])
        dot = directions @ relative
        discriminant = dot * dot - (relative @ relative - radius * radius)
        distances = -dot - np.sqrt(np.maximum(discriminant, 0.0))
        hit = (discriminant >= 0) & (distances > 0) & (distances < ranges)
        ranges[hit] = distances[hit]
        identities[hit] = object_i
    image = np.full((128, 128), 48, dtype=np.uint8)
    image[64:] = np.linspace(90, 160, 64, dtype=np.uint8)[:, None]
    for col, (distance, identity) in enumerate(zip(ranges, identities)):
        height = int(np.clip(72 / max(distance, 0.12), 5, 124))
        top, bottom = max(0, 64 - height // 2), min(128, 64 + height // 2)
        intensity = 200 if identity == len(objects) else 80 + (int(identity) * 37) % 110
        image[top:bottom, col] = int(intensity / (1 + 0.12 * distance))
    person_delta = person - position
    distance = float(np.linalg.norm(person_delta))
    bearing = wrap_angle(math.atan2(person_delta[1], person_delta[0]) - yaw)
    visible = bool(np.count_nonzero(identities == len(objects)) >= 3)
    sector_clearances = [float(np.min(part)) for part in np.array_split(ranges, 3)]
    aux = np.array([
        float(visible), np.clip(bearing / (fov / 2), -1, 1) if visible else 0.0,
        np.clip(1 - distance / depth_scale_m, 0, 1) if visible else 0.0,
        *[np.clip(clearance / depth_scale_m, 0, 1) for clearance in sector_clearances],
    ], dtype=np.float32)
    mask = np.array([True, visible, visible, True, True, True], dtype=np.bool_)
    return image, aux, mask


def make_episode(path: str | Path, *, seed: int, frames: int = 48) -> Path:
    if frames < 4:
        raise ValueError("frames must be >=4")
    rng = np.random.default_rng(seed)
    position = rng.uniform(-0.6, 0.6, size=2)
    yaw = float(rng.uniform(-math.pi, math.pi))
    obstacles = np.array([[-1.3, -0.8, 0.28], [1.4, 0.6, 0.38], [-0.2, 1.6, 0.25]])
    # Distinct layout IDs must correspond to changed geometry, not merely to
    # renamed recordings of one fixed furniture arrangement.
    obstacles[:, :2] += rng.uniform(-0.35, 0.35, size=(3, 2))
    person_base = rng.uniform(-1.8, 1.8, size=2)
    illumination = float(rng.uniform(0.85, 1.10))
    images, targets, masks, actions, intervals, segments = [], [], [], [], [], []
    timestamps = [0.0]
    previous_yaw_control = 0.0
    for i in range(frames):
        time = timestamps[-1]
        person = person_base + np.array([0.15 * math.sin(0.17 * time), 0.1 * math.cos(0.21 * time)])
        moving_obstacles = obstacles.copy()
        moving_obstacles[0, 0] += 0.15 * math.sin(0.11 * time + seed)
        im, target, mask = render_scene(position, yaw, moving_obstacles, person)
        noise = rng.normal(0, 1.5, im.shape)
        images.append(np.clip(im.astype(np.float32) * illumination + noise, 0, 255).astype(np.uint8))
        targets.append(target)
        masks.append(mask)
        if i == frames - 1:
            break
        # Virtual actions only: never run this process on a serial controller.
        previous_yaw_control = float(np.clip(0.6 * previous_yaw_control + rng.normal(0, 0.35), -0.9, 0.9))
        action = np.array([rng.uniform(0.15, 0.8), previous_yaw_control], dtype=np.float32)
        if rng.random() < .15:
            action[:] = 0  # stop -> stop examples, with honest temporal segmentation
        if target[4] < 0.25:
            action = np.array([0.0, 0.8 if target[3] > target[5] else -0.8], dtype=np.float32)
        pulse = float(rng.uniform(0.07, 0.12))
        stopped = float(rng.uniform(0.55, 0.75))
        dt = pulse + stopped
        piecewise = np.array([[action[0], action[1], pulse], [0.0, 0.0, stopped]], dtype=np.float32)
        yaw = wrap_angle(yaw + float(action[1]) * 1.1 * pulse)
        candidate = position + float(action[0]) * 0.65 * pulse * np.array([math.cos(yaw), math.sin(yaw)])
        clear = np.all(np.abs(candidate) < 2.25)
        clear = clear and all(np.linalg.norm(candidate - obj[:2]) > obj[2] + 0.16 for obj in moving_obstacles)
        if clear:
            position = candidate
        actions.append(action * pulse / dt)
        intervals.append(dt)
        segments.append(piecewise)
        timestamps.append(timestamps[-1] + dt)
    metadata = {
        "schema_version": DATASET_SCHEMA_VERSION, "episode_id": f"synthetic-{seed}", "session_id": f"synthetic-session-{seed}",
        "layout_id": f"toy-layout-{seed}", "day_id": f"synthetic-day-{seed}",
        "lighting_id": f"synthetic-illumination-{illumination:.3f}",
        "synthetic": True, "depth_scale_m": 3.0, "calibration_fingerprint": "synthetic",
        "action_semantics": ACTION_SEMANTICS, "control_representation": CONTROL_REPRESENTATION,
        "mcu_confirmation_validated": False, "confirmation_source": "synthetic_generator",
        "action_scale": {"virtual_forward_m_s": 0.65, "virtual_yaw_rad_s": 1.1},
        "purpose": "software smoke test only; no sim-to-real claim",
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path, images=np.asarray(images, dtype=np.uint8), timestamps=np.asarray(timestamps, dtype=np.float64),
        actions=np.asarray(actions, dtype=np.float32), dt=np.asarray(intervals, dtype=np.float32),
        aux=np.asarray(targets, dtype=np.float32), aux_mask=np.asarray(masks, dtype=np.bool_),
        transition_valid=np.ones(frames - 1, dtype=np.bool_),
        action_segments=np.asarray(segments, dtype=np.float32),
        segment_mask=np.ones((frames - 1, 2), dtype=np.bool_), metadata_json=np.array(json.dumps(metadata)),
    )
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("data/synthetic"))
    parser.add_argument("--episodes", type=int, default=6)
    parser.add_argument("--frames", type=int, default=48)
    parser.add_argument("--seed", type=int, default=100)
    args = parser.parse_args()
    if args.episodes < 2:
        parser.error("At least two episodes required for held-out smoke validation")
    for episode in range(args.episodes):
        path = make_episode(args.output / f"episode_{episode:03d}.npz", seed=args.seed + episode, frames=args.frames)
        print(path)
    print("SYNTHETIC ONLY: resulting checkpoints are blocked for live robot use.")


if __name__ == "__main__":
    main()
