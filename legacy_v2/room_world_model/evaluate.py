"""Offline held-out evaluation: real controls, zero/permuted controls, persistence.

This measures prediction quality on recordings. It neither executes commands
nor estimates the real robot's success/collision rate from video prediction.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader

from .data import (AUX_NAMES, SequenceDataset, discover_episodes, episode_content_sha256,
                   file_sha256, validate_episode)
from .model import WorldState, load_checkpoint
from .train import move_batch


class Metrics:
    def __init__(self, depth_scale_m: float):
        self.depth_scale_m = depth_scale_m
        self.pixel_sum: list[float] = []
        self.sample_counts: list[int] = []
        self.aux_error = np.zeros(6, dtype=np.float64)
        self.aux_count = np.zeros(6, dtype=np.int64)
        self.clearance_overestimates: list[np.ndarray] = []
        self.near_count = 0
        self.false_clear_count = 0
        self.aux_horizon_sum: list[float] = []
        self.aux_horizon_count: list[int] = []
        self.visibility_tp = self.visibility_fp = self.visibility_fn = 0

    def update(self, offset: int, predicted_image: torch.Tensor, target_image: torch.Tensor,
               predicted_aux: torch.Tensor, target_aux: torch.Tensor, mask: torch.Tensor) -> None:
        while len(self.pixel_sum) <= offset:
            self.pixel_sum.append(0.0)
            self.sample_counts.append(0)
            self.aux_horizon_sum.append(0.0)
            self.aux_horizon_count.append(0)
        pixel_mse = (predicted_image - target_image).square().flatten(1).mean(1)
        self.pixel_sum[offset] += float(pixel_mse.sum())
        self.sample_counts[offset] += len(pixel_mse)
        self.aux_error += ((predicted_aux - target_aux).abs() * mask).sum(0).cpu().numpy()
        self.aux_count += mask.sum(0).cpu().numpy()
        self.aux_horizon_sum[offset] += float(((predicted_aux - target_aux).abs() * mask).sum())
        self.aux_horizon_count[offset] += int(mask.sum())
        truth, prediction = target_aux[:, 0] >= .5, predicted_aux[:, 0] >= .5
        visible_mask = mask[:, 0]
        self.visibility_tp += int((truth & prediction & visible_mask).sum())
        self.visibility_fp += int((~truth & prediction & visible_mask).sum())
        self.visibility_fn += int((truth & ~prediction & visible_mask).sum())
        error_m = ((predicted_aux[:, 3:] - target_aux[:, 3:]) * self.depth_scale_m)[mask[:, 3:]]
        if error_m.numel():
            self.clearance_overestimates.append(error_m.clamp_min(0).cpu().numpy())
        threshold = 0.6 / self.depth_scale_m
        near = (target_aux[:, 3:] < threshold) & mask[:, 3:]
        self.near_count += int(near.sum())
        self.false_clear_count += int((near & (predicted_aux[:, 3:] >= threshold)).sum())

    def result(self) -> dict[str, Any]:
        errors = np.concatenate(self.clearance_overestimates) if self.clearance_overestimates else np.array([])
        return {
            "pixel_mse": sum(self.pixel_sum) / max(sum(self.sample_counts), 1),
            "pixel_mse_by_future_frame": [total / max(n, 1) for total, n in zip(self.pixel_sum, self.sample_counts)],
            "aux_mae": {name: float(self.aux_error[i] / self.aux_count[i]) if self.aux_count[i] else None
                        for i, name in enumerate(AUX_NAMES)},
            "valid_aux_counts": {name: int(self.aux_count[i]) for i, name in enumerate(AUX_NAMES)},
            "common_mask_aux_mae": float(self.aux_error.sum() / self.aux_count.sum()) if self.aux_count.sum() else None,
            "aux_mae_by_future_frame": [total / n if n else None for total, n in zip(self.aux_horizon_sum, self.aux_horizon_count)],
            "aux_valid_count_by_future_frame": self.aux_horizon_count,
            "person_visibility_recall": self.visibility_tp / (self.visibility_tp + self.visibility_fn)
            if self.visibility_tp + self.visibility_fn else None,
            "person_visibility_f1": 2 * self.visibility_tp / (2 * self.visibility_tp + self.visibility_fp + self.visibility_fn)
            if 2 * self.visibility_tp + self.visibility_fp + self.visibility_fn else None,
            "person_clipped_range_mae_m": float(self.aux_error[2] / self.aux_count[2] * self.depth_scale_m)
            if self.aux_count[2] else None,
            "clearance_overestimate_m": {f"p{percentile}": float(np.percentile(errors, percentile)) if len(errors) else None
                                          for percentile in (50, 95, 99, 100)},
            "near_obstacle_sector_count_below_0_6m": self.near_count,
            "false_clearance_rate_at_0_6m": self.false_clear_count / self.near_count if self.near_count else None,
        }


def permute_pulse_controls(segments: torch.Tensor, mask: torch.Tensor, *, burn_in: int,
                           rng: np.random.Generator) -> tuple[torch.Tensor, dict[str, int]]:
    """Shuffle ONLY active nonzero pulse controls, leaving idle, padding and dt fixed.

    Unlike shuffling padded transition rows, this cannot accidentally replace
    a pulse with a padding zero or turn an observed zero-PWM wait into motion.
    The effect is measured on future actions only, after identical burn-in.
    """
    result = segments.clone()
    future = result[:, burn_in:]
    moving = mask[:, burn_in:] & (segments[:, burn_in:, :, :2].abs().amax(-1) > 1e-8)
    controls = segments[:, burn_in:, :, :2][moving].clone()
    count = len(controls)
    changed = 0
    if count > 1:
        order = rng.permutation(count)
        source = np.roll(order, 1)
        shuffled = controls.clone()
        shuffled[torch.as_tensor(order, device=segments.device)] = controls[torch.as_tensor(source, device=segments.device)]
        changed = int((shuffled - controls).abs().amax(-1).gt(1e-8).sum())
        future[..., :2][moving] = shuffled
    return result, {"eligible_pulses": count, "changed_pulses": changed}


@torch.no_grad()
def run_evaluation(args: argparse.Namespace) -> dict[str, Any]:
    if args.max_batches<0 or args.batch_size<1 or args.cpu_threads<1:
        raise ValueError("Evaluation needs positive batch/thread counts and max_batches >= 0")
    torch.set_num_threads(args.cpu_threads)
    device = ("cuda" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    model, metadata = load_checkpoint(args.checkpoint, device=device)
    sequence_length = args.sequence_length or int(metadata.get("sequence_length", 6))
    burn_in = int(metadata.get("burn_in_frames", 2)) if args.burn_in is None else args.burn_in
    if not 0 <= burn_in < sequence_length:
        raise ValueError("Need 0 <= burn_in < sequence_length")
    infos = [validate_episode(path, max_dt_s=model.config.max_dt_s) for path in discover_episodes(args.data)]
    split = getattr(args, "split", "test")
    excluded_splits = {"train", "validation"} if split == "test" else {"train", "test"}
    excluded_sessions = set().union(*(set(metadata.get(f"{name}_sessions", [])) for name in excluded_splits))
    excluded_hashes = {item["sha256"] for item in metadata.get("episode_provenance", []) if item["split"] in excluded_splits}
    excluded_content = {item["content_sha256"] for item in metadata.get("episode_provenance", [])
                        if item["split"] in excluded_splits and "content_sha256" in item}
    axis = metadata.get("split_by", "session")
    excluded_axis = set().union(*(set(metadata.get(f"{name}_split_values", [])) for name in excluded_splits))
    held_out = [info for info in infos if info.split_group not in excluded_sessions
                and info.group_value(axis) not in excluded_axis
                and file_sha256(info.path) not in excluded_hashes
                and episode_content_sha256(info.path) not in excluded_content]
    if not held_out:
        raise ValueError("No held-out episodes: train/selected validation groups and copied recordings are excluded")
    depth_scale = float(metadata.get("depth_scale_m", 3.0))
    for info in held_out:
        if abs(float(info.metadata.get("depth_scale_m", depth_scale)) - depth_scale) > 1e-6:
            raise ValueError("Held-out label normalization differs from the checkpoint")
        if info.metadata.get("calibration_fingerprint") != metadata.get("calibration_fingerprint"):
            raise ValueError("Held-out camera/motor calibration differs from the checkpoint")
    dataset = SequenceDataset(held_out, sequence_length=sequence_length, stride=args.stride)
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)
    modes = ("actual_actions", "zero_actions", "permuted_actions", "persistence")
    metrics = {mode: Metrics(depth_scale) for mode in modes}
    all_labels_actual = Metrics(depth_scale)
    rng = np.random.default_rng(args.seed)
    windows_evaluated = 0
    shuffle_counts = {"eligible_pulses": 0, "changed_pulses": 0}
    uncertainty_pairs = []
    for batch_i, batch in enumerate(loader):
        if args.max_batches and batch_i >= args.max_batches:
            break
        batch = move_batch(batch, model.device)
        b, frames = batch["images"].shape[:2]
        anchor = model.initial_state(batch["images"][:, 0])
        for t in range(burn_in):
            anchor = model.advance_interval(anchor, batch["actions"][:, t], batch["dt"][:, t],
                                            batch["action_segments"][:, t], batch["segment_mask"][:, t])
            anchor = model.assimilate(anchor, batch["images"][:, t + 1])
        states = {mode: WorldState(anchor.h.clone(), anchor.z.clone()) for mode in modes[:-1]}
        exact_segments = batch["action_segments"]
        permuted, counts = permute_pulse_controls(exact_segments, batch["segment_mask"], burn_in=burn_in, rng=rng)
        for key, count in counts.items():
            shuffle_counts[key] += count
        zero = exact_segments.clone()
        zero[..., :2] = 0
        for offset, t in enumerate(range(burn_in, frames - 1)):
            target_image = batch["images"][:, t + 1]
            target_aux, all_valid = batch["aux"][:, t + 1], batch["aux_mask"][:, t + 1].bool()
            # One common label mask for a fair persistence/WM comparison.
            valid = all_valid & batch["aux_mask"][:, burn_in].bool()
            for mode, segment_array in (("actual_actions", exact_segments), ("zero_actions", zero),
                                        ("permuted_actions", permuted)):
                summary = (segment_array[:, t, :, :2] * segment_array[:, t, :, 2:3]
                           * batch["segment_mask"][:, t, :, None]).sum(1) / batch["dt"][:, t, None]
                states[mode] = model.advance_interval(states[mode], summary, batch["dt"][:, t],
                                                      segment_array[:, t], batch["segment_mask"][:, t])
                state = states[mode]
                predicted_aux = model.predictions(state).mean(0)
                predicted_image = model.decoder(state.z.flatten(0, 1)).reshape(
                    model.config.ensemble_size, b, 1, 128, 128).mean(0)
                metrics[mode].update(offset, predicted_image, target_image, predicted_aux, target_aux, valid)
                if mode == "actual_actions":
                    all_labels_actual.update(offset, predicted_image, target_image, predicted_aux, target_aux, all_valid)
                    ensemble = model.predictions(state)
                    err = (predicted_aux[:, 3:] - target_aux[:, 3:]).abs() * depth_scale
                    disagreement = ensemble[:, :, 3:].std(0, unbiased=False) * depth_scale
                    uncertainty_pairs.extend(zip(disagreement[all_valid[:, 3:]].cpu().tolist(), err[all_valid[:, 3:]].cpu().tolist()))
            metrics["persistence"].update(offset, batch["images"][:, burn_in], target_image,
                                           batch["aux"][:, burn_in], target_aux, valid)
        windows_evaluated += b
    values = {mode: metric.result() for mode, metric in metrics.items()}
    actual_mse = values["actual_actions"]["pixel_mse"]
    persistence_mse = values["persistence"]["pixel_mse"]
    common_actual = values["actual_actions"]["common_mask_aux_mae"]
    aux_deltas = {f"{mode}_minus_actual_aux_mae": values[mode]["common_mask_aux_mae"] - common_actual
                  if common_actual is not None and values[mode]["common_mask_aux_mae"] is not None else None
                  for mode in ("zero_actions", "permuted_actions", "persistence")}
    pairs = np.asarray(uncertainty_pairs, dtype=np.float64).reshape(-1, 2)
    correlation = float(np.corrcoef(pairs.T)[0, 1]) if len(pairs) > 2 and np.all(pairs.std(0) > 1e-12) else None
    report = {
        "checkpoint": str(args.checkpoint), "synthetic_checkpoint": bool(metadata.get("synthetic", True)),
        "held_out_sessions": sorted({info.split_group for info in held_out}),
        "evaluation_split": split, "held_out_axis": axis,
        "held_out_axis_values": sorted({info.group_value(axis) for info in held_out}),
        "excluded_training_episodes": len(infos) - len(held_out),
        "windows_evaluated": windows_evaluated, "sequence_length": sequence_length, "burn_in": burn_in,
        "metrics": values,
        "actual_actions_all_future_valid_labels": all_labels_actual.result(),
        "common_valid_label_mask": True, "shuffle": shuffle_counts,
        "uncertainty": {"clearance_valid_predictions": len(pairs), "disagreement_error_pearson": correlation,
                         "interpretation": "Ensemble disagreement is a heuristic; this is not a calibrated confidence bound."},
        "diagnostics": {
            "pixel_mse_improvement_vs_persistence": persistence_mse - actual_mse,
            "relative_pixel_mse_improvement_vs_persistence": (persistence_mse - actual_mse) / persistence_mse
            if persistence_mse > 1e-12 else None,
            "zero_minus_actual_pixel_mse": values["zero_actions"]["pixel_mse"] - actual_mse,
            "permuted_minus_actual_pixel_mse": values["permuted_actions"]["pixel_mse"] - actual_mse,
            **aux_deltas,
        },
        "interpretation": [
            "Positive improvements favour the actual-action model; negative values are failures on that metric.",
            "A static camera can make persistence very strong; low pixel error alone does not imply useful control.",
            "All primary aux comparisons use the same anchor-and-future valid mask; full-future actual-action errors are separate.",
            "Permuted actions shuffle nonzero pulse controls only: idle segments, durations and padding are unchanged.",
            "Test excludes validation used in checkpoint selection; --split validation is monitoring, not a final test.",
            "Clearance errors are relative to supplied labels, which may themselves be biased pseudo-labels.",
            "Overlapping windows are correlated; no statistical collision guarantee is inferred.",
            "Physical success rate, emergency-stop count and completion time require separate supervised trials.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"report": str(args.output), "windows": windows_evaluated,
                      "synthetic": report["synthetic_checkpoint"], "diagnostics": report["diagnostics"]}, ensure_ascii=False))
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("runs/evaluation.json"))
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--sequence-length", type=int)
    parser.add_argument("--burn-in", type=int)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--max-batches", type=int, default=0)
    parser.add_argument("--cpu-threads", type=int, default=2)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--split", choices=("test", "validation"), default="test")
    run_evaluation(parser.parse_args())


if __name__ == "__main__":
    main()
