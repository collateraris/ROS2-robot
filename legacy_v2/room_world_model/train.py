"""Train the compact world model on recorded chronological episodes."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import hashlib
import math
from pathlib import Path
import random
from typing import Any

import numpy as np
import torch
from torch.utils.data import DataLoader

from .data import (AUX_NAMES, CONTROL_REPRESENTATION, DATASET_SCHEMA_VERSION, EpisodeInfo,
                   SequenceDataset, discover_episodes, episode_content_sha256, file_sha256,
                   partition_episodes, split_audit)
from .model import ACTION_SEMANTICS, ModelConfig, WorldModel, WorldState, save_checkpoint


def move_batch(batch: dict[str, torch.Tensor], device: torch.device) -> dict[str, torch.Tensor]:
    return {key: value.to(device, non_blocking=True) for key, value in batch.items()}


def photometric_augmentation(batch: dict[str, torch.Tensor], strength: float) -> dict[str, torch.Tensor]:
    """One brightness/gamma draw per sequence, shared by every frame."""
    if not 0 <= strength <= 0.4:
        raise ValueError("Photometric strength must lie in [0,0.4]")
    if strength == 0:
        return batch
    images = batch["images"]
    shape = (images.shape[0], 1, 1, 1, 1)
    brightness = 1 + (2 * torch.rand(shape, device=images.device) - 1) * strength
    gamma = 1 + (2 * torch.rand(shape, device=images.device) - 1) * strength
    return {**batch, "images": (images.clamp(0, 1).pow(gamma) * brightness).clamp(0, 1)}


@torch.no_grad()
def evaluate(model: WorldModel, loader: DataLoader, *, burn_in: int, max_batches: int = 0,
             depth_scale_m: float = 3.0) -> dict[str, Any]:
    model.eval()
    weighted_loss = 0.0
    samples = 0
    absolute_errors = np.zeros(6, dtype=np.float64)
    valid_counts = np.zeros(6, dtype=np.int64)
    positive_clearance_errors = np.zeros(3, dtype=np.float64)
    near_count, false_clear_count = 0, 0
    action_sensitivity_sum, action_sensitivity_count = 0.0, 0
    horizon_error_sum: list[float] = []
    horizon_count: list[int] = []
    for batch_i, batch in enumerate(loader):
        if max_batches and batch_i >= max_batches:
            break
        batch = move_batch(batch, model.device)
        loss, _ = model.training_loss(batch, burn_in=burn_in, bootstrap_probability=1)
        b, frames = batch["images"].shape[:2]
        weighted_loss += float(loss) * b
        samples += b
        state = model.initial_state(batch["images"][:, 0])
        for t in range(burn_in):
            state = model.advance_interval(state, batch["actions"][:, t], batch["dt"][:, t],
                                           batch["action_segments"][:, t], batch["segment_mask"][:, t])
            state = model.assimilate(state, batch["images"][:, t + 1])
        counterfactual = WorldState(state.h.clone(), state.z.clone())
        for offset, t in enumerate(range(burn_in, frames - 1)):
            state = model.advance_interval(state, batch["actions"][:, t], batch["dt"][:, t],
                                           batch["action_segments"][:, t], batch["segment_mask"][:, t])
            predicted = model.predictions(state).mean(0)
            target, mask = batch["aux"][:, t + 1], batch["aux_mask"][:, t + 1].bool()
            errors = (predicted - target).abs()
            absolute_errors += (errors * mask).sum(0).cpu().numpy()
            valid_counts += mask.sum(0).cpu().numpy()
            clearance_error = torch.relu(predicted[:, 3:] - target[:, 3:]) * mask[:, 3:]
            positive_clearance_errors += clearance_error.sum(0).cpu().numpy()
            threshold = 0.6 / depth_scale_m
            near = (target[:, 3:] < threshold) & mask[:, 3:]
            false_clear = near & (predicted[:, 3:] >= threshold)
            near_count += int(near.sum())
            false_clear_count += int(false_clear.sum())
            if offset >= len(horizon_error_sum):
                horizon_error_sum.append(0.0)
                horizon_count.append(0)
            horizon_error_sum[offset] += float((errors * mask).sum())
            horizon_count[offset] += int(mask.sum())
            if batch_i == 0:
                altered = batch["action_segments"][:, t].clone()
                altered[..., :2] = -altered[..., :2]
                counterfactual = model.advance_interval(counterfactual, -batch["actions"][:, t], batch["dt"][:, t],
                                                       altered, batch["segment_mask"][:, t])
                sensitivity = (model.predictions(counterfactual).mean(0) - predicted).abs().mean()
                action_sensitivity_sum += float(sensitivity)
                action_sensitivity_count += 1
    if samples == 0:
        raise ValueError("Empty validation loader")
    return {
        "loss": weighted_loss / samples,
        "windows_evaluated": samples,
        "aux_mae": {name: float(absolute_errors[i] / valid_counts[i]) if valid_counts[i] else None
                    for i, name in enumerate(AUX_NAMES)},
        "aux_valid_counts": {name: int(valid_counts[i]) for i, name in enumerate(AUX_NAMES)},
        "clearance_overestimate_mae_m": {
            AUX_NAMES[i + 3]: float(positive_clearance_errors[i] / valid_counts[i + 3] * depth_scale_m)
            if valid_counts[i + 3] else None for i in range(3)
        },
        "near_obstacle_sector_count_below_0_6m": near_count,
        "false_clearance_rate_at_0_6m": false_clear_count / near_count if near_count else None,
        "open_loop_aux_mae_by_future_frame": [error / count if count else None
                                             for error, count in zip(horizon_error_sum, horizon_count)],
        "action_perturbation_mean_prediction_delta": action_sensitivity_sum / max(action_sensitivity_count, 1),
        "interpretation": "Aux errors use masked labels; disagreement/sensitivity are not safety certificates. "
                          "Overlapping windows are correlated. No physical collision rate is inferred.",
    }


def build_metadata(train_infos: list[EpisodeInfo], validation_infos: list[EpisodeInfo],
                   test_infos: list[EpisodeInfo] | None = None, *, split_by: str = "session") -> dict[str, Any]:
    test_infos = test_infos or []
    infos = train_infos + validation_infos + test_infos
    partitions = {"train": train_infos, "validation": validation_infos, "test": test_infos}
    audit = split_audit(partitions, split_by)
    for overlap in audit["overlaps"].values():
        if overlap["session"] or overlap[split_by]:
            raise ValueError("Dataset partitions overlap on session or selected hold-out axis")
    depth_scales = {float(info.metadata.get("depth_scale_m", 3.0)) for info in infos}
    if len(depth_scales) != 1:
        raise ValueError("Episode depth_scale_m values differ; relabel consistently before training")
    calibrations = {str(info.metadata.get("calibration_fingerprint", "unknown")) for info in infos}
    if len(calibrations) != 1:
        raise ValueError("Episode camera/motor calibrations differ; train a separate model or explicitly harmonize data")
    synthetic = any(info.metadata.get("synthetic", True) for info in infos)
    if any(info.metadata.get("action_semantics", ACTION_SEMANTICS) != ACTION_SEMANTICS for info in infos):
        raise ValueError("Mixed action semantics in dataset")
    provenance = [{"name": info.path.name, "sha256": file_sha256(info.path),
                   "content_sha256": episode_content_sha256(info.path), "split": name,
                   "session": info.split_group,
                   "layout": info.metadata.get("layout_id", "unknown"),
                   "day": info.metadata.get("day_id", "unknown")} for name, values in partitions.items() for info in values]
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "research_only": True, "synthetic": synthetic,
        "calibration_confirmed": all(info.metadata.get("calibration_confirmed", False) for info in infos),
        "provenance_complete": all(all(key in info.metadata for key in (
            "synthetic", "depth_scale_m", "calibration_fingerprint", "action_semantics", "schema_version")) for info in infos),
        "dataset_schema_version": DATASET_SCHEMA_VERSION,
        "mcu_confirmation_validated": all(info.metadata.get("mcu_confirmation_validated") is True for info in infos),
        "uncalibrated_pseudo_labels": any(info.metadata.get("uncalibrated_pseudo_labels", True) for info in infos),
        "labels_from_accepted_calibration": all(info.metadata.get("labels_from_accepted_calibration") is True
                                                 for info in infos),
        "label_fingerprint_verified": all(info.metadata.get("label_fingerprint_verified") is True
                                            for info in infos),
        "action_semantics": ACTION_SEMANTICS,
        "control_representation": CONTROL_REPRESENTATION,
        "depth_scale_m": depth_scales.pop(), "calibration_fingerprint": calibrations.pop(),
        "action_scale": train_infos[0].metadata.get("action_scale", {
            key: train_infos[0].metadata.get("robot", {}).get(key) for key in (
                "linear_command_scale_mps", "angular_command_scale_rps", "wheel_command_scale_mps",
                "track_width_m", "pwm_limit")}),
        "held_out_validation": True,
        "train_sessions": sorted({info.split_group for info in train_infos}),
        "validation_sessions": sorted({info.split_group for info in validation_infos}),
        "test_sessions": sorted({info.split_group for info in test_infos}),
        "split_by": split_by, "split_audit": audit,
        "train_split_values": sorted({info.group_value(split_by) for info in train_infos}),
        "validation_split_values": sorted({info.group_value(split_by) for info in validation_infos}),
        "test_split_values": sorted({info.group_value(split_by) for info in test_infos}),
        "train_episode_count": len(train_infos), "validation_episode_count": len(validation_infos),
        "train_aux_label_counts": {name: sum(info.aux_valid_counts[i] for info in train_infos)
                                   for i, name in enumerate(AUX_NAMES)},
        "episode_provenance": provenance,
        "dataset_manifest_sha256": hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest(),
        "live_acceptance": False,
        "limitations": "Uncalibrated ensemble disagreement; automatic labels may be wrong; "
                       "requires independent current-observation guards and supervised physical acceptance.",
    }


def train(args: argparse.Namespace) -> dict[str, Any]:
    if args.epochs < 1 or args.batch_size < 1:
        raise ValueError("epochs and batch_size must be positive")
    if not 0 <= args.burn_in < args.sequence_length:
        raise ValueError("burn_in must lie in [0, sequence_length)")
    if (args.max_train_batches < 0 or args.max_val_batches < 0 or args.workers < 0 or args.cpu_threads < 1
            or args.patience < 0 or not math.isfinite(args.grad_clip) or args.grad_clip <= 0
            or not math.isfinite(args.lr) or args.lr <= 0):
        raise ValueError("Invalid training limits, worker count, learning rate or gradient clip")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.set_num_threads(args.cpu_threads)
    device_name = ("cuda" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    device = torch.device(device_name)
    partitions = partition_episodes(discover_episodes(args.data),
                                    validation_fraction=args.validation_fraction, test_fraction=args.test_fraction,
                                    split_by=args.split_by, seed=args.seed, max_dt_s=args.max_dt)
    train_infos, val_infos = partitions["train"], partitions["validation"]
    metadata = build_metadata(train_infos, val_infos, partitions["test"], split_by=args.split_by)
    metadata["source_files_sha256"] = {path.name:file_sha256(path) for path in sorted(Path(__file__).parent.glob("*.py"))}
    metadata["training_seed"] = args.seed
    metadata["limited_batch_smoke"] = bool(args.max_train_batches or args.max_val_batches)
    train_data = SequenceDataset(train_infos, sequence_length=args.sequence_length, stride=args.stride, max_dt_s=args.max_dt)
    val_data = SequenceDataset(val_infos, sequence_length=args.sequence_length, stride=args.stride, max_dt_s=args.max_dt)
    pin_memory = device.type == "cuda"
    train_loader = DataLoader(train_data, batch_size=args.batch_size, shuffle=True, num_workers=args.workers,
                              pin_memory=pin_memory, drop_last=False)
    val_loader = DataLoader(val_data, batch_size=args.batch_size, shuffle=False, num_workers=args.workers,
                            pin_memory=pin_memory, drop_last=False)
    config = ModelConfig(latent_dim=args.latent_dim, hidden_dim=args.hidden_dim, ensemble_size=args.ensemble_size,
                         nominal_dt_s=args.nominal_dt, max_dt_s=args.max_dt,
                         image_loss_weight=args.image_loss_weight, latent_loss_weight=args.latent_loss_weight,
                         auxiliary_loss_weight=args.auxiliary_loss_weight, multistep_loss_weight=args.multistep_loss_weight)
    model = WorldModel(config).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-5)
    use_amp = device.type == "cuda" and args.mixed_precision
    scaler = torch.amp.GradScaler("cuda", enabled=use_amp)
    args.output.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "model_config": asdict(config), "metadata": metadata,
        "training_config": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "train_windows": len(train_data), "validation_windows": len(val_data), "epochs": [],
        "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
        "test_used_for_selection": False, "torch_version": str(torch.__version__), "mixed_precision": use_amp,
    }
    best, stale_epochs = math.inf, 0
    for epoch in range(1, args.epochs + 1):
        model.train()
        total, seen = 0.0, 0
        train_metrics: dict[str, float] = {}
        for batch_i, batch in enumerate(train_loader):
            if args.max_train_batches and batch_i >= args.max_train_batches:
                break
            batch = move_batch(batch, device)
            batch = photometric_augmentation(batch, args.photometric_strength)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=use_amp):
                loss, metrics = model.training_loss(batch, burn_in=args.burn_in)
            if not torch.isfinite(loss):
                raise FloatingPointError("Nonfinite world-model loss; checkpoint not promoted")
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip, error_if_nonfinite=True)
            scaler.step(optimizer)
            scaler.update()
            n = len(batch["images"])
            total, seen = total + float(loss.detach()) * n, seen + n
            for key, value in metrics.items():
                train_metrics[key] = train_metrics.get(key, 0.0) + value * n
        validation = evaluate(model, val_loader, burn_in=args.burn_in, max_batches=args.max_val_batches,
                              depth_scale_m=metadata["depth_scale_m"])
        record = {"epoch": epoch, "train": {key: value / max(seen, 1) for key, value in train_metrics.items()},
                  "validation": validation}
        report["epochs"].append(record)
        checkpoint_metadata = {**metadata, "epoch": epoch, "validation": validation,
                               "burn_in_frames": args.burn_in, "sequence_length": args.sequence_length}
        save_checkpoint(args.output / "last.pt", model, checkpoint_metadata, optimizer_state=optimizer.state_dict())
        if validation["loss"] < best:
            best = validation["loss"]
            stale_epochs = 0
            report["best_epoch"] = epoch
            save_checkpoint(args.output / "best.pt", model, checkpoint_metadata)
        else:
            stale_epochs += 1
        if args.patience and stale_epochs >= args.patience:
            report["early_stopped_at_epoch"] = epoch
        temporary = args.output / "report.json.tmp"
        temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        temporary.replace(args.output / "report.json")
        print(json.dumps({"epoch": epoch, "train_loss": total / max(seen, 1),
                          "validation_loss": validation["loss"], "synthetic": metadata["synthetic"]}), flush=True)
        if "early_stopped_at_epoch" in report:
            break
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("runs/world_model"))
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--sequence-length", type=int, default=6)
    parser.add_argument("--burn-in", type=int, default=2)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--latent-dim", type=int, default=96)
    parser.add_argument("--hidden-dim", type=int, default=128)
    parser.add_argument("--ensemble-size", type=int, default=3)
    parser.add_argument("--nominal-dt", type=float, default=0.1)
    parser.add_argument("--max-dt", type=float, default=5.0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--cpu-threads", type=int, default=2)
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--test-fraction", type=float, default=0.2)
    parser.add_argument("--split-by", choices=("session", "layout", "day"), default="layout")
    parser.add_argument("--photometric-strength", type=float, default=0.08)
    parser.add_argument("--patience", type=int, default=8, help="Validation early stopping; 0 disables")
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--mixed-precision", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--image-loss-weight", type=float, default=1.0)
    parser.add_argument("--latent-loss-weight", type=float, default=0.25)
    parser.add_argument("--auxiliary-loss-weight", type=float, default=2.0)
    parser.add_argument("--multistep-loss-weight", type=float, default=1.0)
    parser.add_argument("--max-train-batches", type=int, default=0, help="0=all; small values only for smoke tests")
    parser.add_argument("--max-val-batches", type=int, default=0, help="0=all; small values only for smoke tests")
    return parser


def main() -> None:
    train(build_parser().parse_args())


if __name__ == "__main__":
    main()
