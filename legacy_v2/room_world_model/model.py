"""Small action-conditioned visual dynamics ensemble.

The model predicts future latent states, camera images and supervised scene
quantities.  Its uncertainty is a disagreement heuristic, not a safety bound.
Current metric perception and the motor watchdog must remain independent.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import torch
from torch import Tensor, nn
from torch.nn import functional as F

from .data import ACTION_SEMANTICS, AUX_NAMES, CONTROL_REPRESENTATION, DATASET_SCHEMA_VERSION

CHECKPOINT_FORMAT = "room-world-model-v2"


@dataclass(frozen=True)
class ModelConfig:
    image_size: int = 128
    latent_dim: int = 96
    hidden_dim: int = 128
    ensemble_size: int = 3
    nominal_dt_s: float = 0.1
    max_dt_s: float = 5.0
    image_loss_weight: float = 1.0
    latent_loss_weight: float = 0.25
    auxiliary_loss_weight: float = 2.0
    multistep_loss_weight: float = 1.0

    def __post_init__(self) -> None:
        if self.image_size != 128:
            raise ValueError("This encoder/decoder requires 128 x 128 images")
        if self.latent_dim < 8 or self.hidden_dim < 8 or self.ensemble_size < 2:
            raise ValueError("latent/hidden dimensions >= 8 and ensemble_size >= 2 required")
        if not 0 < self.nominal_dt_s <= self.max_dt_s:
            raise ValueError("Require 0 < nominal_dt_s <= max_dt_s")
        import math
        if any(not math.isfinite(x) or x <= 0 for x in (self.image_loss_weight, self.latent_loss_weight,
                                                        self.auxiliary_loss_weight, self.multistep_loss_weight)):
            raise ValueError("All training loss weights must be finite and positive")


@dataclass
class WorldState:
    """Ensemble-major state: h[E,B,H], z[E,B,Z]."""

    h: Tensor
    z: Tensor

    def detach(self) -> "WorldState":
        return WorldState(self.h.detach(), self.z.detach())

    def repeat_batch(self, repeats: int) -> "WorldState":
        if self.h.shape[1] != 1:
            raise ValueError("Candidate expansion requires a single observed robot state")
        return WorldState(self.h.repeat(1, repeats, 1), self.z.repeat(1, repeats, 1))


class Encoder(nn.Module):
    def __init__(self, latent_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 24, 4, 2, 1), nn.SiLU(),
            nn.Conv2d(24, 48, 4, 2, 1), nn.SiLU(),
            nn.Conv2d(48, 64, 4, 2, 1), nn.SiLU(),
            nn.Conv2d(64, 96, 4, 2, 1), nn.SiLU(),
            nn.Flatten(), nn.Linear(96 * 8 * 8, latent_dim), nn.LayerNorm(latent_dim),
            nn.Tanh(),
        )

    def forward(self, image: Tensor) -> Tensor:
        return self.net(image)


class Decoder(nn.Module):
    def __init__(self, latent_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(latent_dim, 96 * 8 * 8), nn.SiLU(), nn.Unflatten(1, (96, 8, 8)),
            nn.ConvTranspose2d(96, 64, 4, 2, 1), nn.SiLU(),
            nn.ConvTranspose2d(64, 48, 4, 2, 1), nn.SiLU(),
            nn.ConvTranspose2d(48, 24, 4, 2, 1), nn.SiLU(),
            nn.ConvTranspose2d(24, 1, 4, 2, 1), nn.Sigmoid(),
        )

    def forward(self, z: Tensor) -> Tensor:
        return self.net(z)


class DynamicsMember(nn.Module):
    def __init__(self, config: ModelConfig):
        super().__init__()
        # z, setpoints, elapsed time, setpoint * elapsed time.
        self.cell = nn.GRUCell(config.latent_dim + 5, config.hidden_dim)
        self.prior = nn.Sequential(
            nn.Linear(config.hidden_dim, config.hidden_dim), nn.SiLU(),
            nn.Linear(config.hidden_dim, config.latent_dim), nn.Tanh(),
        )
        self.aux = nn.Sequential(
            nn.Linear(config.hidden_dim + config.latent_dim, config.hidden_dim),
            nn.SiLU(), nn.Linear(config.hidden_dim, len(AUX_NAMES)),
        )


def aux_from_logits(logits: Tensor) -> Tensor:
    result = logits.sigmoid()
    # Avoid in-place modification of sigmoid output needed by its backward.
    return torch.cat((result[..., :1], logits[..., 1:2].tanh(), result[..., 2:]), dim=-1)


def masked_aux_loss(logits: Tensor, target: Tensor, mask: Tensor) -> Tensor:
    """Return per-sample loss. Unknown targets never become free-space labels."""
    pred = aux_from_logits(logits)
    visible = F.binary_cross_entropy_with_logits(logits[:, :1], target[:, :1], reduction="none")
    other = F.smooth_l1_loss(pred[:, 1:], target[:, 1:], reduction="none", beta=0.1)
    # Overestimating a clearance is more costly than underestimating it.
    unsafe_overestimate = F.relu(pred[:, 3:] - target[:, 3:]).square()
    other = torch.cat((other[:, :2], other[:, 2:] + 2.0 * unsafe_overestimate), dim=1)
    terms = torch.cat((visible, other), dim=1) * mask.to(logits.dtype)
    return terms.sum(dim=-1) / mask.sum(dim=-1).clamp_min(1)


class WorldModel(nn.Module):
    def __init__(self, config: ModelConfig | None = None):
        super().__init__()
        self.config = config or ModelConfig()
        self.encoder = Encoder(self.config.latent_dim)
        self.decoder = Decoder(self.config.latent_dim)
        self.members = nn.ModuleList(DynamicsMember(self.config) for _ in range(self.config.ensemble_size))
        self.checkpoint_metadata: dict[str, Any] = {}

    @property
    def device(self) -> torch.device:
        return next(self.parameters()).device

    def encode(self, images: Tensor) -> Tensor:
        if images.ndim != 4 or tuple(images.shape[1:]) != (1, 128, 128):
            raise ValueError("images must be float [batch,1,128,128], normalized to [0,1]")
        if not torch.is_floating_point(images):
            raise ValueError("Normalize uint8 images to float32 / 255 before encoding")
        return self.encoder(images)

    def initial_state(self, images: Tensor) -> WorldState:
        z = self.encode(images)
        e, b = self.config.ensemble_size, images.shape[0]
        return WorldState(z.new_zeros((e, b, self.config.hidden_dim)), z.unsqueeze(0).expand(e, -1, -1))

    def assimilate(self, state: WorldState, images: Tensor) -> WorldState:
        """Replace the predicted appearance with a new measurement, keeping history."""
        z = self.encode(images)
        if z.shape[0] != state.h.shape[1]:
            raise ValueError("Observation and state batch sizes differ")
        return WorldState(state.h, z.unsqueeze(0).expand(self.config.ensemble_size, -1, -1))

    def advance(self, state: WorldState, actions: Tensor, dt: Tensor | float) -> WorldState:
        """Predict through one interval of acknowledged controls.

        dt may be fractional; uncertainty in latency compensation is still a
        separate reason to stop. Subdivide intervals longer than max_dt_s at the
        caller, and reset on gaps rather than inventing unobserved controls.
        """
        b = state.h.shape[1]
        actions = torch.as_tensor(actions, device=state.h.device, dtype=state.h.dtype)
        if actions.shape != (b, 2):
            raise ValueError(f"actions must have shape {(b, 2)}, got {tuple(actions.shape)}")
        dt = torch.as_tensor(dt, device=state.h.device, dtype=state.h.dtype)
        if dt.ndim == 0:
            dt = dt.expand(b)
        if dt.shape != (b,):
            raise ValueError(f"dt must be a scalar or [{b}]")
        if not torch.isfinite(actions).all() or not torch.isfinite(dt).all():
            raise ValueError("Nonfinite command or dt")
        if torch.any(actions.abs() > 1.00001):
            raise ValueError("Normalized controls must lie in [-1,1]")
        if torch.any(dt < 0) or torch.any(dt > self.config.max_dt_s + 1e-6):
            raise ValueError("dt outside model interval limits; stop/reset or subdivide known controls")
        scaled_dt = (dt / self.config.nominal_dt_s).unsqueeze(-1)
        control = torch.cat((actions, torch.log1p(scaled_dt), actions * scaled_dt), dim=-1)
        hs, zs = [], []
        for i, member in enumerate(self.members):
            h = member.cell(torch.cat((state.z[i], control), dim=-1), state.h[i])
            z = member.prior(h)
            # A zero-duration segment is exactly an identity operation.
            hs.append(torch.where(dt[:, None] > 0, h, state.h[i]))
            zs.append(torch.where(dt[:, None] > 0, z, state.z[i]))
        return WorldState(torch.stack(hs), torch.stack(zs))

    def observe(self, images: Tensor, state: WorldState | None = None,
                action: Tensor | None = None, dt: Tensor | float | None = None) -> WorldState:
        if state is None:
            return self.initial_state(images)
        if action is None or dt is None:
            raise ValueError("Existing state requires the interval's effective action and dt")
        return self.assimilate(self.advance(state, action, dt), images)

    def advance_interval(self, state: WorldState, actions: Tensor, dt: Tensor | float,
                         action_segments: Tensor | None = None,
                         segment_mask: Tensor | None = None) -> WorldState:
        """Replay exact PWM setpoint chronology between camera exposures.

        action_segments is [B,K,3]: forward, yaw, duration. Padded entries have
        mask=False and are identity operations. A 0.08 s pulse followed by
        0.65 s stopped is different from a constant interval-mean command.
        The dataset validates chronology, total duration and interval means.
        """
        if action_segments is None:
            raise ValueError("v2 camera transitions require confirmed action_segments; use advance for one known segment")
        segments = torch.as_tensor(action_segments, device=state.h.device, dtype=state.h.dtype)
        if segments.ndim != 3 or segments.shape[0] != state.h.shape[1] or segments.shape[-1] != 3:
            raise ValueError("action_segments must be [batch,segments,3]")
        if segment_mask is None:
            raise ValueError("Exact action segments require a segment_mask")
        mask = torch.as_tensor(segment_mask, device=state.h.device, dtype=torch.bool)
        if mask.shape != segments.shape[:2]:
            raise ValueError("segment_mask shape differs from action_segments")
        interval_dt = torch.as_tensor(dt, device=state.h.device, dtype=state.h.dtype)
        if interval_dt.ndim == 0:
            interval_dt = interval_dt.expand(state.h.shape[1])
        if interval_dt.shape != (state.h.shape[1],) or not torch.isfinite(interval_dt).all():
            raise ValueError("Interval dt must be finite scalar or [batch]")
        if torch.any(interval_dt < 0) or torch.any(interval_dt > self.config.max_dt_s + 1e-6):
            raise ValueError("Camera interval exceeds model limits; stop/reset across gaps")
        if not torch.isfinite(segments).all() or torch.any(mask & (segments[..., 2] <= 0)):
            raise ValueError("Active segments require finite controls and positive durations")
        if torch.any(mask[:, 1:] & ~mask[:, :-1]):
            raise ValueError("Active action segments must form a prefix")
        covered_dt = torch.where(mask, segments[..., 2], 0.0).sum(1)
        if not torch.allclose(covered_dt, interval_dt, rtol=1e-4, atol=0.002):
            raise ValueError("Action history does not completely cover the camera interval")
        max_active = int(mask.sum(1).max())
        for k in range(max_active):
            segment_dt = torch.where(mask[:, k], segments[:, k, 2], 0.0)
            controls = torch.where(mask[:, k, None], segments[:, k, :2], 0.0)
            state = self.advance(state, controls, segment_dt)
        return state

    def aux_logits(self, state: WorldState) -> Tensor:
        return torch.stack([m.aux(torch.cat((state.h[i], state.z[i]), dim=-1))
                            for i, m in enumerate(self.members)])

    def predictions(self, state: WorldState) -> Tensor:
        return aux_from_logits(self.aux_logits(state))

    def rollout(self, state: WorldState, actions: Tensor,
                dt: Tensor | float) -> tuple[Tensor, WorldState]:
        """Return aux[E,B,T,6] and final state for controls[B,T,2]."""
        actions = torch.as_tensor(actions, dtype=state.h.dtype, device=state.h.device)
        if actions.ndim != 3 or actions.shape[0] != state.h.shape[1] or actions.shape[-1] != 2:
            raise ValueError("rollout actions must be [batch,horizon,2]")
        b, horizon, _ = actions.shape
        if horizon < 1:
            raise ValueError("horizon must be positive")
        dt = torch.as_tensor(dt, dtype=state.h.dtype, device=state.h.device)
        if dt.ndim == 0:
            dt = dt.expand(b, horizon)
        if dt.shape != (b, horizon):
            raise ValueError("rollout dt must be scalar or [batch,horizon]")
        predictions = []
        for t in range(horizon):
            state = self.advance(state, actions[:, t], dt[:, t])
            predictions.append(self.predictions(state))
        return torch.stack(predictions, dim=2), state

    def training_loss(self, batch: dict[str, Tensor], *, burn_in: int = 2,
                      bootstrap_probability: float = 0.8) -> tuple[Tensor, dict[str, float]]:
        """One-step + open-loop multi-step prediction and image reconstruction.

        Teacher-forced filtering builds history from real frames. The second
        rollout feeds predictions back to the model; that term actually trains
        action-conditioned imagined futures, not just an image autoencoder.
        Independent Bernoulli sequence masks bootstrap the ensemble members.
        """
        images, actions, dts = batch["images"], batch["actions"], batch["dt"]
        if "action_segments" not in batch or "segment_mask" not in batch:
            raise ValueError("v2 training requires ordered confirmed action segments")
        aux, aux_mask = batch["aux"], batch["aux_mask"].bool()
        b, frames = images.shape[:2]
        horizon = frames - 1
        if horizon < 2 or not 0 <= burn_in < horizon:
            raise ValueError("Need at least two transitions and burn_in < sequence_length")
        encoded = self.encode(images.flatten(0, 1)).reshape(b, frames, -1)
        reconstructed = self.decoder(encoded.flatten(0, 1)).reshape_as(images)
        reconstruction = F.smooth_l1_loss(reconstructed, images)
        e = self.config.ensemble_size
        if self.training and bootstrap_probability < 1:
            weights = (torch.rand((e, b), device=images.device) < bootstrap_probability).float()
            # No empty member batch; preserve at least one distinct sequence.
            for i in range(e):
                if weights[i].sum() == 0:
                    weights[i, i % b] = 1
        else:
            weights = images.new_ones((e, b))

        def reduce_members(values: Tensor) -> Tensor:
            return ((values * weights).sum(1) / weights.sum(1).clamp_min(1)).mean()

        def prediction_loss(state: WorldState, target_t: int) -> tuple[Tensor, Tensor, Tensor]:
            target_z = encoded[:, target_t].detach().unsqueeze(0)
            latent = (state.z - target_z).square().mean(-1)
            prediction = self.decoder(state.z.flatten(0, 1)).reshape(e, b, 1, 128, 128)
            pixel = F.smooth_l1_loss(
                prediction, images[:, target_t].unsqueeze(0).expand_as(prediction), reduction="none"
            ).mean((2, 3, 4))
            logits = self.aux_logits(state)
            a_loss = torch.stack([masked_aux_loss(logits[i], aux[:, target_t], aux_mask[:, target_t])
                                  for i in range(e)])
            return reduce_members(latent), reduce_members(pixel), reduce_members(a_loss)

        state = WorldState(encoded.new_zeros((e, b, self.config.hidden_dim)),
                           encoded[:, 0].unsqueeze(0).expand(e, -1, -1))
        anchor = state if burn_in == 0 else None
        one_step = images.new_zeros(())
        latent_terms = images.new_zeros(())
        pixel_terms = images.new_zeros(())
        aux_terms = images.new_zeros(())
        counted = 0
        c = self.config

        def combine(latent, pixel, supervised):
            return c.latent_loss_weight * latent + c.image_loss_weight * pixel + c.auxiliary_loss_weight * supervised

        for t in range(horizon):
            prior = self.advance_interval(
                state, actions[:, t], dts[:, t],
                batch.get("action_segments", None)[:, t] if "action_segments" in batch else None,
                batch.get("segment_mask", None)[:, t] if "segment_mask" in batch else None,
            )
            if t >= burn_in:
                lat, pix, sup = prediction_loss(prior, t + 1)
                one_step = one_step + combine(lat, pix, sup)
                latent_terms, pixel_terms, aux_terms = latent_terms + lat, pixel_terms + pix, aux_terms + sup
                counted += 1
            state = WorldState(prior.h, encoded[:, t + 1].unsqueeze(0).expand(e, -1, -1))
            if t + 1 == burn_in:
                anchor = state
        assert anchor is not None
        open_loop = images.new_zeros(())
        imagined = anchor
        for t in range(burn_in, horizon):
            imagined = self.advance_interval(
                imagined, actions[:, t], dts[:, t],
                batch.get("action_segments", None)[:, t] if "action_segments" in batch else None,
                batch.get("segment_mask", None)[:, t] if "segment_mask" in batch else None,
            )
            lat, pix, sup = prediction_loss(imagined, t + 1)
            open_loop = open_loop + combine(lat, pix, sup)
        total = c.image_loss_weight * reconstruction + one_step / counted + c.multistep_loss_weight * open_loop / counted
        metrics = {
            "loss": float(total.detach()), "reconstruction": float(reconstruction.detach()),
            "one_step": float((one_step / counted).detach()),
            "open_loop": float((open_loop / counted).detach()),
            "latent": float((latent_terms / counted).detach()),
            "future_pixels": float((pixel_terms / counted).detach()),
            "auxiliary": float((aux_terms / counted).detach()),
        }
        return total, metrics


def save_checkpoint(path: str | Path, model: WorldModel, metadata: dict[str, Any], *,
                    optimizer_state: dict[str, Any] | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "format": CHECKPOINT_FORMAT, "config": asdict(model.config),
        "state_dict": model.state_dict(), "metadata": metadata,
    }
    if optimizer_state is not None:
        payload["optimizer_state"] = optimizer_state
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)


def check_live_metadata(metadata: dict[str, Any], *, calibration_fingerprint: str | None = None,
                        depth_scale_m: float | None = None) -> None:
    """Necessary provenance checks; supervised acceptance is still external."""
    if metadata.get("synthetic") is not False:
        raise ValueError("Synthetic or unmarked checkpoint cannot control the real robot")
    if metadata.get("provenance_complete") is not True:
        raise ValueError("Checkpoint source metadata is incomplete")
    if metadata.get("calibration_confirmed") is not True:
        raise ValueError("Checkpoint recordings lack confirmed motor/camera calibration")
    if metadata.get("limited_batch_smoke", False):
        raise ValueError("Limited-batch smoke checkpoints cannot control the real robot")
    if metadata.get("uncalibrated_pseudo_labels") is not False or metadata.get("labels_from_accepted_calibration") is not True:
        raise ValueError("Checkpoint labels were not produced with accepted metric calibration")
    if metadata.get("label_fingerprint_verified") is not True:
        raise ValueError("Checkpoint label configuration fingerprint was not verified against its recordings")
    if metadata.get("action_semantics") != ACTION_SEMANTICS:
        raise ValueError("Checkpoint action semantics do not match acknowledged motor setpoints")
    if metadata.get("control_representation") != CONTROL_REPRESENTATION:
        raise ValueError("Live pulse control requires training on exact acknowledged action segments")
    if type(metadata.get("dataset_schema_version")) is not int or metadata["dataset_schema_version"] != DATASET_SCHEMA_VERSION:
        raise ValueError("Live v2 checkpoints require schema-v2 datasets; retrain legacy v1 checkpoints")
    if metadata.get("mcu_confirmation_validated") is not True:
        raise ValueError("Live checkpoint lacks validated matched MCU START/STOP action timing")
    saved_calibration = metadata.get("calibration_fingerprint")
    if not saved_calibration or saved_calibration in {"unknown", "synthetic"}:
        raise ValueError("Checkpoint lacks real camera/motor calibration provenance")
    if calibration_fingerprint is not None and saved_calibration != calibration_fingerprint:
        raise ValueError("Checkpoint and live camera/motor calibration differ")
    if depth_scale_m is not None and abs(float(metadata.get("depth_scale_m", -1)) - depth_scale_m) > 1e-6:
        raise ValueError("Checkpoint depth normalization differs from live perception")
    if metadata.get("held_out_validation") is not True:
        raise ValueError("Checkpoint has no separate held-out episode validation")


def load_checkpoint(path: str | Path, device: str | torch.device = "cpu", *, for_live: bool = False,
                    calibration_fingerprint: str | None = None,
                    depth_scale_m: float | None = None) -> tuple[WorldModel, dict[str, Any]]:
    payload = torch.load(Path(path), map_location=device, weights_only=True)
    if payload.get("format") != CHECKPOINT_FORMAT:
        raise ValueError("Unsupported world-model checkpoint format: v2 requires retraining on v2 MCU-confirmed "
                         "datasets. Renaming a v1 format tag cannot correct action timing/provenance.")
    metadata = dict(payload.get("metadata", {}))
    if for_live:
        check_live_metadata(metadata, calibration_fingerprint=calibration_fingerprint,
                            depth_scale_m=depth_scale_m)
    model = WorldModel(ModelConfig(**payload["config"]))
    model.load_state_dict(payload["state_dict"], strict=True)
    model.to(device).eval()
    model.checkpoint_metadata = metadata
    return model, metadata
