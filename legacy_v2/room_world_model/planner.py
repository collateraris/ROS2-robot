"""Short-horizon CEM over pulse / stop / observe action sequences.

Only the first bounded pulse is proposed. Execution requires the independent
current-frame guard, fresh exposure verification, operator enable and watchdog.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Callable

import numpy as np
import torch

from .model import WorldModel, WorldState, check_live_metadata


@dataclass(frozen=True)
class PlannerConfig:
    horizon: int = 3
    pulse_s: float = 0.08
    observe_wait_s: float = 2.50
    candidates: int = 96
    iterations: int = 3
    elite: int = 12
    max_forward: float = 0.30
    max_yaw: float = 0.35
    max_forward_change: float = 0.20
    max_yaw_change: float = 0.25
    depth_scale_m: float = 3.0
    stop_distance_m: float = 0.8
    clearance_m: float = 0.6
    clearance_error_margin_m: float = 0.15
    max_aux_disagreement: float = 0.12
    seed: int = 17

    def __post_init__(self) -> None:
        if self.horizon < 1 or self.iterations < 1 or self.candidates < 4 or not 2 <= self.elite < self.candidates:
            raise ValueError("Require horizon/iterations>=1 and 2 <= elite < candidates")
        if not 0 < self.pulse_s <= 0.2 or self.observe_wait_s <= 0:
            raise ValueError("Research pulse must be in (0,0.2] seconds and wait must be positive")
        if not 0 < self.max_forward <= 1 or not 0 < self.max_yaw <= 1:
            raise ValueError("Normalized command limits must lie in (0,1]")
        if self.depth_scale_m <= 0 or not 0 < self.clearance_m <= self.stop_distance_m < self.depth_scale_m:
            raise ValueError("Require 0 < clearance_m <= stop_distance_m < depth_scale_m")
        if self.max_forward_change <= 0 or self.max_yaw_change <= 0 or self.clearance_error_margin_m < 0:
            raise ValueError("Invalid slew or clearance-margin setting")
        if self.max_aux_disagreement <= 0:
            raise ValueError("max_aux_disagreement must be positive")


@dataclass
class PlannerResult:
    action: np.ndarray
    reason: str
    diagnostics: dict[str, Any] = field(default_factory=dict)
    planned_actions: np.ndarray | None = None


class MPCPlanner:
    def __init__(self, model: WorldModel, config: PlannerConfig | None = None):
        self.model = model
        self.config = config or PlannerConfig()
        if max(self.config.pulse_s, self.config.observe_wait_s) > model.config.max_dt_s:
            raise ValueError("Planner pulse/wait duration exceeds model max_dt_s")
        self.rng = np.random.default_rng(self.config.seed)

    def _bounded(self, samples: np.ndarray, current_action: np.ndarray) -> np.ndarray:
        c = self.config
        samples[..., 0] = np.clip(samples[..., 0], 0.0, c.max_forward)
        samples[..., 1] = np.clip(samples[..., 1], -c.max_yaw, c.max_yaw)
        # Each pulse begins after a stopped interval. Keeping every pulse within
        # a zero-to-command slew limit avoids a future plan assuming continuous
        # momentum that this stop-look-act executor does not permit.
        samples[..., 0] = np.minimum(samples[..., 0], c.max_forward_change)
        samples[..., 1] = np.clip(samples[..., 1], -c.max_yaw_change, c.max_yaw_change)
        samples[:, 0, 0] = np.clip(samples[:, 0, 0], max(0, current_action[0] - c.max_forward_change),
                                    min(c.max_forward, current_action[0] + c.max_forward_change))
        samples[:, 0, 1] = np.clip(samples[:, 0, 1], max(-c.max_yaw, current_action[1] - c.max_yaw_change),
                                    min(c.max_yaw, current_action[1] + c.max_yaw_change))
        return samples.astype(np.float32)

    def _score(self, predictions: torch.Tensor, actions: torch.Tensor,
               goal: str) -> tuple[torch.Tensor, torch.Tensor, dict[str, torch.Tensor]]:
        # predictions [E,C,H,6] at the end of each pulse+stationary wait.
        c = self.config
        mean = predictions.mean(0)
        disagreement = predictions.std(0, unbiased=False)
        lower_clearance = predictions[..., 3:].amin(0) - c.clearance_error_margin_m / c.depth_scale_m
        threshold = c.clearance_m / c.depth_scale_m
        minimum_clearance = lower_clearance.amin(-1)
        translating = actions[..., 0] > 1e-4
        turning_left, turning_right = actions[..., 1] > 1e-4, actions[..., 1] < -1e-4
        collision = ((minimum_clearance < threshold) & translating)
        collision |= (lower_clearance[..., 0] < threshold) & turning_left
        collision |= (lower_clearance[..., 2] < threshold) & turning_right
        critical_uncertainty = disagreement[..., 2:].amax(-1)
        moving = translating | turning_left | turning_right
        uncertain = (critical_uncertainty > c.max_aux_disagreement) & moving
        feasible = ~(collision | uncertain).any(-1)
        weights = torch.linspace(1.0, 0.85, c.horizon, device=mean.device)
        obstacle_cost = 40.0 * torch.relu(threshold - minimum_clearance).square()
        motion_cost = 0.015 * actions[..., 0].square() + 0.06 * actions[..., 1].square()
        uncertainty_cost = 1.0 * critical_uncertainty
        if goal == "approach_person":
            target_proximity = 1.0 - c.stop_distance_m / c.depth_scale_m
            distance_cost = 3.0 * (mean[..., 2] - target_proximity).square()
            bearing_cost = 0.9 * mean[..., 1].square()
            visibility_cost = 1.5 * (1.0 - mean[..., 0])
            cost = distance_cost + bearing_cost + visibility_cost + obstacle_cost + motion_cost + uncertainty_cost
        elif goal == "avoid_obstacles":
            # Modest progress reward, with independently enforced current safety.
            cost = obstacle_cost + motion_cost + uncertainty_cost - 0.25 * actions[..., 0]
            cost += 0.08 * (1.0 - minimum_clearance.clamp(0, 1))
        else:
            raise ValueError(f"Unsupported goal {goal!r}")
        scores = (cost * weights).sum(-1)
        scores = torch.where(feasible, scores, scores.new_full(scores.shape, 1e6))
        details = {
            "minimum_predicted_clearance_m": minimum_clearance.amin(-1) * c.depth_scale_m,
            "max_aux_disagreement": critical_uncertainty.amax(-1),
            "predicted_person_visibility": mean[..., 0],
            "predicted_person_bearing": mean[..., 1],
            "predicted_person_proximity": mean[..., 2],
        }
        return scores, feasible, details

    @torch.no_grad()
    def plan(self, state: WorldState, *, goal: str = "approach_person",
             current_action: tuple[float, float] | np.ndarray = (0.0, 0.0),
             for_live: bool = False,
             action_allowed: Callable[[np.ndarray], bool] | None = None,
             first_action_options: np.ndarray | list[list[float]] | None = None) -> PlannerResult:
        if goal == "stop":
            return PlannerResult(np.zeros(2, dtype=np.float32), "text_stop")
        if goal not in {"approach_person", "avoid_obstacles"}:
            raise ValueError("Planner supports approach_person, avoid_obstacles, stop only")
        if for_live:
            check_live_metadata(self.model.checkpoint_metadata, depth_scale_m=self.config.depth_scale_m)
        if state.h.shape[1] != 1:
            raise ValueError("Online MPC expects one robot state")
        if not torch.isfinite(state.h).all() or not torch.isfinite(state.z).all():
            return PlannerResult(np.zeros(2, dtype=np.float32), "nonfinite_world_state")
        current = np.asarray(current_action, dtype=np.float32)
        if current.shape != (2,) or not np.isfinite(current).all() or (np.abs(current) > 1).any():
            raise ValueError("current_action must be normalized finite [forward,yaw]")
        c = self.config
        first_options = None
        if first_action_options is not None:
            first_options = np.asarray(first_action_options, dtype=np.float32)
            if first_options.ndim != 2 or first_options.shape[1] != 2 or not 1 <= len(first_options) <= c.candidates:
                raise ValueError("first_action_options must be [K,2], 1 <= K <= candidates")
            if not np.isfinite(first_options).all():
                raise ValueError("first_action_options contain nonfinite values")
            stop_options = (first_options == 0).all(axis=1)
            if not stop_options.any():
                raise ValueError("first_action_options must include a stop [0,0]")
            if np.any(first_options[:, 0] < 0) or np.any(first_options[:, 0] > c.max_forward + 1e-6):
                raise ValueError("Discrete first forward options exceed planner limits")
            if np.any(np.abs(first_options[:, 1]) > c.max_yaw + 1e-6):
                raise ValueError("Discrete first yaw options exceed planner limits")
            changes = np.abs(first_options - current)
            if np.any((~stop_options) & ((changes[:, 0] > c.max_forward_change + 1e-6) |
                                         (changes[:, 1] > c.max_yaw_change + 1e-6))):
                raise ValueError("Discrete first options exceed configured slew limits")
        self.model.eval()
        mean = np.zeros((c.horizon, 2), dtype=np.float32)
        mean[:, 0] = c.max_forward * 0.4
        std = np.broadcast_to(np.array([c.max_forward * 0.6, c.max_yaw * 0.8], dtype=np.float32), mean.shape).copy()
        best_score = float("inf")
        best_actions: np.ndarray | None = None
        best_details: dict[str, Any] = {}
        candidates_state = state.repeat_batch(c.candidates)
        feasible_count = 0
        allowed_cache: dict[tuple[float, float], bool] = {}
        for iteration in range(c.iterations):
            samples = mean + std * self.rng.normal(size=(c.candidates, c.horizon, 2))
            samples = self._bounded(samples, current)
            # Always compare against stopped, straight and two rotation primitives.
            samples[0] = 0
            samples[1] = (min(c.max_forward, c.max_forward_change), 0)
            samples[2] = (0, min(c.max_yaw, c.max_yaw_change))
            samples[3] = (0, -min(c.max_yaw, c.max_yaw_change))
            if best_actions is not None and c.candidates > 4:
                samples[4] = best_actions
            samples = self._bounded(samples, current)
            if first_options is not None:
                # The whole imagined plan uses the same realizable primitive
                # set as its first pulse. In particular, do not imagine reverse
                # or side-only motion unavailable to this camera-only executor.
                distance=((samples[:,:,None,:]-first_options[None,None,:,:])**2).sum(-1)
                samples=first_options[distance.argmin(-1)].copy()
                option_indices = np.arange(c.candidates) % len(first_options)
                self.rng.shuffle(option_indices)
                samples[:, 0] = first_options[option_indices]
                if best_actions is not None and c.candidates > 4:
                    samples[4] = best_actions
            # A stop command may always decelerate immediately.
            samples[0] = 0
            actions = torch.as_tensor(samples, device=self.model.device)
            rollout_controls = torch.zeros((c.candidates, c.horizon * 2, 2), device=self.model.device)
            rollout_controls[:, 0::2] = actions
            rollout_dts = torch.empty((c.candidates, c.horizon * 2), device=self.model.device)
            rollout_dts[:, 0::2], rollout_dts[:, 1::2] = c.pulse_s, c.observe_wait_s
            all_predictions, _ = self.model.rollout(candidates_state, rollout_controls, rollout_dts)
            if not torch.isfinite(all_predictions).all():
                return PlannerResult(np.zeros(2,np.float32),"nonfinite_rollout",{"for_live":for_live})
            # Score both pulse endpoints and later observation endpoints: scoring
            # only after stopping could miss a predicted transient collision.
            after_pulse = all_predictions[:, :, 0::2]
            after_wait = all_predictions[:, :, 1::2]
            pulse_scores, pulse_feasible, _ = self._score(after_pulse, actions, goal)
            scores, feasible, details = self._score(after_wait, actions, goal)
            feasible &= pulse_feasible
            if action_allowed is not None:
                # Current metric geometry may reject the first action of a
                # candidate independently of all learned future predictions.
                allowed = []
                for first_action in samples[:, 0]:
                    if np.max(np.abs(first_action)) <= 1e-5:
                        allowed.append(True)
                    else:
                        key = (float(first_action[0]), float(first_action[1]))
                        if key not in allowed_cache:
                            try:
                                allowed_cache[key] = bool(action_allowed(first_action.copy()))
                            except Exception:
                                allowed_cache[key] = False
                        allowed.append(allowed_cache[key])
                feasible &= torch.tensor(allowed, dtype=torch.bool, device=scores.device)
            scores = torch.where(feasible, scores + 0.25 * pulse_scores, scores.new_full(scores.shape, 1e6))
            feasible_count = int(feasible.sum())
            elite_indices = torch.topk(scores, k=c.elite, largest=False).indices.cpu().numpy()
            best_i = int(elite_indices[0])
            score = float(scores[best_i])
            if score < best_score and bool(feasible[best_i]):
                best_score, best_actions = score, samples[best_i].copy()
                best_details = {key: value[best_i].detach().cpu().tolist() for key, value in details.items()}
            elites = samples[elite_indices]
            mean = 0.15 * mean + 0.85 * elites.mean(0)
            std = np.maximum(0.025, elites.std(0)).astype(np.float32)
        diagnostics = {
            "planner": "CEM pulse-stop-observe", "feasible_candidates_last_iteration": feasible_count,
            "pulse_s": c.pulse_s, "assumed_observation_wait_s": c.observe_wait_s,
            "synthetic_checkpoint": bool(self.model.checkpoint_metadata.get("synthetic", True)),
            "for_live": for_live, "config": asdict(c), **best_details,
            "current_geometry_checks": len(allowed_cache),
            "discrete_first_actions": len(first_options) if first_options is not None else None,
            "all_steps_use_discrete_primitives": first_options is not None,
            "uncertainty_note": "Ensemble disagreement is not a certified error bound.",
        }
        if best_actions is None:
            return PlannerResult(np.zeros(2, dtype=np.float32), "no_acceptable_rollout", diagnostics)
        diagnostics["score"] = best_score
        action = best_actions[0].copy()
        reason = "planned_pulse" if np.max(np.abs(action)) > 1e-4 else "planned_stop"
        return PlannerResult(action, reason, diagnostics, best_actions)
