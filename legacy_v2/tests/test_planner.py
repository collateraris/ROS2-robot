from types import SimpleNamespace

import numpy as np
import pytest

torch = pytest.importorskip("torch")

from room_world_model.model import WorldState
from room_world_model.planner import MPCPlanner, PlannerConfig

torch.set_num_threads(2)


class ToyModel:
    def __init__(self, hazard=False, pulse_only_hazard=False, disagreement=False):
        self.config = SimpleNamespace(max_dt_s=5.0)
        self.device = torch.device("cpu")
        self.checkpoint_metadata = {"synthetic": True}
        self.hazard = hazard
        self.pulse_only_hazard = pulse_only_hazard
        self.disagreement = disagreement

    def eval(self):
        return self

    def rollout(self, state, actions, dt):
        b, horizon, _ = actions.shape
        predictions = torch.zeros(2, b, horizon, 6)
        predictions[..., 0] = 1
        predictions[..., 2] = 0.2
        predictions[..., 3:] = 0.9
        if self.hazard:
            predictions[..., 3:] = 0.0
        if self.pulse_only_hazard:
            predictions[:, :, 0::2, 3:] = 0.0
        if self.disagreement:
            predictions[0, ..., 3:] = 0.2
        return predictions, state


def state():
    return WorldState(torch.zeros(2, 1, 4), torch.zeros(2, 1, 4))


def config():
    return PlannerConfig(horizon=2, candidates=12, elite=3, iterations=2)


def test_planner_proposes_only_bounded_first_pulse():
    result = MPCPlanner(ToyModel(), config()).plan(state(), goal="avoid_obstacles")
    assert result.reason == "planned_pulse"
    assert 0 < result.action[0] <= config().max_forward_change + 1e-6
    assert abs(result.action[1]) <= config().max_yaw_change + 1e-6
    assert result.planned_actions.shape == (2, 2)
    assert result.diagnostics["synthetic_checkpoint"]


@pytest.mark.parametrize("mode", ["hazard", "pulse_only_hazard", "disagreement"])
def test_predicted_hazard_or_uncertainty_stops_motion(mode):
    model = ToyModel(**{mode: True})
    result = MPCPlanner(model, config()).plan(state(), goal="avoid_obstacles")
    np.testing.assert_array_equal(result.action, [0, 0])


def test_stop_and_live_provenance_gate():
    planner = MPCPlanner(ToyModel(), config())
    np.testing.assert_array_equal(planner.plan(state(), goal="stop").action, [0, 0])
    with pytest.raises(ValueError, match="Synthetic"):
        planner.plan(state(), for_live=True)
    broken = state()
    broken.h[0, 0, 0] = float("nan")
    assert planner.plan(broken).reason == "nonfinite_world_state"


def test_discrete_first_options_are_evaluated_exactly_and_geometry_is_cached():
    options = np.array([[0, 0], [0.18, 0], [0.18, 0.1], [0.18, -0.1]], dtype=np.float32)
    checked = []

    def allowed(action):
        checked.append(action.copy())
        return action[1] <= 0

    result = MPCPlanner(ToyModel(), config()).plan(
        state(), goal="avoid_obstacles", first_action_options=options, action_allowed=allowed)
    assert any(np.array_equal(result.action, option) for option in options)
    assert result.action[1] <= 0
    assert len(checked) <= len(options) - 1
    assert all(any(np.array_equal(action, option) for option in options) for action in checked)
    assert result.diagnostics["current_geometry_checks"] == len(checked)


def test_current_geometry_can_reject_every_motion():
    result = MPCPlanner(ToyModel(), config()).plan(
        state(), goal="avoid_obstacles", action_allowed=lambda action: False)
    np.testing.assert_array_equal(result.action, [0, 0])
