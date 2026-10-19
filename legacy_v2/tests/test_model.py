import numpy as np
import pytest

torch = pytest.importorskip("torch")

from room_world_model.data import SequenceDataset
from room_world_model.model import ACTION_SEMANTICS, ModelConfig, WorldModel, check_live_metadata, load_checkpoint, save_checkpoint
from room_world_model.simulate import make_episode

torch.set_num_threads(2)


def small_model():
    torch.manual_seed(4)
    return WorldModel(ModelConfig(latent_dim=16, hidden_dim=24, ensemble_size=2))


def test_exact_pulse_then_stop_differs_from_interval_mean():
    model = small_model().eval()
    state = model.initial_state(torch.rand(1, 1, 128, 128))
    pulse = torch.tensor([[0.4, -0.2]])
    segments = torch.tensor([[[0.4, -0.2, 0.08], [0.0, 0.0, 0.7]]])
    mean = pulse * (0.08 / 0.78)
    exact = model.advance_interval(state, mean, 0.78, segments, torch.ones(1, 2, dtype=torch.bool))
    manual = model.advance(model.advance(state, pulse, 0.08), torch.zeros(1, 2), 0.7)
    approximate = model.advance(state, mean, 0.78)
    torch.testing.assert_close(exact.h, manual.h)
    torch.testing.assert_close(exact.z, manual.z)
    assert not torch.allclose(exact.z, approximate.z)
    identity = model.advance(state, pulse, 0.0)
    torch.testing.assert_close(identity.h, state.h, rtol=0, atol=0)
    torch.testing.assert_close(identity.z, state.z, rtol=0, atol=0)


def test_multistep_training_has_finite_dynamics_gradients(tmp_path):
    path = make_episode(tmp_path / "episode.npz", seed=10, frames=8)
    sample = SequenceDataset([path], sequence_length=3, stride=1)[0]
    batch = {key: torch.from_numpy(value)[None] for key, value in sample.items()}
    model = small_model()
    loss, metrics = model.training_loss(batch, burn_in=1, bootstrap_probability=1.0)
    assert np.isfinite(metrics["open_loop"]) and metrics["open_loop"] > 0
    loss.backward()
    for member in model.members:
        assert member.cell.weight_ih.grad is not None
        assert torch.isfinite(member.cell.weight_ih.grad).all()
        assert member.cell.weight_ih.grad[:, -5:].abs().sum() > 0
        assert member.prior[0].weight.grad.abs().sum() > 0
    assert model.encoder.net[0].weight.grad.abs().sum() > 0


def test_checkpoint_roundtrip_and_synthetic_live_gate(tmp_path):
    model = small_model().eval()
    image = torch.rand(1, 1, 128, 128)
    path = tmp_path / "model.pt"
    save_checkpoint(path, model, {"synthetic": True})
    restored, metadata = load_checkpoint(path)
    torch.testing.assert_close(model.predictions(model.initial_state(image)),
                               restored.predictions(restored.initial_state(image)))
    assert metadata["synthetic"]
    with pytest.raises(ValueError, match="Synthetic"):
        load_checkpoint(path, for_live=True)


def test_control_and_time_validation():
    model = small_model()
    state = model.initial_state(torch.zeros(1, 1, 128, 128))
    with pytest.raises(ValueError, match="interval limits"):
        model.advance(state, torch.zeros(1, 2), -0.1)
    with pytest.raises(ValueError, match="Normalized controls"):
        model.advance(state, torch.tensor([[2.0, 0.0]]), 0.1)


@pytest.mark.parametrize("verified", [None, False, "true", 1])
def test_live_checkpoint_requires_verified_label_configuration(verified):
    metadata = {"synthetic": False, "provenance_complete": True, "calibration_confirmed": True,
                "uncalibrated_pseudo_labels": False, "labels_from_accepted_calibration": True,
                "action_semantics": ACTION_SEMANTICS, "control_representation": "piecewise_mcu_confirmed",
                "dataset_schema_version":2,"mcu_confirmation_validated":True,
                "calibration_fingerprint": "f" * 64, "depth_scale_m": 3.0,
                "held_out_validation": True, "label_fingerprint_verified": True}
    check_live_metadata(metadata, calibration_fingerprint="f" * 64, depth_scale_m=3.0)
    if verified is None:
        metadata.pop("label_fingerprint_verified")
    else:
        metadata["label_fingerprint_verified"] = verified
    with pytest.raises(ValueError, match="label configuration fingerprint"):
        check_live_metadata(metadata, calibration_fingerprint="f" * 64, depth_scale_m=3.0)


def test_missing_piecewise_history_is_not_replaced_by_average_control():
    model = small_model()
    state = model.initial_state(torch.rand(1,1,128,128))
    with pytest.raises(ValueError,match="confirmed action_segments"):
        model.advance_interval(state,torch.zeros(1,2),.7)


def test_unknown_aux_targets_have_zero_effect_on_all_future_loss(tmp_path):
    path=make_episode(tmp_path/"e.npz",seed=14,frames=8)
    sample=SequenceDataset([path],sequence_length=4,stride=1)[0]
    batch={key:torch.from_numpy(value)[None] for key,value in sample.items()}
    batch["aux_mask"][:,:,[1,2,3]]=False
    model=small_model().eval()
    first,_=model.training_loss(batch,burn_in=1,bootstrap_probability=1.)
    altered={key:value.clone() for key,value in batch.items()}
    altered["aux"][:,:,[1,2,3]]=1234.
    second,_=model.training_loss(altered,burn_in=1,bootstrap_probability=1.)
    torch.testing.assert_close(first,second,rtol=0,atol=0)


def test_v1_checkpoint_is_explicitly_rejected(tmp_path):
    path=tmp_path/"v1.pt"
    torch.save({"format":"room-world-model-v1"},path)
    with pytest.raises(ValueError,match="retraining"):
        load_checkpoint(path)
