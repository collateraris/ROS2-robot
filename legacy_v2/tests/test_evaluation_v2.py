import json

import numpy as np
import pytest

torch=pytest.importorskip("torch")

from room_world_model.evaluate import Metrics, permute_pulse_controls
from room_world_model.train import photometric_augmentation
from room_world_model.acceptance import load_acceptance, load_trial_eligibility


def test_action_shuffle_changes_pulses_without_moving_idle_or_padding():
    segments=torch.tensor([[[[0.,0.,.3],[.4,.2,.08],[0.,0.,.7],[0.,0.,0.]],
                            [[0.,0.,.2],[.7,-.2,.07],[0.,0.,.8],[0.,0.,0.]],
                            [[0.,0.,.2],[.2,.1,.1],[0.,0.,.8],[0.,0.,0.]]]])
    mask=segments[...,2]>0
    result,counts=permute_pulse_controls(segments,mask,burn_in=1,rng=np.random.default_rng(5))
    torch.testing.assert_close(result[:,0],segments[:,0],rtol=0,atol=0)
    torch.testing.assert_close(result[...,2],segments[...,2],rtol=0,atol=0)
    zero=segments[...,:2].abs().sum(-1)==0
    assert not result[...,:2][zero].any()
    assert counts=={"eligible_pulses":2,"changed_pulses":2}
    assert not torch.equal(result[:,1:,:,:2],segments[:,1:,:,:2])


def test_single_pulse_shuffle_reports_insufficient_variation():
    segments=torch.tensor([[[[.7,0.,.08],[0.,0.,.7]]]])
    result,counts=permute_pulse_controls(segments,torch.ones((1,1,2),dtype=torch.bool),burn_in=0,
                                        rng=np.random.default_rng(2))
    assert counts=={"eligible_pulses":1,"changed_pulses":0}
    torch.testing.assert_close(segments,result)


def test_missing_labels_never_create_zero_mae_or_false_free_space():
    metric=Metrics(3.)
    image=torch.zeros(1,1,128,128)
    metric.update(0,image,image,torch.ones(1,6),torch.zeros(1,6),torch.zeros(1,6,dtype=torch.bool))
    result=metric.result()
    assert result["common_mask_aux_mae"] is None
    assert all(value is None for value in result["aux_mae"].values())
    assert result["false_clearance_rate_at_0_6m"] is None
    assert result["aux_valid_count_by_future_frame"]==[0]


def test_photometric_augmentation_is_constant_over_time_and_preserves_labels():
    torch.manual_seed(2)
    image=torch.rand(2,1,1,128,128).expand(-1,5,-1,-1,-1).clone()
    batch={"images":image,"aux":torch.rand(2,5,6),"aux_mask":torch.ones(2,5,6,dtype=torch.bool)}
    result=photometric_augmentation(batch,.12)
    for t in range(1,5):
        torch.testing.assert_close(result["images"][:,0],result["images"][:,t],rtol=0,atol=0)
    torch.testing.assert_close(result["aux"],batch["aux"])
    assert not torch.equal(result["images"],image)


def reviewed_evidence():
    return {"schema_version":1,"domain":"real_robot","reviewed":True,"reviewer":"operator-test",
            "reviewed_utc":"2026-10-12T00:00:00Z","checkpoint_sha256":"a"*64,
            "calibration_fingerprint":"b"*64,
            "action_ablation":{"domain":"real_robot","evidence_id":"recorded-test-report",
                "held_out":True,"common_valid_label_mask":True,"actual_action_aux_mae":.1,
                "zero_action_aux_mae":.2,"shuffled_action_aux_mae":.21,"persistence_aux_mae":.15},
            "paired_trials":{"domain":"real_robot","evidence_id":"recorded-test-trials","count":3,
                "same_conditions_and_gate":True,"wm_successes":3,"reactive_successes":2,
                "wm_interventions":0,"reactive_interventions":0,"wm_collisions":0,"reactive_collisions":0,
                "wm_min_clearance_m":.7,"reactive_min_clearance_m":.6,"common_successful_pairs":2,
                "wm_median_time_s":15.,"reactive_median_time_s":17.}}


@pytest.mark.parametrize("bad",["synthetic","wrong_checkpoint","no_action_effect","worse_clearance","all_stopped"])
def test_acceptance_rejects_synthetic_or_nonbeneficial_evidence(tmp_path,bad):
    value=reviewed_evidence()
    if bad=="synthetic":value["domain"]="synthetic_toy"
    elif bad=="wrong_checkpoint":value["checkpoint_sha256"]="c"*64
    elif bad=="no_action_effect":value["action_ablation"]["zero_action_aux_mae"]=.1
    elif bad=="worse_clearance":value["paired_trials"]["wm_min_clearance_m"]=.4
    else:value["paired_trials"].update(wm_successes=0,common_successful_pairs=0)
    path=tmp_path/"acceptance.json";path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        load_acceptance(path,calibration_fingerprint="b"*64,checkpoint_sha256="a"*64)


def test_acceptance_checks_human_record_but_does_not_create_one(tmp_path):
    path=tmp_path/"acceptance.json";path.write_text(json.dumps(reviewed_evidence()))
    before=path.read_bytes()
    assert load_acceptance(path,calibration_fingerprint="b"*64,checkpoint_sha256="a"*64)["reviewed"] is True
    assert path.read_bytes()==before


def test_trial_mode_can_collect_paired_evidence_before_normal_wm_acceptance(tmp_path):
    value=reviewed_evidence();value.pop("paired_trials")
    path=tmp_path/"trial_eligibility.json";path.write_text(json.dumps(value))
    assert load_trial_eligibility(path,calibration_fingerprint="b"*64,checkpoint_sha256="a"*64)["domain"]=="real_robot"
    with pytest.raises(ValueError,match="paired_trials"):
        load_acceptance(path,calibration_fingerprint="b"*64,checkpoint_sha256="a"*64)
    value["action_ablation"]["zero_action_aux_mae"]=.1;path.write_text(json.dumps(value))
    with pytest.raises(ValueError,match="Actual actions"):
        load_trial_eligibility(path,calibration_fingerprint="b"*64,checkpoint_sha256="a"*64)
