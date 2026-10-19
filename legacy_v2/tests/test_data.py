import json

import numpy as np
import pytest

from room_world_model.data import SequenceDataset, read_episode, split_episodes, partition_episodes, split_audit, validate_episode
from room_world_model.simulate import make_episode


def rewrite(path, data):
    np.savez_compressed(path, **data)


def test_sequences_preserve_piecewise_controls_and_do_not_cross_gaps(tmp_path):
    path = make_episode(tmp_path / "episode.npz", seed=7, frames=14)
    info = validate_episode(path)
    assert info.has_exact_segments and info.metadata["synthetic"] is True
    data = read_episode(path)
    data["transition_valid"][5] = False
    rewrite(path, data)
    dataset = SequenceDataset([path], sequence_length=3, stride=1)
    assert all(not (start <= 5 < start + 3) for _, start in dataset.windows)
    sample = dataset[0]
    assert sample["images"].shape == (4, 1, 128, 128)
    assert sample["images"].dtype == np.float32
    assert sample["segment_mask"][:, :2].all()
    assert not sample["segment_mask"][:, 2:].any()
    np.testing.assert_allclose(sample["action_segments"][:, :2, 2].sum(1), sample["dt"], atol=1e-6)
    # Pulse then stopped, not a constant weak motor command.
    assert np.all(sample["action_segments"][:, 1, :2] == 0)


def test_episode_split_groups_whole_sessions(tmp_path):
    paths = [make_episode(tmp_path / f"e{i}.npz", seed=i, frames=8) for i in range(4)]
    for i, path in enumerate(paths):
        data = read_episode(path)
        metadata = json.loads(str(data["metadata_json"].item()))
        metadata["session_id"] = f"session-{i // 2}"
        data["metadata_json"] = np.array(json.dumps(metadata))
        rewrite(path, data)
    train, val = split_episodes(paths, seed=4)
    assert {x.split_group for x in train}.isdisjoint({x.split_group for x in val})
    assert len(train) == len(val) == 2
    with pytest.raises(ValueError, match="two separate sessions"):
        split_episodes(paths[:2])


def test_reject_wrong_exposure_timing_and_segment_coverage(tmp_path):
    path = make_episode(tmp_path / "e.npz", seed=2, frames=8)
    data = read_episode(path)
    data["dt"][0] += 0.1
    rewrite(path, data)
    with pytest.raises(ValueError, match="timestamps"):
        validate_episode(path)
    path = make_episode(path, seed=2, frames=8)
    data = read_episode(path)
    data["action_segments"][0, 0, 2] *= 0.5
    rewrite(path, data)
    with pytest.raises(ValueError, match="durations"):
        validate_episode(path)


def test_unknown_labels_are_masked_and_finite(tmp_path):
    path = make_episode(tmp_path / "e.npz", seed=3, frames=8)
    data = read_episode(path)
    data["aux_mask"][:, 3:] = False
    data["aux"][:, 3:] = 0
    rewrite(path, data)
    assert validate_episode(path).aux_valid_counts[3:] == (0, 0, 0)
    data["aux"][0, 3] = np.nan
    rewrite(path, data)
    with pytest.raises(ValueError, match="nonfinite"):
        validate_episode(path)


def test_v1_or_summary_only_data_is_rejected(tmp_path):
    path=make_episode(tmp_path/"legacy.npz",seed=13,frames=8)
    data=read_episode(path)
    meta=json.loads(str(data["metadata_json"].item()))
    meta["schema_version"]=1
    data["metadata_json"]=np.asarray(json.dumps(meta))
    rewrite(path,data)
    with pytest.raises(ValueError,match="re-exported"):
        validate_episode(path)
    meta["schema_version"]=2
    data["metadata_json"]=np.asarray(json.dumps(meta))
    data.pop("action_segments")
    data.pop("segment_mask")
    rewrite(path,data)
    with pytest.raises(ValueError,match="no interval-mean fallback"):
        validate_episode(path)


def test_three_way_split_keeps_sessions_and_layouts_indivisible(tmp_path):
    paths=[make_episode(tmp_path/f"e{i}.npz",seed=i+30,frames=8) for i in range(7)]
    for i,path in enumerate(paths):
        data=read_episode(path)
        meta=json.loads(str(data["metadata_json"].item()))
        meta.update(session_id=f"s{i//2}" if i<4 else f"s{i}",layout_id=f"layout-{i}")
        data["metadata_json"]=np.asarray(json.dumps(meta));rewrite(path,data)
    parts=partition_episodes(paths,split_by="layout")
    assert all(parts.values())
    audit=split_audit(parts,"layout")
    assert all(not overlap["session"] and not overlap["layout"] for overlap in audit["overlaps"].values())


def test_renaming_same_recording_cannot_cross_partitions(tmp_path):
    original=make_episode(tmp_path/"e0.npz",seed=12,frames=8)
    paths=[original]
    for i in (1,2):
        data=read_episode(original)
        meta=json.loads(str(data["metadata_json"].item()))
        meta.update(session_id=f"copy-{i}",layout_id=f"copied-layout-{i}")
        data["metadata_json"]=np.asarray(json.dumps(meta))
        path=tmp_path/f"e{i}.npz";rewrite(path,data);paths.append(path)
    with pytest.raises(ValueError,match="Duplicate recorded content"):
        partition_episodes(paths)
