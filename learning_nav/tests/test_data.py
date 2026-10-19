import tempfile
from pathlib import Path
import numpy as np
import pytest
from learning_nav.data import grouped_split,sector_scan,relative_motion,candidate_geometry,load_dataset,load_metadata,metadata_fingerprint
from learning_nav.synthetic import generate


def test_sessions_never_cross_splits():
    sessions=np.repeat(["a","b","c","d","e","f"],8)
    split=grouped_split(sessions)
    groups={k:set(sessions[v]) for k,v in split.items()}
    assert not groups["train"]&groups["val"]
    assert not groups["train"]&groups["test"]
    assert not groups["test"]&groups["val"]
    assert sum(map(len,split.values()))==len(sessions)


def test_missing_beams_are_invalid_never_free():
    ranges=[float("nan"),float("inf"),float("-inf"),-.1,.05,11.,2.,10.]
    values,mask=sector_scan(ranges,-np.pi,2*np.pi/8,.1,10.,8,5.)
    assert mask.sum()==2
    assert not mask[:6].any()
    np.testing.assert_allclose(values[mask],[.4,1.])
    assert np.all(values[~mask]==0)


def test_motion_body_frame_and_wrapped_angle():
    old=np.array([[0.,0.,np.pi/2],[0.,0.,np.pi-.01]])
    nxt=np.array([[0.,1.,np.pi/2],[0.,0.,-np.pi+.01]])
    motion=relative_motion(old,nxt)
    np.testing.assert_allclose(motion[0],[1,0,0],atol=1e-6)
    np.testing.assert_allclose(motion[1,2],.02,atol=1e-6)


def test_known_goal_geometry_uses_map_pose():
    meta={"candidates":[{"name":"STOP","pose":None},{"name":"desk","pose":[5.,3.,0.]}]}
    result=candidate_geometry(np.array([[4.,3.,np.pi/2]]),meta)
    np.testing.assert_allclose(result[0,1],[0,-1],atol=1e-6)
    np.testing.assert_array_equal(result[0,0],[0,0])


def test_synthetic_schema_and_fingerprint():
    with tempfile.TemporaryDirectory() as tmp:
        generate(tmp,sessions=3,rows=6)
        meta=load_metadata(Path(tmp)/"metadata.json")
        data=load_dataset(sorted(Path(tmp).glob("*.npz")),meta)
        assert data["rgb"].shape==(18,128,128,3)
        assert data["action_valid"].all()
        assert data["map_pose_valid"].all()
        other=dict(meta,map_id="ANOTHER-MAP")
        assert metadata_fingerprint(meta)!=metadata_fingerprint(other)
        try: load_dataset([Path(tmp)/"session_000.npz"],other)
        except ValueError: pass
        else: raise AssertionError("Must reject a different saved map")


@pytest.mark.parametrize("field,mutate,error", [
    ("next_odom_healthy", lambda x: np.zeros_like(x), "Unhealthy"),
    ("next_odom_ns", lambda x: x - 300000000, "Stale/future"),
    ("rgb_ns", lambda x: x - 300000000, "Stale/future"),
    ("health_received_ns", lambda x: x - 400000000, "Stale/future"),
    ("scan_ns", lambda x: x + 1, "endpoints"),
    ("odometry_source", lambda x: np.array("wheel_encoders"), "source mismatch"),
    ("action_source", lambda x: np.array("measured_velocity"), "source mismatch"),
])
def test_reject_unhealthy_stale_or_misidentified_recordings(tmp_path, field, mutate, error):
    generate(tmp_path, sessions=3, rows=6)
    meta=load_metadata(tmp_path/"metadata.json")
    path=tmp_path/"session_000.npz"
    with np.load(path,allow_pickle=False) as z:
        data={key:z[key] for key in z.files}
    data[field]=mutate(data[field]); np.savez_compressed(path,**data)
    with pytest.raises(ValueError,match=error): load_dataset([path],meta)


def test_source_is_part_of_checkpoint_dataset_fingerprint(tmp_path):
    generate(tmp_path,sessions=3,rows=6)
    meta=load_metadata(tmp_path/"metadata.json")
    other=dict(meta,odometry_source="rf2o_laser_odometry_guarded",synthetic=False)
    assert metadata_fingerprint(meta)!=metadata_fingerprint(other)
    with pytest.raises(ValueError,match="metadata/schema"): load_dataset([tmp_path/"session_000.npz"],other)


def test_platform_sources_are_explicit_and_never_interchangeable(tmp_path):
    import json
    from learning_nav.data import ACTION_SOURCES
    generate(tmp_path,sessions=3,rows=6)
    meta=load_metadata(tmp_path/'metadata.json')
    assert meta['environment']=='synthetic'
    assert meta['action_source']=='synthetic_command'
    for environment in ('hardware','gazebo'):
        m=dict(meta,environment=environment,synthetic=False,
               odometry_source='rf2o_laser_odometry_guarded',
               action_source=ACTION_SOURCES[environment])
        path=tmp_path/(environment+'.json');path.write_text(json.dumps(m))
        assert load_metadata(path)['environment']==environment
        assert metadata_fingerprint(m)!=metadata_fingerprint(meta)
        with pytest.raises(ValueError,match='metadata/schema'):
            load_dataset([tmp_path/'session_000.npz'],m)
        m['action_source']=ACTION_SOURCES['gazebo' if environment=='hardware' else 'hardware']
        path.write_text(json.dumps(m))
        with pytest.raises(ValueError,match='action_source'):load_metadata(path)
