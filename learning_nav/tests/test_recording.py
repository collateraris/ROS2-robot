import numpy as np
from learning_nav.data import DEFAULT_LIMITS_S
from learning_nav.recording import scan_ready, synchronized_pair


def test_wait_for_rf2o_then_match_same_scan_without_future_sensor():
    scan = 10_000_000_000
    images = [(scan - 30_000_000, "rgb-at-scan"), (scan + 30_000_000, "future-rgb")]
    odoms = [(scan - 100_000_000, "previous-scan-pose")]
    assert not scan_ready(scan, scan + 20_000_000, scan + 40_000_000, odoms, .12)
    odoms += [(scan, "current-scan-pose"), (scan + 100_000_000, "future-pose")]
    assert scan_ready(scan, scan + 20_000_000, scan + 50_000_000, odoms, .12)
    pair = synchronized_pair(scan, scan + 50_000_000, images, odoms, True,
                             scan + 40_000_000, DEFAULT_LIMITS_S)
    assert pair == (images[0], odoms[1])


def test_missing_rf2o_timeout_does_not_invent_pose():
    scan = 10_000_000_000
    assert scan_ready(scan, scan + 20_000_000, scan + 150_000_000, [], .12)
    assert synchronized_pair(scan, scan + 150_000_000, [(scan, None)], [], True,
                             scan + 140_000_000, DEFAULT_LIMITS_S) is None


def test_health_loss_or_stale_health_drops_sample_even_with_current_pose():
    scan = 10_000_000_000
    sensors = [(scan, np.zeros(3))]
    assert synchronized_pair(scan, scan + 50_000_000, sensors, sensors, False,
                             scan + 40_000_000, DEFAULT_LIMITS_S) is None
    assert synchronized_pair(scan, scan + 50_000_000, sensors, sensors, True,
                             scan - 300_000_000, DEFAULT_LIMITS_S) is None


def test_stale_scan_camera_and_odom_are_rejected():
    scan = 10_000_000_000
    fresh = [(scan, None)]
    cases = [(scan + 300_000_000, fresh, fresh),
             (scan + 50_000_000, [(scan - 210_000_000, None)], fresh),
             (scan + 50_000_000, fresh, [(scan - 160_000_000, None)])]
    for now, images, odoms in cases:
        assert synchronized_pair(scan, now, images, odoms, True, now - 10_000_000,
                                 DEFAULT_LIMITS_S) is None
