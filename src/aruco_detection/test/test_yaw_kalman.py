import math
import os
import sys

# Allow running the test without installing the package (import the module file directly)
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), "..", "aruco_detection"),
)

from yaw_kalman import YawKalmanFilter  # noqa: E402


def _rad(deg):
    return math.radians(deg)


def test_not_initialized_until_first_update():
    kf = YawKalmanFilter()
    assert not kf.initialized
    kf.predict(1.0)  # predict before any measurement must be a no-op
    assert not kf.initialized


def test_first_update_initializes_at_measurement():
    kf = YawKalmanFilter()
    ok = kf.update(_rad(12.0), _rad(2.0) ** 2)
    assert ok
    assert kf.initialized
    assert abs(kf.state - _rad(12.0)) < 1e-9
    assert abs(kf.covariance - _rad(2.0) ** 2) < 1e-9


def test_converges_to_truth_under_noise():
    kf = YawKalmanFilter(process_noise=1e-3, gate_sigma=3.0,
                         max_cov_deg=20.0, min_cov_deg=0.5)
    true = _rad(45.0)
    # noisy measurements clustered around truth
    for d in [44.0, 46.5, 45.2, 43.8, 45.9, 44.7, 45.1]:
        kf.update(_rad(d), _rad(1.5) ** 2)
        kf.predict(0.05)
    # estimate should be well within a degree of truth
    assert abs(math.degrees(kf.state) - 45.0) < 1.0


def test_wrap_around_no_jump():
    kf = YawKalmanFilter(process_noise=1e-3)
    # measurements straddling the -pi/pi boundary
    for d in [179.0, -179.0, 178.0, -178.5, 179.5]:
        ok = kf.update(_rad(d), _rad(1.0) ** 2)
        assert ok
        kf.predict(0.05)
    # state should stay near +/-180, not flip to 0
    assert abs(abs(math.degrees(kf.state)) - 179.0) < 3.0


def test_gate_rejects_outlier():
    kf = YawKalmanFilter(process_noise=1e-3, gate_sigma=3.0)
    kf.update(_rad(10.0), _rad(1.0) ** 2)
    kf.predict(0.05)
    # an outlier 90 deg away with small reported noise should be rejected
    ok = kf.update(_rad(100.0), _rad(0.5) ** 2)
    assert not ok
    # state must remain near 10 deg
    assert abs(math.degrees(kf.state) - 10.0) < 1.0


def test_coasting_clamps_at_max_covariance():
    kf = YawKalmanFilter(process_noise=0.01, max_cov_deg=20.0)
    kf.update(_rad(5.0), _rad(0.5) ** 2)
    # coast far longer than needed to reach max covariance
    for _ in range(2000):
        kf.predict(0.05)  # 100 s total
    assert kf.initialized  # NOT reset
    assert abs(kf.covariance - _rad(20.0) ** 2) < 1e-9  # clamped at max
    # still usable: a fresh measurement updates and stays initialized
    assert kf.update(_rad(7.0), _rad(1.0) ** 2)
    assert kf.initialized


def test_predict_with_process_noise_override():
    kf = YawKalmanFilter(process_noise=0.01, max_cov_deg=90.0)
    kf.update(_rad(0.0), _rad(0.5) ** 2)
    p0 = kf.covariance
    kf.predict(0.05)  # normal Q
    p_normal = kf.covariance
    kf.update(_rad(0.0), _rad(0.5) ** 2)  # re-center, stay initialized
    p_center = kf.covariance
    kf.predict(0.05, process_noise=0.05)  # larger loss Q
    p_loss = kf.covariance
    # the loss step must grow uncertainty more than the normal step
    assert (p_loss - p_center) > (p_normal - p0)


def test_covariance_floor():
    kf = YawKalmanFilter(process_noise=0.0, min_cov_deg=0.5)
    kf.update(_rad(0.0), _rad(0.1) ** 2)  # tiny measurement noise
    # repeated consistent updates drive P down but publishable cov is floored
    for _ in range(50):
        kf.update(_rad(0.0), _rad(0.1) ** 2)
        kf.predict(0.001)
    assert kf.publishable_covariance() >= _rad(0.5) ** 2 - 1e-12
