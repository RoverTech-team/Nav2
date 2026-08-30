import math


def _wrap_to_pi(angle):
    """Wrap an angle in radians to the interval (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class YawKalmanFilter:
    """1-D Kalman filter over a continuous yaw angle (radians).

    State: x = yaw (continuous, not wrapped), P = variance (rad^2).
    The filter smooths intermittent absolute-yaw observations (e.g. from ArUco
    markers) and grows its uncertainty during gaps via process noise Q so that
    the downstream fusion (robot_localization EKF) can weight detections by
    confidence. Angle wrap is handled with the shortest-arc innovation.
    """

    def __init__(self, process_noise=1e-2, gate_sigma=3.0,
                 max_cov_deg=20.0, min_cov_deg=0.5):
        self._Q = float(process_noise)            # rad^2 / s
        self._gate_sigma = float(gate_sigma)
        self._max_cov = math.radians(max_cov_deg) ** 2
        self._min_cov = math.radians(min_cov_deg) ** 2
        self._x = 0.0
        self._P = 0.0
        self._initialized = False

    @property
    def initialized(self):
        return self._initialized

    @property
    def state(self):
        return self._x

    @property
    def covariance(self):
        return self._P

    def reset(self):
        self._initialized = False
        self._x = 0.0
        self._P = 0.0

    def predict(self, dt, process_noise=None):
        """Propagate uncertainty forward by dt seconds (no state change).

        `process_noise` overrides the configured Q for this step (e.g. a larger
        value while coasting during marker loss so covariance grows faster).
        Covariance is clamped to max_cov; the owning node performs the explicit
        reset/re-latch when the loss timeout elapses.
        """
        if not self._initialized:
            return
        dt = max(dt, 0.0)
        Q = self._Q if process_noise is None else float(process_noise)
        self._P += Q * dt
        if self._P > self._max_cov:
            self._P = self._max_cov

    def update(self, yaw_obs_rad, R_rad2):
        """Fuse a yaw observation. Returns True if accepted, False if gated."""
        R = float(R_rad2)
        if R <= 0.0:
            R = self._min_cov

        if not self._initialized:
            self._x = float(yaw_obs_rad)
            self._P = R
            self._initialized = True
            return True

        innovation = _wrap_to_pi(float(yaw_obs_rad) - self._x)
        gamma = self._P + R
        if abs(innovation) > self._gate_sigma * math.sqrt(gamma):
            return False

        K = self._P / gamma
        self._x = _wrap_to_pi(self._x + K * innovation)
        self._P = (1.0 - K) * self._P
        if self._P < self._min_cov:
            self._P = self._min_cov
        return True

    def publishable_covariance(self):
        return max(self._P, self._min_cov)
