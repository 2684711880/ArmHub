"""Base-frame constant-velocity Kalman filter with bounded occlusion prediction.

The filter time is the last accepted measurement time. Reading a prediction
never advances that time, so delayed camera packets can still update correctly.
"""
import numpy as np


class HolePositionEstimator:
    def __init__(self, measurement_std_m=.005, acceleration_noise=.005,
                 initial_velocity_std=.05, max_prediction_s=3., max_position_std_m=.025,
                 max_prediction_displacement_m=.05, max_speed_m_s=.2,
                 innovation_gate_squared=16., min_prediction_samples=5,
                 prediction_replan_m=.02, normal_gate_deg=15.):
        self.measurement_std_m = float(measurement_std_m)
        self.acceleration_noise = float(acceleration_noise)
        self.initial_velocity_std = float(initial_velocity_std)
        self.max_prediction_s = float(max_prediction_s)
        self.max_position_std_m = float(max_position_std_m)
        self.max_prediction_displacement_m = float(max_prediction_displacement_m)
        self.max_speed_m_s = float(max_speed_m_s)
        self.innovation_gate_squared = float(innovation_gate_squared)
        self.min_prediction_samples = int(min_prediction_samples)
        self.prediction_replan_m = float(prediction_replan_m)
        self.normal_gate_deg = float(normal_gate_deg)
        positive = [self.measurement_std_m, self.acceleration_noise, self.initial_velocity_std,
                    self.max_prediction_s, self.max_position_std_m, self.max_prediction_displacement_m,
                    self.max_speed_m_s, self.innovation_gate_squared, self.prediction_replan_m, self.normal_gate_deg]
        if not np.isfinite(positive).all() or min(positive) <= 0 or self.min_prediction_samples < 3:
            raise ValueError('Invalid hole estimator configuration')
        self.reset()

    def reset(self):
        self.x = None; self.covariance = None; self.stamp = None; self.samples = 0
        self.last_measurement = None; self.rejected = 0

    def _predict(self, stamp):
        dt = max(0., float(stamp)-self.stamp)
        identity = np.eye(3)
        f = np.eye(6); f[:3, 3:] = dt*identity
        # Continuous white acceleration density; covariance grows with dropout.
        q = self.acceleration_noise**2*np.block([
            [dt**3/3*identity, dt**2/2*identity], [dt**2/2*identity, dt*identity]])
        return f@self.x, f@self.covariance@f.T+q

    def update(self, position, stamp, plane_rms=0.):
        position = np.asarray(position, dtype=float); stamp = float(stamp)
        if position.shape != (3,) or not np.isfinite(position).all() or not np.isfinite([stamp, plane_rms]).all() or plane_rms < 0:
            raise ValueError('Invalid world-frame hole measurement')
        if self.stamp is not None and stamp <= self.stamp:
            return False
        variance = self.measurement_std_m**2+float(plane_rms)**2
        if self.x is None or stamp-self.stamp > self.max_prediction_s:
            self.x = np.r_[position, np.zeros(3)]
            self.covariance = np.diag([variance]*3+[self.initial_velocity_std**2]*3)
            self.stamp = stamp; self.samples = 1; self.last_measurement = position.copy()
            return True
        predicted, covariance = self._predict(stamp)
        residual = position-predicted[:3]
        r = np.eye(3)*variance; s = covariance[:3, :3]+r
        if float(residual@np.linalg.solve(s, residual)) > self.innovation_gate_squared:
            self.rejected += 1; return False
        gain = np.linalg.solve(s, covariance[:3, :]).T
        updated = predicted+gain@residual
        if np.linalg.norm(updated[3:]) > self.max_speed_m_s:
            self.rejected += 1; return False
        h = np.zeros((3, 6)); h[:, :3] = np.eye(3)
        kh = np.eye(6)-gain@h
        # Joseph form preserves symmetric positive covariance under rounding.
        self.covariance = kh@covariance@kh.T+gain@r@gain.T
        self.covariance = (self.covariance+self.covariance.T)/2
        self.x = updated; self.stamp = stamp; self.last_measurement = position.copy(); self.samples += 1
        return True

    def estimate(self, stamp):
        if self.x is None:
            return None
        if not np.isfinite(stamp):raise ValueError('Invalid prediction timestamp')
        age = float(stamp)-self.stamp
        # Never extrapolate an unbounded timestamp even after expiry.
        predicted, covariance = self._predict(self.stamp+min(max(age, 0.), self.max_prediction_s))
        sigma = float(np.sqrt(max(0., np.linalg.eigvalsh(covariance[:3, :3]).max())))
        displacement = float(np.linalg.norm(predicted[:3]-self.x[:3]))
        allowed = (0 <= age <= self.max_prediction_s and self.samples >= self.min_prediction_samples
                   and sigma <= self.max_position_std_m and displacement <= self.max_prediction_displacement_m)
        return {'position': predicted[:3].copy(), 'velocity': predicted[3:].copy(),
                'position_covariance': covariance[:3, :3].copy(), 'position_std_m': sigma,
                'age_s': age, 'samples': self.samples, 'prediction_allowed': bool(allowed),
                'rejected_measurements': self.rejected}
