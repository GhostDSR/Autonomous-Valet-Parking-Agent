"""Discrete PID with anti-windup and a filtered derivative."""
import numpy as np


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0, d_tau=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.d_tau = d_tau
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev_err = None
        self.d_filt = 0.0

    def step(self, err, dt):
        # derivative on error, low-pass filtered
        d_raw = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        a = dt / (self.d_tau + dt)
        self.d_filt += a * (d_raw - self.d_filt)
        self.prev_err = err

        u_unsat = self.kp * err + self.i + self.kd * self.d_filt
        u = float(np.clip(u_unsat, self.out_min, self.out_max))

        # conditional integration: don't wind up while saturated in the error's direction
        saturated_same_dir = (u != u_unsat) and (u_unsat * err > 0)
        if self.ki and not saturated_same_dir:
            self.i += self.ki * err * dt
        return u
