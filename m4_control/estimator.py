"""Tiny complementary pose filter: bicycle dead-reckoning + GPS/compass correction.

Use it when your Webots GPS/compass have noise (set 'noise' on the devices) or when
you want smoother stopping decisions. alpha/beta = correction gain per step (0..1).
"""
import numpy as np
from .path import wrap


class PoseFilter:
    def __init__(self, L=2.7, alpha=0.1, beta=0.1):
        self.L, self.alpha, self.beta = L, alpha, beta
        self.x = self.y = self.yaw = None

    def update(self, zx, zy, zyaw, v, steer, dt):
        if self.x is None:
            self.x, self.y, self.yaw = zx, zy, zyaw
            return self.x, self.y, self.yaw
        # predict (rear-axle bicycle model)
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + v * np.tan(steer) / self.L * dt)
        # correct
        self.x += self.alpha * (zx - self.x)
        self.y += self.alpha * (zy - self.y)
        self.yaw = wrap(self.yaw + self.beta * wrap(zyaw - self.yaw))
        return self.x, self.y, self.yaw
