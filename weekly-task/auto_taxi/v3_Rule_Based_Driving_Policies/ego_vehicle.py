import numpy as np


class EgoVehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=5.0):
        self.x = x
        self.y = y
        self.yaw = yaw       # radians
        self.v = v           # m/s
        self.L = 2.990       # wheelbase, m

    def update(self, acceleration, steering_angle, dt, limits):
        acceleration = float(np.clip(
            acceleration,
            -limits.max_decel,
            limits.max_accel,
        ))
        steering_angle = float(np.clip(
            steering_angle,
            -limits.max_steer,
            limits.max_steer,
        ))

        # All derivatives use the state at the start of this time step.
        x_dot = self.v * np.cos(self.yaw)
        y_dot = self.v * np.sin(self.yaw)
        yaw_dot = self.v / self.L * np.tan(steering_angle)

        self.x += x_dot * dt
        self.y += y_dot * dt
        self.yaw += yaw_dot * dt
        self.v = max(self.v + acceleration * dt, 0.0)