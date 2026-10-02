from dataclasses import dataclass
import numpy as np


@dataclass
class VehicleLimits:
    max_accel: float = 2.0          
    max_decel: float = 6.0          # the max decrease accerlation
    comfortable_decel: float = 2.5  
    max_steer: float = 0.5          # rad
    comfortable_lat_accel: float = 2.0  


def speed_control(v, target_speed, limits):
    kp = 1.5
    a = kp * (max(0.0, target_speed) - v)
    return float(np.clip(a, -limits.max_decel, limits.max_accel))


def lawful_target_speed(desired_speed, posted_limit, hazard_limit=None):
    candidates = [desired_speed, posted_limit]
    if hazard_limit is not None:
        candidates.append(hazard_limit)
    return max(0.0, min(candidates))


def braking_distance(v, decel, reaction_time=0.0, margin=0.5): #reaction time and margin is setting by myself
    return v * reaction_time + v**2 / (2 * decel) + margin


def brake_to_line(v, distance_to_line, limits):
    stop_margin = 0.8
    remaining = distance_to_line - stop_margin
    if v <= 0.01:
        return 0.0
    if remaining <= 0:
        return -limits.max_decel
    required_a = -(v**2) / (2 * remaining)
    return float(np.clip(required_a, -limits.max_decel, 0.0))



def can_stop_comfortably(v, distance_to_line, limits):
    return distance_to_line >= braking_distance(
        v,
        limits.comfortable_decel,
        reaction_time=0.2,
        margin=0.5,
    )