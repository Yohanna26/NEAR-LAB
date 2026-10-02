import numpy as np
from policies.speed_policy import speed_control, lawful_target_speed


def steer_to_point(ego, target_x, target_y, limits): #in front of the vehicle
    dx = target_x - ego.x
    dy = target_y - ego.y
    relative_x = np.cos(ego.yaw) * dx + np.sin(ego.yaw) * dy
    relative_y = -np.sin(ego.yaw) * dx + np.cos(ego.yaw) * dy
    distance_squared = relative_x**2 + relative_y**2
    if relative_x <= 0 or distance_squared < 0.01:
        return 0.0
    curvature = 2 * relative_y / distance_squared
    delta = np.arctan(ego.L * curvature)
    return float(np.clip(delta, -limits.max_steer, limits.max_steer))


def lane_follow_policy(ego, lane, limits):
    """
    lane:
        target_point: (world_x, world_y)，地图提供的前方车道中心点
        desired_speed_mps
        posted_limit_mps
    """
    target_x, target_y = lane["target_point"]
    delta = steer_to_point(ego, target_x, target_y, limits)

    target_speed = lawful_target_speed(
        lane["desired_speed_mps"],
        lane["posted_limit_mps"],
        lane.get("hazard_limit_mps"),
    )
    a = speed_control(ego.v, target_speed, limits)

    return a, delta, "LANE_FOLLOW"


def lane_change_policy(ego, lane, change, limits): #Only work for the straight lane
    if not change["requested"]:
        return lane_follow_policy(ego, lane, limits)
    if not (
        change["permitted_by_markings"]
        and change["indicator_on"]
        and change["full_path_clear"]
    ):
        a, delta, _ = lane_follow_policy(ego, lane, limits)
        return a, delta, "WAIT_TO_CHANGE_LANE"
        
    lookahead_x = ego.x + max(4.0, ego.v * 1.2)

    s = (lookahead_x - change["start_x"]) / change["length_m"]
    s = float(np.clip(s, 0.0, 1.0))
    smooth = 3 * s**2 - 2 * s**3
    target_y = (
        change["old_center_y"]
        + (change["new_center_y"] - change["old_center_y"]) * smooth
    )
    delta = steer_to_point(ego, lookahead_x, target_y, limits)
    target_speed = lawful_target_speed(
        lane["desired_speed_mps"],
        lane["posted_limit_mps"],
        lane.get("hazard_limit_mps"),
    )
    a = speed_control(ego.v, target_speed, limits)
    return a, delta, "LANE_CHANGE"