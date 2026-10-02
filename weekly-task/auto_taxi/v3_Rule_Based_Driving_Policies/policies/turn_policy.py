import numpy as np

from policies.lane_policy import steer_to_point
from policies.speed_policy import (
    brake_to_line,
    lawful_target_speed,
    speed_control,
)


def turn_policy(ego, turn, limits):
    if turn is None:
        return None
    kind = turn["type"]
    if kind not in ("LEFT", "RIGHT", "U_TURN"):
        raise ValueError(f"Unknown turn: {kind}")
    legal = turn["permitted_by_signs_and_lanes"]

    # The left and right turns should be continuously indicated for at least 100 ft in advance
    signal_ready = (
        turn["indicator_on"]
        and turn["indicator_distance_m"] >= 30.48
    )

    if kind == "U_TURN" and turn["on_curve_or_crest"]:
        legal = legal and turn["visible_both_directions_m"] >= 152.4

    conflict_clear = (
        turn["path_clear"]
        and turn["pedestrian_clear"]
        and (kind != "LEFT" or turn["oncoming_clear"])
    )

    if not legal:
        return 0.0, 0.0, "TURN_PROHIBITED_REPLAN"

    if not signal_ready or not conflict_clear:
        a = brake_to_line(
            ego.v,
            turn["distance_to_entry_m"],
            limits,
        )
        return a, 0.0, "WAIT_TO_TURN"

    target_x, target_y = turn["path_target_point"]
    delta = steer_to_point(ego, target_x, target_y, limits)

    radius = max(turn["radius_m"], 0.1)
    curve_speed = np.sqrt(limits.comfortable_lat_accel * radius)

    target_speed = lawful_target_speed(
        desired_speed=curve_speed,
        posted_limit=turn["posted_limit_mps"],
    )

    a = speed_control(ego.v, target_speed, limits)
    return a, delta, f"{kind}_TURN"