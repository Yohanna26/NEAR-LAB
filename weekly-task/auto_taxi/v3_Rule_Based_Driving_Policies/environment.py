"""Static world objects and the world-to-ego coordinate transform."""
import numpy as np

ROAD = {
    "x_min": 0.0,
    "x_max": 40.0,
    "lane_center_y": 0.0,
    "lane_width": 3.6,
    "speed_limit": 5.0,
}

BLOCKS = [
    {"name": "block_1", "x": 8.0, "y": 4.0, "width": 4.0, "height": 3.0},
    {"name": "block_2", "x": 23.0, "y": -4.0, "width": 5.0, "height": 3.0},
]

TRAFFIC_LIGHTS = [
    {
        "name": "light_1",
        "x": 30.0,
        "y": 2.5,
        "stop_line_x": 30.0,
        "state": "RED",
    }
]

def world_to_ego(px, py, ego_x, ego_y, ego_yaw):
    dx = px - ego_x
    dy = py - ego_y
    cos_yaw = np.cos(ego_yaw)
    sin_yaw = np.sin(ego_yaw)
    relative_x = cos_yaw * dx + sin_yaw * dy
    relative_y = -sin_yaw * dx + cos_yaw * dy
    return relative_x, relative_y

def observe(ego, agents, current_time):
    light = TRAFFIC_LIGHTS[0]

    front_x = ego.x + 3.7
    distance_to_line = light["stop_line_x"] - front_x

    return {
        "lane": {
            "target_point": (
                ego.x + 10.0,
                ROAD["lane_center_y"],
            ),
            "desired_speed_mps": ROAD["speed_limit"],
            "posted_limit_mps": ROAD["speed_limit"],
            "hazard_limit_mps": None,
        },

        "traffic_light": {
            "id": light["name"],
            "color": light["state"],
            "distance_to_stop_line_m": distance_to_line,
            "past_stop_line": distance_to_line < 0,
            "intersection_clear": True,
            "pedestrian_clear": True,
            "is_turning": False,
            "red_turn_permitted": False,
        },

        "stop_sign": None,
        "turn": None,
        "lane_change": None,
        "pedestrian_conflict": None,
        "vehicle_conflict": None,
    }