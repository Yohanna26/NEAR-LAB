GREEN_TIME_S = 8.0
CONFLICT_CLEAR_TIME_S = 10.0


SCENARIOS = {
    "red_to_green": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "traffic_light": {
            "id": "light_1",
            "stop_line_x": 30.0,
            "green_time_s": GREEN_TIME_S,
        },

        "stop_sign": None,
    },

    "stop_sign": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "traffic_light": None,

        "stop_sign": {
            "id": "stop_1",
            "stop_line_x": 30.0,
            "conflict_clear_time_s": CONFLICT_CLEAR_TIME_S,
        },

    },
        "stop_sign_agent": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "traffic_light": None,

        "stop_sign": {
            "id": "stop_agent_1",
            "stop_line_x": 30.0,
            "conflict_center_x": 32.0,
            "conflict_half_width_x": 3.0,
            "conflict_half_width_y": 5.0,
        },

        "agents": [
            {
                "name": "crossing_vehicle",
                "x": 32.0,
                "y": -4.0,
                "vx": 0.0,
                "vy": 1.0,
            },
        ],
    },
        "lane_follow": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "initial_state": {
            "x": 0.0,
            "y": 1.0,
            "yaw": 0.1,
            "v": 5.0,
        },

        "traffic_light": None,
        "stop_sign": None,
        "agents": [],
    },
        "lane_change": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "initial_state": {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "v": 5.0,
        },

        "traffic_light": None,
        "stop_sign": None,
        "agents": [],

        "lane_change": {
            "requested": True,
            "permitted_by_markings": True,
            "indicator_on": True,
            "full_path_clear": True,
            "start_x": 0.0,
            "length_m": 30.0,
            "old_center_y": 0.0,
            "new_center_y": 3.6,
        },
    },
        "lane_change_blocked": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "initial_state": {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "v": 5.0,
        },

        "traffic_light": None,
        "stop_sign": None,

        "agents": [
            {
                "name": "adjacent_vehicle",
                "x": 5.0,
                "y": 3.6,
                "vx": 5.0,
                "vy": 0.0,
            },
        ],

        "lane_change": {
            "requested": True,
            "permitted_by_markings": True,
            "indicator_on": True,
            "full_path_clear": True,
            "start_x": 0.0,
            "length_m": 30.0,
            "old_center_y": 0.0,
            "new_center_y": 3.6,

            "check_agents": True,
            "front_gap_m": 12.0,
            "rear_gap_m": 12.0,
            "target_lane_half_width_m": 1.8,
        },
    },
        "lane_change_wait_then_go": {
        "lane_center_y": 0.0,
        "speed_limit": 5.0,

        "initial_state": {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "v": 5.0,
        },

        "traffic_light": None,
        "stop_sign": None,

        "agents": [
            {
                "name": "adjacent_vehicle",
                "x": 5.0,
                "y": 3.6,
                "vx": 7.0,
                "vy": 0.0,
            },
        ],

        "lane_change": {
            "requested": True,
            "permitted_by_markings": True,
            "indicator_on": True,
            "full_path_clear": True,
            "start_x": 0.0,
            "length_m": 30.0,
            "old_center_y": 0.0,
            "new_center_y": 3.6,

            "check_agents": True,
            "front_gap_m": 12.0,
            "rear_gap_m": 12.0,
            "target_lane_half_width_m": 1.8,
            "start_when_clear": True,
        },
    },
}


def get_scenario(name):
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {name}")

    return SCENARIOS[name]
