import numpy as np

from scenarios import (
    get_scenario,
    GREEN_TIME_S,
    CONFLICT_CLEAR_TIME_S,
)


FRONT_OFFSET_M = 3.7


def world_to_ego(px, py, ego_x, ego_y, ego_yaw):
    dx = px - ego_x
    dy = py - ego_y
    relative_x = np.cos(ego_yaw) * dx + np.sin(ego_yaw) * dy
    relative_y = -np.sin(ego_yaw) * dx + np.cos(ego_yaw) * dy

    return relative_x, relative_y

def agent_states_at_time(config, current_time):
    states = []

    for initial in config.get("agents", []):
        states.append({
            "name": initial["name"],
            "x": initial["x"] + initial["vx"] * current_time,
            "y": initial["y"] + initial["vy"] * current_time,
            "vx": initial["vx"],
            "vy": initial["vy"],
        })

    return states

def observe(ego, agents, current_time, scenario="red_to_green", scenario_state=None,):
    config = get_scenario(scenario)
    moving_agents = agent_states_at_time(config, current_time)

    front_x = ego.x + FRONT_OFFSET_M * np.cos(ego.yaw)

    observation = {
        "lane": {
            "target_point": (
                ego.x + 10.0,
                config["lane_center_y"],
            ),
            "desired_speed_mps": config["speed_limit"],
            "posted_limit_mps": config["speed_limit"],
            "hazard_limit_mps": None,
        },

        "traffic_light": None,
        "stop_sign": None,
        "turn": None,
        "lane_change": None,
        "pedestrian_conflict": None,
        "vehicle_conflict": None,
    }

    light = config.get("traffic_light")

    if light is not None:
        distance = light["stop_line_x"] - front_x
        color = (
            "RED"
            if current_time < light["green_time_s"]
            else "GREEN"
        )

        observation["traffic_light"] = {
            "id": light["id"],
            "color": color,
            "distance_to_stop_line_m": distance,
            "past_stop_line": distance < 0.0,
            "intersection_clear": True,
            "pedestrian_clear": True,
            "is_turning": False,
            "red_turn_permitted": False,
        }

    sign = config.get("stop_sign")

    if sign is not None:
        distance = sign["stop_line_x"] - front_x

        if "conflict_clear_time_s" in sign:
            # Original experiment: clearance follows a timer.
            occupied = (
                current_time < sign["conflict_clear_time_s"]
            )
        else:
            # New experiment: clearance follows agent positions.
            occupied = any(
                abs(agent["x"] - sign["conflict_center_x"])
                <= sign["conflict_half_width_x"]
                and abs(agent["y"])
                <= sign["conflict_half_width_y"]
                for agent in moving_agents
            )

        observation["stop_sign"] = {
            "id": sign["id"],
            "distance_to_stop_line_m": distance,
            "intersection_clear": not occupied,
            "pedestrian_clear": True,
        }

    observation["agents"] = moving_agents

    change = config.get("lane_change")

    if change is not None:
        evaluated_change = dict(change)
        new_center = change["new_center_y"]

        if change.get("check_agents", False):
            blocked = any(
                abs(agent["y"] - new_center)
                <= change["target_lane_half_width_m"]

                and -change["rear_gap_m"]
                <= agent["x"] - ego.x
                <= change["front_gap_m"]

                for agent in moving_agents
            )

            evaluated_change["full_path_clear"] = (
                change["full_path_clear"] and not blocked
            )

        can_start = (
            evaluated_change["requested"]
            and evaluated_change["permitted_by_markings"]
            and evaluated_change["indicator_on"]
            and evaluated_change["full_path_clear"]
        )

        if change.get("start_when_clear", False):
            if (
                can_start
                and "lane_change_start_x" not in scenario_state
            ):
                scenario_state["lane_change_start_x"] = ego.x

            evaluated_change["start_x"] = scenario_state.get(
                "lane_change_start_x",
                ego.x,
            )

            started = "lane_change_start_x" in scenario_state
        else:
            started = True

        end_x = (
            evaluated_change["start_x"]
            + evaluated_change["length_m"]
        )

        if (
            started
            and ego.x >= end_x
            and abs(ego.y - new_center) < 0.1
            and abs(ego.yaw) < np.deg2rad(1.0)
        ):
            scenario_state["lane_change_completed"] = True

        if scenario_state.get("lane_change_completed", False):
            observation["lane"]["target_point"] = (
                ego.x + 10.0,
                new_center,
            )
        else:
            observation["lane_change"] = evaluated_change
    return observation
