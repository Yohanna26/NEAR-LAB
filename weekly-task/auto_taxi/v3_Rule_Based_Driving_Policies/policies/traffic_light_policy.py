from policies.speed_policy import (
    brake_to_line,
    can_stop_comfortably,
)


class TrafficLightPolicy:
    def __init__(self):
        self.yellow_decision = {}  # light_id -> "STOP" or "CLEAR"
        self.stopped_at_red = set()

    def evaluate(self, ego, light, limits):
        if light is None:
            return None
        if light["past_stop_line"]:
            if (
                light["color"] == "RED"
                and not light.get("entered_legally", False)
            ):
                a = -limits.max_decel if ego.v > 0 else 0.0
                return a, "STOP_LINE_OVERRUN"

            return None
        light_id = light["id"]
        
        d = light["distance_to_stop_line_m"]
        clear = light["intersection_clear"] and light["pedestrian_clear"]
        if light["color"] == "GREEN":
            if clear:
                return None
            return brake_to_line(ego.v, d, limits), "YIELD_ON_GREEN"
        if light["color"] == "YELLOW":
            if light_id not in self.yellow_decision:
                self.yellow_decision[light_id] = (
                    "STOP" if can_stop_comfortably(ego.v, d, limits)
                    else "CLEAR"
                )
            if self.yellow_decision[light_id] == "STOP":
                return brake_to_line(ego.v, d, limits), "STOP_FOR_YELLOW"
            if clear:
                return None
            return brake_to_line(ego.v, d, limits), "YIELD_ON_YELLOW"

        if light["color"] == "RED":
            if ego.v <= 0.05 and 0 <= d <= 1.0:
                self.stopped_at_red.add(light_id)

            may_turn_on_red = (
                light["is_turning"]
                and light["red_turn_permitted"]
                and light_id in self.stopped_at_red
                and clear
            )

            if may_turn_on_red:
                return None

            if light_id in self.stopped_at_red:
                return 0.0, "WAIT_AT_RED"

            return brake_to_line(ego.v, d, limits), "STOP_FOR_RED"

        raise ValueError(f"Unknown traffic-light color: {light['color']}")