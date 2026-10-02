from policies.speed_policy import (
    brake_to_line,
    can_stop_comfortably,
)

def hold_stop(ego, limits, dt):
    return max(-limits.max_decel, -ego.v / dt)


class TrafficLightPolicy:
    def __init__(self):
        self.yellow_decision = {}
        self.stopped_at_red = set()
        self.previous_color = {}
        self.entered_legally = set()

    def evaluate(self, ego, light, limits, dt=0.1):
        if light is None:
            return None

        if dt <= 0:
            raise ValueError("dt must be positive")

        light_id = light["id"]
        color = light["color"]
        distance = light["distance_to_stop_line_m"]
        clear = (
            light["intersection_clear"]
            and light["pedestrian_clear"]
        )

        if color not in ("RED", "YELLOW", "GREEN"):
            raise ValueError(f"Unknown traffic-light color: {color}")

        # Reset phase-specific memory when the signal changes.
        if self.previous_color.get(light_id) != color:
            self.yellow_decision.pop(light_id, None)
            self.stopped_at_red.discard(light_id)

        self.previous_color[light_id] = color

        if light["past_stop_line"]:
            # Record a legal crossing for this simple signal experiment.
            legal_entry = (
                color == "GREEN" and clear
            ) or (
                color == "YELLOW"
                and self.yellow_decision.get(light_id) == "CLEAR"
                and clear
            )

            if legal_entry or light.get("entered_legally", False):
                self.entered_legally.add(light_id)

            if (
                color == "RED"
                and light_id not in self.entered_legally
            ):
                return (
                    hold_stop(ego, limits, dt),
                    "STOP_LINE_OVERRUN",
                )

            return None

        if color == "GREEN":
            if clear:
                return None

            if ego.v <= 0.05 and 0.0 <= distance <= 1.0:
                return hold_stop(ego, limits, dt), "WAIT_ON_GREEN"

            return (
                brake_to_line(ego.v, distance, limits),
                "YIELD_ON_GREEN",
            )

        if color == "YELLOW":
            if light_id not in self.yellow_decision:
                self.yellow_decision[light_id] = (
                    "STOP"
                    if can_stop_comfortably(
                        ego.v, distance, limits
                    )
                    else "CLEAR"
                )

            if (
                self.yellow_decision[light_id] == "CLEAR"
                and clear
            ):
                return None

            if ego.v <= 0.05 and 0.0 <= distance <= 1.0:
                return hold_stop(ego, limits, dt), "WAIT_AT_YELLOW"

            state = (
                "STOP_FOR_YELLOW"
                if self.yellow_decision[light_id] == "STOP"
                else "YIELD_ON_YELLOW"
            )

            return brake_to_line(ego.v, distance, limits), state

        # RED
        if ego.v <= 0.05 and 0.0 <= distance <= 1.0:
            self.stopped_at_red.add(light_id)

        may_turn_on_red = (
            light["is_turning"]
            and light["red_turn_permitted"]
            and light_id in self.stopped_at_red
            and ego.v <= 1e-9
            and clear
        )

        if may_turn_on_red:
            return None

        if light_id in self.stopped_at_red:
            return hold_stop(ego, limits, dt), "WAIT_AT_RED"

        return (
            brake_to_line(ego.v, distance, limits),
            "STOP_FOR_RED",
        )