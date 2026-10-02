from policies.speed_policy import brake_to_line
class StopSignPolicy:
    def __init__(self, minimum_stop_time=1.0):
        self.minimum_stop_time = minimum_stop_time
        self.completed_stops = set()
        self.stationary_time = {}
        
    def evaluate(self, ego, stop_sign, limits, dt=0.1):
        if stop_sign is None:
            return None

        if dt <= 0:
            raise ValueError("dt must be positive")

        sign_id = stop_sign["id"]
        distance = stop_sign["distance_to_stop_line_m"]

        # Negative distance means the vehicle front has crossed the line.
        if distance < 0.0:
            if sign_id not in self.completed_stops:
                return (
                    self.hold_stop(ego, limits, dt),
                    "STOP_SIGN_OVERRUN",
                )
            return None

        # First approach: brake, then remain fully stopped.
        if sign_id not in self.completed_stops:
            near_line = 0.0 <= distance <= 1.0

            if near_line and ego.v <= 0.05:
                # Low speed is not yet a complete stop.
                if ego.v > 1e-9:
                    self.stationary_time[sign_id] = 0.0
                    return (
                        self.hold_stop(ego, limits, dt),
                        "STOP_FOR_SIGN",
                    )

                elapsed = self.stationary_time.get(sign_id, 0.0)

                if elapsed + 1e-9 < self.minimum_stop_time:
                    self.stationary_time[sign_id] = elapsed + dt
                    return 0.0, "HOLD_AT_STOP_SIGN"

                self.completed_stops.add(sign_id)

            else:
                self.stationary_time[sign_id] = 0.0
                return (
                    brake_to_line(ego.v, distance, limits),
                    "STOP_FOR_SIGN",
                )

        # Stopping is complete, but departure also requires a clear road.
        clear = (
            stop_sign["intersection_clear"]
            and stop_sign["pedestrian_clear"]
        )

        if not clear:
            return (
                self.hold_stop(ego, limits, dt),
                "WAIT_AT_STOP_SIGN",
            )

        # Let lane-following control resume.
        return None

    @staticmethod
    def hold_stop(ego, limits, dt):
        return max(-limits.max_decel, -ego.v / dt)