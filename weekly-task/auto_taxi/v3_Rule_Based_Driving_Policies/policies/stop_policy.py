from policies.speed_policy import brake_to_line


class StopSignPolicy:
    def __init__(self):
        self.completed_stops = set()  # It has been completely parked in front of the corresponding parking sign

    def evaluate(self, ego, stop_sign, limits):
        if stop_sign is None:
            return None

        sign_id = stop_sign["id"]
        d = stop_sign["distance_to_stop_line_m"]

        if sign_id not in self.completed_stops:
            if ego.v <= 0.05 and 0 <= d <= 1.0:
                self.completed_stops.add(sign_id)
            else:
                return brake_to_line(ego.v, d, limits), "STOP_FOR_SIGN"

        if not (
            stop_sign["intersection_clear"]
            and stop_sign["pedestrian_clear"]
        ):
            return 0.0, "WAIT_AT_STOP_SIGN"

        return None