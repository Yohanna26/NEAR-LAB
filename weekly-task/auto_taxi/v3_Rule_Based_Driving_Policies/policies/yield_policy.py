from policies.speed_policy import brake_to_line


def pedestrian_yield_policy(ego, pedestrian_conflict, limits):

    if pedestrian_conflict is None:
        return None

    if not pedestrian_conflict["must_yield"]:
        return None

    a = brake_to_line(
        ego.v,
        pedestrian_conflict["distance_to_conflict_m"],
        limits,
    )
    return a, 0.0, "PEDESTRIAN_YIELD"


def vehicle_yield_policy(ego, vehicle_conflict, limits):

    if vehicle_conflict is None:
        return None

    if not vehicle_conflict["must_yield"]:
        return None

    a = brake_to_line(
        ego.v,
        vehicle_conflict["distance_to_conflict_m"],
        limits,
    )
    return a, 0.0, "VEHICLE_YIELD"