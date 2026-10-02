from policies.lane_policy import lane_follow_policy, lane_change_policy
from policies.traffic_light_policy import TrafficLightPolicy
from policies.stop_policy import StopSignPolicy
from policies.turn_policy import turn_policy
from policies.yield_policy import (
    pedestrian_yield_policy,
    vehicle_yield_policy,
)


class PolicyManager:
    def __init__(self, limits):
        self.limits = limits
        self.light_policy = TrafficLightPolicy()
        self.stop_policy = StopSignPolicy()

    def step(self, ego, observation):
        lane = observation["lane"]
        change = observation.get("lane_change")
        turn = observation.get("turn")

        if turn is not None:
            proposed = turn_policy(ego, turn, self.limits)
        elif change is not None:
            proposed = lane_change_policy(
                ego, lane, change, self.limits
            )
        else:
            proposed = lane_follow_policy(ego, lane, self.limits)

        pedestrian = pedestrian_yield_policy(
            ego,
            observation.get("pedestrian_conflict"),
            self.limits,
        )
        if pedestrian is not None:
            return pedestrian

        light = self.light_policy.evaluate(
            ego,
            observation.get("traffic_light"),
            self.limits,
        )
        if light is not None:
            a, state = light
            return a, 0.0, state

        stop_sign = self.stop_policy.evaluate(
            ego,
            observation.get("stop_sign"),
            self.limits,
        )
        if stop_sign is not None:
            a, state = stop_sign
            return a, 0.0, state

        vehicle = vehicle_yield_policy(
            ego,
            observation.get("vehicle_conflict"),
            self.limits,
        )
        if vehicle is not None:
            return vehicle

        return proposed
