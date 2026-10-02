from environment import observe
from ego_vehicle import EgoVehicle
from agents import create_agents, update_agents
from policies import PolicyManager
from policies.speed_policy import VehicleLimits


dt = 0.1
total_time = 12.0

limits = VehicleLimits()
manager = PolicyManager(limits)

ego = EgoVehicle()
ego.v = 8.0

agents = create_agents()

for step in range(int(total_time / dt)):
    current_time = step * dt

    observation = observe(
        ego,
        agents,
        current_time,
    )

    acceleration, steering_angle, state = manager.step(
        ego,
        observation,
    )

    print(
        f"t={current_time:4.1f}  "
        f"x={ego.x:6.2f}  "
        f"v={ego.v:5.2f}  "
        f"a={acceleration:6.2f}  "
        f"delta={steering_angle:6.3f}  "
        f"state={state}"
    )

    ego.update(
        acceleration,
        steering_angle,
        dt,
        limits,
    )

    update_agents(agents, dt)