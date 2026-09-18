## Ego Autonomous (Bicycle Model) + Static Blocks + Moving Agents(Constant-Velocity Model) + 6s Simulation + Ego-Centered Coordicates + Simple Interaction
## Interaction Explain: If another agent enters the safe area (0-8m ahead and +/- 1.5 m laterally)in front of the vehicle, the vehicle slows down. For now, won't take a detour.

import matplotlib
matplotlib.use("Agg")
import numpy as np
import matplotlib.pyplot as plt

L = 2.990 # wheelbase(m)
dt = 0.1 #simulation time step(s)
total_time = 6 #total simulation time(s)

x = 0.0
y = 0.0
yaw = 0.0        # radians
v = 5.0          # m/s

blocks = [
    {"name": "block_1",
    "x": 8.0,
    "y": 4.0,
    "width": 4.0,
    "height": 3.0},

    {"name": "block_2",
    "x": 23.0,
    "y": -4.0,
    "width": 5.0,
    "height": 3.0}
]

agents = [
    {"name": "crossing_agent",
    "x": 16.0,
    "y": -5.0,
    "vx": 0.0,
    "vy": 2.0},

    {"name": "adjacent_agent",
    "x": 10.0,
    "y": 3.0,
    "vx": 3.0,
    "vy": 0.0}
]

def world_to_ego(px,py,ego_x,ego_y,ego_yaw):
    dx = px - ego_x
    dy = py - ego_y
    cos_yaw = np.cos(ego_yaw)
    sin_yaw = np.sin(ego_yaw)
    x_relative = (cos_yaw*dx + sin_yaw*dy)
    y_relative = (-sin_yaw*dx + cos_yaw*dy)
    return x_relative, y_relative

def bicycle_model(x, y, yaw, velocity, acceleration, steering_angle):
    x_dot = velocity * np.cos(yaw)
    y_dot = velocity * np.sin(yaw)
    yaw_dot = (velocity/L) * np.tan(steering_angle)
    velocity_dot = acceleration
    x_new = x + x_dot * dt 
    y_new = y + y_dot * dt
    yaw_new = yaw + yaw_dot * dt
    velocity_new = velocity + velocity_dot * dt
    velocity_new = max(velocity_new, 0,0)
    return x_new, y_new, yaw_new, velocity_new

def Interaction_controller(ego_x, ego_y, ego_yaw, ego_v, agents):
    normal_speed =5.0
    desired_speed = normal_speed
    danger_detected = False
    for agent in agents:
        relative_x, relative_y = world_to_ego(
            agent ["x"],
            agent ["y"],
            ego_x,
            ego_y,
            ego_yaw
        )
        in_danger_zone = (0.0<relative_x<8.0 and abs(relative_y)<1.5)
        print(
            f"    {agent['name']}: "
            f"rel_x={relative_x:.2f}, "
            f"rel_y={relative_y:.2f}, "
            f"in_zone={in_danger_zone}"
        )
        if (in_danger_zone):
            desired_speed=1.5
            danger_detected = True
    speed_error = desired_speed - ego_v 
    acceleration = 1.5 * speed_error
    acceleration = np.clip(acceleration, -3.0, 2.0) # Limit Acceleration
    return(acceleration, danger_detected)

#Data Storage
time_history =[]
ego_x_history = []
ego_y_history =[]
ego_yaw_history =[]
ego_v_history =[]
acceleration_history =[]
danger_history =[]
agent_history={}
for agent in agents:
    agent_history[agent["name"]]={
        "x":[],
        "y":[],
        "relative_x":[],
        "relative_y":[]
    }
block_relative_history={}
for block in blocks:
    block_relative_history[block["name"]]={
        "x":[],
        "y":[]
    }

number_of_steps = int(total_time / dt)
for step in range(number_of_steps):
    current_time = step * dt
    #Save ego State
    time_history.append(current_time)
    ego_x_history.append(x)
    ego_y_history.append(y)
    ego_yaw_history.append(yaw)
    ego_v_history.append(v)
    #Conver agents into ego-centered
    for agent in agents:
        relative_x, relative_y = world_to_ego(agent["x"], agent["y"],x,y,yaw)
        agent_history[agent["name"]]["x"].append(agent["x"])
        agent_history[agent["name"]]["y"].append(agent["y"])
        agent_history[agent["name"]]["relative_x"].append(relative_x)
        agent_history[agent["name"]]["relative_y"].append(relative_y)
    #Convert static blocks into ego-centered
    for block in blocks:
        relative_x, relative_y = world_to_ego(block["x"],block["y"],x,y,yaw)
        block_relative_history[block["name"]]["x"].append(relative_x)
        block_relative_history[block["name"]]["y"].append(relative_y)
    #Intercation
    acceleration, danger = Interaction_controller(x,y,yaw,v,agents)
    print(
    f"t={current_time:.1f}, "
    f"v={v:.2f}, "
    f"a={acceleration:.2f}, "
    f"danger={danger}")
    acceleration_history.append(acceleration)
    danger_history.append(danger)
    steering_angle = 0.0 # Maybe Change next week
    x, y, yaw, v = bicycle_model(x,y, yaw, v,acceleration,steering_angle)
    for agent in agents:
        agent["x"] += (agent["vx"] * dt)
        agent["y"] += (agent["vy"] * dt)









print(
    "Simulation finished."
)

print(
    f"Simulation time: {total_time} seconds"
)

print(
    f"Final ego position: ({x:.2f}, {y:.2f}) m"
)

print(
    f"Final ego velocity: {v:.2f} m/s"
)

print(
    "Interaction detected:",
    any(danger_history)
)


plt.figure(
    figsize=(10, 6)
)

plt.plot(
    ego_x_history,
    ego_y_history,
    label="Ego Taxi"
)


for agent in agents:

    name = agent["name"]

    plt.plot(
        agent_history[name]["x"],
        agent_history[name]["y"],
        label=name
    )


# Draw blocks
for block in blocks:

    rectangle = plt.Rectangle(
        (
            block["x"] - block["width"] / 2,
            block["y"] - block["height"] / 2
        ),
        block["width"],
        block["height"],
        alpha=0.4
    )

    plt.gca().add_patch(
        rectangle
    )


plt.xlabel(
    "World X [m]"
)

plt.ylabel(
    "World Y [m]"
)

plt.title(
    "6-Second Simulation"
)

plt.axis(
    "equal"
)

plt.grid()

plt.legend()

plt.savefig(
    "world_trajectory.png",
    dpi=200,
    bbox_inches="tight"
)

plt.close()



plt.figure(
    figsize=(10, 6)
)


# Ego is always at origin
plt.scatter(
    0,
    0,
    marker="*",
    s=200,
    label="Ego Taxi"
)


# Relative trajectories of dynamic agents
for agent in agents:

    name = agent["name"]

    plt.plot(
        agent_history[name]["relative_x"],
        agent_history[name]["relative_y"],
        label=name
    )


# Relative trajectories of block centers
for block in blocks:

    name = block["name"]

    plt.plot(
        block_relative_history[name]["x"],
        block_relative_history[name]["y"],
        linestyle="--",
        label=name
    )


plt.axhline(
    0.0,
    linewidth=1
)

plt.axvline(
    0.0,
    linewidth=1
)

plt.xlabel(
    "Relative X [m]"
)

plt.ylabel(
    "Relative Y [m]"
)

plt.title(
    "Environment in Ego-Centered Coordinates"
)

plt.axis(
    "equal"
)

plt.grid()

plt.legend()

plt.savefig(
    "ego_centered_trajectory.png",
    dpi=200,
    bbox_inches="tight"
)

plt.close()


plt.figure(
    figsize=(9, 5)
)

plt.plot(
    time_history,
    ego_v_history
)

plt.xlabel(
    "Time [s]"
)

plt.ylabel(
    "Velocity [m/s]"
)

plt.title(
    "Ego Taxi Velocity"
)

plt.grid()

plt.savefig(
    "ego_velocity.png",
    dpi=200,
    bbox_inches="tight"
)

plt.close()


print(
    "Saved:"
)

print(
    "  world_trajectory.png"
)

print(
    "  ego_centered_trajectory.png"
)

print(
    "  ego_velocity.png"
)



