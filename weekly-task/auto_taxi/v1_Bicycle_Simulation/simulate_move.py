import numpy as np
import matplotlib.pyplot as plt

L = 2.990 # wheelbase(m)
dt = 0.1 #simulation time step(s)
total_time = 6 #total simulation time(s)

x = 0.0
y = 0.0
yaw = 0.0        # radians
v = 5.0          # m/s

x_history = [x]
y_history = [y]
yaw_history = [yaw]
v_history = [v]
steering_history = []
time_history = [0.0]

def bicycle_model(x, y, yaw, v, acceleration, steering_angle):
    x_dot = v * np.cos(yaw)
    y_dot = v * np.sin(yaw)
    yaw_dot = (v/L) * np.tan(steering_angle)
    v_dot = acceleration
    x_new = x + x_dot * dt 
    y_new = y + y_dot * dt
    yaw_new = yaw + yaw_dot * dt
    v_new = v + v_dot * dt
    return x_new, y_new, yaw_new, v_new

num_steps = int(total_time / dt)

for step in range(num_steps):
    time = step * dt
    acceleration = 0.0
    if time < 1:
        steering_angle = 0.0
    elif time < 3:
        steering_angle = np.deg2rad(5)
    elif time < 5:
        steering_angle = np.deg2rad(-5)
    else:
        steering_angle = 0.0
    
    x, y, yaw, v = bicycle_model(
        x,
        y,
        yaw,
        v,
        acceleration,
        steering_angle
    )

    x_history.append(x)
    y_history.append(y)
    yaw_history.append(yaw)
    v_history.append(v)
    steering_history.append(np.rad2deg(steering_angle))
    time_history.append((step + 1) * dt)

plt.figure()
plt.plot(
    x_history,
    y_history,
    label="Vehicle Path"
)
plt.scatter(
    x_history[0],
    y_history[0],
    label="Start"
)
plt.scatter(
    x_history[-1],
    y_history[-1],
    label="End"
)
plt.xlabel("X Position [m]")
plt.ylabel("Y Position [m]")
plt.title("Autonomous Taxi Trajectory")
plt.axis("equal")
plt.grid()
plt.legend()
plt.savefig(
    "vehicle_trajectory.png",
    dpi=300,
    bbox_inches="tight"
)
plt.close()


plt.figure()
plt.plot(
    time_history,
    v_history
)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Vehicle Velocity")
plt.grid()
plt.savefig(
    "vehicle_velocity.png",
    dpi=300,
    bbox_inches="tight"
)
plt.close()

plt.figure()
plt.plot(
    time_history,
    np.rad2deg(yaw_history)
)
plt.xlabel("Time [s]")
plt.ylabel("Heading [deg]")
plt.title("Vehicle Heading")
plt.grid()
plt.savefig(
    "vehicle_heading.png",
    dpi=300,
    bbox_inches="tight"
)
plt.close()

plt.figure()
plt.plot(
    time_history[:-1],
    steering_history
)
plt.xlabel("Time [s]")
plt.ylabel("Steering Angle [deg]")
plt.title("Steering Input")
plt.grid()
plt.savefig(
    "steering_input.png",
    dpi=300,
    bbox_inches="tight"
)

plt.close()
print("Simulation complete.")
print("Plots saved successfully.")