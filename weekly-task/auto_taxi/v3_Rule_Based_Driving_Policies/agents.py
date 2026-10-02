def create_agents():
    return [
        {
            "name": "crossing_agent",
            "x": 16.0,
            "y": -5.0,
            "vx": 0.0,
            "vy": 2.0,
        },
        {
            "name": "adjacent_agent",
            "x": 10.0,
            "y": 3.0,
            "vx": 3.0,
            "vy": 0.0,
        },
    ]


def update_agents(agents, dt):
    for agent in agents:
        agent["x"] += agent["vx"] * dt
        agent["y"] += agent["vy"] * dt