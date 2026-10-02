import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import math
from scenarios import get_scenario
LATERAL_SCENARIOS = ('lane_follow', 'lane_change', 'lane_change_blocked', 'lane_change_wait_then_go')
CHANGING_SCENARIOS = ('lane_change', 'lane_change_wait_then_go')
from environment import observe, GREEN_TIME_S, CONFLICT_CLEAR_TIME_S
from ego_vehicle import EgoVehicle
from policies import PolicyManager
from policies.speed_policy import VehicleLimits

def release_time_for(records, scenario):
    if scenario == 'red_to_green':
        return GREEN_TIME_S
    if scenario == 'stop_sign':
        return CONFLICT_CLEAR_TIME_S
    return next((r['t'] for r in records if r['clear']), None)

def run_simulation(scenario):
    dt = 0.1
    total_time = 18.0
    limits = VehicleLimits()
    manager = PolicyManager(limits)
    config = get_scenario(scenario)
    initial_state = config.get('initial_state', {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'v': 8.0})
    ego = EgoVehicle(**initial_state)
    agents = []
    scenario_state = {}
    records = []
    previous_state = None
    steps = round(total_time / dt)
    for step in range(steps + 1):
        t = step * dt
        observation = observe(ego, agents, t, scenario=scenario, scenario_state=scenario_state)
        acceleration, steering, state = manager.step(ego, observation, dt)
        if scenario in LATERAL_SCENARIOS:
            signal = 'NONE'
            clear = True
            distance = float('nan')
        else:
            if scenario == 'red_to_green':
                control = observation['traffic_light']
                signal = control['color']
            else:
                control = observation['stop_sign']
                signal = 'CLEAR' if control['intersection_clear'] and control['pedestrian_clear'] else 'BLOCKED'
            clear = control['intersection_clear'] and control['pedestrian_clear']
            distance = control['distance_to_stop_line_m']
        records.append({'t': t, 'x': float(ego.x), 'y': float(ego.y), 'yaw': float(ego.yaw), 'v': float(ego.v), 'a': float(acceleration), 'delta': float(steering), 'state': state, 'signal': signal, 'clear': clear, 'distance_to_line': float(distance), 'agent_y': float(observation['agents'][0]['y']) if observation['agents'] else None, 'lane_change_clear': observation['lane_change']['full_path_clear'] if observation['lane_change'] is not None else True, 'agent_gap': float(observation['agents'][0]['x'] - ego.x) if observation['agents'] else None})
        if state != previous_state or (scenario in LATERAL_SCENARIOS and step % 20 == 0):
            if scenario in LATERAL_SCENARIOS:
                print(f't={t:5.2f} s | y={ego.y:7.3f} m | yaw={math.degrees(ego.yaw):7.3f} deg | delta={steering:7.4f} rad | {state}')
            else:
                print(f't={t:5.2f} s | v={ego.v:5.2f} m/s | d={distance:6.2f} m | {signal} | {state}')
            previous_state = state
        if step < steps:
            ego.update(acceleration, steering, dt, limits)
    return records

def save_results(records, scenario):
    output_dir = Path(__file__).resolve().parent / 'results'
    output_dir.mkdir(exist_ok=True)
    csv_path = output_dir / f'{scenario}.csv'
    with csv_path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(records[0].keys()))
        writer.writeheader()
        writer.writerows(records)
    if scenario in LATERAL_SCENARIOS:
        config = get_scenario(scenario)
        center_y = config['lane_change']['new_center_y'] if scenario in CHANGING_SCENARIOS else config['lane_center_y']
        time = [r['t'] for r in records]
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        axes[0].plot(time, [r['y'] - center_y for r in records])
        axes[0].set_ylabel('Lateral error (m)')
        axes[1].plot(time, [math.degrees(r['yaw']) for r in records])
        axes[1].set_ylabel('Heading error (deg)')
        axes[2].plot(time, [r['delta'] for r in records])
        axes[2].set_ylabel('Steering angle (rad)')
        axes[2].set_xlabel('Time (s)')
        for ax in axes:
            ax.axhline(0.0, color='black', linestyle=':')
            ax.grid(alpha=0.3)
        fig.suptitle(scenario.replace('_', ' ').title())
        fig.tight_layout()
        figure_path = output_dir / f'{scenario}.png'
        fig.savefig(figure_path, dpi=180)
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(12, 3))
        xs = [r['x'] for r in records]
        ys = [r['y'] for r in records]
        ax.plot(xs, ys, label='Vehicle trajectory')
        ax.axhline(center_y, color='black', linestyle='--', label='Lane center')
        ax.axhline(center_y + 1.8, color='gray', linestyle=':')
        ax.axhline(center_y - 1.8, color='gray', linestyle=':')
        ax.scatter(xs[0], ys[0], color='green', label='Start')
        ax.scatter(xs[-1], ys[-1], color='red', label='End')
        ax.set_xlabel('World x (m)')
        ax.set_ylabel('World y (m)')
        ax.set_title(f'{scenario}: trajectory — lateral scale enlarged')
        old_center = config['lane_center_y']
        ax.axhline(old_center, color='gray', linestyle='--', label='Original lane center')
        ax.set_ylim(min(old_center, center_y) - 2.0, max(old_center, center_y) + 2.0)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / f'{scenario}_trajectory.png', dpi=180)
        plt.close(fig)
        print(f'\nCSV: {csv_path}')
        print(f'Plot: {figure_path}')
        return
    release_time = release_time_for(records, scenario)
    if release_time is None:
        raise RuntimeError('Intersection never became clear')
    time = [r['t'] for r in records]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(time, [r['v'] for r in records])
    axes[0].set_ylabel('Speed (m/s)')
    axes[1].plot(time, [r['a'] for r in records])
    axes[1].set_ylabel('Acceleration (m/s²)')
    axes[2].plot(time, [r['distance_to_line'] for r in records])
    axes[2].axhline(0.0, color='black', linestyle=':')
    axes[2].set_ylabel('Front-to-line distance (m)')
    axes[2].set_xlabel('Time (s)')
    for ax in axes:
        ax.axvline(release_time, color='green', linestyle='--', label='Release condition becomes true')
        ax.grid(alpha=0.3)
    axes[0].legend()
    fig.suptitle(scenario.replace('_', ' ').title())
    fig.tight_layout()
    figure_path = output_dir / f'{scenario}.png'
    fig.savefig(figure_path, dpi=180)
    plt.close(fig)
    print(f'\nCSV: {csv_path}')
    print(f'Plot: {figure_path}')

def check_results(records, scenario):
    if scenario in LATERAL_SCENARIOS:
        config = get_scenario(scenario)
        center_y = config['lane_change']['new_center_y'] if scenario in ('lane_change', 'lane_change_wait_then_go') else config['lane_center_y']
        limits = VehicleLimits()
        tail_start = records[-1]['t'] - 3.0
        tail = [r for r in records if r['t'] >= tail_start]
        checks = {'Lateral error below 0.1 m in final 3 seconds': all((abs(r['y'] - center_y) < 0.1 for r in tail)), 'Heading error below 1 degree in final 3 seconds': all((abs(math.degrees(r['yaw'])) < 1.0 for r in tail)), 'Steering stayed within limits': all((abs(r['delta']) <= limits.max_steer + 1e-09 for r in records)), 'Vehicle continued moving forward': records[-1]['x'] > records[0]['x'] and all((r['v'] > 0.0 for r in records))}
        if scenario in CHANGING_SCENARIOS:
            checks['Lane-change policy activated'] = any((r['state'] == 'LANE_CHANGE' for r in records))
            checks['Resumed lane following on new lane'] = all((r['state'] == 'LANE_FOLLOW' for r in tail))
        if scenario == 'lane_change_blocked':
            checks['Stayed in original lane throughout'] = all((abs(r['y'] - config['lane_center_y']) < 0.1 for r in records))
            checks['Waited instead of changing lanes'] = all((r['state'] == 'WAIT_TO_CHANGE_LANE' for r in records))
        if scenario == 'lane_change_wait_then_go':
            blocked_records = [r for r in records if not r['lane_change_clear']]
            first_change = next((r for r in records if r['state'] == 'LANE_CHANGE'), None)
            checks['Waited in original lane while blocked'] = bool(blocked_records) and all((r['state'] == 'WAIT_TO_CHANGE_LANE' and abs(r['y']) < 0.1 for r in blocked_records))
            checks['Started changing only after gap was safe'] = first_change is not None and first_change['lane_change_clear'] and (first_change['agent_gap'] > config['lane_change']['front_gap_m'])
        print('\nChecks:')
        for name, passed in checks.items():
            print(f"{('PASS' if passed else 'FAIL')}: {name}")
        if not all(checks.values()):
            raise RuntimeError('Lane-following scenario failed')
        return
    release_time = release_time_for(records, scenario)
    if release_time is None:
        raise RuntimeError('Intersection never became clear')
    before_release = [r for r in records if r['t'] < release_time]
    stopped = [r for r in before_release if r['v'] <= 1e-09 and 0.0 <= r['distance_to_line'] <= 1.0]
    checks = {'No crossing before release': all((r['distance_to_line'] >= 0.0 for r in before_release)), 'Stopped within 1 m of the line': bool(stopped), 'Restarted after release': any((r['t'] > release_time and r['v'] > 0.5 for r in records))}
    if stopped:
        stop_time = stopped[0]['t']
        waiting = [r for r in before_release if r['t'] >= stop_time]
        checks['Held position before release'] = max((r['x'] for r in waiting)) - min((r['x'] for r in waiting)) <= 1e-06 and all((r['v'] <= 1e-09 for r in waiting))
    else:
        stop_time = None
        checks['Held position before release'] = False
    if scenario in ('stop_sign', 'stop_sign_agent'):
        departure = next((r for r in records if stop_time is not None and r['t'] > stop_time and (r['v'] > 1e-09)), None)
        checks['Stopped for at least 1 second'] = departure is not None and departure['t'] - stop_time >= 1.0 - 1e-09
        checks['Entered stop-hold state'] = any((r['state'] == 'HOLD_AT_STOP_SIGN' for r in records))
        checks['Waited for intersection clearance'] = any((r['state'] == 'WAIT_AT_STOP_SIGN' and (not r['clear']) for r in records))
    if scenario == 'stop_sign_agent':
        checks['No crossing while agent occupies intersection'] = all((r['distance_to_line'] >= 0.0 for r in records if not r['clear']))
        checks['Agent moved through conflict region'] = records[0]['agent_y'] < 0.0 and records[-1]['agent_y'] > 5.0
    print('\nChecks:')
    for name, passed in checks.items():
        print(f"{('PASS' if passed else 'FAIL')}: {name}")
    if not all(checks.values()):
        raise RuntimeError(f'{scenario} scenario failed')
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=['red_to_green', 'stop_sign', 'stop_sign_agent', 'lane_follow', 'lane_change', 'lane_change_blocked', 'lane_change_wait_then_go'], default='red_to_green')
    args = parser.parse_args()
    results = run_simulation(args.scenario)
    save_results(results, args.scenario)
    check_results(results, args.scenario)
