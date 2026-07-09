"""Run the hockey mission: T1 nav -> T2 pick up stick -> T3 nav -> T4 shoot.

  python main.py                 # live animation
  python main.py --headless      # fast run, prints result, saves report plots
  python main.py --speed 3       # animation at 3x real time

Physics runs at 50 Hz; the controller runs at 10 Hz like the real ROS node.
Report plots are saved to sim/plots/ at the end of either mode.
"""
import argparse
import os

import numpy as np

from simulator import Sim
from controller import HockeyMission, point_p
import report_plots

DT = 0.02                 # physics step (50 Hz)
CONTROL_EVERY = 5         # -> 10 Hz control, like CONTROL_LOOP_FREQUENCIES
PLOTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plots')


def new_log():
    return {'t': [], 'robot_xy': [], 'p': [], 'p_des': [], 'err': [],
            'v': [], 'w': [], 'puck': [], 'phase': []}


def record(log, sim, mission):
    pose = sim.get_pose('dji_robot_3')
    p_ctrl = point_p(pose, mission.ctrl_l)   # point the controller regulates
    log['t'].append(sim.t)
    log['robot_xy'].append(sim.robot[:2].copy())
    log['p'].append(point_p(pose, mission.l))
    log['p_des'].append(mission.p_des.copy())
    log['err'].append(float(np.linalg.norm(mission.p_des - p_ctrl)))
    log['v'].append(sim.cmd_v)
    log['w'].append(sim.cmd_w)
    log['puck'].append(sim.puck.copy())
    log['phase'].append(mission.phase)


def step_world(sim, mission, log, i):
    if i % CONTROL_EVERY == 0:
        mission.update()
    sim.step(DT)
    record(log, sim, mission)


def run_headless(timeout):
    sim, mission, log = Sim(), None, new_log()
    mission = HockeyMission(sim)
    last_phase, i = mission.phase, 0
    print(f'[{sim.t:6.2f}s] phase: {mission.phase}')
    while sim.t < timeout:
        step_world(sim, mission, log, i)
        i += 1
        if mission.phase != last_phase:
            print(f'[{sim.t:6.2f}s] phase: {mission.phase}')
            last_phase = mission.phase
        if mission.phase == 'DONE':
            break
    result = 'GOAL!' if sim.goal_scored else 'no goal (missed or timed out)'
    print(f'\nResult: {result}   (t = {sim.t:.2f} s)')
    paths = report_plots.save_all(log, sim, PLOTS_DIR)
    print('Report plots saved to:')
    for p in paths:
        print(' ', p)
    return 0 if sim.goal_scored else 1


def run_animated(speed, timeout):
    import matplotlib.pyplot as plt
    from matplotlib import animation, patches

    C = report_plots  # palette constants

    sim = Sim()
    mission = HockeyMission(sim)
    log = new_log()
    steps = {'i': 0}

    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    fig.patch.set_facecolor(C.SURFACE)
    fig.canvas.manager.set_window_title('RoboMaster hockey sim')
    report_plots._style(ax)
    f = sim.FIELD
    ax.set_xlim(-f - 0.2, f + 0.2)
    ax.set_ylim(-f - 0.2, f + 0.2)
    ax.set_aspect('equal')
    ax.plot([-f, f, f, -f, -f], [-f, -f, f, f, -f], color=C.MUTED, linewidth=2)

    # static: goal
    p1, p2 = sim.goal_posts()
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=C.GREEN, linewidth=5,
            solid_capstyle='butt')
    ax.annotate('goal', (sim.goal[0], sim.goal[1]), textcoords='offset points',
                xytext=(-12, -12), ha='right', fontsize=10, color=C.INK)

    # dynamic artists
    chassis = patches.Polygon(np.zeros((4, 2)), closed=True, facecolor=C.BLUE,
                              edgecolor='none', zorder=5)
    ax.add_patch(chassis)
    stick_line, = ax.plot([], [], color=C.RED, linewidth=3,
                          solid_capstyle='round', zorder=6)
    puck_dot, = ax.plot([], [], marker='o', color=C.INK, markersize=9, zorder=6)
    p_dot_mark, = ax.plot([], [], marker='+', color=C.VIOLET, markersize=10,
                          markeredgewidth=2, zorder=7)
    trail, = ax.plot([], [], color=C.BLUE, linewidth=1.2, alpha=0.6, zorder=3)
    puck_trail, = ax.plot([], [], color=C.INK, linewidth=1.2, linestyle=':',
                          alpha=0.7, zorder=3)
    status = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top',
                     fontsize=10, color=C.INK, family='monospace')

    W, H = 0.32, 0.24  # chassis footprint

    def chassis_corners():
        x, y, th = sim.robot
        R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        base = np.array([[W / 2, H / 2], [W / 2, -H / 2],
                         [-W / 2, -H / 2], [-W / 2, H / 2]])
        return base @ R.T + [x, y]

    def draw_stick():
        if sim.stick_grasped:
            a = sim.robot[:2] + 0.12 * sim.heading()
            b = sim.stick_tip()
        else:
            sx, sy, syaw = sim.stick
            u = np.array([np.cos(syaw), np.sin(syaw)])
            a, b = np.array([sx, sy]), np.array([sx, sy]) + 0.35 * u
        stick_line.set_data([a[0], b[0]], [a[1], b[1]])

    steps_per_frame = max(1, int(round(speed * 0.04 / DT)))

    def update(_):
        for _ in range(steps_per_frame):
            if mission.phase != 'DONE' and sim.t < timeout:
                step_world(sim, mission, log, steps['i'])
                steps['i'] += 1
        chassis.set_xy(chassis_corners())
        draw_stick()
        puck_dot.set_data([sim.puck[0]], [sim.puck[1]])
        p = point_p(sim.get_pose('dji_robot_3'), mission.l)
        p_dot_mark.set_data([p[0]], [p[1]])
        xy = np.asarray(log['robot_xy'])
        pk = np.asarray(log['puck'])
        if len(xy):
            trail.set_data(xy[:, 0], xy[:, 1])
            puck_trail.set_data(pk[:, 0], pk[:, 1])
        outcome = 'GOAL!' if sim.goal_scored else ''
        status.set_text(f't = {sim.t:5.1f} s   phase: {mission.phase}   {outcome}')
        return chassis, stick_line, puck_dot, p_dot_mark, trail, puck_trail, status

    anim = animation.FuncAnimation(fig, update, interval=40,
                                   cache_frame_data=False, blit=False)
    plt.show()

    if log['t']:
        paths = report_plots.save_all(log, sim, PLOTS_DIR)
        print('Report plots saved to:')
        for p in paths:
            print(' ', p)
    return 0 if sim.goal_scored else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--headless', action='store_true',
                    help='run without animation, save plots, exit')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='animation speed multiplier')
    ap.add_argument('--timeout', type=float, default=90.0,
                    help='mission timeout (sim seconds)')
    args = ap.parse_args()
    if args.headless:
        raise SystemExit(run_headless(args.timeout))
    raise SystemExit(run_animated(args.speed, args.timeout))


if __name__ == '__main__':
    main()
