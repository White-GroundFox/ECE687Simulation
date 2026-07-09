"""Report-quality figures from a simulation log (for the IEEE report)."""
import os

import matplotlib.pyplot as plt
import numpy as np

# validated categorical palette (colorblind-safe, light surface)
BLUE, AQUA, RED, GREEN, VIOLET = '#2a78d6', '#1baf7a', '#e34948', '#008300', '#4a3aa7'
INK, MUTED, GRID, SURFACE = '#0b0b0b', '#898781', '#e1e0d9', '#fcfcfb'


def _style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=9)


def _phase_lines(ax, log):
    """Vertical markers where the task (T1..T4, DONE) changes, labeled on top."""
    t = np.asarray(log['t'])
    tasks = [p.split('_')[0] for p in log['phase']]
    for i in range(1, len(t)):
        if tasks[i] != tasks[i - 1]:
            ax.axvline(t[i], color=GRID, linewidth=1.0, linestyle='--')
            ax.text(t[i], 1.01, tasks[i], transform=ax.get_xaxis_transform(),
                    ha='left', va='bottom', fontsize=8, color=MUTED)


def plot_trajectory(log, sim, path):
    fig, ax = plt.subplots(figsize=(6.5, 6.5))
    fig.patch.set_facecolor(SURFACE)
    _style(ax)

    f = sim.FIELD
    ax.plot([-f, f, f, -f, -f], [-f, -f, f, f, -f], color=MUTED, linewidth=1.5)

    xy = np.asarray(log['robot_xy'])
    p = np.asarray(log['p'])
    puck = np.asarray(log['puck'])
    ax.plot(xy[:, 0], xy[:, 1], color=BLUE, linewidth=2, label='robot center')
    ax.plot(p[:, 0], p[:, 1], color=VIOLET, linewidth=1.2, label='stick tip p')
    ax.plot(puck[:, 0], puck[:, 1], color=INK, linewidth=2, linestyle=':', label='puck')

    ax.plot(*xy[0], marker='o', color=BLUE, markersize=8)
    ax.annotate('start', xy[0], textcoords='offset points', xytext=(8, -4),
                fontsize=9, color=INK)
    ax.plot(*puck[0], marker='o', color=INK, markersize=7)
    ax.annotate('puck', puck[0], textcoords='offset points', xytext=(8, 0),
                fontsize=9, color=INK)
    ax.plot(sim.stick[0], sim.stick[1], marker='s', color=RED, markersize=8)
    ax.annotate('stick pick-up', (sim.stick[0], sim.stick[1]),
                textcoords='offset points', xytext=(-8, 8), ha='right',
                fontsize=9, color=INK)
    p1, p2 = sim.goal_posts()
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=GREEN, linewidth=4,
            solid_capstyle='butt')
    ax.annotate('goal', (sim.goal[0], sim.goal[1]), textcoords='offset points',
                xytext=(-10, -10), ha='right', fontsize=9, color=INK)

    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=INK)
    ax.set_ylabel('y (m)', color=INK)
    ax.set_title('Trajectories in the Robohub frame', color=INK, fontsize=11)
    ax.legend(loc='upper left', fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_tracking_error(log, path):
    fig, ax = plt.subplots(figsize=(7, 3.2))
    fig.patch.set_facecolor(SURFACE)
    _style(ax)
    ax.plot(log['t'], log['err'], color=BLUE, linewidth=2)
    _phase_lines(ax, log)
    ax.set_xlabel('t (s)', color=INK)
    ax.set_ylabel('|p* - p| (m)', color=INK)
    ax.set_title('Tracking error of the control point p', color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_inputs(log, path):
    """v and w have different units, so they get separate stacked axes."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 4.6), sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        _style(ax)
    ax1.plot(log['t'], log['v'], color=BLUE, linewidth=2)
    ax1.set_ylabel('v (m/s)', color=INK)
    ax1.set_title('Unicycle inputs from Eq. (3)', color=INK, fontsize=11)
    _phase_lines(ax1, log)
    ax2.plot(log['t'], log['w'], color=AQUA, linewidth=2)
    ax2.set_ylabel('w (rad/s)', color=INK)
    ax2.set_xlabel('t (s)', color=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_all(log, sim, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    plot_trajectory(log, sim, os.path.join(out_dir, 'trajectory.png'))
    plot_tracking_error(log, os.path.join(out_dir, 'tracking_error.png'))
    plot_inputs(log, os.path.join(out_dir, 'inputs.png'))
    return [os.path.join(out_dir, n)
            for n in ('trajectory.png', 'tracking_error.png', 'inputs.png')]
