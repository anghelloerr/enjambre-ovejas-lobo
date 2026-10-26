"""Analiza el CSV de metrics_node (experimento ovejas-lobo en Gazebo).

Genera figuras de trayectorias y metricas, una animacion GIF reconstruida a partir
de las poses de ground truth registradas en Gazebo y un resumen en CSV.

  python3 analyze_gazebo_swarm.py data/gazebo_swarm_herd_gt.csv --tag gazebo_herd
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIG = os.path.join(ROOT, 'docs', 'figures')
sys.path.insert(0, HERE)
import params as PR  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--tag', default='gazebo_herd')
    ap.add_argument('--gif', action='store_true')
    args = ap.parse_args()
    rows = list(csv.DictReader(open(args.csv)))
    keys = rows[0].keys()
    d = {k: np.array([float(r[k]) for r in rows]) for k in keys}
    robots = sorted({k.rsplit('_', 1)[0] for k in keys if k.endswith('_x')},
                    key=lambda s: (s == 'wolf', s))
    sheep = [r for r in robots if r != 'wolf']
    t = d['t']
    goal, goal_r = PR.SWARM['goal'], PR.SWARM['goal_r']
    in_goal = np.where(d['frac_goal'] >= 1.0)[0]
    t_goal = float(t[in_goal[0]]) if len(in_goal) else float('nan')
    summary = dict(duration_s=float(t[-1]), t_goal_s=t_goal, collisions=int(d['collisions'][-1]),
                   mean_cohesion_m=float(np.mean(d['cohesion'])), min_dist_m=float(np.min(d['min_dist'])),
                   mean_polarization=float(np.mean(d['polarization'])),
                   final_odom_err_m=float(d['odom_err_mean'][-1]),
                   max_odom_err_m=float(np.nanmax(d['odom_err_mean'])))
    # Contencion: fraccion media de ovejas en el corral durante los 60 s tras llegar
    hold = (t > t_goal) & (t <= t_goal + 60.0)
    summary['hold_frac'] = float(np.mean(d['frac_goal'][hold])) if hold.any() else float('nan')
    summary['hold_s'] = float(t[hold][-1] - t_goal) if hold.any() else 0.0
    # Velocidad media de giro de las ovejas: detecta oscilaciones del rumbo
    turn = [np.sum(np.abs(np.diff(np.unwrap(d[f'{r}_th'])))) for r in sheep]
    summary['turn_rate_dps'] = float(np.degrees(np.mean(turn)) / t[-1])
    for k, v in summary.items():
        print(f'{k:20s} {v:.3f}' if isinstance(v, float) else f'{k:20s} {v}')
    with open(os.path.splitext(args.csv)[0] + '_summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=summary.keys())
        w.writeheader()
        w.writerow(summary)

    h = PR.SWARM['arena'] / 2
    fig, ax = plt.subplots(figsize=(5.6, 5.6))
    ax.add_patch(plt.Rectangle((-h, -h), 2 * h, 2 * h, fill=False, lw=1.5))
    ax.add_patch(plt.Circle(goal, goal_r, color='#2ca02c', alpha=0.15, label='Corral'))
    for r in sheep:
        ax.plot(d[f'{r}_x'], d[f'{r}_y'], color='#1f77b4', lw=0.9)
        ax.plot(d[f'{r}_x'][-1], d[f'{r}_y'][-1], 'o', color='#1f77b4', ms=7)
        ax.plot(d[f'{r}_x'][0], d[f'{r}_y'][0], 'o', mfc='none', mec='#1f77b4', ms=7)
    ax.plot(d['wolf_x'], d['wolf_y'], color='#d62728', lw=1.4, label='Lobo')
    ax.plot(d['wolf_x'][-1], d['wolf_y'][-1], 's', color='#d62728', ms=8)
    ax.set_xlim(-h - 0.1, h + 0.1)
    ax.set_ylim(-h - 0.1, h + 0.1)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title('Gazebo Fortress: trayectorias (ground truth)')
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f'{args.tag}_trayectorias.png'), dpi=150)
    plt.close(fig)

    fig, axs = plt.subplots(4, 1, figsize=(7, 7.6), sharex=True)
    axs[0].plot(t, d['cohesion'])
    axs[0].set_ylabel('cohesion [m]')
    axs[1].plot(t, d['min_dist'], color='#ff7f0e')
    axs[1].axhline(0.20, color='k', ls='--', lw=0.8)
    axs[1].set_ylabel('d min [m]')
    axs[2].plot(t, d['frac_goal'], color='#2ca02c')
    axs[2].set_ylabel('en corral')
    axs[3].plot(t, d['odom_err_mean'], color='#9467bd')
    axs[3].set_ylabel('err. odom EKF [m]')
    axs[3].set_xlabel('t [s] (tiempo simulado)')
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f'{args.tag}_metricas.png'), dpi=150)
    plt.close(fig)

    if args.gif:
        idx = np.arange(0, len(t), 10)
        fig, ax = plt.subplots(figsize=(4.2, 4.2))

        def draw(k):
            i = idx[k]
            ax.clear()
            ax.add_patch(plt.Rectangle((-h, -h), 2 * h, 2 * h, fill=False, lw=1.2))
            ax.add_patch(plt.Circle(goal, goal_r, color='#2ca02c', alpha=0.15))
            for r in robots:
                x, y, th = d[f'{r}_x'][i], d[f'{r}_y'][i], d[f'{r}_th'][i]
                c = '#d62728' if r == 'wolf' else '#1f77b4'
                ax.add_patch(plt.Circle((x, y), PR.R_SAFE, color=c, alpha=0.85))
                ax.plot([x, x + 0.16 * math.cos(th)], [y, y + 0.16 * math.sin(th)], 'k', lw=1.2)
            ax.set_xlim(-h, h)
            ax.set_ylim(-h, h)
            ax.set_aspect('equal')
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_title(f'Gazebo (registro)  t = {t[i]:5.1f} s', fontsize=9)

        anim = animation.FuncAnimation(fig, draw, frames=len(idx), interval=66)
        anim.save(os.path.join(FIG, f'{args.tag}.gif'), writer=animation.PillowWriter(fps=15))
        plt.close(fig)


if __name__ == '__main__':
    main()
