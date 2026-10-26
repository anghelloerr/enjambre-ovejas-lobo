"""Analiza el CSV de odom_test_node (prueba del cuadrado en Gazebo).

Calcula el error final y el RMS de odom_raw (ruedas) y odom (EKF) frente al
ground truth de Gazebo, el giro real logrado por cada orden de 90 grados
(factor chi efectivo) y dibuja las trayectorias.

  python3 analyze_gazebo_odom.py data/gazebo_odom_test.csv [--tag gazebo]
"""
import argparse
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, 'docs', 'figures')


def load(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--tag', default='gazebo')
    ap.add_argument('--w-cmd', type=float, default=1.0)
    args = ap.parse_args()
    d = load(args.csv)
    th = np.unwrap(d['gt_th'])
    total_turn = th[-1] - th[0]
    n_turns = round(4 * max(1, round((d['t'][-1]) / 31.0)))
    raw_turn = np.unwrap(d['raw_th'])[-1] - d['raw_th'][0]
    chi_eff = (2 * 0.115 / 0.135) * raw_turn / total_turn if abs(total_turn) > 1e-3 else float('nan')
    out = {}
    for k, lab in (('raw', 'odom_raw (ruedas)'), ('ekf', 'odom (EKF ruedas+IMU)')):
        e = np.hypot(d[f'{k}_x'] - d['gt_x'], d[f'{k}_y'] - d['gt_y'])
        eth = np.degrees(np.abs(np.arctan2(np.sin(d[f'{k}_th'] - d['gt_th']), np.cos(d[f'{k}_th'] - d['gt_th']))))
        out[k] = (e[-1], math.sqrt(np.mean(e ** 2)), eth[-1])
        print(f'{lab:24s}: error final {e[-1]:.3f} m | RMS {out[k][1]:.3f} m | yaw final {eth[-1]:.1f} grados')
    path_len = np.sum(np.hypot(np.diff(d['gt_x']), np.diff(d['gt_y'])))
    print(f'Recorrido real {path_len:.2f} m | giro real {math.degrees(total_turn):.0f} grados '
          f'(ruedas creen {math.degrees(raw_turn):.0f}) -> chi efectivo = {chi_eff:.2f}')

    fig, ax = plt.subplots(figsize=(5.8, 5.6))
    ax.plot(d['gt_x'], d['gt_y'], 'k-', lw=2.2, label='Ground truth (Gazebo)')
    ax.plot(d['raw_x'], d['raw_y'], '--', color='#d62728', label='odom_raw (ruedas)')
    ax.plot(d['ekf_x'], d['ekf_y'], '-', color='#1f77b4', label='odom (EKF + IMU)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Gazebo Fortress: cuadrado de 1 m en lazo abierto')
    ax.legend(fontsize=8, loc='best')
    fig.tight_layout()
    os.makedirs(FIG, exist_ok=True)
    fig.savefig(os.path.join(FIG, f'{args.tag}_odom_cuadrado.png'), dpi=150)
    with open(os.path.splitext(args.csv)[0] + '_summary.csv', 'w') as f:
        f.write('source,final_err_m,rms_err_m,final_yaw_err_deg,path_m,turn_true_deg,turn_wheels_deg,chi_eff\n')
        for k in out:
            f.write(f'{k},{out[k][0]:.4f},{out[k][1]:.4f},{out[k][2]:.2f},{path_len:.3f},'
                    f'{math.degrees(total_turn):.1f},{math.degrees(raw_turn):.1f},{chi_eff:.3f}\n')


if __name__ == '__main__':
    main()
