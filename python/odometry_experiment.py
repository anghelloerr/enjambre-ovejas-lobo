"""Etapa 3: estudio de odometria del MicroROS-Pi5 (simulacion de referencia).

Reproduce las pruebas de calibracion de Yahboom (1 m recto y giro de 360 grados)
y una trayectoria cuadrada de 1 m x 1 m (3 vueltas). Compara:
  E  : solo encoders con la cinematica del firmware
  Ec : solo encoders tras calibrar las escalas lineal y angular
  EG : encoders + giroscopo
  EKF: fusion encoders + giroscopo con estimacion de sesgo

Salida: data/odometry_results.csv, data/odometry_summary.csv y figuras en docs/figures/.
Uso:  python3 odometry_experiment.py [--seeds 10]
"""
import argparse
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import params as PR
from robot_model import (MicroRosRobot, OdometryEncoders, OdometryEncGyro,
                         OdometryEKF, wrap)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIG = os.path.join(ROOT, 'docs', 'figures')
DATA = os.path.join(ROOT, 'data')
DT = PR.TS_PID * 2      # 50 Hz, la frecuencia de odom_raw


def run_profile(profile, seed, chi_true, scale_lin=1.0, scale_ang=1.0, log=False):
    rng = np.random.default_rng(seed)
    robot = MicroRosRobot(chi_true=chi_true, rng=rng)
    est = {
        'E': OdometryEncoders(),
        'Ec': OdometryEncoders(scale_lin=scale_lin, scale_ang=scale_ang),
        'EG': OdometryEncGyro(scale_lin=scale_lin),
        'EKF': OdometryEKF(),
    }
    traj = {k: [] for k in ['true', *est]}
    for v, w, T in profile:
        for _ in range(int(round(T / DT))):
            d, gyro = robot.step(v, w, DT)
            est['E'].update(d, DT)
            est['Ec'].update(d, DT)
            est['EG'].update(d, DT, gyro)
            est['EKF'].update(d * scale_lin, DT, gyro)
            if log:
                traj['true'].append(robot.pose.copy())
                for k, e in est.items():
                    traj[k].append(np.array(e.p, dtype=float).copy())
    final = {k: np.array(e.p, dtype=float) for k, e in est.items()}
    return robot, final, ({k: np.array(t) for k, t in traj.items()} if log else None)


def square_profile(side=1.0, v=0.2, w=1.0, laps=3):
    prof = []
    for _ in range(4 * laps):
        prof.append((v, 0.0, side / v))
        prof.append((0.0, 0.0, 0.3))
        prof.append((0.0, w, (math.pi / 2) / w))
        prof.append((0.0, 0.0, 0.3))
    return prof


def calibrate(chi_true, seed=100):
    """Replica calibrate_linear / calibrate_angular de Yahboom.

    El robot avanza hasta que la ODOMETRIA marca 1 m (o 360 grados); se mide lo
    recorrido de verdad (con regla / transportador) y se ajusta la escala.
    """
    rng = np.random.default_rng(seed)
    # Lineal
    r = MicroRosRobot(chi_true=chi_true, rng=rng)
    o = OdometryEncoders()
    while o.p[0] < 1.0:
        d, _ = r.step(0.2, 0.0, DT)
        o.update(d, DT)
    scale_lin = math.hypot(r.x, r.y) / math.hypot(o.p[0], o.p[1])
    # Angular
    r = MicroRosRobot(chi_true=chi_true, rng=rng)
    o = OdometryEncoders()
    acc_odo, acc_true, prev_odo, prev_true = 0.0, 0.0, 0.0, 0.0
    while acc_odo < 2 * math.pi:
        d, _ = r.step(0.0, 1.0, DT)
        o.update(d, DT)
        acc_odo += wrap(o.p[2] - prev_odo)
        acc_true += wrap(r.th - prev_true)
        prev_odo, prev_true = o.p[2], r.th
    scale_ang = acc_true / acc_odo
    return scale_lin, scale_ang


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=10)
    ap.add_argument('--chi', type=float, default=1.55,
                    help='factor de trocha efectiva real (skid-steer)')
    ap.add_argument('--chi-sd', type=float, default=0.04,
                    help='variacion relativa de chi entre ensayos (suelo, bateria)')
    args = ap.parse_args()
    os.makedirs(FIG, exist_ok=True)
    os.makedirs(DATA, exist_ok=True)

    s_lin, s_ang = calibrate(args.chi)
    print(f'Calibracion: scale_lin={s_lin:.4f}  scale_ang={s_ang:.4f}  '
          f'(teorico chi_fw/chi = {2*PR.APB/PR.W/args.chi:.4f})')

    prof = square_profile()
    rows = []
    chi_rng = np.random.default_rng(1234)
    for seed in range(args.seeds):
        # La calibracion se hizo con chi nominal; cada ensayo tiene un suelo algo distinto
        chi_run = args.chi * (1.0 + args.chi_sd * chi_rng.standard_normal())
        robot, final, _ = run_profile(prof, seed, chi_run, s_lin, s_ang)
        truth = robot.pose
        for k, p in final.items():
            rows.append(dict(seed=seed, estimator=k, chi=round(chi_run, 4),
                             pos_err=float(np.hypot(*(p[:2] - truth[:2]))),
                             yaw_err_deg=float(abs(math.degrees(wrap(p[2] - truth[2]))))))
    with open(os.path.join(DATA, 'odometry_results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)

    summary = []
    print(f'\nCuadrado 1 m x 3 vueltas (12 m), {args.seeds} semillas, '
          f'chi_real={args.chi} (sd relativa {args.chi_sd})')
    print(f'{"estimador":>9} | {"err pos [m] media±sd":>22} | {"err yaw [deg] media±sd":>24}')
    for k in ['E', 'Ec', 'EG', 'EKF']:
        pe = np.array([r['pos_err'] for r in rows if r['estimator'] == k])
        ye = np.array([r['yaw_err_deg'] for r in rows if r['estimator'] == k])
        summary.append(dict(estimator=k, pos_mean=pe.mean(), pos_sd=pe.std(),
                            yaw_mean=ye.mean(), yaw_sd=ye.std(),
                            pos_pct_of_path=100 * pe.mean() / 12.0))
        print(f'{k:>9} | {pe.mean():9.3f} ± {pe.std():6.3f}      | {ye.mean():10.2f} ± {ye.std():6.2f}')
    with open(os.path.join(DATA, 'odometry_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=summary[0].keys())
        w.writeheader()
        w.writerows(summary)
    with open(os.path.join(DATA, 'odometry_calibration.csv'), 'w', newline='') as f:
        f.write('chi_true,scale_lin,scale_ang\n')
        f.write(f'{args.chi},{s_lin:.6f},{s_ang:.6f}\n')

    # Figura de trayectorias (semilla 0)
    _, _, tr = run_profile(prof, 0, args.chi, s_lin, s_ang, log=True)
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    style = {'true': ('k', '-', 'Real (simulador)'), 'E': ('#d62728', '--', 'Encoders (firmware)'),
             'Ec': ('#ff7f0e', '-.', 'Encoders calibrados'), 'EG': ('#2ca02c', ':', 'Encoders + giroscopo'),
             'EKF': ('#1f77b4', '-', 'EKF')}
    for k, (c, ls, lab) in style.items():
        ax.plot(tr[k][:, 0], tr[k][:, 1], color=c, ls=ls, lw=1.6 if k != 'true' else 2.2, label=lab)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Odometria: cuadrado 1 m, 3 vueltas')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'odometria_trayectorias.png'), dpi=150)

    # Error de posicion en el tiempo
    fig, ax = plt.subplots(figsize=(7, 3.4))
    t = np.arange(len(tr['true'])) * DT
    for k in ['E', 'Ec', 'EG', 'EKF']:
        c, ls, lab = style[k]
        ax.plot(t, np.hypot(*(tr[k][:, :2] - tr['true'][:, :2]).T), color=c, ls=ls, label=lab)
    ax.set_xlabel('t [s]')
    ax.set_ylabel('error de posicion [m]')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, 'odometria_error_tiempo.png'), dpi=150)
    print('\nFiguras guardadas en docs/figures/')


if __name__ == '__main__':
    main()
