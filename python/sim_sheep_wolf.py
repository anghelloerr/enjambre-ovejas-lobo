"""Etapas 5 y 6: simulador de referencia del algoritmo ovejas-lobo.

Cada agente es un MicroRosRobot (cinematica del firmware, lazo de motor de
primer orden, deslizamiento skid-steer y ruido) controlado con la MISMA ley de
control que usan los nodos ROS 2 (swarm_sheep_wolf/behaviors.py).

Escenarios
  flock : solo ovejas (comprueba cohesion y ausencia de colisiones)
  herd  : un lobo pastor lleva el rebano al corral (Strömbom et al. 2014)
  hunt  : un lobo cazador persigue a la oveja mas cercana

Percepcion
  truth : cada robot conoce las posiciones reales (como en Gazebo con ground truth)
  odom  : cada robot usa su propia odometria EKF (como en el robot real sin MoCap)

Uso
  python3 sim_sheep_wolf.py --scenario herd --seeds 20
  python3 sim_sheep_wolf.py --all            # tabla completa + figuras + GIF
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
sys.path.insert(0, os.path.join(ROOT, 'ros2_ws', 'src', 'swarm_sheep_wolf'))

import params as PR                                         # noqa: E402
from robot_model import MicroRosRobot, OdometryEKF          # noqa: E402
from swarm_sheep_wolf import behaviors as BH                # noqa: E402

FIG = os.path.join(ROOT, 'docs', 'figures')
DATA = os.path.join(ROOT, 'data')
DT = PR.DT_SIM
SUB = 5                       # sub-pasos de dinamica por paso de control (100 Hz)
HOLD = 60.0                   # s de vigilancia tras llegar al corral (contencion)


def spawn(n, rng, P, scenario):
    """Ovejas en un grupo disperso al otro lado del corral; lobo detras."""
    c = np.array([-0.6, -0.6])
    pts = []
    while len(pts) < n:
        q = c + rng.uniform(-0.7, 0.7, size=2)
        if all(np.linalg.norm(q - p) > 0.35 for p in pts):
            pts.append(q)
    wolf = np.array([-1.6, -1.6]) if scenario != 'hunt' else np.array([1.4, -1.4])
    return np.array(pts), wolf


def run(scenario='herd', seed=0, n_sheep=5, T=220.0, perception='truth', record=False):
    P = BH.SwarmParams.from_dict(PR.SWARM)
    rng = np.random.default_rng(seed)
    sheep0, wolf0 = spawn(n_sheep, rng, P, scenario)
    chi = 1.55 * (1 + 0.04 * rng.standard_normal(n_sheep + 1))
    robots = [MicroRosRobot(x, y, rng.uniform(-math.pi, math.pi), chi_true=chi[i],
                            rng=np.random.default_rng(seed * 100 + i), delay=PR.CMD_DELAY)
              for i, (x, y) in enumerate(sheep0)]
    has_wolf = scenario in ('herd', 'hunt')
    if has_wolf:
        robots.append(MicroRosRobot(*wolf0, math.pi / 4, chi_true=chi[-1],
                                    rng=np.random.default_rng(seed * 100 + 99), delay=PR.CMD_DELAY))
    ekfs = [OdometryEKF(r.x, r.y, r.th) for r in robots]

    def believed():
        if perception == 'truth':
            return np.array([[r.x, r.y, r.th] for r in robots])
        return np.array([e.p for e in ekfs])

    vel = np.zeros((len(robots), 2))
    log = []
    frames = []
    t_goal = None
    t_capture = None
    collisions = 0
    in_contact = set()
    turn = 0.0                # suma de |delta theta| de las ovejas (oscilacion del rumbo)
    th_prev = np.array([r.th for r in robots[:n_sheep]])
    steps = int(T / DT)
    for k in range(steps):
        t = k * DT
        B = believed()
        pos_s = B[:n_sheep, :2]
        wolf_b = B[n_sheep, :2] if has_wolf else None
        cmds = []
        for i in range(n_sheep):
            u, _ = BH.sheep_desired_velocity(i, pos_s, vel[:n_sheep], wolf_b, P)
            cmds.append(BH.unicycle_command(u, B[i, 2], P))
        sub = '-'
        if has_wolf:
            mode = 'hunt' if scenario == 'hunt' else 'herd'
            u, sub = BH.wolf_desired_velocity(wolf_b, pos_s, P, mode=mode)
            cmds.append(BH.unicycle_command(u, B[n_sheep, 2], P))

        for _ in range(SUB):
            for i, r in enumerate(robots):
                d, g = r.step(*cmds[i], DT / SUB)
                ekfs[i].update(d, DT / SUB, g)
        true = np.array([[r.x, r.y, r.th] for r in robots])
        vel = np.array([[r.v_true * math.cos(r.th), r.v_true * math.sin(r.th)] for r in robots])

        ps = true[:n_sheep, :2]
        dth = true[:n_sheep, 2] - th_prev
        turn += float(np.sum(np.abs(np.arctan2(np.sin(dth), np.cos(dth)))))
        th_prev = true[:n_sheep, 2].copy()
        # Colisiones: eventos de contacto entre cualquier par de robots
        allp = true[:, :2]
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                touching = np.linalg.norm(allp[i] - allp[j]) < 2 * PR.R_SAFE * 0.9
                if touching and (i, j) not in in_contact:
                    collisions += 1
                    in_contact.add((i, j))
                elif not touching:
                    in_contact.discard((i, j))
        frac = BH.fraction_in_goal(ps, P)
        if scenario == 'herd' and t_goal is None and frac == 1.0:
            t_goal = t
        d_wolf = np.min(np.linalg.norm(ps - true[n_sheep, :2], axis=1)) if has_wolf else np.nan
        if scenario == 'hunt' and t_capture is None and d_wolf < 2 * PR.R_SAFE:
            t_capture = t
        odo_err = float(np.mean(np.linalg.norm(true[:, :2] - np.array([e.p[:2] for e in ekfs]), axis=1)))
        log.append(dict(t=t, cohesion=BH.cohesion(ps), polarization=BH.polarization(true[:n_sheep, 2]),
                        min_dist=BH.min_pair_distance(allp), frac_goal=frac, wolf_dist=d_wolf,
                        odo_err=odo_err, wolf_mode=sub))
        if record and k % 2 == 0:
            frames.append(true.copy())
        if scenario == 'herd' and t_goal is not None and t > t_goal + HOLD:
            break
        if scenario == 'hunt' and t_capture is not None:
            break

    res = dict(scenario=scenario, seed=seed, n_sheep=n_sheep, perception=perception,
               t_goal=t_goal, t_capture=t_capture, collisions=collisions,
               mean_cohesion=float(np.mean([r['cohesion'] for r in log])),
               final_cohesion=log[-1]['cohesion'],
               min_dist=float(np.min([r['min_dist'] for r in log])),
               mean_polarization=float(np.mean([r['polarization'] for r in log])),
               final_odo_err=log[-1]['odo_err'], duration=log[-1]['t'],
               turn_rate_dps=math.degrees(turn / n_sheep / log[-1]['t']),
               hold_frac=(float(np.mean([r['frac_goal'] for r in log if r['t'] > t_goal]))
                          if t_goal is not None else float('nan')))
    return res, log, frames


def plot_run(log, frames, scenario, n_sheep, tag):
    P = BH.SwarmParams.from_dict(PR.SWARM)
    F = np.array(frames)
    fig, ax = plt.subplots(figsize=(5.6, 5.6))
    h = P.arena / 2
    ax.add_patch(plt.Rectangle((-h, -h), P.arena, P.arena, fill=False, lw=1.5))
    if scenario == 'herd':
        ax.add_patch(plt.Circle(P.goal, P.goal_r, color='#2ca02c', alpha=0.15, label='Corral'))
    for i in range(n_sheep):
        ax.plot(F[:, i, 0], F[:, i, 1], color='#1f77b4', lw=0.9, alpha=0.8)
        ax.plot(*F[-1, i, :2], 'o', color='#1f77b4', ms=7)
    ax.plot(F[0, :n_sheep, 0], F[0, :n_sheep, 1], 'o', mfc='none', mec='#1f77b4', ms=7, label='Ovejas (inicio)')
    if F.shape[1] > n_sheep:
        ax.plot(F[:, -1, 0], F[:, -1, 1], color='#d62728', lw=1.4, label='Lobo')
        ax.plot(*F[-1, -1, :2], 's', color='#d62728', ms=8)
    ax.set_xlim(-h - 0.1, h + 0.1)
    ax.set_ylim(-h - 0.1, h + 0.1)
    ax.set_aspect('equal')
    ax.set_title(f'Escenario {scenario}: trayectorias')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f'swarm_{tag}_trayectorias.png'), dpi=150)
    plt.close(fig)

    t = [r['t'] for r in log]
    fig, axs = plt.subplots(3, 1, figsize=(7, 6.4), sharex=True)
    axs[0].plot(t, [r['cohesion'] for r in log], color='#1f77b4')
    axs[0].set_ylabel('cohesion [m]')
    axs[1].plot(t, [r['min_dist'] for r in log], color='#ff7f0e')
    axs[1].axhline(2 * PR.R_SAFE * 0.9, color='k', ls='--', lw=0.8, label='umbral de colision')
    axs[1].set_ylabel('dist. minima [m]')
    axs[1].legend(fontsize=8)
    if scenario == 'herd':
        axs[2].plot(t, [r['frac_goal'] for r in log], color='#2ca02c')
        axs[2].set_ylabel('fraccion en corral')
    elif scenario == 'hunt':
        axs[2].plot(t, [r['wolf_dist'] for r in log], color='#d62728')
        axs[2].set_ylabel('dist. lobo-oveja [m]')
    else:
        axs[2].plot(t, [r['polarization'] for r in log], color='#9467bd')
        axs[2].set_ylabel('polarizacion')
    axs[2].set_xlabel('t [s]')
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, f'swarm_{tag}_metricas.png'), dpi=150)
    plt.close(fig)


def make_gif(frames, scenario, n_sheep, path):
    P = BH.SwarmParams.from_dict(PR.SWARM)
    F = np.array(frames)[::3]
    fig, ax = plt.subplots(figsize=(4.2, 4.2))
    h = P.arena / 2

    def draw(k):
        ax.clear()
        ax.add_patch(plt.Rectangle((-h, -h), P.arena, P.arena, fill=False, lw=1.2))
        if scenario == 'herd':
            ax.add_patch(plt.Circle(P.goal, P.goal_r, color='#2ca02c', alpha=0.15))
        for i in range(F.shape[1]):
            x, y, th = F[k, i]
            c = '#1f77b4' if i < n_sheep else '#d62728'
            ax.add_patch(plt.Circle((x, y), PR.R_SAFE, color=c, alpha=0.85))
            ax.plot([x, x + 0.16 * math.cos(th)], [y, y + 0.16 * math.sin(th)], color='k', lw=1.2)
        ax.set_xlim(-h, h)
        ax.set_ylim(-h, h)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(f'{scenario}  t = {k * 3 * 2 * DT:5.1f} s', fontsize=9)

    anim = animation.FuncAnimation(fig, draw, frames=len(F), interval=60)
    anim.save(path, writer=animation.PillowWriter(fps=15))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='herd', choices=['flock', 'herd', 'hunt'])
    ap.add_argument('--perception', default='truth', choices=['truth', 'odom'])
    ap.add_argument('--seeds', type=int, default=10)
    ap.add_argument('--n', type=int, default=PR.SWARM['n_sheep'])
    ap.add_argument('--all', action='store_true')
    args = ap.parse_args()
    os.makedirs(FIG, exist_ok=True)
    os.makedirs(DATA, exist_ok=True)

    combos = ([(s, p) for s in ['flock', 'herd', 'hunt'] for p in ['truth', 'odom']]
              if args.all else [(args.scenario, args.perception)])
    results = []
    for scen, perc in combos:
        T = 90.0 if scen == 'flock' else 180.0
        for seed in range(args.seeds):
            res, log, frames = run(scen, seed, args.n, T, perc, record=(seed == 0))
            results.append(res)
            if seed == 0 and perc == 'truth':
                plot_run(log, frames, scen, args.n, scen)
                if args.all:
                    make_gif(frames, scen, args.n, os.path.join(FIG, f'swarm_{scen}.gif'))
                with open(os.path.join(DATA, f'log_{scen}_seed0.csv'), 'w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=log[0].keys())
                    w.writeheader()
                    w.writerows(log)

    with open(os.path.join(DATA, 'swarm_results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)

    print(f'{"escenario":>9} {"percep.":>7} | {"exito":>7} | {"t medio [s]":>11} | {"colis/ensayo":>12} | '
          f'{"cohesion [m]":>12} | {"d_min [m]":>9} | {"err odo [m]":>11} | {"contencion":>10} | {"giro [o/s]":>10}')
    summary = []
    for scen, perc in combos:
        rs = [r for r in results if r['scenario'] == scen and r['perception'] == perc]
        key = 't_goal' if scen == 'herd' else ('t_capture' if scen == 'hunt' else None)
        ok = [r[key] for r in rs if key and r[key] is not None]
        succ = len(ok) / len(rs) if key else float('nan')
        row = dict(scenario=scen, perception=perc, runs=len(rs), success=succ,
                   t_mean=float(np.mean(ok)) if ok else float('nan'),
                   t_sd=float(np.std(ok)) if ok else float('nan'),
                   collisions_per_run=float(np.mean([r['collisions'] for r in rs])),
                   cohesion=float(np.mean([r['mean_cohesion'] for r in rs])),
                   min_dist=float(np.min([r['min_dist'] for r in rs])),
                   odo_err=float(np.mean([r['final_odo_err'] for r in rs])),
                   turn_rate_dps=float(np.mean([r['turn_rate_dps'] for r in rs])),
                   hold_frac=float(np.nanmean([r['hold_frac'] for r in rs])) if scen == 'herd' else float('nan'))
        summary.append(row)
        succ_s = f'{100*succ:5.0f} %' if key else '   n/a'
        print(f'{scen:>9} {perc:>7} | {succ_s:>7} | {row["t_mean"]:6.1f}±{row["t_sd"]:4.1f} | '
              f'{row["collisions_per_run"]:12.2f} | {row["cohesion"]:12.3f} | {row["min_dist"]:9.3f} | '
              f'{row["odo_err"]:11.3f} | {row["hold_frac"]:10.3f} | {row["turn_rate_dps"]:10.1f}')
    with open(os.path.join(DATA, 'swarm_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=summary[0].keys())
        w.writeheader()
        w.writerows(summary)


if __name__ == '__main__':
    main()
