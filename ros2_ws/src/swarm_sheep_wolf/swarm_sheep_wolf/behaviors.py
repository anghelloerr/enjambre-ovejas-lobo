"""Comportamientos ovejas-lobo, independientes de ROS.

Este modulo solo usa numpy. Lo importan los nodos ROS 2 (sheep_node, wolf_node)
y el simulador de referencia de python/, de modo que la misma ley de control
se prueba fuera de Gazebo y dentro de Gazebo.

Convenciones
  pos: (N, 2) posiciones en el marco del mundo [m]
  vel: (N, 2) velocidades en el marco del mundo [m/s]
  Las fuerzas devueltas son vectores deseados de velocidad [m/s].
"""
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class SwarmParams:
    r_sep: float = 0.45
    r_nb: float = 1.00
    r_fear: float = 1.20
    w_sep: float = 3.0
    w_coh: float = 0.6
    w_ali: float = 0.4
    w_flee: float = 2.0
    w_wall: float = 2.0
    v_sheep: float = 0.20
    v_wolf: float = 0.25
    k_heading: float = 2.5
    v_dead: float = 0.02       # [m/s] por debajo de esto el robot se queda quieto
    v_turn_ref: float = 0.05   # [m/s] por debajo de esto omega se reduce en proporcion
    arena: float = 4.0
    wall_margin: float = 0.40
    goal: tuple = (1.2, 1.2)
    goal_r: float = 0.6
    # Pastoreo tipo Strömbom et al. (2014)
    herd_offset: float = 0.45      # distancia detras de la oveja/rebano
    collect_factor: float = 0.45   # f_N = collect_factor * N^(2/3)
    wolf_keepout: float = 0.40     # el lobo no se acerca mas que esto a una oveja
    v_max: float = 1.0
    w_max: float = 5.0
    extra: dict = field(default_factory=dict)

    @classmethod
    def from_dict(cls, d):
        known = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**known)


def _unit(v, eps=1e-9):
    n = np.linalg.norm(v)
    return v / n if n > eps else np.zeros_like(v)


def wall_repulsion(p, P: SwarmParams):
    """Repulsion lineal desde las paredes de una arena cuadrada centrada en 0."""
    half = P.arena / 2.0
    f = np.zeros(2)
    for k in range(2):
        d_low = p[k] + half
        d_high = half - p[k]
        if d_low < P.wall_margin:
            f[k] += (P.wall_margin - d_low) / P.wall_margin
        if d_high < P.wall_margin:
            f[k] -= (P.wall_margin - d_high) / P.wall_margin
    return f


def sheep_desired_velocity(i, pos, vel, wolf_pos, P: SwarmParams):
    """Velocidad deseada de la oveja i (reglas de Reynolds + huida del lobo).

    Devuelve (u, fear) donde fear en [0, 1] indica la intensidad de la amenaza.
    """
    p = pos[i]
    diff = pos - p
    dist = np.linalg.norm(diff, axis=1)
    dist[i] = np.inf

    # Separacion continua: |s_ij| = r_sep/d - 1, nula en d = r_sep, promediada sobre los
    # vecinos cercanos. Continua: un salto en el borde hace oscilar el rumbo. Promediada:
    # con la suma, la rigidez crece con el numero de vecinos y, con ~100 ms de retardo,
    # el rebano apretado en el corral entra en un ciclo limite (docs/07, 7.2).
    close = dist < P.r_sep
    sep = np.zeros(2)
    if np.any(close):
        d = dist[close, None]
        sep = -np.mean(diff[close] / d * (P.r_sep / d - 1.0), axis=0)

    nb = dist < P.r_nb
    coh = np.zeros(2)
    ali = np.zeros(2)
    if np.any(nb):
        coh = pos[nb].mean(axis=0) - p
        ali = vel[nb].mean(axis=0)

    flee = np.zeros(2)
    fear = 0.0
    if wolf_pos is not None:
        dw = p - np.asarray(wolf_pos)
        d = np.linalg.norm(dw)
        if d < P.r_fear:
            fear = 1.0 - d / P.r_fear
            flee = _unit(dw) * fear

    u = (P.w_sep * sep + P.w_coh * coh * (1.0 + fear) + P.w_ali * ali
         + P.w_flee * flee + P.w_wall * wall_repulsion(p, P))

    # Sin amenaza las ovejas pastan: velocidad baja; con amenaza aceleran.
    v_cap = P.v_sheep * (0.35 + 0.65 * fear) if fear > 0 else 0.35 * P.v_sheep
    n = np.linalg.norm(u)
    if n > v_cap:
        u = u / n * v_cap
    return u, fear


def wolf_hunt_target(wolf, pos):
    """Modo caza: persecucion pura de la oveja mas cercana."""
    d = np.linalg.norm(pos - wolf, axis=1)
    j = int(np.argmin(d))
    return pos[j], j, float(d[j])


def wolf_herd_target(wolf, pos, P: SwarmParams):
    """Modo pastoreo (Strömbom et al. 2014): recolectar o conducir hacia el corral.

    Devuelve (punto objetivo, modo) con modo en {'collect', 'drive', 'guard'}.
    """
    goal = np.asarray(P.goal, dtype=float)
    gcm = pos.mean(axis=0)
    n = len(pos)
    f_n = P.collect_factor * n ** (2.0 / 3.0)
    d_gcm = np.linalg.norm(pos - gcm, axis=1)

    # Lado abierto del corral (hacia el centro de la arena): desde ahi se empuja hacia la esquina
    open_dir = _unit(-goal) if np.linalg.norm(goal) > 1e-6 else np.array([-1.0, 0.0])

    if np.all(np.linalg.norm(pos - goal, axis=1) < P.goal_r):
        # Vigilancia: el lobo espera en el lado abierto para que nadie se escape
        return _clamp_to_arena(goal + open_dir * (P.goal_r + P.herd_offset), P), 'guard'

    if np.max(d_gcm) > f_n:
        j = int(np.argmax(d_gcm))
        target = pos[j] + _unit(pos[j] - gcm) * P.herd_offset
        return _clamp_to_arena(target, P), 'collect'

    # Detras del rebano, a herd_offset del borde del grupo: dentro del radio de miedo
    # Si el rebano esta sobre el corral o ya lo ha pasado (lado de la esquina), la
    # direccion gcm - goal llevaria al lobo a la esquina y empujaria a las ovejas fuera:
    # en ese caso se conduce siempre desde el lado abierto.
    away = gcm - goal
    open_side = np.dot(away, open_dir) > 0.5 * P.goal_r
    drive_dir = _unit(away) if open_side else open_dir
    target = gcm + drive_dir * (np.max(d_gcm) + P.herd_offset)
    return _clamp_to_arena(target, P), 'drive'


def _clamp_to_arena(p, P: SwarmParams, margin=0.25):
    half = P.arena / 2.0 - margin
    return np.clip(p, -half, half)


def wolf_desired_velocity(wolf, pos, P: SwarmParams, mode='herd'):
    """Velocidad deseada del lobo. mode in {'herd', 'hunt'}."""
    if mode == 'hunt':
        target, _, _ = wolf_hunt_target(wolf, pos)
        u = _unit(target - wolf) * P.v_wolf
        return u, 'hunt'

    target, sub = wolf_herd_target(wolf, pos, P)
    u = target - wolf
    if sub == 'guard' and np.linalg.norm(u) < 0.05:
        return np.zeros(2), sub
    # Evitar atravesar el rebano: repulsion desde ovejas demasiado cercanas
    d = np.linalg.norm(pos - wolf, axis=1)
    near = d < P.wolf_keepout
    if np.any(near):
        u = u + np.sum((wolf - pos[near]) / d[near, None] ** 2, axis=0) * 0.05
    u = u + P.w_wall * wall_repulsion(wolf, P) * 0.3
    n = np.linalg.norm(u)
    if n > P.v_wolf:
        u = u / n * P.v_wolf
    return u, sub


def unicycle_command(u, theta, P: SwarmParams, v_limit=None):
    """Convierte una velocidad deseada 2D en (v, w) para un robot no holonomo.

    e = angulo entre el rumbo actual y el deseado.
    v = |u| cos(e) si cos(e) > 0, w = k_heading * e * min(1, |u| / v_turn_ref).
    Si |u| < v_dead el robot se detiene. Cerca de un equilibrio la direccion de u no
    esta definida y cambia de lado con el ruido: sin esta zona muerta el robot gira
    en el sitio de un lado a otro (castaneteo del rumbo, docs/07 seccion 7.5).
    En ajustes lentos (|u| < v_turn_ref) con el objetivo detras (|e| > 90 grados) el robot
    retrocede en lugar de dar media vuelta: asi una correccion de milimetros no exige
    girar 180 grados, que es lo que mantenia la oscilacion en Gazebo.
    Se respetan los limites del firmware: |v| <= v_max, |w| <= w_max.
    """
    speed = float(np.linalg.norm(u))
    if speed < P.v_dead:
        return 0.0, 0.0
    desired = math.atan2(u[1], u[0])
    e = math.atan2(math.sin(desired - theta), math.cos(desired - theta))
    if speed < P.v_turn_ref and abs(e) > math.pi / 2:
        e = math.atan2(math.sin(e - math.pi), math.cos(e - math.pi))
        v = -speed * math.cos(e)
    else:
        v = speed * max(math.cos(e), 0.0)
    w = P.k_heading * e * min(1.0, speed / P.v_turn_ref)
    vmax = P.v_max if v_limit is None else min(P.v_max, v_limit)
    v = max(min(v, vmax), -vmax)
    w = max(min(w, P.w_max), -P.w_max)
    return v, w


# ---------------------------------------------------------------- metricas
def cohesion(pos):
    """Distancia media al centroide [m] (menor = mas cohesionado)."""
    return float(np.mean(np.linalg.norm(pos - pos.mean(axis=0), axis=1)))


def polarization(theta):
    """Parametro de orden |<e^{i theta}>| en [0, 1] (1 = todos alineados)."""
    return float(abs(np.mean(np.exp(1j * np.asarray(theta)))))


def min_pair_distance(pos):
    n = len(pos)
    best = np.inf
    for i in range(n):
        for j in range(i + 1, n):
            best = min(best, float(np.linalg.norm(pos[i] - pos[j])))
    return best


def fraction_in_goal(pos, P: SwarmParams):
    return float(np.mean(np.linalg.norm(pos - np.asarray(P.goal), axis=1) < P.goal_r))
