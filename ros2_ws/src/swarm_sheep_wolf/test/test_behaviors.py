"""Pruebas unitarias de la ley de control ovejas-lobo (no requieren ROS)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from swarm_sheep_wolf import behaviors as BH  # noqa: E402

P = BH.SwarmParams()


def test_sheep_flees_from_wolf():
    pos = np.array([[0.0, 0.0], [3.0, 3.0]])
    vel = np.zeros((2, 2))
    u, fear = BH.sheep_desired_velocity(0, pos, vel, np.array([-0.5, 0.0]), P)
    assert fear > 0.5
    assert u[0] > 0, 'la oveja debe alejarse del lobo (hacia +x)'


def test_sheep_ignores_far_wolf():
    pos = np.array([[0.0, 0.0], [0.5, 0.0]])
    _, fear = BH.sheep_desired_velocity(0, pos, np.zeros((2, 2)), np.array([1.9, 1.9]), P)
    assert fear == 0.0


def test_separation_pushes_apart():
    pos = np.array([[0.0, 0.0], [0.1, 0.0]])
    u, _ = BH.sheep_desired_velocity(0, pos, np.zeros((2, 2)), None, P)
    assert u[0] < 0


def test_speed_limits():
    pos = np.array([[0.0, 0.0], [0.05, 0.0]])
    u, _ = BH.sheep_desired_velocity(0, pos, np.zeros((2, 2)), np.array([-0.1, 0.0]), P)
    assert np.linalg.norm(u) <= P.v_sheep + 1e-9


def test_unicycle_respects_firmware_limits():
    v, w = BH.unicycle_command(np.array([5.0, 0.0]), math.pi, P)
    assert v == 0.0 and abs(w) <= P.w_max
    v, w = BH.unicycle_command(np.array([5.0, 0.0]), 0.0, P)
    assert v <= P.v_max and w == 0.0


def test_herd_drive_point_behind_flock():
    pos = np.array([[0.0, 0.0], [0.2, 0.0], [0.0, 0.2]])
    target, mode = BH.wolf_herd_target(np.array([-1.0, -1.0]), pos, P)
    gcm = pos.mean(axis=0)
    goal = np.asarray(P.goal)
    assert mode == 'drive'
    # el objetivo esta del lado opuesto al corral respecto al centro del rebano
    assert np.dot(target - gcm, goal - gcm) < 0


def test_herd_collects_straggler():
    pos = np.array([[0.0, 0.0], [0.1, 0.0], [0.0, 0.1], [1.6, -1.6]])
    target, mode = BH.wolf_herd_target(np.array([0.0, -1.0]), pos, P)
    assert mode == 'collect'
    assert np.linalg.norm(target - pos[3]) < 0.6


def test_metrics():
    pos = np.array([[1.0, 0.0], [-1.0, 0.0]])
    assert abs(BH.cohesion(pos) - 1.0) < 1e-9
    assert abs(BH.min_pair_distance(pos) - 2.0) < 1e-9
    assert abs(BH.polarization([0.0, 0.0]) - 1.0) < 1e-9
    assert BH.polarization([0.0, math.pi]) < 1e-9


def test_wolf_guards_open_side_when_flock_in_corral():
    """Con todo el rebano en el corral, el lobo vigila desde el lado abierto (no desde la esquina)."""
    P = BH.SwarmParams()
    goal = np.array(P.goal)
    pos = goal + np.array([[0.1, 0.0], [-0.1, 0.1], [0.0, -0.1], [0.15, 0.15], [-0.1, -0.1]])
    target, mode = BH.wolf_herd_target(np.array([0.0, 0.0]), pos, P)
    assert mode == 'guard'
    # el punto de vigilancia esta mas cerca del centro de la arena que el corral
    assert np.linalg.norm(target) < np.linalg.norm(goal)


def test_wolf_drives_from_open_side_near_goal():
    """Si el centroide esta casi sobre el corral, el lobo no se va a la esquina."""
    P = BH.SwarmParams()
    goal = np.array(P.goal)
    pos = goal + np.array([[0.3, 0.3], [0.2, 0.35], [0.35, 0.2], [0.25, 0.25], [0.7, 0.1]])
    target, mode = BH.wolf_herd_target(np.array([0.0, 0.0]), pos, P)
    assert mode in ('drive', 'collect')
    if mode == 'drive':
        assert np.linalg.norm(target) < np.linalg.norm(pos.mean(axis=0))


def test_separation_is_continuous_at_r_sep():
    """La separacion se anula en d = r_sep: sin saltos que hagan oscilar el rumbo."""
    P = BH.SwarmParams(w_coh=0.0, w_ali=0.0, w_wall=0.0)
    vel = np.zeros((2, 2))
    inside = np.array([[0.0, 0.0], [P.r_sep - 1e-3, 0.0]])
    u, _ = BH.sheep_desired_velocity(0, inside, vel, None, P)
    assert np.linalg.norm(u) < 0.01
    close = np.array([[0.0, 0.0], [0.5 * P.r_sep, 0.0]])
    u, _ = BH.sheep_desired_velocity(0, close, vel, None, P)
    assert u[0] < 0          # se aleja del vecino


def test_unicycle_deadband_avoids_heading_chatter():
    """Una velocidad deseada minima (equilibrio + ruido) no hace girar al robot."""
    P = BH.SwarmParams()
    assert BH.unicycle_command(np.array([-0.01, 0.0]), 0.0, P) == (0.0, 0.0)
    _, w_slow = BH.unicycle_command(np.array([0.0, 0.03]), 0.0, P)
    _, w_fast = BH.unicycle_command(np.array([0.0, 0.20]), 0.0, P)
    assert 0 < w_slow < w_fast


def test_unicycle_reverses_for_small_corrections_behind():
    """Una correccion lenta hacia atras se hace retrocediendo, sin media vuelta."""
    P = BH.SwarmParams()
    v, w = BH.unicycle_command(np.array([-0.03, 0.0]), 0.0, P)
    assert v < 0 and abs(w) < 1e-9
    v, w = BH.unicycle_command(np.array([-0.20, 0.0]), 0.0, P)   # huida rapida: da la vuelta
    assert v == 0.0 and abs(w) > 0
