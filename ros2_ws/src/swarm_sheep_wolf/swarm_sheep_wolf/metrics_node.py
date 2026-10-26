"""Registro de metricas del experimento en Gazebo.

Usa el ground truth de Gazebo (no disponible en el robot real) para medir:
  cohesion, polarizacion, distancia minima entre robots, colisiones,
  fraccion de ovejas en el corral, error de la odometria EKF de cada robot.

Escribe un CSV (parametro output) y publica /swarm/metrics como
std_msgs/Float64MultiArray [t, cohesion, polarizacion, d_min, frac_corral, err_odom].
"""
import csv
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from swarm_sheep_wolf import behaviors as BH
from swarm_sheep_wolf.agent_node import yaw_from_quat

COLLISION_DIST = 0.20   # [m] 0.9 * 2 * radio circunscrito (0.11 m)


class MetricsNode(Node):

    def __init__(self):
        super().__init__('swarm_metrics')
        self.declare_parameter('sheep_names', ['sheep1'])
        self.declare_parameter('wolf_name', 'wolf')
        self.declare_parameter('goal', [1.2, 1.2])
        self.declare_parameter('goal_r', 0.6)
        self.declare_parameter('output', 'swarm_metrics.csv')
        self.declare_parameter('duration', 0.0)
        self.sheep = list(self.get_parameter('sheep_names').value)
        self.wolf = self.get_parameter('wolf_name').value
        self.robots = self.sheep + [self.wolf]
        self.P = BH.SwarmParams(goal=tuple(self.get_parameter('goal').value),
                                goal_r=self.get_parameter('goal_r').value)
        self.duration = self.get_parameter('duration').value

        self.truth, self.odom = {}, {}
        for ns in self.robots:
            self.create_subscription(Odometry, f'/{ns}/ground_truth',
                                     lambda m, ns=ns: self._store(self.truth, ns, m), 10)
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     lambda m, ns=ns: self._store(self.odom, ns, m), 10)
        self.odom0 = {}
        self.truth0 = {}
        self.pub = self.create_publisher(Float64MultiArray, '/swarm/metrics', 10)
        out = os.path.expanduser(self.get_parameter('output').value)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'cohesion', 'polarization', 'min_dist', 'collisions',
                         'frac_goal', 'odom_err_mean'] +
                        [f'{ns}_{k}' for ns in self.robots for k in ('x', 'y', 'th')] +
                        [f'{ns}_odom_err' for ns in self.robots])
        self.collisions = 0
        self.contact = set()
        self.t0 = None
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'Registrando metricas en {out}')

    @staticmethod
    def _store(d, ns, m):
        p = m.pose.pose
        d[ns] = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _odom_error(self, ns):
        """Error de posicion de la odometria EKF expresada en el marco del mundo."""
        if ns not in self.odom or ns not in self.truth:
            return math.nan
        if ns not in self.odom0:
            self.odom0[ns] = self.odom[ns]
            self.truth0[ns] = self.truth[ns]
        ox, oy, oth0 = self.odom0[ns]
        tx0, ty0, tth0 = self.truth0[ns]
        dx, dy = self.odom[ns][0] - ox, self.odom[ns][1] - oy
        c, s = math.cos(tth0 - oth0), math.sin(tth0 - oth0)
        ex = tx0 + c * dx - s * dy - self.truth[ns][0]
        ey = ty0 + s * dx + c * dy - self.truth[ns][1]
        return math.hypot(ex, ey)

    def _tick(self):
        if not all(ns in self.truth for ns in self.robots):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        allp = np.array([self.truth[ns][:2] for ns in self.robots])
        ps = allp[:len(self.sheep)]
        th = [self.truth[ns][2] for ns in self.sheep]
        for i in range(len(allp)):
            for j in range(i + 1, len(allp)):
                touching = np.linalg.norm(allp[i] - allp[j]) < COLLISION_DIST
                if touching and (i, j) not in self.contact:
                    self.collisions += 1
                    self.contact.add((i, j))
                elif not touching:
                    self.contact.discard((i, j))
        errs = [self._odom_error(ns) for ns in self.robots]
        row = [t, BH.cohesion(ps), BH.polarization(th), BH.min_pair_distance(allp),
               self.collisions, BH.fraction_in_goal(ps, self.P), float(np.nanmean(errs))]
        self.w.writerow([f'{v:.4f}' for v in row] +
                        [f'{v:.4f}' for ns in self.robots for v in self.truth[ns]] +
                        [f'{v:.4f}' for v in errs])
        self.f.flush()
        self.pub.publish(Float64MultiArray(data=[float(v) for v in row]))
        if self.duration > 0 and t > self.duration:
            self.get_logger().info('Duracion alcanzada, fin del registro')
            raise SystemExit


def main():
    rclpy.init()
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
