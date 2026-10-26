"""Etapa 3 en Gazebo: recorre un cuadrado en lazo abierto y registra la odometria.

Envia a /<robot>/cmd_vel la misma secuencia que python/odometry_experiment.py
(lados de 1 m a 0.2 m/s, giros de 90 grados a 1 rad/s) y guarda en CSV:
  ground_truth (Gazebo), odom_raw (ruedas, como el ESP32) y odom (EKF con IMU).
Las tres poses se expresan en el marco del mundo usando la pose inicial.

  ros2 run swarm_sheep_wolf odom_test_node --ros-args -p robot:=sheep1 -p laps:=1
"""
import csv
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from swarm_sheep_wolf.agent_node import yaw_from_quat


class OdomTest(Node):

    def __init__(self):
        super().__init__('odom_test')
        self.declare_parameter('robot', 'sheep1')
        self.declare_parameter('laps', 1)
        self.declare_parameter('side', 1.0)
        self.declare_parameter('v', 0.2)
        self.declare_parameter('w', 1.0)
        self.declare_parameter('output', '~/swarm_logs/odom_test.csv')
        ns = self.get_parameter('robot').value
        side, v, w = (self.get_parameter(k).value for k in ('side', 'v', 'w'))
        laps = self.get_parameter('laps').value
        self.plan = []
        for _ in range(4 * laps):
            self.plan += [(v, 0.0, side / v), (0.0, 0.0, 0.5), (0.0, w, math.pi / 2 / w), (0.0, 0.0, 0.5)]
        self.plan.append((0.0, 0.0, 1.0))
        self.d = {}
        for topic in ('ground_truth', 'odom_raw', 'odom'):
            self.create_subscription(Odometry, f'/{ns}/{topic}', lambda m, k=topic: self._store(k, m), 20)
        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        out = os.path.expanduser(self.get_parameter('output').value)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        self.f = open(out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'gt_x', 'gt_y', 'gt_th', 'raw_x', 'raw_y', 'raw_th', 'ekf_x', 'ekf_y', 'ekf_th'])
        self.i = 0
        self.t_seg = None
        self.t0 = None
        self.origin = {}
        self.done = False
        self.create_timer(0.02, self._tick)

    def _store(self, k, m):
        p = m.pose.pose
        self.d[k] = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _world(self, k):
        """Lleva la pose de odom_raw / odom (que empiezan en 0) al marco del mundo."""
        if k == 'ground_truth':
            return self.d[k]
        if k not in self.origin:
            self.origin[k] = self.d[k]
        ox, oy, oth = self.origin[k]
        gx, gy, gth = self.origin['gt']
        x, y, th = self.d[k]
        dx, dy = x - ox, y - oy
        c, s = math.cos(gth - oth), math.sin(gth - oth)
        return (gx + c * dx - s * dy, gy + s * dx + c * dy,
                math.atan2(math.sin(th - oth + gth), math.cos(th - oth + gth)))

    def _tick(self):
        if self.done or len(self.d) < 3:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = self.t_seg = now
            self.origin['gt'] = self.d['ground_truth']
        v, w, T = self.plan[self.i]
        if now - self.t_seg >= T:
            self.i += 1
            self.t_seg = now
            if self.i >= len(self.plan):
                self.pub.publish(Twist())
                self.done = True
                self.f.close()
                self.get_logger().info('Prueba terminada')
                raise SystemExit
            v, w, T = self.plan[self.i]
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub.publish(cmd)
        row = [now - self.t0]
        for k in ('ground_truth', 'odom_raw', 'odom'):
            row += list(self._world(k))
        self.w.writerow([f'{x:.5f}' for x in row])


def main():
    rclpy.init()
    node = OdomTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
