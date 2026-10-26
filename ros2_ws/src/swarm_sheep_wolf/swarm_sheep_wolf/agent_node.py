"""Nodo de un agente del enjambre (oveja o lobo).

Cada robot corre su propio agent_node: el control es descentralizado. El nodo
solo necesita las posiciones de los demas robots, que en el robot real llegarian
por DDS (cada MicroROS-Pi5 publica su odometria en su namespace).

Parametros
  robot_name   : namespace propio (sheep1, ..., wolf)
  role         : 'sheep' | 'wolf'
  wolf_mode    : 'herd' (pastor) | 'hunt' (cazador)
  sheep_names  : lista de namespaces de las ovejas
  wolf_name    : namespace del lobo
  pose_source  : 'ground_truth' (Gazebo) | 'tf' (odometria EKF via TF world->ns/base_footprint)
  + parametros del algoritmo (config/swarm.yaml)

Publica /<robot_name>/cmd_vel (geometry_msgs/Twist), igual que el teclado de Yahboom.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from swarm_sheep_wolf import behaviors as BH


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AgentNode(Node):

    def __init__(self):
        super().__init__('agent')
        self.declare_parameter('robot_name', 'sheep1')
        self.declare_parameter('role', 'sheep')
        self.declare_parameter('wolf_mode', 'herd')
        self.declare_parameter('sheep_names', ['sheep1'])
        self.declare_parameter('wolf_name', 'wolf')
        self.declare_parameter('pose_source', 'ground_truth')
        self.declare_parameter('rate_hz', 20.0)
        defaults = BH.SwarmParams()
        for k in ['r_sep', 'r_nb', 'r_fear', 'w_sep', 'w_coh', 'w_ali', 'w_flee', 'w_wall',
                  'v_sheep', 'v_wolf', 'k_heading', 'arena', 'goal_r']:
            self.declare_parameter(k, float(getattr(defaults, k)))
        self.declare_parameter('goal', list(defaults.goal))

        gp = lambda n: self.get_parameter(n).value
        self.name = gp('robot_name')
        self.role = gp('role')
        self.wolf_mode = gp('wolf_mode')
        self.sheep = list(gp('sheep_names'))
        self.wolf = gp('wolf_name')
        self.source = gp('pose_source')
        self.P = BH.SwarmParams(**{k: gp(k) for k in [
            'r_sep', 'r_nb', 'r_fear', 'w_sep', 'w_coh', 'w_ali', 'w_flee', 'w_wall',
            'v_sheep', 'v_wolf', 'k_heading', 'arena', 'goal_r']}, goal=tuple(gp('goal')))

        self.robots = self.sheep + [self.wolf]
        self.pose = {}           # ns -> (x, y, th)
        self.state = {}          # ns -> (x, y, th, vx, vy)
        self._prev = {}
        if self.source == 'ground_truth':
            for ns in self.robots:
                self.create_subscription(Odometry, f'/{ns}/ground_truth',
                                         lambda m, ns=ns: self._on_odom(ns, m), 10)
        else:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, f'/{self.name}/swarm_mode', 10)
        self.dt = 1.0 / gp('rate_hz')
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'{self.name}: rol={self.role} modo={self.wolf_mode} fuente={self.source}')

    def _on_odom(self, ns, msg):
        p = msg.pose.pose
        self.pose[ns] = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def _update_from_tf(self):
        for ns in self.robots:
            try:
                tr = self.tf_buffer.lookup_transform('world', f'{ns}/base_footprint', rclpy.time.Time())
            except TransformException:
                continue
            x, y = tr.transform.translation.x, tr.transform.translation.y
            self.pose[ns] = (x, y, yaw_from_quat(tr.transform.rotation))

    def _update_velocities(self):
        """Velocidad de cada robot por diferencias de posicion, filtrada (tau = 0,5 s).

        No se usa el twist de la odometria: un skid-steer que gira en el sitio cabecea y
        su twist muestra hasta 0,08 m/s de velocidad lineal aparente. Esa velocidad entra
        en la regla de alineacion de los vecinos y mantiene una oscilacion del rumbo (docs/07).
        """
        a = self.dt / (0.5 + self.dt)
        for ns, (x, y, th) in self.pose.items():
            px, py, vx, vy = self._prev.get(ns, (x, y, 0.0, 0.0))
            vx += a * ((x - px) / self.dt - vx)
            vy += a * ((y - py) / self.dt - vy)
            self._prev[ns] = (x, y, vx, vy)
            self.state[ns] = (x, y, th, vx, vy)

    def _tick(self):
        if self.source != 'ground_truth':
            self._update_from_tf()
        self._update_velocities()
        if not all(ns in self.state for ns in self.robots):
            return
        pos = np.array([self.state[ns][:2] for ns in self.sheep])
        vel = np.array([self.state[ns][3:5] for ns in self.sheep])
        wolf_pos = np.array(self.state[self.wolf][:2])
        me = self.state[self.name]

        if self.role == 'sheep':
            i = self.sheep.index(self.name)
            u, fear = BH.sheep_desired_velocity(i, pos, vel, wolf_pos, self.P)
            mode = f'fear={fear:.2f}'
        else:
            u, mode = BH.wolf_desired_velocity(wolf_pos, pos, self.P, mode=self.wolf_mode)
        v, w = BH.unicycle_command(u, me[2], self.P)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)
        self.pub_mode.publish(String(data=mode))


def main():
    rclpy.init()
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
