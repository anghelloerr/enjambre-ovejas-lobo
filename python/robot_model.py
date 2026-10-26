"""Modelo de un MicroROS-Pi5 para simulacion de alto nivel y estudio de odometria.

Cadena modelada (igual que el robot real):
  cmd_vel (v, w) -> cinematica inversa del firmware -> 4 velocidades de rueda
  -> lazo PID de cada motor (aprox. primer orden, tau) -> deslizamiento
  -> cinematica directa REAL (skid-steer con factor chi)
  -> encoders cuantizados (1040 cpr) + giroscopo con ruido y sesgo
  -> odometria (solo encoders | encoders + giroscopo | EKF)
"""
import math

import numpy as np

import params as PR

# Trocha efectiva que asume el firmware: w = (vR - vL) / (2*APB)
CHI_FIRMWARE = 2 * PR.APB / PR.W   # = 1.704


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def inverse_kinematics(v, w):
    """Firmware Yahboom (Motion_Ctrl): velocidades lineales de las ruedas [m/s].
    Orden: M1 izq-del, M2 izq-tras, M3 der-del, M4 der-tras."""
    vl = v - w * PR.APB
    vr = v + w * PR.APB
    sat = lambda x: max(min(x, PR.V_MAX), -PR.V_MAX)
    return np.array([sat(vl), sat(vl), sat(vr), sat(vr)])


def forward_kinematics_firmware(wheels):
    """Firmware Yahboom (Motion_Get_Speed): (Vx, Wz) a partir de las ruedas."""
    m1, m2, m3, m4 = wheels
    vx = (m1 + m2 + m3 + m4) / 4.0
    wz = -(m1 + m2 - m3 - m4) / 4.0 / PR.APB
    return vx, wz


class MicroRosRobot:
    """Robot 'real' simulado. Integra la pose verdadera y genera medidas."""

    def __init__(self, x=0.0, y=0.0, th=0.0, chi_true=1.55, tau=0.025,
                 slip_sigma=PR.SLIP_SIGMA, gyro_sigma=PR.GYRO_SIGMA,
                 gyro_bias=PR.GYRO_BIAS, rng=None, delay=0.0):
        self.x, self.y, self.th = x, y, th
        self.chi_true = chi_true
        self.tau = tau
        self.slip_sigma = slip_sigma
        self.gyro_sigma = gyro_sigma
        self.gyro_bias = gyro_bias
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.wheel = np.zeros(4)          # velocidad lineal de cada rueda [m/s]
        self.enc_count = np.zeros(4)      # cuentas acumuladas (enteras)
        self._enc_frac = np.zeros(4)
        self.v_true = 0.0
        self.w_true = 0.0
        self.delay = delay                # [s] retardo puro entre la orden y el lazo de rueda
        self._queue = []                  # (t_aplicacion, v, w)
        self._t = 0.0
        self._cmd = (0.0, 0.0)

    def step(self, v_cmd, w_cmd, dt):
        if self.delay > 0:
            self._queue.append((self._t + self.delay, v_cmd, w_cmd))
            while self._queue and self._queue[0][0] <= self._t + 1e-9:
                self._cmd = self._queue.pop(0)[1:]
            v_cmd, w_cmd = self._cmd
        self._t += dt
        target = inverse_kinematics(v_cmd, w_cmd)
        # Lazo de velocidad cerrado ~ primer orden (ver docs/04_motor_y_control.md)
        a = dt / (self.tau + dt)
        self.wheel = self.wheel + a * (target - self.wheel)

        # Deslizamiento: la rueda gira (encoder) pero el suelo avanza distinto
        slip = 1.0 + self.slip_sigma * self.rng.standard_normal(4)
        ground = self.wheel * slip
        vl = 0.5 * (ground[0] + ground[1])
        vr = 0.5 * (ground[2] + ground[3])
        self.v_true = 0.5 * (vl + vr)
        self.w_true = (vr - vl) / (self.chi_true * PR.W)

        # Integracion exacta del uniciclo (arco de circunferencia)
        th0 = self.th
        if abs(self.w_true) > 1e-9:
            self.x += self.v_true / self.w_true * (math.sin(th0 + self.w_true * dt) - math.sin(th0))
            self.y -= self.v_true / self.w_true * (math.cos(th0 + self.w_true * dt) - math.cos(th0))
        else:
            self.x += self.v_true * math.cos(th0) * dt
            self.y += self.v_true * math.sin(th0) * dt
        self.th = wrap(th0 + self.w_true * dt)

        # Encoders: cuentan el giro de la rueda (no el suelo), cuantizados
        counts = self.wheel * dt / PR.COUNTS_TO_M + self._enc_frac
        whole = np.floor(counts)
        self._enc_frac = counts - whole
        self.enc_count += whole
        gyro = self.w_true + self.gyro_bias + self.gyro_sigma * self.rng.standard_normal()
        return whole.copy(), gyro

    @property
    def pose(self):
        return np.array([self.x, self.y, self.th])


class OdometryEncoders:
    """Odometria solo con encoders (lo que publica el ESP32 en odom_raw)."""

    def __init__(self, x=0.0, y=0.0, th=0.0, scale_lin=1.0, scale_ang=1.0):
        self.p = np.array([x, y, th], dtype=float)
        self.scale_lin = scale_lin
        self.scale_ang = scale_ang

    def update(self, d_counts, dt):
        wheels = d_counts * PR.COUNTS_TO_M / dt
        v, w = forward_kinematics_firmware(wheels)
        v *= self.scale_lin
        w *= self.scale_ang
        th = self.p[2]
        self.p[0] += v * math.cos(th + 0.5 * w * dt) * dt
        self.p[1] += v * math.sin(th + 0.5 * w * dt) * dt
        self.p[2] = wrap(th + w * dt)
        return v, w


class OdometryEncGyro(OdometryEncoders):
    """Encoders para v, giroscopo para w (equivale a confiar 100 % en la IMU)."""

    def update(self, d_counts, dt, gyro=0.0):
        wheels = d_counts * PR.COUNTS_TO_M / dt
        v, _ = forward_kinematics_firmware(wheels)
        v *= self.scale_lin
        w = gyro
        th = self.p[2]
        self.p[0] += v * math.cos(th + 0.5 * w * dt) * dt
        self.p[1] += v * math.sin(th + 0.5 * w * dt) * dt
        self.p[2] = wrap(th + w * dt)
        return v, w


class OdometryEKF:
    """EKF de 7 estados [x, y, th, v, w, b_g, k_w] que fusiona encoders y giroscopo.

    Version reducida de lo que hace robot_localization en el robot real
    (ekf_filter_node con odom_raw + imu/data), con dos estados extra:
      b_g : sesgo del giroscopo                gyro  = w + b_g
      k_w : escala angular de los encoders     w_enc = k_w * w
    k_w modela el deslizamiento del skid-steer (k_w = chi_fw / chi_real), de modo
    que el filtro se autocalibra en linea. Cuando w = 0 el giroscopo observa b_g;
    al girar, la diferencia entre w_enc y gyro observa k_w.
    """

    def __init__(self, x=0.0, y=0.0, th=0.0, q_v=0.5, q_w=2.0, q_b=1e-7, q_k=1e-6,
                 r_v=0.02 ** 2, r_w_enc=0.05 ** 2, r_gyro=PR.GYRO_SIGMA ** 2):
        self.s = np.array([x, y, th, 0.0, 0.0, 0.0, 1.0])
        self.Pm = np.diag([1e-6, 1e-6, 1e-6, 1e-2, 1e-2, 1e-5, 0.1])
        self.q = np.array([q_v, q_w, q_b, q_k])
        self.R = np.diag([r_v, r_w_enc, r_gyro])

    def predict(self, dt):
        x, y, th, v, w, b, k = self.s
        self.s = np.array([x + v * math.cos(th) * dt, y + v * math.sin(th) * dt,
                           wrap(th + w * dt), v, w, b, k])
        F = np.eye(7)
        F[0, 2] = -v * math.sin(th) * dt
        F[0, 3] = math.cos(th) * dt
        F[1, 2] = v * math.cos(th) * dt
        F[1, 3] = math.sin(th) * dt
        F[2, 4] = dt
        Q = np.zeros((7, 7))
        Q[3, 3] = self.q[0] * dt
        Q[4, 4] = self.q[1] * dt
        Q[5, 5] = self.q[2] * dt
        Q[6, 6] = self.q[3] * dt
        self.Pm = F @ self.Pm @ F.T + Q

    def update(self, d_counts, dt, gyro):
        self.predict(dt)
        wheels = d_counts * PR.COUNTS_TO_M / dt
        v_enc, w_enc = forward_kinematics_firmware(wheels)
        z = np.array([v_enc, w_enc, gyro])
        _, _, _, v, w, b, k = self.s
        h = np.array([v, k * w, w + b])
        H = np.zeros((3, 7))
        H[0, 3] = 1.0
        H[1, 4] = k
        H[1, 6] = w
        H[2, 4] = 1.0
        H[2, 5] = 1.0
        y = z - h
        S = H @ self.Pm @ H.T + self.R
        K = self.Pm @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ y
        self.s[2] = wrap(self.s[2])
        I_KH = np.eye(7) - K @ H
        self.Pm = I_KH @ self.Pm @ I_KH.T + K @ self.R @ K.T   # forma de Joseph
        return self.s[3], self.s[4]

    @property
    def p(self):
        return self.s[:3]
