"""Parametros del Yahboom MicroROS-Pi5 (espejo de matlab/params_microros.m).

Origen de cada valor:
  [URDF] yahboomcar_description/urdf/MicroROS.urdf
  [FW]   firmware microROS_Robot_V2.0.0.bin y tutoriales ESP32 de Yahboom
  [MESH] medido sobre las mallas STL oficiales
  [SUP]  supuesto de diseno, a verificar en el robot real
"""
import math

# Geometria
L = 0.0455 + 0.0495          # [m] batalla                      [URDF]
W = 2 * 0.0675               # [m] trocha                       [URDF]
A = W / 2
B = L / 2
APB = A + B                  # [m] ROBOT_APB = 0.115            [FW]
WHEEL_R = 0.024              # [m] radio de rueda               [MESH]
R_SAFE = 0.11                # [m] radio circunscrito

# Masa
MASS = 1.30                  # [kg]                             [SUP]

# Motor y encoder
GEAR = 20                    # [FW]
PPR = 13                     # [FW]
CPR = 4 * PPR * GEAR         # 1040 cuentas / vuelta de rueda  [FW]
TS_PID = 0.010               # [s]                             [FW]
V_MAX = 1.0                  # [m/s] por rueda                 [FW]
W_MAX = 5.0                  # [rad/s]                         [FW]

# Sensores y deslizamiento
GYRO_SIGMA = 0.005           # [rad/s]                         [SUP]
GYRO_BIAS = 0.002            # [rad/s]                         [SUP]
SLIP_SIGMA = 0.02            # [-]                             [SUP]
SKID_CHI = 1.55              # factor de derrape real (docs/03) [SUP]
CMD_DELAY = 0.10             # [s] retardo cmd_vel -> ruedas (DDS/micro-ROS + lazo); reproduce Gazebo [SUP]

# Ovejas-lobo
SWARM = dict(
    n_sheep=5,
    r_sep=0.45, r_nb=1.00, r_fear=1.20,
    w_sep=3.0, w_coh=0.6, w_ali=0.4, w_flee=2.0, w_wall=2.0,
    v_sheep=0.20, v_wolf=0.25, k_heading=2.5,
    arena=4.0, goal=(1.2, 1.2), goal_r=0.6,
)

DT_SIM = 0.05                # [s] paso del simulador de alto nivel (20 Hz)
COUNTS_TO_M = 2 * math.pi * WHEEL_R / CPR
