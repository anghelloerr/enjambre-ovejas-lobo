# 2. Gemelo digital en ROS 2 Humble + Gazebo Fortress

Paquetes en `ros2_ws/src`:

| Paquete | Contenido |
| --- | --- |
| `microros_description` | `urdf/microros.urdf.xacro`, mallas STL oficiales, `display.launch.py` (RViz), `config/robot_params.yaml` |
| `microros_gazebo` | `worlds/arena.sdf` (arena 4 × 4 m con corral), `launch/sim.launch.py` (N robots), `config/ekf.yaml` |
| `swarm_sheep_wolf` | Algoritmo ovejas–lobo, registro de métricas y prueba de odometría (sección 7) |

Probado con ROS 2 Humble y Ignition Gazebo 6.16 (Fortress), instalados con RoboStack (conda) sobre Ubuntu 24.04. La configuración de referencia es **Ubuntu 22.04 + `ros-humble-desktop` + `ros-humble-ros-gz`**, que instala Fortress.

## 2.1 Del URDF de Yahboom al gemelo digital

| Cambio | Motivo |
| --- | --- |
| Prefijo `<ns>/` en todos los links y juntas | Varios robots en el mismo árbol TF, como hace `MicroROS_robot1.urdf` de Yahboom |
| Link `base_footprint` en el suelo, con `base_link` a z = r + 0,02125 | Que las ruedas apoyen (ver docs/01, sección 1.1) |
| Colisiones simplificadas: caja de chasis y cilindros de rueda | Las mallas STL de SolidWorks dan contactos inestables; se mantienen como visual |
| Masas realistas: 1,18 kg de chasis y 4 × 0,03 kg de ruedas | El URDF original suma 0,36 kg |
| Eje de las ruedas derechas (0 −1 0) → (0 1 0) | En el robot real el ESP32 invierte el signo. `DiffDrive` necesita que un giro positivo sea avance en los dos lados; con el eje original el robot giraba en el sitio en lugar de avanzar |
| Gimbal de cámara fijo | No interviene en el enjambre |

## 2.2 Plugins y tópicos (misma interfaz que el robot real)

| Elemento | Plugin de Gazebo | Tópico ROS 2 | Equivalente real |
| --- | --- | --- | --- |
| Tracción 4WD | `DiffDrive` con 2 juntas por lado, `wheel_separation = 2·APB = 0,23 m` | `/<ns>/cmd_vel`, `/<ns>/odom_raw` | ESP32 `Motion_Ctrl` y `odom_raw` |
| IMU | sensor `imu` 100 Hz, giroscopo con σ = 0,005 rad/s y sesgo 0,002 rad/s | `/<ns>/imu` | `imu` del ESP32 |
| LiDAR MS200 | `gpu_lidar` 360 muestras, 10 Hz, 0,03–12 m (opcional: `use_lidar:=true`) | `/<ns>/scan` | `scan` |
| Juntas | `JointStatePublisher` | `/<ns>/joint_states` | |
| Ground truth | `OdometryPublisher` en el marco `world` | `/<ns>/ground_truth` | No existe: hace falta un sistema de captura de movimiento |
| Fusión | `robot_localization` EKF | `/<ns>/odom` + TF `<ns>/odom → <ns>/base_footprint` | `ekf_filter_node` de `yahboomcar_bringup` |

`wheel_separation = 2·APB` hace que Gazebo traduzca `cmd_vel` a velocidades de rueda **exactamente igual que el firmware**. El derrape real queda a cargo del motor de física.

## 2.3 Calibración de la fricción (skid-steer)

Se midió el giro real en el sitio frente al ordenado (4 s a 1 rad/s) con distintos coeficientes de fricción de rueda:

| μ longitudinal | μ lateral | Giro real / ordenado | χ efectivo |
| --- | --- | --- | --- |
| 1,0 | 0,3 | 0,34 | 4,96 |
| 1,0 | 0,05 | 0,35 | 4,86 |
| 1,0 | 1,0 | 1,11 | **1,53** |
| 0,6 | 0,6 | 1,12 | **1,52** |

Con fricción **isótropa**, Gazebo reproduce la teoría $\chi=1+(L/W)^2=1{,}495$ (docs/03). Con fricción anisótropa, DART aplica μ₂ en una dirección que no está ligada a la rueda, y el robot apenas gira. Se deja μ = 1,0 isótropa (`mu_lat`/`mu_long` en el xacro).

## 2.4 EKF

`config/ekf.yaml` parte del de Yahboom (`ekf_robotN.yaml`) con dos cambios, justificados en docs/05:

- De `odom_raw` solo se fusiona *vx*. La ω de las ruedas está sesgada por el derrape.
- De la IMU solo se usa *vyaw*. Sin magnetómetro, el yaw absoluto no es observable.

## 2.5 Cómo ejecutarlo

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# un robot en RViz (como display_launch.py de Yahboom)
ros2 launch microros_description display.launch.py

# gemelo digital: 5 ovejas + 1 lobo, con interfaz grafica
ros2 launch microros_gazebo sim.launch.py n_sheep:=5 gui:=true

# moverlo a mano (en otra terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/sheep1/cmd_vel

# prueba de odometria: cuadrado de 1 m x 3 vueltas
ros2 run swarm_sheep_wolf odom_test_node --ros-args -p robot:=sheep1 -p laps:=3 -p use_sim_time:=true
python3 ../python/analyze_gazebo_odom.py ~/swarm_logs/odom_test.csv
```
