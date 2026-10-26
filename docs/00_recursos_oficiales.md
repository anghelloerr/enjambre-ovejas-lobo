# 0. Recursos oficiales de Yahboom (etapa 1)

Este documento lista de dónde sale cada dato del gemelo digital y qué es trabajo nuevo de este repositorio.

## 0.1 Fuentes

| Fuente | Qué se usó |
| --- | --- |
| [YahboomTechnology/MicroROS-Car-Pi5](https://github.com/YahboomTechnology/MicroROS-Car-Pi5) | Tutoriales en PDF: estimación de estado, calibración lineal y angular, modelo URDF, multirrobot y Gazebo |
| Código fuente oficial (`18. Summary of program source code`, `yahboomcar_ws`) | `yahboomcar_description` (URDF y mallas STL), `yahboomcar_bringup` (EKF, calibración), `yahboomcar_multi` (namespaces y EKF por robot) |
| `Factory-Firmware.zip` (`microROS_Robot_V2.0.0.bin`) | Constantes del ESP32: `ROBOT_APB = 0.115`, límites 1 m/s por rueda y 5 rad/s, PID 1 / 0,2 / 0,2 |

Las mallas de `ros2_ws/src/microros_description/meshes/` son las oficiales de Yahboom, sin modificar.

## 0.2 Lo que aporta Yahboom y lo que es nuevo

| Elemento | Yahboom | Este repositorio |
| --- | --- | --- |
| URDF | `MicroROS.urdf`, solo visual, masa total 0,36 kg | xacro con prefijo por robot, colisiones simplificadas, masas e inercias realistas, plugins de Gazebo |
| Simulación | No hay. El tutorial 14.22 solo carga un cubo en Gazebo Classic | Gemelo digital completo en Gazebo Fortress, N robots |
| Odometría | ESP32 → `odom_raw`; `imu_filter_madgwick`; `robot_localization` | Análisis del error del skid-steer, EKF de 7 estados autocalibrado |
| Control de motores | PID incremental en el ESP32 | Modelo del motor, diseño PI con márgenes, análisis del PID de fábrica |
| Multirrobot | Namespaces `robotN/`, mismo `ROS_DOMAIN_ID`, un EKF por robot | Mismo patrón (`sheepN/`, `wolf/`) en simulación |
| Comportamiento colectivo | No hay | Algoritmo ovejas–lobo (Reynolds + Strömbom) |

## 0.3 Interfaz del robot real que reproduce el gemelo

| Tópico | Tipo | Productor real |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | Lo consume el ESP32 (`Motion_Ctrl`) |
| `odom_raw` | `nav_msgs/Odometry` | ESP32, encoders |
| `imu` | `sensor_msgs/Imu` | ESP32 |
| `scan` | `sensor_msgs/LaserScan` | LiDAR MS200 |
| `odom` + TF `odom → base_footprint` | | `ekf_filter_node` (`robot_localization`) |

Como el gemelo publica y consume los mismos tópicos, los nodos de `swarm_sheep_wolf` deberían funcionar sin cambios en el robot real. La única excepción es la fuente de pose: en Gazebo existe `ground_truth`; en el laboratorio hay que usar `pose_source:=tf` (odometría) o un sistema de captura de movimiento.

## 0.4 Datos que hay que medir en el robot físico

Los valores marcados **SUP** en [01_parametros.md](01_parametros.md) son supuestos. Por orden de impacto:

1. Radio de rueda con regla (las mallas dan 24 mm; en las fotos parece mayor).
2. Masa total con báscula.
3. Curva de velocidad del motor ante un escalón de PWM ([04_motor_y_control.md](04_motor_y_control.md), sección 4.5).
4. `PWM_MOTOR_MAX_VALUE` en el código del ESP32.
5. Factor de derrape χ real con `calibrate_angular` en el suelo del laboratorio.
