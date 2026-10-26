# 1. Parámetros del Yahboom MicroROS-Pi5

Todos los modelos del repositorio (Python, MATLAB/Simulink/Simscape y Gazebo) usan la **misma tabla de parámetros**:

- `matlab/params_microros.m`
- `python/params.py`
- `ros2_ws/src/microros_description/config/robot_params.yaml`

Cada valor tiene una etiqueta de origen:

| Etiqueta | Origen |
| --- | --- |
| **URDF** | `yahboomcar_description/urdf/MicroROS.urdf`, del código fuente oficial de Yahboom |
| **FW** | Firmware `microROS_Robot_V2.0.0.bin` y tutoriales del ESP32 de Yahboom |
| **MESH** | Medido sobre las mallas STL oficiales con `trimesh` |
| **SUP** | Supuesto de diseño que hay que verificar en el robot físico |

## 1.1 Geometría

| Símbolo | Valor | Origen | Cómo se obtuvo |
| --- | --- | --- | --- |
| Batalla *L* | 0,095 m | URDF | Juntas `zq_Joint` (x = +0,0455) y `zh_Joint` (x = −0,0495) |
| Trocha *W* | 0,135 m | URDF | Juntas izquierda (y = +0,0675) y derecha (y = −0,0675) |
| *A* = *W*/2 | 0,0675 m | URDF | |
| *B* = *L*/2 | 0,0475 m | URDF | |
| `ROBOT_APB` = *A* + *B* | **0,115 m** | **FW** | El valor `0.115f` aparece en el binario del firmware, junto a 0,25 y 5,0 (límites). Coincide exactamente con *A* + *B* del URDF |
| Radio de rueda *r* | 0,024 m | MESH | Caja envolvente de `zq_Link.STL`: 48 × 21,5 × 48 mm. **Hay que medirlo con regla**, porque en las fotos la rueda parece algo mayor |
| Chasis | 0,165 × 0,133 × 0,078 m | MESH | `base_link.STL` |
| Altura del eje | 0,02125 m bajo `base_link` | URDF | |

> **Inconsistencia detectada en el modelo de Yahboom.** La TF `base_footprint → base_link` del fabricante es z = 0,05 m. Con *r* = 0,024 m, las ruedas quedarían 4,75 mm por encima del suelo. En el gemelo digital, `base_link` se coloca a *r* + 0,02125 = 0,04525 m para que las ruedas apoyen. Si al medir la rueda real sale *r* ≈ 0,029 m, entonces el 0,05 m de Yahboom era correcto.

## 1.2 Motor y encoder

| Parámetro | Valor | Origen |
| --- | --- | --- |
| Motor | 310 con encoder Hall AB (el README del repositorio dice "370"; los tutoriales del ESP32 dicen "310") | FW |
| Reducción *N* | 1:20 | FW (tutorial *Read motor encoder data*) |
| Líneas del encoder | 13 | FW |
| Cuentas por vuelta de rueda | 4 × 13 × 20 = **1040** (cuadratura) | FW |
| Periodo del PID | 10 ms | FW (`MOTOR_PID_PERIOD`) |
| PID por defecto | incremental, Kp = 1,0, Ki = 0,2, Kd = 0,2 | FW |
| Límite de velocidad por rueda | ±1,0 m/s | FW (`Motor_Set_Speed`) |
| Límite angular | ±5,0 rad/s | FW (`Motion_Ctrl`, también presente en el binario) |
| Batería | 7,4 V, 2000 mAh | Yahboom |
| *R*, *L*, ω en vacío, *J*<sub>m</sub>, *b*<sub>m</sub> | 2,5 Ω, 1 mH, 400 rpm a la salida, 1e-7, 1e-7 | **SUP** |

Los parámetros eléctricos del motor no están publicados. Se eligieron para que la velocidad en vacío a 7,4 V dé ≈ 1,0 m/s en la rueda, que es el límite del firmware. En la sección 4 se describe cómo identificarlos con un ensayo de escalón.

## 1.3 Masa

El URDF exportado de SolidWorks suma **0,36 kg**, porque usa la densidad por defecto. Eso no es realista para un robot con chasis de aluminio, batería de 2000 mAh, Raspberry Pi 5 y LiDAR. Se usa **1,30 kg** (SUP), repartidos en 1,18 kg de chasis y 4 × 0,03 kg de ruedas. **Hay que pesar el robot.**

## 1.4 Tópicos ROS 2 del robot real

Confirmados en las cadenas del firmware y en los tutoriales:

| Tópico | Tipo | Publica |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | consumido por el ESP32 |
| `odom_raw` | `nav_msgs/Odometry` | ESP32 (encoders) |
| `imu` | `sensor_msgs/Imu` | ESP32 |
| `scan` | `sensor_msgs/LaserScan` | ESP32 (LiDAR MS200 por UART) |
| `battery`, `beep`, `servo_s1`, `servo_s2` | | ESP32 |
| `imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` (Raspberry Pi) |
| `odom` | `nav_msgs/Odometry` | `robot_localization` EKF (Raspberry Pi) |

En multirrobot, cada ESP32 recibe un `ROS_NAMESPACE` (`robot1`, `robot2`…) y todos comparten el mismo `ROS_DOMAIN_ID`. El gemelo digital reproduce esta misma interfaz con los namespaces `sheep1…sheepN` y `wolf`.

## 1.5 Lista de verificación en el robot físico

1. Medir el diámetro de la rueda con un calibre.
2. Pesar el robot completo.
3. Leer `PWM_MOTOR_MAX_VALUE` y `MOTOR_WHEEL_CIRCLE` del código del ESP32, si se consigue el proyecto ESP-IDF.
4. Hacer un ensayo de escalón de una rueda en vacío y con carga para identificar *K* y τ (ver `docs/04_motor_y_control.md`).
5. Calibrar el giro con `calibrate_angular` de Yahboom para obtener χ real (ver `docs/05_odometria.md`).
