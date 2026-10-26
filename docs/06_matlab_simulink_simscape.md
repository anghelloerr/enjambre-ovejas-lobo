# 6. MATLAB, Simulink y Simscape

Todos los scripts toman los parámetros de `matlab/params_microros.m`.

| Script | Qué hace | Verificado |
| --- | --- | --- |
| `m01_cinematica.m` | Matrices de cinemática, χ, región (v, ω), ejemplo resuelto | ✔ GNU Octave 8.4 |
| `m02_motor_pid.m` | Modelo del motor, diseño PI, márgenes, análisis del PID de Yahboom, simulación no lineal | ✔ Octave 8.4 + `control` |
| `m03_odometria.m` | Error sistemático del skid-steer, Monte Carlo de 20 ensayos, deriva del giroscopo | ✔ Octave 8.4 |
| `m04_ovejas_lobo.m` | Algoritmo ovejas–lobo (pastoreo + 60 s de vigilancia) con la misma ley de control que ROS 2 | ✔ Octave 8.4 |
| `simulink/build_rueda_pid.m` | Genera `microros_rueda_pid.slx`: PI discreto + motor + encoder cuantizado | ✘ Requiere Simulink |
| `simulink/build_robot_odometria.m` | Genera `microros_robot_odometria.slx`: cinemática, 4 lazos, skid-steer, 2 odometrías | ✘ Requiere Simulink |
| `simscape/build_simscape_motor.m` | Genera `microros_motor_simscape.slx`: red eléctrica + mecánica (Foundation Library) | ✘ Requiere Simscape |
| `simscape/import_multibody.m` | `smimport` del URDF y actuación de las 4 ruedas | ✘ Requiere Simscape Multibody |

> Los modelos de Simulink y Simscape **se construyen por script**: no hay `.slx` en el repositorio. No se pudieron ejecutar aquí porque no hay licencia de MATLAB. Las ecuaciones y números que usan son los mismos de `m02` y `m03`, que sí están verificados en Octave. Si un nombre de bloque o de puerto cambia en tu versión de MATLAB, `build_simscape_motor.m` indica exactamente qué conexión hay que hacer a mano.

## Orden de ejecución recomendado

`run_all` ejecuta los cuatro scripts verificados seguidos (MATLAB u Octave). Uno a uno:

```matlab
cd matlab
m01_cinematica          % figuras en docs/figures/m01_*
m02_motor_pid           % m02_*
m03_odometria
m04_ovejas_lobo
cd simulink
build_rueda_pid         % crea y simula microros_rueda_pid.slx
build_robot_odometria   % crea y simula microros_robot_odometria.slx
cd ../simscape
build_simscape_motor    % crea y simula microros_motor_simscape.slx
import_multibody        % crea microros_multibody.slx (Mechanics Explorer)
```

## Resultados esperados al ejecutarlos en MATLAB

Estos son los valores que debe reproducir cada modelo. Sirven para validar la ejecución.

- **`build_rueda_pid`:** escalón de 0,5 m/s con subida de unos 50 ms, sin sobreimpulso y con un rizado de ±0,0145 m/s por la cuantización del encoder.
- **`build_robot_odometria`:** con χ = 1,55 y sin ruido, el error final del cuadrado con solo encoders debe ser del orden de 1 m (≈ 100° de rumbo acumulado). Con encoders + giroscopo debe quedar en torno a 0,1 m, que es la deriva del sesgo de 0,002 rad/s durante 86 s (≈ 10°).
- **`build_simscape_motor`:** la misma respuesta que `build_rueda_pid`. La diferencia es que aquí la corriente y el par salen del modelo físico.

## Simscape Multibody: siguiente paso para tracción real

`import_multibody.m` hace que las ruedas giren con movimiento prescrito. Sirve para visualizar el modelo y comprobar la cinemática. Para simular la tracción y el derrape, como en Gazebo, falta añadir:

1. Un bloque `World Frame` con un plano de suelo (`Brick Solid` grande y fijo).
2. Un `Spatial Contact Force` por rueda, entre el cilindro de la rueda y el plano, con μ<sub>s</sub> = μ<sub>k</sub> = 1,0, que es lo calibrado en Gazebo.
3. Juntas de rueda en modo de par (`Torque: Provided by Input`), con el motor de `build_simscape_motor` conectado a cada una.

Con ese modelo, el giro en el sitio debería dar χ ≈ 1,5, igual que la teoría y Gazebo (docs/03, sección 3.2).
