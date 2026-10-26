# 7. Algoritmo ovejas–lobo

Código:
- `ros2_ws/src/swarm_sheep_wolf/swarm_sheep_wolf/behaviors.py`: ley de control (Python puro, sin ROS). La usan los nodos ROS 2 y el simulador de Python.
- `matlab/sw_*.m`: la misma ley en MATLAB.
- `agent_node.py`: un nodo por robot (control descentralizado).
- `metrics_node.py`: registro de métricas.
- `test/test_behaviors.py`: 13 pruebas unitarias.

## 7.1 Arquitectura

Cada robot ejecuta su propio `agent_node` a 20 Hz. El nodo lee la pose de todos los robots, calcula una **velocidad deseada en el plano** $\mathbf u_i$ y la convierte en `cmd_vel` (*v*, ω) para un robot no holónomo.

| Fuente de pose (`pose_source`) | De dónde sale | Equivalente en el laboratorio |
| --- | --- | --- |
| `ground_truth` | `OdometryPublisher` de Gazebo | Captura de movimiento (OptiTrack, cámara cenital) |
| `tf` | EKF de cada robot, TF `world → ns/base_footprint` | Odometría a bordo, compartida por DDS |

## 7.2 Ovejas: reglas de Reynolds + huida

Para la oveja *i* en $\mathbf p_i$, con vecinos $\mathcal N_i=\{j: \|\mathbf p_j-\mathbf p_i\|<r_{nb}\}$ y cercanos $\mathcal S_i=\{j: \|\mathbf p_j-\mathbf p_i\|<r_{sep}\}$:

| Regla | Expresión |
| --- | --- |
| Separación | $\mathbf s_i=-\dfrac{1}{\lvert\mathcal S_i\rvert}\displaystyle\sum_{j\in\mathcal S_i}\hat{\mathbf e}_{ij}\left(\frac{r_{sep}}{d_{ij}}-1\right)$, con $d_{ij}=\|\mathbf p_j-\mathbf p_i\|$ y $\hat{\mathbf e}_{ij}=(\mathbf p_j-\mathbf p_i)/d_{ij}$ |
| Cohesión | $\mathbf c_i=\overline{\mathbf p}_{\mathcal N_i}-\mathbf p_i$ |
| Alineación | $\mathbf a_i=\overline{\mathbf v}_{\mathcal N_i}$ |
| Miedo | $f_i=\max\!\left(0,\,1-\dfrac{\|\mathbf p_i-\mathbf p_w\|}{r_{fear}}\right)\in[0,1]$ |
| Huida | $\mathbf h_i=f_i\,\dfrac{\mathbf p_i-\mathbf p_w}{\|\mathbf p_i-\mathbf p_w\|}$ |
| Paredes | $\mathbf b_i$: repulsión lineal desde 0 (a `wall_margin` = 0,4 m de la pared) hasta 1 (sobre la pared) |

$$\mathbf u_i=w_{sep}\mathbf s_i+w_{coh}(1+f_i)\,\mathbf c_i+w_{ali}\mathbf a_i+w_{flee}\mathbf h_i+w_{wall}\mathbf b_i$$

El término $(1+f_i)$ hace que el rebaño **se agrupe más cuando siente al lobo**, como las ovejas reales.

Velocidad máxima: $v_{cap}=v_{sheep}(0{,}35+0{,}65f_i)$. Sin amenaza pastan a 0,07 m/s; con el lobo encima corren a 0,20 m/s.

La separación es **continua** (se anula en $d=r_{sep}$) y está **promediada** sobre los vecinos cercanos. Las dos cosas son necesarias para que el rumbo no oscile (sección 7.6).

## 7.3 Lobo pastor (Strömbom et al., 2014)

> D. Strömbom et al., "Solving the shepherding problem: heuristics for herding autonomous, interacting agents", *J. R. Soc. Interface* 11: 20140719, 2014.

Sea $\mathbf g$ el centroide del rebaño, $\mathbf G$ el corral, $d_{max}=\max_i\|\mathbf p_i-\mathbf g\|$ y $f_N=0{,}45\,N^{2/3}$ (≈ 1,32 m con *N* = 5). El lado abierto del corral es $\hat{\mathbf o}=-\mathbf G/\|\mathbf G\|$, la dirección hacia el centro de la arena.

| Modo | Condición | Punto objetivo del lobo |
| --- | --- | --- |
| **Recolectar** | $d_{max}>f_N$ | Detrás de la oveja más alejada: $\mathbf p_j+0{,}45\,\widehat{(\mathbf p_j-\mathbf g)}$ |
| **Conducir** | rebaño compacto | $\mathbf g+\hat{\mathbf d}\,(d_{max}+0{,}45)$, con $\hat{\mathbf d}=\widehat{\mathbf g-\mathbf G}$ si $(\mathbf g-\mathbf G)\cdot\hat{\mathbf o}>0{,}5\,r_G$, y si no $\hat{\mathbf d}=\hat{\mathbf o}$ |
| **Vigilar** | todas las ovejas dentro del corral | $\mathbf G+\hat{\mathbf o}\,(r_G+0{,}45)$ |

Además, el objetivo se limita a la arena con 0,25 m de margen, y el lobo se aparta de las ovejas a menos de 0,4 m (no atraviesa el rebaño). Su velocidad se satura en $v_{wolf}$ = 0,25 m/s.

**Por qué hacen falta la vigilancia y el lado abierto.** En la primera ejecución en Gazebo, el lobo se paraba al llegar al corral. Las ovejas seguían pastando, una salía, y la dirección $\widehat{\mathbf g-\mathbf G}$ apuntaba a la esquina porque el centroide estaba casi sobre el corral. El lobo cruzaba el corral hacia la esquina y dispersaba al rebaño: llegó a quedar el 40 % dentro y hubo un contacto. Con las dos reglas, el lobo siempre empuja desde el lado abierto y se queda de guardia (sección 8.3).

## 7.4 Lobo cazador (`wolf_mode:=hunt`)

Persecución pura de la oveja más cercana a $v_{wolf}$. Sirve como prueba de estrés: las ovejas huyen al máximo y se comprueba que no chocan entre sí.

## 7.5 De velocidad plana a `cmd_vel`

Con $e=\operatorname{atan2}(u_y,u_x)-\theta$ envuelto a $(-\pi,\pi]$:

$$v=\|\mathbf u\|\max(\cos e,0),\qquad \omega=k_h\,e\,\min\!\left(1,\frac{\|\mathbf u\|}{v_{ref}}\right),\qquad |v|\le1\ \text{m/s},\ |\omega|\le5\ \text{rad/s}$$

y $v=\omega=0$ si $\|\mathbf u\|<v_{dead}$, con $v_{dead}$ = 0,02 m/s y $v_{ref}$ = 0,05 m/s.

Si el error de rumbo pasa de 90°, el robot gira en el sitio antes de avanzar. La excepción son los ajustes lentos ($\|\mathbf u\|<v_{ref}$) con el objetivo detrás: ahí el robot **retrocede** ($v<0$, con $e$ desplazado π) en vez de dar media vuelta. Los límites son los del firmware (docs/03).

## 7.6 Oscilación del rumbo: diagnóstico y corrección

La primera versión completa funcionaba (100 % al corral, sin choques), pero las ovejas **castañeteaban**: el rumbo oscilaba ±20–40° varias veces por segundo, con ω saturado en ±5 rad/s. La métrica *giro medio* ($\sum|\Delta\theta|/T$) lo detecta. En un robot real eso gasta batería, calienta los motores y, en un skid-steer, cada giro arrastra las ruedas lateralmente. Hubo tres causas, y cada una se aisló con un experimento:

| # | Causa | Cómo se aisló | Corrección |
| --- | --- | --- | --- |
| 1 | **Fuerza discontinua.** La separación original, $\propto r_{sep}/d^2$, valía ≈ 1 justo dentro de $r_{sep}$ y 0 justo fuera. Las ovejas se quedaban en esa frontera, donde la cohesión la equilibra, y la dirección deseada cambiaba de lado cada 50 ms | Python: con $w_{sep}=0$ o $w_{coh}=0$ el giro bajaba de 196°/s a 1–4°/s | Fuerza continua $(r_{sep}/d-1)$, nula en el borde: 196 → 10°/s |
| 2 | **Dirección indefinida cerca de $\mathbf u=0$.** En equilibrio, milímetros de ruido invierten $\mathbf u$ y el uniciclo gira 180° | Python con poses del EKF: 128°/s | Zona muerta $v_{dead}$, ω escalado por $\|\mathbf u\|/v_{ref}$ y marcha atrás en ajustes lentos: 128 → 12°/s |
| 3 | **Rigidez × retardo.** Con la suma sobre vecinos, una oveja rodeada por 4 en el corral siente una rigidez de ≈ 4·w<sub>sep</sub>·r<sub>sep</sub>/d² ≈ 32 s⁻¹. Con ≈ 100 ms de retardo entre `cmd_vel` y las ruedas (Gazebo, DDS), *k·T* > 2 y aparece un ciclo límite | Se reprodujo en Python añadiendo un retardo de actuación: 7°/s con 0 ms, 31°/s con 50 ms y **148°/s con 100 ms**, como en Gazebo | Promedio en lugar de suma: rigidez independiente del número de vecinos. Con 100 ms: 148 → 30°/s |

Hubo además una causa menor en ROS 2: el `ground_truth` de Gazebo da hasta 0,08 m/s de velocidad lineal aparente cuando el robot gira en el sitio (cabeceo del skid-steer). Esa velocidad entraba en la regla de alineación de los vecinos. Ahora `agent_node` estima la velocidad de cada robot por diferencias de posición filtradas (τ = 0,5 s).

Resultado en Gazebo, fase de vigilancia: el giro medio de las ovejas pasó de ≈ 180°/s a ≈ 3°/s (docs/08).

La lección de control es general: un comportamiento de enjambre es un **sistema realimentado con retardo**, y sus "fuerzas virtuales" tienen que ser continuas y con una rigidez acotada, igual que cualquier lazo de control. Por eso el simulador de Python incluye ahora un retardo de actuación de 100 ms (`CMD_DELAY`) además del lazo de motor de 25 ms.

## 7.7 Parámetros

| Parámetro | Valor | Significado |
| --- | --- | --- |
| $r_{sep}$ | 0,45 m | ≈ 2 veces el diámetro del robot (2 × 0,11 m) |
| $r_{nb}$ | 1,0 m | Radio de vecindad |
| $r_{fear}$ | 1,2 m | Radio de percepción del lobo |
| $w_{sep}, w_{coh}, w_{ali}$ | 3,0 / 0,6 / 0,4 | Pesos de Reynolds |
| $w_{flee}, w_{wall}$ | 2,0 / 2,0 | Huida y paredes |
| $v_{sheep}, v_{wolf}$ | 0,20 / 0,25 m/s | El lobo es algo más rápido |
| $k_h$ | 2,5 s⁻¹ | Ganancia de rumbo |
| $v_{dead}$, $v_{ref}$ | 0,02 / 0,05 m/s | Zona muerta y velocidad de referencia del giro |
| Arena | 4 × 4 m | Centrada en el origen |
| Corral | centro (1,2, 1,2), radio 0,6 m | |

Los mismos valores están en `config/swarm.yaml`, `python/params.py` y `matlab/params_microros.m`.

## 7.8 Métricas

| Métrica | Definición |
| --- | --- |
| Cohesión | Distancia media de las ovejas a su centroide [m] |
| Polarización | $\left\lvert\frac1N\sum_i e^{j\theta_i}\right\rvert\in[0,1]$ |
| Distancia mínima | Mínima distancia entre centros de cualquier par de robots |
| Colisión | Evento de contacto: distancia < 0,20 m (0,9 × 2 × radio circunscrito). Se cuenta una vez por contacto |
| Tiempo al corral | Primer instante con todas las ovejas a menos de $r_G$ del corral |
| Contención | Fracción media de ovejas dentro del corral durante los 60 s siguientes |
| Error de odometría | Distancia media entre la pose del EKF y el ground truth |
| Giro medio | Media de $\sum|\Delta\theta|/T$ de las ovejas [°/s]. Detecta oscilaciones del rumbo |
