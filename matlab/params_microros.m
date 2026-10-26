function P = params_microros()
%PARAMS_MICROROS Parametros del Yahboom MicroROS-Pi5 usados en todo el proyecto.
%   P = params_microros() devuelve una estructura con los parametros fisicos,
%   del motor, del controlador y del algoritmo ovejas-lobo.
%
%   Cada valor lleva su origen:
%     [URDF]  yahboomcar_description/urdf/MicroROS.urdf (Yahboom)
%     [FW]    firmware microROS_Robot_V2.0.0.bin / tutoriales ESP32 de Yahboom
%     [MESH]  medido sobre las mallas STL oficiales
%     [SUP]   supuesto de diseno: debe verificarse en el robot real
%
%   La misma tabla esta replicada en python/params.py y en
%   ros2_ws/src/microros_description/config/robot_params.yaml.

%% Geometria (URDF)
P.geo.L      = 0.0455 + 0.0495;   % [m]  distancia eje delantero-trasero  [URDF]
P.geo.W      = 2*0.0675;          % [m]  distancia rueda izq.-der.       [URDF]
P.geo.A      = P.geo.W/2;         % [m]  semi-trocha  (0.0675)
P.geo.B      = P.geo.L/2;         % [m]  semi-batalla (0.0475)
P.geo.APB    = P.geo.A + P.geo.B; % [m]  ROBOT_APB = 0.115                [FW]
P.geo.r      = 0.024;             % [m]  radio de rueda                   [MESH] (verificar con regla)
P.geo.body   = [0.165 0.133 0.0775]; % [m] caja envolvente del chasis    [MESH]
P.geo.R_safe = 0.11;              % [m]  radio circunscrito del robot (para colisiones)

%% Masa e inercia
% El URDF exportado de SolidWorks suma ~0.36 kg, lo que no es realista
% (bateria 7.4 V 2000 mAh + Pi 5 + LiDAR + aluminio). Se usa un valor supuesto.
P.mass.total = 1.30;              % [kg]                                  [SUP] pesar el robot
P.mass.wheel = 0.030;             % [kg] por rueda                        [SUP]
P.mass.body  = P.mass.total - 4*P.mass.wheel;
b = P.geo.body;
P.mass.Izz   = P.mass.body*(b(1)^2 + b(2)^2)/12;   % [kg m^2] caja solida

%% Motor 310 con reductora (FW + SUP)
P.mot.N      = 20;                % reduccion 1:20                        [FW]
P.mot.ppr    = 13;                % lineas del encoder Hall AB            [FW]
P.mot.cpr    = 4*P.mot.ppr*P.mot.N;  % 1040 cuentas por vuelta de rueda   [FW]
P.mot.Vbat   = 7.4;               % [V] bateria nominal                   [FW]
P.mot.R      = 2.5;               % [ohm] resistencia de armadura         [SUP]
P.mot.La     = 1.0e-3;            % [H]   inductancia                     [SUP]
P.mot.w_nl   = 400*2*pi/60;       % [rad/s] vel. en vacio a la salida     [SUP] (~400 rpm)
P.mot.Ke     = P.mot.Vbat/(P.mot.w_nl*P.mot.N); % [V s/rad] eje motor
P.mot.Kt     = P.mot.Ke;          % [N m/A] (SI: Kt = Ke)
P.mot.Jm     = 1.0e-7;            % [kg m^2] rotor                        [SUP]
P.mot.bm     = 1.0e-7;            % [N m s/rad] friccion viscosa rotor    [SUP]
P.mot.eta    = 0.80;              % eficiencia de la reductora            [SUP]

%% Lazo de velocidad del ESP32 (FW)
P.ctl.Ts      = 0.010;            % [s] periodo del PID (MOTOR_PID_PERIOD) [FW]
P.ctl.Kp_fw   = 1.0;              % ganancias por defecto del firmware      [FW]
P.ctl.Ki_fw   = 0.2;
P.ctl.Kd_fw   = 0.2;
P.ctl.v_max   = 1.0;              % [m/s] limite por rueda                  [FW]
P.ctl.w_max   = 5.0;              % [rad/s] limite angular                  [FW]

%% Sensores
P.imu.gyro_sigma = 0.005;         % [rad/s] ruido blanco del giroscopo     [SUP]
P.imu.gyro_bias  = 0.002;         % [rad/s] sesgo constante                [SUP]
P.enc.slip_sigma = 0.02;          % deslizamiento relativo por rueda       [SUP]
P.skid.chi       = 1.55;          % trocha efectiva real / W (skid-steer)  [SUP] (calibrar)

%% Algoritmo ovejas-lobo (diseno propio)
P.swarm.n_sheep   = 5;
P.swarm.r_sep     = 0.45;   % [m] radio de separacion
P.swarm.r_nb      = 1.00;   % [m] radio de vecindad (cohesion/alineacion)
P.swarm.r_fear    = 1.20;   % [m] radio de deteccion del lobo
P.swarm.w_sep     = 3.0;
P.swarm.w_coh     = 0.6;
P.swarm.w_ali     = 0.4;
P.swarm.w_flee    = 2.0;
P.swarm.w_wall    = 2.0;
P.swarm.v_sheep   = 0.20;   % [m/s] velocidad de crucero de las ovejas
P.swarm.v_wolf    = 0.25;   % [m/s]
P.swarm.k_heading = 2.5;    % [1/s] ganancia del controlador de rumbo
P.swarm.arena     = 4.0;    % [m] lado de la arena cuadrada
P.swarm.goal      = [1.2 1.2];  % [m] corral (modo pastoreo)
P.swarm.goal_r    = 0.6;    % [m]
end
