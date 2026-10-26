%% M02 - Modelo del motor 310 y diseno del lazo de velocidad (ESP32, Ts = 10 ms)
% 1. Modelo electromecanico del motor DC + reductora + carga del robot
% 2. Reduccion a primer orden y discretizacion ZOH
% 3. Diseno de un PI por cancelacion de polo (tau_cl = 40 ms) y margenes
% 4. Analisis del PID incremental del firmware de Yahboom (Kp=1, Ki=0.2, Kd=0.2)
% 5. Simulacion discreta con saturacion PWM y cuantizacion del encoder
% Requiere: Control System Toolbox (MATLAB) o paquete control (Octave).
clear; clc; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
if exist('OCTAVE_VERSION', 'builtin'), pkg load control; end
P = params_microros();
figdir = fullfile(here, '..', 'docs', 'figures'); if ~exist(figdir, 'dir'), mkdir(figdir); end
M = P.mot; G = P.geo; Ts = P.ctl.Ts;

%% 1. Inercia equivalente vista desde el eje del motor
% Cada motor mueve 1/4 de la masa del robot a traves de la rueda (radio r) y la reductora (N).
J_rob   = (P.mass.total/4) * G.r^2 / M.N^2;           % traslacion del robot
J_wheel = 0.5 * P.mass.wheel * G.r^2 / M.N^2;          % rueda
J       = M.Jm + J_rob + J_wheel;
fprintf('Inercias en eje motor [kg m^2]: rotor %.2e, robot %.2e, rueda %.2e -> J = %.2e\n', ...
        M.Jm, J_rob, J_wheel, J);

% w_m(s)/V(s) = Kt / ((La s + R)(J s + b) + Kt Ke)
s = tf('s');
Gm = M.Kt / ((M.La*s + M.R)*(J*s + M.bm) + M.Kt*M.Ke);
tau_e = M.La/M.R;
tau_m = J*M.R/(M.Kt*M.Ke + M.R*M.bm);
fprintf('Constante electrica tau_e = %.2f ms, mecanica tau_m = %.2f ms\n', 1e3*tau_e, 1e3*tau_m);

% Planta de control: duty d en [-1, 1] -> velocidad lineal de la rueda [m/s]
Gp = M.Vbat * Gm * G.r / M.N;
Kdc = dcgain(Gp);
fprintf('Ganancia estatica: %.3f m/s por unidad de duty (velocidad en vacio con carga)\n', Kdc);

%% 2. Aproximacion de primer orden (tau_e << tau_m) y discretizacion
G1 = Kdc / (tau_m*s + 1);
G1d = c2d(G1, Ts, 'zoh');
Gpd = c2d(Gp, Ts, 'zoh');
fprintf('Polo discreto del primer orden: z = %.4f\n', exp(-Ts/tau_m));

%% 3. Diseno PI por cancelacion de polo
% C(s) = Kp (1 + 1/(Ti s)), Ti = tau_m  ->  L(s) = Kp Kdc / (tau_m s)
% Lazo cerrado 1/(tau_cl s + 1) con tau_cl = tau_m / (Kp Kdc)
tau_cl = 0.040;
Kp = tau_m / (Kdc * tau_cl);
Ki = Kp / tau_m;
fprintf('\nPI de diseno: Kp = %.4f [duty/(m/s)], Ki = %.3f [duty/(m s^-1 s)]\n', Kp, Ki);
C  = Kp + Ki/s;
Cd = c2d(C, Ts, 'tustin');
% Retardo de una muestra: el encoder mide la velocidad media del periodo anterior
z = tf('z', Ts);
Ld = Cd * Gpd * z^-1;
[gm_db, pm, wcp, w180] = loop_margins(Ld, Ts);
fprintf('Margen de ganancia %.1f dB (w = %.1f rad/s), margen de fase %.1f grados (w_c = %.1f rad/s)\n', ...
        gm_db, w180, pm, wcp);
T_cl = feedback(Cd*Gpd, z^-1);
[ys, ts] = step(T_cl, 0.5);
info = step_metrics(ys, ts);
fprintf('Lazo cerrado (con retardo): t_subida %.0f ms, sobreimpulso %.1f %%, t_establec. %.0f ms\n', ...
        1e3*info.RiseTime, info.Overshoot, 1e3*info.SettlingTime);

%% 4. PID incremental del firmware de Yahboom
% du_k = Kp (e_k - e_{k-1}) + Ki e_k + Kd (e_k - 2 e_{k-1} + e_{k-2})
% e en cuentas de encoder por 10 ms; u en unidades PWM (+-PWM_MAX).
% PWM_MAX no aparece en los tutoriales: se barre un rango para ver su efecto.
cnt_per_mps = Ts * M.cpr / (2*pi*G.r);                 % cuentas/10 ms por m/s
fprintf('\n1 m/s = %.1f cuentas por periodo de 10 ms\n', cnt_per_mps);
Cfw = (P.ctl.Kp_fw*(1 - z^-1) + P.ctl.Ki_fw + P.ctl.Kd_fw*(1 - z^-1)^2) / (1 - z^-1);
pwm_list = [100 255 1000 3600];
figure('Color', 'w', 'Position', [100 100 640 420]); hold on;
for k = 1:numel(pwm_list)
    Gcnt = Gpd * cnt_per_mps / pwm_list(k);             % PWM -> cuentas/10 ms
    Tfw = feedback(Cfw*Gcnt, z^-1);
    p = pole(Tfw);
    [y, t] = step(Tfw, 1.5);
    plot(t, y, 'LineWidth', 1.4);
    fprintf('PWM_MAX = %4d: |polo| max = %.4f, valor final %.3f\n', pwm_list(k), max(abs(p)), y(end));
end
[y, t] = step(T_cl, 1.5);
plot(t, y, 'k--', 'LineWidth', 1.8);
legend([arrayfun(@(v) sprintf('Firmware, PWM_{MAX}=%d', v), pwm_list, 'UniformOutput', false), ...
        {'PI de diseno (\tau_{cl}=40 ms)'}], 'Location', 'southeast');
xlabel('t [s]'); ylabel('velocidad / referencia'); grid on;
title('Escalon de velocidad de una rueda (lazo discreto, Ts = 10 ms)');
print(gcf, fullfile(figdir, 'm02_escalon_pid.png'), '-dpng', '-r150');

%% 5. Simulacion no lineal: saturacion de duty y cuantizacion del encoder
N = round(1.0/Ts);
ref = 0.30*ones(1, N); ref(round(N/2):end) = 0.60;   % m/s
[A1, B1, C1, D1] = ssdata(Gpd);
x = zeros(size(A1,1), 1); u = 0; e_prev = 0; integ = 0; acc = 0;
v_meas = zeros(1, N); v_true = zeros(1, N); duty = zeros(1, N);
for k = 1:N
    % encoder: cuentas enteras acumuladas en el periodo anterior
    e = ref(k) - (k > 1)*v_meas(max(k-1, 1));
    integ = integ + Ki*Ts*e;
    u = Kp*e + integ;
    if abs(u) > 1, integ = integ - Ki*Ts*e; end       % anti-windup por integracion condicional
    u = max(min(u, 1), -1);
    duty(k) = u;
    y = C1*x + D1*u;  x = A1*x + B1*u;
    v_true(k) = y;
    acc = acc + y*cnt_per_mps;
    cnts = floor(acc);  acc = acc - cnts;
    v_meas(k) = cnts / cnt_per_mps;
end
t = (0:N-1)*Ts;
figure('Color', 'w', 'Position', [100 100 640 480]);
subplot(2,1,1); stairs(t, ref, 'k--'); hold on; plot(t, v_true, 'LineWidth', 1.4); stairs(t, v_meas, ':');
ylabel('v rueda [m/s]'); legend('referencia', 'real', 'medida (encoder)', 'Location', 'southeast'); grid on;
title('PI con saturacion y encoder de 1040 cuentas/vuelta');
subplot(2,1,2); stairs(t, duty, 'LineWidth', 1.2); ylabel('duty [-]'); xlabel('t [s]'); grid on;
print(gcf, fullfile(figdir, 'm02_pi_no_lineal.png'), '-dpng', '-r150');

% Constante de tiempo equivalente que usa el simulador de Python (robot_model.tau)
fprintf('\nConstante de tiempo equivalente del lazo cerrado: ~%.0f ms (python/robot_model.py usa 25 ms)\n', ...
        1e3*(info.RiseTime/2.2));
