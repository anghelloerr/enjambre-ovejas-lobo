%% M01 - Cinematica del Yahboom MicroROS-Pi5 (4WD, skid-steer)
% Calcula y verifica:
%   1. Cinematica inversa/directa del firmware (Motion_Ctrl / Motion_Get_Speed)
%   2. Factor de trocha efectiva chi que asume el firmware
%   3. Region alcanzable (v, w) con el limite de 1 m/s por rueda
%   4. Radio de giro minimo a cada velocidad
% Compatible con MATLAB R2021a+ y GNU Octave 8 (sin toolboxes).
clear; clc; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
P = params_microros();
figdir = fullfile(here, '..', 'docs', 'figures'); if ~exist(figdir, 'dir'), mkdir(figdir); end

%% 1. Matrices de cinematica (orden de motores: M1 izq-del, M2 izq-tras, M3 der-del, M4 der-tras)
APB = P.geo.APB;
J_inv = [1 -APB;            % v_wheel = J_inv * [v; w]
         1 -APB;
         1  APB;
         1  APB];
J_dir = [ 1/4      1/4      1/4      1/4;      % [v; w] = J_dir * v_wheel
         -1/(4*APB) -1/(4*APB) 1/(4*APB) 1/(4*APB)];
fprintf('J_dir * J_inv = \n'); disp(J_dir * J_inv);     % debe ser la identidad 2x2
assert(norm(J_dir*J_inv - eye(2)) < 1e-12, 'La cinematica directa no invierte a la inversa');

%% 2. Factor de trocha efectiva
% Para un diferencial ideal w = (vR - vL)/W. El firmware usa w = (vR - vL)/(2*APB).
chi_fw = 2*APB / P.geo.W;
fprintf('Trocha W = %.4f m, 2*APB = %.4f m  =>  chi_firmware = %.3f\n', P.geo.W, 2*APB, chi_fw);
fprintf('Si el robot real gira con chi = %.2f, la odometria de ruedas sobrestima\n', P.skid.chi);
fprintf('el giro en un factor chi_fw/chi = %.4f (%.1f %%)\n', chi_fw/P.skid.chi, 100*(chi_fw/P.skid.chi-1));

%% 3. Region alcanzable con |v_rueda| <= v_max
vmax = P.ctl.v_max; wfw = P.ctl.w_max;
w_lim = vmax/APB;                         % giro sobre si mismo a v = 0
fprintf('w maximo por limite de ruedas = %.2f rad/s (firmware limita a %.1f rad/s)\n', w_lim, wfw);
v = linspace(-vmax, vmax, 401);
w_top = min((vmax - abs(v))/APB, wfw);
figure('Color', 'w', 'Position', [100 100 560 420]);
fill([v fliplr(v)], [w_top -fliplr(w_top)], [0.75 0.85 1.0], 'EdgeColor', [0.1 0.3 0.7]); hold on;
plot([0.2 0.2], [-wfw wfw], 'k--');
text(0.21, 4.2, 'v ovejas/lobo', 'FontSize', 8);
xlabel('v [m/s]'); ylabel('\omega [rad/s]'); grid on; axis([-1.1 1.1 -6 6]);
title('Region (v, \omega) alcanzable: |v \pm \omega APB| \leq 1 m/s');
print(gcf, fullfile(figdir, 'm01_region_v_w.png'), '-dpng', '-r150');

%% 4. Ejemplo numerico (comprobacion a mano en docs/03_cinematica.md)
cmd = [0.20; 1.00];
vw = J_inv * cmd;
fprintf('\ncmd_vel v=0.20 m/s, w=1.00 rad/s -> ruedas [m/s]: %s\n', mat2str(vw', 4));
rpm = vw / P.geo.r * 60/(2*pi);
fprintf('                                -> ruedas [rpm]: %s\n', mat2str(rpm', 4));
cnt = vw / (2*pi*P.geo.r) * P.mot.cpr * P.ctl.Ts;
fprintf('                                -> cuentas/10 ms: %s\n', mat2str(cnt', 4));
res_v = 2*pi*P.geo.r / P.mot.cpr / P.ctl.Ts;
fprintf('Resolucion de velocidad del encoder en 10 ms: %.4f m/s por cuenta\n', res_v);
