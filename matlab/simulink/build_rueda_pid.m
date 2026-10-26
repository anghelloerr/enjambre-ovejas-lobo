%% Construye y simula en Simulink el lazo de velocidad de UNA rueda (ESP32, Ts = 10 ms)
% Modelo generado por script: microros_rueda_pid.slx
%   referencia -> PI discreto (saturado, anti-windup) -> motor 310 + reductora + carga
%   -> encoder (cuantizador 1040 cpr + retardo de una muestra) -> realimentacion
% Requiere: MATLAB + Simulink. Las ganancias salen del diseno de m02_motor_pid.m.
% Uso:  cd matlab/simulink; build_rueda_pid
clear; clc;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here, '..'));
P = params_microros(); M = P.mot; G = P.geo; Ts = P.ctl.Ts;

% --- Planta (misma que M02) ----------------------------------------------------------
J = M.Jm + (P.mass.total/4)*G.r^2/M.N^2 + 0.5*P.mass.wheel*G.r^2/M.N^2;
num = M.Vbat * M.Kt * G.r / M.N;                               % duty -> v rueda [m/s]
den = conv([M.La M.R], [J M.bm]); den(end) = den(end) + M.Kt*M.Ke;
tau_m = J*M.R/(M.Kt*M.Ke + M.R*M.bm); Kdc = num/den(end);
tau_cl = 0.040; Kp = tau_m/(Kdc*tau_cl); Ki = Kp/tau_m;
res_v = 2*pi*G.r/M.cpr/Ts;                                    % m/s por cuenta en 10 ms

% --- Modelo ----------------------------------------------------------------------------
mdl = 'microros_rueda_pid';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'StopTime', '1.0', 'Solver', 'ode45', 'MaxStep', '1e-4');

add_block('simulink/Sources/Step', [mdl '/Referencia'], 'Time', '0.1', 'After', '0.5', ...
          'Position', [30 100 60 130]);
add_block('simulink/Math Operations/Sum', [mdl '/Error'], 'Inputs', '+-', ...
          'Position', [110 105 130 125]);
add_block('simulink/Discrete/Discrete PID Controller', [mdl '/PI ESP32'], ...
          'Controller', 'PI', 'P', num2str(Kp), 'I', num2str(Ki), 'SampleTime', num2str(Ts), ...
          'LimitOutput', 'on', 'UpperSaturationLimit', '1', 'LowerSaturationLimit', '-1', ...
          'AntiWindupMode', 'clamping', 'Position', [170 95 240 135]);
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Motor 310 + carga'], ...
          'Numerator', mat2str(num), 'Denominator', mat2str(den), 'Position', [290 95 400 135]);
add_block('simulink/Discontinuities/Quantizer', [mdl '/Encoder 1040 cpr'], ...
          'QuantizationInterval', num2str(res_v), 'Position', [450 180 520 210]);
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Muestreo'], 'SampleTime', num2str(Ts), ...
          'Position', [560 180 600 210]);
add_block('simulink/Discrete/Unit Delay', [mdl '/Retardo medida'], 'SampleTime', num2str(Ts), ...
          'Orientation', 'left', 'Position', [300 180 340 210]);
add_block('simulink/Sinks/Scope', [mdl '/Scope'], 'NumInputPorts', '2', 'Position', [520 60 550 140]);
add_block('simulink/Sinks/To Workspace', [mdl '/v_rueda'], 'VariableName', 'v_rueda', ...
          'SaveFormat', 'Timeseries', 'Position', [520 20 580 40]);
add_block('simulink/Sinks/To Workspace', [mdl '/duty'], 'VariableName', 'duty', ...
          'SaveFormat', 'Timeseries', 'Position', [290 20 350 40]);

add_line(mdl, 'Referencia/1', 'Error/1', 'autorouting', 'on');
add_line(mdl, 'Error/1', 'PI ESP32/1', 'autorouting', 'on');
add_line(mdl, 'PI ESP32/1', 'Motor 310 + carga/1', 'autorouting', 'on');
add_line(mdl, 'PI ESP32/1', 'duty/1', 'autorouting', 'on');
add_line(mdl, 'Motor 310 + carga/1', 'Encoder 1040 cpr/1', 'autorouting', 'on');
add_line(mdl, 'Motor 310 + carga/1', 'Scope/1', 'autorouting', 'on');
add_line(mdl, 'Motor 310 + carga/1', 'v_rueda/1', 'autorouting', 'on');
add_line(mdl, 'Referencia/1', 'Scope/2', 'autorouting', 'on');
add_line(mdl, 'Encoder 1040 cpr/1', 'Muestreo/1', 'autorouting', 'on');
add_line(mdl, 'Muestreo/1', 'Retardo medida/1', 'autorouting', 'on');
add_line(mdl, 'Retardo medida/1', 'Error/2', 'autorouting', 'on');
save_system(mdl, fullfile(here, [mdl '.slx']));

% --- Simulacion y figura -------------------------------------------------------------------
out = sim(mdl);
v = out.v_rueda; d = out.duty;
figure('Color', 'w');
subplot(2,1,1); plot(v.Time, v.Data, 'LineWidth', 1.4); hold on;
plot([0 0.1 0.1 1], [0 0 0.5 0.5], 'k--'); grid on; ylabel('v rueda [m/s]');
title(sprintf('Simulink: PI Kp=%.3f, Ki=%.1f, Ts=10 ms', Kp, Ki));
subplot(2,1,2); stairs(d.Time, d.Data); grid on; ylabel('duty'); xlabel('t [s]');
print(gcf, fullfile(here, '..', '..', 'docs', 'figures', 'simulink_rueda_pid.png'), '-dpng', '-r150');
fprintf('Modelo guardado en %s\n', fullfile(here, [mdl '.slx']));
