%% Modelo fisico (Simscape) del motor 310 + reductora + rueda + carga, con PI del ESP32
% Modelo generado por script: microros_motor_simscape.slx
%
% Red electrica:   fuente controlada (duty*Vbat) -> R -> L -> conversor electromecanico
% Red mecanica:    rotor (Jm, bm) -> reductora 1:20 -> rueda + 1/4 de la masa del robot
% Control:         PI discreto (mismas ganancias que M02) con realimentacion del sensor
%
% Solo usa la Simscape Foundation Library (no requiere Simscape Electrical).
% Los nombres de puertos de Simscape cambian poco entre versiones; si alguna
% conexion falla, el script indica cual es para hacerla a mano en el diagrama.
clear; clc;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here, '..'));
P = params_microros(); M = P.mot; G = P.geo; Ts = P.ctl.Ts;
J_load = (P.mass.total/4)*G.r^2 + 0.5*P.mass.wheel*G.r^2;   % en el eje de la rueda
J = M.Jm + J_load/M.N^2;
tau_m = J*M.R/(M.Kt*M.Ke + M.R*M.bm);
Kdc = M.Vbat*M.Kt/(M.R*M.bm + M.Kt*M.Ke) * G.r/M.N;
Kp = tau_m/(Kdc*0.040); Ki = Kp/tau_m;

mdl = 'microros_motor_simscape';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'StopTime', '1.0', 'Solver', 'ode23t', 'MaxStep', '1e-4');

b = @(name) [mdl '/' name];
add_block('nesl_utility/Solver Configuration', b('Solver'), 'Position', [40 300 80 330]);
add_block('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source', b('Puente H'), 'Position', [180 200 220 260]);
add_block('fl_lib/Electrical/Electrical Elements/Resistor', b('Ra'), 'R', num2str(M.R), 'Position', [260 150 300 170]);
add_block('fl_lib/Electrical/Electrical Elements/Inductor', b('La'), 'l', num2str(M.La), 'Position', [340 150 380 170]);
add_block('fl_lib/Electrical/Electrical Elements/Rotational Electromechanical Converter', b('Motor Kt'), ...
          'K', num2str(M.Kt), 'Position', [420 190 470 250]);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', b('GND'), 'Position', [190 300 210 320]);
add_block('fl_lib/Mechanical/Rotational Elements/Inertia', b('Rotor Jm'), 'inertia', num2str(M.Jm), 'Position', [520 120 560 150]);
add_block('fl_lib/Mechanical/Rotational Elements/Rotational Damper', b('Friccion bm'), 'D', num2str(M.bm), 'Position', [520 260 560 290]);
add_block('fl_lib/Mechanical/Mechanisms/Gear Box', b('Reductora 1:20'), 'ratio', num2str(M.N), 'Position', [600 190 650 230]);
add_block('fl_lib/Mechanical/Rotational Elements/Inertia', b('Rueda + robot/4'), 'inertia', num2str(J_load), 'Position', [720 120 760 150]);
add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference', b('Chasis'), 'Position', [520 330 560 350]);
add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor', b('Encoder'), 'Position', [700 220 750 270]);

add_block('simulink/Sources/Step', b('Referencia v'), 'Time', '0.1', 'After', '0.5', 'Position', [20 40 50 70]);
add_block('simulink/Math Operations/Sum', b('Error'), 'Inputs', '+-', 'Position', [80 45 100 65]);
add_block('simulink/Discrete/Discrete PID Controller', b('PI ESP32'), 'Controller', 'PI', ...
          'P', num2str(Kp), 'I', num2str(Ki), 'SampleTime', num2str(Ts), 'LimitOutput', 'on', ...
          'UpperSaturationLimit', '1', 'LowerSaturationLimit', '-1', 'AntiWindupMode', 'clamping', ...
          'Position', [130 35 190 75]);
add_block('simulink/Math Operations/Gain', b('Vbat'), 'Gain', num2str(M.Vbat), 'Position', [220 40 250 70]);
add_block('nesl_utility/Simulink-PS Converter', b('S->PS'), 'Position', [280 45 300 65]);
add_block('nesl_utility/PS-Simulink Converter', b('PS->S'), 'Position', [790 230 810 250]);
add_block('simulink/Math Operations/Gain', b('w a v'), 'Gain', num2str(G.r), 'Position', [830 225 860 255]);
add_block('simulink/Discrete/Zero-Order Hold', b('Muestreo'), 'SampleTime', num2str(Ts), 'Position', [880 225 910 255]);
add_block('simulink/Sinks/Scope', b('Scope'), 'Position', [950 225 980 255]);
add_block('simulink/Sinks/To Workspace', b('v_rueda'), 'VariableName', 'v_rueda', 'SaveFormat', 'Timeseries', 'Position', [950 280 1010 300]);

% --- Conexiones Simulink -----------------------------------------------------------------
add_line(mdl, 'Referencia v/1', 'Error/1', 'autorouting', 'on');
add_line(mdl, 'Error/1', 'PI ESP32/1', 'autorouting', 'on');
add_line(mdl, 'PI ESP32/1', 'Vbat/1', 'autorouting', 'on');
add_line(mdl, 'Vbat/1', 'S->PS/1', 'autorouting', 'on');
add_line(mdl, 'PS->S/1', 'w a v/1', 'autorouting', 'on');
add_line(mdl, 'w a v/1', 'Muestreo/1', 'autorouting', 'on');
add_line(mdl, 'Muestreo/1', 'Scope/1', 'autorouting', 'on');
add_line(mdl, 'Muestreo/1', 'v_rueda/1', 'autorouting', 'on');
add_line(mdl, 'Muestreo/1', 'Error/2', 'autorouting', 'on');

% --- Conexiones fisicas (puertos de conservacion) --------------------------------------------
% ph(blk) devuelve los PortHandles; LConn/RConn numerados como aparecen en el bloque.
ph = @(name) get_param(b(name), 'PortHandles');
conn = { ...
  'S->PS',          'RConn', 1, 'Puente H',        'RConn', 1;   % senal fisica de tension
  'Puente H',       'LConn', 1, 'Ra',              'LConn', 1;   % + de la fuente -> Ra
  'Ra',             'RConn', 1, 'La',              'LConn', 1;
  'La',             'RConn', 1, 'Motor Kt',        'LConn', 1;   % + del motor
  'Motor Kt',       'LConn', 2, 'GND',             'LConn', 1;   % - del motor
  'Puente H',       'LConn', 2, 'GND',             'LConn', 1;   % - de la fuente
  'Solver',         'RConn', 1, 'GND',             'LConn', 1;
  'Motor Kt',       'RConn', 1, 'Rotor Jm',        'LConn', 1;   % eje (C)
  'Motor Kt',       'RConn', 2, 'Chasis',          'LConn', 1;   % carcasa (R)
  'Motor Kt',       'RConn', 1, 'Friccion bm',     'LConn', 1;
  'Friccion bm',    'RConn', 1, 'Chasis',          'LConn', 1;
  'Motor Kt',       'RConn', 1, 'Reductora 1:20',  'LConn', 1;   % entrada S
  'Reductora 1:20', 'RConn', 1, 'Rueda + robot/4', 'LConn', 1;   % salida O
  'Reductora 1:20', 'RConn', 1, 'Encoder',         'LConn', 1;   % R del sensor
  'Encoder',        'RConn', 1, 'Chasis',          'LConn', 1;   % C del sensor
  'Encoder',        'RConn', 2, 'PS->S',           'LConn', 1};  % W (velocidad angular)
failed = {};
for k = 1:size(conn, 1)
    try
        A = ph(conn{k,1}); B = ph(conn{k,4});
        add_line(mdl, A.(conn{k,2})(conn{k,3}), B.(conn{k,5})(conn{k,6}), 'autorouting', 'on');
    catch err
        failed{end+1} = sprintf('%s.%s%d -> %s.%s%d (%s)', conn{k,1}, conn{k,2}, conn{k,3}, ...
                                conn{k,4}, conn{k,5}, conn{k,6}, err.message); %#ok<SAGROW>
    end
end
save_system(mdl, fullfile(here, [mdl '.slx']));
if ~isempty(failed)
    fprintf(2, 'Conexiones que hay que hacer a mano en %s.slx:\n', mdl);
    fprintf(2, '  %s\n', failed{:});
    return
end

out = sim(mdl);
v = out.v_rueda;
figure('Color', 'w'); stairs(v.Time, v.Data, 'LineWidth', 1.4); hold on;
plot([0 0.1 0.1 1], [0 0 0.5 0.5], 'k--'); grid on;
xlabel('t [s]'); ylabel('v rueda [m/s]'); title('Simscape: motor 310 + reductora + PI (Ts = 10 ms)');
print(gcf, fullfile(here, '..', '..', 'docs', 'figures', 'simscape_motor.png'), '-dpng', '-r150');
