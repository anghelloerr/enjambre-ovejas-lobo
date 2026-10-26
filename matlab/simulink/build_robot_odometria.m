%% Construye y simula en Simulink el robot completo con odometria (cuadrado 1 m)
% Modelo generado por script: microros_robot_odometria.slx
%   cmd_vel (v, w) -> cinematica inversa del firmware (matriz 4x2)
%   -> 4 lazos de velocidad (primer orden, tau_cl = 25 ms, vectorizado 4x)
%   -> cinematica directa REAL (skid-steer, chi = 1.55)  -> pose real (x, y, th)
%   -> cinematica directa del FIRMWARE (chi = 1.70)      -> odometria de ruedas
%   -> giroscopo (w real + sesgo)                        -> odometria ruedas + giroscopo
% Requiere: MATLAB + Simulink (solo bloques basicos).
clear; clc;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here, '..'));
P = params_microros(); G = P.geo;
APB = G.APB; chi = P.skid.chi; tau = 0.025;

% Perfil del cuadrado (3 vueltas): [t v w]
v0 = 0.2; w0 = 1.0; seg = [];
for k = 1:12, seg = [seg; v0 0 1/v0; 0 0 0.3; 0 w0 (pi/2)/w0; 0 0 0.3]; end %#ok<AGROW>
t = [0; cumsum(seg(:,3))];
cmd_ts = zeros(2*size(seg,1) + 1, 3);                 % escalones: [t_k v w; t_{k+1}-eps v w]
for k = 1:size(seg,1)
    cmd_ts(2*k-1, :) = [t(k)          seg(k,1:2)];
    cmd_ts(2*k,   :) = [t(k+1) - 1e-6 seg(k,1:2)];
end
cmd_ts(end, :) = [t(end) 0 0];
assignin('base', 'cmd_ts', cmd_ts);

J_inv = [1 -APB; 1 -APB; 1 APB; 1 APB];
J_fw  = [1/4 1/4 1/4 1/4; -1/(4*APB) -1/(4*APB) 1/(4*APB) 1/(4*APB)];
J_real = [1/4 1/4 1/4 1/4; -1/(2*chi*G.W) -1/(2*chi*G.W) 1/(2*chi*G.W) 1/(2*chi*G.W)];

mdl = 'microros_robot_odometria';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); open_system(mdl);
set_param(mdl, 'StopTime', num2str(t(end)), 'Solver', 'ode45', 'MaxStep', '0.01');

add_block('simulink/Sources/From Workspace', [mdl '/cmd_vel'], 'VariableName', 'cmd_ts', ...
          'Position', [20 100 90 130]);
add_block('simulink/Math Operations/Gain', [mdl '/Cinematica inversa'], 'Gain', mat2str(J_inv), ...
          'Multiplication', 'Matrix(K*u)', 'Position', [130 95 210 135]);
add_block('simulink/Continuous/State-Space', [mdl '/4 lazos de velocidad'], ...
          'A', mat2str(-eye(4)/tau), 'B', mat2str(eye(4)/tau), 'C', 'eye(4)', 'D', 'zeros(4)', ...
          'Position', [250 95 340 135]);
add_block('simulink/Math Operations/Gain', [mdl '/Cinematica real'], 'Gain', mat2str(J_real), ...
          'Multiplication', 'Matrix(K*u)', 'Position', [390 40 470 80]);
add_block('simulink/Math Operations/Gain', [mdl '/Cinematica firmware'], 'Gain', mat2str(J_fw), ...
          'Multiplication', 'Matrix(K*u)', 'Position', [390 150 470 190]);

% Subsistema uniciclo reutilizable: entrada [v; w] -> salida [x; y; th]
uni = [mdl '/Uniciclo'];
add_block('simulink/Ports & Subsystems/Subsystem', uni, 'Position', [530 40 610 80]);
delete_line(uni, 'In1/1', 'Out1/1');
add_block('simulink/Signal Routing/Demux', [uni '/vw'], 'Outputs', '2', 'Position', [60 30 65 70]);
add_block('simulink/Continuous/Integrator', [uni '/th'], 'Position', [140 80 170 110]);
add_block('simulink/Signal Routing/Mux', [uni '/v_th'], 'Inputs', '2', 'Position', [200 20 205 60]);
add_block('simulink/User-Defined Functions/Fcn', [uni '/xdot'], 'Expr', 'u(1)*cos(u(2))', 'Position', [240 10 320 30]);
add_block('simulink/User-Defined Functions/Fcn', [uni '/ydot'], 'Expr', 'u(1)*sin(u(2))', 'Position', [240 50 320 70]);
add_block('simulink/Continuous/Integrator', [uni '/x'], 'Position', [350 5 380 35]);
add_block('simulink/Continuous/Integrator', [uni '/y'], 'Position', [350 45 380 75]);
add_block('simulink/Signal Routing/Mux', [uni '/pose'], 'Inputs', '3', 'Position', [420 20 425 100]);
add_line(uni, 'In1/1', 'vw/1');
add_line(uni, 'vw/2', 'th/1');
add_line(uni, 'vw/1', 'v_th/1');
add_line(uni, 'th/1', 'v_th/2');
add_line(uni, 'v_th/1', 'xdot/1'); add_line(uni, 'v_th/1', 'ydot/1');
add_line(uni, 'xdot/1', 'x/1'); add_line(uni, 'ydot/1', 'y/1');
add_line(uni, 'x/1', 'pose/1'); add_line(uni, 'y/1', 'pose/2'); add_line(uni, 'th/1', 'pose/3');
add_line(uni, 'pose/1', 'Out1/1');
add_block(uni, [mdl '/Odometria ruedas'], 'Position', [530 150 610 190]);
add_block(uni, [mdl '/Odometria ruedas+giro'], 'Position', [530 240 610 280]);

% Giroscopo: w real + sesgo; v de los encoders
add_block('simulink/Signal Routing/Demux', [mdl '/real_vw'], 'Outputs', '2', 'Position', [490 100 495 130]);
add_block('simulink/Signal Routing/Demux', [mdl '/fw_vw'], 'Outputs', '2', 'Position', [490 200 495 230]);
add_block('simulink/Math Operations/Bias', [mdl '/Giroscopo'], 'Bias', num2str(P.imu.gyro_bias), ...
          'Position', [440 260 470 280]);
add_block('simulink/Signal Routing/Mux', [mdl '/v_enc_w_gyro'], 'Inputs', '2', 'Position', [500 240 505 280]);
add_block('simulink/Sinks/To Workspace', [mdl '/pose_real'], 'VariableName', 'pose_real', 'SaveFormat', 'Array', 'Position', [660 50 720 70]);
add_block('simulink/Sinks/To Workspace', [mdl '/pose_odo'], 'VariableName', 'pose_odo', 'SaveFormat', 'Array', 'Position', [660 160 720 180]);
add_block('simulink/Sinks/To Workspace', [mdl '/pose_gyro'], 'VariableName', 'pose_gyro', 'SaveFormat', 'Array', 'Position', [660 250 720 270]);

add_line(mdl, 'cmd_vel/1', 'Cinematica inversa/1', 'autorouting', 'on');
add_line(mdl, 'Cinematica inversa/1', '4 lazos de velocidad/1', 'autorouting', 'on');
add_line(mdl, '4 lazos de velocidad/1', 'Cinematica real/1', 'autorouting', 'on');
add_line(mdl, '4 lazos de velocidad/1', 'Cinematica firmware/1', 'autorouting', 'on');
add_line(mdl, 'Cinematica real/1', 'Uniciclo/1', 'autorouting', 'on');
add_line(mdl, 'Cinematica firmware/1', 'Odometria ruedas/1', 'autorouting', 'on');
add_line(mdl, 'Cinematica real/1', 'real_vw/1', 'autorouting', 'on');
add_line(mdl, 'Cinematica firmware/1', 'fw_vw/1', 'autorouting', 'on');
add_line(mdl, 'real_vw/2', 'Giroscopo/1', 'autorouting', 'on');
add_line(mdl, 'fw_vw/1', 'v_enc_w_gyro/1', 'autorouting', 'on');
add_line(mdl, 'Giroscopo/1', 'v_enc_w_gyro/2', 'autorouting', 'on');
add_line(mdl, 'v_enc_w_gyro/1', 'Odometria ruedas+giro/1', 'autorouting', 'on');
add_line(mdl, 'Uniciclo/1', 'pose_real/1', 'autorouting', 'on');
add_line(mdl, 'Odometria ruedas/1', 'pose_odo/1', 'autorouting', 'on');
add_line(mdl, 'Odometria ruedas+giro/1', 'pose_gyro/1', 'autorouting', 'on');
save_system(mdl, fullfile(here, [mdl '.slx']));

out = sim(mdl);
pr = out.pose_real; po = out.pose_odo; pg = out.pose_gyro;
fprintf('Error final ruedas: %.3f m | ruedas+giroscopo: %.3f m\n', ...
        norm(po(end,1:2) - pr(end,1:2)), norm(pg(end,1:2) - pr(end,1:2)));
figure('Color', 'w'); plot(pr(:,1), pr(:,2), 'k', 'LineWidth', 2); hold on;
plot(po(:,1), po(:,2), 'r--'); plot(pg(:,1), pg(:,2), 'b');
axis equal; grid on; legend('real', 'odometria ruedas', 'ruedas + giroscopo');
title('Simulink: cuadrado 1 m x 3 vueltas (sin ruido)');
print(gcf, fullfile(here, '..', '..', 'docs', 'figures', 'simulink_odometria.png'), '-dpng', '-r150');
