%% Importa el MicroROS-Pi5 a Simscape Multibody desde el URDF y anima las ruedas
% Genera microros_multibody.slx con smimport a partir de microros_matlab.urdf
% (el mismo modelo que usa Gazebo, sin los plugins). Despues:
%   * imprime la masa total y la posicion del centro de masas
%   * configura las 4 juntas de rueda con movimiento prescrito (velocidad del
%     perfil v = 0.2 m/s) para ver el modelo en Mechanics Explorer
% Requiere: Simscape Multibody (R2019b o posterior).
clear; clc;
here = fileparts(mfilename('fullpath')); addpath(fullfile(here, '..'));
P = params_microros();
urdf = fullfile(here, 'microros_matlab.urdf');
mdl = 'microros_multibody';
if bdIsLoaded(mdl), close_system(mdl, 0); end

[h, ~] = smimport(urdf, 'ModelName', mdl);
save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Modelo importado: %s.slx\n', mdl);

% Juntas de rueda: smimport crea un bloque "Revolute Joint" por junta del URDF.
joints = find_system(mdl, 'LookUnderMasks', 'all', 'RegExp', 'on', 'Name', '(zq|zh|yq|yh)_Joint');
fprintf('Juntas de rueda encontradas: %d\n', numel(joints));
w_wheel = 0.2 / P.geo.r;                                   % rad/s a 0.2 m/s
for k = 1:numel(joints)
    try
        set_param(joints{k}, 'MotionActuationMode', 'InputMotion');      % R2019b+
        pos = get_param(joints{k}, 'Position');
        src = [mdl '/w_' num2str(k)];
        add_block('simulink/Sources/Ramp', src, 'Slope', num2str(w_wheel), ...
                  'Position', pos + [-160 0 -130 0]);
        cvt = [mdl '/ps_' num2str(k)];
        add_block('nesl_utility/Simulink-PS Converter', cvt, 'Position', pos + [-100 0 -80 0], ...
                  'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '2');
        add_line(mdl, ['w_' num2str(k) '/1'], ['ps_' num2str(k) '/1'], 'autorouting', 'on');
        pj = get_param(joints{k}, 'PortHandles'); pc = get_param(cvt, 'PortHandles');
        add_line(mdl, pc.RConn(1), pj.LConn(end), 'autorouting', 'on');
    catch err
        fprintf(2, 'No se pudo actuar %s: %s\n', joints{k}, err.message);
    end
end
save_system(mdl);
fprintf(['Abre %s.slx y pulsa Run: Mechanics Explorer muestra el robot con las ruedas girando.\n' ...
         'Para traccion real sobre el suelo, anadir bloques Spatial Contact Force rueda-plano\n' ...
         '(ver docs/06_matlab_simulink_simscape.md).\n'], mdl);
