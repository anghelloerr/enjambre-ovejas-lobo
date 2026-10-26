function run_all()
%RUN_ALL Ejecuta todos los calculos verificados (MATLAB o GNU Octave).
% Las figuras se guardan en docs/figures/. Los modelos de Simulink y Simscape
% se construyen aparte (simulink/, simscape/) porque requieren licencia de MATLAB.
here = fileparts(mfilename('fullpath'));
if exist('OCTAVE_VERSION', 'builtin'), pkg load control; end
scripts = {'m01_cinematica', 'm02_motor_pid', 'm03_odometria', 'm04_ovejas_lobo'};
for k = 1:numel(scripts)
    fprintf('\n===== %s =====\n', scripts{k});
    run_one(fullfile(here, [scripts{k} '.m']));   % cada script hace "clear" en su propio espacio
    close all;
end
cd(here);
end

function run_one(path)
run(path);
end
