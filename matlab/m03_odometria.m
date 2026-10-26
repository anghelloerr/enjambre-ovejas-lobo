%% M03 - Odometria del MicroROS-Pi5: error sistematico del skid-steer y fusion con giroscopo
% 1. Error de rumbo por la diferencia entre chi del firmware y chi real
% 2. Prediccion analitica del error en el cuadrado de 1 m x 3 vueltas
% 3. Simulacion Monte Carlo (encoders cuantizados + giroscopo con sesgo) de tres estimadores
% 4. Calibracion tipo Yahboom (calibrate_angular) y su sensibilidad a cambios de suelo
% Resultados comparables con python/odometry_experiment.py.
clear; clc; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
P = params_microros();
figdir = fullfile(here, '..', 'docs', 'figures');
G = P.geo;
chi_fw = 2*G.APB/G.W;

%% 1. Error sistematico
% El firmware calcula w_odo = (vR - vL)/(2 APB); la realidad es w = (vR - vL)/(chi W).
% => w_odo / w = chi / chi_fw
ratio = P.skid.chi / chi_fw;
fprintf('chi_fw = %.3f, chi_real = %.2f -> w_odo/w = %.4f\n', chi_fw, P.skid.chi, ratio);
% El robot ejecuta el giro hasta que la odometria marca 90 grados (lazo cerrado sobre odom)
% o gira w_cmd*T (lazo abierto). En nuestro ensayo se manda w_cmd en lazo abierto:
% el robot gira de verdad w_cmd * chi_fw/chi * T y la odometria cree w_cmd * T.
turn_real = 90 * chi_fw / P.skid.chi;
fprintf('Orden de 90 grados en lazo abierto: el robot gira %.2f grados reales\n', turn_real);
fprintf('Error de rumbo acumulado tras 12 giros: %.1f grados\n', 12*(turn_real - 90));

%% 2. Monte Carlo
Ts = 0.02; nseed = 20; v = 0.2; w = 1.0; side = 1.0;
plan = [];
for k = 1:12
    plan = [plan; v 0 side/v; 0 0 0.3; 0 w (pi/2)/w; 0 0 0.3]; %#ok<AGROW>
end
cpm = P.mot.cpr / (2*pi*G.r);                  % cuentas por metro de rueda
err = zeros(nseed, 3);
for s = 1:nseed
    rng_state = s; rand('seed', rng_state); randn('seed', rng_state); %#ok<RAND>
    chi = P.skid.chi * (1 + 0.04*randn);       % suelo distinto en cada ensayo
    x = [0 0 0]; xe = [0 0 0]; xg = [0 0 0]; acc = zeros(1, 4); wheel = zeros(1, 4);
    for k = 1:size(plan, 1)
        for n = 1:round(plan(k,3)/Ts)
            tgt = [plan(k,1) - plan(k,2)*G.APB, plan(k,1) - plan(k,2)*G.APB, ...
                   plan(k,1) + plan(k,2)*G.APB, plan(k,1) + plan(k,2)*G.APB];
            wheel = wheel + Ts/(0.025 + Ts) * (tgt - wheel);
            ground = wheel .* (1 + P.enc.slip_sigma*randn(1, 4));
            vl = mean(ground(1:2)); vr = mean(ground(3:4));
            vt = (vl + vr)/2; wt = (vr - vl)/(chi*G.W);
            x = x + [vt*cos(x(3) + wt*Ts/2)*Ts, vt*sin(x(3) + wt*Ts/2)*Ts, wt*Ts];
            % encoders
            acc = acc + wheel*Ts*cpm; c = floor(acc); acc = acc - c;
            we = c/cpm/Ts;
            ve = mean(we); w_enc = (mean(we(3:4)) - mean(we(1:2)))/(2*G.APB);
            gyro = wt + P.imu.gyro_bias + P.imu.gyro_sigma*randn;
            xe = xe + [ve*cos(xe(3) + w_enc*Ts/2)*Ts, ve*sin(xe(3) + w_enc*Ts/2)*Ts, w_enc*Ts];
            xg = xg + [ve*cos(xg(3) + gyro*Ts/2)*Ts, ve*sin(xg(3) + gyro*Ts/2)*Ts, gyro*Ts];
        end
    end
    err(s, :) = [norm(xe(1:2) - x(1:2)), norm(xg(1:2) - x(1:2)), ...
                 abs(atan2(sin(xg(3) - x(3)), cos(xg(3) - x(3))))*180/pi];
end
fprintf('\nCuadrado 1 m x 3 vueltas, %d ensayos:\n', nseed);
fprintf('  Solo encoders      : error final %.3f +- %.3f m\n', mean(err(:,1)), std(err(:,1)));
fprintf('  Encoders + giro    : error final %.3f +- %.3f m (yaw %.1f grados)\n', ...
        mean(err(:,2)), std(err(:,2)), mean(err(:,3)));

%% 3. Deriva del giroscopo: limite teorico del estimador encoders + giroscopo
T_total = sum(plan(:,3));
fprintf('\nDuracion %.0f s; sesgo %.3f rad/s -> deriva de rumbo %.1f grados\n', ...
        T_total, P.imu.gyro_bias, P.imu.gyro_bias*T_total*180/pi);
fprintf('Recomendacion: estimar el sesgo con el robot parado (2 s) antes de cada experimento.\n');
