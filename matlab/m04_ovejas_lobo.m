%% M04 - Algoritmo ovejas-lobo en MATLAB (pastoreo al corral)
% Misma ley de control que ros2_ws/src/swarm_sheep_wolf/swarm_sheep_wolf/behaviors.py.
% Cada robot se modela como uniciclo con la cinematica del firmware (limites
% de 1 m/s por rueda y 5 rad/s) y un retardo de primer orden de 25 ms (M02).
clear; clc; close all;
here = fileparts(mfilename('fullpath')); addpath(here);
P = params_microros(); S = P.swarm;
figdir = fullfile(here, '..', 'docs', 'figures');
rand('seed', 3); randn('seed', 3); %#ok<RAND>

n = S.n_sheep; dt = 0.05; T = 240; HOLD = 60;   % 60 s de vigilancia tras llegar
pos = zeros(n, 2); k = 0;
while k < n                                     % rebano disperso, sin solaparse
    q = [-0.6 -0.6] + (rand(1,2) - 0.5)*1.4;
    if k == 0 || all(sqrt(sum((pos(1:k,:) - q).^2, 2)) > 0.35), k = k + 1; pos(k,:) = q; end
end
th = (rand(n,1) - 0.5)*2*pi; vel = zeros(n, 2); vs = zeros(n,1); ws = zeros(n,1);
wolf = [-1.6 -1.6]; thw = pi/4; vw = 0; ww = 0;
tau = 0.025; a = dt/(tau + dt);
hist = zeros(round(T/dt), n+1, 2); coh = zeros(round(T/dt), 1); dmin = coh; fin = coh; t_goal = NaN;
for it = 1:round(T/dt)
    for i = 1:n
        u = sw_sheep_velocity(i, pos, vel, wolf, S);
        [vc, wc] = sw_unicycle(u, th(i), S, P.ctl.v_max, P.ctl.w_max);
        vs(i) = vs(i) + a*(vc - vs(i)); ws(i) = ws(i) + a*(wc - ws(i));
    end
    [u, mode] = sw_wolf_velocity(wolf, pos, S);
    [vc, wc] = sw_unicycle(u, thw, S, P.ctl.v_max, P.ctl.w_max);
    vw = vw + a*(vc - vw); ww = ww + a*(wc - ww);
    th = th + ws*dt; thw = thw + ww*dt;
    vel = [vs.*cos(th), vs.*sin(th)];
    pos = pos + vel*dt; wolf = wolf + vw*[cos(thw) sin(thw)]*dt;
    hist(it, 1:n, :) = pos; hist(it, n+1, :) = wolf;
    coh(it) = mean(sqrt(sum((pos - mean(pos,1)).^2, 2)));
    allp = [pos; wolf]; dd = inf;
    for p1 = 1:n+1, for p2 = p1+1:n+1, dd = min(dd, norm(allp(p1,:) - allp(p2,:))); end, end
    dmin(it) = dd;
    fin(it) = mean(sqrt(sum((pos - S.goal).^2, 2)) < S.goal_r);
    if isnan(t_goal) && fin(it) == 1, t_goal = it*dt; end
    if ~isnan(t_goal) && it*dt > t_goal + HOLD, break; end
end
nt = it; ig = round(t_goal/dt);
fprintf('Pastoreo terminado en %.1f s, cohesion media %.3f m, distancia minima %.3f m\n', ...
        t_goal, mean(coh(1:nt)), min(dmin(1:nt)));
fprintf('Contencion durante %d s de vigilancia: %.1f %% de las ovejas dentro del corral\n', ...
        HOLD, 100*mean(fin(ig:nt)));

figure('Color', 'w', 'Position', [100 100 520 500]); hold on; axis equal;
h = S.arena/2; plot([-h h h -h -h], [-h -h h h -h], 'k', 'LineWidth', 1.4);
tt = linspace(0, 2*pi, 100); fill(S.goal(1) + S.goal_r*cos(tt), S.goal(2) + S.goal_r*sin(tt), ...
     [0.8 0.93 0.8], 'EdgeColor', 'none');
for i = 1:n, plot(hist(1:nt,i,1), hist(1:nt,i,2), 'Color', [0.12 0.47 0.71]); end
plot(hist(1:nt,n+1,1), hist(1:nt,n+1,2), 'Color', [0.84 0.15 0.16], 'LineWidth', 1.5);
plot(hist(nt,1:n,1), hist(nt,1:n,2), 'o', 'MarkerFaceColor', [0.12 0.47 0.71], 'MarkerEdgeColor', 'none');
plot(hist(nt,n+1,1), hist(nt,n+1,2), 's', 'MarkerFaceColor', [0.84 0.15 0.16], 'MarkerEdgeColor', 'none');
xlim([-h h]*1.05); ylim([-h h]*1.05); grid on; xlabel('x [m]'); ylabel('y [m]');
title(sprintf('MATLAB: pastoreo ovejas-lobo (%.0f s)', t_goal));
print(gcf, fullfile(figdir, 'm04_ovejas_lobo_matlab.png'), '-dpng', '-r150');
