function [u, mode] = sw_wolf_velocity(wolf, pos, S)
%SW_WOLF_VELOCITY Pastor de Strombom et al. (2014): recolectar o conducir al corral.
goal = S.goal; gcm = mean(pos, 1); n = size(pos, 1);
fN = 0.45 * n^(2/3); dg = sqrt(sum((pos - gcm).^2, 2));
half = S.arena/2 - 0.25;
opn = -goal/max(norm(goal), 1e-9);   % lado abierto del corral (hacia el centro)
if all(sqrt(sum((pos - goal).^2, 2)) < S.goal_r)
    target = goal + opn*(S.goal_r + 0.45); mode = 'guard';   % vigilancia
elseif max(dg) > fN
    [~, j] = max(dg); dirv = pos(j,:) - gcm;
    target = pos(j,:) + dirv/max(norm(dirv), 1e-9)*0.45; mode = 'collect';
else
    dirv = gcm - goal;
    if dot(dirv, opn) > 0.5*S.goal_r, dirv = dirv/norm(dirv); else, dirv = opn; end   % nunca desde la esquina
    target = gcm + dirv*(max(dg) + 0.45); mode = 'drive';
end
target = min(max(target, -half), half);
u = target - wolf;
if strcmp(mode, 'guard') && norm(u) < 0.05, u = [0 0]; return; end
d = sqrt(sum((pos - wolf).^2, 2)); near = d < 0.40;
if any(near), u = u + sum((wolf - pos(near,:)) ./ (d(near).^2), 1)*0.05; end
u = u + S.w_wall*sw_wall(wolf, S)*0.3;
nu = norm(u); if nu > S.v_wolf, u = u/nu*S.v_wolf; end
end
