function [u, fear] = sw_sheep_velocity(i, pos, vel, wolf, S)
%SW_SHEEP_VELOCITY Reynolds (separacion, cohesion, alineacion) + huida del lobo.
%   Traduccion directa de behaviors.sheep_desired_velocity (Python).
p = pos(i,:);
diff = pos - p; d = sqrt(sum(diff.^2, 2)); d(i) = inf;
sep = [0 0]; close = d < S.r_sep;
if any(close)
    dc = d(close);   % separacion continua: nula en d = r_sep (evita el castaneteo del rumbo)
    sep = -mean(diff(close,:) ./ dc .* (S.r_sep ./ dc - 1), 1);   % promedio: rigidez independiente del n. de vecinos
end
coh = [0 0]; ali = [0 0]; nb = d < S.r_nb;
if any(nb), coh = mean(pos(nb,:), 1) - p; ali = mean(vel(nb,:), 1); end
flee = [0 0]; fear = 0;
dw = p - wolf; dn = norm(dw);
if dn < S.r_fear, fear = 1 - dn/S.r_fear; flee = dw/max(dn, 1e-9)*fear; end
u = S.w_sep*sep + S.w_coh*coh*(1 + fear) + S.w_ali*ali + S.w_flee*flee + S.w_wall*sw_wall(p, S);
if fear > 0, vcap = S.v_sheep*(0.35 + 0.65*fear); else, vcap = 0.35*S.v_sheep; end
n = norm(u); if n > vcap, u = u/n*vcap; end
end
