function f = sw_wall(p, S)
%SW_WALL Repulsion lineal desde las paredes de la arena (igual que behaviors.wall_repulsion).
half = S.arena/2; m = 0.40; f = [0 0];
for k = 1:2
    dl = p(k) + half; dh = half - p(k);
    if dl < m, f(k) = f(k) + (m - dl)/m; end
    if dh < m, f(k) = f(k) - (m - dh)/m; end
end
end
