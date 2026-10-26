function [v, w] = sw_unicycle(u, th, S, vmax, wmax)
%SW_UNICYCLE Velocidad deseada 2D -> (v, w) de un robot no holonomo.
sp = norm(u);
if sp < 0.02, v = 0; w = 0; return; end   % zona muerta (v_dead)
e = atan2(sin(atan2(u(2), u(1)) - th), cos(atan2(u(2), u(1)) - th));
if sp < 0.05 && abs(e) > pi/2          % ajuste lento con el objetivo detras: retroceder
    e = atan2(sin(e - pi), cos(e - pi)); v = -sp*cos(e);
else
    v = min(sp*max(cos(e), 0), vmax);
end
w = S.k_heading*e*min(1, sp/0.05);             % omega reducido a baja velocidad (v_turn_ref)
w = max(min(w, wmax), -wmax);
end
