function [gm_db, pm_deg, wc, w180] = loop_margins(L, Ts)
%LOOP_MARGINS Margenes de ganancia y fase de un lazo discreto L(z) por barrido.
%   Implementacion propia (funciona igual en MATLAB y en Octave).
w = logspace(-1, log10(pi/Ts*0.999), 20000);
H = squeeze(freqresp(L, w));
mag = abs(H); ph = unwrap(angle(H))*180/pi;
ic = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
if isempty(ic), wc = NaN; pm_deg = Inf;
else
    wc = interp1(log(mag(ic:ic+1)), w(ic:ic+1), 0);
    pm_deg = 180 + interp1(w(ic:ic+1), ph(ic:ic+1), wc);
end
k = find((ph(1:end-1) + 180).*(ph(2:end) + 180) <= 0, 1);
if isempty(k), w180 = NaN; gm_db = Inf;
else
    w180 = interp1(ph(k:k+1), w(k:k+1), -180);
    gm_db = -20*log10(interp1(w(k:k+1), mag(k:k+1), w180));
end
end
