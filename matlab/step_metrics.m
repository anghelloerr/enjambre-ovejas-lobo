function m = step_metrics(y, t)
%STEP_METRICS Tiempo de subida (10-90 %), sobreimpulso y establecimiento (2 %).
y = y(:); t = t(:); yf = y(end);
i10 = find(y >= 0.1*yf, 1); i90 = find(y >= 0.9*yf, 1);
m.RiseTime = t(i90) - t(i10);
m.Overshoot = max(0, (max(y) - yf)/abs(yf)*100);
out = find(abs(y - yf) > 0.02*abs(yf));
if isempty(out), m.SettlingTime = 0; else, m.SettlingTime = t(min(out(end)+1, numel(t))); end
m.Final = yf;
end
