function m = compute_metrics(t, y, r_final, t_step, y0)
%COMPUTE_METRICS  Step-response metrics for a signal y(t) after a step at t_step.
%   m = compute_metrics(t, y, r_final, t_step, y0)
%   Returns struct with:
%     overshoot_pct   percentage overshoot [%]
%     rise_time_s     10%-to-90% rise time [s]
%     settling_2pct_s 2% settling time measured from t_step [s]
%     settling_5pct_s 5% settling time measured from t_step [s]
%     sse             steady-state error (average over final 0.5 s)
%     rmse            root-mean-square tracking error after t_step
%     peak            peak value after t_step

if nargin < 5 || isempty(y0), y0 = 0; end
A = r_final - y0;
mask = t >= t_step;
tt = t(mask) - t_step;
yy = y(mask);

if abs(A) < 1e-9
    m = struct('overshoot_pct',0,'rise_time_s',0,'settling_2pct_s',0,'settling_5pct_s',0, ...
               'sse',r_final - mean(yy(max(1,end-100):end)),'rmse',sqrt(mean((yy-r_final).^2)),'peak',max(abs(yy)));
    return;
end

sA = sign(A);
ynorm = (yy - y0) / A;

% Overshoot
m.overshoot_pct = max(0, (max(ynorm) - 1) * 100);
m.peak = sA * max(sA * yy);

% Rise time (10% to 90%)
i10 = find(ynorm >= 0.10, 1, 'first');
i90 = find(ynorm >= 0.90, 1, 'first');
if ~isempty(i10) && ~isempty(i90) && i90 >= i10
    m.rise_time_s = tt(i90) - tt(i10);
else
    m.rise_time_s = NaN;
end

% Settling times (2% and 5% of step amplitude)
i2 = find(abs(yy - r_final) > 0.02 * abs(A), 1, 'last');
if isempty(i2), m.settling_2pct_s = 0; else, m.settling_2pct_s = tt(i2); end

i5 = find(abs(yy - r_final) > 0.05 * abs(A), 1, 'last');
if isempty(i5), m.settling_5pct_s = 0; else, m.settling_5pct_s = tt(i5); end

% Steady-state error (last 0.5 s) and RMSE
tail = tt >= max(0, tt(end) - 0.5);
m.sse  = r_final - mean(yy(tail));
m.rmse = sqrt(mean((yy - r_final).^2));
end
