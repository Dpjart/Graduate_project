function m = rov_metrics(t, err, window, tol, dwell, event_time)
%ROV_METRICS  Tracking metrics with UAV-style recovery definition.
%   m = rov_metrics(t, err, window, tol, dwell, event_time)
%   t          1xN time [s]
%   err        3xN position error, reference - actual [m]
%   window     [start end] of the disturbance, or [] if none
%   tol, dwell recovery = error <= tol continuously for dwell seconds
%   event_time time of another event (e.g. thruster failure), or [] if none
%
%   Recovery (UAV definition): first time after the disturbance window ends
%   that the error stays <= tol for dwell seconds, measured from window end.
%   Convergence: time after which the error stays <= tol until the run ends.

t = t(:)';
e3 = sqrt(sum(err.^2, 1));
m.rmse    = sqrt(mean(err.^2, 2));          % per axis [m]
m.rmse3D  = sqrt(mean(e3.^2));
m.maximum = max(e3);
m.final   = e3(end);
m.initial = e3(1);
m.IAE     = trapz(t, e3);                   % integral of absolute error [m s]

% Convergence (settling) time: after this the error stays <= tol until the
% end of the run. NaN if the run ends outside tolerance.
last_out = find(e3 > tol, 1, 'last');
if isempty(last_out),          m.convergence = 0;
elseif last_out == numel(e3),  m.convergence = NaN;
else,                          m.convergence = t(last_out + 1) - t(1);
end

m.disturbanceMaximum = NaN;  m.recovery = NaN;  m.recoveryStatus = 'not_applicable';
if ~isempty(window)
    mask = t >= window(1) & t < window(2);
    if any(mask), m.disturbanceMaximum = max(e3(mask)); end
    if t(end) < window(2) + dwell
        m.recoveryStatus = 'insufficient_post_disturbance_time';
    else
        m.recovery = settle_from(t, e3, window(2), tol, dwell);
        if isnan(m.recovery), m.recoveryStatus = 'not_recovered';
        else,                 m.recoveryStatus = 'recovered';
        end
    end
end

m.eventMaximum = NaN;  m.eventRMSE = NaN;
if nargin >= 6 && ~isempty(event_time)
    after = t >= event_time;
    m.eventMaximum = max(e3(after));
    m.eventRMSE    = sqrt(mean(e3(after).^2));
end
end

function ts = settle_from(t, e3, t0, tol, dwell)
% First time after t0 at which the error stays <= tol for at least dwell.
% Returned relative to t0. NaN if never.
ts = NaN;  start = [];
for k = find(t >= t0, 1):numel(t)
    if e3(k) > tol
        start = [];
    elseif isempty(start)
        start = k;
    end
    if ~isempty(start) && t(k) - t(start) >= dwell - 1e-9
        ts = t(start) - t0;  return;
    end
end
end
