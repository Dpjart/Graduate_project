function res = run_sim(opts)
%RUN_SIM  Simulate one configuration on one manoeuvre.
%   res = run_sim()        demo: 4-thruster, 1 m surge step, with plots
%   res = run_sim(opts)    see fields below (all optional)
%
%   opts.cfg          thruster layout struct (cfg_4/cfg_5/cfg_6)  [cfg_4(p)]
%   opts.p            parameter struct                           [rov_params]
%   opts.manoeuvre    name from trajectories.m                   ['step_surge']
%   opts.t_end        duration [s]                               [20]
%   opts.controller   'mpc' or 'open'                            ['mpc']
%   opts.f_open       @(t) -> nx1 thrust, for 'open'             [zeros]
%   opts.x0           12x1 initial state            [on reference, at rest]
%   opts.current      @(t) -> 3x1 NED current [m/s]               [zeros]
%   opts.tether       include tether drag                         [false]
%   opts.failed       indices of failed thrusters (force = 0)     [[]]
%   opts.fail_time    time the failure happens [s]                [0]
%   opts.mpc_knows_failure  MPC told about the failure            [true]
%   opts.thruster_lag first-order thruster lag on/off             [true]
%   opts.mpc_opts     struct passed to mpc_setup (keep identical!) [struct()]
%   opts.dt_sim       integration step [s]                        [0.01]
%   opts.plot         make figures                                [true]
%
%   res fields: t, x, f_cmd, f_act, tau, current, eta_ref, metrics, cfg, opts

if nargin < 1, opts = struct(); end
if ~isfield(opts, 'p'),            opts.p = rov_params();           end
p = opts.p;
def = struct('manoeuvre', 'step_surge', 't_end', 20, 'controller', 'mpc', ...
             'tether', false, 'failed', [], 'fail_time', 0, 'mpc_knows_failure', true, ...
             'thruster_lag', true, 'dt_sim', 0.01, 'plot', true);
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end
if ~isfield(opts, 'cfg'),      opts.cfg      = cfg_4(p);           end
if ~isfield(opts, 'mpc_opts'), opts.mpc_opts = struct();           end
if ~isfield(opts, 'current'),  opts.current  = @(t) zeros(3,1);    end
cfg = opts.cfg;
if ~isfield(opts, 'f_open'),   opts.f_open   = @(t) zeros(cfg.n,1); end

%% Setup
T = tcm(cfg);
n = cfg.n;
if ~isfield(opts, 'x0')
    [er0, ~] = trajectories(opts.manoeuvre, 0);
    opts.x0 = [er0; zeros(6,1)];
end

use_mpc = strcmp(opts.controller, 'mpc');
if use_mpc
    mpc   = mpc_setup(p, cfg, opts.mpc_opts);
    dt_c  = mpc.opts.dt;
    Nh    = mpc.opts.N;
    ratio = max(1, round(dt_c / opts.dt_sim));
end

dt   = opts.dt_sim;
Ns   = round(opts.t_end / dt);
t    = (0:Ns) * dt;
X    = zeros(12, Ns+1);   X(:,1) = opts.x0(:);
Fc   = zeros(n, Ns+1);
Fa   = zeros(n, Ns+1);
Eref = zeros(6, Ns+1);
Cur  = zeros(3, Ns+1);

f_cmd = zeros(n,1);
f_act = zeros(n,1);
env.tether = opts.tether;
failed_now = false;
mpc_informed = false;

%% Main loop
for k = 1:Ns+1
    tk = t(k);
    x  = X(:,k);
    [Eref(:,k), ~] = trajectories(opts.manoeuvre, tk);
    failed_now = ~isempty(opts.failed) && tk >= opts.fail_time;

    % --- controller ---
    if use_mpc
        if failed_now && opts.mpc_knows_failure && ~mpc_informed
            idx = [];
            for j = 0:Nh-1, idx = [idx, j*n + opts.failed(:)']; end %#ok<AGROW>
            mpc.lb(idx) = 0;  mpc.ub(idx) = 0;  mpc.U_prev(idx) = 0;
            mpc_informed = true;
        end
        if mod(k-1, ratio) == 0
            er = zeros(6, Nh);  nr = zeros(6, Nh);
            for j = 1:Nh
                [e_j, ed_j] = trajectories(opts.manoeuvre, tk + j*dt_c);
                Rz = [cos(e_j(6)) -sin(e_j(6)) 0; sin(e_j(6)) cos(e_j(6)) 0; 0 0 1];
                er(:,j) = e_j;
                nr(:,j) = [Rz' * ed_j(1:3); ed_j(4:6)];
            end
            [f_cmd, mpc] = mpc_step(mpc, x, er, nr);
        end
    else
        f_cmd = min(max(opts.f_open(tk), cfg.fmin), cfg.fmax);
    end
    Fc(:,k) = f_cmd;
    Cur(:,k) = opts.current(tk);

    if k == Ns+1, Fa(:,k) = f_act; break; end

    % --- actuator ---
    if opts.thruster_lag
        a = exp(-dt / p.tau_thr);
        f_act = f_cmd + (f_act - f_cmd) * a;
    else
        f_act = f_cmd;
    end
    if failed_now, f_act(opts.failed) = 0; end
    Fa(:,k) = f_act;

    % --- plant (RK4) ---
    env.v_current = Cur(:,k);
    X(:,k+1) = rk4(@(xx) rov_dynamics(xx, f_act, T, p, env), x, dt);
end

res.t = t;  res.x = X;  res.f_cmd = Fc;  res.f_act = Fa;
res.tau = T * Fa;  res.current = Cur;
res.eta_ref = Eref;  res.cfg = cfg;  res.opts = opts;
res.metrics = compute_metrics(res);

if opts.plot, plot_results(res); end
end

%% ======================================================================
function x1 = rk4(fun, x, h)
k1 = fun(x);
k2 = fun(x + h/2*k1);
k3 = fun(x + h/2*k2);
k4 = fun(x + h*k3);
x1 = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

%% ======================================================================
function m = compute_metrics(res)
t = res.t;  X = res.x;  Er = res.eta_ref;  cfg = res.cfg;
dt = t(2) - t(1);

e_pos = X(1:3,:) - Er(1:3,:);
e_ang = atan2(sin(X(4:6,:) - Er(4:6,:)), cos(X(4:6,:) - Er(4:6,:)));

m.rmse_pos     = sqrt(mean(sum(e_pos.^2, 1)));          % [m]
m.max_pos_err  = max(sqrt(sum(e_pos.^2, 1)));           % [m]
m.rmse_yaw_deg = rad2deg(sqrt(mean(e_ang(3,:).^2)));    % [deg]
m.max_roll_deg = rad2deg(max(abs(X(4,:))));
m.max_pitch_deg= rad2deg(max(abs(X(5,:))));
m.peak_yaw_rate_dps = rad2deg(max(abs(X(12,:))));
m.peak_surge_mps    = max(abs(X(7,:)));

lim = [cfg.fmin(:), cfg.fmax(:)];
F   = res.f_cmd;
sat = (F >= 0.98*lim(:,2)) | (F <= 0.98*lim(:,1));
sat(lim(:,2) == 0, :) = false;
m.sat_frac = mean(any(sat, 1));                         % share of time any thruster saturated
m.effort   = sum(sum(F.^2)) * dt;                       % [N^2 s]

% Step metrics (only for step manoeuvres)
m.rise = NaN; m.settle = NaN; m.overshoot = NaN; m.coupling = NaN;
name = res.opts.manoeuvre;
if strncmp(name, 'step_', 5)
    switch name
        case 'step_surge',  ax = 1;
        case 'step_sway',   ax = 2;
        case 'step_heave',  ax = 3;
        otherwise,          ax = 6;
    end
    y0 = Er(ax, 1);  yf = Er(ax, end);  A = yf - y0;
    y  = X(ax, :);
    if ax == 6, y = unwrap(y); end
    idx = t >= 1.0;                      % step time in trajectories.m
    tt = t(idx);  yy = (y(idx) - y0) / A;
    i10 = find(yy >= 0.1, 1);  i90 = find(yy >= 0.9, 1);
    if ~isempty(i10) && ~isempty(i90), m.rise = tt(i90) - tt(i10); end
    outside = find(abs(yy - 1) > 0.02, 1, 'last');
    if isempty(outside), m.settle = 0;
    elseif outside < numel(tt), m.settle = tt(outside+1) - 1.0;
    end
    m.overshoot = max(0, (max(yy) - 1) * 100);          % [%]
    other = setdiff(1:6, ax);
    dev = [e_pos; e_ang];
    m.coupling = max(abs(dev(other, idx)), [], 2)';     % [m m m rad rad rad] minus ax
    m.coupling_axes = other;
end
end

%% ======================================================================
function plot_results(res)
t = res.t;  X = res.x;  Er = res.eta_ref;
figure('Name', res.cfg.name);
lab = {'x [m]','y [m]','z [m]','\phi [deg]','\theta [deg]','\psi [deg]'};
for i = 1:6
    subplot(4,2,i);
    if i <= 3
        plot(t, X(i,:), 'b', t, Er(i,:), 'r--');
    else
        plot(t, rad2deg(unwrap(X(i,:))), 'b', t, rad2deg(unwrap(Er(i,:))), 'r--');
    end
    ylabel(lab{i}); grid on;
end
subplot(4,2,[7 8]);
plot(t, res.f_cmd'); grid on; xlabel('t [s]'); ylabel('f_{cmd} [N]');
legend(arrayfun(@(i) sprintf('f%d', i), 1:res.cfg.n, 'UniformOutput', false), ...
       'Location', 'eastoutside');
title(sprintf('%s | %s | RMSE %.3f m', res.cfg.name, res.opts.manoeuvre, ...
      res.metrics.rmse_pos), 'Interpreter', 'tex');

if any(strcmp(res.opts.manoeuvre, {'survey','helix','turn'}))
    figure('Name', [res.cfg.name ' path']);
    plot3(X(2,:), X(1,:), -X(3,:), 'b', Er(2,:), Er(1,:), -Er(3,:), 'r--');
    xlabel('East y [m]'); ylabel('North x [m]'); zlabel('-z (up) [m]');
    legend('actual', 'reference'); grid on; axis equal;
end
end
