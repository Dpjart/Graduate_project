function out = analysis(mode, z_off)
%ANALYSIS  Run the test plan across all thruster configurations.
%   out = analysis('geometry')     Tests 1-2: thrust matrix, authority, failures
%   out = analysis('sanity')       Test 3:    open-loop model checks (cfg_4)
%   out = analysis('steps')        Test 4:    step responses per DOF (MPC)
%   out = analysis('turning')      Test 5:    peak yaw rate + tight circle
%   out = analysis('tracking')     Test 6:    survey + helix tracking
%   out = analysis('disturbance')  Test 7:    station keeping, current step
%   out = analysis('failure')      Test 8:    survey with thruster 1 failed
%   out = analysis('all')          everything above
%   out = analysis(mode, z_off)    all thrusters shifted vertically by z_off [m]
%                                  (e.g. -0.10 = top-mounted placement test)
%
%   Configurations compared: 4, 5, 6@30deg, 6@45deg, 6@60deg.
%   MPC tuning is identical for every configuration (fairness).

if nargin < 1, mode  = 'geometry'; end
if nargin < 2, z_off = 0;          end

p    = rov_params();
cfgs = {cfg_4(p, z_off), cfg_5(p, z_off), ...
        cfg_6(p, 30, z_off), cfg_6(p, 45, z_off), cfg_6(p, 60, z_off)};
tags = {'4', '5', '6@30', '6@45', '6@60'};
mpc_opts = struct();            % one tuning for all configs - do not change per config

out = struct();
if any(strcmp(mode, {'geometry','all'})),    out.geometry    = test_geometry(p, cfgs, tags);           end
if any(strcmp(mode, {'sanity','all'})),      out.sanity      = test_sanity(p);                         end
if any(strcmp(mode, {'steps','all'})),       out.steps       = test_steps(p, cfgs, tags, mpc_opts);    end
if any(strcmp(mode, {'turning','all'})),     out.turning     = test_turning(p, cfgs, tags, mpc_opts);  end
if any(strcmp(mode, {'tracking','all'})),    out.tracking    = test_tracking(p, cfgs, tags, mpc_opts); end
if any(strcmp(mode, {'disturbance','all'})), out.disturbance = test_disturbance(p, cfgs, tags, mpc_opts); end
if any(strcmp(mode, {'failure','all'})),     out.failure     = test_failure(p, cfgs, tags, mpc_opts);  end
end

%% ======================================================================
%  TESTS 1-2: GEOMETRY
%% ======================================================================
function G = test_geometry(p, cfgs, tags)
dof = {'X','Y','Z','K','M','N'};
Lc  = p.L/2;                         % characteristic length to normalise moments
S   = diag([1 1 1 1/Lc 1/Lc 1/Lc]);
nc  = numel(cfgs);
G.pure_pos = zeros(nc, 6);  G.pure_neg = zeros(nc, 6);

fprintf('\n=== TEST 1: THRUST MATRIX ===\n');
for c = 1:nc
    cfg = cfgs{c};
    T   = tcm(cfg);
    sv  = svd(S*T);
    nz  = sv(sv > 1e-9);
    G.T{c}    = T;
    G.rank(c) = rank(T, 1e-9);
    G.cond(c) = max(nz) / min(nz);
    for j = 1:6
        G.pure_pos(c,j) = pure_axis_max(T, cfg,  j);
        G.pure_neg(c,j) = pure_axis_max(T, cfg, -j);
    end
    G.coupled_max(c,:) = sum(max(T .* cfg.fmax', T .* cfg.fmin'), 2)';
    % coupling: does a pure surge allocation create other moments?
    f_s = pinv(T) * [1;0;0;0;0;0];
    G.surge_coupling(c,:) = (T * f_s)';

    fprintf('\n[%s] %s\n', tags{c}, cfg.name);
    fprintf('  rank = %d, cond (moments / %.2f m) = %.2f\n', G.rank(c), Lc, G.cond(c));
    fprintf('  T =\n'); disp(T);
end

fprintf('\nPure-axis max (others held at zero), + / - direction\n');
fprintf('%-6s', 'cfg'); fprintf('%14s', dof{:}); fprintf('\n');
for c = 1:nc
    fprintf('%-6s', tags{c});
    for j = 1:6, fprintf('%7.1f/%-6.1f', G.pure_pos(c,j), G.pure_neg(c,j)); end
    fprintf('\n');
end
fprintf('(forces N, moments N.m)\n');

figure('Name', 'Pure-axis authority');
bar(G.pure_pos'); set(gca, 'XTickLabel', dof); grid on;
legend(tags, 'Location', 'northeast'); ylabel('max pure force [N] / moment [N.m]');
title('Test 1: pure-axis authority (+ direction)');

fprintf('\n=== TEST 2: SINGLE-THRUSTER FAILURE ===\n');
for c = 1:nc
    cfg = cfgs{c};  T = tcm(cfg);
    fprintf('[%s]\n', tags{c});
    for i = 1:cfg.n
        keep = setdiff(1:cfg.n, i);
        cf = cfg;  cf.n = numel(keep);
        cf.r = cfg.r(:,keep); cf.e = cfg.e(:,keep);
        cf.fmax = cfg.fmax(keep); cf.fmin = cfg.fmin(keep);
        Tf = tcm(cf);
        pp = zeros(1,6);
        for j = 1:6, pp(j) = min(pure_axis_max(Tf, cf, j), pure_axis_max(Tf, cf, -j)); end
        intact = min(G.pure_pos(c,:), G.pure_neg(c,:));
        lost   = dof(intact > 1e-6 & pp < 1e-6);
        ret    = pp(intact > 1e-6) ./ intact(intact > 1e-6);
        G.fail{c}(i).rank = rank(Tf, 1e-9);
        G.fail{c}(i).lost = lost;
        G.fail{c}(i).min_retained = min(ret);
        if isempty(lost), ls = 'none'; else, ls = strjoin(lost, ','); end
        fprintf('  f%d off: rank %d, DOF lost: %-10s worst retained authority %.0f%%\n', ...
                i, G.fail{c}(i).rank, ls, 100*min(ret));
    end
end
end

function s = pure_axis_max(T, cfg, jsigned)
% Largest s >= 0 such that T*f = s*e_j with fmin <= f <= fmax.
j  = abs(jsigned);  sg = sign(jsigned);
n  = size(T, 2);
ej = zeros(6,1);  ej(j) = sg;
c   = [zeros(n,1); -1];
Aeq = [T, -ej];  beq = zeros(6,1);
lb  = [cfg.fmin(:); 0];  ub = [cfg.fmax(:); 1e6];
if exist('linprog', 'file') == 2 || exist('linprog', 'builtin') == 5
    o = optimoptions('linprog', 'Display', 'off');
    [z, ~, flag] = linprog(c, [], [], Aeq, beq, lb, ub, o);
    if flag ~= 1, s = 0; return; end
    s = z(end);
elseif exist('glpk') %#ok<EXIST>
    [z, ~, st] = glpk(c, Aeq, beq, lb, ub, repmat('S',1,6), repmat('C',1,n+1), 1);
    if st ~= 0 && st ~= 5, s = 0; return; end
    s = z(end);
else
    % Fallback without Optimization Toolbox: min-norm allocation (lower bound)
    fd = pinv(T) * ej;
    if norm(T*fd - ej) > 1e-6, s = 0; return; end
    k = inf;
    for i = 1:n
        if fd(i) > 1e-12, k = min(k, cfg.fmax(i)/fd(i)); end
        if fd(i) < -1e-12, k = min(k, cfg.fmin(i)/fd(i)); end
    end
    s = k;
end
if s < 1e-6, s = 0; end
end

%% ======================================================================
%  TEST 3: OPEN-LOOP SANITY (no controller, cfg_4, no tether)
%% ======================================================================
function S = test_sanity(p)
fprintf('\n=== TEST 3: OPEN-LOOP SANITY (4-thruster, tether off) ===\n');
cfg = cfg_4(p);
base = struct('cfg', cfg, 'p', p, 'controller', 'open', 'manoeuvre', 'hold', ...
              'tether', false, 'plot', false);

% (a) zero thrust from rest
o = base; o.t_end = 20;
r = run_sim(o);
S.a_dz   = r.x(3,end) - r.x(3,1);
S.a_roll = rad2deg(max(abs(r.x(4,:))));
fprintf('(a) zero thrust 20 s: dz = %+.3f m (negative = rising), max roll %.2f deg\n', ...
        S.a_dz, S.a_roll);

% (b) release from 20 deg roll
o = base; o.t_end = 20; o.x0 = [0;0;0.5; deg2rad(20); 0; 0; zeros(6,1)];
r = run_sim(o);
S.b_roll_end = rad2deg(r.x(4,end));
S.b_roll_min = rad2deg(min(r.x(4,:)));
fprintf('(b) 20 deg roll release: roll after 20 s = %.2f deg, min = %.2f deg\n', ...
        S.b_roll_end, S.b_roll_min);
figure('Name', 'Sanity: roll release'); plot(r.t, rad2deg(r.x(4,:))); grid on;
xlabel('t [s]'); ylabel('\phi [deg]'); title('Test 3b: roll righting');

% (c) moderate surge thrust (10%) -> terminal speed vs hand calc
o = base; o.t_end = 30; fs = 0.1*cfg.fmax(1);
o.f_open = @(t) [fs; fs; 0; 0];
r = run_sim(o);
F  = 2*fs;
d1 = p.D_lin(1,1);  d2 = p.D_quad(1,1);
u_hand = (-d1 + sqrt(d1^2 + 4*d2*F)) / (2*d2);
S.c_u_sim = r.x(7,end);  S.c_u_hand = u_hand;
fprintf('(c) 10%% surge thrust: u_sim = %.3f m/s, u_hand = %.3f m/s (%.1f%% diff), max pitch %.1f deg\n', ...
        S.c_u_sim, u_hand, 100*abs(S.c_u_sim - u_hand)/u_hand, rad2deg(max(abs(r.x(5,:)))));

% (d) full surge thrust open loop -> passive pitch stability at speed
o.f_open = @(t) [cfg.fmax(1); cfg.fmax(2); 0; 0];  o.t_end = 10;
r = run_sim(o);
S.d_max_pitch = rad2deg(max(abs(r.x(5,:))));
if S.d_max_pitch > 90
    fprintf(['(d) full surge thrust, no controller: vehicle TUMBLES in pitch ' ...
             '(Munk moment beats BG righting at speed; needs active pitch control)\n']);
else
    fprintf('(d) full surge thrust, no controller: max pitch %.1f deg (passively stable)\n', ...
            S.d_max_pitch);
end
end

%% ======================================================================
%  TEST 4: STEP RESPONSES
%% ======================================================================
function R = test_steps(p, cfgs, tags, mpc_opts)
fprintf('\n=== TEST 4: STEP RESPONSES (MPC) ===\n');
steps = {'step_surge','step_sway','step_heave','step_yaw'};
fprintf('%-6s %-11s %8s %8s %9s %12s\n', 'cfg', 'step', 'rise[s]', 'settle', 'OS[%]', 'max couple');
for c = 1:numel(cfgs)
    for s = 1:numel(steps)
        o = struct('cfg', cfgs{c}, 'p', p, 'manoeuvre', steps{s}, 't_end', 15, ...
                   'mpc_opts', mpc_opts, 'plot', false);
        r = run_sim(o);
        m = r.metrics;
        R.(sprintf('c%d', c)).(steps{s}) = m;
        fprintf('%-6s %-11s %8.2f %8.2f %9.1f %12.3f\n', tags{c}, steps{s}, ...
                m.rise, m.settle, m.overshoot, max(m.coupling));
    end
end
fprintf('(NaN = never reached; coupling = largest error in the other 5 DOF, m or rad)\n');
end

%% ======================================================================
%  TEST 5: TURNING
%% ======================================================================
function R = test_turning(p, cfgs, tags, mpc_opts)
fprintf('\n=== TEST 5: TURNING (MPC) ===\n');
fprintf('%-6s %16s %18s %14s\n', 'cfg', 'peak r [deg/s]', 'circle RMSE [m]', 'circle sat');
for c = 1:numel(cfgs)
    o = struct('cfg', cfgs{c}, 'p', p, 'manoeuvre', 'step_yaw180', 't_end', 10, ...
               'mpc_opts', mpc_opts, 'plot', false);
    r1 = run_sim(o);
    o.manoeuvre = 'turn';  o.t_end = 2*pi*0.5/0.3 + 2;
    r2 = run_sim(o);
    R.peak_r(c)   = r1.metrics.peak_yaw_rate_dps;
    R.circle(c)   = r2.metrics.rmse_pos;
    R.circ_sat(c) = r2.metrics.sat_frac;
    fprintf('%-6s %16.1f %18.3f %13.0f%%\n', tags{c}, R.peak_r(c), R.circle(c), 100*R.circ_sat(c));
end
end

%% ======================================================================
%  TEST 6: PATH TRACKING
%% ======================================================================
function R = test_tracking(p, cfgs, tags, mpc_opts)
fprintf('\n=== TEST 6: PATH TRACKING (MPC) ===\n');
paths = {'survey', 'helix'};  tend = [50, 40];
fprintf('%-6s %-7s %10s %10s %10s %10s %8s\n', 'cfg', 'path', 'RMSE[m]', 'max[m]', ...
        'yaw[deg]', 'pitch[deg]', 'sat');
for c = 1:numel(cfgs)
    for k = 1:2
        o = struct('cfg', cfgs{c}, 'p', p, 'manoeuvre', paths{k}, 't_end', tend(k), ...
                   'mpc_opts', mpc_opts, 'plot', false);
        r = run_sim(o);  m = r.metrics;
        R.(paths{k})(c) = m;
        fprintf('%-6s %-7s %10.3f %10.3f %10.2f %10.2f %7.0f%%\n', tags{c}, paths{k}, ...
                m.rmse_pos, m.max_pos_err, m.rmse_yaw_deg, m.max_pitch_deg, 100*m.sat_frac);
    end
end
end

%% ======================================================================
%  TEST 7: STATION KEEPING UNDER CURRENT
%% ======================================================================
function R = test_disturbance(p, cfgs, tags, mpc_opts)
fprintf('\n=== TEST 7: STATION KEEPING, 0.2 m/s current step at t = 5 s (east) ===\n');
fprintf('%-6s %12s %14s %14s\n', 'cfg', 'max drift[m]', 'recovery[s]', 'ss error[m]');
for c = 1:numel(cfgs)
    o = struct('cfg', cfgs{c}, 'p', p, 'manoeuvre', 'hold', 't_end', 30, ...
               'mpc_opts', mpc_opts, 'plot', false, 'tether', false);
    o.current = @(t) (t >= 5) * [0; 0.2; 0];      % side current: tests sway authority
    r = run_sim(o);
    e = sqrt(sum((r.x(1:3,:) - r.eta_ref(1:3,:)).^2, 1));
    after = r.t >= 5;
    R.max_drift(c) = max(e(after));
    i_out = find(e > 0.05 & after, 1, 'last');
    if isempty(i_out), R.recovery(c) = 0;
    elseif i_out == numel(e), R.recovery(c) = NaN;
    else, R.recovery(c) = r.t(i_out+1) - 5;
    end
    R.ss_err(c) = mean(e(r.t >= r.t(end) - 5));
    fprintf('%-6s %12.3f %14.2f %14.3f\n', tags{c}, R.max_drift(c), R.recovery(c), R.ss_err(c));
end
fprintf('(recovery = time until error stays below 0.05 m; NaN = never)\n');
end

%% ======================================================================
%  TEST 8: THRUSTER FAILURE IN CLOSED LOOP
%% ======================================================================
function R = test_failure(p, cfgs, tags, mpc_opts)
fprintf('\n=== TEST 8: SURVEY WITH THRUSTER f1 FAILED (MPC informed) ===\n');
fprintf('%-6s %14s %14s %10s\n', 'cfg', 'RMSE ok [m]', 'RMSE fail [m]', 'increase');
for c = 1:numel(cfgs)
    o = struct('cfg', cfgs{c}, 'p', p, 'manoeuvre', 'survey', 't_end', 50, ...
               'mpc_opts', mpc_opts, 'plot', false);
    r0 = run_sim(o);
    o.failed = 1;
    r1 = run_sim(o);
    R.ok(c) = r0.metrics.rmse_pos;  R.fail(c) = r1.metrics.rmse_pos;
    fprintf('%-6s %14.3f %14.3f %9.1fx\n', tags{c}, R.ok(c), R.fail(c), R.fail(c)/R.ok(c));
end
end
