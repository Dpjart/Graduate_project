function [summary, runs] = run_rov_experiments(c, makePlots)
%RUN_ROV_EXPERIMENTS  Run every experiment for every thruster layout and
%   save results (ROV counterpart of the UAV run_experiments).
%
%   [summary, runs] = run_rov_experiments();          % defaults, with plots
%   [summary, runs] = run_rov_experiments(c, false);  % no figures
%   c = rov_config(); c.duration = 20; run_rov_experiments(c);
%
%   Outputs in c.outputDir (default .\results):
%     <layout>\experiment_k.mat        full signals ('out' struct)
%     <layout>\experiment_k.png/.fig   presentation figure per run
%     compare_experiment_k.png/.fig    layouts side by side per experiment
%     summary_rmse.png/.fig            RMSE bar chart, all runs
%     summary.csv, all_experiments.mat, analysis.md
%   Videos: make_rov_video(k) after this has run.
%
%   runs{i,k} = layout i, experiment k.

if nargin < 1 || isempty(c), c = rov_config(); end
if nargin < 2, makePlots = true; end
if ~exist(c.outputDir, 'dir'), mkdir(c.outputDir); end

p  = c.p;
nL = numel(c.layouts);
nE = numel(c.experimentNames);
runs = cell(nL, nE);
rows = {};

for i = 1:nL
    L   = c.layouts{i};
    cfg = L.build(p);
    dirL = fullfile(c.outputDir, L.tag);
    if ~exist(dirL, 'dir'), mkdir(dirL); end
    for k = 1:nE
        fprintf('Layout %s (%d/%d), experiment %d/%d: %s\n', L.tag, i, nL, k, nE, c.experimentNames{k});
        out = simulate_rov(c, cfg, L.tag, k);
        runs{i,k} = out;
        save(fullfile(dirL, sprintf('experiment_%d.mat', k)), 'out');
        if makePlots, plot_rov_experiment(out, dirL, false); end
        rows(end+1, :) = summary_row(out); %#ok<AGROW>
    end
end

%% Summary table / CSV
header = {'Layout','Experiment','Name','RMSE_X_m','RMSE_Y_m','RMSE_Z_m','RMSE_3D_m', ...
          'Maximum_3D_m','Final_3D_m','Initial_3D_m','Convergence_s','IAE_m_s', ...
          'DisturbanceMaximum_m','Recovery_s','RecoveryStatus','AfterFailure_RMSE_m', ...
          'MaxRoll_deg','MaxPitch_deg','SaturationFraction','Effort_N2s'};
write_csv(fullfile(c.outputDir, 'summary.csv'), header, rows);
summary = cell2struct(rows, header, 2);
if exist('struct2table', 'file') || exist('struct2table', 'builtin')
    summary = struct2table(summary);
end
save(fullfile(c.outputDir, 'all_experiments.mat'), 'summary', 'runs', 'c');

if makePlots, compare_rov_results(runs, c, false); end
write_analysis(fullfile(c.outputDir, 'analysis.md'), header, rows, c);
disp(summary);
fprintf('Saved results to %s\n', c.outputDir);
end

%% ======================================================================
function out = simulate_rov(c, cfg, tag, k)
o = struct('cfg', cfg, 'p', c.p, 'manoeuvre', c.manoeuvres{k}, 't_end', c.duration, ...
           'mpc_opts', c.mpc_opts, 'tether', c.tether, 'thruster_lag', c.thruster_lag, ...
           'dt_sim', c.dt_sim, 'plot', false);
window = [];  event = [];
switch k
    case 3
        w = c.disturbanceWindow;  v = c.current(:);
        o.current = @(t) (t >= w(1) && t < w(2)) * v;
        window = w;
    case 4
        [er0, ~] = trajectories(c.manoeuvres{k}, 0);
        o.x0 = [er0(1:3) + c.initialOffset(:); 0; 0; er0(6) + c.initialYawOffset; zeros(6,1)];
    case 5
        o.failed = c.failThruster;  o.fail_time = c.failTime;
        event = c.failTime;
end
r = run_sim(o);

out.experiment     = k;
out.experimentName = c.experimentNames{k};
out.layout         = tag;
out.cfg            = cfg;
out.params         = c.p;
out.time           = r.t;
out.state          = r.x;                    % [x y z phi theta psi u v w p q r] (NED / body)
out.reference      = r.eta_ref(1:3, :);
out.referencePose  = r.eta_ref;
out.thrust         = r.f_act;                % applied thruster forces [N]
out.thrustCommand  = r.f_cmd;
out.tau            = r.tau;                  % body forces/moments [X Y Z K M N]
out.current        = r.current;              % NED current [m/s]
out.disturbanceWindow = window;
out.failed         = o_field(o, 'failed', []);
out.failTime       = o_field(o, 'fail_time', []);
out.error          = out.reference - out.state(1:3, :);
out.error3D        = sqrt(sum(out.error.^2, 1));
out.metrics        = rov_metrics(out.time, out.error, window, ...
                                 c.recoveryTolerance, c.recoveryDwell, event);
out.metrics.maxRoll_deg  = rad2deg(max(abs(out.state(4,:))));
out.metrics.maxPitch_deg = rad2deg(max(abs(out.state(5,:))));
out.metrics.saturation   = r.metrics.sat_frac;
out.metrics.effort       = r.metrics.effort;
out.config = c;
end

function v = o_field(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function row = summary_row(out)
m = out.metrics;
row = {out.layout, out.experiment, out.experimentName, m.rmse(1), m.rmse(2), m.rmse(3), ...
       m.rmse3D, m.maximum, m.final, m.initial, m.convergence, m.IAE, ...
       m.disturbanceMaximum, m.recovery, m.recoveryStatus, m.eventRMSE, ...
       m.maxRoll_deg, m.maxPitch_deg, m.saturation, m.effort};
end

function write_csv(file, header, rows)
fid = fopen(file, 'w');
assert(fid >= 0, 'Could not open %s', file);
fprintf(fid, '%s\n', strjoin(header, ','));
for r = 1:size(rows, 1)
    parts = cell(1, size(rows, 2));
    for j = 1:size(rows, 2)
        v = rows{r, j};
        if ischar(v), parts{j} = v; else, parts{j} = sprintf('%.6g', v); end
    end
    fprintf(fid, '%s\n', strjoin(parts, ','));
end
fclose(fid);
end

function write_analysis(file, header, rows, c)
fid = fopen(file, 'w');
assert(fid >= 0, 'Could not open %s', file);
fprintf(fid, '# ROV thruster-layout comparison\n\n');
fprintf(fid, 'Layouts: %s. Same hull, MPC weights and thruster rating for every layout.\n', ...
        strjoin(cellfun(@(L) L.tag, c.layouts, 'UniformOutput', false), ', '));
fprintf(fid, 'Duration %.0f s per experiment. Current %.2f m/s (NED [%g %g %g]) on [%g, %g) s in experiment 3. ', ...
        c.duration, norm(c.current), c.current, c.disturbanceWindow);
fprintf(fid, 'Experiment 4 starts offset by [%g %g %g] m and %.2f rad yaw. ', c.initialOffset, c.initialYawOffset);
fprintf(fid, 'Experiment 5: thruster f%d fails at %g s.\n\n', c.failThruster, c.failTime);
fprintf(fid, 'Errors are reference minus true state. RMSE includes the start-up transient. ');
fprintf(fid, ['Convergence = time after which the 3D error stays within %.2f m until the end (NaN = ends outside). ' ...
              'Recovery = time after the current stops until the error stays within %.2f m for %.1f s.\n\n'], ...
        c.recoveryTolerance, c.recoveryTolerance, c.recoveryDwell);
col = @(name) find(strcmp(header, name));
for k = 1:numel(c.experimentNames)
    fprintf(fid, '## Experiment %d: %s\n\n', k, c.experimentNames{k});
    fprintf(fid, '| Layout | 3D RMSE (m) | Max (m) | Final (m) | Convergence (s) | Recovery (s) | After-failure RMSE (m) | Max pitch (deg) | Saturation |\n');
    fprintf(fid, '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n');
    for r = 1:size(rows, 1)
        if rows{r, col('Experiment')} ~= k, continue; end
        fprintf(fid, '| %s | %.4f | %.4f | %.4f | %.2f | %.2f | %.4f | %.2f | %.0f%% |\n', ...
            rows{r, col('Layout')}, rows{r, col('RMSE_3D_m')}, rows{r, col('Maximum_3D_m')}, ...
            rows{r, col('Final_3D_m')}, rows{r, col('Convergence_s')}, rows{r, col('Recovery_s')}, ...
            rows{r, col('AfterFailure_RMSE_m')}, rows{r, col('MaxPitch_deg')}, ...
            100*rows{r, col('SaturationFraction')});
    end
    fprintf(fid, '\n');
end
fprintf(fid, 'NaN = not applicable or not reached. Full signals: <layout>/experiment_k.mat.\n');
fclose(fid);
end
