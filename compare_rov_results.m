function compare_rov_results(runs, c, showFigures)
%COMPARE_ROV_RESULTS  Side-by-side layout comparison figures.
%   compare_rov_results()                 loads results/all_experiments.mat
%   compare_rov_results(runs, c)          from run_rov_experiments output
%   compare_rov_results(runs, c, true)    leave figures open
%
%   Saves in c.outputDir:
%     compare_experiment_k.png/.fig  top: 3D path per layout (shared axes)
%                                    bottom: 3D error vs time, all layouts
%     summary_rmse.png/.fig          RMSE and max error, all layouts x experiments

if nargin < 1 || isempty(runs)
    s = load(fullfile(fileparts(mfilename('fullpath')), 'results', 'all_experiments.mat'), 'runs', 'c');
    runs = s.runs;  c = s.c;
end
if nargin < 3, showFigures = true; end
vis = 'off';  if showFigures, vis = 'on'; end
[nL, nE] = size(runs);
colors = lines(nL);
tags = cellfun(@(o) o.layout, runs(:,1), 'UniformOutput', false);
% Line styles so identical runs stay visible when they overlap:
% earlier layouts drawn wider underneath, later ones narrower on top.
widths = linspace(3.2, 1.2, nL);
styles = repmat({'-'}, 1, nL);  if nL >= 3, styles{end} = '--'; end

%% Per experiment: paths + error overlay
for k = 1:nE
    f = figure('Visible', vis, 'Color', 'w', 'Position', [40 60 1700 1000]);
    % Fixed layout instead of tiledlayout: 3D axes push their titles out of
    % their tile and into the figure title. Each axes gets an OuterPosition
    % band, which MATLAB keeps its title and labels inside, and the figure
    % title gets its own strip at the top that no axes can reach.
    titleH = 0.05;  topY = 0.47;  topH = 1 - titleH - topY;
    annotation(f, 'textbox', [0 1-titleH 1 titleH], 'String', ...
        sprintf('Experiment %d: %s | same reference, controller and hull', k, c.experimentNames{k}), ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontWeight', 'bold', 'FontSize', 13, 'Interpreter', 'none');
    allP = [];
    for i = 1:nL, allP = [allP, runs{i,k}.state(1:3,:), runs{i,k}.reference]; end %#ok<AGROW>
    lo = min(allP, [], 2) - 0.15;  hi = max(allP, [], 2) + 0.15;
    % Keep x and y at true scale (circle radius stays honest) but stretch depth
    % when it is small next to the horizontal extent, so its ticks are readable.
    xyr = max(hi(1:2) - lo(1:2));  zr = hi(3) - lo(3);
    zs  = min(1, zr / (0.35 * xyr));            % < 1 = depth exaggerated
    zlab = 'Depth z (m)';
    if zs < 1, zlab = sprintf('Depth z (m, \\times%.0f)', 1/zs); end
    for i = 1:nL
        o = runs{i,k};
        axes(f, 'OuterPosition', [(i-1)/nL, topY, 1/nL, topH]);
        plot3(o.reference(1,:), o.reference(2,:), o.reference(3,:), 'k--', 'LineWidth', 1.5); hold on;
        plot3(o.state(1,:), o.state(2,:), o.state(3,:), 'Color', colors(i,:), 'LineWidth', 1.2);
        plot3(o.state(1,1), o.state(2,1), o.state(3,1), 'o', 'Color', colors(i,:), ...
              'MarkerFaceColor', colors(i,:));
        set(gca, 'YDir', 'reverse', 'ZDir', 'reverse'); grid on;
        xlim([lo(1) hi(1)]); ylim([lo(2) hi(2)]); zlim([lo(3) hi(3)]);
        daspect([1 1 zs]); view(35, 25);
        xlabel('North x (m)'); ylabel('East y (m)'); zlabel(zlab);
        title(sprintf('Layout %s (%d thrusters)\n3D RMSE %.4f m | max %.3f m', ...
              o.layout, o.cfg.n, o.metrics.rmse3D, o.metrics.maximum), 'Interpreter', 'none');
        legend('Reference', 'Actual', 'Start', 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');
    end
    axes(f, 'OuterPosition', [0 0 1 topY]); hold on;
    for i = 1:nL
        plot(runs{i,k}.time, runs{i,k}.error3D, styles{i}, 'Color', colors(i,:), ...
             'LineWidth', widths(i));
    end
    o = runs{1,k};
    if ~isempty(o.disturbanceWindow)
        xline(o.disturbanceWindow(1), ':', 'current on', 'HandleVisibility', 'off');
        xline(o.disturbanceWindow(2), ':', 'current off', 'HandleVisibility', 'off');
    end
    if ~isempty(o.failTime) && ~isempty(o.failed)
        xline(o.failTime, '-r', sprintf('f%d fails', o.failed(1)), 'HandleVisibility', 'off');
    end
    yline(c.recoveryTolerance, '--', 'tolerance', 'HandleVisibility', 'off');
    grid on; xlabel('Time (s)'); ylabel('3D error (m)');
    legend(strcat('Layout', {' '}, tags), 'Location', 'best', 'Interpreter', 'none');
    title('3D tracking error, all layouts');
    save_fig(f, c.outputDir, sprintf('compare_experiment_%d', k), showFigures);
end

%% Summary bars
rmse = zeros(nE, nL);  mx = zeros(nE, nL);
for i = 1:nL
    for k = 1:nE
        rmse(k,i) = runs{i,k}.metrics.rmse3D;
        mx(k,i)   = runs{i,k}.metrics.maximum;
    end
end
f = figure('Visible', vis, 'Color', 'w', 'Position', [60 60 1400 600]);
t = tiledlayout(1, 2, 'TileSpacing', 'compact');
labels = arrayfun(@(k) sprintf('E%d %s', k, c.experimentNames{k}), 1:nE, 'UniformOutput', false);
nexttile; b = bar(max(rmse, 1e-6)); set(gca, 'YScale', 'log', 'XTickLabel', labels); xtickangle(20);
for i = 1:nL, set(b(i), 'FaceColor', colors(i,:)); end
grid on; ylabel('3D RMSE (m, log scale)'); title('3D RMSE');
legend(strcat('Layout', {' '}, tags), 'Location', 'northwest', 'Interpreter', 'none');
nexttile; b = bar(max(mx, 1e-6)); set(gca, 'YScale', 'log', 'XTickLabel', labels); xtickangle(20);
for i = 1:nL, set(b(i), 'FaceColor', colors(i,:)); end
grid on; ylabel('Maximum 3D error (m, log scale)'); title('Maximum error');
title(t, 'Thruster layout comparison, all experiments', 'FontWeight', 'bold');
save_fig(f, c.outputDir, 'summary_rmse', showFigures);
end

function save_fig(f, dir, name, keep)
savefig(f, fullfile(dir, [name '.fig']));
exportgraphics(f, fullfile(dir, [name '.png']), 'Resolution', 140);
if ~keep, close(f); end
end
