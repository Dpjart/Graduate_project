function f = plot_rov_experiment(out, outputDir, showFigure)
%PLOT_ROV_EXPERIMENT  One presentation figure per run, saved as PNG and FIG.
%   plot_rov_experiment(out, outputDir)          saves, figure hidden
%   plot_rov_experiment(out, outputDir, true)    also leaves it open
%   Panels: 3D path | position vs time | tracking error
%           thruster forces | body forces/moments | Euler angles

if nargin < 3, showFigure = false; end
vis = 'off';  if showFigure, vis = 'on'; end
t = out.time;  X = out.state;  R = out.reference;
f = figure('Visible', vis, 'Color', 'w', 'Position', [50 50 1450 900]);
% Fixed layout instead of tiledlayout + sgtitle: the 3D panel can push its
% title into the figure title. Each panel gets an OuterPosition cell (title
% and labels kept inside it); the figure title has its own strip at the top.
titleH = 0.05;  rowH = (1 - titleH) / 2;
cell_ = @(r, col) [(col-1)/3, (2-r)*rowH, 1/3, rowH];
annotation(f, 'textbox', [0 1-titleH 1 titleH], 'String', ...
    sprintf('Layout %s | Experiment %d: %s | 3D RMSE %.4f m', ...
            out.layout, out.experiment, out.experimentName, out.metrics.rmse3D), ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
    'FontWeight', 'bold', 'FontSize', 13, 'Interpreter', 'none');

% 1) 3D path (NED: depth drawn downward)
axes(f, 'OuterPosition', cell_(1, 1));
plot3(R(1,:), R(2,:), R(3,:), 'k--', 'LineWidth', 1.5); hold on;
plot3(X(1,:), X(2,:), X(3,:), 'Color', [0 .4 .8], 'LineWidth', 1.2);
plot3(X(1,1), X(2,1), X(3,1), 'o', 'Color', [0 .4 .8], 'MarkerFaceColor', [0 .4 .8]);
ned_axes(gca); title('3D trajectory');
legend('Reference', 'Actual', 'Start', 'Location', 'southoutside', 'Orientation', 'horizontal', 'Box', 'off');

% 2) positions
axes(f, 'OuterPosition', cell_(1, 2)); colors = lines(3); hold on;
for j = 1:3
    plot(t, R(j,:), '--', 'Color', colors(j,:));
    plot(t, X(j,:), 'Color', colors(j,:));
end
mark_events(out); grid on; xlabel('Time (s)'); ylabel('Position (m)');
legend('x ref','x','y ref','y','z ref','z (depth)', 'Location', 'best');
title('Position');

% 3) tracking error
axes(f, 'OuterPosition', cell_(1, 3));
plot(t, out.error); hold on; plot(t, out.error3D, 'k', 'LineWidth', 1.2);
mark_events(out); grid on; xlabel('Time (s)'); ylabel('Error (m)');
legend('x','y','z','3D', 'Location', 'best'); title('Tracking error (ref - actual)');

% 4) thruster forces with limits
axes(f, 'OuterPosition', cell_(2, 1)); hold on;
n = out.cfg.n;
plot(t, out.thrust', 'LineWidth', 1);
yline(max(out.cfg.fmax), ':', 'HandleVisibility', 'off');
yline(min(out.cfg.fmin), ':', 'HandleVisibility', 'off');
mark_events(out); grid on; xlabel('Time (s)'); ylabel('Thrust (N)');
legend(arrayfun(@(i) sprintf('f%d', i), 1:n, 'UniformOutput', false), 'Location', 'best');
title(sprintf('Thruster forces (%d thrusters)', n));

% 5) body forces and moments
axes(f, 'OuterPosition', cell_(2, 2)); hold on;
plot(t, out.tau(1:3,:)', 'LineWidth', 1);
plot(t, out.tau(4:6,:)', '--', 'LineWidth', 1);
mark_events(out); grid on; xlabel('Time (s)'); ylabel('Force (N) / moment (N m)');
legend('X surge','Y sway','Z heave','K roll','M pitch','N yaw', 'Location', 'best');
title('Body forces and moments  \tau = T f');

% 6) attitude
axes(f, 'OuterPosition', cell_(2, 3));
plot(t, rad2deg(X(4:5,:))); hold on;
plot(t, rad2deg(unwrap(X(6,:))));
plot(t, rad2deg(unwrap(out.referencePose(6,:))), 'k--');
mark_events(out); grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
legend('roll','pitch','yaw','yaw ref', 'Location', 'best'); title('Attitude');

if nargin >= 2 && ~isempty(outputDir)
    name = fullfile(outputDir, sprintf('experiment_%d', out.experiment));
    savefig(f, [name '.fig']);
    exportgraphics(f, [name '.png'], 'Resolution', 150);
end
if ~showFigure, close(f); end
end

function ned_axes(ax)
set(ax, 'YDir', 'reverse', 'ZDir', 'reverse');   % NED shown right-handed, depth down
grid(ax, 'on'); view(ax, 35, 25);
% x and y at true scale; depth stretched when its range is small next to the
% horizontal one (e.g. the straight line), so its ticks stay readable.
axis(ax, 'tight');
lx = xlim(ax); ly = ylim(ax); lz = zlim(ax);
xyr = max(diff(lx), diff(ly));  zr = max(diff(lz), 1e-3);
zs  = min(1, zr / (0.35 * xyr));
daspect(ax, [1 1 zs]);
zlab = 'Depth z (m)';
if zs < 1, zlab = sprintf('Depth z (m, \\times%.0f)', 1/zs); end
xlabel(ax, 'North x (m)'); ylabel(ax, 'East y (m)'); zlabel(ax, zlab);
end

function mark_events(out)
if ~isempty(out.disturbanceWindow)
    xline(out.disturbanceWindow(1), ':', 'current on',  'HandleVisibility', 'off');
    xline(out.disturbanceWindow(2), ':', 'current off', 'HandleVisibility', 'off');
end
if ~isempty(out.failTime) && ~isempty(out.failed)
    xline(out.failTime, '-r', sprintf('f%d fails', out.failed(1)), 'HandleVisibility', 'off');
end
end
