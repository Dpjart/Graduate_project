function videoPath = make_rov_video(source, varargin)
%MAKE_ROV_VIDEO  Animate saved runs as a 3D ROV MP4 (no simulation rerun).
%
%   make_rov_video(3)                        experiment 3, all layouts side by side
%   make_rov_video(3, 'Layouts', {'4','6_45'})  chosen layouts only
%   make_rov_video('results/5/experiment_2.mat')   one run
%   make_rov_video(out)                      one 'out' struct
%   make_rov_video({outA, outB})             any runs side by side
%
%   Options (name/value):
%     'FrameRate'      30
%     'PlaybackSpeed'  1          (2 = twice real time)
%     'TimeRange'      []         e.g. [5 20]
%     'OutputFile'     ''         default results/experiment_k_<layouts>.mp4
%     'Visible'        false
%     'Layouts'        {}         layout tags when source is an experiment number
%
%   Each panel shows the frame, the tube, every thruster with a live thrust
%   arrow (green = forward, orange = reverse, length ~ |f|/Fmax), the reference
%   path, the actual trail and a status readout. Failed thrusters turn red.
%   Axes are NED drawn right-handed: depth increases downward.
%   Requires base MATLAB with MPEG-4 VideoWriter support.

root = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(source), source = 3; end
ps = inputParser;
addParameter(ps, 'FrameRate', 30);
addParameter(ps, 'PlaybackSpeed', 1);
addParameter(ps, 'TimeRange', []);
addParameter(ps, 'OutputFile', '');
addParameter(ps, 'Visible', false);
addParameter(ps, 'Layouts', {});
parse(ps, varargin{:});  opt = ps.Results;

%% Load runs
outs = load_runs(source, opt.Layouts, root);
P = numel(outs);
k = outs{1}.experiment;

tEnd = min(cellfun(@(o) o.time(end), outs));
interval = [outs{1}.time(1), tEnd];
if ~isempty(opt.TimeRange), interval = opt.TimeRange(:)'; end
nF = max(1, ceil(diff(interval) / opt.PlaybackSpeed * opt.FrameRate));
tf = interval(1) + (0:nF-1)' * opt.PlaybackSpeed / opt.FrameRate;

%% Output path
videoPath = char(opt.OutputFile);
if isempty(videoPath)
    tags = strjoin(cellfun(@(o) o.layout, outs, 'UniformOutput', false), '_vs_');
    videoPath = fullfile(root, 'results', sprintf('experiment_%d_%s.mp4', k, tags));
end
[folder, name] = fileparts(videoPath);
if isempty(folder), folder = pwd; videoPath = fullfile(folder, [name '.mp4']); end
if ~exist(folder, 'dir'), mkdir(folder); end

%% Figure layout
if P == 1, figSize = [1280 720]; else, figSize = [1920 1080]; end
vis = 'off';  if opt.Visible, vis = 'on'; end
fig = figure('Visible', vis, 'Color', [0.96 0.97 0.99], 'Position', [40 40 figSize], ...
             'MenuBar', 'none', 'ToolBar', 'none', 'Resize', 'off', 'Name', 'ROV animation');
cleanupFig = onCleanup(@() close(fig));

% shared limits so panels are comparable
allP = [];
for i = 1:P, allP = [allP, outs{i}.state(1:3,:), outs{i}.reference]; end %#ok<AGROW>
pad = 0.3;
lo = min(allP, [], 2) - pad;  hi = max(allP, [], 2) + pad;

titleBox = annotation(fig, 'textbox', [0.02 0.93 0.96 0.06], 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 15, 'Interpreter', 'none');
footer = annotation(fig, 'textbox', [0.02 0.0 0.96 0.035], 'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 10, 'Color', [0.3 0.33 0.4], 'Interpreter', 'none');

w = 0.96 / P;
S = struct([]);
for i = 1:P
    o = outs{i};
    if P == 1        % UAV-video style: scene left, readout right
        axPos  = [0.06 0.12 0.66 0.76];  hudPos = [0.76 0.25 0.22 0.50];  zoomF = 1.0;
    else             % side by side: readout under each scene
        axPos  = [0.02 + (i-1)*w + 0.03*w, 0.22, 0.94*w, 0.68];
        hudPos = [0.02 + (i-1)*w + 0.03*w, 0.05, 0.94*w, 0.16];  zoomF = 1.0;
    end
    ax = axes(fig, 'Position', axPos); hold(ax, 'on');
    S(i).ax = ax;
    S(i).ref = plot3(ax, o.reference(1,:), o.reference(2,:), o.reference(3,:), '--', ...
                     'Color', [0.45 0.48 0.52], 'LineWidth', 1.3);
    S(i).trail = animatedline(ax, 'Color', [0.05 0.42 0.82], 'LineWidth', 2);
    S(i).target = plot3(ax, o.reference(1,1), o.reference(2,1), o.reference(3,1), 'o', ...
                        'Color', [0.05 0.65 0.35], 'MarkerSize', 7, 'LineWidth', 1.5);
    S(i).veh = create_rov_graphics(ax, o.cfg, o.params);
    set(ax, 'YDir', 'reverse', 'ZDir', 'reverse');
    xlim(ax, [lo(1) hi(1)]); ylim(ax, [lo(2) hi(2)]); zlim(ax, [lo(3) hi(3)]);
    daspect(ax, [1 1 1]); axis(ax, 'vis3d'); grid(ax, 'on'); view(ax, 35, 25); camzoom(ax, zoomF);
    xlabel(ax, 'North x (m)'); ylabel(ax, 'East y (m)'); zlabel(ax, 'Depth z (m)');
    title(ax, sprintf('Layout %s  (%d thrusters)', o.layout, o.cfg.n), 'Interpreter', 'none', 'FontSize', 13);
    camlight(ax, 'headlight'); lighting(ax, 'gouraud');
    S(i).hud = annotation(fig, 'textbox', hudPos, ...
        'EdgeColor', [0.8 0.82 0.86], 'BackgroundColor', 'w', 'FontName', 'Consolas', ...
        'FontSize', 11, 'Interpreter', 'none', 'VerticalAlignment', 'top');

    % resample to frame times
    S(i).pos = interp1(o.time, o.state(1:3,:)', tf, 'linear');
    S(i).att = interp1(o.time, unwrap(o.state(4:6,:)', [], 1), tf, 'linear');
    S(i).ref_now = interp1(o.time, o.reference', tf, 'linear');
    S(i).f   = interp1(o.time, o.thrust', tf, 'linear');
end
% Arrow scale shared by all panels: the largest thrust in the clip draws
% L_arrow long (forces are often far below Fmax, so Fmax would hide them).
Fs = max(cellfun(@(s) max(abs(s(:))), {S.f}));
Fs = max(Fs, 1e-3);
for i = 1:P, S(i).Fs = Fs; end
set(footer, 'String', sprintf(['Pose and thrust from simulated data | red bar = front | arrows: ' ...
    'green forward, orange reverse, 0.2 m = %.1f N (largest thrust in clip) | red thruster = failed'], Fs));

%% Render
writer = VideoWriter(videoPath, 'MPEG-4');
writer.FrameRate = opt.FrameRate;  writer.Quality = 95;
open(writer);  cleanupW = onCleanup(@() close(writer));
L_arrow = 0.20;                                   % arrow length at largest thrust [m]
fprintf('Rendering %d frames to %s\n', nF, videoPath);
for kf = 1:nF
    t = tf(kf);
    o1 = outs{1};
    curOn = ~isempty(o1.disturbanceWindow) && t >= o1.disturbanceWindow(1) && t < o1.disturbanceWindow(2);
    status = '';
    if ~isempty(o1.disturbanceWindow)
        if curOn, status = sprintf(' | CURRENT ON %.2f m/s', norm(o1.config.current));
        else,     status = ' | current off';
        end
    end
    if ~isempty(o1.failTime) && ~isempty(o1.failed)
        if t >= o1.failTime, status = sprintf(' | THRUSTER f%d FAILED', o1.failed(1));
        else,                status = sprintf(' | f%d fails at %.0f s', o1.failed(1), o1.failTime);
        end
    end
    set(titleBox, 'String', sprintf('Experiment %d: %s | t = %6.2f s | %.1fx%s', ...
                              k, o1.experimentName, t, opt.PlaybackSpeed, status));

    for i = 1:P
        o = outs{i};  V = S(i).veh;
        pos = S(i).pos(kf,:)';  att = S(i).att(kf,:);
        [R, ~] = rov_kinematics(att(1), att(2), att(3));
        set(V.body, 'Matrix', [R pos; 0 0 0 1]);
        failedNow = ~isempty(o.failed) && ~isempty(o.failTime) && t >= o.failTime;
        for j = 1:o.cfg.n
            fj = S(i).f(kf, j);
            e  = o.cfg.e(:,j) / norm(o.cfg.e(:,j));
            r0 = o.cfg.r(:,j);
            r1 = r0 + e * (fj / S(i).Fs) * L_arrow;
            w0 = R*r0 + pos;  w1 = R*r1 + pos;           % body -> NED
            if fj >= 0, col = [0.1 0.7 0.2]; else, col = [0.95 0.45 0.1]; end
            set(V.arrow(j), 'XData', [w0(1) w1(1)], 'YData', [w0(2) w1(2)], 'ZData', [w0(3) w1(3)], 'Color', col);
            set(V.tip(j), 'XData', w1(1), 'YData', w1(2), 'ZData', w1(3), 'Color', col);
            if failedNow && any(o.failed == j)
                set(V.thruster(j), 'FaceColor', [0.9 0.1 0.1]);
                set([V.arrow(j) V.tip(j)], 'Visible', 'off');
            end
        end
        addpoints(S(i).trail, pos(1), pos(2), pos(3));
        rn = S(i).ref_now(kf,:);
        set(S(i).target, 'XData', rn(1), 'YData', rn(2), 'ZData', rn(3));
        ang = rad2deg(att);
        fl = '';
        for j = 1:o.cfg.n
            fl = [fl sprintf('f%d %+6.1f  ', j, S(i).f(kf,j))]; %#ok<AGROW>
            if mod(j, 3) == 0, fl = [fl newline]; end %#ok<AGROW>
        end
        set(S(i).hud, 'String', sprintf(['3D error: %.3f m\nPosition: %6.2f %6.2f %6.2f m\n' ...
            'Roll %6.1f  Pitch %6.1f  Yaw %7.1f deg\n\nThrust (N):\n%s'], ...
            norm(rn' - pos), pos, ang, fl));
    end
    drawnow;
    frame = getframe(fig);
    px = frame.cdata;                     % undo Windows display scaling
    frame.cdata = px(round(linspace(1, size(px,1), figSize(2))), ...
                     round(linspace(1, size(px,2), figSize(1))), :);
    writeVideo(writer, frame);
    if kf == max(1, round(nF/2)), imwrite(frame.cdata, fullfile(folder, [name '_preview.png'])); end
    if mod(kf, max(1, round(nF/4))) == 0, fprintf('Rendered %d/%d frames\n', kf, nF); end
end
close(writer);  clear cleanupW;
fprintf('Saved %s (%.1f s of video).\n', videoPath, nF / opt.FrameRate);
end

%% ======================================================================
function outs = load_runs(source, layouts, root)
if isnumeric(source)
    k = source;
    c = rov_config();
    tags = cellfun(@(L) L.tag, c.layouts, 'UniformOutput', false);
    if ~isempty(layouts), tags = layouts; end
    outs = cell(1, numel(tags));
    for i = 1:numel(tags)
        f = fullfile(root, 'results', tags{i}, sprintf('experiment_%d.mat', k));
        assert(exist(f, 'file') == 2, 'Missing %s - run run_rov_experiments first.', f);
        s = load(f, 'out');  outs{i} = s.out;
    end
elseif ischar(source) || isstring(source)
    s = load(char(source), 'out');  outs = {s.out};
elseif iscell(source)
    outs = source(:)';
else
    outs = {source};
end
end
