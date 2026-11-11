function v = create_rov_graphics(ax, cfg, p)
%CREATE_ROV_GRAPHICS  Procedural ROV model in body coordinates (FRD, metres).
%   v = create_rov_graphics(ax, cfg, p)
%   Aluminium-extrusion box frame (p.L x p.W x p.H), central tube along x,
%   one cylinder per thruster at cfg.r along cfg.e, and a thrust arrow per
%   thruster that make_rov_video updates every frame.
%   Red bar = front (+x), top edge.
%
%   v.body      hgtransform - set v.body.Matrix to move the vehicle
%   v.thruster  surface handle per thruster (recoloured when failed)
%   v.arrow     line handle per thruster (thrust arrow, world coordinates)
%   v.tip       line handle per thruster (arrow tip marker, world coordinates)

v.body = hgtransform('Parent', ax);
B = v.body;
L = p.L;  W = p.W;  H = p.H;  s = 0.02;          % 20 mm extrusion
alu = [0.72 0.74 0.78];

% 12 frame edges
for yy = [-1 1], for zz = [-1 1]
    box(B, [0, yy*W/2, zz*H/2], [L+s, s, s], alu);           % along x
end, end
for xx = [-1 1], for zz = [-1 1]
    box(B, [xx*L/2, 0, zz*H/2], [s, W, s], alu);             % along y
end, end
for xx = [-1 1], for yy = [-1 1]
    box(B, [xx*L/2, yy*W/2, 0], [s, s, H], alu);             % along z
end, end
% Front marker: top-front edge (FRD: top is -z)
box(B, [L/2 + 0.012, 0, -H/2], [0.012, 0.6*W, 0.03], [0.85 0.15 0.1]);

% Central pressure tube along x
[cx, cy, cz] = cylinder(0.05, 24);
len = 0.8*L;
surf('Parent', B, 'XData', (cz - 0.5)*len, 'YData', cx, 'ZData', cy, ...
     'FaceColor', [0.55 0.75 0.95], 'FaceAlpha', 0.6, 'EdgeColor', 'none');

% Thrusters + thrust arrows
n = cfg.n;
v.thruster = gobjects(n, 1);  v.arrow = gobjects(n, 1);  v.tip = gobjects(n, 1);
[tx, ty, tz] = cylinder(0.022, 16);
tl = 0.05;
for i = 1:n
    e  = cfg.e(:,i) / norm(cfg.e(:,i));
    Rt = align_z_to(e);
    P  = Rt * [tx(:)'; ty(:)'; (tz(:)' - 0.5)*tl] + cfg.r(:,i);
    v.thruster(i) = surf('Parent', B, ...
        'XData', reshape(P(1,:), size(tx)), 'YData', reshape(P(2,:), size(tx)), ...
        'ZData', reshape(P(3,:), size(tx)), 'FaceColor', [0.2 0.22 0.26], 'EdgeColor', 'none');
    % arrows live in the axes (world/NED coordinates), updated by make_rov_video
    v.arrow(i) = line('Parent', ax, 'XData', [0 0], 'YData', [0 0], 'ZData', [0 0], ...
        'LineWidth', 3, 'Color', [0.1 0.7 0.2]);
    v.tip(i) = line('Parent', ax, 'XData', 0, 'YData', 0, 'ZData', 0, ...
        'Marker', '.', 'MarkerSize', 22, 'Color', [0.1 0.7 0.2]);
end
end

function R = align_z_to(e)
% Rotation that maps [0 0 1] onto unit vector e.
z = [0; 0; 1];
c = dot(z, e);
if c > 1 - 1e-9, R = eye(3); return; end
if c < -1 + 1e-9, R = diag([1 -1 -1]); return; end
k = cross(z, e);  sn = norm(k);  k = k / sn;
K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
R = eye(3) + sn*K + (1 - c)*(K*K);
end

function box(parent, centre, dims, color)
V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* (dims/2) + centre;
F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Parent', parent, 'Vertices', V, 'Faces', F, 'FaceColor', color, ...
      'EdgeColor', 'none', 'FaceLighting', 'gouraud');
end
