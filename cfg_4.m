function cfg = cfg_4(p, z_off)
%CFG_4  4-thruster layout: one thruster on each side face of the frame.
%   cfg = cfg_4(p)          thrusters at CoG height (z = 0)
%   cfg = cfg_4(p, z_off)   all thrusters shifted vertically by z_off [m]
%                           (FRD: negative = above CoG, e.g. -0.10 = top-mounted)
%
%   Layout (top view, x forward, y right):
%
%                 f3 (front, vertical)
%                 +--------------+
%                 |              |
%   f2 (left,  -> |      CoG     | <- f1 (right,
%   horizontal)   |              |    horizontal)
%                 +--------------+
%                 f4 (rear, vertical)
%
%   f1, f2 : point +x (forward)  -> surge X and yaw N
%   f3, f4 : point +z (down)     -> heave Z and pitch M
%   No sway (Y) or roll (K) authority; roll is passively stabilised by BG.
%
%   Positive f_i = thrust along e_i. Reverse thrust is limited by Fmax_rev.
%
%   Output struct:
%     cfg.name  - label
%     cfg.n     - number of thrusters
%     cfg.r     - 3xn thruster positions relative to CoG, body frame [m]
%     cfg.e     - 3xn unit thrust directions, body frame
%     cfg.fmax  - nx1 max forward thrust [N]
%     cfg.fmin  - nx1 max reverse thrust (negative) [N]

if nargin < 2, z_off = 0; end

hx = p.L/2;   % 0.15 m : front/rear face
hy = p.W/2;   % 0.10 m : left/right face

cfg.name = sprintf('4-thruster (side faces, z_{off} = %.2f m)', z_off);
cfg.n    = 4;

%            f1     f2     f3     f4
cfg.r = [    0,     0,     hx,   -hx ;     % x
             hy,   -hy,    0,     0  ;     % y
           z_off, z_off, z_off, z_off ];   % z

cfg.e = [    1,     1,     0,     0  ;     % x
             0,     0,     0,     0  ;     % y
             0,     0,     1,     1  ];    % z

cfg.fmax =  p.Fmax_fwd * ones(cfg.n, 1);
cfg.fmin = -p.Fmax_rev * ones(cfg.n, 1);

end
