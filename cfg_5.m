function cfg = cfg_5(p, z_off, x_L)
%CFG_5  5-thruster layout: cfg_4 + one lateral (sway) thruster.
%   cfg = cfg_5(p)              all thrusters at CoG height, lateral at x = 0
%   cfg = cfg_5(p, z_off)       all thrusters shifted vertically by z_off [m]
%   cfg = cfg_5(p, z_off, x_L)  lateral thruster moved fore/aft to x = x_L [m]
%
%   f1..f4 : identical to cfg_4 (same vertical pair => fair comparison)
%   f5     : lateral, points +y (right) at (x_L, 0, z_off)
%            -> sway Y; couples into yaw (N = x_L*f5) if x_L ~= 0
%               and into roll (K = -z_off*f5) if z_off ~= 0
%
%   Actuated: surge, sway, heave, pitch, yaw. Roll passive (BG).

if nargin < 2, z_off = 0; end
if nargin < 3, x_L   = 0; end

cfg = cfg_4(p, z_off);
cfg.name = sprintf('5-thruster (z_{off} = %.2f m, x_L = %.2f m)', z_off, x_L);
cfg.n    = 5;

cfg.r(:, 5) = [x_L; 0; z_off];
cfg.e(:, 5) = [0; 1; 0];

cfg.fmax =  p.Fmax_fwd * ones(cfg.n, 1);
cfg.fmin = -p.Fmax_rev * ones(cfg.n, 1);

end
