function cfg = cfg_6(p, alpha_deg, z_off)
%CFG_6  6-thruster layout: 4 vectored horizontal + 2 vertical.
%   cfg = cfg_6(p)                  alpha = 45 deg, CoG height
%   cfg = cfg_6(p, alpha_deg)       vectoring angle sweep (e.g. 30/45/60)
%   cfg = cfg_6(p, alpha_deg, z_off)
%
%   Layout (top view, x forward, y right):
%
%        f2 (FL) \         f5 (front, vertical)        / f1 (FR)
%                 +-----------------------------------+
%                 |                CoG                |
%                 +-----------------------------------+
%        f4 (RL) /         f6 (rear, vertical)         \ f3 (RR)
%
%   Horizontal thrusters at the corners (+-L/2, +-W/2), each angled alpha
%   from the x axis:
%       f1 FR : [cos a, -sin a, 0]     f2 FL : [cos a, +sin a, 0]
%       f3 RR : [cos a, +sin a, 0]     f4 RL : [cos a, -sin a, 0]
%   => X = cos a * (f1+f2+f3+f4)
%      Y = sin a * (-f1+f2+f3-f4)
%      N = (lx sin a + ly cos a) * (-f1+f2-f3+f4)
%   Vertical pair f5/f6 identical to cfg_4's f3/f4 (front/rear) -> heave, pitch.
%
%   Actuated: surge, sway, heave, pitch, yaw (5 DOF, 1 redundant thruster).

if nargin < 2, alpha_deg = 45; end
if nargin < 3, z_off = 0; end

a  = deg2rad(alpha_deg);
hx = p.L/2;
hy = p.W/2;
c  = cos(a);  s = sin(a);

cfg.name  = sprintf('6-thruster (\\alpha = %d deg, z_{off} = %.2f m)', round(alpha_deg), z_off);
cfg.n     = 6;
cfg.alpha = alpha_deg;

%          f1 FR   f2 FL   f3 RR   f4 RL   f5 F    f6 R
cfg.r = [  hx,     hx,    -hx,    -hx,     hx,    -hx  ;
           hy,    -hy,     hy,    -hy,     0,      0   ;
          z_off,  z_off,  z_off,  z_off,  z_off,  z_off ];

cfg.e = [  c,      c,      c,      c,      0,      0   ;
          -s,      s,      s,     -s,      0,      0   ;
           0,      0,      0,      0,      1,      1   ];

cfg.fmax =  p.Fmax_fwd * ones(cfg.n, 1);
cfg.fmin = -p.Fmax_rev * ones(cfg.n, 1);

end
