function [eta_r, etadot_r] = trajectories(name, t)
%TRAJECTORIES  Reference pose and its time derivative (NED, Euler angles).
%   [eta_r, etadot_r] = trajectories(name, t)
%
%   name:
%     'hold'        hold origin at 0.5 m depth
%     'step_surge'  1.0 m step in x   at t = 1 s
%     'step_sway'   1.0 m step in y   at t = 1 s
%     'step_heave'  0.5 m step in z   at t = 1 s (deeper)
%     'step_yaw'    90 deg step in psi at t = 1 s
%     'step_yaw180' 180 deg step (peak yaw-rate test)
%     'line'        straight line at 0.22 m/s, descending 0.02 m/s, heading along it
%     'turn'        circle R = 0.5 m at 0.3 m/s, heading tangent
%     'survey'      lawnmower: 3 m legs, 1 m spacing, 0.2 m/s, heading tangent
%     'helix'       R = 1 m circle at 0.2 m/s, descending 0.02 m/s
%
%   All paths start at (0, 0, 0.5) with psi = 0, so the vehicle can be
%   initialised on the reference.

z0 = 0.5;
eta_r    = [0; 0; z0; 0; 0; 0];
etadot_r = zeros(6,1);
ts = 1.0;                         % step time

switch name
    case 'hold'
        % nothing

    case 'step_surge'
        if t >= ts, eta_r(1) = 1.0; end
    case 'step_sway'
        if t >= ts, eta_r(2) = 1.0; end
    case 'step_heave'
        if t >= ts, eta_r(3) = z0 + 0.5; end
    case 'step_yaw'
        if t >= ts, eta_r(6) = pi/2; end
    case 'step_yaw180'
        if t >= ts, eta_r(6) = pi; end

    case 'line'
        v = [0.2; 0.1; 0.02];                 % m/s, gently descending
        eta_r(1:3)    = [0; 0; z0] + v*t;
        eta_r(6)      = atan2(v(2), v(1));     % heading along the line
        etadot_r(1:3) = v;

    case 'turn'
        R = 0.5; V = 0.3; w = V/R;
        eta_r(1)    = R*sin(w*t);
        eta_r(2)    = R*(1 - cos(w*t));
        eta_r(6)    = w*t;
        etadot_r(1) = V*cos(w*t);
        etadot_r(2) = V*sin(w*t);
        etadot_r(6) = w;

    case 'helix'
        R = 1.0; V = 0.2; vz = 0.02; w = V/R;
        eta_r(1)    = R*sin(w*t);
        eta_r(2)    = R*(1 - cos(w*t));
        eta_r(3)    = z0 + vz*t;
        eta_r(6)    = w*t;
        etadot_r(1) = V*cos(w*t);
        etadot_r(2) = V*sin(w*t);
        etadot_r(3) = vz;
        etadot_r(6) = w;

    case 'survey'
        Lg = 3.0;  rt = 0.5;  V = 0.2;
        C  = 2*Lg + 2*pi*rt;          % arc length of one up-and-back cycle
        s  = V*t;
        k  = floor(s / C);
        sc = s - k*C;
        y0 = 4*rt*k;
        if sc < Lg                                    % leg 1: +x
            x = sc;  y = y0;  psi = 0;
            dx = V;  dy = 0;  dpsi = 0;
        elseif sc < Lg + pi*rt                        % turn 1: psi 0 -> pi
            a = (sc - Lg)/rt;
            x = Lg + rt*sin(a);  y = y0 + rt - rt*cos(a);  psi = a;
            dx = V*cos(a);  dy = V*sin(a);  dpsi = V/rt;
        elseif sc < 2*Lg + pi*rt                      % leg 2: -x
            d = sc - Lg - pi*rt;
            x = Lg - d;  y = y0 + 2*rt;  psi = pi;
            dx = -V;  dy = 0;  dpsi = 0;
        else                                          % turn 2: psi pi -> 0
            a = (sc - 2*Lg - pi*rt)/rt;
            x = -rt*sin(a);  y = y0 + 3*rt - rt*cos(a);  psi = pi - a;
            dx = -V*cos(a);  dy = V*sin(a);  dpsi = -V/rt;
        end
        eta_r(1) = x;  eta_r(2) = y;  eta_r(6) = psi;
        etadot_r(1) = dx;  etadot_r(2) = dy;  etadot_r(6) = dpsi;

    otherwise
        error('trajectories: unknown manoeuvre "%s"', name);
end
end
