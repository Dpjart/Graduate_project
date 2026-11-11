function xdot = rov_dynamics(x, f, T, p, env)
%ROV_DYNAMICS  6-DOF nonlinear ROV model (Fossen form).
%   xdot = rov_dynamics(x, f, T, p, env)
%
%   x   : 12x1 state [eta; nu]
%         eta = [x y z phi theta psi]'  (NED position, Euler angles)
%         nu  = [u v w p q r]'          (body velocities)
%   f   : nx1 applied thruster forces [N]
%   T   : 6xn thruster configuration matrix (tcm.m)
%   p   : parameter struct (rov_params.m)
%   env : (optional) struct with
%         .v_current  3x1 water current velocity, NED [m/s]   (default 0)
%         .tether     true/false - include tether drag         (default false)
%         .tau_ext    6x1 extra body-frame force/moment        (default 0)
%
%   M*nu_dot + C(nu_r)*nu_r + D(nu_r)*nu_r + g(eta) = T*f + tau_tether + tau_ext
%   eta_dot  = J(eta)*nu

if nargin < 5 || isempty(env), env = struct(); end
if ~isfield(env, 'v_current'), env.v_current = zeros(3,1); end
if ~isfield(env, 'tether'),    env.tether    = false;      end
if ~isfield(env, 'tau_ext'),   env.tau_ext   = zeros(6,1); end

eta = x(1:6);
nu  = x(7:12);
phi = eta(4); th = eta(5); psi = eta(6);

%% Kinematics
[R, Tth] = rov_kinematics(phi, th, psi);
eta_dot  = [R * nu(1:3); Tth * nu(4:6)];

%% Relative velocity (current acts through hydrodynamic terms)
nu_c = [R' * env.v_current(:); 0; 0; 0];
nu_r = nu - nu_c;

%% Coriolis-centripetal (rigid body + added mass, diagonal M, origin at CoG)
mu = p.M(1,1); mv = p.M(2,2); mw = p.M(3,3);
Ip = p.M(4,4); Iq = p.M(5,5); Ir = p.M(6,6);
u = nu_r(1); v = nu_r(2); w = nu_r(3);
pr = nu_r(4); q = nu_r(5); r = nu_r(6);

F_cor = [ mv*v*r - mw*w*q ;
          mw*w*pr - mu*u*r ;
          mu*u*q - mv*v*pr ;
          (Iq - Ir)*q*r  + (mv - mw)*v*w ;
          (Ir - Ip)*pr*r + (mw - mu)*u*w ;
          (Ip - Iq)*pr*q + (mu - mv)*u*v ];

%% Damping
F_damp = -p.D_lin * nu_r - p.D_quad * (abs(nu_r) .* nu_r);

%% Restoring (CoG at origin, CoB at r_b = [0 0 -BG])
WB = p.Wt - p.B;
g_eta = [  WB * sin(th) ;
          -WB * cos(th) * sin(phi) ;
          -WB * cos(th) * cos(phi) ;
           p.BG * p.B * cos(th) * sin(phi) ;
           p.BG * p.B * sin(th) ;
           0 ];

%% Tether drag (lumped at CoG, earth-frame relative velocity)
tau_teth = zeros(6,1);
if env.tether
    v_rel_n = R * nu(1:3) - env.v_current(:);
    F_n     = -0.5 * p.rho * p.tether_Cd * p.tether_d * p.tether_L ...
              * norm(v_rel_n) * v_rel_n;
    tau_teth(1:3) = R' * F_n;
end

%% Equations of motion
tau    = T * f(:) + tau_teth + env.tau_ext(:);
nu_dot = p.M \ (tau + F_cor + F_damp - g_eta);

xdot = [eta_dot; nu_dot];
end
