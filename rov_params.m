function p = rov_params()
%ROV_PARAMS  Physical parameters of the small ROV.
%   p = rov_params() returns a struct used by every other file.
%
%   Frames : body = FRD (x fwd, y right, z down), origin at CoG.
%            earth = NED.
%   Units  : SI (kg, m, s, N).
%
%   Values marked [MEASURE] are placeholders - replace with measured /
%   CAD values. Values marked [IDENTIFY] are first estimates to be
%   refined from pool tests (coast-down, step responses).

%% ---------------- Environment ----------------
p.rho = 1000;          % water density [kg/m^3] (1025 for sea water)
p.g   = 9.81;          % gravity [m/s^2]

%% ---------------- Geometry (frame envelope) ----------------
p.L = 0.30;            % length along x [m]  (assumed 300 mm axis = x)
p.W = 0.20;            % width  along y [m]
p.H = 0.20;            % height along z [m]

%% ---------------- Mass & inertia ----------------
p.m = 4.5;             % total dry mass incl. thrusters [kg]   [MEASURE]

% Rigid-body inertia about CoG - solid-box approximation.
% Replace with CAD values once available.                       [MEASURE]
p.Ix = p.m/12 * (p.W^2 + p.H^2);
p.Iy = p.m/12 * (p.L^2 + p.H^2);
p.Iz = p.m/12 * (p.L^2 + p.W^2);

%% ---------------- Weight, buoyancy, CoB ----------------
p.Wt = p.m * p.g;      % weight [N]

% Displaced volume (tube + extrusion + thrusters + foam).       [MEASURE]
% Target slightly positive buoyancy (B a few % above W).
p.V  = 4.6e-3;         % displaced volume [m^3]
p.B  = p.rho * p.g * p.V;   % buoyancy [N]

% Centre of buoyancy relative to CoG, body frame (z down => negative = above).
p.r_b = [0; 0; -0.02]; % [m]  BG = 20 mm                        [MEASURE]
p.BG  = -p.r_b(3);     % metacentric height used in restoring moments [m]

%% ---------------- Added mass (diagonal) ----------------
% First estimate: fraction of rigid mass/inertia for a blunt open frame.
% Open frames typically 0.3-1.0 x m.                            [IDENTIFY]
ca_lin = [0.6; 0.8; 1.0];     % surge, sway, heave ratios
ca_rot = [0.3; 0.3; 0.3];     % roll, pitch, yaw ratios
p.Xud = -ca_lin(1) * p.m;
p.Yvd = -ca_lin(2) * p.m;
p.Zwd = -ca_lin(3) * p.m;
p.Kpd = -ca_rot(1) * p.Ix;
p.Mqd = -ca_rot(2) * p.Iy;
p.Nrd = -ca_rot(3) * p.Iz;

%% ---------------- Damping ----------------
% Quadratic: -0.5*rho*Cd*A for each translational axis.
Cd = [1.1; 1.2; 1.2];                 % open box frame          [IDENTIFY]
A  = [p.W*p.H; p.L*p.H; p.L*p.W];     % frontal areas x, y, z  [m^2]
p.Xuu = -0.5*p.rho*Cd(1)*A(1);
p.Yvv = -0.5*p.rho*Cd(2)*A(2);
p.Zww = -0.5*p.rho*Cd(3)*A(3);

% Rotational quadratic: strip theory over the six box faces
% (each face panel's normal velocity during pure rotation x moment arm).
Cd_rot = 1.2;                                                  % [IDENTIFY]
k_rot  = rot_drag_coeffs(p.L, p.W, p.H, p.rho, Cd_rot);
p.Kpp = -k_rot(1);
p.Mqq = -k_rot(2);
p.Nrr = -k_rot(3);

% Linear (dominant at low speed).                              [IDENTIFY]
p.Xu = -4.0;  p.Yv = -6.0;  p.Zw = -6.0;
p.Kp = -0.07; p.Mq = -0.07; p.Nr = -0.07;

%% ---------------- Assembled matrices ----------------
p.M_RB = diag([p.m p.m p.m p.Ix p.Iy p.Iz]);
p.M_A  = -diag([p.Xud p.Yvd p.Zwd p.Kpd p.Mqd p.Nrd]);
p.M    = p.M_RB + p.M_A;        % total inertia matrix (6x6)

p.D_lin  = -diag([p.Xu  p.Yv  p.Zw  p.Kp  p.Mq  p.Nr ]);   % positive-definite
p.D_quad = -diag([p.Xuu p.Yvv p.Zww p.Kpp p.Mqq p.Nrr]);   % multiplied by |nu|

%% ---------------- Thruster model ----------------
% Same per-thruster spec for every configuration (fairness).
% Defaults: Blue Robotics T200 @ 12 V - replace with your motor.  [MEASURE]
p.Fmax_fwd = 36.4;     % max forward thrust [N]
p.Fmax_rev = 28.6;     % max reverse thrust [N] (asymmetric)
p.tau_thr  = 0.08;     % thruster first-order time constant [s]

%% ---------------- Tether ----------------
p.tether_Cd = 1.2;     % cylinder drag coefficient
p.tether_d  = 0.008;   % tether diameter [m]                    [MEASURE]
p.tether_L  = 2.0;     % immersed length [m]                    [MEASURE]

end

%% ======================================================================
function k = rot_drag_coeffs(L, W, H, rho, Cd)
%ROT_DRAG_COEFFS  Quadratic rotational drag k (moment = -k*w*|w|) about x,y,z
%   for a box L x W x H, by summing drag on small panels of every face.
ng = 40;
half = [L W H] / 2;
k = zeros(3,1);
for ax = 1:3
    e_ax = zeros(3,1);  e_ax(ax) = 1;
    Mtot = 0;
    for fn = 1:3                         % face normal direction
        for sgn = [-1 1]
            idx = setdiff(1:3, fn);
            a = linspace(-half(idx(1)), half(idx(1)), ng+1);
            b = linspace(-half(idx(2)), half(idx(2)), ng+1);
            ac = (a(1:end-1) + a(2:end))/2;  bc = (b(1:end-1) + b(2:end))/2;
            dA = (a(2)-a(1)) * (b(2)-b(1));
            nvec = zeros(3,1);  nvec(fn) = sgn;
            for i = 1:ng
                for j = 1:ng
                    r = zeros(3,1);
                    r(fn) = sgn*half(fn);  r(idx(1)) = ac(i);  r(idx(2)) = bc(j);
                    vn = dot(cross(e_ax, r), nvec);        % unit rate
                    if vn > 0                              % windward side only
                        F = 0.5*rho*Cd*vn^2*dA * nvec;     % drag magnitude along normal
                        Mtot = Mtot + dot(cross(r, F), e_ax);
                    end
                end
            end
        end
    end
    k(ax) = abs(Mtot);
end
end
