function mpc = mpc_setup(p, cfg, opts)
%MPC_SETUP  Build the linear MPC used for every thruster configuration.
%   mpc = mpc_setup(p, cfg)          default tuning
%   mpc = mpc_setup(p, cfg, opts)    override fields below
%
%   Prediction model: 6-DOF model linearised about hover (nu = 0,
%   phi = theta = 0), re-linearised in yaw at every step (LTV).
%   Decision variables are the individual thruster forces, so thrust
%   allocation and the asymmetric thrust limits are handled inside the MPC.
%
%   Unmodelled effects (quadratic drag, Coriolis, current, tether) are
%   lumped into a disturbance acceleration d_hat estimated online
%   (offset-free MPC; same idea as the ESO in Ma et al. 2016).
%
%   IMPORTANT for a fair comparison: keep opts identical for all configs.
%
%   opts fields (defaults):
%     dt     0.1                     MPC sample time [s]
%     N      20                      prediction horizon [steps]
%     q_eta  [20 20 20 5 5 10]       weight on pose error  [x y z phi theta psi]
%     q_nu   [1 1 1 0.5 0.5 1]       weight on velocity error
%     r_f    0.5                     weight on (f/Fmax)^2 per thruster
%     P_mult 10                      terminal weight = P_mult * Q
%     L_d    0.3                     disturbance-estimator gain (0 = off)
%     iters  100                     max solver iterations
%     solver 'boxqp'                 'boxqp' (exact, no toolbox) or 'quadprog'

if nargin < 3, opts = struct(); end
def = struct('dt', 0.1, 'N', 20, ...
             'q_eta', [20 20 20 5 5 10], 'q_nu', [1 1 1 0.5 0.5 1], ...
             'r_f', 0.5, 'P_mult', 10, 'L_d', 0.3, 'iters', 100, ...
             'solver', 'boxqp');
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(opts, fn{k}), opts.(fn{k}) = def.(fn{k}); end
end

n = cfg.n;
T = tcm(cfg);

mpc.opts = opts;
mpc.p    = p;
mpc.cfg  = cfg;
mpc.T    = T;
mpc.n    = n;
mpc.nx   = 12;

% Linearised dynamics pieces (yaw-dependent part rebuilt in mpc_step)
Minv = inv(p.M);
WB   = p.Wt - p.B;
G = zeros(6,6);                    % d g(eta) / d eta at hover
G(1,5) =  WB;                      % (W-B) sin(theta)
G(2,4) = -WB;                      % -(W-B) cos(theta) sin(phi)
G(4,4) =  p.BG * p.B;              % roll righting
G(5,5) =  p.BG * p.B;              % pitch righting
mpc.Ac_lower = [-Minv*G, -Minv*p.D_lin];   % rows for nu_dot
mpc.Bc_lower = Minv * T;
mpc.w_const  = Minv * [0; 0; WB; 0; 0; 0]; % net weight (sinks if W > B)

% Cost matrices
Q  = diag([opts.q_eta, opts.q_nu]);
P  = opts.P_mult * Q;
N  = opts.N;
Qbar = kron(eye(N), Q);
Qbar(end-11:end, end-11:end) = P;
Fscale = max(abs([cfg.fmax; cfg.fmin]));
Rbar = kron(eye(N), opts.r_f * eye(n) / Fscale^2);
mpc.Qbar = Qbar;
mpc.Rbar = Rbar;

% Bounds over the horizon
mpc.lb = repmat(cfg.fmin(:), N, 1);
mpc.ub = repmat(cfg.fmax(:), N, 1);

% Runtime state
mpc.U_prev  = zeros(n*N, 1);       % warm start
mpc.d_hat   = zeros(6, 1);         % disturbance acceleration estimate
mpc.nu_pred = [];                  % one-step prediction for the estimator
end
