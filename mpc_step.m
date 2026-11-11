function [f, mpc, info] = mpc_step(mpc, x, eta_ref, nu_ref)
%MPC_STEP  Solve one MPC step.
%   [f, mpc, info] = mpc_step(mpc, x, eta_ref, nu_ref)
%
%   x       : 12x1 measured state [eta; nu]
%   eta_ref : 6xN reference poses over the horizon (k = 1..N)
%   nu_ref  : 6xN reference body velocities over the horizon
%   f       : nx1 thruster force command for this sample
%   info    : struct with solver iterations and predicted cost

o  = mpc.opts;
N  = o.N;  n = mpc.n;  nx = mpc.nx;  dt = o.dt;
x  = x(:);

%% 1) Disturbance estimate update (offset-free MPC)
if ~isempty(mpc.nu_pred) && o.L_d > 0
    e_nu = x(7:12) - mpc.nu_pred;
    mpc.d_hat = mpc.d_hat + o.L_d * e_nu / dt;
end

%% 2) Linearise about current yaw and velocity, discretise (ZOH)
%  p_dot = Rz(psi)*v  ~  Rz(psi0)*v + dRz(psi0)*v0*(psi - psi0)
%  The dRz term lets heading changes steer the position, which an
%  underactuated vehicle (no sway) needs to correct lateral error.
psi = x(6);
v0  = x(7:9);
Rz  = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
dRz = [-sin(psi) -cos(psi) 0; cos(psi) -sin(psi) 0; 0 0 0];
Ac  = [zeros(6), blkdiag(Rz, eye(3)); mpc.Ac_lower];
Ac(1:3, 6) = dRz * v0;
Bc  = [zeros(6, n); mpc.Bc_lower];
wc  = [-dRz * v0 * psi; zeros(3,1); mpc.w_const + mpc.d_hat];

Maug = zeros(nx + n + 1);
Maug(1:nx, 1:nx)          = Ac;
Maug(1:nx, nx+1:nx+n)     = Bc;
Maug(1:nx, end)           = wc;
E  = expm(Maug * dt);
Ad = E(1:nx, 1:nx);
Bd = E(1:nx, nx+1:nx+n);
wd = E(1:nx, end);

%% 3) Condensed prediction  X = Phi*x + Gam*U + Wv
Phi = zeros(nx*N, nx);
Gam = zeros(nx*N, n*N);
Wv  = zeros(nx*N, 1);
Ak = eye(nx);  Gk = zeros(nx, n*N);  wk = zeros(nx, 1);
for k = 1:N
    Ak = Ad * Ak;
    Gk = Ad * Gk;  Gk(:, (k-1)*n+1:k*n) = Bd;
    wk = Ad * wk + wd;
    rows = (k-1)*nx+1 : k*nx;
    Phi(rows, :) = Ak;
    Gam(rows, :) = Gk;
    Wv(rows)     = wk;
end

%% 4) Reference stack (yaw unwrapped around the current heading)
Xref = zeros(nx*N, 1);
psi_prev = psi;
for k = 1:N
    er = eta_ref(:, k);
    er(6) = psi_prev + atan2(sin(er(6) - psi_prev), cos(er(6) - psi_prev));
    psi_prev = er(6);
    Xref((k-1)*nx+1 : k*nx) = [er; nu_ref(:, k)];
end

%% 5) QP:  min 0.5 U'HU + g'U   s.t. lb <= U <= ub
H = Gam' * mpc.Qbar * Gam + mpc.Rbar;
H = (H + H') / 2;
g = Gam' * mpc.Qbar * (Phi * x + Wv - Xref);

U0 = [mpc.U_prev(n+1:end); mpc.U_prev(end-n+1:end)];   % shifted warm start
lb = mpc.lb;  ub = mpc.ub;

switch o.solver
    case 'quadprog'
        qopt = optimoptions('quadprog', 'Display', 'off');
        [U, ~, ~, out] = quadprog(H, g, [], [], [], [], lb, ub, U0, qopt);
        it = out.iterations;
    otherwise   % exact projected-Newton box QP, no toolbox needed
        [U, it] = boxqp(H, g, lb, ub, U0, o.iters);
end

f = U(1:n);
mpc.U_prev = U;

%% 6) One-step prediction for next disturbance update
x1 = Ad * x + Bd * f + wd;
mpc.nu_pred = x1(7:12);

info.iters = it;
info.cost  = 0.5*U'*H*U + g'*U;
end

%% ======================================================================
function [x, it] = boxqp(H, g, lb, ub, x0, maxit)
%BOXQP  min 0.5 x'Hx + g'x  s.t. lb <= x <= ub   (H positive definite)
%   Projected Newton with Armijo line search (Bertsekas 1982; Tassa 2014).
%   Converges to the exact optimum in a handful of iterations.
x   = min(max(x0, lb), ub);
val = 0.5*x'*H*x + g'*x;
for it = 1:maxit
    grad = H*x + g;
    clamped = (x <= lb & grad > 0) | (x >= ub & grad < 0);
    free = ~clamped;
    if ~any(free) || norm(grad(free)) < 1e-8 * (1 + norm(g))
        break;
    end
    % Newton step on the free set
    dx = zeros(size(x));
    Rc = chol(H(free, free));
    dx(free) = -(Rc \ (Rc' \ grad(free)));
    % projected Armijo line search
    step = 1;  improved = false;
    for ls = 1:30
        xn = min(max(x + step*dx, lb), ub);
        vn = 0.5*xn'*H*xn + g'*xn;
        if vn <= val + 1e-4 * grad' * (xn - x)
            improved = true;  break;
        end
        step = step * 0.5;
    end
    if ~improved, break; end
    if norm(xn - x) < 1e-10 * (1 + norm(x))
        x = xn;  val = vn;  break;
    end
    x = xn;  val = vn;
end
end
