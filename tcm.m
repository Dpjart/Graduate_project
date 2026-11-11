function T = tcm(cfg)
%TCM  Thruster configuration matrix.
%   T = tcm(cfg) returns the 6xn matrix mapping thruster forces f (nx1)
%   to the body-frame generalised force tau = [X Y Z K M N]' = T*f.
%
%   Column i:  [ e_i ; r_i x e_i ]

n = cfg.n;
T = zeros(6, n);
for i = 1:n
    e = cfg.e(:, i) / norm(cfg.e(:, i));
    T(1:3, i) = e;
    T(4:6, i) = cross(cfg.r(:, i), e);
end
T(abs(T) < 1e-12) = 0;   % clean round-off for readability
end
