function P9t = propagate_covariance(o, mdl)
%PROPAGATE_COVARIANCE Model-predicted estimation-error covariance P(t).
% Propagates Pdot = (A-LC)P + P(A-LC)' + Q + L R(t) L' with the SAME fixed
% gain schedule the filter actually used (fix log + re-acquisition timer
% reconstruction; hard switch for blend_s = 0), RK4 on the run's fixed grid.
% This is the covariance PREDICTED under the declared noise/model assumptions
% (independent process and measurement noise), not a measured covariance.
% Initial error covariance is zero: true and estimated initial states are
% identical by construction. Returns the output-space covariance
% P9(t) = Tc*P*Tc' for every logged step (N x 9 x 9).

dt = o.t(2) - o.t(1);
N  = numel(o.t);
nx = mdl.nx;
P  = zeros(nx);                       % identical initial states -> e(0) = 0
ts = 1e3;                             % filter timer starts as "good for long"
P9t = zeros(N, 9, 9);
A = mdl.A;  B = mdl.B;  C = mdl.C;  Q = mdl.Q;
for k = 1:N
    fix = o.fix(k);
    % L(t) and R(t): measurement noise follows the fix status, L follows the
    % filter schedule (never blend R)
    if     fix > 0.5
        if mdl.blend_s > 0, w = min(1, ts/mdl.blend_s); else, w = 1; end
        L = w*mdl.Lg + (1-w)*mdl.Lb;  R = mdl.Rg;
    elseif fix > -0.5
        L = mdl.Lb;                   R = mdl.Rb;
    else
        L = zeros(nx, 3);             R = mdl.Rg;   % L = 0: R term vanishes
    end
    ts = ts + dt*((fix > 0.5) - (fix <= 0.5)*ts/0.02);
    ts = max(ts, 0);
    if k < N
        Ac = A - L*C;
        W  = L*R*L';
        k1 = Ac*P + P*Ac' + Q + W;
        k2 = Ac*(P + dt/2*k1) + (P + dt/2*k1)*Ac' + Q + W;
        k3 = Ac*(P + dt/2*k2) + (P + dt/2*k2)*Ac' + Q + W;
        k4 = Ac*(P + dt*k3)   + (P + dt*k3)*Ac'   + Q + W;
        P  = P + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        P  = (P + P')/2;              % symmetry against round-off
    end
    P9 = mdl.Tc*P*mdl.Tc';
    P9t(k,:,:) = P9;
end
end
