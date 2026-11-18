function [A, B, Ks, cl] = lqr_design(P)
%LQR_DESIGN One multivariable LQR with integral action (Bryson's rule).
% Returns the linearised model, gains K{rho} for the three weightings, and
% closed-loop analysis struct cl (eigenvalues, K patterns, step metrics).

[A, B] = lin_hover(P);

% Bryson's rule; R scaled by rho
sx  = [P.lqr.ximax*ones(1,3), P.lqr.emax*ones(1,3), P.lqr.vmax*ones(1,3), ...
       P.lqr.etamax*ones(1,3), P.lqr.wmax*ones(1,3)];   % state maxima
Q = diag(1 ./ sx.^2);
ru = [P.lqr.dTmax, P.lqr.nmax'];                                   % input maxima
Ks = cell(1, numel(P.lqr.rhos));
cl = struct('eig', {}, 'K', {}, 'step', {}, 'effort', {});
for i = 1:numel(P.lqr.rhos)
    R = P.lqr.rhos(i) * diag(1 ./ ru.^2);
    Ks{i} = lqr(A, B, Q, R);
    assert(all(real(eig(A - B*Ks{i})) < 0), 'A-BK not Hurwitz');
end

% closed-loop step metrics: 1 m position step per axis (initial condition),
% reported as rise/overshoot/settling of the position error decay
for i = 1:numel(P.lqr.rhos)
    cl(i).eig  = eig(A - B*Ks{i});
    cl(i).K    = Ks{i};
    cl(i).step = step_metrics(A, B, Ks{i}, P);
end
end

function sm = step_metrics(A, B, K, P)
% initial condition x0 = 1 m error in each axis, simulated on the linear loop;
% reported as the equivalent unit-step RECOVERY response y = 1 - e/e(0):
% rise = 10%->90% of y, overshoot beyond y = 1 in %, settling = 2% error band
Ac = A - B*K;
sm = struct('t', {}, 'e', {});
for ax = 1:3
    x0 = zeros(15,1); x0(3+ax) = 1;          % e = 1 m (x15: int(1:3), e(4:6), ed(7:9), etat(10:12), etatd(13:15))
    [y, t] = initial(ss(Ac, zeros(15,4), eye(15), zeros(15,4)), x0, 12);
    ex = y(:, 3+ax);
    yr = 1 - ex/ex(1);                       % normalized recovery
    sm(ax).t    = t;
    sm(ax).e    = ex;
    sm(ax).rise = rise_time(t, yr);
    sm(ax).over = max(0, max(yr) - 1)*100;
    sm(ax).sett = settling_time(t, abs(ex));
end
end

function tr = rise_time(t, yr)
% first 10% and first 90% crossings of the recovery response
i10 = find(yr >= 0.1, 1);  i90 = find(yr >= 0.9, 1);
if isempty(i10) || isempty(i90), tr = NaN; else, tr = t(i90) - t(i10); end
end

function ts = settling_time(t, e)
tol = 0.02 * e(1);
idx = find(abs(e) > tol, 1, 'last');
if isempty(idx), ts = 0; else, ts = t(min(idx+1, numel(t))); end
end
