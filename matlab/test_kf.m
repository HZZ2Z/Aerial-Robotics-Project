%TEST_KF Filter validation: linear model-matched statistics check, then a
% tuning sweep over the parameters that are ACTUALLY consumed by the current
% P.kf.mode (rw9: qa/qd; ou_bias12: qa/qf/qs), on the wind scenario.
t_all = tic;
P = params();
[~, ~, Ks, ~] = lqr_design(P);  K = Ks{2};
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
mdl = rmfield(mdl, setdiff(fieldnames(mdl), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
Fe = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);

%% --- part A: linear model-matched check of the statistics machinery ---
% error process e_dot = (A-LC)e + w - L n (CT), BLWN discretization on 1 ms:
% w_k ~ N(0, Q dt), n_k ~ N(0, R/dt). P from the DT recursion must match
% propagate_covariance; ensemble NEES over independent runs ~ 9 for rw9.
rng(1729);
A = mdl.A; C = mdl.C; L = mdl.Lg; Q = mdl.Q; R = mdl.Rg;
dt = 1e-3; ns = 1000;  nm = 300;                 % 1 s per run, 300 runs
Ac = A - L*C;  M = Q + L*R*L';
VL = expm([Ac M; zeros(9) Ac']*dt);              % Van Loan: exact DT recursion
Ad = VL(1:9, 1:9);  Qd = VL(1:9, 10:18);
Pd = zeros(9);                                    % P(0) = 0
for k = 1:ns-1                                    % Pa(ns) is P at t = (ns-1)*dt
    Pd = Ad*Pd*Ad' + Qd;
end
oA = struct('t', (0:ns-1)*dt, 'fix', ones(ns,1));
mdlA = mdl;                                       % fixed good schedule
Pa = propagate_covariance(oA, mdlA);
errP = max(abs(squeeze(Pa(ns,:,:)) - Pd), [], 'all');
fprintf('linear check: max |P_prop - P_dt| at t=1 s = %.2e (P scale %.2e)\n', errP, max(Pd(:)));
% tolerance = RK4 global truncation O(dt^4) over the 1 s window
assert(errP < 2e-5*max(Pd(:)) + 1e-12, 'propagate_covariance does not match the exact DT recursion');
nees = zeros(nm,1); covn = 0;
sd = sqrt(diag(squeeze(Pa(ns,:,:))));             % 1-sigma at t = 1 s
chQ = chol(Qd + 1e-30*eye(9), 'lower');           % exact per-step increment cov
for m = 1:nm
    e = zeros(9,1);
    for k = 1:ns
        e = Ad*e + chQ*randn(9,1);
    end
    nees(m) = e'*(squeeze(Pa(ns,:,:))\e);
    covn = covn + mean(abs(e) <= 3*sd);      % per-axis coverage (metric definition)
end
ci = 1.96*sqrt(2/nm)*9;                        % 95% CI half-width for ANEES
fprintf('linear check: ensemble ANEES = %.2f (theory 9, 95%% CI +-%.2f), 9-dim 3sig cov = %.1f%%\n', ...
    mean(nees), ci, 100*covn/nm);
assert(abs(mean(nees) - 9) < 2*ci, 'NEES not calibrated in the model-matched linear case');
assert(abs(covn/nm - 0.997) < 0.02, '3-sigma coverage off in the linear check');

%% --- part B: tuning sweep with the active parameters of each mode ---
fprintf('\nsweep (wind scenario, dt=2.5 ms, seed 1729):\n');
Pw = P;
% rw9 baseline: qa x qd grid (both consumed by kf_design in rw9 mode)
for qa = [0.5 1 2]
    for qd = [0.3 1 3]
        Pw.kf.mode = 'rw9';  Pw.kf.qa = qa;  Pw.kf.qd = qd;
        [~, ~, ~, ~, ~, ~, ~, mw] = kf_design(Pw);
        mw = rmfield(mw, setdiff(fieldnames(mw), {'mode','nx','tau','A','B','C','Tc', ...
            'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
        o = run_closed(Pw, K, Fe, mw, struct('wind_mode', 'mean_gust', 'dt', 2.5e-3, ...
            'tag', sprintf('kf_qa%g_qd%g', qa, qd)));
        M = metrics(o, Pw, mw);
        fprintf('rw9  qa=%4.1f qd=%4.1f | cov=%5.1f%% NEES=%5.1f | est_d=%6.3f | posRMS=%6.3f m\n', ...
            qa, qd, 100*M.cov3.all.frac, M.nees.all.mean, M.est_rms(3), M.pos_rms);
    end
end
% ou_bias12 extension at the tuned operating point
Pw.kf.mode = 'ou_bias12';  Pw.kf.qa = 1;  Pw.kf.qf = 4;  Pw.kf.qs = 0.05;
[~, ~, ~, ~, ~, ~, ~, mw] = kf_design(Pw);
mw = rmfield(mw, setdiff(fieldnames(mw), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
o = run_closed(Pw, K, Fe, mw, struct('wind_mode', 'mean_gust', 'dt', 2.5e-3, 'tag', 'kf_ou12'));
M = metrics(o, Pw, mw);
fprintf('ou12 qa=%4.1f qf=%4.1f qs=%4.2f | cov=%5.1f%% NEES=%5.1f | est_d=%6.3f | posRMS=%6.3f m\n', ...
    Pw.kf.qa, Pw.kf.qf, Pw.kf.qs, 100*M.cov3.all.frac, M.nees.all.mean, M.est_rms(3), M.pos_rms);
disp('KF TESTS DONE');
toc(t_all)
