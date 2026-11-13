%TEST_KF Estimated-state closed loop + filter validation (task 5) + tuning.
P = params();
[A, B, Ks] = lqr_design(P); K = Ks{2};
Fe = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);

% --- tuning sweep on the wind scenario (fast solver step) ---
for qa = [0.2 0.5 1.0]
    for qd = [0.1 0.5 1.0]
        P.kf.qa = qa; P.kf.qd = qd;
        [~, ~, Lg, Lb, Pg, Pb] = kf_design(P);
        kfdat = struct('Lg', Lg, 'Lb', Lb, 'Pg', Pg, 'Pb', Pb);
        o = run_closed(P, K, Fe, kfdat, struct('wind', 1, 'dt', 2.5e-3));
        [cov3, nees, dmse] = kf_check(o, P, Pg);
        fprintf('qa=%.1f qd=%.1f | 3sig cov=%5.1f%% | NEES=%5.1f | d RMS=%5.3f | posRMS=%6.3f m\n', ...
            qa, qd, 100*cov3, nees, dmse, ...
            sqrt(mean((o.p(o.t>10,:) - o.ref(o.t>10,1:3)).^2, 'all')));
    end
end

function [cov3, nees, dmse] = kf_check(o, P, Pg)
sel = o.t > 10 & o.fix > 0.5;                 % RTK-good phases only
ep = o.kf(sel, 1:3) - o.p(sel, :);            % p-hat - p
ev = o.kf(sel, 4:6) - o.v(sel, :);            % v-hat - v
sdp = sqrt(diag(Pg(1:3, 1:3)))';              % 1-sigma position
sdv = sqrt(diag(Pg(4:6, 4:6)))';
covp = mean(abs(ep) <= 3*sdp, 'all');
covv = mean(abs(ev) <= 3*sdv, 'all');
cov3 = (covp + covv) / 2;

% NEES with the full 9-state steady-state covariance
Pinv = inv(Pg);
nees = 0;
xx = [ep ev o.kf(sel,7:9)];                   % errors: need d error too
% build full 9x1 estimation errors per sample
Ef = [ep, ev, o.kf(sel,7:9) - (-P.kd*(o.v(sel,:) - o.vw(sel,:))/P.m)];
n = 0;
for k = 1:size(Ef, 1)
    nees = nees + (Ef(k,:)*Pinv*Ef(k,:)');
    n = n + 1;
end
nees = nees/n;

d_true = -P.kd*(o.v(sel,:) - o.vw(sel,:))/P.m;
dmse = sqrt(mean((o.kf(sel,7:9) - d_true).^2, 'all'));
end
