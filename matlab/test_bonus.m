%TEST_BONUS Task 3 Q(phi,theta) vs Q~I (dedicated manoeuvre) + bonus items.
P = params();
[A, B, Ks] = lqr_design(P); K = Ks{2};
[~, ~, Lg, Lb, Pg, Pb] = kf_design(P);
kfdat = struct('Lg', Lg, 'Lb', Lb, 'Pg', Pg, 'Pb', Pb);

% --- task 3: dedicated combined manoeuvre (tilt + simultaneous 90 deg yaw) ---
Pq = P; Pq.mis.man = 1;
oq0 = run_closed(Pq, K, struct('use_est',0,'use_Qapprox',0,'ff_wind',0), kfdat, ...
    struct('wind', 0, 'tfinal', 14, 'tag', 'qexact'));
oq1 = run_closed(Pq, K, struct('use_est',0,'use_Qapprox',1,'ff_wind',0), kfdat, ...
    struct('wind', 0, 'tfinal', 14, 'tag', 'qapprox'));
win = oq0.t > 5 & oq0.t < 9;
e0 = oq0.p - oq0.ref(:,1:3); e1 = oq1.p - oq1.ref(:,1:3);
fprintf('Q exact vs Q~I (manoeuvre): posRMS window %.4f vs %.4f m | max window %.4f vs %.4f m\n', ...
    sqrt(mean(e0(win,:).^2,'all')), sqrt(mean(e1(win,:).^2,'all')), ...
    max(vecnorm(e0(win,:),2,2)), max(vecnorm(e1(win,:),2,2)));

% --- bonus 1: wind feed-forward via d_hat (mission, wind) ---
off = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',1,'ff_tau',1), kfdat, ...
    struct('wind', 1, 'tag', 'ffwind'));
eon = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), kfdat, ...
    struct('wind', 1, 'tag', 'noffwind'));
Moff = metrics(off, P, Pg); Mon = metrics(eon, P, Pg);
fprintf('feed-forward: posRMS %.4f (on) vs %.4f m (off) | hovRMS %.4f vs %.4f\n', ...
    Moff.pos_rms, Mon.pos_rms, Moff.hov_off_rms, Mon.hov_off_rms);

% --- bonus 2: complete GNSS loss 1 s and 5 s ---
for w = [1 5]
    o = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), kfdat, ...
        struct('wind', 1, 'no_fix', 1, 'fix_loss', [20 20+w], 'tag', sprintf('nofix%d', w)));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    win2 = o.t >= 20 & o.t <= 20+w;
    fprintf('GNSS off %d s: drift at outage end %.3f m, max in window %.3f m\n', ...
        w, dr(find(o.t >= 20+w, 1)), max(dr(win2)));
end
disp('BONUS TESTS DONE');
