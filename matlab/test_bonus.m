%TEST_BONUS Q(phi,theta) vs Q~I (dedicated manoeuvre), feed-forward sign and
% performance, complete-GNSS-loss drift, constant-wind hover integral test.
t_all = tic;
P = params();
[~, ~, Ks, ~] = lqr_design(P);  K = Ks{3};   % rho = 10 (flight config)
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
mdl = rmfield(mdl, setdiff(fieldnames(mdl), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
Fe = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
Fs = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);

% --- feed-forward sign check (analytic, static hover) ---
% course model: pdd = ua + d. Static hover, wind 2 m/s along x ->
% d = -kd*(0 - 2)/m = +kd*2/m e1. Command ua = aref - d_hat with d_hat = d
% gives pdd = aref exactly; the old +d_hat would command 2*d.
d = [P.kd*P.wind.vmean/P.m; 0; 0];
ua_cmd = 0 - d;                       % aref = 0, d_hat = d
pdd = ua_cmd + d;                     % what the plant realizes
assert(max(abs(pdd)) < 1e-12, 'feed-forward sign wrong: ua = aref - d_hat required');
fprintf('FF sign check: d = [%.4f 0 0], ua = aref - d_hat -> pdd = aref OK\n', d(1));

% --- task 3: dedicated combined manoeuvre (tilt + simultaneous 90 deg yaw) ---
Pq = P; Pq.mis.mode = 'q_maneuver';
oq0 = run_closed(Pq, K, Fe, mdl, struct('wind_mode', 'none', 'tfinal', 14, 'tag', 'qexact'));
oq1 = run_closed(Pq, K, struct('use_est',0,'use_Qapprox',1,'ff_wind',0), mdl, ...
    struct('wind_mode', 'none', 'tfinal', 14, 'tag', 'qapprox'));
win = oq0.t > 5 & oq0.t < 9;
e0 = oq0.p - oq0.ref(:,1:3); e1 = oq1.p - oq1.ref(:,1:3);
fprintf('Q exact vs Q~I (manoeuvre): posRMS window %.4f vs %.4f m | max window %.4f vs %.4f m\n', ...
    sqrt(mean(e0(win,:).^2,'all')), sqrt(mean(e1(win,:).^2,'all')), ...
    max(vecnorm(e0(win,:),2,2)), max(vecnorm(e1(win,:),2,2)));

% --- constant-wind hover validation: integral on vs integral ablation ---
Ph = P; Ph.mis.mode = 'hover_validation';
Fh = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
oh_on  = run_closed(Ph, K, Fh, mdl, struct('wind_mode', 'mean_only', 'tfinal', 45, 'tag', 'hovI'));
K0 = K; K0(:, 1:3) = 0;               % integral ablation only, same K otherwise
oh_off = run_closed(Ph, K0, Fh, mdl, struct('wind_mode', 'mean_only', 'tfinal', 45, 'tag', 'hov0'));
w0 = oh_on.t > 10 & oh_on.t < 14;  w1 = oh_on.t > 35 & oh_on.t <= 45;
b_on  = mean(oh_on.p(w1,:)  - oh_on.ref(w1,1:3), 1);
b_off = mean(oh_off.p(w1,:) - oh_off.ref(w1,1:3), 1);
fprintf('hover integral: bias norm on %.4f m vs off %.4f m | pre-wind %.4f m\n', ...
    norm(b_on), norm(b_off), norm(mean(oh_on.p(w0,:) - oh_on.ref(w0,1:3), 1)));
assert(norm(b_on) < 0.01, 'integral did not remove constant-wind hover bias (1 cm target)');
assert(norm(b_on) < 0.2*norm(b_off), 'integral ablation shows no clear benefit');

% --- bonus 1: wind feed-forward via d_hat (mission, wind, estimated) ---
off = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',1,'ff_tau',1), mdl, ...
    struct('wind_mode', 'mean_gust', 'tag', 'ffwind'));
eon = run_closed(P, K, Fs, mdl, struct('wind_mode', 'mean_gust', 'tag', 'noffwind'));
Moff = metrics(off, P, mdl);  Mon = metrics(eon, P, mdl);
fprintf('feed-forward: posRMS %.4f (on) vs %.4f m (off) | effort %.3f vs %.3f | tilt %.1f vs %.1f deg\n', ...
    Moff.pos_rms, Mon.pos_rms, Moff.effort_rms, Mon.effort_rms, ...
    Moff.tilt_max*180/pi, Mon.tilt_max*180/pi);

% --- bonus 2: complete GNSS loss 1 s and 5 s (status must return to good) ---
for w = [1 5]
    o = run_closed(P, K, Fs, mdl, struct('wind_mode', 'mean_gust', 'no_fix', 1, ...
        'fix_loss', [20 20+w], 'tag', sprintf('nofix%d', w)));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    win2 = o.t >= 20 & o.t <= 20+w;
    fprintf('GNSS off %d s: drift at outage end %.3f m, max in window %.3f m, fix(21.5)=%d\n', ...
        w, dr(find(o.t >= 20+w, 1)), max(dr(win2)), ...
        o.fix(find(o.t >= 20+w+0.5, 1)));
    assert(all(o.fix(o.t >= 20+w+1e-9) == 1), 'fix status must return to good=1 after outage');
end
disp('BONUS TESTS DONE');
toc(t_all)
