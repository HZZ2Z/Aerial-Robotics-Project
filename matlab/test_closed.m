%TEST_CLOSED Ideal-state closed-loop checks: build, design, run nominal & wind.
t_all = tic;
P = params();
build_closed(P);
[~, ~, Ks, ~] = lqr_design(P);
K = Ks{3};                                   % rho = 10 (flight config)
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
mdl = rmfield(mdl, setdiff(fieldnames(mdl), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
F = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);

% --- scenario (i): nominal, ideal states ---
o1 = run_closed(P, K, F, mdl, struct('wind_mode', 'none', 'tag', 'ideal_nom'));
err1 = o1.p - o1.ref(:, 1:3);
rms1 = sqrt(mean(err1(o1.t <= 61, 1:3).^2, 'all'));
fprintf('nominal ideal: RMS pos err = %.4f m, max = %.4f m, final p = [%.3f %.3f %.3f]\n', ...
    rms1, max(vecnorm(err1, 2, 2)), o1.p(end, :));
assert(rms1 < 0.15, 'nominal tracking too poor');
assert(all(abs(o1.p(end, :)) < 0.1), 'did not return to origin');
assert(max(abs(o1.vw(:))) == 0, 'wind_mode none must apply zero wind');

% --- scenario (ii): wind, ideal states ---
o2 = run_closed(P, K, F, mdl, struct('wind_mode', 'mean_gust', 'tag', 'ideal_wind'));
err2 = o2.p - o2.ref(:, 1:3);
fprintf('wind ideal: RMS pos err = %.4f m, max = %.4f m, gust std = %.3f m/s\n', ...
    sqrt(mean(err2(o2.t <= 61, 1:3).^2, 'all')), max(vecnorm(err2, 2, 2)), ...
    std(o2.vw(o2.t > 25, 1)));
% hover windows from the actual timeline: [5.5,7.5) pre-wind, [47.5,49.5) post-wind
h0 = o2.t > P.mis.tup + 0.5 & o2.t < P.mis.tup + P.mis.thov - 0.5;
t4 = P.mis.tup + P.mis.thov + P.mis.tb + P.mis.laps*P.mis.T8 + P.mis.tb;
h1 = o2.t > t4 + 0.5 & o2.t < t4 + P.mis.thov - 0.5;
off0 = mean(err2(h0, :), 1);
off1 = mean(err2(h1, :), 1);
fprintf('hover offset pre-wind [%.4f %.4f %.4f] m, post-wind [%.4f %.4f %.4f] m\n', off0, off1);
assert(all(abs(off1) < 0.05), 'constant-wind hover offset not rejected (all axes)');
% plant/log wind are the same signal: vw == what the plant saw (log IS the plant input)
fprintf('max tilt = %.1f deg, thrust in [%.2f %.2f] N\n', ...
    max(abs(o2.eta(o2.t > 12, 1:2)), [], 'all')*180/pi, min(o2.Tn(:, 1)), max(o2.Tn(:, 1)));
% identity residual of the disturbance bookkeeping
assert(o2.resid < 1e-9, 'a_true != ua_meas + d_model + d_input_noise');
disp('CLOSED-LOOP IDEAL TESTS PASSED');
toc(t_all)
