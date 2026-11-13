%TEST_CLOSED Ideal-state closed-loop checks: build, design, run nominal & wind.
t_all = tic;
P = params();
build_closed(P);
[A, B, Ks, cl] = lqr_design(P); %#ok<NASGU>
K = Ks{2};                                   % rho = 1
F = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
[A9, C9, Lg, Lb, Pg, Pb] = kf_design(P); %#ok<NASGU>
kfdat = struct('Lg', Lg, 'Lb', Lb, 'Pg', Pg, 'Pb', Pb);

% --- scenario (i): nominal, ideal states ---
o1 = run_closed(P, K, F, kfdat, struct('wind', 0, 'tag', 'ideal_nom'));
err1 = o1.p - o1.ref(:, 1:3);
rms1 = sqrt(mean(err1(o1.t <= 61, 1:3).^2, 'all'));
fprintf('nominal ideal: RMS pos err = %.4f m, max = %.4f m, final p = [%.3f %.3f %.3f]\n', ...
    rms1, max(vecnorm(err1, 2, 2)), o1.p(end, :));
assert(rms1 < 0.15, 'nominal tracking too poor');
assert(all(abs(o1.p(end, :)) < 0.1), 'did not return to origin');

% --- scenario (ii): wind, ideal states ---
o2 = run_closed(P, K, F, kfdat, struct('wind', 1, 'tag', 'ideal_wind'));
err2 = o2.p - o2.ref(:, 1:3);
fprintf('wind ideal: RMS pos err = %.4f m, max = %.4f m, gust std = %.3f m/s\n', ...
    sqrt(mean(err2(o2.t <= 61, 1:3).^2, 'all')), max(vecnorm(err2, 2, 2)), ...
    std(o2.vw(o2.t > 25, 1)));
% hover offset before/after wind: mean error over [12,15) vs [55,62)
off0 = mean(err2(o2.t > 12 & o2.t < 15, :), 1);
off1 = mean(err2(o2.t > 55, :), 1);
fprintf('hover offset pre-wind [%.4f %.4f %.4f] m, post-wind [%.4f %.4f %.4f] m\n', off0, off1);
assert(off1(3) < 0.05, 'vertical offset not rejected');
fprintf('max tilt = %.1f deg, thrust in [%.2f %.2f] N\n', ...
    max(abs(o2.eta(o2.t > 12, 1:2)), [], 'all')*180/pi, min(o2.Tn(:, 1)), max(o2.Tn(:, 1)));
disp('CLOSED-LOOP IDEAL TESTS PASSED');
toc(t_all)
