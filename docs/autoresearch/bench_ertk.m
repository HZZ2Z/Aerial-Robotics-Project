%BENCH_ERTK Autoresearch benchmark driver (called only by .auto/measure.sh).
% Estimated-state flight: RTK outage 20-25 s in wind (primary) + plain wind.
% Prints METRIC lines and writes .auto/bench_last.txt for checks.sh.
root = fileparts(fileparts(mfilename('fullpath')));
delete(fullfile(root, '.auto', 'bench_last.txt'));   % stale output must not pass checks
addpath(fullfile(root, 'matlab'));

P = params();
[~, ~, Ks, ~] = lqr_design(P);   K = Ks{2};             % rho = 1, as in main.m
[~, ~, Lg, Lb, Pg, Pb, ~] = kf_design(P);
kfdat = struct('Lg', Lg, 'Lb', Lb, 'Pg', Pg, 'Pb', Pb);

F = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
o1 = run_closed(P, K, F, kfdat, struct('wind', 1, 'fix_loss', [20 25], 'tag', 'bench_ertk'));
M1 = metrics(o1, P, Pg);
o2 = run_closed(P, K, F, kfdat, struct('wind', 1, 'tag', 'bench_ewind'));
M2 = metrics(o2, P, Pg);

vals = [M1.pos_rms, M1.pos_max, M1.tilt_max*180/pi, M2.pos_rms, ...
        M2.tilt_max*180/pi, M1.est_rms(1)];
assert(all(isfinite(vals)), 'bench produced non-finite metrics');

fprintf('METRIC ertk_rms=%.6f\n',      M1.pos_rms);
fprintf('METRIC ertk_max=%.6f\n',      M1.pos_max);
fprintf('METRIC tilt_ertk_deg=%.4f\n', M1.tilt_max*180/pi);
fprintf('METRIC ewind_rms=%.6f\n',     M2.pos_rms);
fprintf('METRIC tilt_ewind_deg=%.4f\n', M2.tilt_max*180/pi);
fprintf('METRIC est_p=%.6f\n',         M1.est_rms(1));

fid = fopen(fullfile(root, '.auto', 'bench_last.txt'), 'w');
fprintf(fid, 'ertk_rms=%.6f\newind_rms=%.6f\ntilt_ertk_deg=%.4f\ntilt_ewind_deg=%.4f\n', ...
    M1.pos_rms, M2.pos_rms, M1.tilt_max*180/pi, M2.tilt_max*180/pi);
fclose(fid);
