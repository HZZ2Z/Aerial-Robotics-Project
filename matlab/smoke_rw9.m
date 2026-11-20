function smoke_rw9()
%SMOKE_RW9 Smoke check of the submission-default configuration (rw9 + hard
% RTK gain switch): estimated-state wind run and the 5 s RTK outage, before
% committing to a full batch. Writes to results/scratch via run tags.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'));
P = params();
assert(strcmp(P.kf.mode, 'rw9') && P.kf.reacq_blend_s == 0, ...
    'params default is not the rw9 hard-switch configuration');
fprintf('config: kf.mode=%s blend=%g qa=%g qd=%g xi_max=[%g %g %g]\n', ...
    P.kf.mode, P.kf.reacq_blend_s, P.kf.qa, P.kf.qd, P.lqr.xi_max);
[~, ~, Ks, ~] = lqr_design(P);  K = Ks{3};
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
F = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
o = run_closed(P, K, F, mdl, struct('wind_mode', 'mean_gust', 'tag', 'smoke_est_wind'));
M = metrics(o, P, mdl);
fprintf('smoke est_wind | posRMS %.3f m | tilt %.1f deg\n', M.pos_rms, M.tilt_max*180/pi);
o = run_closed(P, K, F, mdl, struct('wind_mode', 'mean_gust', 'no_fix', 1, ...
    'fix_loss', [20 25], 'tag', 'smoke_nofix5s'));
M = metrics(o, P, mdl);
fprintf('smoke nofix5s | posRMS %.3f m | posMax %.3f m | tilt %.1f deg\n', ...
    M.pos_rms, M.pos_max, M.tilt_max*180/pi);
disp('SMOKE DONE');
end
