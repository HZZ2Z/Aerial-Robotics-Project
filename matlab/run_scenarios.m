function T = run_scenarios(P, K, kfdat, tag)
%RUN_SCENARIOS The four scenarios with ideal and estimated states (task 6).
% Returns struct array T of metrics; saves each run to results/run_<...>.mat.
Fe = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
Fs = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
S = { % scen name, flags, opts, file tag
    'nominal',  Fe, struct('wind', 0),                                       'inom'
    'wind',     Fe, struct('wind', 1),                                       'iwind'
    'rtk_lost', Fe, struct('wind', 1, 'fix_loss', [20 25]),                  'irtk'
    'uncert',   Fe, struct('wind', 1, 'm_scale', 1.2, 'J_scale', 1.2, ...
                       'in_delay', 0.02),                                    'iunc'
    'nominal',  Fs, struct('wind', 0),                                       'enom'
    'wind',     Fs, struct('wind', 1),                                       'ewind'
    'rtk_lost', Fs, struct('wind', 1, 'fix_loss', [20 25]),                  'ertk'
    'uncert',   Fs, struct('wind', 1, 'm_scale', 1.2, 'J_scale', 1.2, ...
                       'in_delay', 0.02),                                    'eunc'
    };
T = struct('mode', {}, 'scen', {}, 'M', {}, 'o', {});
fprintf('\n%-9s %-9s | %8s %8s | %8s | %9s %9s %9s | %6s\n', ...
    'mode', 'scen', 'posRMS', 'posMax', 'hovRMS', 'est_p', 'est_v', 'est_d', 'tilt');
for k = 1:size(S, 1)
    opts = S{k,3}; opts.tag = ['scen_' S{k,4}];
    o = run_closed(P, K, S{k,2}, kfdat, opts);
    M = metrics(o, P, kfdat.Pg);
    if k <= 4, md = 'ideal'; else, md = 'estimated'; end
    T(k).mode = md; T(k).scen = S{k,1}; T(k).M = M; T(k).o = o;
    fprintf('%-9s %-9s | %8.3f %8.3f | %8.3f | %9.4f %9.4f %9.4f | %5.1f\n', ...
        md, S{k,1}, M.pos_rms, M.pos_max, M.hov_off_rms, ...
        M.est_rms(1), M.est_rms(2), M.est_rms(3), M.tilt_max*180/pi);
end
root = fileparts(fileparts(mfilename('fullpath')));
save(fullfile(root, 'results', ['scenarios_' tag '.mat']), 'T');
end
