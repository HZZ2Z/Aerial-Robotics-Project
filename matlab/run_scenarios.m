function T = run_scenarios(P, K, mdl, opts)
%RUN_SCENARIOS The 10 core course runs: ideal/estimated x 5 scenarios
% (nominal, wind, rtk_lost, uncert_Jplus, uncert_Jminus; both uncertainty
% cases are mass+20% with 20 ms input delay, inertia +-20%).
% Each run is saved to results/<execution_id>/<case_id>/run.mat.
% opts.execution_id: batch id (default 'main_<timestamp>'); reused only
% when explicitly passed, so reruns never overwrite a frozen batch.
if nargin < 4, opts = struct(); end
o = setdef(opts, 'execution_id', sprintf('main_%s', datestr_(now)));
Fe = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
Fs = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
unc = setdef(struct(), 'm_scale', 1.2, 'in_delay', 0.02, 'wind_mode', 'mean_gust');
S = { % scen name, flags, opts, case_id
    'nominal',        Fe, struct('wind_mode', 'none'),                        'ideal_nominal'
    'wind',           Fe, struct('wind_mode', 'mean_gust'),                   'ideal_wind'
    'rtk_lost',       Fe, struct('wind_mode', 'mean_gust', 'fix_loss', [20 25]), 'ideal_rtk_lost'
    'uncert_Jplus',   Fe, setdef(unc, 'J_scale', 1.2),                        'ideal_uncert_Jplus'
    'uncert_Jminus',  Fe, setdef(unc, 'J_scale', 0.8),                        'ideal_uncert_Jminus'
    'nominal',        Fs, struct('wind_mode', 'none'),                        'est_nominal'
    'wind',           Fs, struct('wind_mode', 'mean_gust'),                   'est_wind'
    'rtk_lost',       Fs, struct('wind_mode', 'mean_gust', 'fix_loss', [20 25]), 'est_rtk_lost'
    'uncert_Jplus',   Fs, setdef(unc, 'J_scale', 1.2),                        'est_uncert_Jplus'
    'uncert_Jminus',  Fs, setdef(unc, 'J_scale', 0.8),                        'est_uncert_Jminus'
    };
T = struct('mode', {}, 'scen', {}, 'case_id', {}, 'seed', {}, 'M', {}, 'o', {});
fprintf('\n%-9s %-13s | %8s %8s %8s | %9s %9s %9s | %6s %6s\n', ...
    'mode', 'case', 'posRMS', 'posMax', 'hovNorm', 'est_p', 'est_v', 'est_d', 'tilt', 'eff');
for k = 1:size(S, 1)
    ro = S{k,3};
    ro.execution_id = o.execution_id;  ro.case_id = S{k,4};
    ro.seed = field_or(opts, 'seed', P.noise.seed_base);
    run = run_closed(P, K, S{k,2}, mdl, ro);
    M = metrics(run, P, mdl);
    if ~S{k,2}.use_est, run.P_out = []; end    % ideal runs: keep the batch lean
    md = 'ideal'; if S{k,2}.use_est, md = 'estimated'; end
    T(k).mode = md; T(k).scen = S{k,1}; T(k).case_id = S{k,4};
    T(k).seed = ro.seed; T(k).M = M; T(k).o = run;
    fprintf('%-9s %-13s | %8.3f %8.3f %8.3f | %9.4f %9.4f %9.4f | %5.1f %6.2f\n', ...
        md, S{k,1}, M.pos_rms, M.pos_max, M.hov_off_norm, ...
        M.est_rms(1), M.est_rms(2), M.est_rms(3), M.tilt_max*180/pi, M.effort_rms);
end
root = fileparts(fileparts(mfilename('fullpath')));
d = fullfile(root, 'results', o.execution_id);
if exist(d, 'dir') ~= 7, mkdir(d); end
save(fullfile(d, 'scenarios.mat'), 'T', 'o');
end

function v = field_or(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function s = datestr_(n)
s = sprintf('%04d%02d%02d_%02d%02d%02d', ...
    year(n), month(n), day(n), hour(n), minute(n), second(n));
end

function s = setdef(s, varargin)
for k = 1:2:numel(varargin)
    if ~isfield(s, varargin{k}), s.(varargin{k}) = varargin{k+1}; end
end
end
