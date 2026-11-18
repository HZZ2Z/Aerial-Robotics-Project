function run_project(mode)
%RUN_PROJECT One-entry reproduction of every figure/table of the report.
%   run_project('full')  rebuilds the models, runs every experiment behind
%                        the paper (35 missions, ~1.5-2 h), writes figures,
%                        animation and the metrics export.
%   run_project('quick') build + design + nominal/wind sanity runs (~10 min).
% Each invocation creates a NEW results/<execution_id>/ batch; nothing from
% a previous batch is overwritten.
if nargin < 1, mode = 'full'; end
t0 = tic;
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'));
P = params();

fprintf('== 1/6 building models ==\n');
build_plant(P);
build_closed(P);

fprintf('== 2/6 control design ==\n');             % includes lin_hover self-checks
[~, ~, Ks, cl] = lqr_design(P);
K = Ks{3};   % rho = 10: constraint-aware selection - at rho = 1 the estimated
             % RTK-outage transient violates the 35 deg tilt limit (50.6 deg);
             % rho = 10 keeps every core scenario inside the limit (see PROGRESS)

fprintf('== 3/6 filter design ==\n');             % includes observability checks
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);

switch lower(mode)
    case 'quick'
        eid = sprintf('quick_%s', timestamp());
        % sanity: nominal ideal + wind estimated end-to-end (no figures)
        F0 = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
        F1 = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
        run = run_closed(P, K, F0, mdl, struct('wind_mode', 'none', ...
            'execution_id', eid, 'case_id', 'ideal_nominal'));
        M = metrics(run, P, mdl);
        fprintf('quick %-14s posRMS %.4f m | tilt %.1f deg\n', 'ideal_nominal', M.pos_rms, M.tilt_max*180/pi);
        run = run_closed(P, K, F1, mdl, struct('wind_mode', 'mean_gust', ...
            'execution_id', eid, 'case_id', 'est_wind'));
        M = metrics(run, P, mdl);
        fprintf('quick %-14s posRMS %.4f m | tilt %.1f deg\n', 'est_wind', M.pos_rms, M.tilt_max*180/pi);
        fprintf('QUICK DONE in %.1f min (batch %s)\n', toc(t0)/60, eid);
        return
    case 'full'
        eid = sprintf('full_%s', timestamp());
    otherwise
        error('run_project: mode must be ''quick'' or ''full''');
end

fprintf('== 4/6 rho comparison, control experiments, bonus ==\n');
% three LQR weightings x {no wind, wind}, ideal states, everything else equal
rho = cell(1, 6);
for r = 1:3
    for s = 1:2
        wm = 'none'; if s == 2, wm = 'mean_gust'; end
        ro = struct('wind_mode', wm, 'execution_id', eid, ...
            'case_id', sprintf('rho%g_%s', P.lqr.rhos(r), char('a' + s - 1)));
        run = run_closed(P, Ks{r}, struct('use_est',0,'use_Qapprox',0,'ff_wind',0), mdl, ro);
        M = metrics(run, P, mdl);
        rho{r}(s).M = M;
        fprintf('rho=%4.1f %-10s | posRMS %6.3f max %6.3f | effort %5.2f | tilt %4.1f deg\n', ...
            P.lqr.rhos(r), wm, M.pos_rms, M.pos_max, M.effort_rms, M.tilt_max*180/pi);
    end
end

% ten core scenarios (the course matrix)
T = run_scenarios(P, K, mdl, struct('execution_id', eid));

% constant-wind hover: integral action vs integral ablation
Ph = P; Ph.mis.mode = 'hover_validation';
Fh = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
hov = struct();
hov.on  = run_closed(Ph, K, Fh, mdl, struct('wind_mode', 'mean_only', 'tfinal', 45, ...
    'execution_id', eid, 'case_id', 'hover_integral_on'));
hov.off = run_closed(Ph, K0(K), Fh, mdl, struct('wind_mode', 'mean_only', 'tfinal', 45, ...
    'execution_id', eid, 'case_id', 'hover_integral_off'));

% exact Q vs Q~I on the dedicated manoeuvre (ideal states, no wind)
Pq = P; Pq.mis.mode = 'q_maneuver';
qex = run_closed(Pq, K, struct('use_est',0,'use_Qapprox',0,'ff_wind',0), mdl, ...
    struct('wind_mode', 'none', 'tfinal', 14, 'execution_id', eid, 'case_id', 'q_exact'));
qap = run_closed(Pq, K, struct('use_est',0,'use_Qapprox',1,'ff_wind',0), mdl, ...
    struct('wind_mode', 'none', 'tfinal', 14, 'execution_id', eid, 'case_id', 'q_approx'));

% bonus: feed-forward on/off, complete GNSS loss 1 s / 5 s
ff  = struct();
ff.on  = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',1,'ff_tau',1), mdl, ...
    struct('wind_mode', 'mean_gust', 'execution_id', eid, 'case_id', 'ff_on'));
ff.off = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), mdl, ...
    struct('wind_mode', 'mean_gust', 'execution_id', eid, 'case_id', 'ff_off'));
nf = struct();
for w = [1 5]
    nf.(sprintf('s%d', w)) = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), ...
        mdl, struct('wind_mode', 'mean_gust', 'no_fix', 1, 'fix_loss', [20 20+w], ...
        'execution_id', eid, 'case_id', sprintf('nofix%ds', w)));
end

% step-size sensitivity: same source noise, half step (core est_wind = dt 1 ms)
dt2 = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), mdl, ...
    struct('wind_mode', 'mean_gust', 'dt', P.sim.dt/2, ...
    'execution_id', eid, 'case_id', 'est_wind_dt05'));

% holdout seeds: independent validation of the two key estimated scenarios
hold_seeds = [1009 2027 4093 8191 16381];
holdout = struct();
for sd = hold_seeds
    holdout.(sprintf('wind%d', sd)) = run_closed(P, K, ...
        struct('use_est',1,'use_Qapprox',0,'ff_wind',0), mdl, ...
        struct('wind_mode', 'mean_gust', 'seed', sd, 'execution_id', eid, ...
        'case_id', sprintf('holdout_wind_%d', sd)));
    holdout.(sprintf('rtk%d', sd)) = run_closed(P, K, ...
        struct('use_est',1,'use_Qapprox',0,'ff_wind',0), mdl, ...
        struct('wind_mode', 'mean_gust', 'fix_loss', [20 25], 'seed', sd, ...
        'execution_id', eid, 'case_id', sprintf('holdout_rtk_%d', sd)));
end

fprintf('== 5/6 figures, animation, metrics export ==\n');
S = struct('eid', eid, 'P', P, 'Ks', {Ks}, 'cl', {cl}, 'T', {T}, 'rho', {rho}, ...
    'hov', hov, 'qex', qex, 'qap', qap, 'ff', ff, 'nf', nf, 'dt2', dt2, ...
    'holdout', holdout);
make_figures(S);
animate_mission(T(find(strcmp({T.case_id}, 'est_wind'))).o, P, 'mission_anim.mp4');
export_results(S);

fprintf('== 6/6 summary ==\n');
fprintf('FULL DONE in %.1f min (batch %s)\n', toc(t0)/60, eid);
end

function K0 = K0(K)
K0 = K;  K0(:, 1:3) = 0;        % integral ablation: same gains, integral rows off
end

function s = timestamp()
n = now;
s = sprintf('%04d%02d%02d_%02d%02d%02d', year(n), month(n), day(n), hour(n), minute(n), floor(second(n)));
end
