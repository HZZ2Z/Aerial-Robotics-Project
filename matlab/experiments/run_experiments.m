function [rho, hov, qex, qap, ff, nf, dt2, holdout] = run_experiments(P, K, Ks, mdl, eid, core)
%RUN_EXPERIMENTS Comparisons and bonus cases used by the report.
ideal = struct('use_est',0,'use_Qapprox',0,'ff_wind',0);
estimated = ideal;
estimated.use_est = 1;
windModes = {'none','mean_gust'};

% three LQR weightings x {no wind, wind}, ideal states, everything else equal
rho = cell(1, numel(Ks));
for r = 1:numel(Ks)
    for s = 1:numel(windModes)
        wm = windModes{s};
        ro = struct('wind_mode', wm, 'execution_id', eid, ...
            'case_id', sprintf('rho%g_%s', P.lqr.rhos(r), char('a' + s - 1)));
        run = run_closed(P, Ks{r}, ideal, mdl, ro);
        M = metrics(run, P, mdl);
        rho{r}(s).M = M;
        fprintf('rho=%4.1f %-10s | posRMS %6.3f max %6.3f | effort %5.2f | tilt %4.1f deg\n', ...
            P.lqr.rhos(r), wm, M.pos_rms, M.pos_max, M.effort_rms, M.tilt_max*180/pi);
    end
end

% constant-wind hover: integral action vs integral ablation
Ph = P;
Ph.mis.mode = 'hover_validation';
withoutIntegral = K;
withoutIntegral(:,1:4) = 0;
hov = struct();
hov.on  = run_closed(Ph, K, ideal, mdl, struct('wind_mode', 'mean_only', 'tfinal', Ph.mis.t_hv, ...
    'execution_id', eid, 'case_id', 'hover_integral_on'));
hov.off = run_closed(Ph, withoutIntegral, ideal, mdl, struct('wind_mode', 'mean_only', 'tfinal', Ph.mis.t_hv, ...
    'execution_id', eid, 'case_id', 'hover_integral_off'));

% exact Q vs Q~I on the dedicated manoeuvre (ideal states, no wind)
Pq = P;
Pq.mis.mode = 'q_maneuver';
approximateQ = ideal;
approximateQ.use_Qapprox = 1;
qex = run_closed(Pq, K, ideal, mdl, ...
    struct('wind_mode', 'none', 'tfinal', 14, 'execution_id', eid, 'case_id', 'q_exact'));
qap = run_closed(Pq, K, approximateQ, mdl, ...
    struct('wind_mode', 'none', 'tfinal', 14, 'execution_id', eid, 'case_id', 'q_approx'));

% bonus: feed-forward on/off, complete GNSS loss 1 s / 5 s
feedforward = estimated;
feedforward.ff_wind = 1;
ff.on  = run_closed(P, K, feedforward, mdl, ...
    struct('wind_mode', 'mean_gust', 'execution_id', eid, 'case_id', 'ff_on'));
ff.off = core(strcmp({core.case_id},'est_wind')).o;
nf = struct();
for w = [1 5]
    nf.(sprintf('s%d', w)) = run_closed(P, K, estimated, ...
        mdl, struct('wind_mode', 'mean_gust', 'no_fix', 1, 'fix_loss', P.mis.fix_loss(1)+[0 w], ...
        'execution_id', eid, 'case_id', sprintf('nofix%ds', w)));
end

% step-size sensitivity: same source noise, half step (core est_wind = dt 1 ms)
dt2 = run_closed(P, K, estimated, mdl, ...
    struct('wind_mode', 'mean_gust', 'dt', P.sim.dt/2, ...
    'execution_id', eid, 'case_id', 'est_wind_dt05'));

% holdout seeds: independent validation of the two key estimated scenarios
hold_seeds = [1009 2027 4093 8191 16381];
holdout = struct();
for sd = hold_seeds
    holdout.(sprintf('wind%d', sd)) = run_closed(P, K, ...
        estimated, mdl, ...
        struct('wind_mode', 'mean_gust', 'seed', sd, 'execution_id', eid, ...
        'case_id', sprintf('holdout_wind_%d', sd)));
    holdout.(sprintf('rtk%d', sd)) = run_closed(P, K, ...
        estimated, mdl, ...
        struct('wind_mode', 'mean_gust', 'fix_loss', P.mis.fix_loss, 'seed', sd, ...
        'execution_id', eid, 'case_id', sprintf('holdout_rtk_%d', sd)));
end
end
