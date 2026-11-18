%TEST_WIND P1 component checks: wind modes, mean step timing, OU statistics,
% GNSS fix status sequence, input delay bookkeeping.
t_all = tic;
P = params();
build_closed(P);
[~, ~, Ks, ~] = lqr_design(P);  K = Ks{2};
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
mdl = rmfield(mdl, setdiff(fieldnames(mdl), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
F = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);

% --- mode none: plant/log wind strictly zero ---
o = run_closed(P, K, F, mdl, struct('wind_mode', 'none', 'tfinal', 10, 'tag', 'w_none'));
assert(all(o.vw(:) == 0), 'mode none: wind must be exactly zero');
fprintf('none: vw max = %g OK\n', max(abs(o.vw(:))));

% --- mode mean_only: 2 m/s step exactly at t0, no gust ---
o = run_closed(P, K, F, mdl, struct('wind_mode', 'mean_only', 'tfinal', 20, 'tag', 'w_mean'));
pre  = o.t < P.wind.t0 - 1e-9;  post = ~pre;
assert(all(o.vw(pre, 1) == 0) && all(abs(o.vw(post, 1) - 2) < 1e-9), 'mean step wrong');
assert(all(o.vw(:, 2:3) == 0, 'all'), 'mean wind is x-only');
fprintf('mean_only: step at t=%g s to 2 m/s OK\n', o.t(find(post, 1)));

% --- mode gust_only: OU component, sigma ~ 1 m/s, correlation time ~ tau ---
% deterministic evidence: the built filter has pole -1/tau (tau = 2 s by
% construction), then stochastic stats are judged over a 3-seed ensemble,
% never by a single 60 s sample (finite-sample tolerance)
assert(strcmp(get_param('closed_loop/tf1', 'Denominator'), sprintf('[1 %g]', 1/P.wind.tau)), ...
    'OU denominator must be [1 1/tau]');
assert(strcmp(get_param('closed_loop/wn1', 'Cov'), 'gust_gain'), 'BLWN Cov must be gust_gain');
h = tf(1, [1 1/P.wind.tau]);                      % sigma = 1, tau = 2: num = 1
ti = (0:0.01:20)';
yi = impulse(h, ti);
tau_hat = interp1(yi, ti, yi(1)*exp(-1));         % time to decay to 1/e
fprintf('OU impulse decay to 1/e at t = %.3f s (tau = %g s)\n', tau_hat, P.wind.tau);
assert(abs(tau_hat - P.wind.tau) < 0.05, 'OU time constant wrong');
g = [];  dtx = [];
for sd = [1729 2027 4093]
    o = run_closed(P, K, F, mdl, struct('wind_mode', 'gust_only', 'tfinal', 60, ...
        'seed', sd, 'tag', sprintf('w_gust%d', sd)));
    g = [g; o.vw(o.t > 20, :)];                   % past the filter transient
    dtx = mean(diff(o.t));
end
mmean = mean(g);  sg = std(g);
lags = round(P.wind.tau/dtx);  nl = size(g,1);
ac = zeros(3,1);
for c = 1:3, ac(c) = g(1:nl-lags,c)' * g(1+lags:nl,c) / (sum(g(:,c).^2)); end
fprintf('gust ensemble (3 seeds): mean [%.3f %.3f %.3f], std [%.3f %.3f %.3f], rho(tau) [%.3f %.3f %.3f]\n', ...
    mmean, sg, ac);
assert(all(abs(mmean) < 0.2), 'gust mean not zero over ensemble');
assert(all(abs(sg - 1) < 0.2), 'gust std not ~1 m/s (OU gain/Cov wrong)');
% corroborating check with estimator-variance tolerance: the tau=2 s proof is
% the deterministic impulse decay above; a 120 s pooled sample autocorrelation
% still carries O(0.1) estimation noise
assert(all(abs(ac - exp(-1)) < 0.2), 'OU correlation at lag tau not ~exp(-1): tau wrong');

% --- GNSS fix status sequences, [t0,t1) semantics ---
for case_ = [0 1]   % 0: degraded (fix_drop=1), 1: no_fix (fix_drop=2)
    o = run_closed(P, K, F, mdl, struct('wind_mode', 'none', 'tfinal', 4, ...
        'fix_loss', [2 3], 'no_fix', case_, 'tag', sprintf('w_fix%d', case_)));
    f19 = o.fix(find(o.t >= 1.9, 1));  f25 = o.fix(find(o.t >= 2.5, 1));
    f35 = o.fix(find(o.t >= 3.5, 1));
    want = 1 - (1 + case_);   % degraded: 0, no_fix: -1
    fprintf('fix status (%s): t=1.9: %d, t=2.5: %d, t=3.5: %d\n', ...
        char('a' + case_), f19, f25, f35);
    assert(f19 == 1 && f25 == want && f35 == 1, 'fix status sequence wrong');
end

% --- input delay bookkeeping: u_delayed = ucmd shifted by in_delay ---
% (Transport Delay holds its initial output 0 up to and including t = delay)
o = run_closed(P, K, F, mdl, struct('wind_mode', 'mean_gust', 'in_delay', 0.02, ...
    'tfinal', 25, 'tag', 'w_delay'));
nd = round(0.02/P.sim.dt);
assert(all(abs(o.u_delayed(1:nd+1, :)) < 1e-9, 'all'), ...
    'u_delayed must hold 0 until t = in_delay');
assert(max(abs(o.u_delayed(nd+2:end, :) - o.ucmd(2:end-nd, :)), [], 'all') < 1e-9, ...
    'u_delayed is not ucmd shifted by in_delay');
% and the KF input thrust is the Memory-held current command (one sample older)
assert(max(abs(o.T_model(2:end) - o.ucmd(1:end-1, 1)), [], 'all') < 1e-9, ...
    'T_model is not the Memory-held command thrust');
fprintf('delay bookkeeping OK (u_delayed = ucmd-20ms, T_model = Memory(cmd))\n');

% --- saturation is part of the applied input ---
assert(all(o.Tn(:,1) <= P.Tmax + 1e-9) && all(o.Tn(:,1) >= 0), ...
    'applied thrust outside limits');
disp('WIND COMPONENT TESTS PASSED');
toc(t_all)
