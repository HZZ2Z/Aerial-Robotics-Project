function M = metrics(o, P, mdl)
%METRICS 统一计算单次仿真运行的定量性能指标与滤波器一致性统计。
%   包含：3D 跟踪误差（RMS 与峰值）、悬停稳态偏移、估计误差（vs 物理扰动真值 d_model）、
%   最大倾角、归一化控制代价，以及 NEES、+/-3sigma 覆盖率与新息零均值检验。
err = o.p - o.ref(:, 1:3);
e3d = vecnorm(err, 2, 2);
M.pos_rms = sqrt(mean(e3d.^2));
M.pos_max = max(e3d);

% hover windows from the actual timeline: [tup+0.5, tup+thov-0.5] and
% [t4+0.5, t4+thov-0.5] (never the blend-in or landing segments)
h1 = o.t > P.mis.tup + 0.5 & o.t < P.mis.tup + P.mis.thov - 0.5;
t4 = P.mis.tup + P.mis.thov + P.mis.laps*P.mis.T8 + P.mis.phase_ramp;
h2 = o.t > t4 + 0.5 & o.t < t4 + P.mis.thov - 0.5;
if any(h2), hsel = h2; else, hsel = h1; end
% hover_validation mission: the dedicated constant-wind hover run reports the
% offset over the wind-on window [35,45] s — the SAME window the paper macros
% use, so metrics.json and \hovBiasOn/\hovBiasOff agree by construction (the
% timeline window above lands before the wind step on this 45 s mission)
if strcmp(P.mis.mode, 'hover_validation')
    hsel = o.t > 35 & o.t <= 45;
end
M.hov_off = mean(err(hsel, :), 1);            % mean offset vector (hover bias)
M.hov_off_norm = norm(M.hov_off, 2);
M.hov_rms = sqrt(mean(sum(err(hsel, :).^2, 2)));   % dynamic RMS in the window

% estimation errors vs the physical disturbance truth
ep = o.kf(:, 1:3) - o.p;  ev = o.kf(:, 4:6) - o.v;
ed = o.kf(:, 7:9) - o.d_model;
M.est_rms = [sqrt(mean(sum(ep.^2,2))), sqrt(mean(sum(ev.^2,2))), sqrt(mean(sum(ed.^2,2)))];
M.est_max = max(vecnorm([ep ev ed], 2, 2));

% attitude / effort (effort on the APPLIED, post-delay post-saturation input;
% normalization uses the nominal Bryson scales and nominal mg for all runs)
tilt = acos(min(1, max(-1, o.R(:, 9))));      % R(3,3): tilt from vertical
M.tilt_max = max(tilt);
M.T_min = min(o.Tn(:,1));
M.T_max = max(o.Tn(:,1));
M.n_peak = max(abs(o.Tn(:,2:4)),[],1);
un = [(o.Tn(:,1) - P.m*P.g)/P.lqr.dTmax, o.Tn(:,2)/P.lqr.nmax(1), ...
      o.Tn(:,3)/P.lqr.nmax(2), o.Tn(:,4)/P.lqr.nmax(3)];
M.effort_rms = sqrt(mean(sum(un.^2, 2)));     % dimensionless normalized effort
M.T_dev_rms = sqrt(mean((o.Tn(:,1) - P.m*P.g).^2));   % [N], nominal mass scale
M.n_max = max(abs(o.Tn(:, 2:4)), [], 'all');

% --- filter consistency vs propagated P9(t) ---
if ~isfield(o, 'P_out') || isempty(o.P_out), return; end
t = o.t;
sel_t = t > min(5,t(end)/2);                 % exclude the singular zero-P start
seg = ones(size(t));                          % 1 good, 0 degraded, -1 no fix, 3 re-acq
seg(o.fix <= 0.5 & o.fix > -0.5) = 0;
seg(o.fix <= -0.5) = -1;
reacq = [false; o.fix(2:end) > 0.5 & o.fix(1:end-1) <= 0.5];  % re-acq instant
seg(reacq) = 3;
E = [ep, ev, ed];
N = numel(t);
nees = zeros(N,1);
    for k = find(sel_t)'                        % row: scalar loop variable
        Pk = squeeze(o.P_out(k,:,:));
        nees(k) = E(k,:)*(Pk\E(k,:)');
    end
names = {'good', 'degraded', 'no_fix', 'reacq'};
segs  = [1 0 -1 3];
M.nees = struct();
for j = 1:4
    ix = sel_t & seg == segs(j);
    if any(ix), M.nees.(names{j}) = struct('mean', mean(nees(ix)), 'n', sum(ix)); end
end
M.nees.all = struct('mean', mean(nees(sel_t)), 'n', sum(sel_t));

% per-axis 3-sigma coverage, zero-centered: |e_i(t)| <= 3*sqrt(P9_ii(t))
sd3 = zeros(N, 9);
for k = 1:N
    sd3(k,:) = 3*sqrt(diag(squeeze(o.P_out(k,:,:))))';
end
cov = abs(E) <= sd3;
M.cov3 = struct();
for j = 1:4
    ix = sel_t & seg == segs(j);
    if any(ix), M.cov3.(names{j}) = struct('frac', mean(cov(ix,:), 'all'), 'n', sum(ix)); end
end
M.cov3.all = struct('frac', mean(cov(sel_t,:), 'all'), 'n', sum(sel_t));

% innovation = p_meas - C*xhat, zero-mean check per fix mode (good/degraded).
% The finite-bandwidth noise has correlation time dt_gnss; single-run means
% are reported descriptively, not judged as independent white samples.
innov = o.p_meas - o.kf(:, 1:3);
M.innov = struct();
for j = 1:2
    ix = sel_t & seg == segs(j);   % good, degraded
    if any(ix)
        M.innov.(names{j}) = struct('mean', mean(innov(ix,:), 1), ...
            'std', std(innov(ix,:), 0, 1), 'n', sum(ix));
    end
end
end
