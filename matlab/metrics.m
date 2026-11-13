function M = metrics(o, P, Pg)
%METRICS Performance metrics for one run_closed output (task 6 table).
% M fields: pos_rms, pos_max, hov_off (1x3), est_rms (1x3: p, v, d),
% tilt_max, T_dev_rms, n_max.
err = o.p - o.ref(:, 1:3);
e3d = vecnorm(err, 2, 2);
M.pos_rms = sqrt(mean(e3d.^2));
M.pos_max = max(e3d);

% hover windows (timeline with tb from params): [5,8] and [t4,t5]
h1 = o.t > P.mis.tup + 0.5 & o.t < P.mis.tup + P.mis.thov - 0.5;
t4 = P.mis.tup + P.mis.thov + P.mis.tb + P.mis.laps*P.mis.T8 + P.mis.tb;
h2 = o.t > t4 + 0.5 & o.t < t4 + P.mis.thov - 0.5;
if any(h2), hsel = h2; else, hsel = h1; end
M.hov_off = mean(err(hsel, :), 1);
M.hov_off_rms = sqrt(mean(M.hov_off.^2));

% estimation errors (against steady-state covariance Pg)
ep = o.kf(:, 1:3) - o.p;  ev = o.kf(:, 4:6) - o.v;
d_true = -P.kd*(o.v - o.vw)/P.m;
ed = o.kf(:, 7:9) - d_true;
M.est_rms = [sqrt(mean(ep.^2,'all')), sqrt(mean(ev.^2,'all')), sqrt(mean(ed.^2,'all'))];
M.est_max = max(vecnorm([ep ev ed], 2, 2));

% attitude / effort
tilt = acos(min(1, max(-1, o.R(:, 9))));        % R(3,3): tilt from vertical
M.tilt_max = max(tilt);
M.T_dev_rms = sqrt(mean((o.Tn(:,1) - P.m*P.g).^2));
M.n_max = max(abs(o.Tn(:, 2:4)), [], 'all');

% filter consistency (RTK-good phases): 3-sigma coverage and NEES
if nargin > 2 && ~isempty(Pg)
    sel = o.t > 5 & o.fix > 0.5;
    E = [ep, ev, ed];  E = E(sel, :);
    Pinv = inv(Pg);
    nees = zeros(size(E,1),1);
    for k = 1:size(E,1), nees(k) = E(k,:)*Pinv*E(k,:)'; end
    M.nees = mean(nees);
    sd = sqrt(diag(Pg))';
    c = mean(abs(E) <= 3*sd, 'all');
    M.cov3 = c;
end
end
