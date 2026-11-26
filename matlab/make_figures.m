function make_figures(S)
%MAKE_FIGURES All report figures from ONE execution batch (results/<eid>/).
% S fields: eid, P, Ks, cl, T (core scenarios), rho (3x2 comparison),
% hov, qex, qap, ff, nf, dt2, holdout. Every figure is titled from the
% actual run configuration; no figure is copied from an older batch.
root = fileparts(fileparts(mfilename('fullpath')));
fgs  = fullfile(root, 'report', 'figs');
if exist(fgs, 'dir') ~= 7, mkdir(fgs); end
T = S.T;
ix = @(md, sc) find(strcmp({T.mode}, md) & strcmp({T.scen}, sc));
wind = T(ix('estimated', 'wind')).o;
inom = T(ix('ideal', 'nominal')).o;
ertk = T(ix('estimated', 'rtk_lost')).o;

% --- fig 1: 3-D + top view trajectories ---
f = figure('Color', 'w', 'Position', [50 50 430 167]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact');
ax1 = nexttile(tl); grid on; hold on;
plot3(inom.ref(:,1), inom.ref(:,2), inom.ref(:,3), 'k--', 'LineWidth', 1);
plot3(wind.p(:,1), wind.p(:,2), wind.p(:,3), 'b', 'LineWidth', 1);
plot3(wind.kf(:,1), wind.kf(:,2), wind.kf(:,3), 'r-.', 'LineWidth', 0.8);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); view(35, 20); axis equal;
legend({'reference', 'true', 'estimated'}, 'Location', 'best');
title('3-D mission trajectory');
ax2 = nexttile(tl); grid on; hold on;
plot(wind.ref(:,1), wind.ref(:,2), 'k--', 'LineWidth', 1);
plot(wind.p(:,1), wind.p(:,2), 'b', 'LineWidth', 1);
plot(wind.kf(:,1), wind.kf(:,2), 'r-.', 'LineWidth', 0.8);
quiver(wind.ref(8000:1700:52000,1), wind.ref(8000:1700:52000,2), ...
       cos(wind.ref(8000:1700:52000,10)), sin(wind.ref(8000:1700:52000,10)), 0.3, 'k');
xlabel('x North [m]'); ylabel('y East [m]'); axis equal;
title('top view (arrows: heading \psi^*)');
savepdf(f, fgs, 'traj3d');

% --- fig 2: position errors, wind scenario ---
f = figure('Color', 'w', 'Position', [50 50 430 153]);
plot(wind.t, wind.p(:,1) - wind.ref(:,1), wind.t, wind.p(:,2) - wind.ref(:,2), ...
     wind.t, wind.p(:,3) - wind.ref(:,3), 'LineWidth', 0.8); grid on;
xline(S.P.wind.t0, ':', 'wind onset');
xlabel('t [s]'); ylabel('error [m]'); legend('x', 'y', 'z');
title(sprintf('position error, wind + gusts, estimated states (seed %d)', S.T(ix('estimated','wind')).seed));
savepdf(f, fgs, 'pos_err');

% --- fig 3: attitude and heading ---
f = figure('Color', 'w', 'Position', [50 50 430 153]);
plot(wind.t, wind.eta*180/pi, wind.t, wind.ref(:,10)*180/pi, '--', 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[deg]'); legend('\phi', '\theta', '\psi', '\psi^*');
title('attitude, wind scenario');
savepdf(f, fgs, 'attitude');

% --- fig 4: thrust and torques ---
f = figure('Color', 'w', 'Position', [50 50 430 182]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile(tl); plot(wind.t, wind.Tn(:,1), 'LineWidth', 0.8); grid on;
yline(S.P.m*S.P.g, ':'); ylabel('T [N]'); title('thrust (dotted: mg)');
nexttile(tl); plot(wind.t, wind.Tn(:,2:4), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[N m]'); legend('n_x', 'n_y', 'n_z');
savepdf(f, fgs, 'commands');

% --- fig 5: wind profile (the signal actually applied to the plant) ---
f = figure('Color', 'w', 'Position', [50 50 430 172]);
plot(wind.t, wind.vw, 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[m/s]'); legend('v_{w,x}', 'v_{w,y}', 'v_{w,z}');
title('wind velocity applied to the plant');
savepdf(f, fgs, 'wind');

% --- fig 5b: combined mission panel (tracking error, attitude, wind) ---
% single-column composite of figs 2/3/5 for the report; the standalone
% versions above stay on disk for the README's figure listing.
f = figure('Color', 'w', 'Position', [50 50 430 290]);
tl = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile(tl); hold on; grid on;
plot(wind.t, wind.p(:,1) - wind.ref(:,1), wind.t, wind.p(:,2) - wind.ref(:,2), ...
     wind.t, wind.p(:,3) - wind.ref(:,3), 'LineWidth', 0.8);
xline(S.P.wind.t0, ':', 'HandleVisibility', 'off');
ylabel('error [m]'); legend('x', 'y', 'z', 'Location', 'northeast');
title('wind + gusts, estimated states: tracking error, attitude, applied wind');
nexttile(tl); grid on;
plot(wind.t, wind.eta*180/pi, wind.t, wind.ref(:,10)*180/pi, '--', 'LineWidth', 0.8);
ylabel('[deg]'); legend('\phi', '\theta', '\psi', '\psi^*', 'Location', 'northeast');
nexttile(tl); grid on;
plot(wind.t, wind.vw, 'LineWidth', 0.8);
xlabel('t [s]'); ylabel('[m/s]'); legend('v_{w,x}', 'v_{w,y}', 'v_{w,z}', 'Location', 'northeast');
savepdf(f, fgs, 'mission_wind');

% --- fig 6: estimation errors with ZERO-CENTERED +-3sqrt(P9_ii(t)) bounds ---
ep = wind.kf(:,1:3) - wind.p;
Ns = numel(wind.t);
sd3 = zeros(Ns, 3);
for k = 1:Ns, sd3(k,:) = 3*sqrt(diag(squeeze(wind.P_out(k,1:3,1:3))))'; end
f = figure('Color', 'w', 'Position', [50 50 430 182]);
for k = 1:3
    subplot(3, 1, k); hold on; grid on;
    lo = -sd3(:, k); hi = sd3(:, k);      % centered at ZERO
    plot(wind.t,lo,'--','Color',[0.7 0.2 0.2]);
    plot(wind.t,hi,'--','Color',[0.7 0.2 0.2]);
    plot(wind.t, ep(:, k), 'b', 'LineWidth', 0.8);
    shade_fix(gca, wind);
    ylabel(sprintf('e_{p,%c} [m]', 'x' + k - 1));
end
xlabel('t [s]');
title('Position error and propagated \pm3\sigma');
savepdf(f, fgs, 'est_err');

% --- fig 7: d-hat vs disturbance truth (d_model) and wind-drag component ---
f = figure('Color', 'w', 'Position', [50 50 430 182]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile(tl); hold on; grid on;
plot(wind.t, wind.kf(:,7:9), wind.t, wind.d_model, '--', 'LineWidth', 0.8);
ylabel('[m/s^2]');
legend('\hat d_x', '\hat d_y', '\hat d_z', 'd_{model,x}', 'd_{model,y}', 'd_{model,z}');
title('estimated disturbance vs physical truth a_{true} - ua_{ideal}');
nexttile(tl); hold on; grid on;
plot(wind.t, wind.d_model(:,1), wind.t, wind.d_wind(:,1), 'LineWidth', 0.8);
xlabel('t [s]'); ylabel('[m/s^2]'); legend('d_{model,x}', 'wind drag x');
savepdf(f, fgs, 'dhat');

% --- fig 8: RTK outage, propagated covariance vs actual drift ---
f = figure('Color', 'w', 'Position', [50 50 430 143]);
hold on; grid on;
fixWindow = ertk.opts.fix_loss;
win = ertk.t >= fixWindow(1)-1 & ertk.t <= fixWindow(2)+2;
dr = vecnorm(ertk.kf(:,1:3) - ertk.p, 2, 2);
ph = sqrt(squeeze(ertk.P_out(:,1,1)) + squeeze(ertk.P_out(:,2,2)));
plot(ertk.t(win), dr(win), 'b', 'LineWidth', 1);
plot(ertk.t(win), ph(win), 'r', 'LineWidth', 1);
xline(fixWindow, ':');
xlabel('t [s]'); ylabel('[m]');
legend('||\hat p - p||', '1\sigma propagated (P_{11}+P_{22})^{1/2}', 'Location', 'best');
title('RTK degradation: error and propagated uncertainty');
savepdf(f, fgs, 'rtk_outage');

% --- fig 9: LQR analysis (eigenvalues, K pattern, step recovery) ---
f = figure('Color', 'w', 'Position', [50 50 430 132]);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact');
nexttile(tl); hold on; grid on;
for i = 1:numel(S.Ks)
    e = S.cl(i).eig; plot(real(e), imag(e), 'o', 'MarkerSize', 4);
end
xlabel('Re'); ylabel('Im'); title('eig(A - BK) for \rho = 0.1, 1, 10');
nexttile(tl); imagesc(abs(S.Ks{S.selected})); colorbar; colormap('gray');
xticks([2.5 6 9 12 15]); xticklabels({'\int e','e','\dot e','e_t','\dot e_t'});
xtickangle(30); yticks(1:4); yticklabels({'\delta T', 'n_x', 'n_y', 'n_z'});
title(sprintf('|K|, \\rho = %g',S.P.lqr.flight_rho));
nexttile(tl); hold on; grid on;
st = S.cl(S.selected).step;
plot(st(3).t, 1 - st(3).e/st(3).e(1), st(1).t, 1 - st(1).e/st(1).e(1), ...
     st(2).t, 1 - st(2).e/st(2).e(1), st(4).t, 1 - st(4).e/st(4).e(1), 'LineWidth', 0.9);
legend('z', 'x', 'y', '\psi', 'Location', 'southeast');
xlabel('t [s]'); ylabel('y = 1 - e/e_0'); title('Normalised step recovery');
savepdf(f, fgs, 'lqr');

% --- fig 10: rho comparison, error vs effort (actual applied input) ---
f = figure('Color', 'w', 'Position', [50 50 430 197]);
hold on; grid on;
mk = {'o', 's', '^'};
dy = 0.010;                                   % stagger labels of near-coincident points
for r = 1:3
    for s = 1:2
        M = S.rho{r}(s).M;
        plot(M.effort_rms, M.pos_rms, mk{r}, 'MarkerSize', 8, ...
            'MarkerFaceColor', [0.2 0.4 1 - 0.15*(s-1)], ...
            'MarkerEdgeColor', 'k');
        text(M.effort_rms, M.pos_rms + dy*(r - 2), ...
            sprintf('  \\rho=%g, %s', S.P.lqr.rhos(r), char('a' + s - 1)), 'FontSize', 8);
    end
end
xlabel('normalized effort U_{RMS} (applied input)'); ylabel('3-D position error E_{RMS} [m]');
title('LQR weight comparison: a = no wind, b = wind + gusts');
savepdf(f, fgs, 'rho');

% --- fig 11: Q comparison on the dedicated manoeuvre ---
f = figure('Color', 'w', 'Position', [50 50 430 143]);
plot(S.qex.t, vecnorm(S.qex.p - S.qex.ref(:,1:3), 2, 2), ...
     S.qap.t, vecnorm(S.qap.p - S.qap.ref(:,1:3), 2, 2), 'LineWidth', 0.8); grid on;
xline(5, ':'); xline(9, ':');
xlabel('t [s]'); ylabel('||e|| [m]');
legend('exact Q(\phi,\theta)', 'Q \approx I');
title('dedicated manoeuvre (diagonal step + 90^{\circ} yaw, window 5-9 s)');
savepdf(f, fgs, 'qcompare');

% --- fig 12: hover integral validation ---
f = figure('Color', 'w', 'Position', [50 50 430 143]);
plot(S.hov.on.t, S.hov.on.p(:,1) - S.hov.on.ref(:,1), ...
     S.hov.off.t, S.hov.off.p(:,1) - S.hov.off.ref(:,1), 'LineWidth', 0.8); grid on;
xline(S.P.wind.t0, ':', '2 m/s mean wind');
xlabel('t [s]'); ylabel('e_x [m]');
legend('integral on', 'integral ablated');
title('constant-wind hover: integral action vs ablation (mean only, no gusts)');
savepdf(f, fgs, 'hover_int');

% --- fig 13: feed-forward on/off ---
f = figure('Color', 'w', 'Position', [50 50 430 143]);
plot(S.ff.on.t, vecnorm(S.ff.on.p - S.ff.on.ref(:,1:3), 2, 2), ...
     S.ff.off.t, vecnorm(S.ff.off.p - S.ff.off.ref(:,1:3), 2, 2), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('||e|| [m]'); legend('FF on', 'FF off');
title('bonus: disturbance feed-forward, wind scenario, estimated states');
savepdf(f, fgs, 'ff');

% --- fig 14: complete GNSS loss ---
f = figure('Color', 'w', 'Position', [50 50 430 220]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
ax1 = nexttile(tl); hold(ax1, 'on'); grid(ax1, 'on');
ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on');
for w = [1 5]
    o = S.nf.(sprintf('s%d', w));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    tilt = acosd(min(1, max(-1, o.R(:,9))));
    label = sprintf('%d s outage', w);
    plot(ax1, o.t, dr, 'LineWidth', 1, 'DisplayName', label);
    plot(ax2, o.t, tilt, 'LineWidth', 1, 'DisplayName', label);
end
for ax = [ax1 ax2]
    xline(ax, S.P.mis.fix_loss(1)+[0 1 5], ':', 'HandleVisibility', 'off');
    xlim(ax, [S.P.mis.fix_loss(1)-0.5 S.nf.s5.t(end)]);
end
yline(ax2, S.P.tiltmax*180/pi, '--', '0.61 rad limit', 'HandleVisibility', 'off');
xlabel(ax2, 't [s]'); ylabel(ax2, 'tilt [deg]');
ylabel(ax1, '$\Vert\hat{p} - p\Vert$ [m]', 'Interpreter', 'latex');
legend(ax1, 'Location', 'best');
savepdf(f, fgs, 'nofix');

% --- fig 15: holdout seed validation ---
f = figure('Color', 'w', 'Position', [50 50 430 197]);
hold on; grid on;
hs = [1009 2027 4093 8191 16381];
prms = @(o) sqrt(mean(vecnorm(o.p - o.ref(:,1:3), 2, 2).^2));
cw = [T(ix('estimated','wind')).M.pos_rms, arrayfun(@(s) prms(S.holdout.(sprintf('wind%d', s))), hs)];
cr = [T(ix('estimated','rtk_lost')).M.pos_rms, arrayfun(@(s) prms(S.holdout.(sprintf('rtk%d', s))), hs)];
plot(1:6, cw, 'o-', 'LineWidth', 1); plot(1:6, cr, 's-', 'LineWidth', 1);
set(gca, 'XTick', 1:6, 'XTickLabel', [{'dev'}, arrayfun(@num2str, [1009 2027 4093 8191 16381], 'UniformOutput', false)]);
xlabel('seed'); ylabel('3-D position error E_{RMS} [m]');
legend('estimated wind', 'estimated rtk\_lost', 'Location', 'best');
title('holdout seed validation (seed 1729 = development run)');
savepdf(f, fgs, 'holdout');

% --- fig 16: step-size sensitivity ---
f = figure('Color', 'w', 'Position', [50 50 430 143]);
plot(wind.t, vecnorm(wind.p - wind.ref(:,1:3), 2, 2), ...
     S.dt2.t, vecnorm(S.dt2.p - S.dt2.ref(:,1:3), 2, 2), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('||e|| [m]');
legend('dt = 1 ms', 'dt = 0.5 ms', 'Location', 'best');
title(sprintf('step-size sensitivity, same noise configuration (est. wind, seed %d)', S.T(ix('estimated','wind')).seed));
savepdf(f, fgs, 'dt');

fprintf('figures written to %s\n', fgs);
end

function shade_fix(ax, o)
% background shading for degraded/no-fix windows
d = o.fix <= 0.5;
if ~any(d), return; end
edges = find([~d(1); diff(d)] | [diff(d); ~d(end)]);  % run boundaries
for k = 1:2:numel(edges)-1
    tt = o.t(edges([k k+1]));
    yl = ylim(ax);
    patch(ax, [tt(1) tt(1) tt(2) tt(2)], yl([1 2 2 1]), ...
        [0.85 0.85 1], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
end
end

function savepdf(f, dir, name)
exportgraphics(f, fullfile(dir, [name '.pdf']), 'ContentType', 'image', 'Resolution', 260);
exportgraphics(f, fullfile(dir, [name '.png']), 'Resolution', 220);
close(f);
end
