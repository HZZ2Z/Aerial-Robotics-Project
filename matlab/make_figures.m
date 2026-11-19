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
quiver(wind.ref(200:800:end,1), wind.ref(200:800:end,2), ...
       cos(wind.ref(200:800:end,10)), sin(wind.ref(200:800:end,10)), 0.3, 'k');
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

% --- fig 6: estimation errors with ZERO-CENTERED +-3sqrt(P9_ii(t)) bounds ---
ep = wind.kf(:,1:3) - wind.p;
Ns = numel(wind.t);
sd3 = zeros(Ns, 3);
for k = 1:Ns, sd3(k,:) = 3*sqrt(diag(squeeze(wind.P_out(k,1:3,1:3))))'; end
f = figure('Color', 'w', 'Position', [50 50 430 182]);
for k = 1:3
    subplot(3, 1, k); hold on; grid on;
    lo = -sd3(:, k); hi = sd3(:, k);      % centered at ZERO
    fill([wind.t; flipud(wind.t)], [lo; flipud(hi)], [1 0.85 0.85], 'EdgeColor', 'none');
    plot(wind.t, ep(:, k), 'b', 'LineWidth', 0.8);
    shade_fix(gca, wind);
    ylabel(sprintf('e_{p,%c} [m]', 'x' + k - 1));
end
xlabel('t [s]');
title('position estimation error, zero-centered \pm3\sigma from propagated P(t) (shaded: degraded/no fix)');
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
win = ertk.t >= 19 & ertk.t <= 27;
dr = vecnorm(ertk.kf(:,1:3) - ertk.p, 2, 2);
ph = sqrt(squeeze(ertk.P_out(:,1,1)) + squeeze(ertk.P_out(:,2,2)));
plot(ertk.t(win), dr(win), 'b', 'LineWidth', 1);
plot(ertk.t(win), ph(win), 'r', 'LineWidth', 1);
xline(20, ':'); xline(25, ':');
xlabel('t [s]'); ylabel('[m]');
legend('||\hat p - p||', '1\sigma propagated (P_{11}+P_{22})^{1/2}', 'Location', 'best');
title('5 s RTK outage (t \in [20,25)): error vs propagated covariance');
savepdf(f, fgs, 'rtk_outage');

% --- fig 9: LQR analysis (eigenvalues, K pattern, step recovery) ---
f = figure('Color', 'w', 'Position', [50 50 430 132]);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact');
nexttile(tl); hold on; grid on;
for i = 1:numel(S.Ks)
    e = S.cl(i).eig; plot(real(e), imag(e), 'o', 'MarkerSize', 4);
end
xlabel('Re'); ylabel('Im'); title('eig(A - BK) for \rho = 0.1, 1, 10');
nexttile(tl); imagesc(abs(S.Ks{3})); colorbar; colormap('gray');
xticks(1:3:15); xticklabels({'\int e_x','e_x','\dot e_x','\phi','\dot\phi',''});
xtickangle(0); yticks(1:4); yticklabels({'\delta T', 'n_x', 'n_y', 'n_z'});
title('|K|, \rho = 10');
nexttile(tl); hold on; grid on;
st = S.cl(3).step;                                % rho = 10 flight config
plot(st(3).t, 1 - st(3).e/st(3).e(1), st(1).t, 1 - st(1).e/st(1).e(1), ...
     st(2).t, 1 - st(2).e/st(2).e(1), 'LineWidth', 0.9);
legend('z', 'x', 'y', 'Location', 'southeast');
xlabel('t [s]'); ylabel('y = 1 - e/e_0'); title('recovery from 1 m step, \rho = 10');
savepdf(f, fgs, 'lqr');

% --- fig 10: rho comparison, error vs effort (actual applied input) ---
f = figure('Color', 'w', 'Position', [50 50 430 197]);
hold on; grid on; xlim([0 0.13]);
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
f = figure('Color', 'w', 'Position', [50 50 430 143]);
hold on; grid on;
for w = [1 5]
    o = S.nf.(sprintf('s%d', w));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    win = o.t >= 19.5 & o.t <= 27;
    plot(o.t(win), dr(win), 'LineWidth', 1, 'DisplayName', sprintf('%d s outage', w));
end
xline(20, ':', 'HandleVisibility', 'off'); xline(25, ':', 'HandleVisibility', 'off');
xlabel('t [s]'); ylabel('||\hat p - p|| [m]');
legend('Location', 'best');
title('complete GNSS loss: drift during and after the outage');
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
    patch(ax, [tt(1) tt(1) tt(2) tt(2)], [-10 -10 10 10], ...
        [0.85 0.85 1], 'EdgeColor', 'none', 'FaceAlpha', 0.35);
end
end

function savepdf(f, dir, name)
exportgraphics(f, fullfile(dir, [name '.pdf']), 'ContentType', 'vector');
saveas(f, fullfile(dir, [name '.png']));
close(f);
end
