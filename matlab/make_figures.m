function make_figures(tag)
%MAKE_FIGURES All report figures from saved scenario runs (results/).
root = fileparts(fileparts(mfilename('fullpath')));
fgs  = fullfile(root, 'report', 'figs');
if exist(fgs, 'dir') ~= 7, mkdir(fgs); end
S = load(fullfile(root, 'results', ['scenarios_' tag '.mat']));  T = S.T;
ix = @(md, sc) find(strcmp({T.mode}, md) & strcmp({T.scen}, sc));
wind = T(ix('estimated', 'wind')).o;
inom = T(ix('ideal', 'nominal')).o;
iunc = T(ix('ideal', 'uncert')).o;
ertk = T(ix('estimated', 'rtk_lost')).o;

% --- fig: 3-D + top view trajectories ---
f = figure('Color', 'w', 'Position', [50 50 980 380]);
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

% --- fig: position errors, wind scenario ---
f = figure('Color', 'w', 'Position', [50 50 900 320]);
plot(wind.t, wind.p(:,1) - wind.ref(:,1), wind.t, wind.p(:,2) - wind.ref(:,2), ...
     wind.t, wind.p(:,3) - wind.ref(:,3), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('error [m]'); legend('x', 'y', 'z');
title('position error, wind + gusts, estimated states');
savepdf(f, fgs, 'pos_err');

% --- fig: attitude and heading ---
f = figure('Color', 'w', 'Position', [50 50 900 320]);
plot(wind.t, wind.eta*180/pi, wind.t, wind.ref(:,10)*180/pi, '--', 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[deg]'); legend('\phi', '\theta', '\psi', '\psi^*');
title('attitude, wind scenario');
savepdf(f, fgs, 'attitude');

% --- fig: thrust and torques ---
f = figure('Color', 'w', 'Position', [50 50 900 380]);
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact');
nexttile(tl); plot(wind.t, wind.Tn(:,1), 'LineWidth', 0.8); grid on;
yline(1.2*9.81, ':'); ylabel('T [N]'); title('thrust (dotted: mg)');
nexttile(tl); plot(wind.t, wind.Tn(:,2:4), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[N m]'); legend('n_x', 'n_y', 'n_z');
savepdf(f, fgs, 'commands');

% --- fig: wind profile ---
f = figure('Color', 'w', 'Position', [50 50 700 280]);
plot(wind.t, wind.vw, 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[m/s]'); legend('v_{w,x}', 'v_{w,y}', 'v_{w,z}');
title('wind velocity');
savepdf(f, fgs, 'wind');

% --- fig: estimation errors with 3-sigma bounds (position) ---
P = params();
sel = wind.t > 5;
[~, ~, Lg, Lb, Pg, Pb] = kf_design(P);
sdg = sqrt(diag(Pg))'; sdb = sqrt(diag(Pb))';
ep = wind.kf(:,1:3) - wind.p;
Ns = numel(wind.t);
sig3 = repmat(sdg, Ns, 1);                     % per-axis 1-sigma by fix status
degr = find(wind.fix <= 0.5);
sig3(degr, :) = repmat(sdb, numel(degr), 1);
f = figure('Color', 'w', 'Position', [50 50 900 380]);
for k = 1:3
    subplot(3, 1, k); hold on; grid on;
    tb = wind.t(sel); lo = ep(sel,k) - 3*sig3(sel,k); hi = ep(sel,k) + 3*sig3(sel,k);
    fill([tb; flipud(tb)], [lo; flipud(hi)], [1 0.85 0.85], 'EdgeColor', 'none');
    plot(wind.t(sel), ep(sel, k), 'b', 'LineWidth', 0.8);
    ylabel(sprintf('e_p_%c [m]', 'x' + k - 1));
end
xlabel('t [s]');
title('position estimation error with \pm3\sigma bounds (shaded: degraded fix)');
savepdf(f, fgs, 'est_err');

% --- fig: d-hat vs true wind drag ---
d_true = -P.kd*(wind.v - wind.vw)/P.m;
f = figure('Color', 'w', 'Position', [50 50 900 300]);
plot(wind.t, wind.kf(:,7:9), wind.t, d_true, '--', 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('[m/s^2]');
legend('\hat d_x', '\hat d_y', '\hat d_z', 'true d_x', 'true d_y', 'true d_z');
title('estimated disturbance acceleration vs wind drag');
savepdf(f, fgs, 'dhat');

% --- fig: RTK outage, covariance growth ---
f = figure('Color', 'w', 'Position', [50 50 900 300]);
hold on; grid on;
win = ertk.t >= 19 & ertk.t <= 27;
dr = vecnorm(ertk.kf(:,1:3) - ertk.p, 2, 2);
plot(ertk.t(win), dr(win), 'b', 'LineWidth', 1);
sdh = sqrt(Pg(1,1) + Pg(2,2));
sdl = sqrt(Pb(1,1) + Pb(2,2));
yline(sdh, ':'); yline(sdl, '--');
xline(20, ':'); xline(25, ':');
xlabel('t [s]'); ylabel('[m]');
legend('||\hat p - p||', '1\sigma RTK', '1\sigma degraded', 'Location', 'best');
title('estimation error during 5 s RTK outage (t = 20...25 s)');
savepdf(f, fgs, 'rtk_outage');

% --- fig: LQR analysis (eigenvalues, K pattern, steps) ---
[A, B, Ks, cl] = lqr_design(P);
f = figure('Color', 'w', 'Position', [50 50 980 300]);
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact');
nexttile(tl); hold on; grid on;
for i = 1:numel(Ks)
    e = cl(i).eig; plot(real(e), imag(e), 'o', 'MarkerSize', 4);
end
xlabel('Re'); ylabel('Im'); title('eig(A - BK) for \rho = 0.1, 1, 10');
nexttile(tl); imagesc(abs(Ks{2})); colorbar; colormap('gray');
xticks(1:15); xticklabels({'\int e_x','\int e_y','\int e_z','e_x','e_y','e_z','\dot e_x','\dot e_y','\dot e_z','\phi','\theta','\psi','\dot\phi','\dot\theta','\dot\psi'});
xtickangle(45); yticks(1:4); yticklabels({'\delta T', 'n_x', 'n_y', 'n_z'});
title('|K|, \rho = 1');
nexttile(tl); hold on; grid on;
st = cl(2).step;
plot(st(3).t, st(3).e, st(1).t, st(1).e, st(2).t, st(2).e, 'LineWidth', 0.9);
legend('z', 'x', 'y', 'Location', 'northeast');
xlabel('t [s]'); ylabel('e [m]'); title('1 m position-step error decay, \rho = 1');
savepdf(f, fgs, 'lqr');

% --- fig: Q comparison ---
S2 = load(fullfile(root, 'results', 'run_qexact.mat'));  oq0 = S2.out;
S2 = load(fullfile(root, 'results', 'run_qapprox.mat')); oq1 = S2.out;
f = figure('Color', 'w', 'Position', [50 50 900 300]);
plot(oq0.t, vecnorm(oq0.p - oq0.ref(:,1:3), 2, 2), ...
     oq1.t, vecnorm(oq1.p - oq1.ref(:,1:3), 2, 2), 'LineWidth', 0.8); grid on;
xlabel('t [s]'); ylabel('||e|| [m]');
legend('exact Q(\phi,\theta)', 'Q \approx I');
title('combined roll-pitch-yaw manoeuvre (heading 0 \rightarrow \pm90^{\circ})');
savepdf(f, fgs, 'qcompare');

fprintf('figures written to %s\n', fgs);
end

function savepdf(f, dir, name)
exportgraphics(f, fullfile(dir, [name '.pdf']), 'ContentType', 'vector');
saveas(f, fullfile(dir, [name '.png']));
close(f);
end
