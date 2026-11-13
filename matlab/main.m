%MAIN One-click reproduction of every figure and table of the report.
% Runtime ~25-35 min (11 mission simulations + figures + animation).
clear; close all; clc;
t0 = tic;
addpath(fileparts(mfilename('fullpath')));        % this folder
P = params();
fprintf('== 1/6 building models ==\n');
build_plant(P);
build_closed(P);

fprintf('== 2/6 control design ==\n');             % includes lin_hover self-checks
[A15, B15, Ks, cl] = lqr_design(P);
K = Ks{2};                                        % rho = 1 used in the loop
fprintf('== 3/6 filter design ==\n');             % includes observability checks
[A9, C9, Lg, Lb, Pg, Pb, ob] = kf_design(P);
kfdat = struct('Lg', Lg, 'Lb', Lb, 'Pg', Pg, 'Pb', Pb);

fprintf('== 4/6 four scenarios, ideal + estimated states ==\n');
T = run_scenarios(P, K, kfdat, 'main');

fprintf('== 5/6 Q(phi,theta) comparison + bonus runs ==\n');
Fq0 = struct('use_est', 0, 'use_Qapprox', 0, 'ff_wind', 0);
Fq1 = struct('use_est', 0, 'use_Qapprox', 1, 'ff_wind', 0);
oq0 = run_closed(P, K, Fq0, kfdat, struct('wind', 1, 'tag', 'qexact'));
oq1 = run_closed(P, K, Fq1, kfdat, struct('wind', 1, 'tag', 'qapprox'));
off = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',1), kfdat, ...
    struct('wind', 1, 'tag', 'ffwind'));
eon = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), kfdat, ...
    struct('wind', 1, 'tag', 'noffwind'));
on1 = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), kfdat, ...
    struct('wind', 1, 'no_fix', 1, 'fix_loss', [20 21], 'tag', 'nofix1'));
on5 = run_closed(P, K, struct('use_est',1,'use_Qapprox',0,'ff_wind',0), kfdat, ...
    struct('wind', 1, 'no_fix', 1, 'fix_loss', [20 25], 'tag', 'nofix5'));
for w = [1 5]
    o = eval(sprintf('on%d', w));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    fprintf('GNSS off %d s: drift at outage end %.3f m\n', w, dr(find(o.t >= 20+w, 1)));
end
Moff = metrics(off, P, Pg); Mon = metrics(eon, P, Pg);
fprintf('feed-forward: pos RMS %.4f (ff on) vs %.4f m (ff off)\n', Moff.pos_rms, Mon.pos_rms);

fprintf('== 6/6 figures and animation ==\n');
make_figures('main');
animate_mission(T(6).o, P, 'mission_anim.mp4');   % estimated-state wind run

fprintf('ALL DONE in %.1f min\n', toc(t0)/60);
