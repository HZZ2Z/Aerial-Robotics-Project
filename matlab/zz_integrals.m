%ZZ_INTEGRALS quantify integral-state windup during the degraded window
addpath(fileparts(mfilename('fullpath')));
P = params();
d = dir(fullfile('results', 'full_*'));
S = load(fullfile(d(end).folder, d(end).name, 'est_rtk_lost', 'run.mat'));
o = S.out;
w = o.t >= 19.5 & o.t <= 26;
fprintf('integral states (x15 1:3) at t=20 / 22.5 / 25 / 26 / 30:\n');
for tt = [20 22.5 25 26 30]
    k = find(o.t >= tt, 1);
    fprintf('  t=%5.1f  int e = [%7.3f %7.3f %7.3f]  tilt=%5.1f deg\n', ...
        o.t(k), o.x15(k,1:3), acos(min(1,max(-1,o.R(k,9))))*180/pi);
end
% correlation between integral magnitude and tilt during the window
inrm = vecnorm(o.x15(w, 1:3), 2, 2);
trm = acos(min(1, max(-1, o.R(w, 9))))*180/pi;
fprintf('integral norm: max %.3f m*s in window; tilt corr with int norm: %.2f\n', ...
    max(inrm), corr(inrm, trm));
% thrust saturation fraction inside window
fprintf('thrust saturated frac in window: %.2f (0 or 24 N)\n', ...
    mean(o.Tn(w,1) <= 1e-6 | o.Tn(w,1) >= P.Tmax - 1e-6));
