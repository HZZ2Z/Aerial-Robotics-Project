%ZZ_SPIKE what exactly drives the tilt spike at t ~ 23.4-24.6 s
addpath(fileparts(mfilename('fullpath')));
P = params();
d = dir(fullfile('results', 'full_*'));
S = load(fullfile(d(end).folder, d(end).name, 'est_rtk_lost', 'run.mat'));
o = S.out;
kpk = find(abs(o.t - 24.0) < 1e-9, 1);
fprintf('at t=24.0:\n');
fprintf('  true  p = [%.3f %.3f %.3f], ref = [%.3f %.3f %.3f]\n', o.p(kpk,:), o.ref(kpk,1:3));
fprintf('  est   p = [%.3f %.3f %.3f], est err = [%.3f %.3f %.3f]\n', ...
    o.kf(kpk,1:3), o.kf(kpk,1:3) - o.p(kpk,:));
fprintf('  true  v = [%.3f %.3f %.3f], ref v = [%.3f %.3f %.3f]\n', o.v(kpk,:), o.ref(kpk,4:6));
fprintf('  est   v = [%.3f %.3f %.3f], v est err = [%.3f %.3f %.3f]\n', ...
    o.kf(kpk,4:6), o.kf(kpk,4:6) - o.v(kpk,:));
fprintf('  x15 e=[%.3f %.3f %.3f] ed=[%.3f %.3f %.3f] etat=[%.1f %.1f %.1f] deg\n', ...
    o.x15(kpk,4:6), o.x15(kpk,7:9), o.x15(kpk,10:12)*180/pi);
fprintf('  applied Tn = [%.2f %.3f %.3f %.3f]  ucmd = [%.2f %.3f %.3f %.3f]\n', ...
    o.Tn(kpk,:), o.ucmd(kpk,:));
% time series around the spike
w = o.t >= 22.8 & o.t <= 25.2;
fprintf('\n  t      ep_est(3)   ev_est(3)   tilt    Tapp    nx     ny\n');
ks = find(w);  ks = ks(1:24:end);
for k = ks
    fprintf('%6.2f  %7.3f  %7.3f  %5.1f  %6.2f  %6.3f %6.3f\n', o.t(k), ...
        vecnorm(o.kf(k,1:3) - o.p(k,:)), vecnorm(o.kf(k,4:6) - o.v(k,:)), ...
        acos(min(1,max(-1,o.R(k,9))))*180/pi, o.Tn(k,1), o.Tn(k,2), o.Tn(k,3));
end
