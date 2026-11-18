%ZZ_TILTFIX diagnostic: candidate designs for the degraded-RTK tilt constraint
% Set WHICH = 'ABCD' for the full battery; 'E' runs only the qd=0.3 follow-up.
addpath(fileparts(mfilename('fullpath')));
P = params();
[~, ~, Ks, ~] = lqr_design(P);
slim = @(m) rmfield(m, setdiff(fieldnames(m), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
mdl = slim(kf_design8(P));
Fe = struct('use_est', 1, 'use_Qapprox', 0, 'ff_wind', 0);
opt = struct('wind_mode', 'mean_gust', 'fix_loss', [20 25], 'tag', 'zz');
pos = @(o) sqrt(mean(vecnorm(o.p - o.ref(:,1:3), 2, 2).^2));
tilt = @(o) max(acos(min(1, max(-1, o.R(:,9))))*180/pi);
which = 'FGH';

if any(which == 'A')  % rho = 10 (softer control)
    oa = run_closed(P, Ks{3}, Fe, mdl, opt);
    fprintf('A rho=10        : tilt %.1f deg, posRMS %.3f m\n', tilt(oa), pos(oa));
end
if any(which == 'B')  % qd = 3 (larger Lb -> more measurement chasing)
    Pb = P;  Pb.kf.qd = 3;
    ob = run_closed(Pb, Ks{2}, Fe, slim(kf_design8(Pb)), opt);
    fprintf('B qd=3 rho=1    : tilt %.1f deg, posRMS %.3f m\n', tilt(ob), pos(ob));
end
if any(which == 'C')  % blend_s = 1.5 (soft re-acquisition; peak is mid-window)
    Pc = P;  Pc.kf.reacq_blend_s = 1.5;
    oc = run_closed(Pc, Ks{2}, Fe, slim(kf_design8(Pc)), opt);
    fprintf('C blend 1.5s    : tilt %.1f deg, posRMS %.3f m\n', tilt(oc), pos(oc));
end
if any(which == 'D')  % rho = 10 AND qd = 3
    Pd = P;  Pd.kf.qd = 3;
    od = run_closed(Pd, Ks{3}, Fe, slim(kf_design8(Pd)), opt);
    fprintf('D rho=10 qd=3   : tilt %.1f deg, posRMS %.3f m\n', tilt(od), pos(od));
end
if any(which == 'E')  % qd = 0.3 (smaller Lb -> smoother estimates)
    PE = P;  PE.kf.qd = 0.3;
    oe = run_closed(PE, Ks{2}, Fe, slim(kf_design8(PE)), opt);
    fprintf('E qd=0.3 rho=1  : tilt %.1f deg, posRMS %.3f m\n', tilt(oe), pos(oe));
end

if any(which == 'F')  % rho = 10 AND qd = 0.3
    PF = P;  PF.kf.qd = 0.3;
    of = run_closed(PF, Ks{3}, Fe, slim(kf_design8(PF)), opt);
    fprintf('F rho=10 qd=0.3 : tilt %.1f deg, posRMS %.3f m', tilt(of), pos(of));
end
if any(which == 'G')  % qd = 0.1
    PG = P;  PG.kf.qd = 0.1;
    og = run_closed(PG, Ks{2}, Fe, slim(kf_design8(PG)), opt);
    fprintf('G qd=0.1 rho=1  : tilt %.1f deg, posRMS %.3f m', tilt(og), pos(og));
end
if any(which == 'H')  % rho = 10, qa = 0.5, qd = 0.3
    PH = P;  PH.kf.qa = 0.5;  PH.kf.qd = 0.3;
    oh = run_closed(PH, Ks{3}, Fe, slim(kf_design8(PH)), opt);
    fprintf('H rho=10 qa=.5 qd=.3: tilt %.1f deg, posRMS %.3f m', tilt(oh), pos(oh));
end

function m = kf_design8(P)
[~, ~, ~, ~, ~, ~, ~, m] = kf_design(P);
end
