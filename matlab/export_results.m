function export_results(S)
%EXPORT_RESULTS Same-batch machine-readable data, gains and paper tables.
root = fileparts(fileparts(mfilename('fullpath')));
d = fullfile(root,'results',S.eid);
report = fullfile(root,'report');
P = S.P;
M = S.mdl;
J = struct('execution_id',S.eid,'matlab',version('-release'),'solver','ode4', ...
    'seed_dev',P.noise.seed_base,'parameters',P,'A',S.A,'B',S.B, ...
    'K',{S.Ks},'kf',M,'hover_decay',reshape(max(real(S.poles),[],1),2,[]), ...
    'hover_real',real(S.poles),'hover_imag',imag(S.poles));
J.kf = rmfield(J.kf,{'poles_g','poles_b'});
for k = 1:numel(S.T)
    c = S.T(k);
    J.core(k) = struct('case_id',c.case_id,'feedback',c.mode,'case',c.scen, ...
        'seed',c.seed,'opts',c.o.opts,'metrics',c.M);
end
for r = 1:numel(S.cl)
    C = S.cl(r);
    J.lqr(r) = struct('rho',P.lqr.rhos(r),'Q',C.Q,'R',C.R, ...
        'poles_real',real(C.eig),'poles_imag',imag(C.eig), ...
        'step_rise',[C.step.rise],'step_overshoot',[C.step.over], ...
        'step_settling',[C.step.sett],'mission',S.rho{r});
end
for mode = ["on","off"]
    J.hover.(mode) = metrics(S.hov.(mode),S.hov.(mode).P,M);
    J.ff.(mode) = metrics(S.ff.(mode),P,M);
end
for w = [1 5]
    key = sprintf('s%d',w);
    o = S.nf.(key);
    dr = vecnorm(o.kf(:,1:3)-o.p,2,2);
    window = o.t >= o.opts.fix_loss(1) & o.t < o.opts.fix_loss(2);
    a = metrics(o,P,M);
    J.nofix.(key) = struct('drift_end',dr(find(window,1,'last')), ...
        'drift_max',max(dr(window)),'tilt_max',a.tilt_max,'pos_max',a.pos_max);
end
for key = string(fieldnames(S.holdout))'
    J.holdout.(key) = metrics(S.holdout.(key),P,M);
end
J.dt05 = metrics(S.dt2,P,M);
J.q_exact = metrics(S.qex,P,M);
J.q_approx = metrics(S.qap,P,M);
o = S.T(strcmp({S.T.case_id},'est_wind')).o;
wind_on = o.t > P.wind.t0 & o.t < P.sim.tfinal-P.mis.tland-P.mis.end_rest;
J.disturbance_mean = struct('estimate',mean(o.kf(wind_on,7:9),1), ...
    'wind_drag',mean(o.d_wind(wind_on,:),1));
ertk = S.T(strcmp({S.T.case_id},'est_rtk_lost')).o;
before = find(ertk.t < P.mis.fix_loss(1),1,'last');
during = find(ertk.t < P.mis.fix_loss(2),1,'last');
J.rtk_horizontal_sigma = sqrt(ertk.P_out([before during],1,1)+ertk.P_out([before during],2,2));
write_text(fullfile(d,'metrics.json'),jsonencode(J,'PrettyPrint',true));

f = fopen(fullfile(report,'metrics_macros.tex'),'w');
closeFile = onCleanup(@() fclose(f));
W = J.core(strcmp({J.core.case_id},'est_wind')).metrics;
R = J.core(strcmp({J.core.case_id},'est_rtk_lost')).metrics;
macro(f,'windRms','%.3f',W.pos_rms);
macro(f,'rtkRms','%.3f',R.pos_rms);
macro(f,'rtkPeak','%.3f',R.pos_max);
macro(f,'maxTilt','%.1f',max(arrayfun(@(c)c.metrics.tilt_max,J.core))*180/pi);
macro(f,'hoverOn','%.4f',J.hover.on.hov_off_norm);
macro(f,'hoverOff','%.4f',J.hover.off.hov_off_norm);
macro(f,'poleGood','%.3f',max(J.hover_decay(2,:)));
macro(f,'poleBad','%.3f',max(J.hover_decay(1,:)));
macro(f,'flightRho','%g',P.lqr.flight_rho);
macro(f,'dirtyEps','%.3f',P.lqr.eps_d);
macro(f,'processA','%g',P.kf.qa);
macro(f,'processD','%g',P.kf.qd);
macro(f,'goodCov','%.2f',100*R.cov3.good.frac);
macro(f,'badCov','%.2f',100*R.cov3.degraded.frac);
macro(f,'goodInnovation','(%+.3f,%+.3f,%+.3f)',R.innov.good.mean);
macro(f,'badInnovation','(%+.3f,%+.3f,%+.3f)',R.innov.degraded.mean);
macro(f,'goodSigma','%.3f',J.rtk_horizontal_sigma(1));
macro(f,'badSigma','%.3f',J.rtk_horizontal_sigma(2));
macro(f,'meanEstimate','%.3f',J.disturbance_mean.estimate(1));
macro(f,'meanDrag','%.3f',J.disturbance_mean.wind_drag(1));
macro(f,'ffOn','%.3f',J.ff.on.pos_rms);
macro(f,'ffOff','%.3f',J.ff.off.pos_rms);
macro(f,'driftOne','%.2f',J.nofix.s1.drift_end);
macro(f,'driftFive','%.2f',J.nofix.s5.drift_end);
macro(f,'driftMaxOne','%.2f',J.nofix.s1.drift_max);
macro(f,'driftMaxFive','%.2f',J.nofix.s5.drift_max);
macro(f,'tiltNoFix','%.1f',max(J.nofix.s1.tilt_max,J.nofix.s5.tilt_max)*180/pi);
macro(f,'dtRatio','%.3f',J.dt05.pos_rms/W.pos_rms);
macro(f,'batchId','%s',strrep(S.eid,'_','\_'));
clear closeFile

f = fopen(fullfile(report,'metrics_table.tex'),'w');
closeFile = onCleanup(@() fclose(f));
fprintf(f,'\\begin{tabular}{llrrrrrr}\\toprule\n');
fprintf(f,'Feedback & Case & RMS (m) & Max (m) & Hover (m) & Est. $p/v/d$ RMS & Tilt (deg) & Effort \\\\\\midrule\n');
for c = J.core
    a = c.metrics;
    est = '--';
    if strcmp(c.feedback,'estimated'), est = sprintf('%.3f/%.2f/%.2f',a.est_rms); end
    label = strrep(c.case,'_',' ');
    fprintf(f,'%s & %s & %.3f & %.3f & %.3f & %s & %.1f & %.3f \\\\\n', ...
        c.feedback,label,a.pos_rms,a.pos_max,a.hov_off_norm,est,a.tilt_max*180/pi,a.effort_rms);
end
table_end(f);
clear closeFile

f = fopen(fullfile(report,'rho_table.tex'),'w');
closeFile = onCleanup(@() fclose(f));
fprintf(f,'\\begin{tabular}{lrrrr}\\toprule\n');
fprintf(f,'$\\rho$ & RMS (still/wind) & Effort (still/wind) & Channel & $t_r$/OS/$t_s$ \\\\\\midrule\n');
for r = 1:numel(S.cl)
    Wt = S.rho{r};
    st = S.cl(r).step;
    fprintf(f,'%g & %.3f/%.3f & %.3f/%.3f',P.lqr.rhos(r), ...
        Wt(1).M.pos_rms,Wt(2).M.pos_rms,Wt(1).M.effort_rms,Wt(2).M.effort_rms);
    fprintf(f,' & $x$ & %.2f/%.1f/%.2f \\\\\n',st(1).rise,st(1).over,st(1).sett);
    fprintf(f,' & & & $y$ & %.2f/%.1f/%.2f \\\\\n',st(2).rise,st(2).over,st(2).sett);
    fprintf(f,' & & & $z$ & %.2f/%.1f/%.2f \\\\\n',st(3).rise,st(3).over,st(3).sett);
    fprintf(f,' & & & $\\psi$ & %.2f/%.1f/%.2f \\\\\n',st(4).rise,st(4).over,st(4).sett);
end
table_end(f);
clear closeFile

f = fopen(fullfile(report,'gains_table.tex'),'w');
closeFile = onCleanup(@() fclose(f));
fprintf(f,'\\begin{tabular}{lll}\\toprule\n');
fprintf(f,'Input & State indices & Corresponding entries of $K$ \\\\\\midrule\n');
K = S.Ks{S.selected};
rows = {'\delta T','n_x','n_y','n_z'};
for k = 1:4
    nz = find(abs(K(k,:)) > 1e-7);
    fprintf(f,'$%s$ & %s & %s \\\\\n',rows{k}, ...
        strjoin(compose('%d',nz),', '),strjoin(compose('%.4f',K(k,nz)),', '));
end
fprintf(f,'\\midrule\n Mode & Axis & $L_{p}, L_{v}, L_{d}$ \\\\\\midrule\n');
fprintf(f,'RTK & $x,y$ & %.3f, %.3f, %.3f \\\\\n',M.Lg([1 4 7],1));
fprintf(f,' & $z$ & %.3f, %.3f, %.3f \\\\\n',M.Lg([3 6 9],3));
fprintf(f,'Degraded & $x,y$ & %.3f, %.3f, %.3f \\\\\n',M.Lb([1 4 7],1));
fprintf(f,' & $z$ & %.3f, %.3f, %.3f \\\\\n',M.Lb([3 6 9],3));
table_end(f);
fprintf('Metrics and gains exported: %s\n',d);
end

function macro(f,name,fmt,data)
fprintf(f,'\\newcommand{\\%s}{',name);
fprintf(f,fmt,data);
fprintf(f,'}\n');
end

function table_end(f)
fprintf(f,'\\bottomrule\\end{tabular}\n');
end

function write_text(path,data)
f = fopen(path,'w');
assert(f >= 0,'Unable to open %s',path);
c = onCleanup(@() fclose(f));
fprintf(f,'%s\n',data);
end
