function export_results(S)
%EXPORT_RESULTS Structured metrics export from ONE execution batch.
% Writes results/<eid>/metrics.json (all runs) and report/metrics_macros.tex
% (numeric macros used by the paper and the README - single rounding source).
root = fileparts(fileparts(mfilename('fullpath')));
d = fullfile(root, 'results', S.eid);
if exist(d, 'dir') ~= 7, mkdir(d); end
J = struct('execution_id', S.eid, 'git_sha', git_sha(root), 'date', char(datetime('now')), ...
    'matlab', version('-release'), 'solver', 'ode4 (fixed step)', ...
    'seed_dev', S.P.noise.seed_base, 'kf_mode', S.T(1).o.mdl.mode);
J.core = batch_table(S.T);
J.rho = cell(1, 3);
for r = 1:3
    for s = 1:2
        J.rho{r}(s) = slim(S.rho{r}(s).M, sprintf('rho%g_%s', S.P.lqr.rhos(r), char('a'+s-1)));
    end
end
J.hover = struct('on', slim(metrics(S.hov.on, S.P, S.hov.on.mdl), 'hover_integral_on'), ...
                 'off', slim(metrics(S.hov.off, S.P, S.hov.off.mdl), 'hover_integral_off'));
J.q = struct('exact', slim(metrics(S.qex, S.P, S.qex.mdl), 'q_exact'), ...
             'approx', slim(metrics(S.qap, S.P, S.qap.mdl), 'q_approx'));
J.ff  = struct('on', slim(metrics(S.ff.on, S.P, S.ff.on.mdl), 'ff_on'), ...
               'off', slim(metrics(S.ff.off, S.P, S.ff.off.mdl), 'ff_off'));
J.nofix = struct();
for w = [1 5]
    o = S.nf.(sprintf('s%d', w));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    win = o.t >= 20 & o.t <= 20 + w;
    tilt = acos(min(1, max(-1, o.R(:, 9))));   % re-acquisition transient visibility:
                                               % recorded so a post-outage divergence
                                               % cannot hide behind the drift window
    J.nofix.(sprintf('s%d', w)) = struct('drift_end', dr(find(o.t >= 20+w, 1)), ...
        'drift_max_window', max(dr(win)), 'tilt_max_deg', r1(max(tilt)*180/pi), ...
        'pos_max', r3(max(vecnorm(o.p - o.ref(:,1:3), 2, 2))));
end
J.dt_sensitivity = struct('dt1', slim(metrics(S.T(find(strcmp({S.T.case_id},'est_wind'))).o, ...
                    S.P, S.T(find(strcmp({S.T.case_id},'est_wind'))).o.mdl), 'est_wind_dt1'), ...
    'dt05', slim(metrics(S.dt2, S.P, S.dt2.mdl), 'est_wind_dt05'));
J.holdout = struct();
fns = fieldnames(S.holdout);
for k = 1:numel(fns)
    J.holdout.(fns{k}) = slim(metrics(S.holdout.(fns{k}), S.P, S.holdout.(fns{k}).mdl), fns{k});
end
% in-batch control ablation: est_rtk_lost with the integrator clamp removed
oru = S.rtkun;
if iscell(oru), oru = oru{1}; end   % run_project passes {rtkun}, resume_stage5 plain
J.ablation = struct('rtk_unbounded', ...
    slim(metrics(oru, oru.P, oru.mdl), 'rtk_unbounded'));

% JSON
fid = fopen(fullfile(d, 'metrics.json'), 'w');
fwrite(fid, jsonmin(J));  fclose(fid);

% LaTeX macros (rounded once here, reused verbatim everywhere)
ci = @(c) find(strcmp({S.T.case_id}, c), 1);
iw = ci('est_wind');  irtk = ci('est_rtk_lost');  inom = ci('ideal_nominal');
dr1 = drift_end(S.nf.s1, 1);  dr5 = drift_end(S.nf.s5, 5);
ien = ci('est_nominal');  ijpl = ci('est_uncert_Jplus');  ijmi = ci('est_uncert_Jminus');
Mac = sprintf(['\\newcommand{\\posRMSwind}{%.3f}\n', ...
    '\\newcommand{\\posRMSnominalI}{%.3f}\n', ...
    '\\newcommand{\\posRMSnominalE}{%.3f}\n', ...
    '\\newcommand{\\posRMSrtk}{%.3f}\n', ...
    '\\newcommand{\\posRMSuncertPlus}{%.3f}\n', ...
    '\\newcommand{\\posRMSuncertMinus}{%.3f}\n', ...
    '\\newcommand{\\neesGood}{%.1f}\n', ...
    '\\newcommand{\\neesDegraded}{%.1f}\n', ...
    '\\newcommand{\\neesAll}{%.1f}\n', ...
    '\\newcommand{\\covGood}{%.1f}\n', ...
    '\\newcommand{\\covDegraded}{%.1f}\n', ...
    '\\newcommand{\\tiltMax}{%.1f}\n', ...
    '\\newcommand{\\tiltRtk}{%.1f}\n', ...
    '\\newcommand{\\estDwind}{%.2f}\n', ...
    '\\newcommand{\\hovBiasOn}{%.4f}\n', ...
    '\\newcommand{\\hovBiasOff}{%.4f}\n', ...
    '\\newcommand{\\ffOn}{%.3f}\n', ...
    '\\newcommand{\\ffOff}{%.3f}\n', ...
    '\\newcommand{\\driftOne}{%.2f}\n', ...
    '\\newcommand{\\driftFive}{%.2f}\n', ...
    '\\newcommand{\\dtRatio}{%.3f}\n', ...
    '\\newcommand{\\neesNx}{%d}\n', ...
    '\\newcommand{\\posRMSrtkUnbounded}{%.3f}\n', ...
    '\\newcommand{\\tiltNofixFive}{%.1f}\n', ...
    '\\newcommand{\\batchId}{%s}\n'], ...
    S.T(iw).M.pos_rms, S.T(inom).M.pos_rms, S.T(ien).M.pos_rms, S.T(irtk).M.pos_rms, ...
    S.T(ijpl).M.pos_rms, S.T(ijmi).M.pos_rms, ...
    S.T(iw).M.nees.good.mean, S.T(irtk).M.nees.degraded.mean, S.T(iw).M.nees.all.mean, ...
    100*S.T(iw).M.cov3.good.frac, 100*S.T(irtk).M.cov3.degraded.frac, ...
    S.T(iw).M.tilt_max*180/pi, S.T(irtk).M.tilt_max*180/pi, S.T(iw).M.est_rms(3), ...
    norm(mean(S.hov.on.p(S.hov.on.t > 35 & S.hov.on.t <= 45, :) - ...
              S.hov.on.ref(S.hov.on.t > 35 & S.hov.on.t <= 45, 1:3), 1)), ...
    norm(mean(S.hov.off.p(S.hov.off.t > 35 & S.hov.off.t <= 45, :) - ...
              S.hov.off.ref(S.hov.off.t > 35 & S.hov.off.t <= 45, 1:3), 1)), ...
    metrics(S.ff.on, S.P, S.ff.on.mdl).pos_rms, metrics(S.ff.off, S.P, S.ff.off.mdl).pos_rms, ...
    dr1, dr5, ...
    metrics(S.dt2, S.P, S.dt2.mdl).pos_rms / S.T(iw).M.pos_rms, ...
    S.T(iw).o.mdl.nx, J.ablation.rtk_unbounded.pos_rms, ...
    J.nofix.s5.tilt_max_deg, strrep(S.eid, '_', '\_'));
fid = fopen(fullfile(root, 'report', 'metrics_macros.tex'), 'w');
fwrite(fid, Mac);  fclose(fid);

% rho comparison + step-recovery metrics table (design-time outputs)
st = S.cl(3).step;   % rho = 10 (flight config)
fid = fopen(fullfile(root, 'report', 'rho_table.tex'), 'w');
fprintf(fid, '%% auto-generated by export_results.m - do not edit\n');
fprintf(fid, '\\begin{tabular}{lrrrrrr}\n\\toprule\n');
fprintf(fid, ['& \\multicolumn{2}{c}{$E_{\\rm RMS}$ [m]} & ' ...
    '\\multicolumn{2}{c}{$U_{\\rm RMS}$} & \\multicolumn{2}{c}{max tilt [$^\\circ$]}\\\\\n']);
fprintf(fid, ['$\\rho$ & no wind & wind & no wind & wind & no wind & wind \\\\\n\\midrule\n']);
for r = 1:3
    fprintf(fid, '%g & %.3f & %.3f & %.2f & %.2f & %.1f & %.1f \\\\\n', ...
        S.P.lqr.rhos(r), S.rho{r}(1).M.pos_rms, S.rho{r}(2).M.pos_rms, ...
        S.rho{r}(1).M.effort_rms, S.rho{r}(2).M.effort_rms, ...
        S.rho{r}(1).M.tilt_max*180/pi, S.rho{r}(2).M.tilt_max*180/pi);
end
fprintf(fid, ['\\midrule\n\\multicolumn{7}{l}{recovery from 1 m step, $\\rho=%g$ ' ...
    '(10--90\\%% rise / overshoot / 2\\%%-band settling, per axis):}\\\\\n'], ...
    S.P.lqr.rhos(3));
fprintf(fid, 'x & %.2f s & %.1f\\%% & %.2f s & & & \\\\\n', st(1).rise, st(1).over, st(1).sett);
fprintf(fid, 'y & %.2f s & %.1f\\%% & %.2f s & & & \\\\\n', st(2).rise, st(2).over, st(2).sett);
fprintf(fid, 'z & %.2f s & %.1f\\%% & %.2f s & & & \\\\\n', st(3).rise, st(3).over, st(3).sett);
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);

% actual gains and covariance parameters of the delivered configuration
% (flight rho; filter intensities, fix-status error poles, steady-state
% position-error std from the design Riccati solutions)
eg = S.cl(3).eig;                             % eig(A - BK), rho = 10
mdl = S.mdl;
pg = sqrt(diag(mdl.Pg));  pb = sqrt(diag(mdl.Pb));   % steady-state 1sigma
fid = fopen(fullfile(root, 'report', 'gains_table.tex'), 'w');
fprintf(fid, '%% auto-generated by export_results.m from batch %s - do not edit\n', S.eid);
fprintf(fid, '\\begin{tabular}{lll}\n\\toprule\n');
fprintf(fid, ['\\multicolumn{3}{l}{\\textbf{Controller} ($\\rho=%g$, Bryson scales of Sec.~III):}\\\\\n'], S.P.lqr.rhos(3));
fprintf(fid, ['integrator clamp $\\xi_{\\max}$ & [%g, %g, %g] m\\,s & per-axis anti-windup\\\\\n'], S.P.lqr.xi_max);
fprintf(fid, ['closed-loop poles $\\mathrm{eig}(A\\!-\\!BK)$ & $\\max_t \\Re\\lambda_t=%.2f$ s$^{-1}$ & slowest decay\\\\\n'], max(real(eg)));
fprintf(fid, ['\\multicolumn{3}{l}{\\textbf{Estimator} (rw9, $n_x=%d$):}\\\\\n'], mdl.nx);
fprintf(fid, ['drive intensities $(q_a,q_d)$ & (%g, %g) & $Q_9=\\mathrm{diag}(0,q_a^2I,q_d^2I)$\\\\\n'], S.P.kf.qa, S.P.kf.qd);
fprintf(fid, ['measurement intensity $R$ & (%.2e, %.2e, %.2e) & RTK fix, $r=\\sigma^2/f_{\\rm rate}$\\\\\n'], mdl.Rg(1,1), mdl.Rg(2,2), mdl.Rg(3,3));
fprintf(fid, ['degraded $R_b$ & (%.2e, %.2e, %.2e) & fix lost\\\\\n'], mdl.Rb(1,1), mdl.Rb(2,2), mdl.Rb(3,3));
fprintf(fid, ['error poles $\\mathrm{eig}(A\\!-\\!L_gC)$ & $\\max_t\\Re\\lambda_t=%.2f$ s$^{-1}$ & RTK-good schedule\\\\\n'], max(real(eig(mdl.A - mdl.Lg*mdl.C))));
fprintf(fid, ['error poles $\\mathrm{eig}(A\\!-\\!L_bC)$ & $\\max_t\\Re\\lambda_t=%.2f$ s$^{-1}$ & degraded schedule\\\\\n'], max(real(eig(mdl.A - mdl.Lb*mdl.C))));
fprintf(fid, ['steady-state $1\\sigma$ of $\\tilde p$ & (%.3f, %.3f, %.3f) m & RTK good\\\\\n'], pg(1), pg(2), pg(3));
fprintf(fid, [' & (%.3f, %.3f, %.3f) m & degraded\\\\\n'], pb(1), pb(2), pb(3));
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);

% core-results table (booktabs), generated from the same metrics
fid = fopen(fullfile(root, 'report', 'metrics_table.tex'), 'w');
fprintf(fid, '%% auto-generated by export_results.m from batch %s - do not edit\n', S.eid);
fprintf(fid, '\\begin{tabular}{llrrrrrr}\n\\toprule\n');
fprintf(fid, ['Feedback & Scenario & $E_{\\rm RMS}$ [m] & $E_{\\max}$ [m] & ' ...
    '$\\|\\mathbf{b}\\|$ [m] & est.\\ $p$/$v$/$d$ RMS & tilt [$^\\circ$] & $U_{\\rm RMS}$ \\\\\n']);
fprintf(fid, '\\midrule\n');
for k = 1:numel(S.T)
    M = S.T(k).M;
    if strcmp(S.T(k).mode, 'ideal')
        est = 'N/A';
    else
        est = sprintf('%.3f/%.2f/%.2f', M.est_rms);
    end
    fprintf(fid, '%s & %s & %.3f & %.3f & %.3f & %s & %.1f & %.2f \\\\\n', ...
        S.T(k).mode, strrep(S.T(k).scen, '_', ' '), M.pos_rms, M.pos_max, ...
        M.hov_off_norm, est, M.tilt_max*180/pi, M.effort_rms);
end
fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
fclose(fid);
fprintf('metrics exported: %s\n', fullfile(d, 'metrics.json'));
end

function T = batch_table(T)
for k = 1:numel(T)
    T(k).metrics = slim(T(k).M, T(k).case_id);
    T(k).seed = T(k).seed;
    T(k).o = [];                              % raw logs stay in run.mat
    T(k).M = [];
end
end

function m = slim(M, name)
m = struct('case', name, ...
    'pos_rms', r3(M.pos_rms), 'pos_max', r3(M.pos_max), ...
    'hov_off', arrayfun(@r3, M.hov_off), 'hov_off_norm', r3(M.hov_off_norm), ...
    'hov_rms', r3(M.hov_rms), ...
    'est_rms_pvd', arrayfun(@r4, M.est_rms), 'est_max', r3(M.est_max), ...
    'tilt_max_deg', r1(M.tilt_max*180/pi), ...
    'effort_rms', r3(M.effort_rms), 'T_dev_rms', r3(M.T_dev_rms), ...
    'n_max', arrayfun(@r3, M.n_max));
if isfield(M, 'nees')
    m.nees = r2(M.nees.all.mean);  m.cov3 = r2(100*M.cov3.all.frac);
end
end

function s = git_sha(root)
manifest = fullfile(root, 'MANIFEST.json');
if isfile(manifest)
    info = jsondecode(fileread(manifest));
    s = info.git_head;
    return;
end
[status, s] = system(sprintf('git -C "%s" rev-parse --short HEAD 2>&1', root));
if status == 0
    s = strtrim(s);
else
    s = 'unavailable';
end
end

function s = jsonmin(x)
% minimal JSON writer for the plain nested structs used here
s = enc(x, 0);
    function out = enc(v, ind)
        pad = repmat(' ', 1, ind*2);
        if isstruct(v) && numel(v) == 1
            fn = fieldnames(v);
            parts = cell(1, numel(fn));
            for k = 1:numel(fn)
                parts{k} = sprintf('%s"%s": %s', pad, fn{k}, enc(v.(fn{k}), ind+1));
            end
            out = ['{' newline strjoin(parts, [',' newline]) newline pad '}'];
        elseif isstruct(v)
            parts = cellfun(@(c) enc(c, ind+1), num2cell(v), 'UniformOutput', false);
            out = ['[' newline strjoin(parts, [',' newline]) newline pad ']'];
        elseif isnumeric(v) && isscalar(v)
            out = sprintf('%.6g', v);
        elseif isnumeric(v)
            out = ['[' strjoin(arrayfun(@(z) sprintf('%.6g', z), v, 'UniformOutput', false), ', ') ']'];
        elseif ischar(v)
            out = sprintf('"%s"', v);
        elseif iscell(v)
            parts = cellfun(@(c) enc(c, ind+1), v, 'UniformOutput', false);
            out = ['[' strjoin(parts, ', ') ']'];
        elseif islogical(v)
            out = num2str(v);
        else
            out = 'null';
        end
    end
end

function y = r1(x), y = round(x, 1); end
function y = r2(x), y = round(x, 2); end
function y = r3(x), y = round(x, 3); end
function y = r4(x), y = round(x, 4); end

function d = drift_end(o, w)
dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
d = dr(find(o.t >= 20 + w, 1));
end
