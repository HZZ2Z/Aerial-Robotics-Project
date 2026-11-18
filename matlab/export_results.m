function export_results(S)
%EXPORT_RESULTS Structured metrics export from ONE execution batch.
% Writes results/<eid>/metrics.json (all runs) and report/metrics_macros.tex
% (numeric macros used by the paper and the README - single rounding source).
root = fileparts(fileparts(mfilename('fullpath')));
d = fullfile(root, 'results', S.eid);
if exist(d, 'dir') ~= 7, mkdir(d); end
J = struct('execution_id', S.eid, 'git_sha', git_sha(), 'date', char(datetime('now')), ...
    'matlab', version('-release'), 'solver', 'ode4 (fixed step)', ...
    'seed_dev', S.P.noise.seed_base, 'kf_mode', S.T(1).o.mdl.mode);
J.core = batch_table(S.T);
J.rho = cell(1, 3);
for r = 1:3
    for s = 1:2
        J.rho{r}(s) = slim(S.rho{r}(s).M, sprintf('rho%g_%s', S.P.lqr.rhos(r), char('a'+s-1)));
    end
end
J.hover = struct('on', slim(S.hov.on.M, 'hover_integral_on'), ...
                 'off', slim(S.hov.off.M, 'hover_integral_off'));
J.q = struct('exact', slim(metrics(S.qex, S.P, S.qex.mdl), 'q_exact'), ...
             'approx', slim(metrics(S.qap, S.P, S.qap.mdl), 'q_approx'));
J.ff  = struct('on', slim(metrics(S.ff.on, S.P, S.ff.on.mdl), 'ff_on'), ...
               'off', slim(metrics(S.ff.off, S.P, S.ff.off.mdl), 'ff_off'));
J.nofix = struct();
for w = [1 5]
    o = S.nf.(sprintf('s%d', w));
    dr = vecnorm(o.kf(:,1:3) - o.p, 2, 2);
    win = o.t >= 20 & o.t <= 20 + w;
    J.nofix.(sprintf('s%d', w)) = struct('drift_end', dr(find(o.t >= 20+w, 1)), ...
        'drift_max_window', max(dr(win)));
end
J.dt_sensitivity = struct('dt1', slim(metrics(S.T(find(strcmp({S.T.case_id},'est_wind'))).o, ...
                    S.P, S.T(find(strcmp({S.T.case_id},'est_wind'))).o.mdl), 'est_wind_dt1'), ...
    'dt05', slim(metrics(S.dt2, S.P, S.dt2.mdl), 'est_wind_dt05'));
J.holdout = struct();
fns = fieldnames(S.holdout);
for k = 1:numel(fns)
    J.holdout.(fns{k}) = slim(metrics(S.holdout.(fns{k}), S.P, S.holdout.(fns{k}).mdl), fns{k});
end

% JSON
fid = fopen(fullfile(d, 'metrics.json'), 'w');
fwrite(fid, jsonmin(J));  fclose(fid);

% LaTeX macros (rounded once here, reused verbatim everywhere)
iw = find(strcmp({S.T.case_id}, 'est_wind'));  irtk = find(strcmp({S.T.case_id}, 'est_rtk_lost'));
inom = find(strcmp({S.T.case_id}, 'ideal_nominal'));
Mac = sprintf(['\\newcommand{\\posRMSwind}{%.3f}\n', ...
    '\\newcommand{\\posRMSnominalI}{%.3f}\n', ...
    '\\newcommand{\\posRMSrtk}{%.3f}\n', ...
    '\\newcommand{\\neesGood}{%.1f}\n', ...
    '\\newcommand{\\covGood}{%.1f}\n', ...
    '\\newcommand{\\tiltMax}{%.1f}\n', ...
    '\\newcommand{\\neesNx}{%d}\n'], ...
    S.T(iw).M.pos_rms, S.T(inom).M.pos_rms, S.T(irtk).M.pos_rms, ...
    S.T(iw).M.nees.good.mean, 100*S.T(iw).M.cov3.good.frac, ...
    S.T(iw).M.tilt_max*180/pi, S.T(iw).o.mdl.nx);
fid = fopen(fullfile(root, 'report', 'metrics_macros.tex'), 'w');
fwrite(fid, Mac);  fclose(fid);
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
    'n_max', arrayfun(@r3, M.n_max), 'sat_frac', r4(M.sat_frac));
if isfield(M, 'nees')
    m.nees = r2(M.nees.all.mean);  m.cov3 = r2(100*M.cov3.all.frac);
end
end

function s = git_sha()
[~, s] = system('git rev-parse --short HEAD');
s = strtrim(s);
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
