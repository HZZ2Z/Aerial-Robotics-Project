%RESUME_STAGE5 rebuild the batch struct from a saved full batch and run the
% figure/animation/export stages (used when a batch's sims are complete but
% the rendering stage failed; also serves as the RERUN comparison helper).
function resume_stage5(eid, export_only_mode)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'matlab'));
d = fullfile(root, 'results', eid);
P = params();
[~, ~, Ks, cl] = lqr_design(P);
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
B = load(fullfile(d, 'scenarios.mat'));  T = B.T;
g = @(c) load(fullfile(d, c, 'run.mat')).out;
rho = cell(1, 3);
for r = 1:3
    rho{r} = struct('M', [], 'o', []);
    for s = 1:2
        rho{r}(s).o = g(sprintf('rho%g_%s', P.lqr.rhos(r), char('a' + s - 1)));
        rho{r}(s).M = metrics(rho{r}(s).o, P, mdl);
    end
end
hov = struct('on', g('hover_integral_on'), 'off', g('hover_integral_off'));
S = struct('eid', eid, 'P', P, 'Ks', {Ks}, 'cl', {cl}, 'mdl', mdl, 'T', {T}, 'rho', {rho}, ...
    'hov', hov, 'qex', g('q_exact'), 'qap', g('q_approx'), ...
    'ff', struct('on', g('ff_on'), 'off', g('ff_off')), ...
    'nf', struct('s1', g('nofix1s'), 's5', g('nofix5s')), ...
    'dt2', g('est_wind_dt05'), 'holdout', struct(), 'rtkun', g('rtk_unbounded'));
hs = [1009 2027 4093 8191 16381];
for sd = hs
    S.holdout.(sprintf('wind%d', sd)) = g(sprintf('holdout_wind_%d', sd));
    S.holdout.(sprintf('rtk%d', sd))  = g(sprintf('holdout_rtk_%d', sd));
end
mode = 'all';
if nargin > 1, mode = export_only_mode; end
if any(strcmp(mode, {'all', 'figures_only'})), make_figures(S); end
if strcmp(mode, 'all')
    animate_mission(T(find(strcmp({T.case_id}, 'est_wind'))).o, P, 'mission_anim.mp4');
end
if any(strcmp(mode, {'all', 'export_only'})), export_results(S); end
fprintf('STAGE5 DONE (%s) for %s\n', mode, eid);
end
