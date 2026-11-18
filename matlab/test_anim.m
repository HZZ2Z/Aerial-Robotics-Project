%TEST_ANIM Regenerate the mission animation from a saved run.mat.
P = params();
[~, ~, ~, ~, ~, ~, ~, mdl] = kf_design(P);
mdl = rmfield(mdl, setdiff(fieldnames(mdl), {'mode','nx','tau','A','B','C','Tc', ...
    'Q','Rg','Rb','Lg','Lb','Pg','Pb','blend_s','m','g','q'}));
% latest full batch, estimated-state wind run
d = dir(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'full_*'));
assert(~isempty(d), 'no full_* batch in results/');
S = load(fullfile(d(end).folder, d(end).name, 'scenarios.mat'));
o = S.T(find(strcmp({S.T.case_id}, 'est_wind'), 1)).o;
anim = animate_mission(o, P, 'mission_anim.mp4');
d = dir(anim); fprintf('mp4 size: %.1f MB\n', d.bytes/1e6);
