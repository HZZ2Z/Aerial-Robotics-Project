P = params();
[A, B, Ks] = lqr_design(P); K = Ks{2};
[~, ~, Lg, Lb, Pg, Pb] = kf_design(P);
assignin('base','Pg',Pg); assignin('base','Pb',Pb);
S = load(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'scenarios_main.mat'));
o = S.T(6).o;   % estimated-state wind run
anim = animate_mission(o, P, 'mission_anim.mp4');
d = dir(anim); fprintf('mp4 size: %.1f MB\n', d.bytes/1e6);
