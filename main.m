function main(mode)
%MAIN Run the Aerial Robotics coursework from design to simulation and results.
%   main('quick') runs two representative cases.
%   main('full')  runs all cases and generates the report data and animation.
if nargin < 1
    mode = 'full';
end
mode = lower(char(mode));
assert(any(strcmp(mode,{'quick','full'})), 'Use main(''quick'') or main(''full'').');

root = fileparts(mfilename('fullpath'));
source = fullfile(root,'matlab');
kit = fullfile(root,'simulink','student_kit');
required = {'quad_arch.slx','mission_reference.m','euler_R.m'};
missing = required(~isfile(fullfile(kit,required)));
if ~isempty(missing)
    error('Place %s from the course student kit in %s.',strjoin(missing,', '),kit);
end
addpath(source,fullfile(source,'experiments'),kit);
started = tic;
batchId = [mode '_' char(datetime('now','Format','yyyyMMdd_HHmmss'))];

%% 1. Coursework parameters and controller/observer design
P = params();
[A, B, gains, design] = lqr_design(P);
flightIndex = find(P.lqr.rhos == P.lqr.flight_rho,1);
K = gains{flightIndex};
[~,~,~,~,~,~,~,filter] = kf_design(P);
hoverPoles = course_hover_stability(P,K,filter);

%% 2. Teacher's Simulink model and core flight cases
build_course_model(P);
cases = run_scenarios(P,K,filter,struct('execution_id',batchId,'mode',mode));
if strcmp(mode,'quick')
    fprintf('QUICK DONE in %.1f min (batch %s)\n',toc(started)/60,batchId);
    return
end
assert(all(arrayfun(@(c) c.M.tilt_max,cases) <= P.tiltmax), ...
    'A core scenario exceeded the course tilt limit.');

%% 3. Comparisons and bonus experiments
[rho, hover, exactQ, approxQ, feedforward, noFix, halfStep, holdout] = ...
    run_experiments(P,K,gains,filter,batchId,cases);

%% 4. Report tables, figures and animation
S = struct('eid',batchId,'P',P,'Ks',{gains},'cl',design,'mdl',filter,'T',cases, ...
    'rho',{rho},'hov',hover,'qex',exactQ,'qap',approxQ,'ff',feedforward,'nf',noFix, ...
    'dt2',halfStep,'holdout',holdout,'A',A,'B',B,'poles',hoverPoles,'selected',flightIndex);
save(fullfile(root,'results',batchId,'batch.mat'),'S','-v7.3');
export_results(S);
make_figures(S);
animate_mission(feedforward.off,P,fullfile(batchId,'mission_anim.mp4'));
fprintf('FULL DONE in %.1f min (batch %s)\n',toc(started)/60,batchId);
end
