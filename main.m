function main(mode)
%MAIN Run the Aerial Robotics coursework from design to simulation and results.
%   主入口脚本：一键完成控制器/滤波器设计、Simulink建模、全工况仿真与报告图表生成。
%   main('quick') runs two representative cases.   (快速模式：设计校验 + 2个代表性工况)
%   main('full')  runs all cases and generates the report data and animation. (完整模式：全部实验 + 论文图表 + 3D动画)
if nargin < 1
    mode = 'full';
end
mode = lower(char(mode));
assert(any(strcmp(mode,{'quick','full'})), 'Use main(''quick'') or main(''full'').');

% 1) 检查课程提供的 student_kit 模板文件是否就位，并加入路径
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

%% 1. Coursework parameters and controller/observer design (参数加载与控制器/观测器设计)
P = params();
[A, B, gains, design] = lqr_design(P);                      % 悬停线性化与 16 维积分 LQR 设计
flightIndex = find(P.lqr.rhos == P.lqr.flight_rho,1);       % 选取默认飞行权重 rho = 10 对应的增益
K = gains{flightIndex};
[~,~,~,~,~,~,~,filter] = kf_design(P);                      % 9 维稳态连续时间卡尔曼滤波器设计
hoverPoles = course_hover_stability(P,K,filter);            % 31 维输出反馈全闭环悬停稳定性校验

%% 2. Teacher's Simulink model and core flight cases (构建 Simulink 模型并运行核心飞行工况)
build_course_model(P);
cases = run_scenarios(P,K,filter,struct('execution_id',batchId,'mode',mode));
if strcmp(mode,'quick')
    fprintf('QUICK DONE in %.1f min (batch %s)\n',toc(started)/60,batchId);
    return
end
% 校验所有核心工况均未超过最大倾角限制 (0.61 rad)
assert(all(arrayfun(@(c) c.M.tilt_max,cases) <= P.tiltmax), ...
    'A core scenario exceeded the course tilt limit.');

%% 3. Comparisons and bonus experiments (对比实验与 Bonus 附加实验)
[rho, hover, exactQ, approxQ, feedforward, noFix, halfStep, holdout] = ...
    run_experiments(P,K,gains,filter,batchId,cases);

%% 4. Report tables, figures and animation (导出实验数据、生成论文图表与 3D 飞行动画)
S = struct('eid',batchId,'P',P,'Ks',{gains},'cl',design,'mdl',filter,'T',cases, ...
    'rho',{rho},'hov',hover,'qex',exactQ,'qap',approxQ,'ff',feedforward,'nf',noFix, ...
    'dt2',halfStep,'holdout',holdout,'A',A,'B',B,'poles',hoverPoles,'selected',flightIndex);
save(fullfile(root,'results',batchId,'batch.mat'),'S','-v7.3');
export_results(S);
make_figures(S);
animate_mission(feedforward.off,P,fullfile(batchId,'mission_anim.mp4'));
fprintf('FULL DONE in %.1f min (batch %s)\n',toc(started)/60,batchId);
end
