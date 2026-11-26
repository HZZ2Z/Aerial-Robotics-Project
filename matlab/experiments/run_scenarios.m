function results = run_scenarios(P, K, filter, options)
%RUN_SCENARIOS 运行课程要求的 5 种典型工况 x 2 种反馈模式（理想真值 vs 卡尔曼估计），共 10 组核心仿真。
%   options.mode = 'quick' 时仅运行 ideal_nominal 与 est_rtk_lost 两组快速验证。
if nargin < 4
    options = struct();
end
if ~isfield(options,'mode')
    options.mode = 'full';
end
if ~isfield(options,'execution_id')
    options.execution_id = ['main_' char(datetime('now','Format','yyyyMMdd_HHmmss'))];
end
if ~isfield(options,'seed')
    options.seed = P.noise.seed_base;
end
quick = strcmp(options.mode,'quick');

% 核心工况矩阵：工况名 | 风场模式 | RTK 失锁区间 | 质量倍率 | 惯量倍率 | 输入延迟 [s]
conditions = {
    'nominal',     'none',      [inf inf],      1,   1,   0
    'wind',        'mean_gust', [inf inf],      1,   1,   0
    'rtk_lost',    'mean_gust', P.mis.fix_loss, 1,   1,   0
    'uncert_Jplus','mean_gust', [inf inf],      1.2, 1.2, 0.02
    'uncert_Jminus','mean_gust',[inf inf],      1.2, 0.8, 0.02
    };
feedbackNames = {'ideal','estimated'};
casePrefixes = {'ideal','est'};
quickCases = {'ideal_nominal','est_rtk_lost'};
results = struct('mode',{},'scen',{},'case_id',{},'seed',{},'M',{},'o',{});

fprintf('\n%-9s %-13s | %8s %8s %8s | %9s %9s %9s | %6s %6s\n', ...
    'mode','case','posRMS','posMax','hovNorm','est_p','est_v','est_d','tilt','eff');
for useEstimate = 0:1
    feedback = feedbackNames{useEstimate+1};
    prefix = casePrefixes{useEstimate+1};
    flags = struct('use_est',useEstimate,'use_Qapprox',0,'ff_wind',0);
    for row = 1:size(conditions,1)
        name = conditions{row,1};
        caseId = [prefix '_' name];
        if quick && ~ismember(caseId,quickCases)
            continue
        end
        opts = struct('wind_mode',conditions{row,2},'fix_loss',conditions{row,3}, ...
            'm_scale',conditions{row,4},'J_scale',conditions{row,5}, ...
            'in_delay',conditions{row,6},'execution_id',options.execution_id, ...
            'case_id',caseId,'seed',options.seed);
        run = run_closed(P,K,flags,filter,opts);
        stats = metrics(run,P,filter);
        results(end+1) = struct('mode',feedback,'scen',name,'case_id',caseId, ...
            'seed',opts.seed,'M',stats,'o',run);
        fprintf('%-9s %-13s | %8.3f %8.3f %8.3f | %9.4f %9.4f %9.4f | %6.1f %6.2f\n', ...
            feedback,name,stats.pos_rms,stats.pos_max,stats.hov_off_norm, ...
            stats.est_rms(1),stats.est_rms(2),stats.est_rms(3), ...
            stats.tilt_max*180/pi,stats.effort_rms);
    end
end

if ~quick
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    T = results;
    o = options;
    save(fullfile(root,'results',options.execution_id,'scenarios.mat'),'T','o');
end
end
