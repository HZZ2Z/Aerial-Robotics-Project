function mdl = build_closed(P)
%BUILD_CLOSED Build simulink/closed_loop.slx: reference, wind, sensors,
% steady-state Kalman filter, LQR controller (exact Q(phi,theta)), plant.
% Runtime knobs are base-workspace variables: P, Pp, K, F, Lg, Lb, Pg, Pb,
% wind_gain, gust_power, fix_loss_t0, fix_loss_t1, in_delay.
% Layout: explicit positions, left-to-right signal flow, functional zones
% (main chain / wind / sensors / fix / filter / selector / log rail).
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'closed_loop';
slx  = fullfile(root, 'simulink', [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(slx, 'file'), delete(slx); end
new_system(mdl);
A = @(b) [mdl '/' b];

% S-function blocks resolve dialog params at add time: seed base workspace
if ~evalin('base', 'exist(''P'',''var'')'),  assignin('base', 'P', P); end
if ~evalin('base', 'exist(''Pp'',''var'')'), assignin('base', 'Pp', P); end
knobs = {'wind_gain', 0, 'gust_power', 4, 'fix_loss_t0', 1e6, ...
         'fix_loss_t1', 1e6+5, 'in_delay', 0, 'fix_drop', 1};
for k = 1:2:numel(knobs)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', knobs{k}))
        assignin('base', knobs{k}, knobs{k+1});
    end
end
if ~evalin('base', 'exist(''K'',''var'')'),    assignin('base', 'K', zeros(4,15)); end
if ~evalin('base', 'exist(''F'',''var'')'),    assignin('base', 'F', struct()); end
if ~evalin('base', 'exist(''Lg'',''var'')'),   assignin('base', 'Lg', zeros(12,3)); end
if ~evalin('base', 'exist(''Lb'',''var'')'),   assignin('base', 'Lb', zeros(12,3)); end
if ~evalin('base', 'exist(''Pg'',''var'')'),   assignin('base', 'Pg', zeros(9)); end
if ~evalin('base', 'exist(''Pb'',''var'')'),   assignin('base', 'Pb', zeros(9)); end

% ---- blocks ----
% zone: main control chain (left -> right)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('ref'), ...
    'FunctionName', 'ref_sfun', 'Parameters', 'P', 'Position', [40 300 130 350]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('ctrl'), ...
    'FunctionName', 'ctrl_sfun', 'Parameters', 'P,K,F', 'Position', [300 270 400 390]);
add_block('simulink/Continuous/Transport Delay', A('udelay'), 'DelayTime', 'in_delay', ...
    'Position', [480 305 530 345]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('plant'), ...
    'FunctionName', 'quad_sfun', 'Parameters', 'Pp', 'Position', [640 260 740 400]);

% zone: wind field (below plant)
add_block('simulink/Sources/Step', A('wstep'), 'Time', '15', 'Before', '[0;0;0]', ...
    'After', '[wind_gain*2;0;0]', 'Position', [300 640 340 680]);
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['wn' num2str(k)]), ...
        'Cov', 'gust_power', 'Ts', '0.01', 'seed', num2str(20+k), ...
        'Position', [300 720+70*(k-1) 340 760+70*(k-1)]);
    add_block('simulink/Continuous/Transfer Fcn', A(['tf' num2str(k)]), ...
        'Numerator', sprintf('sqrt(2/%g)*%g', P.wind.tau, P.wind.sigma), ...
        'Denominator', sprintf('[1 %g]', P.wind.tau), ...
        'Position', [400 710+75*(k-1) 460 760+75*(k-1)]);
end
add_block('simulink/Signal Routing/Mux', A('wmux'), 'Inputs', '3', ...
    'Position', [500 750 515 870]);
add_block('simulink/Math Operations/Sum', A('wsum'), 'Inputs', '++', ...
    'Position', [560 790 590 820]);
add_block('simulink/Sinks/To Workspace', A('vw_log'), 'VariableName', 'wind_vw', ...
    'Position', [680 780 780 810]);

% zone: sensors (right of plant)
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('ahrs'), ...
    'FunctionName', 'sensor_sfun', 'Parameters', 'P,1', 'Position', [820 480 900 530]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('gyro'), ...
    'FunctionName', 'sensor_sfun', 'Parameters', 'P,2', 'Position', [820 570 900 620]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('gnss'), ...
    'FunctionName', 'sensor_sfun', 'Parameters', 'P,3', 'Position', [820 660 900 710]);

% zone: GNSS fix status (below sensors)
add_block('simulink/Sources/Constant', A('fixone'), 'Value', '1', ...
    'Position', [600 780 640 810]);
add_block('simulink/Sources/Step', A('fixdn'), 'Time', 'fix_loss_t0', 'Before', '0', ...
    'After', '-fix_drop', 'Position', [600 830 640 860]);
add_block('simulink/Sources/Step', A('fixup'), 'Time', 'fix_loss_t1', 'Before', '0', ...
    'After', '1', 'Position', [600 880 640 910]);
add_block('simulink/Math Operations/Sum', A('fixsum'), 'Inputs', '+++', ...
    'Position', [700 815 730 855]);
add_block('simulink/Sources/Constant', A('fixconst'), 'Value', '1', ...
    'Position', [600 930 640 960]);   % parked: not wired (kept for parity with original build)
add_block('simulink/Sinks/To Workspace', A('fix_log'), 'VariableName', 'gnss_fix', ...
    'Position', [820 815 910 845]);

% zone: kalman filter + its input path (above sensors)
add_block('simulink/Signal Routing/Selector', A('sel_T'), 'Indices', '[1]', ...
    'InputPortWidth', '4', 'Position', [790 160 820 190]);
add_block('simulink/Discrete/Memory', A('memT'), 'InitialCondition', '0', ...
    'Position', [850 170 890 220]);
add_block('simulink/Signal Routing/Mux', A('kfin'), 'Inputs', '2', ...
    'Position', [930 180 945 270]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('kf'), ...
    'FunctionName', 'kf_sfun', 'Parameters', 'P,Lg,Lb,Pg,Pb', 'Position', [990 300 1110 430]);

% zone: kf output selectors + estimated-state mux
add_block('simulink/Signal Routing/Selector', A('sel_p'), 'Indices', '[1 2 3]', ...
    'InputPortWidth', '9', 'Position', [1160 300 1190 340]);
add_block('simulink/Signal Routing/Selector', A('sel_v'), 'Indices', '[4 5 6]', ...
    'InputPortWidth', '9', 'Position', [1160 360 1190 400]);
add_block('simulink/Signal Routing/Selector', A('sel_d'), 'Indices', '[7 8 9]', ...
    'InputPortWidth', '9', 'Position', [1160 420 1190 460]);
add_block('simulink/Signal Routing/Mux', A('xest'), 'Inputs', '4', ...
    'Position', [1230 330 1245 440]);
add_block('simulink/Signal Routing/Mux', A('xtrue'), 'Inputs', '4', ...
    'Position', [760 270 775 400]);

% zone: log rail (top edge, left -> right in dependency order)
logs = {'p','v','eta','omega','R','Tn'};
for k = 1:6
    add_block('simulink/Sinks/To Workspace', A(['lp_' logs{k}]), ...
        'VariableName', ['plant_' logs{k}], 'Position', [490+120*(k-1) 40 590+120*(k-1) 70]);
end
log2 = {'ref','p_meas','eta_meas','omega_meas','p_true','p_est','kf_out','x15','ucmd'};
for k = 1:numel(log2)
    add_block('simulink/Sinks/To Workspace', A(['lg_' log2{k}]), ...
        'VariableName', ['log_' log2{k}], 'Position', [40+130*(k-1) 130 140+130*(k-1) 160]);
end

% ---- wiring (unchanged) ----
add_line(mdl, 'ref/1',    'ctrl/3', 'autorouting', 'on');
add_line(mdl, 'ctrl/1',   'udelay/1', 'autorouting', 'on');
add_line(mdl, 'udelay/1', 'plant/1',  'autorouting', 'on');
add_line(mdl, 'wmux/1',   'plant/2',  'autorouting', 'on');
add_line(mdl, 'plant/1',  'xtrue/1', 'autorouting', 'on');
add_line(mdl, 'plant/2',  'xtrue/2', 'autorouting', 'on');
add_line(mdl, 'plant/3',  'xtrue/3', 'autorouting', 'on');
add_line(mdl, 'plant/4',  'xtrue/4', 'autorouting', 'on');
for k = 1:6
    if k == 6
        add_line(mdl, 'ctrl/3', 'lp_Tn/1', 'autorouting', 'on');
    else
        add_line(mdl, sprintf('plant/%d', k), sprintf('lp_%s/1', logs{k}), 'autorouting', 'on');
    end
end
% wind: mean (x only) + filtered gusts
for k = 1:3
    add_line(mdl, sprintf('wn%d/1', k), sprintf('tf%d/1', k), 'autorouting', 'on');
    add_line(mdl, sprintf('tf%d/1', k), sprintf('wmux/%d', k), 'autorouting', 'on');
end
add_line(mdl, 'wstep/1', 'wsum/1', 'autorouting', 'on');
add_line(mdl, 'wmux/1',  'wsum/2', 'autorouting', 'on');
add_line(mdl, 'wsum/1',  'vw_log/1', 'autorouting', 'on');
% fix status
add_line(mdl, 'fixone/1', 'fixsum/1', 'autorouting', 'on');
add_line(mdl, 'fixdn/1',  'fixsum/2', 'autorouting', 'on');
add_line(mdl, 'fixup/1',  'fixsum/3', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'gnss/2', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'kf/3', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'fix_log/1', 'autorouting', 'on');
% sensors
add_line(mdl, 'plant/3', 'ahrs/1', 'autorouting', 'on');
add_line(mdl, 'plant/4', 'gyro/1', 'autorouting', 'on');
add_line(mdl, 'plant/1', 'gnss/1', 'autorouting', 'on');
add_line(mdl, 'plant/1', 'lg_p_true/1', 'autorouting', 'on');
add_line(mdl, 'ahrs/1',  'lg_eta_meas/1', 'autorouting', 'on');
add_line(mdl, 'gyro/1',  'lg_omega_meas/1', 'autorouting', 'on');
add_line(mdl, 'gnss/1',  'lg_p_meas/1', 'autorouting', 'on');
% kalman filter: in1 = [Tcmd(mem); eta_meas]
add_line(mdl, 'ctrl/1', 'sel_T/1', 'autorouting', 'on');
add_line(mdl, 'sel_T/1', 'memT/1', 'autorouting', 'on');
add_line(mdl, 'memT/1', 'kfin/1', 'autorouting', 'on');
add_line(mdl, 'ahrs/1', 'kfin/2', 'autorouting', 'on');
add_line(mdl, 'kfin/1', 'kf/1', 'autorouting', 'on');
add_line(mdl, 'gnss/1', 'kf/2', 'autorouting', 'on');
add_line(mdl, 'kf/1', 'sel_p/1', 'autorouting', 'on');
add_line(mdl, 'kf/1', 'sel_v/1', 'autorouting', 'on');
add_line(mdl, 'kf/1', 'sel_d/1', 'autorouting', 'on');
add_line(mdl, 'kf/1', 'lg_kf_out/1', 'autorouting', 'on');
add_line(mdl, 'sel_p/1', 'lg_p_est/1', 'autorouting', 'on');
% controller inputs
add_line(mdl, 'xtrue/1', 'ctrl/1', 'autorouting', 'on');
add_line(mdl, 'xest/1',  'ctrl/2', 'autorouting', 'on');
add_line(mdl, 'sel_p/1', 'xest/1', 'autorouting', 'on');
add_line(mdl, 'sel_v/1', 'xest/2', 'autorouting', 'on');
add_line(mdl, 'ahrs/1',  'xest/3', 'autorouting', 'on');
add_line(mdl, 'gyro/1',  'xest/4', 'autorouting', 'on');
add_line(mdl, 'sel_d/1', 'ctrl/4', 'autorouting', 'on');
% logs
add_line(mdl, 'ctrl/1', 'lg_ucmd/1', 'autorouting', 'on');
add_line(mdl, 'ctrl/2', 'lg_x15/1', 'autorouting', 'on');
add_line(mdl, 'ref/1',  'lg_ref/1', 'autorouting', 'on');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(P.sim.dt), ...
    'StopTime', num2str(P.sim.tfinal), 'SaveOutput', 'off', 'SaveTime', 'off', ...
    'SaveState', 'off', 'SignalLogging', 'off');
save_system(mdl, slx);
disp(['built ' slx]);
end
