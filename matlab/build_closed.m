function mdl = build_closed(P)
%BUILD_CLOSED Build simulink/closed_loop.slx: reference, wind, sensors,
% steady-state Kalman filter, LQR controller (exact Q(phi,theta)), plant.
% Runtime knobs are base-workspace variables: P, Pp, K, F, Lg, Lb, Pg, Pb,
% wind_gain, gust_power, fix_loss_t0, fix_loss_t1, in_delay.
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'closed_loop';
slx  = fullfile(root, 'simulink', [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(slx, 'file'), delete(slx); end
new_system(mdl);
A = @(b) [mdl '/' b];

% S-function blocks resolve dialog params at add time: seed base workspace
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
if ~evalin('base', 'exist(''Lg'',''var'')'),   assignin('base', 'Lg', zeros(9,3)); end
if ~evalin('base', 'exist(''Lb'',''var'')'),   assignin('base', 'Lb', zeros(9,3)); end
if ~evalin('base', 'exist(''Pg'',''var'')'),   assignin('base', 'Pg', zeros(9)); end
if ~evalin('base', 'exist(''Pb'',''var'')'),   assignin('base', 'Pb', zeros(9)); end

% ---- blocks ----
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('ref'), ...
    'FunctionName', 'ref_sfun', 'Parameters', 'P');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('plant'), ...
    'FunctionName', 'quad_sfun', 'Parameters', 'Pp');   % plant may have model uncertainty
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('ctrl'), ...
    'FunctionName', 'ctrl_sfun', 'Parameters', 'P,K,F');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('kf'), ...
    'FunctionName', 'kf_sfun', 'Parameters', 'P,Lg,Lb,Pg,Pb');
mds = {'ahrs','gyro','gnss'};
for k = 1:3
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A(mds{k}), ...
        'FunctionName', 'sensor_sfun', 'Parameters', sprintf('P,%d', k));
end

% wind: x-mean step + 3 filtered white-noise gusts
add_block('simulink/Sources/Step', A('wstep'), 'Time', '15', 'Before', '[0;0;0]', 'After', '[wind_gain*2;0;0]');
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['wn' num2str(k)]), ...
        'Cov', 'gust_power', 'Ts', '0.01', 'seed', num2str(20+k));
    add_block('simulink/Continuous/Transfer Fcn', A(['tf' num2str(k)]), ...
        'Numerator', sprintf('sqrt(2/%g)*%g', P.wind.tau, P.wind.sigma), ...
        'Denominator', sprintf('[1 %g]', P.wind.tau));
end
add_block('simulink/Signal Routing/Mux', A('wmux'), 'Inputs', '3');
add_block('simulink/Math Operations/Sum', A('wsum'), 'Inputs', '++');
add_block('simulink/Sinks/To Workspace', A('vw_log'), 'VariableName', 'wind_vw');

% GNSS fix status: 1 normally, 0 between the loss times
add_block('simulink/Sources/Step', A('fixdn'), 'Time', 'fix_loss_t0', 'Before', '0', 'After', '-fix_drop');
add_block('simulink/Sources/Step', A('fixup'), 'Time', 'fix_loss_t1', 'Before', '0', 'After', '1');
add_block('simulink/Math Operations/Sum', A('fixsum'), 'Inputs', '+++');
add_block('simulink/Sources/Constant', A('fixone'), 'Value', '1');
add_block('simulink/Sinks/To Workspace', A('fix_log'), 'VariableName', 'gnss_fix');

% input delay (model-uncertainty scenario)
add_block('simulink/Continuous/Transport Delay', A('udelay'), 'DelayTime', 'in_delay');

% kf output selectors
add_block('simulink/Signal Routing/Selector', A('sel_p'), 'Indices', '[1 2 3]', ...
    'InputPortWidth', '9');
add_block('simulink/Signal Routing/Selector', A('sel_v'), 'Indices', '[4 5 6]', ...
    'InputPortWidth', '9');
add_block('simulink/Signal Routing/Selector', A('sel_d'), 'Indices', '[7 8 9]', ...
    'InputPortWidth', '9');

% state muxes / kf input mux / memory
add_block('simulink/Signal Routing/Mux', A('xtrue'), 'Inputs', '4');
add_block('simulink/Signal Routing/Mux', A('xest'),  'Inputs', '4');
add_block('simulink/Signal Routing/Mux', A('kfin'),  'Inputs', '2');
add_block('simulink/Sources/Constant', A('fixconst'), 'Value', '1');
add_block('simulink/Signal Routing/Selector', A('sel_T'), 'Indices', '[1]', 'InputPortWidth', '4');
add_block('simulink/Discrete/Memory', A('memT'), 'InitialCondition', '0');

% logging
logs = {'p','v','eta','omega','R','Tn'};
for k = 1:6
    add_block('simulink/Sinks/To Workspace', A(['lp_' logs{k}]), ...
        'VariableName', ['plant_' logs{k}]);
end
log2 = {'ref','p_meas','kf_out','x15','ucmd','eta_meas','omega_meas','p_est','p_true'};
for k = 1:numel(log2)
    add_block('simulink/Sinks/To Workspace', A(['lg_' log2{k}]), ...
        'VariableName', ['log_' log2{k}]);
end

% ---- wiring ----
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
