function mdl = build_closed(P)
%BUILD_CLOSED Build simulink/closed_loop.slx: reference, wind, sensors,
% steady-state Kalman filter, LQR controller (exact Q(phi,theta)), plant.
% Runtime knobs are base-workspace variables: P, Pp, K, F, kfmdl,
% wind_gain, gust_gain, fix_loss_t0, fix_loss_t1, in_delay, fix_drop.
% Layout: explicit positions, left-to-right signal flow, functional zones
% (main chain / wind / sensors / fix / filter / selector / log rail).
% Wind: mean step + OU-filtered gusts are summed into ONE vw signal that is
% wired to BOTH the plant and the wind log (plant/log same source).
% Sensors: continuous true signal + band-limited white noise of spectral
% density r, noise held on a fixed grid (P.noise.dt_*); no sample-and-hold
% of the true signal. GNSS noise switches RTK/degraded/none with fix status.
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
knobs = {'wind_gain', 1, 'gust_gain', 1, 'fix_loss_t0', 1e6, ...
         'fix_loss_t1', 1e6+5, 'in_delay', 0, 'fix_drop', 1};
for k = 1:2:numel(knobs)
    if ~evalin('base', sprintf('exist(''%s'',''var'')', knobs{k}))
        assignin('base', knobs{k}, knobs{k+1});
    end
end
if ~evalin('base', 'exist(''K'',''var'')'),     assignin('base', 'K', zeros(4,15)); end
if ~evalin('base', 'exist(''F'',''var'')'),     assignin('base', 'F', struct()); end
if ~evalin('base', 'exist(''kfmdl'',''var'')'), assignin('base', 'kfmdl', struct('nx', 9)); end

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

% zone: wind field (below plant). Mean step (x) + filtered gusts -> wsum = vw
add_block('simulink/Sources/Step', A('wstep'), 'Time', 'P.wind.t0', ...
    'Before', '[0;0;0]', 'After', '[wind_gain*P.wind.vmean;0;0]', ...
    'Position', [300 640 340 680]);
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['wn' num2str(k)]), ...
        'Cov', 'gust_gain', 'Ts', 'P.noise.dt_wind', ...
        'Seed', sprintf('P.noise.seed_base + P.noise.seed_wind + %d', k-1), ...
        'Position', [300 720+70*(k-1) 340 760+70*(k-1)]);
    add_block('simulink/Continuous/Transfer Fcn', A(['tf' num2str(k)]), ...
        'Numerator', sprintf('sqrt(2*%g^2/%g)', P.wind.sigma, P.wind.tau), ...
        'Denominator', sprintf('[1 %g]', 1/P.wind.tau), ...
        'Position', [400 710+75*(k-1) 460 760+75*(k-1)]);
end
add_block('simulink/Signal Routing/Mux', A('wmux'), 'Inputs', '3', ...
    'Position', [500 750 515 870]);
add_block('simulink/Math Operations/Sum', A('wsum'), 'Inputs', '++', ...
    'Position', [560 790 590 820]);
add_block('simulink/Sinks/To Workspace', A('vw_log'), 'VariableName', 'wind_vw', ...
    'Position', [680 780 780 810]);

% zone: sensors (right of plant). True signal (continuous) + BLWN(r, Ts=dt)
% AHRS: eta_meas = eta + noise
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['an' num2str(k)]), ...
        'Cov', 'P.sens.ahrs.r', 'Ts', 'P.noise.dt_sensor', ...
        'Seed', sprintf('P.noise.seed_base + P.noise.seed_ahrs + %d', k-1), ...
        'Position', [800 450+40*(k-1) 840 490+40*(k-1)]);
end
add_block('simulink/Signal Routing/Mux', A('anmux'), 'Inputs', '3', ...
    'Position', [855 460 870 580]);
add_block('simulink/Math Operations/Sum', A('asum'), 'Inputs', '++', ...
    'Position', [900 480 930 510]);
% gyro: omega_meas = omega + noise
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['gn' num2str(k)]), ...
        'Cov', 'P.sens.gyro.r', 'Ts', 'P.noise.dt_sensor', ...
        'Seed', sprintf('P.noise.seed_base + P.noise.seed_gyro + %d', k-1), ...
        'Position', [800 620+40*(k-1) 840 660+40*(k-1)]);
end
add_block('simulink/Signal Routing/Mux', A('gnmux'), 'Inputs', '3', ...
    'Position', [855 630 870 750]);
add_block('simulink/Math Operations/Sum', A('gsum'), 'Inputs', '++', ...
    'Position', [900 650 930 680]);
% GNSS: p_meas = p + bg*noise_good + bd*noise_bad (zero noise when no_fix)
for k = 1:3
    add_block('simulink/Sources/Band-Limited White Noise', A(['pg' num2str(k)]), ...
        'Cov', sprintf('P.sens.gnss.rvec(%d)', k), 'Ts', 'P.noise.dt_gnss', ...
        'Seed', sprintf('P.noise.seed_base + P.noise.seed_gnss + %d', k-1), ...
        'Position', [900 740+40*(k-1) 940 780+40*(k-1)]);
    add_block('simulink/Sources/Band-Limited White Noise', A(['pb' num2str(k)]), ...
        'Cov', sprintf('P.sens.gnss.rbad(%d)', k), 'Ts', 'P.noise.dt_gnss', ...
        'Seed', sprintf('P.noise.seed_base + P.noise.seed_gnss + 10 + %d', k-1), ...
        'Position', [900 900+40*(k-1) 940 940+40*(k-1)]);
end
add_block('simulink/Signal Routing/Mux', A('pgmux'), 'Inputs', '3', ...
    'Position', [955 750 970 870]);
add_block('simulink/Signal Routing/Mux', A('pbmux'), 'Inputs', '3', ...
    'Position', [955 910 970 1030]);
% fix-status masks: bg = (fix > 0.5), bd = (fix > -0.5)*(fix < 0.5)
add_block('simulink/Logic and Bit Operations/Relational Operator', A('relg'), ...
    'Operator', '>', 'Position', [820 1050 850 1080]);
add_block('simulink/Sources/Constant', A('half'), 'Value', '0.5', ...
    'Position', [770 1065 800 1085]);
add_block('simulink/Logic and Bit Operations/Relational Operator', A('relb1'), ...
    'Operator', '>', 'Position', [820 1110 850 1140]);
add_block('simulink/Sources/Constant', A('mhalf'), 'Value', '-0.5', ...
    'Position', [770 1125 800 1145]);
add_block('simulink/Logic and Bit Operations/Relational Operator', A('relb2'), ...
    'Operator', '<', 'Position', [820 1170 850 1200]);
add_block('simulink/Math Operations/Product', A('pmaskb'), 'Inputs', '2', ...
    'Position', [880 1130 910 1190]);
add_block('simulink/Math Operations/Product', A('pgate'), 'Inputs', '2', ...
    'Position', [1010 760 1040 830]);
add_block('simulink/Math Operations/Product', A('bgate'), 'Inputs', '2', ...
    'Position', [1010 920 1040 990]);
add_block('simulink/Math Operations/Sum', A('nsum'), 'Inputs', '++', ...
    'Position', [1070 960 1100 1000]);
add_block('simulink/Math Operations/Sum', A('gnsssum'), 'Inputs', '++', ...
    'Position', [1130 660 1160 700]);

% zone: GNSS fix status (below sensors). fix = 1 - fix_drop*H(t-t0) + fix_drop*H(t-t1)
% good=1 for t<[t0,t1) outside, degraded=0 (fix_drop=1) or no_fix=-1 (fix_drop=2) inside
add_block('simulink/Sources/Constant', A('fixone'), 'Value', '1', ...
    'Position', [600 780 640 810]);
add_block('simulink/Sources/Step', A('fixdn'), 'Time', 'fix_loss_t0', 'Before', '0', ...
    'After', '-fix_drop', 'Position', [600 830 640 860]);
add_block('simulink/Sources/Step', A('fixup'), 'Time', 'fix_loss_t1', 'Before', '0', ...
    'After', 'fix_drop', 'Position', [600 880 640 910]);
add_block('simulink/Math Operations/Sum', A('fixsum'), 'Inputs', '+++', ...
    'Position', [700 815 730 855]);
add_block('simulink/Sinks/To Workspace', A('fix_log'), 'VariableName', 'gnss_fix', ...
    'Position', [820 815 910 845]);

add_block('simulink/Sinks/Terminator', A('termT'), 'Position', [430 250 450 270]);
% zone: kalman filter + its input path (above sensors)
add_block('simulink/Signal Routing/Selector', A('sel_T'), 'Indices', '[1]', ...
    'InputPortWidth', '4', 'Position', [790 160 820 190]);
add_block('simulink/Discrete/Memory', A('memT'), 'InitialCondition', '0', ...
    'Position', [850 170 890 220]);
add_block('simulink/Signal Routing/Mux', A('kfin'), 'Inputs', '2', ...
    'Position', [930 180 945 270]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', A('kf'), ...
    'FunctionName', 'kf_sfun', 'Parameters', 'kfmdl', 'Position', [1240 300 1360 430]);

% zone: kf output selectors + estimated-state mux
add_block('simulink/Signal Routing/Selector', A('sel_p'), 'Indices', '[1 2 3]', ...
    'InputPortWidth', '9', 'Position', [1400 300 1430 340]);
add_block('simulink/Signal Routing/Selector', A('sel_v'), 'Indices', '[4 5 6]', ...
    'InputPortWidth', '9', 'Position', [1400 360 1430 400]);
add_block('simulink/Signal Routing/Selector', A('sel_d'), 'Indices', '[7 8 9]', ...
    'InputPortWidth', '9', 'Position', [1400 420 1430 460]);
add_block('simulink/Signal Routing/Mux', A('xest'), 'Inputs', '4', ...
    'Position', [1470 330 1485 440]);
add_block('simulink/Signal Routing/Mux', A('xtrue'), 'Inputs', '4', ...
    'Position', [760 270 775 400]);

% zone: log rail (top edge, left -> right in dependency order)
logs = {'p','v','eta','omega','R'};
for k = 1:numel(logs)
    add_block('simulink/Sinks/To Workspace', A(['lp_' logs{k}]), ...
        'VariableName', ['plant_' logs{k}], 'Position', [490+120*(k-1) 40 590+120*(k-1) 70]);
end
log2 = {'ref','p_meas','eta_meas','omega_meas','p_true','p_est','kf_out','x15','ucmd', ...
        'u_delayed','T_model'};
for k = 1:numel(log2)
    add_block('simulink/Sinks/To Workspace', A(['lg_' log2{k}]), ...
        'VariableName', ['log_' log2{k}], 'Position', [40+130*(k-1) 130 140+130*(k-1) 160]);
end

% ---- wiring ----
add_line(mdl, 'ref/1',    'ctrl/3', 'autorouting', 'on');
add_line(mdl, 'ctrl/1',   'udelay/1', 'autorouting', 'on');
add_line(mdl, 'udelay/1', 'plant/1',  'autorouting', 'on');
add_line(mdl, 'wsum/1',   'plant/2',  'autorouting', 'on');   % applied wind = mean + gust
add_line(mdl, 'plant/1',  'xtrue/1', 'autorouting', 'on');
add_line(mdl, 'plant/2',  'xtrue/2', 'autorouting', 'on');
add_line(mdl, 'plant/3',  'xtrue/3', 'autorouting', 'on');
add_line(mdl, 'plant/4',  'xtrue/4', 'autorouting', 'on');
for k = 1:numel(logs)
    add_line(mdl, sprintf('plant/%d', k), sprintf('lp_%s/1', logs{k}), 'autorouting', 'on');
end
% wind: mean (x only) + filtered gusts, one vw to plant and log
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
add_line(mdl, 'fixsum/1', 'fix_log/1', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'relg/1', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'relb1/1', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'relb2/1', 'autorouting', 'on');
add_line(mdl, 'half/1',   'relg/2', 'autorouting', 'on');
add_line(mdl, 'mhalf/1',  'relb1/2', 'autorouting', 'on');
add_line(mdl, 'half/1',   'relb2/2', 'autorouting', 'on');
add_line(mdl, 'relb1/1',  'pmaskb/1', 'autorouting', 'on');
add_line(mdl, 'relb2/1',  'pmaskb/2', 'autorouting', 'on');
% sensors: continuous truth + gated band-limited noise
add_line(mdl, 'plant/3', 'asum/1', 'autorouting', 'on');
add_line(mdl, 'anmux/1', 'asum/2', 'autorouting', 'on');
for k = 1:3
    add_line(mdl, sprintf('an%d/1', k), sprintf('anmux/%d', k), 'autorouting', 'on');
end
add_line(mdl, 'plant/4', 'gsum/1', 'autorouting', 'on');
add_line(mdl, 'gnmux/1', 'gsum/2', 'autorouting', 'on');
for k = 1:3
    add_line(mdl, sprintf('gn%d/1', k), sprintf('gnmux/%d', k), 'autorouting', 'on');
end
% GNSS noise: bg*good + bd*bad (element-wise, scalar mask broadcast)
add_line(mdl, 'relg/1',  'pgate/1', 'autorouting', 'on');
add_line(mdl, 'pgmux/1', 'pgate/2', 'autorouting', 'on');
add_line(mdl, 'pmaskb/1', 'bgate/1', 'autorouting', 'on');
add_line(mdl, 'pbmux/1', 'bgate/2', 'autorouting', 'on');
for k = 1:3
    add_line(mdl, sprintf('pg%d/1', k), sprintf('pgmux/%d', k), 'autorouting', 'on');
    add_line(mdl, sprintf('pb%d/1', k), sprintf('pbmux/%d', k), 'autorouting', 'on');
end
add_line(mdl, 'pgate/1', 'nsum/1', 'autorouting', 'on');
add_line(mdl, 'bgate/1', 'nsum/2', 'autorouting', 'on');
add_line(mdl, 'plant/1',  'gnsssum/1', 'autorouting', 'on');
add_line(mdl, 'nsum/1',   'gnsssum/2', 'autorouting', 'on');
% kalman filter: in1 = [Tcmd(mem); eta_meas], in2 = p_meas, in3 = fix
add_line(mdl, 'ctrl/1', 'sel_T/1', 'autorouting', 'on');
add_line(mdl, 'sel_T/1', 'memT/1', 'autorouting', 'on');
add_line(mdl, 'memT/1', 'kfin/1', 'autorouting', 'on');
add_line(mdl, 'asum/1', 'kfin/2', 'autorouting', 'on');
add_line(mdl, 'kfin/1', 'kf/1', 'autorouting', 'on');
add_line(mdl, 'gnsssum/1', 'kf/2', 'autorouting', 'on');
add_line(mdl, 'fixsum/1', 'kf/3', 'autorouting', 'on');
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
add_line(mdl, 'asum/1',  'xest/3', 'autorouting', 'on');
add_line(mdl, 'gsum/1',  'xest/4', 'autorouting', 'on');
add_line(mdl, 'sel_d/1', 'ctrl/4', 'autorouting', 'on');
% logs
add_line(mdl, 'ctrl/1',    'lg_ucmd/1', 'autorouting', 'on');
add_line(mdl, 'ctrl/2',    'lg_x15/1', 'autorouting', 'on');
add_line(mdl, 'ctrl/3',    'termT/1', 'autorouting', 'on');
add_line(mdl, 'ref/1',     'lg_ref/1', 'autorouting', 'on');
add_line(mdl, 'udelay/1',  'lg_u_delayed/1', 'autorouting', 'on');
add_line(mdl, 'memT/1',    'lg_T_model/1', 'autorouting', 'on');
add_line(mdl, 'gnsssum/1', 'lg_p_meas/1', 'autorouting', 'on');
add_line(mdl, 'asum/1',    'lg_eta_meas/1', 'autorouting', 'on');
add_line(mdl, 'gsum/1',    'lg_omega_meas/1', 'autorouting', 'on');
add_line(mdl, 'plant/1',   'lg_p_true/1', 'autorouting', 'on');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(P.sim.dt), ...
    'StopTime', num2str(P.sim.tfinal), 'SaveOutput', 'off', 'SaveTime', 'off', ...
    'SaveState', 'off', 'SignalLogging', 'off');
save_system(mdl, slx);
disp(['built ' slx]);
end
