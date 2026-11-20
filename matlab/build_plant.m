function mdl = build_plant(P)
%BUILD_PLANT Minimal open-loop Simulink test model for the quadrotor plant.
% cmd = [T; n] and vw are constants; all plant outputs logged to workspace.
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'plant_test';
slx  = fullfile(root, 'simulink', [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(slx, 'file'), delete(slx); end
new_system(mdl);
if ~evalin('base', 'exist(''P'',''var'')'), assignin('base', 'P', P); end

add_block('simulink/Sources/Constant', [mdl '/cmd'], 'Value', '[P.Ttrim; 0; 0; 0]', ...
    'Position', [40 100 90 130]);
add_block('simulink/Sources/Constant', [mdl '/vw'],  'Value', '[0; 0; 0]', ...
    'Position', [40 180 90 210]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/plant'], ...
    'FunctionName', 'quad_sfun', 'Parameters', 'P', 'Position', [160 90 280 220]);
outs = {'p','v','eta','omega','R'};
for k = 1:5
    add_block('simulink/Sinks/To Workspace', [mdl '/' outs{k}], ...
        'VariableName', ['plant_' outs{k}], ...
        'Position', [340 30+50*(k-1) 430 60+50*(k-1)]);
    add_line(mdl, ['plant/' num2str(k)], [outs{k} '/1'], 'autorouting', 'on');
end
add_line(mdl, 'cmd/1', 'plant/1', 'autorouting', 'on');
add_line(mdl, 'vw/1',  'plant/2', 'autorouting', 'on');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(P.sim.dt), ...
    'StopTime', num2str(5), 'SaveOutput', 'off', 'SaveTime', 'off', ...
    'SaveState', 'off', 'SignalLogging', 'off');
save_system(mdl, slx);
end
