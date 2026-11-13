function mdl = build_plant(P)
%BUILD_PLANT Minimal open-loop Simulink test model for the quadrotor plant.
% cmd = [T; n] and vw are constants; all plant outputs logged to workspace.
root = fileparts(fileparts(mfilename('fullpath')));
mdl  = 'plant_test';
slx  = fullfile(root, 'simulink', [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(slx, 'file'), delete(slx); end
new_system(mdl);

add_block('simulink/Sources/Constant', [mdl '/cmd'], 'Value', '[P.Ttrim; 0; 0; 0]');
add_block('simulink/Sources/Constant', [mdl '/vw'],  'Value', '[0; 0; 0]');
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', [mdl '/plant'], ...
    'FunctionName', 'quad_sfun', 'Parameters', 'P');
outs = {'p','v','eta','omega','R'};
for k = 1:5
    add_block('simulink/Sinks/To Workspace', [mdl '/' outs{k}], ...
        'VariableName', ['plant_' outs{k}]);
    add_line(mdl, ['plant/' num2str(k)], [outs{k} '/1'], 'autorouting', 'on');
end
add_line(mdl, 'cmd/1', 'plant/1', 'autorouting', 'on');
add_line(mdl, 'vw/1',  'plant/2', 'autorouting', 'on');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(P.sim.dt), ...
    'StopTime', num2str(5), 'SaveOutput', 'off', 'SaveTime', 'off', ...
    'SaveState', 'off', 'SignalLogging', 'off');
save_system(mdl, slx);
end
