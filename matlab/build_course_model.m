function mdl = build_course_model(P)
%BUILD_COURSE_MODEL 基于课程提供的 quad_arch.slx 模板，通过脚本自动装配完整的 Simulink 闭环模型。
%   包含：注入各 MATLAB Function 模块代码、设置传感器/阵风噪声种子与功率谱密度、
%   插入执行器输入延迟、添加 Bonus 风扰平滑前馈通道，以及配置信号记录与 ode4 求解器。
root = fileparts(fileparts(mfilename('fullpath')));
kit = fullfile(root, 'simulink', 'student_kit');
addpath(kit);
mdl = 'quad_arch';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(fullfile(kit, [mdl '.slx']));
set_param(mdl, 'InitFcn', '');

% 1) 向控制器、卡尔曼滤波器、GNSS 接收机与刚体动力学模块注入 MATLAB 代码

chart(mdl, 'Controller/D law', [
    "function [w_dot,z_dot] = d_law(meas,x,pvd_hat,ref,z,P)"
    "[w_dot,z_dot] = course_d_law(meas,x,pvd_hat,ref,z,P);"
    "end"]);
chart(mdl, 'Controller/Thrust and torque laws', [
    "function [u,T,raw] = act_law(meas,x,ulin,ref,P)"
    "[u,raw] = course_actuation(meas,x,ulin,ref,P);"
    "T = u(1);"
    "end"]);
chart(mdl, 'Kalman filter/Filter equation', [
    "function [xh_dot,pvd_hat] = filter_eq(xh,T,meas,y,fix,P)"
    "[xh_dot,pvd_hat] = course_filter(xh,T,meas,y,fix,P);"
    "end"]);
chart(mdl, 'RTK GNSS/Receiver', [
    "function [y,fix] = receiver(x,t,n,P)"
    "if t < P.deg_t0 || t >= P.deg_t1"
    "    fix = 1; y = x(1:3) + n(:);"
    "elseif P.no_fix"
    "    fix = -1; y = zeros(3,1);"
    "else"
    "    fix = 0; y = x(1:3) + n(:).*sqrt(P.r_gnss_deg./P.r_gnss);"
    "end"
    "end"]);
chart(mdl, 'Quadrotor/Rigid-body model', [
    "function xdot = dynamics(x,u,vw,P)"
    "xdot = course_plant_rate(x,u,vw,P);"
    "end"]);

set_param([mdl '/AHRS + gyros/AHRS noise'], 'seed', 'P.seed_ahrs');
set_param([mdl '/AHRS + gyros/Gyro noise'], 'seed', 'P.seed_gyro');
set_param([mdl '/RTK GNSS/GNSS noise'], 'seed', 'P.seed_gnss');
set_param([mdl '/Wind/White noise'], 'seed', 'P.seed_wind', ...
    'Cov', 'P.gust_on*2*P.gust_sd^2/P.gust_tau*[1 1 1]');

add_block('simulink/Continuous/Transport Delay', [mdl '/Actuator delay'], ...
    'DelayTime', 'P.delay', 'InitialOutput', '[P.m*P.g;0;0;0]', ...
    'Position', [555 105 610 135]);
delete_line(mdl, 'Controller/1', 'Quadrotor/1');
add_line(mdl, 'Controller/1', 'Actuator delay/1', 'autorouting', 'on');
add_line(mdl, 'Actuator delay/1', 'Quadrotor/1', 'autorouting', 'on');

kalman = [mdl '/Kalman filter'];
add_block('simulink/Sinks/Out1', [kalman '/State estimate'], 'Port', '2', ...
    'Position', [650 210 680 230]);
add_line(kalman, 'Estimate/1', 'State estimate/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Demux', [mdl '/Split estimate'], ...
    'Outputs', '[6 3]', 'Position', [740 520 755 560]);
add_block('simulink/Sinks/Terminator', [mdl '/Discard position and velocity'], ...
    'Position', [780 515 805 535]);
add_line(mdl, 'Kalman filter/2', 'Split estimate/1', 'autorouting', 'on');
add_line(mdl, 'Split estimate/1', 'Discard position and velocity/1', 'autorouting', 'on');

add_block('simulink/Continuous/State-Space', [mdl '/Smooth disturbance'], ...
    'A', 'kron([-1 0 0;1 -1 0;0 1 -1],eye(3))', ...
    'B', '[eye(3);zeros(6,3)]', ...
    'C', 'kron([0 0 1;0 1 -1;1 -2 1],eye(3))', ...
    'D', 'zeros(9,3)', 'X0', 'zeros(9,1)', 'Position', [810 550 880 585]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/Reference feedforward'], ...
    'Position', [200 210 305 258]);
refChart = chart(mdl, 'Reference feedforward', [
    "function out = reference_ff(ref,d,P)"
    "out = course_feedforward(ref,d,P);"
    "end"]);
parameter = refChart.find('-isa', 'Stateflow.Data', 'Name', 'P');
parameter.Scope = 'Parameter';
delete_line(mdl, 'Reference/1', 'Controller/3');
add_line(mdl, 'Reference/1', 'Reference feedforward/1', 'autorouting', 'on');
add_line(mdl, 'Split estimate/2', 'Smooth disturbance/1', 'autorouting', 'on');
add_line(mdl, 'Smooth disturbance/1', 'Reference feedforward/2', 'autorouting', 'on');
add_line(mdl, 'Reference feedforward/1', 'Controller/3', 'autorouting', 'on');

src = {'Reference/1', 'Quadrotor/1', 'Wind/1', 'RTK GNSS/1', 'RTK GNSS/2', ...
       'AHRS + gyros/1', 'Kalman filter/2', 'Controller/1', 'Actuator delay/1'};
names = {'ref','state','wind','p_meas','fix','meas','kf','command','applied'};
for k = 1:numel(names)
    log_signal(mdl, src{k}, names{k}, 1000, 40+38*k);
end
log_signal([mdl '/Controller'], 'Input integrators/1', 'w', 690, 125);
log_signal([mdl '/Controller'], 'Thrust and torque laws/3', 'raw_command', 690, 160);
set_param(mdl, 'Solver', 'ode4', 'FixedStep', 'P.h', 'StopTime', 'P.t_end', ...
    'SignalLogging', 'off', 'SaveOutput', 'off', 'SaveTime', 'off', ...
    'AlgebraicLoopMsg', 'error', 'ReturnWorkspaceOutputs', 'on');
dest = fullfile(root, 'simulink', [mdl '.slx']);
save_system(mdl, dest);
fprintf('Built course model: %s (ode4, default %.3g s)\n', dest, P.sim.dt);
end

function c = chart(mdl, name, code)
r = sfroot;
c = r.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/' name]);
assert(isscalar(c), 'Expected one MATLAB Function block: %s', name);
c.Script = char(join(code, newline));
end

function log_signal(system, source, name, x, y)
block = ['log_' name];
add_block('simulink/Sinks/To Workspace', [system '/' block], ...
    'VariableName', block, 'SaveFormat', 'Timeseries', 'Position', [x y x+110 y+25]);
add_line(system, source, [block '/1'], 'autorouting', 'on');
end
