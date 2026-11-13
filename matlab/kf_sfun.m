function kf_sfun(block)
%KF_SFUN Continuous-time steady-state Kalman filter, x=[p;v;d] in R^9.
% in1 = [Tcmd; eta_meas] (4, via Memory), in2 = p_meas (3), in3 = fix status
%   (1: RTK good, 0: degraded, -1: no fix / prediction only)
% out1 = [p_est; v_est; d_est] (9), out2 = diag of selected Pss (9, for logging)
setup(block);
end

function setup(block)
block.NumDialogPrms = 5;
block.NumInputPorts  = 3;
block.NumOutputPorts = 2;
block.InputPort(1).Dimensions  = 4;
block.InputPort(2).Dimensions  = 3;
block.InputPort(3).Dimensions  = 1;
block.OutputPort(1).Dimensions = 9;
block.OutputPort(2).Dimensions = 9;
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(2).SamplingMode = 'Sample';
for k = 1:3, block.InputPort(k).DirectFeedthrough = true; end
block.NumContStates = 9;
block.SampleTime = [0, 0];
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Outputs',     @Outputs);
end

function Outputs(block)
x = block.ContStates.Data;
fix = block.InputPort(3).Data;
block.OutputPort(1).Data = x;
block.OutputPort(2).Data = diag(Pss_sel(block, fix));
end

function Derivatives(block)
P = block.DialogPrm(1).Data;
Tcmd = block.InputPort(1).Data(1);
eta  = block.InputPort(1).Data(2:4);
pmeas = block.InputPort(2).Data;
fix   = block.InputPort(3).Data;
Rm  = euler2rot(eta);
ua  = -(Tcmd/P.m)*Rm*[0;0;1] + P.g*[0;0;1];   % model acceleration
L   = L_sel(block, fix);
A9  = [zeros(3) eye(3) zeros(3); zeros(3) zeros(3) eye(3); zeros(3) zeros(3) zeros(3)];
B9  = [zeros(3); eye(3); zeros(3)];
C9  = [eye(3) zeros(3) zeros(3)];
x   = block.ContStates.Data;
block.Derivatives.Data = A9*x + B9*ua + L*(pmeas - C9*x);
end

function L = L_sel(block, fix)
if     fix >  0.5, L = block.DialogPrm(2).Data;   % RTK good
elseif fix > -0.5, L = block.DialogPrm(3).Data;   % degraded
else               L = zeros(9,3);                % prediction only
end
end

function Pc = Pss_sel(block, fix)
if     fix >  0.5, P = block.DialogPrm(4).Data;
elseif fix > -0.5, P = block.DialogPrm(5).Data;
else               P = block.DialogPrm(5).Data;
end
Pc = P;
end
