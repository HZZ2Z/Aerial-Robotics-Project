function quad_sfun(block)
%QUAD_SFUN Level-2 MATLAB S-function wrapper for the nonlinear quadrotor.
% Inputs:  u1 = [Tcmd; ncmd] (4), u2 = vw (3)
% Outputs: y1 = p (3), y2 = v (3), y3 = eta (3), y4 = omega (3), y5 = R(:) (9)
% No direct feedthrough: all outputs depend on states only (no algebraic loop).
setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
block.NumInputPorts  = 2;
block.NumOutputPorts = 5;
block.InputPort(1).Dimensions  = 4;
block.InputPort(2).Dimensions  = 3;
for k = 1:4, block.OutputPort(k).Dimensions = 3; end
block.OutputPort(5).Dimensions = 9;
for k = 1:5, block.OutputPort(k).SamplingMode = 'Sample'; end
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(2).DirectFeedthrough = false;
block.NumContStates = 18;
block.NumDworks = 0;
block.SampleTime = [0, 0];                       % continuous
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('Start',       @Start);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Outputs',     @Outputs);
end

function Start(block)
R0 = eye(3);
block.ContStates.Data = [zeros(3,1); zeros(3,1); R0(:); zeros(3,1)];  % p=0 at rest, R=I
end

function Derivatives(block)
P = block.DialogPrm(1).Data;
u  = block.InputPort(1).Data;
vw = block.InputPort(2).Data;
block.Derivatives.Data = quad_dynamics(block.ContStates.Data, u, vw, P);
end

function Outputs(block)
x = block.ContStates.Data;
block.OutputPort(1).Data = x(1:3);
block.OutputPort(2).Data = x(4:6);
block.OutputPort(3).Data = rot2euler(reshape(x(7:15),3,3));
block.OutputPort(4).Data = x(16:18);
block.OutputPort(5).Data = x(7:15);
end
