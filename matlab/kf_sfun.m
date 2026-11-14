function kf_sfun(block)
%KF_SFUN Continuous-time steady-state Kalman filter, x=[p;v;df;ds] in R^12.
% df: gust disturbance (OU, tau = P.wind.tau), ds: slow random-walk bias.
% in1 = [Tcmd; eta_meas] (4, via Memory), in2 = p_meas (3), in3 = fix status
%   (1: RTK good, 0: degraded, -1: no fix / prediction only)
% out1 = [p_est; v_est; df+ds] (9), out2 = diag of selected Pss (9, for logging)
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
block.NumContStates = 13;   % [x(12); ts = time since last non-good fix (soft re-acquisition)]
block.SampleTime = [0, 0];
block.RegBlockMethod('Start',       @StartFcn);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Outputs',     @Outputs);
end

function StartFcn(block)
block.ContStates.Data = [zeros(12,1); 1e3];   % start as if RTK good for a long time
end

function Outputs(block)
x = block.ContStates.Data;
fix = block.InputPort(3).Data;
block.OutputPort(1).Data = [x(1:6); x(7:9) + x(10:12)];   % combined [p; v; df+ds]
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
A12 = [zeros(3) eye(3) zeros(3) zeros(3);
       zeros(3) zeros(3) eye(3) eye(3);
       zeros(3) zeros(3) -eye(3)/P.wind.tau zeros(3);
       zeros(3) zeros(3) zeros(3) zeros(3)];
B12 = [zeros(3); eye(3); zeros(3); zeros(3)];
C12 = [eye(3) zeros(3) zeros(3) zeros(3)];
x   = block.ContStates.Data(1:12);
ts  = block.ContStates.Data(13);
tsdot = (fix > 0.5) - (fix <= 0.5)*ts/0.02;   % grow while fix good, fast reset otherwise
block.Derivatives.Data = [A12*x + B12*ua + L*(pmeas - C12*x); tsdot];
end

function L = L_sel(block, fix)
Dg = block.DialogPrm(2).Data;
Db = block.DialogPrm(3).Data;
w  = min(1, block.ContStates.Data(13)/1.5);   % 1.5 s Lb->Lg ramp after re-acquisition
if     fix >  0.5, L = w*Dg + (1-w)*Db;        % RTK good (blended in after outage)
elseif fix > -0.5, L = Db;                     % degraded
else               L = zeros(12,3);            % prediction only
end
end

function Pc = Pss_sel(block, fix)
Pg = block.DialogPrm(4).Data;
Pb = block.DialogPrm(5).Data;
w  = min(1, block.ContStates.Data(10)/1.5);
if     fix >  0.5, P = w*Pg + (1-w)*Pb;
elseif fix > -0.5, P = Pb;
else               P = Pb;
end
Pc = P;
end
