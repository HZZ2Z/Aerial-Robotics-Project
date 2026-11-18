function kf_sfun(block)
%KF_SFUN Continuous-time steady-state Kalman filter.
% Two model modes carried by the dialog struct kfmdl (from kf_design):
%   rw9:       x = [p; v; d]          in R^9,  out = [p; v; d]
%   ou_bias12: x = [p; v; df; ds]     in R^12, out = [p; v; df+ds]
% One extra timer state (index nx+1) tracks time since RTK re-acquisition
% for the optional soft gain blend (kfmdl.blend_s; 0 = hard switch).
% in1 = [Tcmd; eta_meas] (4, Tcmd via Memory), in2 = p_meas (3), in3 = fix
%   (1: RTK good, 0: degraded, -1: no fix / prediction only)
% out1 = output state (9), out2 = diag of selected steady-state output
%   covariance P9 (9, diagnostic only; metrics uses propagated P(t)).
setup(block);
end

function setup(block)
block.NumDialogPrms = 1;
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
block.NumContStates = block.DialogPrm(1).Data.nx + 1;   % [x(nx); ts re-acquisition timer]
block.SampleTime = [0, 0];
block.RegBlockMethod('Start',       @StartFcn);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Outputs',     @Outputs);
end

function StartFcn(block)
nx = block.DialogPrm(1).Data.nx;
block.ContStates.Data = [zeros(nx,1); 1e3];   % start as if RTK good for a long time
end

function Outputs(block)
M = block.DialogPrm(1).Data;
x = block.ContStates.Data;
if M.nx == 9
    y = x(1:9);                               % [p; v; d]
else
    y = [x(1:6); x(7:9) + x(10:12)];          % [p; v; df+ds]
end
block.OutputPort(1).Data = y;
block.OutputPort(2).Data = diag(Pss_sel(M, block.ContStates.Data(M.nx+1)));
end

function Derivatives(block)
M    = block.DialogPrm(1).Data;
Tcmd = block.InputPort(1).Data(1);
eta  = block.InputPort(1).Data(2:4);
pmeas = block.InputPort(2).Data;
fix   = block.InputPort(3).Data;
Rm  = euler2rot(eta);
ua  = -(Tcmd/M.m)*Rm*[0;0;1] + M.g*[0;0;1];   % model acceleration
L   = L_sel(M, fix, block.ContStates.Data(M.nx+1));
tsdot = (fix > 0.5) - (fix <= 0.5)*block.ContStates.Data(M.nx+1)/0.02;
block.Derivatives.Data = [M.A*block.ContStates.Data(1:M.nx) + M.B*ua ...
                          + L*(pmeas - M.C*block.ContStates.Data(1:M.nx)); tsdot];
end

function L = L_sel(M, fix, ts)
% actual gain schedule (hard switch for blend_s = 0, course baseline)
if     fix >  0.5
    if M.blend_s > 0, w = min(1, ts/M.blend_s); else, w = 1; end
    L = w*M.Lg + (1-w)*M.Lb;                  % RTK good (blended in after outage)
elseif fix > -0.5, L = M.Lb;                  % degraded
else               L = zeros(M.nx,3);         % prediction only
end
end

function Pc = Pss_sel(M, ts)
% diagnostic steady-state output covariance (not used by metrics)
if M.blend_s > 0, w = min(1, ts/M.blend_s); else, w = 1; end
Pc = w*M.Pg + (1-w)*M.Pb;
end
