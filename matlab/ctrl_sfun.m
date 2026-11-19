function ctrl_sfun(block)
%CTRL_SFUN Multivariable LQR with integral action + exact Q(phi,theta) torque mapping.
% in1 = true states  [p; v; eta; omega] (12)
% in2 = estimated states [p_est; v_est; eta_meas; omega_meas] (12)
% in3 = reference [p*; v*; a*; psi*; psidot*; psiddot*] (15)
% in4 = disturbance estimate d_est (3)
% out1 = [Tcmd; ncmd] (4), out2 = x15 (15), out3 = saturated [T; n] (4)
% Dialog prms: P, K (4x15), F flags struct (use_est, use_Qapprox, ff_wind)
% Integral states have conditional-integration anti-windup.
setup(block);
end

function setup(block)
block.NumDialogPrms = 3;
block.NumInputPorts  = 4;
block.NumOutputPorts = 3;
dims = [12 12 12 3];
for k = 1:4
    block.InputPort(k).Dimensions = dims(k);
    block.InputPort(k).DirectFeedthrough = true;
end
block.OutputPort(1).Dimensions = 4;
block.OutputPort(2).Dimensions = 15;
block.OutputPort(3).Dimensions = 4;
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(3).SamplingMode = 'Sample';
block.NumContStates = 6;   % [int e (3); low-passed d_hat (3)]
block.SampleTime = [0, 0];
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Outputs',     @Outputs);
end

function [x15, u, usat, sat] = local_all(block)
P  = block.DialogPrm(1).Data;
K  = block.DialogPrm(2).Data;
F  = block.DialogPrm(3).Data;
xs = block.InputPort(1).Data;
if F.use_est, xs = block.InputPort(2).Data; end
ref = block.InputPort(3).Data;
pr = ref(1:3); vr = ref(4:6); ar = ref(7:9);
psir = ref(10); psidr = ref(11); psiddr = ref(12);
if F.ff_wind, ar = ar - block.ContStates.Data(4:6); end   % pdd = ua + d: command ua = aref - d_hat

e3 = [0;0;1];
Q0 = euler_kin(xs(7:9));
rates = Q0*xs(10:12);                  % Euler rates from measured body rates
[Q, Qd] = euler_kin(xs(7:9), rates);

% feed-forward attitude and thrust: invert dynamics for (aref, psir)
af  = ar - P.g*e3;                    % thrust direction: -R*e3 = af/|af|
Tff = P.m*norm(af);
zb  = -af/norm(af);                   % body z axis in {I}
xc  = [cos(psir); sin(psir); 0];
yb  = cross(zb, xc); yb = yb/norm(yb);
xb  = cross(yb, zb);
Rff = [xb yb zb];
ef = rot2euler(Rff);
eta_ff = [ef(1); ef(2); psir];

% error states (heading frame of the measured yaw)
ps  = xs(9);
Rp  = [cos(ps) -sin(ps) 0; sin(ps) cos(ps) 0; 0 0 1];
e   = Rp'*(xs(1:3) - pr);
ed  = Rp'*(xs(4:6) - vr);
et  = [wrapPi(xs(7) - eta_ff(1)); wrapPi(xs(8) - eta_ff(2)); wrapPi(ps - psir)];
etd = Q*xs(10:12) - [0;0;psidr];      % etadot = Q(phi,theta)*omega, exact
x15 = [block.ContStates.Data(1:3); e; ed; et; etd];

u  = -K*x15;                          % [dT; n_att]
eta_dd = [0;0;psiddr] + (P.J\u(2:4)); % commanded Euler accelerations
if F.use_Qapprox, Q = eye(3); Qd = zeros(3); end
n = P.J*(Q\ (eta_dd - Qd*xs(10:12))) + cross(xs(10:12), P.J*xs(10:12));
T = Tff + u(1);

% saturation check for anti-windup
Tl = min(max(T, 0), P.Tmax);
nl = [P.nmaxxy; P.nmaxxy; P.nmaxz];
nl = min(max(n, -nl), nl);
sat = any(abs([Tl - T; nl - n]) > 1e-9);
u = [T; n];
usat = [Tl; nl];
end

function Outputs(block)
[x15, u, usat] = local_all(block);
block.OutputPort(1).Data = u;
block.OutputPort(2).Data = x15;
block.OutputPort(3).Data = usat;
end

function Derivatives(block)
[~, ~, ~, sat] = local_all(block);   % sat flag for anti-windup
xs = block.InputPort(1).Data;
if block.DialogPrm(3).Data.use_est, xs = block.InputPort(2).Data; end
ref = block.InputPort(3).Data;
ps = xs(9);
Rp = [cos(ps) -sin(ps) 0; sin(ps) cos(ps) 0; 0 0 1];
e  = Rp'*(xs(1:3) - ref(1:3));
if sat                              % anti-windup: stop integrating
    e = 0*e;
end
% integrator clamp (bounds outage windup: drift integrates e freely and the
% certainty-equivalence LQR then chases it — see ximax/etamax probes)
P = block.DialogPrm(1).Data;
if isfield(P.lqr, 'xi_max')
    xi = block.ContStates.Data(1:3);
    free = (abs(xi) < P.lqr.xi_max) | (sign(e) ~= sign(xi));
    e = e .* free;                  % integrate only inside the box or inward
end
F  = block.DialogPrm(3).Data;
dhf = block.ContStates.Data(4:6);
tau = 1; if isfield(F, 'ff_tau'), tau = F.ff_tau; end
dhf_dot = (block.InputPort(4).Data - dhf)/max(tau, 1e-3);
block.Derivatives.Data = [e; dhf_dot];
end

function d = wrapPi(a)
d = mod(a + pi, 2*pi) - pi;
end
