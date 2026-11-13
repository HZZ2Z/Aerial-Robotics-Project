function xdot = quad_dynamics(x, u, vw, P)
%QUAD_DYNAMICS Nonlinear 6-DOF quadrotor, project model (1), NED frame.
% x = [p; v; R(:); w] (18), u = [T; n(3)], vw = wind velocity in {I}.
% Thrust and torque saturation are part of the vehicle model.
p = x(1:3); v = x(4:6); R = reshape(x(7:15),3,3); w = x(16:18);

T = min(max(u(1), 0), P.Tmax);                       % 0 <= T <= Tmax
nl = [P.nmaxxy; P.nmaxxy; P.nmaxz];
n = min(max(u(2:4), -nl), nl);                       % torque limits

fw  = -P.kd*(v - vw);                                % wind drag force
pdd = -(T/P.m)*R*[0;0;1] + P.g*[0;0;1] + fw/P.m;     % pdd = -T/m*R*e3 + g*e3 + fw/m
Rd  = R*skew(w);                                     % Rdot = R*S(w)
wd  = (P.J\(n - cross(w, P.J*w)));                   % J*wdot = -w x J w + n

xdot = [v; pdd; Rd(:); wd];
end

function S = skew(w)
S = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
end
