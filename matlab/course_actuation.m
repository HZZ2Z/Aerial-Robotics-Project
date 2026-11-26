function [u, raw] = course_actuation(meas, x, w, ref, P)
%COURSE_ACTUATION Map the integrated corrections to physical thrust and torque.
if P.use_kf
    eta = meas(1:3); omega = meas(4:6);
else
    eta = x(7:9); omega = x(10:12);
end
Q = euler_kin(eta);
[Q, Qdot] = euler_kin(eta, Q*omega);
if P.use_Qapprox
    Q = eye(3);
    Qdot = zeros(3);
end

% The tilt denominator gives the commanded vertical force in the upright domain.
T = (P.m*(P.g - ref(9)) + w(1))/max(cos(eta(1))*cos(eta(2)), 0.05);
eta_dd = [ref(17:18); ref(12)] + P.J\w(2:4);
n = P.J*(Q\(eta_dd - Qdot*omega)) + cross(omega, P.J*omega);
raw = [T; n];
u = [min(max(T, 0), P.Tmax); min(max(n, -P.nmax), P.nmax)];
end
