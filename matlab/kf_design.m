function [A9, C9, Lg, Lb, Pg, Pb, ob] = kf_design(P)
%KF_DESIGN 9-state steady-state Kalman filter design (slides 13/14).
% x = [p; v; d], pdot=v, vdot = ua + d, ddot=0; ua from commanded thrust+AHRS.
% Returns steady-state gains for RTK-good and degraded accuracy, the
% covariances, and the observability analysis.

A9 = [zeros(3) eye(3)  zeros(3);
      zeros(3) zeros(3) eye(3);
      zeros(3) zeros(3) zeros(3)];
B9 = [zeros(3); eye(3); zeros(3)];
C9 = [eye(3) zeros(3) zeros(3)];

% observability: position measurements observable, velocity-only not
Opos = [C9; C9*A9; C9*A9^2];
rpos = rank(Opos);
Cvel = [zeros(3) eye(3) zeros(3)];
rvel = rank([Cvel; Cvel*A9; Cvel*A9^2]);
ob = struct('rank_pos', rpos, 'rank_vel', rvel);
fprintf('kf_design: rank O (position) = %d, rank O (velocity only) = %d\n', rpos, rvel);
assert(rpos == 9 && rvel < 9);

% process noise: accel mismatch q_a on v-equation, random walk q_d on d
Q9 = diag([zeros(1,3), P.kf.qa^2*ones(1,3), P.kf.qd^2*ones(1,3)]);
Rg = diag(P.sens.gnss.rvec);           % RTK fix
Rb = diag(P.sens.gnss.rbad);           % fix lost

% filter ARE:  A P + P A' + Q - P C' R^-1 C P = 0  ->  are(A', C'R^-1C, Q)
Pg = are(A9', C9'*(Rg\C9), Q9);
Pb = are(A9', C9'*(Rb\C9), Q9);
Lg = Pg*C9'/Rg;
Lb = Pb*C9'/Rb;
assert(all(real(eig(A9 - Lg*C9)) < 0), 'A-LC not Hurwitz (good)');
assert(all(real(eig(A9 - Lb*C9)) < 0), 'A-LC not Hurwitz (bad)');
end
