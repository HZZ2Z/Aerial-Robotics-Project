function [A12, C12, Lg, Lb, Pg, Pb, ob] = kf_design(P)
%KF_DESIGN 12-state steady-state Kalman filter design, x = [p; v; df; ds].
% pdot=v, vdot = ua + df + ds, where df is the gust disturbance (OU with the
% wind model's time constant) and ds a slow random-walk bias (mean wind drag,
% model error); ua comes from commanded thrust + AHRS attitude.
% Returns steady-state gains for RTK-good and degraded accuracy, the covariance
% of the COMBINED state [p; v; df+ds] (9x9, matches the estimator output and
% the NEES / 3-sigma checks in metrics.m), and the observability analysis.

tau = P.wind.tau;
A12 = [zeros(3) eye(3)  zeros(3)      zeros(3);
       zeros(3) zeros(3) eye(3)       eye(3);
       zeros(3) zeros(3) -eye(3)/tau  zeros(3);
       zeros(3) zeros(3) zeros(3)     zeros(3)];
B12 = [zeros(3); eye(3); zeros(3); zeros(3)];
C12 = [eye(3) zeros(3) zeros(3) zeros(3)];
Tc  = [eye(6) zeros(6,6); zeros(3,6) eye(3) eye(3)];   % combined [p; v; df+ds]

% observability: position measurements observable, velocity-only not
Opos = [C12; C12*A12; C12*A12^2; C12*A12^3];
rpos = rank(Opos);
Cvel = [zeros(3) eye(3) zeros(3) zeros(3)];
rvel = rank([Cvel; Cvel*A12; Cvel*A12^2; Cvel*A12^3]);
ob = struct('rank_pos', rpos, 'rank_vel', rvel);
fprintf('kf_design: rank O (position) = %d, rank O (velocity only) = %d\n', rpos, rvel);
assert(rpos == 12 && rvel < 12);

% process noise: accel mismatch q_a on v-equation, gust OU intensity q_f,
% slow bias random walk q_s
Q12 = diag([zeros(1,3), P.kf.qa^2*ones(1,3), P.kf.qf^2*ones(1,3), P.kf.qs^2*ones(1,3)]);
Rg = diag(P.sens.gnss.rvec);           % RTK fix
Rb = diag(P.sens.gnss.rbad);           % fix lost

% filter ARE:  A P + P A' + Q - P C' R^-1 C P = 0  ->  are(A', C'R^-1C, Q)
Pg12 = are(A12', C12'*(Rg\C12), Q12);
Pb12 = are(A12', C12'*(Rb\C12), Q12);
Lg = Pg12*C12'/Rg;
Lb = Pb12*C12'/Rb;
assert(all(real(eig(A12 - Lg*C12)) < 0), 'A-LC not Hurwitz (good)');
assert(all(real(eig(A12 - Lb*C12)) < 0), 'A-LC not Hurwitz (bad)');

% combined-state covariance (9x9) for metrics.m consistency checks
Pg = Tc*Pg12*Tc';
Pb = Tc*Pb12*Tc';
end
