function [A, C, Lg, Lb, Pg, Pb, ob, mdl] = kf_design(P)
%KF_DESIGN Steady-state Kalman filter design, two model modes:
%   'rw9'       (course baseline) x = [p; v; d] in R^9, d random walk,
%               pdot=v, vdot = ua + d, C observes p. Rank O (position) = 9,
%               rank O (velocity only) = 6. Tuned by P.kf.qa / P.kf.qd.
%   'ou_bias12' (extension)     x = [p; v; df; ds] in R^12, df gust (OU,
%               pole -1/tau), ds slow random walk. Output projection
%               Tc*x = [p; v; df+ds]. Rank O (position) = 12, velocity 9.
%               Tuned by P.kf.qa / P.kf.qf / P.kf.qs.
% ua comes from commanded thrust + AHRS attitude: ua = -T/m0*R(eta_meas)*e3 + g0*e3.
% Returns steady-state gains for RTK-good and degraded accuracy, the 9x9
% covariance of the OUTPUT state [p; v; d_total] (Tc projection for 12-state),
% and the observability analysis. mdl carries every matrix the filter, the
% covariance propagation and the metrics need (single source of matrices).

switch P.kf.mode
    case 'rw9'
        nx = 9;  tau = 0;  % d is a random walk: no OU pole
        A = [zeros(3) eye(3)  zeros(3);
             zeros(3) zeros(3) eye(3);
             zeros(3) zeros(3) zeros(3)];
        B = [zeros(3); eye(3); zeros(3)];
        C = [eye(3) zeros(3) zeros(3)];
        Tc = eye(9);
        Q  = diag([zeros(1,3), P.kf.qa^2*ones(1,3), P.kf.qd^2*ones(1,3)]);
        qa = P.kf.qa;  qd = P.kf.qd;  qf = 0;  qs = 0;
    case 'ou_bias12'
        nx = 12;  tau = P.wind.tau;
        A = [zeros(3) eye(3)  zeros(3)      zeros(3);
             zeros(3) zeros(3) eye(3)       eye(3);
             zeros(3) zeros(3) -eye(3)/tau  zeros(3);
             zeros(3) zeros(3) zeros(3)     zeros(3)];
        B = [zeros(3); eye(3); zeros(3); zeros(3)];
        C = [eye(3) zeros(3) zeros(3) zeros(3)];
        Tc = [eye(6) zeros(6,6); zeros(3,6) eye(3) eye(3)];   % [p; v; df+ds]
        Q  = diag([zeros(1,3), P.kf.qa^2*ones(1,3), ...
                   P.kf.qf^2*ones(1,3), P.kf.qs^2*ones(1,3)]);
        qa = P.kf.qa;  qd = 0;  qf = P.kf.qf;  qs = P.kf.qs;
    otherwise
        error('kf_design: unknown P.kf.mode %s', P.kf.mode);
end

% observability: position measurements observable, velocity-only not
Opos = [C; C*A; C*A^2; C*A^3];
rpos = rank(Opos);
Cvel = [zeros(3) eye(3) zeros(3, nx-6)];
rvel = rank([Cvel; Cvel*A; Cvel*A^2; Cvel*A^3]);
ob = struct('rank_pos', rpos, 'rank_vel', rvel, 'nx', nx);
fprintf('kf_design[%s]: nx=%d, rank O (position) = %d, rank O (velocity only) = %d\n', ...
    P.kf.mode, nx, rpos, rvel);
assert(rpos == nx && rvel < nx);

Rg = diag(P.sens.gnss.rvec);           % RTK fix
Rb = diag(P.sens.gnss.rbad);           % fix lost

% filter ARE:  A P + P A' + Q - P C' R^-1 C P = 0  ->  are(A', C'R^-1C, Q)
PgN = are(A', C'*(Rg\C), Q);
PbN = are(A', C'*(Rb\C), Q);
Lg = PgN*C'/Rg;
Lb = PbN*C'/Rb;
assert(all(real(eig(A - Lg*C)) < 0), 'A-LC not Hurwitz (good)');
assert(all(real(eig(A - Lb*C)) < 0), 'A-LC not Hurwitz (bad)');

% covariance of the OUTPUT state [p; v; d_total] (9x9) for metrics/NEES
Pg = Tc*PgN*Tc';
Pb = Tc*PbN*Tc';

mdl = struct('mode', P.kf.mode, 'nx', nx, 'tau', tau, 'A', A, 'B', B, 'C', C, ...
    'Tc', Tc, 'Q', Q, 'Rg', Rg, 'Rb', Rb, 'Lg', Lg, 'Lb', Lb, ...
    'Pg', Pg, 'Pb', Pb, 'blend_s', P.kf.reacq_blend_s, ...
    'm', P.m, 'g', P.g, 'q', struct('qa', qa, 'qd', qd, 'qf', qf, 'qs', qs), ...
    'poles_g', eig(A - Lg*C), 'poles_b', eig(A - Lb*C));
end
