function [A, B] = lin_hover(P)
%LIN_HOVER Linearised model at hover (slides 7/8): x=[int e; e; edot; etat; etatdot].
% e is the position error in the heading frame (e = p - p*), so that
% eddot = pddot - pddot* = G*etat - dT/m*e3,  etatddot = J^-1*n.
% Returns A (15x15), B (15x4) and prints rank/structure self-checks.

e3 = [0;0;1];
G  = P.g*[0 -1 0; 1 0 0; 0 0 0];      % d pddot / d eta at hover
Z3 = zeros(3); I3 = eye(3);

A = [Z3 I3  Z3  Z3  Z3;
     Z3 Z3 I3  Z3  Z3;
     Z3 Z3 Z3  G   Z3;
     Z3 Z3 Z3  Z3  I3;
     Z3 Z3 Z3  Z3  Z3];
B = [zeros(3,1), Z3;
     zeros(3,1), Z3;
     -e3/P.m,    Z3;
     zeros(3,1), Z3;
     zeros(3,1), P.J\I3];

% --- self-checks: numeric Jacobian, controllability, decoupled chains ---
check_lin(A, B, P);
end

function check_lin(A, B, P)
% numeric Jacobian of the nonlinear error dynamics at the hover trim
n = 15; m = 4; Jn = zeros(n, n+m); h = 1e-6;
f = @(x,u) err_dot(x, u, P);
x0 = zeros(n,1); u0 = zeros(m,1);
for k = 1:n
    e = zeros(n,1); e(k) = h;
    Jn(:,k) = (f(x0+e,u0) - f(x0-e,u0)) / (2*h);
end
for k = 1:m
    e = zeros(m,1); e(k) = h;
    Jn(:,n+k) = (f(x0,u0+e) - f(x0,u0-e)) / (2*h);
end
Jabs = max(abs(Jn(:))); errAB = max(abs(Jn - [A B]), [], 'all') / max(Jabs, eps);
fprintf('lin_hover: max rel. |numeric - analytic| = %.2e\n', errAB);
assert(errAB < 1e-6, 'analytic A,B do not match numeric Jacobian');

% controllability: 15 with integral states, 12 without
rc = rank(ctrb(A, B));
A12 = A(4:end, 4:end); B12 = B(4:end, :);
rc12 = rank(ctrb(A12, B12));
fprintf('lin_hover: rank ctrb = %d (with int), %d (without int)\n', rc, rc12);
assert(rc == 15 && rc12 == 12);

% permutation reveals four decoupled chains: x-theta (5), y-phi (5), z (3), psi (2)
pm = [1 4 7 11 14  2 5 8 10 13  3 6 9  12 15];
Ap = A(pm, pm); Bp = B(pm, :);
sz = [5 5 3 2]; o = 0; ranks = zeros(1,4);
for k = 1:4
    ix = o+(1:sz(k)); ranks(k) = rank(ctrb(Ap(ix,ix), Bp(ix,:))); o = o+sz(k);
end
blk = zeros(15,15); o = 0;
for k = 1:4, ix = o+(1:sz(k)); blk(ix,ix) = 1; o = o+sz(k); end
res = max(max(abs(Ap .* (~blk & eye(15)==0))));
fprintf('lin_hover: chain ranks = [%d %d %d %d], off-block |A| = %.1e\n', ranks, res);
assert(all(ranks == sz) && res == 0);
end

function xd = err_dot(x, u, P)
% error-state dynamics at the psi=0 hover trim (used for the numeric check)
% kd=0: the wind drag is an unmodelled disturbance (slides 7/13 have no drag
% term in A); the estimator's d-state captures it.
P.kd = 0;  % structs are by-value
e  = x(4:6);  ed = x(7:9);  etat = x(10:12); etatd = x(13:15);
R = euler2rot(etat);
xx = [e; ed; R(:); etatd];
xd_full = quad_dynamics(xx, [P.Ttrim + u(1); u(2:4)], [0;0;0], P);
xd = [e; ed; xd_full(4:6); etatd; xd_full(16:18)];
end
