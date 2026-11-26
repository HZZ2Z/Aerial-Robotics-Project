function test_course()
%TEST_COURSE Check the reference, actuator mapping and coupled design.
P = params();
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'simulink','student_kit'));
[A,B,K] = lqr_design(P);
[~,~,~,~,~,~,ob,M] = kf_design(P);
assert(isequal(size(A),[16 16]) && isequal(size(B),[16 4]));
assert(ob.rank_pos == 9 && ob.rank_vel == 6);
course_hover_stability(P,K{P.lqr.rhos == P.lqr.flight_rho},M);

ref = course_reference_table(P,P.sim.dt,P.sim.tfinal);
t = ref(:,1);
i5 = find(t >= 5,1);
i8 = find(t >= 8,1);
i52 = find(t >= 52,1);
assert(abs(t(end)-62) < eps(62) && norm(ref(1,2:10),inf) < 1e-10);
assert(abs(ref(i5,4)+2) < 1e-10);
assert(norm(ref(end,2:10),inf) < 1e-10);
assert(max(abs(diff(ref(:,11)))) < 0.01,'Reference heading jumps.');
ii = (i8:i52)';
ii = ii(vecnorm(ref(ii,5:6),2,2) > 1e-4);
direction = [cos(ref(ii,11)),sin(ref(ii,11))];
assert(max(vecnorm(direction-ref(ii,5:6)./vecnorm(ref(ii,5:6),2,2),2,2)) < 1e-10);
for tt = [5 8 12 48 52 55 60]
    k = find(t >= tt,1);
    assert(norm(ref(k+1,2:10)-ref(k-1,2:10),inf) < 0.005, ...
        'Discontinuous position, velocity or acceleration at %g s.',tt);
end

a = [0.18;-0.26;0.41];
omega = [0.12;0.3;-0.18];
Q = euler_kin(a);
[~,Qd,Qphi,Qtheta] = euler_kin(a,Q*omega);
h = 1e-6;
assert(norm(Qphi-(euler_kin(a+[h;0;0])-euler_kin(a-[h;0;0]))/(2*h),inf)<1e-8);
assert(norm(Qtheta-(euler_kin(a+[0;h;0])-euler_kin(a-[0;h;0]))/(2*h),inf)<1e-8);
C = struct('use_kf',false,'use_Qapprox',false,'m',P.m, ...
    'g',P.g,'J',P.J,'nmax',P.lqr.nmax,'Tmax',P.Tmax);
w = [0;0.012;-0.014;0.006];
r = zeros(18,1);
r([17 18 12]) = [0.3;0.2;0.09];
u = course_actuation([a;omega],[zeros(6,1);a;omega],w,r,C);
actual = Qd*omega + Q*(P.J\(u(2:4)-cross(omega,P.J*omega)));
assert(norm(actual-r([17 18 12])-P.J\w(2:4),inf) < 1e-10,'Exact torque mapping failed.');
fprintf('Course reference, Euler partial derivatives and 16-state design passed.\n');
end
