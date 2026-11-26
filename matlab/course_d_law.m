function [wd, zd] = course_d_law(meas, x, feedback, ref, z, P)
%COURSE_D_LAW Input-integrator rates and filtered acceleration errors.
if P.use_kf
    p = feedback(1:3);
    v = feedback(4:6);
    pd = feedback(7:9);
    eta = meas(1:3);
    omega = meas(4:6);
else
    p = x(1:3);
    v = x(4:6);
    pd = v;
    eta = x(7:9);
    omega = x(10:12);
end

c = cos(eta(3)); s = sin(eta(3));
Rt = [c, s, 0; -s, c, 0; 0, 0, 1];
rate = euler_kin(eta)*omega;
e = Rt*(p - ref(1:3));
rotation = rate(3)*[-e(2); e(1); 0];
de = Rt*(pd - ref(4:6)) - rotation;
ev = Rt*(v - ref(4:6)) - rotation;
etaerr = eta - [ref(13:14); ref(10)];
etaerr = atan2(sin(etaerr), cos(etaerr));
dw = rate - [ref(15:16); ref(11)];

zd = ([ev; dw] - z)/P.eps_d;
wd = -P.K(:,1:4)*[e; etaerr(3)] - P.K(:,5:7)*de ...
     -P.K(:,8:10)*zd(1:3) - P.K(:,11:13)*dw - P.K(:,14:16)*zd(4:6);
end
