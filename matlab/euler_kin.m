function [Q, Qdot] = euler_kin(eta, euldot)
%EULER_KIN Q(phi,theta) in etadot = Q*omega, and its time derivative Qdot.
% euldot = [phidot; thetadot] (needed only for Qdot). Singularity at theta=+-90 deg.
sf = sin(eta(1)); cf = cos(eta(1));
st = sin(eta(2)); ct = cos(eta(2)); tt = tan(eta(2)); sc = 1/ct^2;
Q = [1, sf*tt, cf*tt;
     0, cf,    -sf;
     0, sf/ct, cf/ct];
if nargout > 1
    pd = euldot(1); td = euldot(2);
    Qdot = [0, cf*pd*tt + sf*sc*td, -sf*pd*tt + cf*sc*td;
            0, -sf*pd,               -cf*pd;
            0, cf*pd/ct + sf*st*sc*td, -sf*pd/ct + cf*st*sc*td];
end
end
