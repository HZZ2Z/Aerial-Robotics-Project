function [Q, Qdot, Qphi, Qtheta] = euler_kin(eta, euldot)
%EULER_KIN Q(phi,theta) in etadot = Q*omega, and its time derivative Qdot.
% euldot = [phidot; thetadot] (needed only for Qdot). Singularity at theta=+-90 deg.
sf = sin(eta(1)); cf = cos(eta(1));
st = sin(eta(2)); ct = cos(eta(2)); tt = tan(eta(2)); sc = 1/ct^2;
Q = [1, sf*tt, cf*tt;
     0, cf,    -sf;
     0, sf/ct, cf/ct];
if nargout > 1
    Qphi   = [0, cf*tt,    -sf*tt;    0, -sf, -cf; 0, cf/ct,    -sf/ct];
    Qtheta = [0, sf*sc,     cf*sc;    0,   0,   0; 0, sf*st*sc, cf*st*sc];
    Qdot = Qphi*euldot(1) + Qtheta*euldot(2);
end
end
