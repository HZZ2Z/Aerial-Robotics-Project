function [Q, Qdot, Qphi, Qtheta] = euler_kin(eta, euldot)
%EULER_KIN 计算欧拉角运动学矩阵 Q(phi,theta)（满足 eta_dot = Q*omega）及其时间导数 Qdot。
%   输入: eta = [phi; theta; psi], euldot = [phi_dot; theta_dot]（仅计算 Qdot 时需要）。
%   注意: 当俯仰角 theta = +-90 deg 时，tan(theta) 与 sec(theta) 无界，存在万向节死锁奇异性。
sf = sin(eta(1)); cf = cos(eta(1));
st = sin(eta(2)); ct = cos(eta(2)); tt = tan(eta(2)); sc = 1/ct^2;
Q = [1, sf*tt, cf*tt;
     0, cf,    -sf;
     0, sf/ct, cf/ct];
if nargout > 1
    % 解析偏导数 dQ/dphi 与 dQ/dtheta（Q 与偏航角 psi 无关）
    Qphi   = [0, cf*tt,    -sf*tt;    0, -sf, -cf; 0, cf/ct,    -sf/ct];
    Qtheta = [0, sf*sc,     cf*sc;    0,   0,   0; 0, sf*st*sc, cf*st*sc];
    % 链式法则求时间导数: Qdot = (dQ/dphi)*phi_dot + (dQ/dtheta)*theta_dot
    Qdot = Qphi*euldot(1) + Qtheta*euldot(2);
end
end
