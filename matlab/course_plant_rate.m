function xd = course_plant_rate(x, u, vw, C)
%COURSE_PLANT_RATE Nonlinear 12-state NED plant, including linear wind drag.
eta = x(7:9);
omega = x(10:12);
R = euler_R(eta);
v = x(4:6);
vd = -u(1)/C.m_true*R(:,3) + [0; 0; C.g] - C.kd/C.m_true*(v-vw);
wd = C.J_true\(u(2:4)-cross(omega,C.J_true*omega));
xd = [v; vd; euler_kin(eta)*omega; wd];
end
