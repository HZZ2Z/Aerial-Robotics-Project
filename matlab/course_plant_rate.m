function xd = course_plant_rate(x, u, vw, C)
%COURSE_PLANT_RATE 非线性 6-DOF 四旋翼刚体动力学模型（NED 坐标系，含线性风阻）。
%   状态 x (12x1): [p(1:3); v(4:6); eta(7:9); omega(10:12)]
%   输入 u (4x1):  [总推力 T; 三轴机体力矩 n(2:4)]
%   风速 vw (3x1): 惯性系下的三维风速向量 [m/s]
eta = x(7:9);
omega = x(10:12);
R = euler_R(eta);                                      % 机体系 {B} 到惯性系 {I} 的旋转矩阵
v = x(4:6);
% 平动动力学: m*p_ddot = -T*R*e3 + m*g*e3 - kd*(v - vw)
vd = -u(1)/C.m_true*R(:,3) + [0; 0; C.g] - C.kd/C.m_true*(v-vw);
% 转动动力学: J*omega_dot = -omega x (J*omega) + n
wd = C.J_true\(u(2:4)-cross(omega,C.J_true*omega));
% 欧拉角运动学: eta_dot = Q(phi,theta)*omega
xd = [v; vd; euler_kin(eta)*omega; wd];
end
