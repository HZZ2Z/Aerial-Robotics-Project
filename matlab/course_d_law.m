function [wd, zd] = course_d_law(meas, x, feedback, ref, z, P)
%COURSE_D_LAW 基于 Kaminer 速度算法（D-methodology）计算输入端积分器导数 wd 与脏微分器导数 zd。
%   所有反馈通道均通过被控对象输入端的 4 个限幅积分器实现（无直通项），从而自然具备抗饱和能力。

% 1) 根据开关选择反馈源：估计状态（KF + AHRS/陀螺仪）或理想真值状态
if P.use_kf
    p = feedback(1:3);
    v = feedback(4:6);
    pd = feedback(7:9);     % 滤波器内部给出的位置导数 xh_dot(1:3)
    eta = meas(1:3);
    omega = meas(4:6);
else
    p = x(1:3);
    v = x(4:6);
    pd = v;
    eta = x(7:9);
    omega = x(10:12);
end

% 2) 将位置/速度误差投影到当前航向坐标系（heading frame），并计入航向旋转引起的科氏项
c = cos(eta(3)); s = sin(eta(3));
Rt = [c, s, 0; -s, c, 0; 0, 0, 1];
rate = euler_kin(eta)*omega;                   % 欧拉角变化率 eta_dot = [phi_dot; theta_dot; psi_dot]
e = Rt*(p - ref(1:3));                         % 航向系位置误差
rotation = rate(3)*[-e(2); e(1); 0];           % 旋转坐标系导数附加项: -psi_dot * S(e3) * e
de = Rt*(pd - ref(4:6)) - rotation;
ev = Rt*(v - ref(4:6)) - rotation;

% 3) 计算姿态角误差（对角度差做 [-pi, pi] 缠绕处理）与欧拉角速率误差
etaerr = eta - [ref(13:14); ref(10)];
etaerr = atan2(sin(etaerr), cos(etaerr));
dw = rate - [ref(15:16); ref(11)];

% 4) 脏微分器（近似微分 s/(eps_d*s + 1)）估算加速度误差 e_ddot 与角加速度误差 eta_ddot
zd = ([ev; dw] - z)/P.eps_d;

% 5) 速度算法积分器变化率: w_dot = -K_I*epsilon - K_e*e_dot - K_eta*eta_dot - K_edot*e_ddot - K_etadot*eta_ddot
wd = -P.K(:,1:4)*[e; etaerr(3)] - P.K(:,5:7)*de ...
     -P.K(:,8:10)*zd(1:3) - P.K(:,11:13)*dw - P.K(:,14:16)*zd(4:6);
end
