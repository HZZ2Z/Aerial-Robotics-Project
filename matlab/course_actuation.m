function [u, raw] = course_actuation(meas, x, w, ref, P)
%COURSE_ACTUATION 将输入积分器输出 w 与参考前馈映射为物理推力 T 与三轴机体力矩 n。
%   为避免与卡尔曼滤波器形成代数环，此处仅使用姿态角 eta 与机体系角速度 omega，不使用位置估计。
if P.use_kf
    eta = meas(1:3); omega = meas(4:6);
else
    eta = x(7:9); omega = x(10:12);
end

% 1) 计算精确欧拉角运动学矩阵 Q(phi,theta) 及其时间导数 Qdot
Q = euler_kin(eta);
[Q, Qdot] = euler_kin(eta, Q*omega);
if P.use_Qapprox
    % 对比实验开关：用小角度近似 Q=I, Qdot=0 替代精确运动学
    Q = eye(3);
    Qdot = zeros(3);
end

% 2) 推力律：重力与垂直前馈加速度补偿 + 倾角余弦投影补偿（限幅分母防止接近 90 deg 奇异）
T = (P.m*(P.g - ref(9)) + w(1))/max(cos(eta(1))*cos(eta(2)), 0.05);
% 3) 力矩律：将期望角加速度 eta_dd 通过精确运动学与科氏力项映射为机体力矩 n
eta_dd = [ref(17:18); ref(12)] + P.J\w(2:4);
n = P.J*(Q\(eta_dd - Qdot*omega)) + cross(omega, P.J*omega);

% 4) 执行器物理饱和限幅
raw = [T; n];
u = [min(max(T, 0), P.Tmax); min(max(n, -P.nmax), P.nmax)];
end
