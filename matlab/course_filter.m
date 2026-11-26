function [xd, feedback] = course_filter(x, T, meas, y, fix, P)
%COURSE_FILTER 连续时间稳态卡尔曼观测器在线方程（支持 RTK 正常/降级/完全失锁三模式）。
%   1) 根据接收机状态 fix 切换观测器增益 L：
%      fix > 0.5 : RTK 固定解正常模式 (L_rtk)
%      fix >= 0  : RTK 失锁降级为普通 GNSS 模式 (L_deg)
%      fix < 0   : GNSS 完全失锁模式 (L = 0，仅靠动力学模型开环预测，用于 Bonus 实验)
if fix > 0.5
    L = P.L_rtk;
elseif fix >= 0
    L = P.L_deg;
else
    L = zeros(9,3);
end
% 2) 利用指令推力 T 与 AHRS 量测姿态构造模型预测加速度 ua
R = euler_R(meas(1:3));
ua = -(T/P.m)*R(:,3) + [0; 0; P.g];
% 3) 连续时间状态估计导数: xhat_dot = A*xhat + B*ua + L*(y - C*xhat)
xd = P.A_kf*x + P.B_kf*ua + L*(y - P.C_kf*x);
% 输出 [p_hat; v_hat; p_hat_dot] 供控制器使用
feedback = [x(1:6); xd(1:3)];
end
