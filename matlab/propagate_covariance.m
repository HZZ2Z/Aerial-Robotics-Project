function P9t = propagate_covariance(o, mdl)
%PROPAGATE_COVARIANCE 按实际运行的增益调度律离线传播估计误差协方差矩阵 P(t)。
%   使用 RK4 积分连续 Lyapunov 方程: P_dot = (A-LC)P + P(A-LC)' + Q + L*R(t)*L'。
%   用于展示 RTK 降级/失锁期间协方差的膨胀过程，并计算 +/-3sigma 包络与 NEES。

dt = o.t(2) - o.t(1);
N  = numel(o.t);
nx = mdl.nx;
P  = zeros(nx);                       % 初始真值与估计值重合，故初始协方差 P(0) = 0
ts = 1e3;                             % 重捕计时器初值（默认长期处于良好 RTK 状态）
P9t = zeros(N, 9, 9);
A = mdl.A;  C = mdl.C;  Q = mdl.Q;
for k = 1:N
    P9t(k,:,:) = mdl.Tc*P*mdl.Tc';
    fix = o.fix(k);
    % 根据当前 fix 状态选择对应的滤波器增益 L(t) 与量测噪声强度 R(t)
    if     fix > 0.5
        if mdl.blend_s > 0, w = min(1, ts/mdl.blend_s); else, w = 1; end
        L = w*mdl.Lg + (1-w)*mdl.Lb;  R = mdl.Rg;
    elseif fix > -0.5
        L = mdl.Lb;                   R = mdl.Rb;
    else
        L = zeros(nx, 3);             R = mdl.Rg;   % 完全失锁时 L=0，量测噪声项自动消失
    end
    ts = ts + dt*((fix > 0.5) - (fix <= 0.5)*ts/0.02);
    ts = max(ts, 0);
    if k < N
        % 四阶龙格库塔 (RK4) 单步更新协方差并保持对称性
        Ac = A - L*C;
        W  = L*R*L';
        k1 = Ac*P + P*Ac' + Q + W;
        k2 = Ac*(P + dt/2*k1) + (P + dt/2*k1)*Ac' + Q + W;
        k3 = Ac*(P + dt/2*k2) + (P + dt/2*k2)*Ac' + Q + W;
        k4 = Ac*(P + dt*k3)   + (P + dt*k3)*Ac'   + Q + W;
        P  = P + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        P  = (P + P')/2;              % 消除浮点舍入导致的不对称
    end
end
end
