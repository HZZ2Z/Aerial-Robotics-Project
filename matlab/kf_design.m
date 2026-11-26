function [A, C, Lg, Lb, Pg, Pb, ob, mdl] = kf_design(P)
%KF_DESIGN 连续时间稳态卡尔曼滤波器设计与可观性分析，支持两种模型模式：
%   1) 'rw9'（课程基线）: 状态 x = [p; v; d] in R^9，扰动加速度 d 建模为随机游走。
%      p_dot = v, v_dot = ua + d, 仅观测位置 p。位置可观性秩=9，仅测速度秩=6（不可观）。
%   2) 'ou_bias12'（扩展对比）: 状态 x = [p; v; df; ds] in R^12，含 OU 阵风 df 与慢变偏置 ds。
%   模型加速度 ua 由指令推力与 AHRS 姿态计算: ua = -T/m0*R(eta_meas)*e3 + g0*e3。
%   返回 RTK 正常增益 Lg、降级增益 Lb、稳态协方差 Pg/Pb 及能观性结构体 ob。

switch P.kf.mode
    case 'rw9'
        nx = 9;  tau = 0;  % 扰动加速度 d 为随机游走（无一阶惯性极点）
        A = [zeros(3) eye(3)  zeros(3);
             zeros(3) zeros(3) eye(3);
             zeros(3) zeros(3) zeros(3)];
        B = [zeros(3); eye(3); zeros(3)];
        C = [eye(3) zeros(3) zeros(3)];
        Tc = eye(9);
        Q  = diag([zeros(1,3), P.kf.qa^2*ones(1,3), P.kf.qd^2*ones(1,3)]);
        qa = P.kf.qa;  qd = P.kf.qd;  qf = 0;  qs = 0;
    case 'ou_bias12'
        nx = 12;  tau = P.wind.tau;
        A = [zeros(3) eye(3)  zeros(3)      zeros(3);
             zeros(3) zeros(3) eye(3)       eye(3);
             zeros(3) zeros(3) -eye(3)/tau  zeros(3);
             zeros(3) zeros(3) zeros(3)     zeros(3)];
        B = [zeros(3); eye(3); zeros(3); zeros(3)];
        C = [eye(3) zeros(3) zeros(3) zeros(3)];
        Tc = [eye(6) zeros(6,6); zeros(3,6) eye(3) eye(3)];   % 投影到 [p; v; df+ds]
        Q  = diag([zeros(1,3), P.kf.qa^2*ones(1,3), ...
                   P.kf.qf^2*ones(1,3), P.kf.qs^2*ones(1,3)]);
        qa = P.kf.qa;  qd = 0;  qf = P.kf.qf;  qs = P.kf.qs;
    otherwise
        error('kf_design: unknown P.kf.mode %s', P.kf.mode);
end

% 能观性检验：仅位置观测时满秩（完全可观），仅速度观测时降秩（位置与扰动不可观）
Opos = [C; C*A; C*A^2; C*A^3];
rpos = rank(Opos);
Cvel = [zeros(3) eye(3) zeros(3, nx-6)];
rvel = rank([Cvel; Cvel*A; Cvel*A^2; Cvel*A^3]);
ob = struct('rank_pos', rpos, 'rank_vel', rvel, 'nx', nx);
fprintf('kf_design[%s]: nx=%d, rank O (position) = %d, rank O (velocity only) = %d\n', ...
    P.kf.mode, nx, rpos, rvel);
assert(rpos == nx && rvel < nx);

Rg = diag(P.sens.gnss.rvec);           % RTK 固定解量测噪声强度
Rb = diag(P.sens.gnss.rbad);           % RTK 失锁降级量测噪声强度

% 求解连续时间代数 Riccati 方程 (ARE): A*P + P*A' + Q - P*C'*R^-1*C*P = 0
PgN = are(A', C'*(Rg\C), Q);
PbN = are(A', C'*(Rb\C), Q);
Lg = PgN*C'/Rg;                        % RTK 正常稳态增益
Lb = PbN*C'/Rb;                        % RTK 降级稳态增益
assert(all(real(eig(A - Lg*C)) < 0), 'A-LC not Hurwitz (good)');
assert(all(real(eig(A - Lb*C)) < 0), 'A-LC not Hurwitz (bad)');

% 映射到 9 维输出空间 [p; v; d_total] 的稳态误差协方差，用于一致性检验与 NEES
Pg = Tc*PgN*Tc';
Pb = Tc*PbN*Tc';

mdl = struct('mode', P.kf.mode, 'nx', nx, 'tau', tau, 'A', A, 'B', B, 'C', C, ...
    'Tc', Tc, 'Q', Q, 'Rg', Rg, 'Rb', Rb, 'Lg', Lg, 'Lb', Lb, ...
    'Pg', Pg, 'Pb', Pb, 'blend_s', P.kf.reacq_blend_s, ...
    'm', P.m, 'g', P.g, 'q', struct('qa', qa, 'qd', qd, 'qf', qf, 'qs', qs), ...
    'poles_g', eig(A - Lg*C), 'poles_b', eig(A - Lb*C));
end
