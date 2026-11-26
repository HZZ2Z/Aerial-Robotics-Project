function [A, B, Ks, cl] = lqr_design(P)
%LQR_DESIGN 多变量积分 LQR 控制器设计（基于 Bryson 法则设定 Q 与 R）。
%   返回悬停线性化矩阵 (A, B)、三组权重 rho 对应的反馈增益 Ks，以及闭环极点与阶跃指标 cl。

[A, B] = lin_hover(P);

% 1) 按 Bryson 法则构造对角权重矩阵 Q (16x16) 与 R (4x4)，并对 R 乘以缩放系数 rho
sx  = [P.lqr.ximax*ones(1,3), P.lqr.yaw_int_max, P.lqr.emax*ones(1,3), P.lqr.vmax*ones(1,3), ...
       P.lqr.etamax*ones(1,3), P.lqr.wmax*ones(1,3)];   % 各状态最大允许误差尺度
Q = diag(1 ./ sx.^2);
ru = [P.lqr.dTmax, P.lqr.nmax'];                        % 各控制输入最大允许偏差尺度
Ks = cell(1, numel(P.lqr.rhos));
cl = struct('eig', {}, 'K', {}, 'step', {}, 'Q', {}, 'R', {});
for i = 1:numel(P.lqr.rhos)
    R = P.lqr.rhos(i) * diag(1 ./ ru.^2);
    Ks{i} = lqr(A, B, Q, R);
    assert(all(real(eig(A - B*Ks{i})) < 0), 'A-BK not Hurwitz');
    cl(i).Q = Q;
    cl(i).R = R;
end

% 2) 计算各组增益下的闭环特征值及单轴阶跃恢复动态指标（上升时间、超调量、调节时间）
for i = 1:numel(P.lqr.rhos)
    cl(i).eig  = eig(A - B*Ks{i});
    cl(i).K    = Ks{i};
    cl(i).step = step_metrics(A, B, Ks{i});
end
end

function sm = step_metrics(A, B, K)
% 以初始位置/航向偏差 x0 作为阶跃输入，在理想线性闭环上仿真其收敛恢复过程
Ac = A - B*K;
sm = struct('t', {}, 'e', {});
channels = [5:7 13];                       % 对应 x, y, z 位置误差及 psi 航向误差状态索引
steps = [1 1 1 10*pi/180];                 % 1 m 位置阶跃与 10 deg 航向阶跃
for ax = 1:4
    x0 = zeros(16,1); x0(channels(ax)) = steps(ax);
    [y, t] = initial(ss(Ac, zeros(16,4), eye(16), zeros(16,4)), x0, (0:.01:25)');
    ex = y(:, channels(ax));
    yr = 1 - ex/ex(1);                       % 归一化恢复响应 (0 -> 1)
    sm(ax).t    = t;
    sm(ax).e    = ex;
    sm(ax).rise = rise_time(t, yr);
    sm(ax).over = max(0, max(yr) - 1)*100;
    sm(ax).sett = settling_time(t, abs(ex));
end
end

function tr = rise_time(t, yr)
% 计算 10% -> 90% 上升时间
i10 = find(yr >= 0.1, 1);  i90 = find(yr >= 0.9, 1);
if isempty(i10) || isempty(i90), tr = NaN; else, tr = t(i90) - t(i10); end
end

function ts = settling_time(t, e)
% 计算进入并保持在 2% 误差带内的调节时间
tol = 0.02 * e(1);
idx = find(abs(e) > tol, 1, 'last');
if isempty(idx)
    ts = 0;
elseif idx == numel(t)
    ts = NaN;
else
    ts = t(idx+1);
end
end
