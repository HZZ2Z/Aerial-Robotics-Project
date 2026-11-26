function poles = course_hover_stability(P,K,M)
%COURSE_HOVER_STABILITY 对 31 维耦合输出反馈全闭环系统在悬停点求线性化雅可比矩阵并检验稳定性。
%   因分离原理在此不适用（滤波器仅估计平动状态且扰动状态依赖闭环运动），必须联合检验全闭环极点：
%   31 维状态 = 刚体状态 x(12) + 输入积分器 w(4) + 脏微分器 z(6) + 卡尔曼滤波器 xh(9)。
opt = struct('m_scale',1,'J_scale',1,'dt',P.sim.dt,'tfinal',P.sim.tfinal, ...
    'no_fix',false,'in_delay',0,'fix_loss',[inf inf],'seed',P.noise.seed_base, ...
    'wind_mode','none');
F = struct('use_est',true,'use_Qapprox',false,'ff_wind',false);
C = course_config(P,K,F,M,opt);
yaw = [0 pi/2 pi];                         % 分别检验航向角 0 / 90 / 180 deg 处的悬停平衡点
poles = zeros(31,2,numel(yaw));
for k = 1:numel(yaw)
    x = zeros(31,1);
    x(9) = yaw(k);
    ref = zeros(18,1);
    ref(10) = yaw(k);
    for fix = 0:1                          % fix=0（降级 GNSS）与 fix=1（正常 RTK）两种模式
        assert(norm(rate(x,fix,ref,C),inf) < 1e-10,'Invalid hover operating point.');
        A = course_jacobian(@(s) rate(s,fix,ref,C),x);
        poles(:,fix+1,k) = eig(A);
    end
end
disp('Full hover Jacobian: max Re(lambda), rows degraded / RTK, columns heading 0 / 90 / 180 deg');
disp(reshape(max(real(poles),[],1),2,[]));
assert(all(real(poles) < 0,'all'),'The full output-feedback hover loop is not Hurwitz.');
end

function sd = rate(s,fix,ref,C)
% 组合 31 维闭环系统的非线性状态导数
x = s(1:12); w = s(13:16); z = s(17:22); xh = s(23:31);
meas = x(7:12);
u = course_actuation(meas,x,w,ref,C);
[hd,feedback] = course_filter(xh,u(1),meas,x(1:3),fix,C);
[wd,zd] = course_d_law(meas,x,feedback,ref,z,C);
sd = [course_plant_rate(x,u,zeros(3,1),C); wd; zd; hd];
end
