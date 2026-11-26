function [A, B] = lin_hover(P)
%LIN_HOVER 在航向坐标系（heading frame）下对悬停平衡点进行线性化，构建 16 维增广状态空间。
%   增广状态 x = [int(epsilon); e; e_dot; eta_tilde; eta_tilde_dot] in R^16
%   其中跟踪输出误差 epsilon = [e(1:3); psi_tilde] in R^4（位置 + 偏航角，对应 4 个控制输入）。
I = eye(3); Z = zeros(3);
G = P.g*[0 -1 0; 1 0 0; 0 0 0];               % 姿态角对水平加速度的线性耦合项（滚转->东向，俯仰->北向负号）
Ap = [Z I Z Z; Z -P.kd/P.m*I G Z; Z Z Z I; Z Z Z Z];
Bp = [zeros(5,4); -1/P.m 0 0 0; zeros(3,4); zeros(3,1) P.J\I];
E = eye(12);
E = E([1:3 9],:);                              % 提取被积分的 4 个输出状态: x, y, z, psi
A = [zeros(4), E; zeros(12,4), Ap];
B = [zeros(4); Bp];

% 使用中心差分数值雅可比与解析矩阵对比，并校验可控性矩阵是否满秩 (16/16)
plant = struct('m_true',P.m,'J_true',P.J,'g',P.g,'kd',P.kd);
J = course_jacobian(@rate, zeros(20,1));
assert(norm(J-[A B],inf) < 1e-6, 'Hover blocks differ from the nonlinear Jacobian.');
r = rank(ctrb(A,B));
assert(r == 16, 'The augmented hover plant must be controllable.');
fprintf('Hover Jacobian residual %.2g, controllability rank %d/16\n',norm(J-[A B],inf),r);

    function dx = rate(z)
        x = z(5:16);
        u = z(17:20) + [P.Ttrim; 0; 0; 0];
        dx = [E*x; course_plant_rate(x,u,zeros(3,1),plant)];
    end
end
