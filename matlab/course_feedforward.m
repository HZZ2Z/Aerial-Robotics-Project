function ref = course_feedforward(ref, d, P)
%COURSE_FEEDFORWARD Bonus 实验：利用平滑后的卡尔曼扰动估计 d_hat 及其一二阶导进行风扰前馈补偿。
%   输入 d (9x1) 来自 Simulink 中的三阶平滑滤波器: [d_hat(1:3); d_hat_dot(4:6); d_hat_ddot(7:9)]。
if P.ff_wind
    % 将扰动加速度及其一、二阶导数旋转到参考航向坐标系下
    c = cos(ref(10)); s = sin(ref(10));
    Rt = [c, s; -s, c];
    G = [0, -1; 1, 0];
    dh = Rt*d(1:2);
    dhd = Rt*d(4:5) - ref(11)*G*dh;
    dhdd = Rt*d(7:8) - 2*ref(11)*G*dhd - ref(12)*G*dh + ref(11)^2*dh;
    % 水平扰动加速度通过重力倒数映射为前馈滚转/俯仰倾角增量及其角速率、角加速度增量
    correction = [0, -1; 1, 0]/P.g;
    ref(9) = ref(9) - d(3);                    % 垂直方向扰动直接抵消参考垂直加速度
    ref(13:14) = ref(13:14) + correction*dh;   % 叠加前馈姿态角
    ref(15:16) = ref(15:16) + correction*dhd;  % 叠加前馈姿态角速率
    ref(17:18) = ref(17:18) + correction*dhdd; % 叠加前馈姿态角加速度
end
end
