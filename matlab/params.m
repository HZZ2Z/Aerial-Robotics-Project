function P = params()
%PARAMS All vehicle, sensor, wind, mission, controller and filter parameters.
%   全局唯一参数配置源（Single Source of Truth）：统一管理四旋翼物理参数、
%   风场模型、传感器噪声、任务轨迹、LQR 权重与卡尔曼滤波器超参数。

% --- 四旋翼物理参数（对应课程任务书表格） ---
P.m   = 1.2;                       % 整机质量 [kg]
P.J   = diag([0.015 0.015 0.026]); % 转动惯量矩阵 [kg m^2]
P.g   = 9.81;                      % 重力加速度 [m/s^2]
P.Tmax    = 24;                    % 总推力上限 [N]（下限为 0）
P.nmaxxy  = 0.5;                   % 滚转/俯仰力矩上限 [N m]
P.nmaxz   = 0.1;                   % 偏航力矩上限 [N m]
P.tiltmax = 0.61;                  % 最大允许倾角 [rad]（约 35 度）
P.kd  = 0.25;                      % 线性气动阻尼系数 [N s/m]
P.Ttrim = P.m*P.g;                 % 悬停配平推力 [N]

% --- 风场模型（t=15s 起沿 x 轴施加 2 m/s 常风 + 一阶滤波随机阵风） ---
P.wind.mode  = 'mean_gust';        % 风场模式: none | mean_only | mean_gust | gust_only
P.wind.vmean = 2;                  % 常值平均风速（北向 x 轴）[m/s]
P.wind.t0    = 15;                 % 常值风起始时间 [s]
P.wind.tau   = 2;                  % 阵风一阶滤波器时间常数 [s]
P.wind.sigma = 1;                  % 三轴阵风标准差 [m/s]

% --- 传感器噪声参数：采样率、1-sigma 标准差及等效连续白噪声功率谱密度 r = sigma^2 * dt ---
P.sens.model = 'ct_white_approx';  % 连续时间白噪声近似模型
P.sens.ahrs.rate = 200;  P.sens.ahrs.sigma = 0.5*pi/180;          % 姿态测量 AHRS: 200 Hz, 8.7e-3 rad
P.sens.gyro.rate = 200;  P.sens.gyro.sigma = 0.01;                % 三轴陀螺仪: 200 Hz, 0.01 rad/s
P.sens.gnss.rate = 50;                                            % GNSS 接收机: 50 Hz
P.sens.gnss.sigma = [0.02; 0.02; 0.04];        % RTK 固定解精度: 水平 0.02 m / 垂直 0.04 m
P.sens.gnss.bad   = [0.5;  0.5;  1.0];         % RTK 失锁降级精度: 水平 0.5 m / 垂直 1.0 m
for f = {'ahrs','gyro','gnss'}
    s = P.sens.(f{1});  s.r = s.sigma.^2 / s.rate;  P.sens.(f{1}) = s;
end
% GNSS 三轴噪声功率谱密度向量（正常 RTK 与降级模式）
P.sens.gnss.rvec  = P.sens.gnss.sigma.^2 / P.sens.gnss.rate;
P.sens.gnss.rbad  = P.sens.gnss.bad.^2   / P.sens.gnss.rate;

% --- 随机噪声种子与更新周期（固定种子以保证不同实验组对比的公平性） ---
P.noise.seed_base = 1729;          % 开发主种子；盲测保留种子: 1009 2027 4093 8191 16381
P.noise.dt_wind   = 1e-2;          % 阵风白噪声步长 [s]
P.noise.dt_sensor = 5e-3;          % AHRS/陀螺仪噪声步长 [s]
P.noise.dt_gnss   = 2e-2;          % GNSS 噪声步长 [s]
P.noise.seed_wind = 21;            % 各通道种子偏移（x/y/z 分别为 +0, +1, +2）
P.noise.seed_ahrs = 31;
P.noise.seed_gyro = 41;
P.noise.seed_gnss = 51;

% --- 任务轨迹参数（NED 北东地坐标系：高度 h 对应 z* = -h） ---
P.mis.h     = 2;        % 巡航高度 [m]
P.mis.tup   = 5;        % 起飞爬升时长 [s]
P.mis.thov  = 3;        % 起飞后与降落前的悬停时长 [s]
P.mis.a     = 3;        % 8 字双纽线幅值 [m]
P.mis.T8    = 20;       % 单圈 8 字周期 [s]
P.mis.laps  = 2;        % 8 字飞行圈数
P.mis.tland = 5;        % 降落时长 [s]
P.mis.tb    = 3;        % gen_ref 辅助轨迹段平滑过渡时长 [s]
P.mis.phase_ramp = 4;   % 课程模板轨迹在相位速率加减速引入的额外过渡总时长 [s]
P.mis.end_rest = 2;     % 落地后静止保持时长 [s]
P.mis.fix_loss = [30 35];          % 默认 RTK 失锁降级时间窗 [s]
P.mis.mode  = 'mission';           % 任务模式: mission | q_maneuver | hover_validation
P.mis.t_hv  = 45;                  % 悬停积分验证实验总时长 [s]

% --- 仿真求解器参数 ---
P.sim.tfinal = P.mis.tup + 2*P.mis.thov + P.mis.laps*P.mis.T8 ...
             + P.mis.phase_ramp + P.mis.tland + P.mis.end_rest;  % 总仿真时长 62 s
P.sim.dt     = 1e-3;                     % RK4 固定步长 [s]

% --- LQR 控制器参数（Bryson 法则最大允许误差尺度；R 矩阵乘以缩放系数 rho） ---
P.lqr.ximax  = 0.5;            % 位置积分误差尺度 [m s]
P.lqr.yaw_int_max = 20*pi/180; % 航向积分误差尺度 [rad s]
P.lqr.eps_d  = 0.05;           % 脏微分器（近似微分）时间常数 [s]
P.lqr.flight_rho = 10;         % 主飞行任务选用的 R 权重系数 rho
P.lqr.emax   = 1.0;            % 位置误差尺度 [m]
P.lqr.vmax   = 1.0;            % 速度误差尺度 [m/s]
P.lqr.etamax = 20*pi/180;      % 姿态角误差尺度 [rad]
P.lqr.wmax   = 1.0;            % 角速度误差尺度 [rad/s]
P.lqr.dTmax  = 6;              % 推力偏差尺度 [N]
P.lqr.nmax   = [0.5; 0.5; 0.1];% 三轴力矩极限 [N m]
P.lqr.rhos   = [0.1 1 10];     % 课程要求对比的三组权重因子

% --- 卡尔曼滤波器参数（用于 kf_design 调参） ---
% 课程基线设计：9 状态随机游走扰动模型（rw9）+ RTK 状态硬切换增益
P.kf.mode          = 'rw9';
P.kf.qa            = 0.5;    % 速度通道过程噪声强度（建模/加速度误差）
P.kf.qd            = 0.3;    % 扰动加速度随机游走过程噪声强度
P.kf.qf            = 0.75;   % [扩展模式 ou_bias12] 阵风 OU 过程噪声强度
P.kf.qs            = 0.2;    % [扩展模式 ou_bias12] 慢变偏置随机游走强度
P.kf.reacq_blend_s = 0;      % 0 表示硬切换增益（课程基线默认）；>0 表示重捕平滑过渡时长 [s]
end
