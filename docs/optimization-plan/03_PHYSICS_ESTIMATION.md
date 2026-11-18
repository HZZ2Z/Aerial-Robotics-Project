# P1–P2 专项修改方案：风场、测量、扰动真值与估计统计

## 适用基线与目标

本方案针对 `HZZ2Z/Aerial-Robotics-Project` 的固定提交 [`1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9`](https://github.com/HZZ2Z/Aerial-Robotics-Project/tree/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9)。这是待实施计划，不表示修改已经落地或测试已经通过。沿用 MATLAB/Simulink、现有模型构建脚本、S-function、`run_closed`、`run_scenarios` 和绘图入口；不另写仿真框架。

阶段统一为：P0 冻结与环境核实；P1 修正风场、传感器、真值和 GNSS 状态；P2 修正估计器接口、协方差及统计；P3 控制实验和 Bonus；P4 全场景验证；P5 论文与复现；P6 冻结和 RERUN。P1、P2 通过前，旧 README 的误差、NEES、覆盖率与调参收益只能标为历史数据，不能用于最终结论。

## 1. 当前缺陷及文件落点

| 文件（均按固定提交核查） | 当前行为 | 修改范围 |
|---|---|---|
| [`build_closed.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/build_closed.m) | 第123行把纯阵风送入 plant；140–142行把恒风+阵风仅送日志；滤波器分母写成 `[1 tau]`；完全失锁恢复只加1 | 单一实际风信号；OU 系数；连续测量噪声；明确 fix 状态；增补实际输入日志 |
| [`run_closed.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/run_closed.m) | wind 只影响恒风；gust_power 固定4；没有实际延迟输入和滤波器预测输入的完整记录 | 显式场景配置、种子、运行参数快照和统一日志 |
| [`kf_sfun.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/kf_sfun.m) | 12状态+计时器；增益读取状态13，协方差读取状态10；只选稳态协方差 | 维度描述、唯一增益调度、记录实际 L(t)/调度量，移除错误协方差出口 |
| [`metrics.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/metrics.m) | 用错误风日志、标称质量计算 d 真值；全程套 Pg | 匹配定义的物理扰动真值、传播协方差与分阶段统计 |
| [`make_figures.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/make_figures.m) | ±3σ 带以误差自身为中心；RTK 图仅画两个常数边界 | 零中心边界、实际 P(t)、明确场景和时间区间 |
| [`test_kf.m`](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/test_kf.m) | 扫描已无消费者的 qd | 按模型选择真实生效参数，输出设计指纹，避免重复无效实验 |

## 2. 权威配置与日志接口

统一在 `params.m` 定义默认值；场景覆盖由 `run_closed` 接收并解析为一次运行的不可变配置。禁止同一含义同时由 base workspace、硬编码常数和 opts 独立控制。

| 字段 | 取值/语义 |
|---|---|
| `P.wind.mode` | `'none'|'mean_only'|'mean_gust'`；`'gust_only'`仅用于组件检查 |
| `P.wind.vmean/t0/tau/sigma` | 2 m/s、15 s、2 s、每轴1 m/s；正式课程场景固定，组件验证的覆盖必须独立命名 |
| `P.sens.model` | 默认 `'ct_white_approx'`；`'sample_hold'`只作单独工程扩展，不混入课程主结果 |
| `P.noise.seed_base` | 单次运行根种子；各噪声流使用固定、记录在案的偏移 |
| `P.noise.dt_sensor/dt_wind` | 噪声更新周期，均为求解步长的整数倍；与物理传感器标称 rate 分开 |
| `P.kf.mode` | 默认 `'rw9'`；可选 `'ou_bias12'` |
| `P.kf.reacq_blend_s` | 正式基线默认0；扩展可设置1.5 s软恢复 |
| `P.mis.mode` | `'mission'|'q_maneuver'|'hover_validation'`；旧 man 仅允许一次迁移兼容 |
| `opts.fix_loss/no_fix` | 指定时间窗及 degraded / prediction-only；窗外状态必须恢复 good |

每次结果保存 resolved 配置、种子、求解步长、模型版本、Git SHA、MATLAB/工具箱版本、时间网格，以及 `out.vw`、`out.ucmd`、`out.u_delayed`、`out.Tn`、`kf_input`、`fix`、`gain_alpha`。日志命名以06_FILE_CHANGE_MAP.md为准：保留out.vw并修正为真实施加风，不额外维护第二份风真值。实际日志允许只补必要信号，不要求大规模重构结构体。

## 3. P1-A：风场修正

**接线要求。** 先分别生成 mean、gust，再按 mode 组合出唯一 `out.vw`。该输出同时分支到 `plant/2` 与风日志。none 模式必须同时关闭恒风和阵风；组件可以继续在内部生成噪声，但送入 plant 和日志的值必须严格为零。恒风为 `vmean * H(t-t0)`，正式场景按reference清单：阵风从0 s开始演化，恒风在15 s阶跃。这是对课程时间描述的明确实施约定；两者都在15 s启用只能作为另外命名的扩展场景，不得修改正式场景后沿用同一标识。

**OU 模型。** 每轴采用

\[
\dot v_g=-v_g/\tau+\sqrt{2\sigma^2/\tau}\,\xi(t),\qquad S_\xi=1.
\]

因此 Transfer Fcn 的分子为 `sqrt(2*sigma^2/tau)`，分母为 `[1 1/tau]`。当 σ=1、τ=2 时，传递函数为 `1/(s+0.5)`。Band-Limited White Noise 的 `Cov` 表示 Noise power；取 **Cov=1**、Ts=`dt_wind`，离散保持噪声方差约为 `1/Ts`。不得把 Cov 又乘一次 `1/Ts`，也不得保留旧 `gust_power=4` 来补偿错误极点。全项目统一采用 `E[xi(t)xi(s)]=delta(t-s)` 的强度定义，不自行另加2π因子。

**验收。** 硬门禁：none 模式 plant/log 风严格相等且为零；mean_only 在15 s产生2 m/s阶跃；所有模式 plant/log 同源；确定性一阶响应验证 τ=2 s。建议另做短脚本驱动的较长风组件采样，报告稳态标准差、自相关或拟合时间常数及有限样本误差；不以一段62 s飞行样本恰好达到1.000 m/s作为通过标准。

## 4. P1-B：连续时间测量噪声近似

保留课程指定谱密度 `r = sigma^2 / rate`，不要因为改求解步长改变物理 r。若数值噪声每 `dt_sensor` 秒更新一次，噪声幅度应为 `sqrt(r/dt_sensor)`；使用 BLWN 时直接配置 **Cov=r、Ts=dt_sensor**。例如水平 RTK 的 r=8e−6，在1 ms噪声步长下幅度约0.0894 m，而不是0.02 m；这表示不同带宽下的白噪声近似，不表示实际 RTK 精度被改差。

默认测量应为“当前连续真实信号 + 分段保持噪声”，不能继续把真实位置也保持20 ms。优先在现有构建脚本中用 Sum 与 BLWN 构成三个传感器子系统，保留原信号接口；若继续使用 `sensor_sfun`，必须区分连续真值通路与离散噪声缓存。禁止在 Derivatives 或任意被求解器反复调用的 Outputs 中重新抽样。噪声流按种子重置，求解器小步计算不能改变随机序列。

GNSS 标准差切换作用于噪声强度：good 使用 Rg，degraded 使用 Rb。no_fix 时保留测量日志供诊断即可，估计器必须彻底关闭该测量通道。先沿用固定步长1 ms；P4再用一组代表场景减半步长检查主要指标变化，避免把未收敛数值结果当控制性能。

硬门禁是 r 不变、模式切换正确、种子重置后可重复、真值通路无额外采样保持。保持式硬件传感器可作为补充实验，但其采样/延迟模型和结果必须单独标注。

## 5. P1-C：物理扰动真值与输入时序

术语映射以06文件为准：本节d_phys=out.d_model，ua0=out.ua_ideal，uam=out.ua_meas，nu_a=-out.d_input_noise，d_eff=out.d_effective；这是同一组分解，不是两套评分真值。

定义 `T_kf` 为**实际送入滤波器预测端**的命令推力，包含当前 Memory 路径的时序；`T_app` 为延迟之后、经过真实 plant 限幅后施加的推力。现有 `ctrl/3` 只是当前命令的限幅预览，不能代替延迟场景中的 `out.Tn`。

用真实质量 m_t、真实姿态 R_t 与真实风计算

\[
a_t=-T_{app}R_t e_3/m_t+g_t e_3+f_w/m_t.
\]

再定义两个共享同一 `T_kf` 的预测加速度：

\[
u_a^0=-T_{kf}R_t e_3/m_0+g_0e_3,\qquad
u_a^m=-T_{kf}R_m e_3/m_0+g_0e_3.
\]

评分使用的物理扰动真值为 `d_phys = a_t - ua0`，包括风、质量偏差、推力限幅和延迟产生的模型误差；输入测量误差为 `nu_a = uam - ua0`。于是严格恒等式为 `a_t = uam + d_phys - nu_a`。

**禁止把 `d_eff = a_t-uam` 当作无噪声扰动真值，同时又把 AHRS 诱导噪声加到过程协方差中。** 这样会把测量噪声混入 d 的评分目标并重复解释。d_eff 可作为恒等式诊断单独保存；主扰动误差与9维 NEES 使用 d_phys，并说明随机游走/OU只是对该物理扰动的近似。

实现优先从 `udelay` 和 `kfin` 直接记录实际输入；后处理用已记录状态、Pp 和同一 `quad_dynamics` 计算加速度，避免对位置二次差分。若需提取限幅逻辑，增加一个小型共享函数供 plant 与诊断复用，保持原“延迟后限幅”语义。不要复制两套饱和与动力学公式。记录闭环恒等式残差；在参数完全匹配且 `T_kf=T_app` 时检查 d_phys 等于实际风阻加速度。若保留 Memory 的一步滞后，扰动还应包含已记录的推力时间差，不能把它误当风。质量偏差、限幅和延迟应逐项开启验证。

所有参与评分的信号必须带时间戳并对齐到同一评价网格。连续状态允许按约定插值；命令、fix、保持噪声和增益调度采用明确的保持语义，不得用跨越切换时刻的线性插值制造未来信息。保持 `kf_input` 的真实时序，不用当前控制命令倒推出滤波器“应该收到”的输入。组件验证至少覆盖一次推力阶跃和20 ms延迟，确认 `out.u_delayed` 及 `out.Tn` 的时刻、限幅与模型内实际值一致。

## 6. P1-D：GNSS 状态恢复

状态固定为 good=1、degraded=0、no_fix=−1。优先由一个明确的时间窗函数生成状态，供传感器、KF和日志共享。若保留加法阶跃块，恢复幅度必须等于下降幅度 `fix_drop`，不能固定为1。分别验收 `[20,25]` 降级、`[20,21]`完全失锁、`[20,25]`完全失锁：按照[t0,t1)定义，窗前和t1起为1，窗内分别为0或−1；no_fix期间L必须严格为零。用状态序列检查保证时间窗闭开边界一致，不能仅凭最终轨迹看起来正常放行。

## 7. P2-A：9状态基线与12状态扩展

保留 `kf_design` 与 `kf_sfun` 一个运行链，扩展现有 `kfdat` 描述：A、B、C、Q、Rg/Rb、Lg/Lb、Pgood/Pbad全矩阵、nx、输出投影T、状态索引和调度参数。不要复制两套 S-function。

rw9 的状态为 `[p;v;d]`，A为三级积分链，B向v注入模型加速度，C只观测p；输出投影为I9。必须显式证明位置观测秩9、仅速度观测秩6。ou_bias12 保留 `[p;v;df;ds]`，快扰动极点−1/τ；投影 `T*x=[p;v;df+ds]`，位置观测秩12、仅速度观测秩9。

课程主结果先完成 rw9；ou_bias12 为可选扩展，只有在相同修正场景、相同种子和相同评价定义下比较后才讨论收益。9状态扫描 qa/qd，12状态扫描 qa/qf/qs；测试必须打印实际Q与增益变化，禁止设置无消费者字段仍宣称完成扫描。

把计时索引写成 `idx.schedule=nx+1` 并集中定义，或把调度器独立成一个标量状态组件。禁止在不同函数里分别写10或13。使用一个调度函数产生实际 L(t)、alpha 和 measurement_active；记录这些量，协方差后处理不得另写一套“近似相同”的增益切换规则。

## 8. P2-B：与实际固定增益匹配的协方差传播

控制用固定稳态增益及可选重获平滑，不改成未经要求的时变最优滤波器。对其预测误差协方差传播

\[
\dot P=(A-LC)P+P(A-LC)^T+Q+LR(t)L^T.
\]

式中独立过程噪声与GNSS噪声的假设须写明；若确有交叉强度S，需要增加 `-S*L' - L*S'`，不能一边制造相关噪声一边默认为独立。AHRS诱导输入噪声可在悬停线性化下通过加速度对姿态的Jacobian映射到过程强度，额外未建模部分再由qa解释；避免重复计入同一噪声源。

no_fix时 L=0，故 `Pdot=A*P+P*A'+Q`；重获瞬间测量噪声立即回到Rg，但L按实际alpha平滑变化。不得将R也无依据地按alpha混合。除非实际用 `L=P*C'/R` 更新增益，否则不能用最优Kalman Riccati简式冒充当前固定L观察器的协方差方程。

P0按初始误差假设设置并存档；已知完全相同初态可为零，存在初始风/偏置不确定性则加入对应方差。Pgood/Pbad是初始化或比较参考，不能在失锁时直接跳成Pb、恢复时跳回Pg。每步检查对称性、有限性和允许数值误差内的半正定性；重大负特征值应失败，不能靠大幅裁剪隐藏。

12状态在内部传播完整P12，输出评价使用 `P9=T*P12*T'`；9状态直接使用P9。组件检查先验证固定good和固定degraded条件下，稳态P代入上述方程的归一化残差接近数值精度；再检查no_fix窗口和恢复窗口的传播连续性。不要用“换档时P变大了”代替方程验证。

该P(t)是**在声明噪声/模型假设下的误差协方差预测**，不是已经测得的真实协方差。真实误差分布应由独立随机实现验证；饱和、非线性和模型不确定性下尤其不能保证模型预测严格校准。

## 9. P2-C：图表与统计验收

1. 每轴误差曲线与以零为中心的 `±3*sqrt(P9_ii(t))` 同图；失锁背景只标状态时间窗。覆盖率统计采用同一P(t)，分别列 good、degraded/no_fix、reacquisition，不用一个全程百分比掩盖瞬态。
2. 创新保存为 `p_meas - C*xhat`，按good/degraded分段报告均值、标准差、时间序列和相关性。连续白噪声数值近似的点方差含 `R/dt_noise`，不能照搬离散KF的NIS公式。课程要求零均值检验；不把有限带宽、闭环、有时间相关性的数据强行按独立白样本判失败。
3. NEES使用 `[ep;ev;ed_phys]' / P9 * [ep;ev;ed_phys]`，实现用Cholesky/线性求解，不显式求逆。投影误差为9维时理论参照为9；若另报完整12维NEES，必须能定义df/ds各自真值，不能凭空拆分总扰动。
4. 已知零初始协方差的奇异阶段不计算不可定义NEES，明确排除窗口及样本数，不能悄悄用任意epsilon改变得分。单条轨迹的时间点有相关性；禁止把几万时间点当独立试验套卡方置信区间。多种子可在相同时间点计算ensemble ANEES；少量种子只给描述性分布与适当宽的区间。
5. 动画椭圆使用当前 `P9(1:2,1:2)` 的特征分解，保留协方差方向；图注明确“1σ轴长”等尺度，不能把二维1σ椭圆写成68%或95%覆盖区。

硬门禁是公式、维度、状态、数据来源和图形语义正确，以及数值稳定。NEES接近9、覆盖率接近99.7%是需要分析的校准目标；不允许仅为得到漂亮数字盲目放大Q/R。创新均值也应结合相关时间和统计不确定性解释，而非硬设等于零。

## 10. 向P3–P6交接

P0预先固定开发种子和少量holdout种子，例如根种子1729用于调参，1009、2027、4093、8191、16381用于留出验证，并在首次调参前写入清单。建议P4至少对 estimated/wind 与 estimated/rtk_lost 各跑五个holdout；不能用看过的holdout继续调参后仍声称独立验证。

P4核心矩阵为 ideal/estimated × nominal、wind、rtk_lost、uncert_Jplus、uncert_Jminus，共10次；质量+20%、惯量分别+20%/−20%、20 ms输入延迟明确记录。先重建物理正确的基线，再决定12状态扩展和Bonus是否采用。旧0.125 m、8.7%改善不能作为必须复现的门槛；课程的35°最大倾角、推力/力矩上限是硬约束。

P5从新结果自动生成表格、图与参数快照，说明9/12状态、真实噪声近似、固定增益协方差、统计适用条件；同步修复归档autoresearch脚本的 `.auto` 旧路径。P6冻结源码与配置，重新从单一主入口运行并记录RERUN证据。任何影响物理场景、噪声定义或指标真值的后续修改，都使已有结果失效，必须针对受影响场景重跑，不能只改论文数字。

## 官方实现参考

[MathWorks Band-Limited White Noise](https://www.mathworks.com/help/simulink/slref/bandlimitedwhitenoise.html) 用于核对Cov与Ts的含义。公式与接口以本包06文件为准；所有数值门槛区分课程硬限制和工程建议。
