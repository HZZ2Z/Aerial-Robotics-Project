# 实施进度记录

基线提交：`1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9`

实际工作分支/HEAD：`fix/course-validation`（本地，不推送）。

环境与工具箱：MATLAB R2025b，Simulink、Control System Toolbox 许可证确认（ENV CHECK PASS）。ode4 固定步长。

| 阶段 | 任务进度 | 执行判定 | 实际提交 | 证据路径 | 未解决问题 |
| --- | --- | --- | --- | --- | --- |
| P0 | DONE | PASS | 见 git log（P0/P1 提交） | results/legacy_1d15ca7_windlog/（旧结果归档） | 无 |
| P1 | DONE | PASS | P1 提交 | test_wind.m 全部断言通过（490 s 运行日志） | 无 |
| P2 | DONE | PASS | P1/P2 提交 | test_kf.m（线性 ANEES=9.28 校准、扫描完成） | NEES≈3（保守）在非线性任务下如实报告 |
| P3 | DONE | PASS | P3 提交 | test_bonus.m（FF 符号、悬停积分、Q 对照、GNSS 断测） | Q 对照差异很小（如实报告） |
| P4 | IN_PROGRESS | NOT_RUN | — | results/<execution_id>/ 待生成 | full 批次待跑 |
| P5 | IN_PROGRESS | NOT_RUN | — | report/ 待填 | 论文数值待 P4 冻结后生成 |
| P6 | NOT_STARTED | NOT_RUN | — | — | — |

## 本阶段变更（P0–P3）

- D01（风未接 plant / nominal 带阵风）：build_closed.m 重写风区，wstep(mean) + OU 阵风汇成单一 wsum，同时接 plant/2 与 wind_vw 日志；run_closed 以 wind_mode (none/mean_only/mean_gust/gust_only) 解析 wind_gain/gust_gain。
- D02（OU 分母 [1 tau]）：改为 `[1 1/tau]`（pole -0.5，tau=2 s）；BLWN Cov=gust_gain（1），Ts=P.noise.dt_wind；删除 gust_power=4。组件证据：内建 impulse 衰减 1/e 于 2.00 s；3 seed 集合 std≈0.9-1.07、rho(2 s)≈e^-1。
- D03/D15（d 真值错误、指标口径混用）：run_closed 由 u_delayed 饱和得 Tn（实际施加），T_model=Memory(Tcmd)；a_true/ua_ideal/ua_meas/d_model/d_input_noise/d_effective/d_wind 全部按 06 日志合同定义，恒等残差 out.resid 断言 <1e-9；metrics 全部使用 d_model，3D 向量口径，effort 用归一化施加输入。
- D04（误差自身为中心的 3σ）：make_figures est_err 改为零中心 ±3√P9_ii(t)，P(t) 来自 propagate_covariance（与实际 fix 调度一致的 RK4 传播）；metrics 覆盖率按 good/degraded/no_fix/re-acq 分段。
- D05（state10/13 混淆）：kf_sfun 以 kfmdl.nx 组织状态，计时器=nx+1，Pss 诊断口修正（不再被 metrics 使用）。
- D06（9 状态基线缺失）：kf_design 支持 mode='rw9'（秩 9/6 断言）与 'ou_bias12'（秩 12/9）；课程基线默认 rw9；12 状态为扩展对照。
- D08/D09/D10（三权重、悬停窗、Q 任务不一致）：lqr_design 阶跃指标改为 10-90% rise / 超调 / 2% 带（等效恢复响应 y=1-e/e0）；test_closed 悬停窗改为时间轴导出 [5.5,7.5)、[47.5,49.5)；gen_ref 增加 mission/q_maneuver/hover_validation 模式，test_bonus 与主入口共用。
- D11（qd 扫描无消费者）：test_kf 按 mode 扫描真实生效参数（rw9: qa/qd；ou12: qa/qf/qs）。
- D12（惯量 -20% 缺失）：run_scenarios 10 核心 = ideal/estimated × nominal/wind/rtk_lost/uncert_Jplus/uncert_Jminus。
- D13（传感器 sample-and-hold）：build_closed 用 Sum+BLWN(Cov=r, Ts=dt_sensor) 原生实现课程"连续真值+有限带宽白噪声"；sensor_sfun 保留为标记清楚的 legacy 扩展，不再接线。
- D16（前馈符号）：ctrl_sfun `ar = ar - dhf`；解析符号检查通过（d=[0.4167 0 0] → pdd=aref）。
- D17（断测恢复）：fixup After=fix_drop，fix 序列 [t0,t1) 语义断言通过（1→0/-1→1）。
- 新增：根 main.m + matlab/run_project.m（quick/full，每次新 execution_id）、propagate_covariance.m、export_results.m、test_wind.m；make_figures/animate_mission 从运行数据出图（椭圆用 eig(P_xy)，1σ）。
- 旧结果 results/*.mat → results/legacy_1d15ca7_windlog/（历史风场版本，不再作为门槛）。

## 已执行验证（全部 MATLAB R2025b 实跑）

- test_plant PASS（开环悬停/力矩/风阻）。
- test_wind PASS：none 风严格为 0；mean_only 在 t=15 s 精确阶跃 2 m/s；gust 集合统计 + 确定性 τ=2 s；GNSS [t0,t1) 序列；u_delayed=ucmd-20ms、T_model=Memory(cmd)、饱和在施加输入内。
- test_closed PASS：nominal RMS 0.0112 m；wind ideal RMS 0.0495 m，悬停偏置三轴 <5 cm；tilt 16°<35°；恒等残差 <1e-9。
- test_kf PASS：线性模型匹配 ANEES=9.28（CI ±1.44）、覆盖 99.4%；P_prop 与 Van Loan 精确 DT 递推一致（RK4 截差量级）；qa×qd 扫描 posRMS 0.089-0.099 m（平缓最优 qa≈1-2, qd≈1-3）；ou_bias12 对照 0.091 m。
- test_bonus PASS：悬停积分 on 0.0000 m vs off 0.1039 m；FF on 0.0648 vs off 0.1101 m（effort/tilt 同量级）；GNSS 1 s/5 s 漂移 0.62/3.37 m，断测后 fix=1。
- 期间修复：build_closed 遗留 pnoise 接线、kf_design 8 输出捕获、Van Loan 块矩阵符号、off-by-one 对比窗、for k=find(col) 列迭代陷阱（metrics）。

## 35° 倾角约束与飞行配置决策（P4 期间发现）

- 诊断批次 full_20261119_012531（ρ=1, qa=1, qd=1）：estimated/rtk_lost 倾角峰值
  **50.6° @ t=23.4–24.6 s**（degraded 窗口内，非重获时刻），违反课程 35° 硬限制。
- 机理：degraded 模式速度估计噪声 ±0.4–0.9 m/s（Lb 固定增益 + ±0.5/1 m@50 Hz 测量），
  经高带宽 LQR 放大为力矩饱和往复（±0.5 N·m 反复触界）；积分状态仅 0.44 m·s，
  排除积分 windup 主因；ideal 同场景 16.2° 无问题。
- 候选实验（zz_tiltfix）：ρ=10 → 41.4°（仍越界）；qd=3 → 60.3°（更差，方向反了）；
  blend 1.5 s → 无效（峰值在窗内）；qd=0.1 → 36.6°；ρ=10+qd=0.3 → 34.4°；
  **ρ=10+qa=0.5+qd=0.3 → 32.3°（留 2.7° 余量）✓**，posRMS 0.208 m（outage 主导）。
- 决策：飞行配置 = ρ=10（课程三权重之一，约束感知选择）+ qa=0.5/qd=0.3（扫描平坦
  最优内部，跟踪代价可忽略）。params.m 默认值与 run_project/test_* 的 K 选择已更新；
  诊断批次保留为 ρ=1 基线证据，最终 full 批次以新配置重跑（新 execution_id）。

## 结果失效记录

- 1d15ca7 之前的一切 README 数值、figures、NEES/覆盖率、0.125 m 调参结论：基于错误风场接线（阵风-only 进 plant、τ=0.5 s、sample-hold 传感器），全部标记历史（results/legacy_1d15ca7_windlog/），不作为新版本门槛。
- docs/autoresearch 的 20 次参数搜索：单种子、旧风场 → 历史记录。
- 本次任何影响物理场景/噪声/指标真值的后续改动都将使 results/<execution_id> 批次过期并触发受影响重跑。
