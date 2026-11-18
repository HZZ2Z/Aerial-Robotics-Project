# 问题登记与修复优先级

以下结论针对固定基线 `1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9`。行号是定位提示，实施前应按函数及表达式复核。`已确认` 表示源码或模型结构可以直接确认；不代表已经在 MATLAB 复现。所有整改状态初始为 `NOT_STARTED`。

## 关键问题

| ID | 优先级 | 基线证据 | 影响 | 负责阶段 |
| --- | --- | --- | --- | --- |
| D01 | 阻断实验结论 | `build_closed.m` 123、140–142：wmux→plant，wsum→日志；已提交 slx 同样接线 | 恒风未施加；nominal 仍有阵风；两个场景不成立 | P1 |
| D02 | 高 | `build_closed.m` 52–53：分母 `[1 tau]`，`run_closed.m` 固定 gust_power=4 | τ=2 被实现成0.5秒；调整幅值不能修复频谱 | P1 |
| D03 | 高 | `metrics.m` 20、`test_kf.m` 35/43 用错误 vw 日志计算 d | x 扰动真值额外出现0.4167 m/s²，污染 d RMS/NEES | P1/P2 |
| D04 | 高 | `make_figures.m` 76：lo=ep−3σ，hi=ep+3σ | 图上误差必然居中，无法证明一致性 | P2 |
| D05 | 高 | `kf_sfun.m` 74 用 state10 混合协方差，而计时器是 state13；第二输出未接日志 | 图/动画未使用真实传播协方差；失锁增长论证不足 | P2 |
| D06 | 高 | `kf_design.m` 已为12状态；论文仍描述9状态 | 必做9状态基线与实际实现、报告不一致 | P2/P5 |
| D07 | 高 | `report/main.tex`：数值占位、结果表/讨论/结论占位、0个外部图引用 | 不满足最终6页论文交付；没有完整报告PDF可验 | P5 |
| D08 | 中高 | `lqr_design.m` 仅设计三组K；cl.effort未赋值；任务使用Ks{2} | 缺三权重的跟踪误差与控制努力比较 | P3 |
| D09 | 中高 | `test_closed.m` 把12–15秒当悬停，t>55包含降落；断言仅z<0.05 | 不能证明恒风下悬停偏差消除 | P3 |
| D10 | 中 | `main.m` 与 `test_bonus.m` 对Q对比使用不同任务，结果文件同名 | 一键生成结果与宣传的专用机动不一致 | P3/P5 |
| D11 | 中 | 未发现创新日志与均值分析；`test_kf.m` 扫qd，而设计读取qa/qf/qs | 创新验证缺失；旧调参网格含无效重复 | P2 |
| D12 | 中 | `run_scenarios.m` 仅J_scale=1.2 | 未覆盖课程惯量−20%方向 | P4 |
| D13 | 中 | `sensor_sfun.m` 对真信号整体sample-and-hold；课程规定连续真信号加白噪声 | 当前额外采样保持/时延未与课程模型说明区分 | P1 |
| D14 | 中 | 根目录无main.m；README依赖遗漏；归档benchmark仍引用.auto位置 | 首次使用与归档复现不可靠 | P0/P5 |
| D15 | 中 | `metrics.m` hover为各轴均值的RMS；不同脚本使用不同RMS定义；uncert下d仅计算风阻 | 同名指标可差√3；模型误差未按d定义计入 | P1/P2/P4 |
| D16 | Bonus | `ctrl_sfun.m` 43：a_ref + d_hat | 前馈符号与pdd=ua+d不符；不能按有效Bonus计分 | P3 |
| D17 | Bonus | no_fix使fix_drop=2，fixup只+1 | 完全断测后恢复到degraded，非原来的RTK good | P1/P3 |
| D18 | 中 | 20次autoresearch为单固定种子参数实验；缺独立验证种子 | 不能称为20次Monte Carlo；最优参数泛化未证 | P4 |

## 影响关系

D01/D02 必须先于新的调参和性能比较。D03/D15 必须先于 d RMS 和 NEES 的最终解释。D04/D05/D11 完成后才能采用一致性结论。P5 应使用 P4 冻结的结果生成论文，不能在论文里手填旧数字。D16/D17 是可选加分项，失败时清楚标注，不用牺牲核心目标修复时间。

## 不应误判为缺陷的事项

- 课程 slides 第7页的悬停A本来就省略风阻。现有无风阻设计线性化可以保留，但应解释假设，不能把其Jacobian自洽检查描述成全扰动模型已验证。
- 12状态估计器本体可观测、误差设计极点稳定；其输出投影到9维后，NEES参考维数9可以合理。需修的是模型/报告一致性、误差真值与协方差处理。
- 位置参考的C²拼接已有实现。Euler yaw标量的±2π跳变可通过unwrap作图和包角误差处理，不应只因atan2分支就否定物理姿态连续性。
- 课程要求最大倾角35°，不等同于必须裁剪姿态状态。不得修改plant状态来伪造约束满足；应测试并调整允许范围内的控制参数或参考过渡。
- 本次未运行MATLAB是审查证据范围，不是项目缺陷。未能编译论文的本地LaTeX环境问题也不应归因于源码。

## 证据索引

所有代码路径相对仓库根目录。下列为固定版本链接，供核查：

- [build_closed.m](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/build_closed.m)
- [run_closed.m](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/run_closed.m)
- [metrics.m](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/metrics.m)
- [make_figures.m](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/make_figures.m)
- [kf_sfun.m](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/matlab/kf_sfun.m)
- [report/main.tex](https://github.com/HZZ2Z/Aerial-Robotics-Project/blob/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9/report/main.tex)

完整路径与哈希见 [reference/BASELINE.json](reference/BASELINE.json)。
