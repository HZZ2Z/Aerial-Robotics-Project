# Aerial Robotics 项目优化与修改计划

本开发包用于把现有 MATLAB / Simulink 课程项目推进到“场景正确、结果可验证、论文与代码一致、可独立复现”的状态。执行者应保留已实现的六自由度动力学、15 状态 LQR 和现有 S-function 组织方式，优先修复实验有效性，再完成课程交付。

## 适用版本与交付性质

- 仓库：<https://github.com/HZZ2Z/Aerial-Robotics-Project>
- 审查基线：`1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9`，`main`。
- 核对日期：2026-11-17，Asia/Taipei；远端 main 与上一轮审查一致。
- 本包包含具体方案、文件修改清单、验收规则和 agent 提示词；项目生产代码尚未修改。
- 本包的辅助 Python 脚本只检查基线与文件完整性，不运行 MATLAB，也不能证明飞行仿真通过。
- 课程要求来源为用户上传的 `project2026.pdf` 和 `project2026_slides.pdf`。本包保存页码、内容摘要和文件哈希，不附原课程 PDF。

## 首先处理什么

当前最高优先级是风场与验证链路：恒风没有连接到 plant、nominal 仍带阵风、阵风时间常数错误；风日志因此不等于真实输入，扰动误差和 NEES 受到影响。误差图又使用 `error ± 3σ`，不能验证误差是否落在零中心置信界内。修正这些问题以前，旧 README 的性能数值只作为历史记录，不作为新版本通过门槛。

主要算法已有基础：15 状态 LQR、精确 Euler 力矩映射、积分抗饱和、任务轨迹、12 状态估计器和动画都有实现。课程明确要求的 9 状态滤波器应恢复为正式基线，12 状态模型保留为对照扩展，不需要抛弃已有工作。

## 阅读与使用顺序

1. 阅读 [01_AUDIT_REGISTER.md](01_AUDIT_REGISTER.md)，了解问题及证据边界。
2. 按 [02_PHASED_PLAN.md](02_PHASED_PLAN.md) 执行 P0–P6，每阶段留下可复查结果。
3. 实现时使用 [03_PHYSICS_ESTIMATION.md](03_PHYSICS_ESTIMATION.md)、[04_CONTROL_EXPERIMENTS.md](04_CONTROL_EXPERIMENTS.md) 和 [06_FILE_CHANGE_MAP.md](06_FILE_CHANGE_MAP.md)。
4. 用 [05_ACCEPTANCE_AND_REPORT.md](05_ACCEPTANCE_AND_REPORT.md) 验收课程要求；遵守 [07_DEVELOPMENT_RULES.md](07_DEVELOPMENT_RULES.md)。
5. 将 [08_AGENT_START_PROMPT.txt](08_AGENT_START_PROMPT.txt) 全文交给能够访问仓库的开发 agent。中断后使用 [09_REVIEW_AND_RESUME.md](09_REVIEW_AND_RESUME.md)。

可将整个文件夹放在仓库外，或放到单独的 `docs/optimization-plan/`。不要覆盖仓库已有 `AGENTS.md`。不要把包内模板当作已经完成的测试记录。

## 阶段概览

| 阶段 | 目标 | 必须留下的成果 |
| --- | --- | --- |
| P0 | 冻结版本、核实环境、打通入口 | 基线记录、依赖清单、运行状态与历史结果归档 |
| P1 | 修正风场、传感器和真实信号 | 风场组件验证、单一真实输入、统一日志、GNSS 状态测试 |
| P2 | 恢复课程 KF 基线，修正统计验证 | 9/12 状态模型、协方差传播、创新及覆盖率结果 |
| P3 | 完成控制对照实验 | 三种权重权衡、悬停抗风、专用 Q 对照、Bonus 独立结果 |
| P4 | 运行完整场景并形成证据 | 10 次核心场景、固定种子验证、指标表、步长敏感性记录 |
| P5 | 完成论文与复现包 | 6 页论文、生成的图表、根入口、准确 README、提交 ZIP |
| P6 | 冻结交付版本并复跑 | 同一提交及配置的 RERUN 记录和最终校验清单 |

## 辅助检查命令

Python 3.9+，只用标准库。以下是开发包工具，当前即可运行。

```bash
python tools/check_baseline.py --repo /absolute/path/to/Aerial-Robotics-Project
python tools/verify_package.py
```

第一条返回 `BASELINE_MATCH` 只表示版本和关键文件与审查基线一致。修改之后返回 `DRIFT` 是正常现象，必须人工检查差异；它不会判断修复是否正确。第二条只校验本包的文件哈希。

## 完成定义

只有在 P1–P4 的结果有效、P5 的报告完整、P6 从干净导出重新运行成功后，才把项目标记为可提交。没有 MATLAB / Simulink、工具箱、作者学号或课程期限时，对受影响项记为 `BLOCKED`，缺身份资料时原因填写 `MISSING_METADATA`，继续完成其他独立工作，不用虚构结果或身份补齐。

62/100 是上一轮按自拟权重给出的审查评价；本计划不承诺老师最终分数。达到验收条件比追求某个主观分数更有意义。
