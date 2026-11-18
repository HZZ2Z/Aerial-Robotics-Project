# 历史记录说明（read-only archive）

本目录是 2026-11 一次 autoresearch 参数搜索会话的归档，**只读历史**，不是当前
可运行的基准：

- 其基准 `bench_ertk.m` 建立在**修复前**的闭环上：阵风未接 plant、OU 时间常数
  实现为 0.5 s、传感器为整体 sample-and-hold。该风场自 P1 修复后不再存在，
  所有 RMS 数字（0.1369 → 0.1249 m）只反映旧实现，不可与当前结果比较。
- 文中引用的 `.auto/...` 路径是会话运行时的临时目录位置，仓库内不存在；
  对应脚本已随会话拷贝到本目录（bench_ertk.m、measure.sh、checks.sh）。
- 20 次运行是同一固定种子（1729）上的参数扫描，不构成 Monte Carlo 验证；
  最终参数（rw9 基线 qa/qd、扩展 ou_bias12 qa/qf/qs）以 `matlab/test_kf.m`
  的实际扫描为准。
- 12 状态 two-time-scale 估计器与 1.5 s 软重获增益在当前代码中保留为
  **扩展配置**（`P.kf.mode='ou_bias12'`、`P.kf.reacq_blend_s`），课程基线为
  rw9 硬切换。交互快照见 [autoresearch-dashboard.html](autoresearch-dashboard.html)。
