# 证据范围与来源

## 审查对象

仓库固定提交：[1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9](https://github.com/HZZ2Z/Aerial-Robotics-Project/tree/1d15ca7a728d2022cc373d52bea6dbbb2fbdd3d9)。2026-11-17核对远端main仍指向此提交。关键源码与模型哈希见 [reference/BASELINE.json](reference/BASELINE.json)。

本开发包中的问题来自源码与已提交Simulink模型接线检查、报告/图像检查以及上一轮独立矩阵计算。没有运行MATLAB/Simulink完整闭环。本包中的目标接口、测试和性能门槛均为实施方案；已有缺陷、工程建议、将来的实测结果分别标明。

## 课程材料

| 来源 | 用途 | 关键页码 |
| --- | --- | --- |
| 用户上传`project2026.pdf` | 课程参数、Objectives1–7、Bonus、论文与代码交付要求 | pp.2–5 |
| 用户上传`project2026_slides.pdf` | 教师设计模型、LQR/KF理论、场景和交付说明 | pp.3–19 |

任务书SHA256：`df0091f44004bba764bdb95ac9ff37fe4b4dd2e611121f9bbe46ea98956d4bfc`。

课件SHA256：`a327284494d409b31834ae444842d70492d255ff40d72b972bfd484a21035b9f`。

课程硬约束：MATLAB/Simulink、连续时间问题、规定质量/惯量/重力/风/传感器规格、任务轨迹、15状态LQR、9状态KF、四类闭环场景、35°倾角、推力/力矩限幅、动画、6页IEEE论文和独立复现代码ZIP。课程未给统一RMS通过线、分项评分或有效提交日期；不能补造。

课程原件不附在本包。实施者没有原件时可先依据本包完整的要求追踪与参数摘要推进，若发生实质歧义再请求原件核实，不必阻塞所有工作。

## 官方技术参考

1. [MathWorks Band-Limited White Noise](https://www.mathworks.com/help/simulink/slref/bandlimitedwhitenoise.html)，核对2026-11-17。`Cov`是白噪声功率谱密度参数，`Ts`是数值相关时间，输出样本方差随相关时间缩放；它不等于未缩放的样本方差。实现时使用本机MATLAB版本的对应文档核对参数名称。
2. [MathWorks lqr](https://www.mathworks.com/help/control/ref/lti.lqr.html)，核对2026-11-17。用于连续时间LQR、Bryson初始权重和Riccati/增益接口说明。实际依赖包含Control System Toolbox，不能只声称需要基础MATLAB与Simulink。

本包OU稳态方差、固定增益协方差传播、扰动分解和指标定义由相应方程推导；这些公式是实施设计与验证依据，不是已测得性能。

## 证据使用规则

- README的0.125m、8.7%改善以及单次种子调参日志属于旧场景下的记录。修复后不得强求复现这些数值，也不得删除历史让其看似新实验。
- 预先选定的5个holdout种子用于小规模独立复核，不称为充分的大样本Monte Carlo证明。
- 独立Python矩阵计算只能支持矩阵层面的数学检查；不能支持完整Simulink执行通过。
- 包内脚本完整性和基线检查通过，仅支持“文件未损坏、版本可识别”，不支持“生产修复已完成”。
