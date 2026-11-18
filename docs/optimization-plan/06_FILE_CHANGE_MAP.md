# 文件修改清单与统一接口

本文件给出目标接口。它们需要在实施阶段落地，不能把名称存在于计划中当作仓库已有功能。保留现有按功能组织的MATLAB函数；仅在多个消费者需要同一公式/定义时提取小函数。

## 修改地图

| 文件 | 具体修改 | 必须同步的消费者 | 主要验收 |
| --- | --- | --- | --- |
| 根`main.m`（新增） | 薄函数入口`main(mode)`，默认full；定位根路径，调用run_project | README、batch命令、交付包 | 根目录直接可运行；不依赖会话旧path |
| `matlab/main.m`→`matlab/run_project.m` | script改函数，去clear，统一实验编排；quick/full结果隔离 | 根main、test脚本 | full覆盖论文所有数据来源；不误用quick产物 |
| `matlab/params.m` | 增加KF/风/参考模式、噪声seed/correlation time、结果路径；移除无消费者字段 | build/run/design/test | 配置序列化；没有隐藏硬编码 |
| `matlab/build_closed.m` | 修风sum/gate/plant/log，OU分母，GNSS恢复；传感器和新增日志接线 | run_closed、quad_sfun、sensor_sfun | 构建脚本与重建slx一致；实际信号与日志一致 |
| `matlab/run_closed.m` | opts不再静默覆盖硬编码；记录原始/延迟/施加输入及实际配置；每run独立目录 | metrics、make_figures、tests | 不同场景只在声明字段变化；不存在旧数据覆盖 |
| `matlab/quad_dynamics.m` | 保持动力学；若需复用，提取唯一饱和/加速度计算，或增加可选诊断输出 | quad_sfun、评价日志 | 真实加速度由同一动力学计算，非差分猜测 |
| `matlab/quad_sfun.m` | 输出或旁路记录a_true和实际施加输入；正确声明新端口feedthrough | build_closed/build_plant | 保留原状态输出无直接馈通；新增诊断不引入反馈 |
| `matlab/sensor_sfun.m` | 连续truth+固定网格噪声；per-instance状态/种子；旧采样保持模式明确隔离 | build_closed、test_kf | 相同seed同噪声；r与dt_n方差缩放正确 |
| `matlab/kf_design.m` | 基线rw9与扩展ou_bias12；返回模型与矩阵结构，不混用A9变量名 | kf_sfun、tests、run_project | 9/12模型维度与可观秩/增益稳定性正确 |
| `matlab/kf_sfun.m` | 根据模型维度组织状态和计时器；记录L/R/status或可重构配置 | run_closed、covariance后处理 | 无state10/13硬编码混淆；no_fix时L=0 |
| `matlab/metrics.m` | 统一指标口径；使用run内配置/真值/协方差；线性求解NEES | 所有图表/README导出 | 手算小样本与指标一致；不使用当前params替换旧配置 |
| `matlab/lqr_design.m` | 三rho、Q/R/K导出；标准或明确命名的恢复响应指标 | 控制实验/报告 | 每轴数值定义清楚，无假settling |
| `matlab/ctrl_sfun.m` | 前馈减号；保留exact Q及anti-windup；可选积分项消融 | hover/Q/FF试验 | 解析符号验证和配对试验 |
| `matlab/gen_ref.m` | mission/q_maneuver/hover_validation模式，输出段边界元数据 | metrics/tests/主入口 | C²拼接；真正悬停窗口可机器选择 |
| `matlab/run_scenarios.m` | 明确10核心场景；不确定性±惯量；runID/配置/seed | full编排和表格 | 实际执行清单等于计划清单 |
| `matlab/make_figures.m` | 零中心σ界、动态P、三权重、创新、对照图；标题来自配置 | report、README | 数据/曲线/图例同源，版本一致 |
| `matlab/animate_mission.m` | 从该run的Pxy画椭圆，用eig(Pxy)而非默认轴对齐；记录场景/时间 | run_project | 动画包含姿态或明确飞行器图形、三轨迹和椭圆 |
| `matlab/test_*.m` | 修过时参数/窗口、统一场景；仅增加捕获真实问题的断言 | quick/full | 失败不被print DONE掩盖 |
| `simulink/*.slx` | 由构建脚本重建并提交匹配版本；检查连线语义 | 打包/复跑 | 不能只手改GUI未更新构建脚本 |
| `report/main.tex` | 替换占位、实际模型推导、插入结果图与表、正文引用、真实身份 | 导出数据/作者资料 | 6页且结果一致；无未解析引用 |
| `README.md`、`README.txt` | 准确入口/依赖/模式/耗时；旧结果历史标识；新数字生成 | full导出 | 干净环境按文档可复现 |
| `docs/autoresearch/*` | 修根路径与参数，或明确归档只读性质 | README | 不再指向不存在的.auto；旧记录不伪装新试验 |

## 最少新增辅助函数

推荐仅在复用确实需要时增加以下小函数；可合并到现有函数，不要求为数量而新增。

- `kf_model.m`：集中A/B/C/Q/R/Tc，避免设计、在线滤波、协方差三处手写不一致。
- `propagate_covariance.m`：使用同一模型与L/R时序离线传播P。
- `experiment_config.m`：生成课程场景/参考/seed，供主入口与独立测试共用。
- `export_results.m`：从metrics/manifest生成JSON、LaTeX表和数值宏；无第二套指标公式。
- `package_submission.m`或小型Python打包工具：只按清单收集文件；不依赖外部服务。

不新增ROS、学习型控制器、数据库、插件系统或Web界面。本次任务不需要大规模重构。

## 权威配置约定

| 字段 | 约定 |
| --- | --- |
| `P.wind.mode` | `none` / `mean_only` / `mean_gust`；`gust_only`仅组件测试 |
| `P.wind.tau/sigma/t0/vmean` | 2 s / 1 m/s / 15 s / 2 m/s，直接来自课程；不得为了通过修改 |
| `P.noise.seed_base` | 单次运行的根seed，确定性映射到风和各传感器独立流 |
| `P.noise.dt_wind/dt_sensor` | 有限带宽噪声的相关时间，需与求解步长对齐；不是物理传感器采样率 |
| `P.sens.model` | 正式`ct_white_approx`；旧`sample_hold`只作单独扩展 |
| `P.kf.mode` | 正式`rw9`，扩展`ou_bias12` |
| `P.kf.reacq_blend_s` | 基线0，扩展可1.5；状态及协方差采用相同L(t) |
| `P.mis.mode` | `mission` / `q_maneuver` / `hover_validation` |
| `opts.feedback` | `ideal` / `estimated`；由一处适配至现有F.use_est |
| `opts.scenario_id` | `nominal` / `wind` / `rtk_lost` / `uncert_Jplus` / `uncert_Jminus`等稳定标识 |
| `opts.execution_id` | 每次主入口的新执行批次；创建独立results目录，复跑不会覆盖随包参考结果 |
| `opts.case_id` | 稳定场景/模式/seed标识，对应reference场景清单；用于跨批次比较 |
| `opts.run_id` | 由execution_id/case_id组成，关联源码、完整参数、seed；同批次重复标识应显式报错 |

结果目录统一为`results/<execution_id>/<case_id>/`。full默认创建新批次，旧批次仅在配置/提交相同时显式复用。P6新解压后仍创建新批次，与随包冻结结果对比，不覆盖原件；时间戳与execution_id不同不属于数值不一致。

`opts`与`P`可以保留当前函数参数形态，但字段含义只能有一个来源。禁止继续“声明P.wind.t0却在构建脚本另写15”的两套配置。

## 关键日志与扰动合同

以下命名为实施约定。保留已有`out.p/v/R/kf/ref/t`，新增/修正字段必须同步metrics和绘图，不维持含义冲突的永久兼容层。

| 日志 | 含义 |
| --- | --- |
| `out.vw` | 唯一实际施加到plant的风，与绘制风完全一致 |
| `out.ucmd` | 控制器输出、延迟前的未饱和命令 |
| `out.u_delayed` | 进入plant之前的延迟后命令 |
| `out.Tn` | 真正用于plant动力学的饱和后推力/力矩；不能再记录控制器侧估计值代替 |
| `out.T_model` | KF实际用于ua的推力标量（包括当前Memory等处理） |
| `out.a_true` | 根据实际plant状态、施加输入和实际参数计算的真实平动加速度 |
| `out.ua_ideal` | 使用out.T_model、真实姿态、标称质量计算的无AHRS噪声预测输入 |
| `out.ua_meas` | KF实际使用AHRS计算的预测输入 |
| `out.d_wind` | 仅风阻加速度，分母用实际质量 |
| `out.d_model` | `a_true-ua_ideal`，包含风、质量、执行器/输入差异；作为总扰动误差的评价真值：rw9对应d，ou_bias12对应df+ds，不是ds单独的真值 |
| `out.d_input_noise` | `ua_ideal-ua_meas`，由测量输入噪声产生，单独记录 |
| `out.d_effective` | `a_true-ua_meas = d_model+d_input_noise`，作为实际预测残差诊断 |
| `out.P_out` | 与[p,v,d_total]对应的9×9模型误差协方差时序；扩展通过Tc投影 |
| `out.innovation` | 测量减预测位置，注明样本时间与有限噪声相关时间 |

闭合检查应满足 `a_true = ua_ideal+d_model = ua_meas+d_model+d_input_noise`（数值容差内）。这些真实量只流向日志和离线评价，不得偷偷进入estimated闭环控制器或KF。由于非线性/模型失配，P_out属于模型预测协方差，不称为实测真实误差协方差；一致性必须由误差数据检验。

直接馈通的诊断输出优先通过独立日志/后处理计算，避免把新诊断端口接回控制回路。任何改动到plant函数均要保留原开环物理检查。
