# cc-gins：一致性校准的学习辅助 GNSS/INS 组合导航

**cc-gins**（Consistency-Calibrated GNSS/INS）是一个可运行、可测试的松组合导航原型。它解决的问题是：在 GNSS 中断时，用学习得到的车辆运动伪观测来约束惯导漂移，同时评估学习观测方差与滤波协方差的经验一致性；在 GNSS 被慢漂移欺骗时，能够检测出来并把它剔除。

> ⚠️ 本仓库的数值结果全部由下文脚本实际运行得到，分三部分：(a) **仿真数据**（`ccgins/sim.py`）；(b) **KITTI raw 真实 OXTS 数据**（真实 IMU 与真值轨迹，但 GNSS 为合成：1 Hz、带中断/欺骗；并在 OXTS IMU 上注入了 MEMS 级误差）；(c) **i2Nav awesome-gins 真实数据**（真实 MEMS IMU×4 + 真实 RTK + 真值，第 6 节）。

调研文档位于 `docs/`：`literature_review.md`（文献综述）、`improvement_opportunities.md`（可改进点）、`project_proposal.md`（方案）。

## 方案对齐与主路径

逐项审计见 [proposal_alignment_matrix.md](docs/proposal_alignment_matrix.md) / [matrix.json](results/proposal_alignment/matrix.json)。方案/综述是设计证据，不代表实现。ACI 不构成数学上的闭环完好性保证。

**Primary（M0–M6）**：批量 InEKF-LC + 学习车体速度/hybrid + ACI/中断膨胀 + Huber GNSS + 双通道 DR-CUSUM。`configs/proposal_primary.yaml` 使用历史已验证 legacy 检查点运行该研究主线；TCN 不作默认工作方法。学习/ACI/监测仍为 batch-only，StreamingNavigator 对这些配置报错。
**Exploratory**：`baseline_range_anchor` 单 RTK 基线距离、`adaptive_r` Sage-Husa、流式扩展、ZUPT/轮速研究。保留这些能力，不替代主线验证。

此前拉回 §3.4：新增 `MotionNetTCN`，三层膨胀卷积 1/2/4 + roll/pitch + 均值/log 方差头 + β-NLL。尾窗内采用对称 padding，不宣称内部特征逐点因果；端点推理只读取过去 IMU/滤波姿态，用当前滤波零偏扣除 IMU 窗口。历史 `MotionNet` 和检查点保留兼容。**下方历史表格全部使用 legacy CNN，未重跑或替换。** 仿真新训练脚本默认 TCN；旧 Python 六通道训练调用、KITTI/i2Nav 历史复现脚本仍为 legacy。

```bash
# 重现此前迷你配对，另存 debug 目录；全部合成 IMU/GNSS
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_experiment --out results/proposal_alignment_debug/smoke
# 主线 assembly，默认历史 legacy 模型；短场景仅作 smoke
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.cli run --config configs/proposal_primary.yaml --out results/proposal_alignment_debug/legacy_primary_cli
# 新训练默认 TCN，使用仿真真值姿态/零偏作监督输入，部署使用滤波估计
.venv/bin/python scripts/train_motionnet.py --architecture proposal_tcn --variance-activation smooth --out results/proposal_alignment_next/exploratory_motionnet_tcn.pt
# 历史架构复现另存，避免覆盖发布模型
.venv/bin/python scripts/train_motionnet.py --architecture legacy_cnn --out results/proposal_alignment_next/exploratory_legacy.pt
```

固定参数与实测配对结果见 [smoke report](results/proposal_alignment/report.md) / [metrics](results/proposal_alignment/metrics.json)。此前两种子平均水平 RMSE（本批复跑完全一致）：legacy CNN **3.001 m**、TCN **3.849 m**、NHC **3.963 m**；TCN 的 NLL **11.596** 对比 legacy **1.938**，全程位置覆盖率 **0.682** 对比 **0.795**，因此本次 TCN 表现更差，没有调参掩盖退化。小样本不证明 TCN 精度提升；网络 NLL/覆盖率用参考姿态/仿真真零偏，导航推理用滤波姿态/估计零偏，分开报告。

下一批调试与完整名义中断配对见 [next report](results/proposal_alignment_next/report.md) / [metrics](results/proposal_alignment_next/metrics.json) / [debug findings](docs/proposal_tcn_debug.md)。预先锁定四条 600 s 训练（100–103）、验证 500–503、**测试 900–909 全十种子**；属于 reduced-train，不是完整 M3 验收。验证选择相同 smooth 方差边界，修复 hard clamp 越界零梯度风险，旧模型数值行为保留。原 smoke 的灾难 NLL 主要来自垂向方差外推过度自信；并未证实训练时垂向下限死梯度是其原因。新网络 NLL legacy **−1.850** / TCN **−1.669**，但导航 RMSE 为 **4.138 / 13.242 m**（NHC 控制 **3.269 m**）。TCN 验证导航较好但测试未确认，**not improved / not primary；legacy 仍为已验证工作主方法**，两种卷积架构均保留。验证 pilot 显示无攻击时双监测误报警并锁存 GNSS 排除可显著恶化导航；没有看测试后改阈值。历史发布表格与检查点未重写，NHC 控制的较低 RMSE 也不代表完整学习/ACI/双监测主链验收。

```bash
# 新配对协议：仅验证选择，完整十测试种子/30、60、120 s 中断
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.alignment_next
```

此前批次剩余偏差（κ_g 现状见下方第三批）：GNSS κ_g、在线安装角/18维状态、HPL/超界率、报警恢复、流式 learning/ACI/integrity、B5 VB/Student-t、≥20 次 MC/全消融/独立道路泛化，以及 M7 UrbanNav 原始伪距/平滑/在线微调。KITTI GNSS 为合成；i2Nav 为真实 RTK，但仅一次行驶的时间折。

## 原始训练预算恢复（第三批）

[完整协议报告](results/proposal_alignment_full/report.md) / [实现与边界](docs/proposal_alignment_full.md)：两种架构均完成 **100–119 共 20 条 × 600 s、相同 12 epoch** 的原始训练预算；验证 500–503、测试 900–909，smooth 方差预先固定。完整测试 RMSE：legacy **5.154 m**、TCN **31.537 m**、NHC 控制 **3.269 m**。TCN 验证略优（4.043 vs 4.061 m）但测试失败，**不提升为 primary；legacy 保留工作主方法**。NHC 控制未启用学习 ACI/双监测，配置差异见报告；历史发布表格/检查点未改写，旧 reduced-train 证据保留。

新增 `Config(gnss_aci=True)`：从三维向量 NIS 在线求 GNSS κ_g，前一尺度进入当前 GNSS/监测更新，新尺度仅用于下一历元；合成校准种子 pilot 与主测试分开。分组相关性估计逐条轨迹计算再平均，不混同此前双监测误报警/锁存问题。完整 streaming learning/ACI/DR-CUSUM、在线安装角/18 状态、HPL/恢复、独立道路泛化与 UrbanNav TC 仍未完成，无完整导航完好性保证。详见报告的实测范围。

## 已实现内容

| 模块 | 文件 | 说明 |
|---|---|---|
| SO(3) 工具 | `ccgins/geometry.py` | Exp/Log、左雅可比、欧拉角 |
| 仿真器 | `ccgins/sim.py` | 车辆轨迹（含停车与转弯，侧偏角）；MEMS IMU（零偏、零偏随机游走、ARW/VRW、随速度增大的振动）；1 Hz GNSS（多路径 Gauss-Markov 误差、杆臂、可配置中断、斜坡式慢漂移欺骗） |
| 惯导机械编排 | `ccgins/filters.py::mechanize` | 采用 ENU 导航系与 FLU 机体系，支持每滤波器地球自转、Coriolis 与当地重力；对应 ESKF/InEKF 协方差动力学；`i2nav_scenarios.earth_config(seq)` 从实测元数据配置，默认仿真关闭 |
| **稳健 GNSS 更新** | `NavFilter.update_gnss` | ESKF/InEKF 共用白化新息 Huber 权重与协方差膨胀；`Config.robust_gnss_delta=1.5` 启用，默认关闭 |
| ESKF 基线 | `ccgins/filters.py::ESKF` | 15 维误差状态，使用 Joseph 形式更新，支持 χ² 门限 |
| **探索：基线距离耦合原型** | `ccgins/range_coupling.py` | `Config.baseline_range_anchor`：每历元将 RTK 基线及其协方差投影为单个距离，与 IMU 预测天线位置耦合；预更新标量 CUSUM 捕获径向杆臂误差。仅径向可观，替代原 RTK 更新，不重复融合；非原始卫星伪距 |
| InEKF 松组合 | `ccgins/filters.py::InEKF` | 右不变 SE₂(3) 误差加可加性零偏，GNSS 观测带杆臂 |
| **零速检测 / ZUPT / 里程计** | `ccgins/zupt.py` | 按传感器噪声密度与留出 IMU 振动标定的因果检测，含速度/转动/重力/持续时间防护；主 `pipeline.run` 可选 ZUPT 与带安装角/轮速点杆臂的里程计更新；ESKF/InEKF 共用 Joseph 更新和 χ² 门限 |
| **实时流式封装** | `ccgins/stream.py` | `StreamingNavigator`：`push_imu` / `push_gnss` / `push_odometry` 增量处理，乱序 GNSS 缓存后按时间戳插入，`get_state` 返回位置、速度、姿态与协方差；复用现有 ESKF/InEKF |
| **探索：自适应 GNSS 量测噪声 R** | `ccgins/adaptive.py` | 基于新息的 Sage-Husa 滑窗估计（遗忘因子加权），按 `[min_scale, max_scale] × R_nominal` 逐轴夹紧；`Config.adaptive_r=False`（默认）时不启用 |
| **配置驱动 CLI** | `ccgins/cli.py` | `ccgins run --config … --out …`（或 `python -m ccgins.cli`）跑仿真/中断场景，写出 `report.md` 与 `metrics.json`；示例配置 `configs/example_sim.yaml` |
| NHC 基线 | `pipeline.py`（`pseudo="nhc"`） | 侧向和垂向速度为 0，σ 固定为 0.1 m/s |
| 学习速度伪观测 | `ccgins/learning.py` | MotionNetTCN：三层膨胀 1/2/4、IMU + roll/pitch；legacy MotionNet 保留，历史模型输入 1 s 原始 IMU（100 个采样），输出机体系速度（前、左、上）的均值和方差，用 β-NLL 训练 |
| **相关性膨胀** | `learning.correlation_factor` | 网络误差存在强时间相关性，用验证集估计 n_eff = 1+2Σρ_k，再按 n_eff 放大伪观测方差 |
| **ACI 在线校准** | `ccgins/aci.py` | 自适应共形推断（Gibbs & Candès 2021）：GNSS 健康时，以先验速度为参考，在线求解各通道方差尺度 κ；GNSS 中断时冻结 κ，并按中断时长做膨胀：κ(1+β·τ/T_ref) |
| **双通道 DR-CUSUM 欺骗检测** | `ccgins/integrity.py` | 独立航位推算通道 B（只用 IMU 和伪观测，不用 GNSS）每 30 s 由主滤波器重置一次；用 GNSS 与 B 的预测之差做 χ² 统计量，送入 CUSUM；报警后剔除 GNSS |
| 单通道 NIS-CUSUM 基线 | `pipeline.py`（`monitor="single"`） | 对主滤波器的 GNSS 新息 NIS 做 CUSUM |
| **InEKF / ESKF 一致性对照** | `ccgins/comparison.py` | i300 实测配对中断实验；`python -m ccgins.comparison` 输出 `results/inekf_vs_eskf/{report.md,metrics.json}`，报告 95% 覆盖率与位置 ANEES |
| 评估 | `ccgins/metrics.py` | 水平/3D RMSE、各段中断内的最大和末端误差、95% 覆盖率（2D NEES ≤ χ²₂(0.95)）、ANEES、单步耗时 |
| **混合伪观测** | `pipeline.py`（`pseudo="hybrid"`） | 侧向/垂向用 NHC（σ=0.1），前向用学习速度；相关膨胀与 ACI 只作用于前向通道 |
| v2：快速 ACI + κ 回退 | `pipeline.py`（`hybrid_max_kappa`） | ACI γ=0.05、窗口 100；前向 κ>10 时该历元丢弃前向通道（退化为纯 NHC） |
| **v3：残差偏差监测（锁存）** | `pipeline.py`（`hybrid_bias_*`） | GNSS 健康时统计前向残差 z−h 的滑窗均值（W=100），z=|均值|/√(平均方差·n_eff/W)；z>4 即永久关闭前向学习通道（锁存），不足 30 个残差前也走 NHC。专门针对 ACI 无法察觉的“方差正常但有偏”的网络输出 |
| KITTI OXTS 读取 | `ccgins/data/kitti.py` | `segments_from_npz`：按时间间隙(>0.1 s)与真值跳变切段，计算机体系速度真值 |
| KITTI 场景 | `ccgins/kitti_scenarios.py` | 驱动划分、中断时间表、合成 GNSS + 斜坡欺骗、MEMS 误差注入 |

## 快速开始

```bash
cd /workspace/cc-gins
python3 -m venv .venv && . .venv/bin/activate
pip install --index-url https://download.pytorch.org/whl/cpu torch
pip install numpy scipy pandas matplotlib pyyaml tqdm pytest
pip install -e .

python -m pytest -q                                   # 单元测试
python scripts/train_motionnet.py --architecture proposal_tcn  # 另存 results/proposal_alignment_next/，保留历史模型
python scripts/run_outage_experiment.py               # GNSS 中断实验（10 个测试种子，约 4 分钟）
python scripts/run_outage_experiment.py --shift       # 分布偏移下的中断实验
python scripts/run_spoof_experiment.py                # 慢漂移欺骗实验（约 3.5 分钟）
python scripts/sweep_aci_beta_val.py                  # 在验证种子上选择 β
python scripts/sweep_hybrid_val.py                    # 在验证种子上选择混合/v2 参数
python scripts/sweep_bias_val.py                      # 在验证种子上选择 v3 偏差监测阈值

# KITTI（真实数据）
python scripts/fetch_kitti_oxts.py                    # 仅下载 8 个 drive 的 oxts 部分（HTTP Range），转存 data/kitti/*.npz（已 gitignore）
python scripts/train_motionnet_kitti.py               # 在 4 个训练 drive 上微调 MotionNet → results/kitti/motionnet_kitti.pt
python scripts/run_kitti_experiment.py --workers 8    # 3 个测试 drive 上的中断实验 → results/kitti/

# i2Nav awesome-gins（真实 MEMS + RTK）
git clone https://github.com/i2Nav-WHU/awesome-gins-datasets   # 约 113 MB，解压其中的 7z（py7zr）到 /tmp/ag
python scripts/prepare_i2nav.py /tmp/ag               # → data/i2nav/*.npz（已 gitignore，约 111 MB）
python scripts/train_motionnet_i2nav.py               # 两折微调 → results/i2nav/motionnet_fold{A,B}.pt
python scripts/run_i2nav_experiment.py --workers 8    # 中断实验（约 25 分钟）
python scripts/run_i2nav_spoof.py                     # 真实 RTK 欺骗实验（约 9 分钟）
python scripts/plot_i2nav_summary.py
```

### 实时流式导航（StreamingNavigator）

`ccgins/stream.py` 把批量滤波器封装为增量接口，可用于在线回放或实时数据。IMU 与 GNSS 都按各自时间戳喂入，GNSS 可以乱序到达：

```python
import numpy as np
from ccgins.pipeline import Config
from ccgins.sim import GnssSpec, ImuSpec, generate_trajectory, simulate_gnss, simulate_imu
from ccgins.stream import StreamingNavigator

truth = generate_trajectory(60.0, seed=0)
imu_spec, gnss_spec = ImuSpec(), GnssSpec()
imu = simulate_imu(truth, imu_spec, seed=0)
gnss = simulate_gnss(truth, gnss_spec, seed=0)

nav = StreamingNavigator.from_truth(
    truth, imu_spec, gnss_spec, Config(pseudo="none", monitor="none"))

# 乱序：全部倒序喂入；内部缓存并按时间戳插入
for m in range(len(gnss.t) - 1, -1, -1):
    nav.push_gnss(gnss.t[m], gnss.pos[m], np.diag(gnss.sigma ** 2))

for k in range(len(imu.t)):
    nav.push_imu(imu.t[k], imu.accel[k], imu.gyro[k])

state = nav.get_state()
print(state.t, state.position, state.velocity, state.attitude)
print(state.position_covariance)
```

滤波器推进时会按时间顺序应用缓冲的 GNSS；若 GNSS 历元落在两个 IMU 采样之间，会在该时刻切分积分区间，在 IMU 网格上的 GNSS 与 `pipeline.run` 顺序一致，离网格切分是流式扩展。`tests/test_stream.py` 验证了同一仿真数据上的批处理一致性（位置/速度/位置协方差）以及乱序 GNSS 的等价性。流式接口支持 IMU+GNSS、因果自适应 ZUPT 与显式安装/杆臂轮速；固定安装角 NHC 已支持；学习伪观测、ACI、完整性监测与自适应 R 仍为批量功能，传入不支持的配置会报错。已经推进过去的 GNSS 历元无法回溯，会抛出 `ValueError`。

### 自适应 GNSS 量测噪声 R（可选，默认关闭）

`ccgins/adaptive.py` 提供 `AdaptiveR`：利用 GNSS 位置新息 `r = z − h` 的协方差 `S = H P Hᵀ + R`，在滑窗内对 `r rᵀ − H P Hᵀ` 做遗忘因子加权平均，得到逐轴 R 估计，并夹紧到 `[min_scale, max_scale] × R_nominal`，避免异常新息把 R 推到 0 或无穷。`gnss_update(..., estimator=None)` 与原有 `nav.update` 完全一致；主流程仅在 `Config(adaptive_r=True)` 时于 GNSS 更新前调用它，因此所有已发表结果不受影响。`tests/test_adaptive.py` 验证了 4 倍 R 噪声下的收敛与上下限夹紧。

### 配置驱动的命令行仿真（`ccgins.cli`）

`ccgins/cli.py` 读取 YAML（也接受 JSON）配置，运行一次仿真/中断场景，并把结果写入 `--out`：

```bash
python -m ccgins.cli run --config configs/example_sim.yaml --out results/cli_demo
# 安装后等价的 console script：
ccgins run --config configs/example_sim.yaml --out results/cli_demo
```

配置段包括 `scenario`、`imu`、`gnss`（含 `outages`）、`filter`（任意 `Config` 字段）、可选 `learning.model` / `learning.architecture`（`legacy_cnn` 或 `proposal_tcn`，须匹配检查点），以及 `metrics.t_start`。`--seed` / `--duration` 可覆盖配置。输出：
- `report.md`：RMSE、最大水平误差、NEES 一致性（ANEES、95% 覆盖率）、耗时、中断表
- `metrics.json`：指标与解析后的配置

示例配置见 `configs/example_sim.yaml`。冒烟测试：`python -m pytest -q tests/test_cli.py`。

## 实验设置

- **数据划分**：训练种子 100–119（20 条，每条 600 s），验证种子 500–503，测试种子 900–909，阈值标定种子 700–705。各组互不重叠。
- **IMU**：工业级 MEMS 量级。陀螺零偏 30°/h，加速度计零偏 5 mg，ARW 0.3°/√h。
- **GNSS**：水平 σ = 1 m，垂向 σ = 2 m，叠加 0.5 m、时间常数 τ = 30 s 的多路径误差。
- **中断**：150 s 起中断 30 s，280 s 起中断 60 s，420 s 起中断 120 s。
- 表中数值为 10 个测试种子的均值±标准差。各方法统一从 60 s 开始统计。

### 1. MotionNet 验证集（`results/motionnet_val.json`）

| 通道 | 前向 | 侧向 | 垂向 |
|---|---|---|---|
| RMSE [m/s] | 0.752 | 0.0033 | 0.0030 |
| 网络方差的 95% 覆盖率（逐点） | 0.939 | 0.935 | 0.941 |
| 误差自相关 n_eff（相关性膨胀因子） | 38.0 | 61.1 | 39.1 |

关键观察：网络给出的方差在**逐点（边缘）意义上已基本校准**，但误差在 10 Hz 下的时间相关性极强（n_eff 约 40–60）。如果把它当作白噪声输入卡尔曼滤波，滤波器会严重过度自信，这正是下表第 5 行的情况。

### 2. GNSS 中断实验，标称条件（`results/outage_table.md`）

| 方法 | 水平RMSE[m] | 30s中断最大误差[m] | 60s中断最大误差[m] | 120s中断最大误差[m] | 95%覆盖率(全程) | 95%覆盖率(中断内) | ANEES(位置) | 耗时[µs/步] |
|---|---|---|---|---|---|---|---|---|
| ESKF (纯惯导外推) | 31.05±10.95 | 10.64±4.85 | 32.74±17.64 | 138.48±58.65 | 0.826 | 0.891 | 1.64±0.27 | 43 |
| ESKF+NHC | 3.33±1.12 | 4.16±2.49 | 10.22±6.79 | 14.47±7.85 | 0.718 | 0.862 | 2.36±0.55 | 49 |
| InEKF+NHC | 3.34±1.12 | 4.17±2.49 | 10.27±6.82 | 14.50±7.81 | 0.718 | 0.862 | 2.36±0.55 | 58 |
| InEKF+学习NHC | 3.42±1.15 | 4.12±2.34 | 10.71±7.50 | 16.86±8.56 | 0.665 | 0.794 | 2.83±0.70 | 58 |
| InEKF+学习速度(白噪声假设) | 4.88±2.53 | 8.69±4.96 | 9.97±5.50 | 12.65±5.84 | 0.200 | 0.118 | 13.70±11.06 | 59 |
| InEKF+学习速度+相关膨胀 | 2.51±0.64 | 4.17±2.32 | 6.23±1.93 | 10.66±4.75 | 0.638 | 0.722 | 2.68±0.61 | 60 |
| InEKF+学习速度+相关膨胀+ACI(β=0) | 2.61±0.70 | 4.68±3.36 | 5.81±1.91 | 11.08±4.97 | 0.623 | 0.695 | 2.79±0.66 | 100 |
| ESKF+学习速度+相关膨胀+ACI | 2.73±0.81 | 4.53±2.94 | 6.57±2.51 | 12.26±5.81 | 0.637 | 0.731 | 2.75±0.64 | 100 |
| InEKF+学习速度+相关膨胀+ACI(v1) | 2.73±0.81 | 4.57±2.96 | 6.56±2.45 | 12.29±5.82 | 0.640 | 0.734 | 2.74±0.64 | 104 |
| InEKF+混合(NHC侧/垂+学习前向)+相关膨胀 | 2.37±0.65 | 4.14±2.64 | 5.96±2.70 | 9.77±4.65 | 0.714 | 0.861 | 2.35±0.57 | 65 |
| InEKF+混合+相关膨胀+快速ACI+κ回退(v2) | 2.61±0.80 | 4.49±3.82 | 6.47±3.87 | 11.53±5.87 | 0.698 | 0.831 | 2.43±0.62 | 80 |
| InEKF+v2+偏差监测锁存回退(v3,本文) | 2.62±0.79 | 4.50±3.82 | 6.57±3.79 | 11.63±5.73 | 0.699 | 0.833 | 2.43±0.62 | 82 |

### 3. GNSS 中断实验，分布偏移（`results/outage_shift_table.md`）

测试 IMU 与训练分布不同：振动系数 ×1.5，加速度计零偏 8 mg。网络**不重新训练**，n_eff 也沿用标称验证集的估计值。

| 方法 | 水平RMSE[m] | 30s中断最大误差[m] | 60s中断最大误差[m] | 120s中断最大误差[m] | 95%覆盖率(全程) | 95%覆盖率(中断内) | ANEES(位置) | 耗时[µs/步] |
|---|---|---|---|---|---|---|---|---|
| ESKF+NHC | 4.11±1.41 | 5.10±3.21 | 12.92±9.47 | 17.78±8.81 | 0.729 | 0.876 | 2.27±0.53 | 53 |
| InEKF+NHC | 4.12±1.41 | 5.11±3.19 | 12.97±9.46 | 17.84±8.79 | 0.730 | 0.876 | 2.27±0.53 | 64 |
| InEKF+学习速度(白噪声假设) | 116.25±27.15 | 102.74±47.76 | 198.87±96.51 | 341.45±107.57 | 0.000 | 0.000 | 1453.85±287.28 | 61 |
| InEKF+学习速度+相关膨胀 | 39.28±19.61 | 17.61±13.78 | 85.31±52.35 | 160.56±88.58 | 0.366 | 0.121 | 11.45±3.28 | 63 |
| InEKF+学习速度+相关膨胀+ACI(v1) | 9.09±12.58 | 4.46±3.15 | 17.03±12.32 | 38.53±46.54 | 0.625 | 0.683 | 49.32±139.76 | 107 |
| InEKF+混合(NHC侧/垂+学习前向)+相关膨胀 | 40.29±19.93 | 17.95±13.67 | 87.84±53.60 | 163.25±89.78 | 0.398 | 0.147 | 11.31±3.42 | 66 |
| InEKF+混合+相关膨胀+快速ACI+κ回退(v2) | 13.03±19.34 | 10.03±16.54 | 15.76±11.80 | 61.37±89.92 | 0.667 | 0.729 | 5.94±8.95 | 79 |
| InEKF+v2+偏差监测锁存回退(v3,本文) | 4.12±1.41 | 5.11±3.19 | 12.97±9.46 | 17.84±8.79 | 0.730 | 0.876 | 2.27±0.53 | 78 |

完整表格（包括其余基线）见 `results/outage_shift_table.md`。

### 4. 慢漂移欺骗实验（`results/spoof_table.md`）

- **场景**：无中断；从 300 s 开始施加斜坡式位置偏移，方向 45°。滤波器为 **v3**（InEKF + 混合伪观测 + 相关膨胀 + 快速 ACI + κ 回退 + 偏差监测锁存）。
- **阈值**：由标定种子干净数据上 CUSUM 最大值的 1.5 倍确定，单通道 h = 42.4，双通道 h = 309.3。
- 报警后剔除 GNSS，剩余时段纯靠 DR 推算。“欺骗后最大误差”统计的是 300–600 s。

| 漂移速率[m/s] | 方法 | 检出率 | 平均检测延迟[s] | 欺骗前误报 | 欺骗后最大水平误差[m] |
|---|---|---|---|---|---|
| 0.2 | 无监测 | - | - | - | 58.13±1.83 |
| 0.2 | 单通道 NIS-CUSUM | 100% | 85.9 | 0 | 28.76±11.87 |
| 0.2 | 双通道 DR-CUSUM(本文) | 100% | 53.4 | 0 | 22.84±7.89 |
| 0.5 | 无监测 | - | - | - | 144.84±5.29 |
| 0.5 | 单通道 NIS-CUSUM | 100% | 26.6 | 0 | 25.95±16.75 |
| 0.5 | 双通道 DR-CUSUM(本文) | 100% | 22.8 | 0 | 26.32±11.74 |
| 1.0 | 无监测 | - | - | - | 289.35±10.26 |
| 1.0 | 单通道 NIS-CUSUM | 100% | 18.0 | 0 | 26.89±32.83 |
| 1.0 | 双通道 DR-CUSUM(本文) | 100% | 11.6 | 0 | 20.62±8.43 |

在干净测试数据（10 条 × 600 s）上，单通道和双通道的误报次数都是 0。（上一版以“学习速度+ACI”为底座时：双通道延迟 56.5/23.6/12.5 s，单通道 77.9/24.3/17.9 s，结论一致。）

### 5. KITTI raw 真实数据验证（`results/kitti/`）

**数据获取**：KITTI raw 的 `*_extract.zip` 内文件为不压缩存储，`scripts/fetch_kitti_oxts.py` 用 remotezip（HTTP Range）只拉取 `oxts/` 成员（100 Hz OXTS），转存为 `data/kitti/<drive>_extract.npz` 后删除文本文件。8 个 drive **实际下载共 132.6 MB，落盘约 39 MB**（npz 合计 40.2 MB），不含相机/激光雷达。原始数据不入库（`.gitignore`：`/data/`、`*.npz`）。

**划分**（按 drive，互不重叠）：
- 训练：2011_09_30_drive_0018 / 0027 / 0034、2011_10_03_drive_0027（共 928 s）
- 验证：2011_09_30_drive_0033（用于 n_eff 与阈值）
- 测试：2011_09_30_drive_0028、2011_10_03_drive_0034、2011_10_03_drive_0042 → 长度 ≥100 s 的 4 段：0028#0 (407 s)、0034#0 (279 s)、0034#1 (139 s)、0042#0 (122 s，高速公路约 26 m/s)

**设置**：OXTS 的 ax/ay/az、wx/wy/wz 作为 IMU 输入，并注入工业级 MEMS 误差（零偏、随机游走、ARW/VRW，不加仿真振动）；OXTS 位姿为真值；GNSS 由真值合成（1 Hz，σ 同仿真），中断时间表为 60 s 起依次 30/120/60 s、间隔 40 s（段长不够则截断）；每段 5 个噪声种子。统计从 30 s 开始。

**数据处理中发现并修正的问题**（如实记录）：
1. OXTS 的 `af/wf` 等字段不是车体系；改用 `ax/wx` 后，纯惯导 20 s 漂移从 38.6 m 降到 3.8 m。
2. `vf/vl/vu` 是水平化坐标系而非机体系；机体速度真值改为由 `ve/vn/vu` 经 Rᵀ 旋转得到（与积分一致性残差约 0.1 m/s）。
3. drive 0028 的 OXTS 真值在约 60–130 s 存在 30 m 级跳变；加入真值跳变检测（`_gt_bad_mask`）并在此处切段。

**MotionNet（验证 drive，`results/kitti/motionnet_val.json`）**：

| 模型 | 前向 RMSE [m/s] | 侧向 | 垂向 | 原始方差 95% 覆盖率（前/侧/垂） |
|---|---|---|---|---|
| 仅仿真训练（零样本） | 4.11 | 0.198 | 0.146 | 0.16 / 0.05 / 0.06 |
| KITTI 微调（10 epoch，从仿真模型初始化） | 2.05 | 0.095 | 0.064 | 0.60 / 0.75 / 0.91 |

真实数据上前向速度只能学到 2 m/s 的精度（仿真里是 0.75 m/s），说明仿真中“振动随车速增大”的假设让前向速度显得比真实情况更可观。

**结果**（`results/kitti/outage_table.md`，4 段 × 5 种子；样本数：30 s 中断 20 个、60 s 中断 5 个、120 s 中断 10 个）：

| 方法 | 水平RMSE[m] | 30s中断最大误差[m] | 60s中断最大误差[m] | 120s中断最大误差[m] | 95%覆盖率(中断内) | 前向回退比例 |
|---|---|---|---|---|---|---|
| ESKF (纯惯导外推) | 46.03±43.23 | 20.54±11.70 | 42.12±13.23 | 328.82±56.50 | 0.777 | - |
| ESKF+NHC | 2.90±1.04 | 4.25±2.15 | 6.32±0.60 | 13.05±1.99 | 0.675 | - |
| InEKF+NHC | 2.92±1.03 | 4.27±2.14 | 6.40±0.70 | 13.08±1.95 | 0.674 | - |
| 学习速度(白噪声)[FT] | 50.25±69.38 | 100.10±159.44 | 7.28±0.61 | 20.86±10.84 | 0.039 | - |
| 学习速度+相关膨胀[FT] | 7.78±8.80 | 22.49±29.83 | 9.32±0.68 | 10.84±3.41 | 0.702 | - |
| v1：+ACI [FT] | 3.33±1.33 | 6.97±4.17 | 9.86±1.54 | 14.46±4.62 | 0.703 | - |
| 混合+相关膨胀[FT] | 6.36±6.71 | 17.26±24.07 | 6.31±0.76 | 10.57±2.43 | 0.549 | - |
| v2 [FT] | 2.83±1.02 | 4.17±2.14 | 6.33±0.67 | 12.37±1.72 | 0.679 | 0.146 |
| **v3（本文）[FT]** | **2.83±1.02** | **4.18±2.13** | **6.33±0.67** | **12.37±1.72** | 0.681 | 0.395 |
| v1 [仿真零样本] | 11.45±9.96 | 28.48±24.74 | 12.47±3.52 | 16.05±5.06 | 0.491 | - |
| v2 [仿真零样本] | 9.43±11.10 | 21.59±26.80 | 6.76±0.56 | 11.90±2.49 | 0.444 | 0.804 |
| v3（本文）[仿真零样本] | 2.92±1.03 | 4.27±2.14 | 6.40±0.70 | 13.08±1.95 | 0.674 | 1.000 |

**v3[FT] 与 InEKF+NHC 的配对比较**（同段同种子）：RMSE 20 次中 14 次更好、1 次更差，平均 −0.08 m；30 s 中断 12 好/3 差，平均 −0.09 m；60 s 中断 5/5 更好，平均 −0.07 m；120 s 中断 10/10 更好，平均 −0.71 m。

**分段观察**：高速段 0042（约 26 m/s，超出训练速度范围）上 v3 前向回退比例 1.0，RMSE 3.25 m，与 NHC 相同；而不带监测的“学习速度+相关膨胀”在该段 RMSE 达 22.87 m。其它段 v3 对 NHC：0034#0 3.03 vs 3.25 m，0028#0 3.60 vs 3.62 m，0034#1 1.44 vs 1.54 m。

### 6. i2Nav 真实数据验证：真实 MEMS IMU + 真实 RTK（`results/i2nav/`）

**数据**：武汉大学 i2Nav [awesome-gins-datasets](https://github.com/i2Nav-WHU/awesome-gins-datasets)，同一辆车上同步记录的 4 个 IMU：ICM20602（消费级）、ADIS16460、ADIS16465、i300（工业级 MEMS），真实 RTK 定位（1 Hz，水平 σ 中位数约 1 cm），真值来自高精度 POS 后处理。一次约 1617 s 的开阔路段行驶，最高车速约 13.6 m/s。原始数据不入库，`data/i2nav/*.npz` 已 gitignore。

**读取器**（`ccgins/data/i2nav.py`，`scripts/prepare_i2nav.py`）：
- 坐标系：机体 FRD→FLU（D=diag(1,−1,−1)），导航 NED→ENU，R_enu_flu = T·R_ned_frd·D；经纬高→ECEF→以起点为原点的 ENU。
- IMU 增量（Δθ、Δv）对累积量插值后重采样到 100 Hz；真值姿态用 SLERP，位置与速度线性插值；全部按 GPS 周内秒对齐。
- 每个 IMU 的天线杆臂（FRD）来自数据集说明，转为 FLU；GNSS 使用逐历元标准差（下限 5 cm）。
- 可选地球模型 `filters.set_earth()`：地球自转（姿态与 Coriolis）和当地重力（武汉约 9.7935 m/s²）。默认关闭，仿真与 KITTI 结果不变。
- 真实 IMU 的上电零偏很大，没有标定（例如 ICM20602 x 轴陀螺约 −1267°/h），因此初始零偏先验统一设为 2000°/h、20 mg。

**实验设计（时间留出）**：整段数据切成前后两半，中间留 10 s 隔离。A 折用后半段训练、在前半段测试，B 折反过来。
- MotionNet 从仿真模型初始化，只用 ICM20602 和 ADIS16460 的训练半段微调；训练半段最后 150 s 作验证集，用来估计 n_eff。数据增强加入随机常值零偏，共 12 个 epoch，每折约 22 s。
- ADIS16465 和 i300 **完全没有参与训练**。
- 测试半段使用真实 RTK，并人为注入中断（30/120/60/30/120/60 s，起点 50/70/90 s）。2 折 × 4 个 IMU × 3 个起点 × 2 个初始误差种子，共 48 次运行，每种中断时长 96 次。
- 统计：配对 bootstrap 95% CI、胜率、Wilcoxon 检验。
- 复现：`python scripts/train_motionnet_i2nav.py && python scripts/run_i2nav_experiment.py --workers 8 && python scripts/run_i2nav_spoof.py`。

**发现 1：原始 NHC 在真实数据上失效，原因是安装角。** 各 IMU 相对车体有 1.1–2.9° 的航向安装角和约 0.4° 的俯仰安装角，所以“侧向速度 = 0”的假设不成立：侧向残差 0.2–0.46 m/s，而 NHC 设定的 σ 是 0.1 m/s。
- **修正方法（不用真值）**：在训练半段运行不带 NHC 的 RTK/INS 滤波器，用滤波器自己估计的机体系速度方向求出安装俯仰角和航向角。各折估计结果一致：ICM20602 航向 1.09–1.17°，ADIS16460 2.76–2.82°，ADIS16465 −1.22 至 −1.29°，i300 2.85–2.87°，与真值推算的角度相差约 0.1°。
- 安装角作用于 NHC、混合和学习伪观测。在测试半段上，NHC 相比未标定版本的变化为 −6.4 / −14.5 / −11.3 m（30/60/120 s，p<1e-7）。

**发现 2：NHC 的 σ=0.1 在真实车辆上过度自信，原因是误差强时间相关。** 标定后侧向/垂向残差的 RMS 只有约 0.035–0.055 m/s，但 10 Hz 下 n_eff 约 47–137。
- 本仓库对学习伪观测已经做了相关性膨胀，这里用**同一套方法**处理 NHC：σ_eff = max_c RMS_c·√n_eff,c，同样只用训练半段估计，不用真值。各 IMU 得到 σ_eff ≈ 0.27–0.65 m/s。
- 效果：中断内 95% 覆盖率从 0.58 提高到 0.92，120 s 中断最大误差从 30.4 m 降到 20.6 m。

**中断结果（48 次运行均值±标准差，每种时长 n=96；完整表见 `results/i2nav/outage_table.md`）**

| 方法 | 水平RMSE[m] | 30 s | 60 s | 120 s | 中断内95%覆盖率 |
|---|---|---|---|---|---|
| ESKF 纯惯导 | 31.47±27.26 | 6.09 | 26.30 | 126.77 | 0.874 |
| InEKF+NHC（未标定安装角） | 13.65±7.90 | 9.01 | 31.92 | 41.68 | 0.232 |
| InEKF+NHC+安装角标定（基线） | 9.19±11.03 | 2.59 | 17.43 | 30.39 | 0.579 |
| InEKF+NHC+安装角+相关σ（更强基线） | 5.60±5.86 | 2.77 | 15.07 | 20.61 | 0.924 |
| InEKF+学习速度+相关膨胀[FT] | 8.77±4.00 | 7.44 | 15.28 | 25.70 | 0.422 |
| v1（全通道 ACI）[FT] | 15.00±24.63 | 8.06 | 19.06 | 41.94 | 0.629 |
| v3+安装角标定[FT] | 5.53±4.41 | 2.58 | 8.90 | 18.62 | 0.574 |
| **v3+安装角+相关σ（本文）[FT]** | **4.25±2.67** | 2.60 | **8.59** | **14.62** | 0.887 |
| v1 [仿真零样本] | 1172±1117 | 821 | 1481 | 3074 | 0.004 |
| v3+安装角 [仿真零样本] | 9.19±11.03 | 2.59 | 17.43 | 30.39 | 0.579（前向 100% 回退） |
| v3+安装角 [KITTI 模型零样本] | 9.14±11.04 | 2.59 | 17.08 | 30.20 | 0.583（回退 67.5%） |

**配对比较（各中断时长的最大误差，n=96；详见 `outage_paired.md`）**

| 比较 | 60 s 差值 (95% CI) | 120 s 差值 (95% CI) |
|---|---|---|
| v3+安装角+相关σ vs NHC+安装角 | −8.83 m [−16.80, −2.41]，−50.7%，胜率 0.56，p=0.043 | −15.78 m [−28.47, −5.61]，−51.9%，胜率 0.65，p=0.0018 |
| v3+安装角+相关σ vs NHC+安装角+相关σ | −6.48 m [−12.13, −1.87]，−43.0%，胜率 0.46，p=0.24 | −5.99 m [−13.19, −0.56]，−29.1%，胜率 0.54，p=0.23 |
| NHC+安装角+相关σ vs NHC+安装角 | −2.36 m [−5.17, +0.23]，p=0.0079 | −9.78 m [−17.32, −3.20]，p=0.00099 |

30 s 中断时各种 NHC 类方法之间没有实质差别（约 ±0.2 m）。

**按 IMU 拆开看（相对 NHC+安装角）。改进主要来自消费级 ICM20602：**
- ICM20602（参与训练）：v3+安装角+相关σ 在 60 s 中断下 −34.1 m（−71%），120 s 下 −60.1 m（−70%），p≤0.005。
- ADIS16460（参与训练）：60 s 下 −2.9 m（p=0.75，不显著）；120 s 下 **+1.45 m（+12%，略差，p=0.1）**。
- ADIS16465（未参与训练）：60 s 下 **+1.9 m（+35%，略差，p=0.97）**；120 s 下 −2.1 m（−19%，p=0.015）。不带“相关σ”的 v3 在 60 s 下更差（+2.9 m，+53%，p=0.06）。
- i300（未参与训练）：60 s 下 −0.2 m；120 s 下 −2.3 m（p=0.1）。
- 对两个工业级的未见过 IMU，单独的“NHC+安装角+相关σ”就已经拿到大部分收益：ADIS16465 在 60 s 和 120 s 下分别 −35% 和 −27%，i300 在 120 s 下 −17%，与 v3 相当甚至更好。**学习前向速度只在零偏最大的 ICM20602 上明显有用**；在工业级 IMU 上基本持平，个别情况略差。

**欺骗（真实 RTK + 叠加斜坡漂移，`results/i2nav/spoof_table.md`）**
- 底座为 v3+安装角+相关σ。在测试半段的第 300 s 开始注入 45° 方向的斜坡漂移；阈值取训练半段干净运行统计量最大值的 1.5 倍；共 16 次测试。

| 漂移速率 | 单通道 NIS-CUSUM：检出率 / 延迟 / 欺骗后最大误差 | 双通道 DR-CUSUM：检出率 / 延迟 / 欺骗后最大误差 | 不监测 |
|---|---|---|---|
| 0.2 m/s | 50% / 38.5 s / 81.2 m | 50% / 273.1 s / 84.7 m | 100.5 m |
| 0.5 m/s | 100% / 3.2 s / 45.5 m | 100% / 89.0 s / 132.9 m | 251.4 m |
| 1.0 m/s | 100% / 2.0 s / 45.5 m | 100% / 17.5 s / 196.9 m | 502.7 m |

- 两种监测在干净测试数据上的误报都是 0/16。
- **在 RTK 精度下，本文的 DR-CUSUM 不如单通道 NIS-CUSUM**，这与仿真中的结论（GNSS σ=1 m）相反。原因：RTK 的厘米级新息让 NIS 对漂移极其敏感；而真实 MEMS 的航位推算通道 B 在 30 s 内会漂移若干米，使干净数据上的统计量最大值很大（ICM20602 约 8000–10000），全局阈值被抬到 15422。
- 尝试过的修正：改为按 IMU×折分别标定阈值。结果在留出的测试半段上误报升到 7/16（单通道 6/16），因此**未采用**。
- “欺骗后最大误差”在检测到之后基本由剩余约 500 s 没有 GNSS 的推算误差决定，因为报警会锁存，不会恢复使用 GNSS。

图：`results/i2nav/outage_error.png`（ADIS16465，A 折，起点 50 s 的误差曲线）；`results/i2nav/per_imu_summary.png`（各 IMU 在 120 s 中断下的最大误差与覆盖率）。

### 图

- `results/outage_error.png`：种子 900 的水平误差，以及误差与 σ 之比（灰色区域为 GNSS 中断）
- `results/outage_consistency.png`：各方法 120 s 中断的最大误差，以及中断内的 95% 覆盖率
- `results/outage_shift_error.png`、`results/outage_shift_consistency.png`：分布偏移下的对应图
- `results/spoof_stats.png`：欺骗场景下的检验统计量、CUSUM 和水平误差（v3 底座）
- `results/i2nav/outage_error.png`、`results/i2nav/per_imu_summary.png`：i2Nav 真实数据（第 6 节）
- `results/kitti/outage_error.png`：KITTI 测试段 0028#0 的水平误差（灰色为中断；黑色虚线为 InEKF+NHC，与“v3 仿真零样本”重合）

## 结论（如实陈述）

1. **相关性膨胀是关键修正。** 学习速度按白噪声处理会严重过度自信（仿真 ANEES 13.7、中断内覆盖率 0.12；KITTI 上 30 s 中断最大误差 100 m）。按 n_eff 放大方差后，仿真标称下 RMSE 2.51 m（NHC 3.34 m），120 s 中断 10.66 m（NHC 14.50 m）。
2. **仿真标称下最好的是“混合+相关膨胀”**（RMSE 2.37 m，120 s 中断 9.77 m，中断内覆盖率 0.861 与 NHC 持平）。但它在分布偏移下崩溃（120 s 中断 163 m）。
3. **ACI 改进的过程（含失败）**：
   - v1（全通道 ACI）在偏移下 120 s 中断 38.5 m，仍远差于 NHC 17.8 m。
   - v2（混合 + 快速 ACI + κ 回退）在验证种子上看起来可以，但在**测试偏移种子上失败**（120 s 中断 61.4±89.9 m，种子 905、909 灾难性发散）。根因：偏移下网络前向速度出现 +2–6 m/s 的系统偏差，而残差方差并不异常，ACI 的 κ 察觉不到。
   - v3 增加“前向残差偏差监测 + 锁存回退”。在偏移测试集上 v3 **与 NHC 完全相同**（RMSE 4.12 m，120 s 中断 17.84 m），因为它在所有种子上都关闭了前向通道（回退比例 1.0）。即：v3 达到了“至少不差于 NHC”的目标，但**并没有超过 NHC**；不锁存的版本会在短暂低偏差时重新启用前向通道，中断时发散，效果更差。
   - 代价：仿真标称下 v3（2.62 m / 11.63 m）略差于不带保护的混合方案（2.37 m / 9.77 m），但仍优于 NHC。
4. **KITTI 真实数据**：微调后的 v3 相比 NHC 有**小幅但一致**的改进（RMSE 2.83 vs 2.92 m；120 s 中断 12.37 vs 13.08 m，10/10 配对更好）。不带保护的学习方法在真实数据上都不可靠（例如“学习速度+相关膨胀” RMSE 7.78 m，主要坏在高速段）。仅用仿真训练的网络直接用于 KITTI 时，v3 全部回退，等于 NHC，这正是保护机制的作用；v1/v2 零样本则明显差于 NHC。
5. **一致性未达标。** 仿真中带伪观测的方法（白噪声方案除外）全程覆盖率约 0.62–0.73；KITTI 上中断内覆盖率约 0.67（包括 NHC），说明 NHC 的 σ=0.1 m/s 对真实车辆偏紧（真实侧向/垂向机体速度 RMS 约 0.08–0.2 m/s）。
6. **InEKF 与 ESKF** 在两类数据上精度几乎相同（良好初始化下）。
7. **欺骗检测**（v3 底座，仿真）：双通道 DR-CUSUM 在 0.2 m/s 和 1.0 m/s 漂移下检测更快（53.4 vs 85.9 s，11.6 vs 18.0 s），0.5 m/s 时相当（22.8 vs 26.6 s）；两者误报为 0；欺骗后最大误差比不监测下降约 2.5–14 倍。**但在 i2Nav 真实 RTK 上结论相反**：单通道 NIS-CUSUM 更快，例如 0.5 m/s 时延迟 3.2 s，而 DR-CUSUM 为 89 s（见第 6 节）。
8. **i2Nav 真实 MEMS + RTK**：
   - 真实数据中最关键的两点：(a) 不用真值的安装角标定；(b) 对 NHC 也做相关性膨胀（σ_eff）。两者合起来，120 s 中断从 41.7 m（原始 NHC）降到 20.6 m，覆盖率从 0.23 升到 0.92。
   - 在此基础上，微调的 v3 再降到 14.6 m（相对 NHC+安装角 −52%，p=0.0018；相对更强的 NHC+安装角+相关σ −29%，p=0.23，不显著）。
   - 改进主要来自消费级 ICM20602。在未见过的工业级 IMU 上，v3 与更强基线基本持平。
   - 仿真零样本网络直接用在真实数据上会灾难性失败（v1 误差达千米级），v3 的保护机制把它全部回退为 NHC。

## 局限性

- **KITTI 的 GNSS 是合成的**，IMU 为 OXTS（高等级）再注入 MEMS 误差，并非真实的 MEMS IMU + 真实 GNSS 接收机；真实多路径、遮挡、欺骗未覆盖。
- **KITTI 测试规模小**：只有 4 个测试段、3 个 drive；60 s 中断只出现在 1 段（5 个样本），120 s 中断只出现在 2 段（10 个样本）。改进幅度（约 0.1–0.7 m）应视为初步结论。
- 真实数据上 MotionNet 前向 RMSE 约 2 m/s，训练数据只有 928 s；在高速等训练分布外场景网络不可用（v3 只能回退）。
- OXTS 真值本身有质量问题（drive 0028 有 30 m 级跳变），已用跳变检测剔除，但可能仍有残余误差。
- 仿真器假设振动幅度随车速增大，使前向速度在仿真中更可观；KITTI 结果证实真实情况要差得多。
- v3 的偏差监测在 GNSS 健康期才能工作；若一开始就没有 GNSS，则一直使用 NHC。锁存后不会恢复。
- **ZUPT/里程计已接入批量主流程，默认关闭**：ZUPT 振动标定在留出半段冻结，速度门限依赖滤波速度，默认还需真实 GNSS 位移确认停车；因此中断中新发生的停车无法确认，漂移较大时也可能漏掉真停车。车辆起步到下一次 GNSS 之间仍有因果延迟风险；未验证独立道路泛化。里程计支持显式安装旋转/杆臂与地球自转修正，但 i2Nav 没有轮速流，尚无实测轮速精度结论；滑移、比例因子、时间偏差和几何不确定性未估计。流式接口也支持这两种更新（默认关闭），以及固定安装角 NHC。
- **i2Nav 只有一次约 1617 s 的开阔路段行驶**，最高车速约 13.6 m/s。留出方式是同一次行驶内的时间切分，4 个 IMU 共享同一条轨迹，并非独立场景，因此跨场景泛化（城市峡谷、高速、不同车辆）未验证。
- **i2Nav-Robot 数据集未使用**：数据托管在 OneDrive，访问未获批准。KF-GINS 示例数据是 Leador-A15（战术级，非 MEMS），因此也未使用。
- i2Nav 上 MotionNet 的前向速度 RMSE 仍有约 1.2–1.4 m/s；在 ADIS16465（安装航向角方向与训练 IMU 相反）上侧向 RMSE 为 0.38 m/s。仿真零样本网络的前向 RMSE 约 44 m/s，完全不可用。
- i2Nav 欺骗阈值用训练半段的干净运行标定，其中 v3 前向通道使用的是该网络在训练数据上的预测（样本内），可能偏乐观；而且 DR-CUSUM 在 RTK 上本身就不占优。
- 安装角在训练半段估计一次后固定，没有在线估计；σ_eff 取两个通道中的较大者，为全局常数。
- 安装角与地球自转目前只在 i2Nav 流程中使用（离线标定 / 可选开关），未作为滤波状态；也未建模杆臂误差状态和 GNSS 有色噪声。
- 双通道 B 由 A 周期性重置，存在“极慢漂移”盲区（实验最低速率 0.2 m/s）；CUSUM 报警为锁存式，无恢复逻辑。
- 仿真每个测试只用 10 个种子，标准差较大。
- 单步耗时为 Python 实现的数值（约 40–110 µs/步，100 Hz IMU），不代表嵌入式 C 实现。

## 目录

```
ccgins/      核心库（geometry, sim, filters, zupt, adaptive, stream, cli, aci, integrity, learning, pipeline, metrics, scenarios, kitti_scenarios, i2nav_scenarios, data/kitti, data/i2nav）
scripts/     训练与实验脚本
tests/       pytest 单元测试
results/     模型、指标 JSON/MD、图（results/kitti/ 为 KITTI 结果，results/i2nav/ 为 i2Nav 结果；模型与图已纳入版本库）
data/        KITTI / i2Nav 预处理数据（不入库）
docs/        调研与方案文档
```

### ADIS16465 基线伪距实验

`OPENBLAS_NUM_THREADS=1 python -m ccgins.range_experiment` 运行实测 ADIS16465 + RTK，
将 RTK 相对固定 ENU 锚点的基线长度作为单个伪距，并通过几何雅可比传播 RTK 协方差。
结果位于 `results/range_coupling/`。注入项是沿机体前向缓慢增长的天线杆臂误差；
对照使用相同 IMU 与初值。标量残差只能检测不一致，不能唯一归因于杆臂，也不能独立约束切向位置。
这一步没有原始卫星观测、接收机钟差或多星几何，不代表完整 GNSS 紧组合。

### 自适应 ZUPT 与显式几何里程计（批量与流式）

`Config(zupt=True)` 在 `pipeline.run` 中启用零速更新；默认仅用噪声密度地板，
实车应先用独立标定 IMU 的 `AdaptiveZeroVelocity.calibrate(gyro, accel, spec, dt)`，
将其 `accel_var` / `gyro_var` 写入 `Config.zupt_accel_var` / `zupt_gyro_var`。
标定取 1 s 窗方差下十分位与噪声地板的较大者，乘 4，分别封顶于
3² (m/s²)² 与 0.05² (rad/s)²（车辆振动配置；更低阈值的保守试验也保留）。检测采用过去 0.5 s 的数据，连续满足 1 s 后
才允许 10 Hz、σ=0.05 m/s 的零速观测。原始陀螺均值 <0.02 rad/s、
重力模长误差 <0.25 m/s²、滤波速度 <0.5 m/s；χ²(3,0.999) 门控不强制重捕获。
默认 `zupt_require_gnss_stop=True`：连续两次被接受的真实 GNSS 位置对满足
`(位移模长 + 2√λmax(R₁+R₂))/Δt < 0.25 m/s`（历元间隔 ≤2 s）才确认停车。
任何 IMU 持续条件中断，或任何表明运动的有效 GNSS 对（即使被滤波器拒绝）都会撤销确认。
确认停车可在 IMU 持续静止时延续到中断内；默认 GNSS-only 保护无法确认中断中新停车。可显式启用 `zupt_odometry_stop=True`，使用独立有效轮速的零速证据（详见下节）。
仅在有独立外部停车保证时才能关闭此保护。未保护的车辆振动配置实测曾因误检导致严重发散，
完整失败结果保留于 `results/adaptive_motion_vehicle/`，低阈值无更新结果保留于
`results/adaptive_motion_conservative/`。这些开发试验不是盲测或泛化验证。
这不是仅靠 IMU 就能识别匀速运动的保证，尤其不能把滤波速度当独立传感器。

外部轮速使用 `run(..., odometry=OdometryData(t, speed, sigma, C_vb, lever_b, valid))`：
有符号前向 m/s，显式 IMU FLU → 车辆 FLU 旋转 `C_vb`，IMU 到轮速测点的 FLU 米制杆臂
`lever_b`（零杆臂也须显式给出）。观测为 `C_vb (Rᵀv + (gyro−bg−Rᵀω_ie)×lever_b)`
的前向分量，含零偏与姿态雅可比、标量 NIS 门控。时间戳须严格递增并落在已推进的 IMU 网格，
不插值、不保持旧值；`valid=False` 用于排除已知滑移。日志 `extra["motion"]` 记录尝试/接受计数与零速更新时间。

实测配对协议、逐次结果、所有改善和退化见
[results/adaptive_motion/report.md](results/adaptive_motion/report.md)，机器指标为同目录 `metrics.json`。
复现：`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.motion_experiment --workers 4`。


此前自适应 ZUPT 批次实际运行 **192 条轨迹 / 96 组配对**：4 IMU × 2 时间折 × ESKF/InEKF ×
无中断/六段中断 × 有/无更新 × 3 个保留配置；每配置有 96 个配对中断事件。
使用初值种子 0，基线为安装角标定 + 相关性膨胀 NHC，启用地球补偿和真实 RTK 历元 σ。
全部原 73 测试与新增 10 测试通过：**83 passed in 32.45s**。

| 配置 | 无中断全程水平 RMSE，基线→更新 [m] | 中断场景全程水平 RMSE，基线→更新 [m] | 中断场景改善/相同/退化组数 |
|---|---:|---:|---:|
| 低阈值保守试验（未接受任何零速更新） | 0.0282→0.0282 | 5.9700→5.9700 | 0/16/0 |
| 未加 GNSS 停车确认的振动配置（失败消融） | 0.0282→2.4824 | 5.9700→41.8525 | 10/0/6 |
| **默认：振动标定 + GNSS 停车确认** | **0.0282→0.0275** | **5.9700→5.9694** | **8/7/1** |

默认配置共接受 4592 次零速更新，参考速度 >0.5 m/s 时接受 0 次；这不保证更慢蠕行或其他道路上的安全性。
默认配置的中断场景平均 RMSE：ICM20602 14.0610→14.0610 m，ADIS16460 4.0023→3.9998 m，
ADIS16465 3.2207→3.2207 m，i300 2.5962→2.5962 m。
60 s 中断平均最大水平误差 18.2851→18.2798 m（32 事件，2 改善/28 相同/2 退化）；
30/120 s 对应 4.5080/23.1857 m，基本不变。
**保守默认没有带来实质中断性能突破**；它避免了本次未保护配置的误零速发散，但无法确认中断中新出现的停车。
所有配置为同一条行驶的开发试验，GNSS 保护是在观察到失败后加入，不应视为独立盲测。
完整配置、逐 IMU 一致性、全部中断最大/末端误差、失败和复现命令见
[批次报告](results/adaptive_motion/batch_summary.md)；里程计仅完成集成与控制测试，未声称实测轮速性能。

### 流式 motion 更新与 SYNTHETIC 轮速中断试验

`StreamingNavigator.from_truth(..., cfg)` 现在读取全部 ZUPT 标定方差、重力、
持续时间/速度防护和更新频率；直接构造时传 `motion_cfg=cfg, imu_spec=spec, dt=dt`。
外部轮速用 `nav.push_odometry(t, speed, sigma, C_vb, lever_b, valid=True)`，
必须在相应 `push_imu(t, ...)` **之前**入队，可预先缓存；不接受重复/过去历元，
不插值或保持轮速。安装矩阵、测点杆臂、地球旋转修正与批量共用同一观测模型。
`nav.motion` 对应批量 `log.extra['motion']`。

```python
cfg = Config(zupt=True, zupt_accel_var=cal.accel_var,
             zupt_gyro_var=cal.gyro_var, zupt_odometry_stop=True)
nav = StreamingNavigator.from_truth(truth, imu_spec, gnss_spec, cfg)
# 每次先缓存该历元的轮速，再推进 IMU；GNSS 可提前缓存或在同历元 IMU 后喂入。
nav.push_odometry(t, wheel_speed, wheel_sigma, C_vb, lever_b, valid=not slipping)
nav.push_imu(t, accel, gyro)
```

可选的独立轮速停车保护要求连续两次**被滤波器接受**的轮速满足
`abs(speed)+3*sigma < 0.1 m/s`，相邻间隔 ≤0.2 s，证据在 0.2 s 后过期。
它只替代 GNSS 停车确认，仍须满足 IMU 因果检测、滤波速度与 ZUPT 门控。
轮速更新在该历元 ZUPT **之后**，所以证据最早用于下一历元，避免同历元循环自证。
必须由调用方保证车辆前向零速可作为停车证据、安装/方差已标定，并排除滑移；横向运动无法靠单轴轮速排除。
默认配置和旧批量行为保持不变。

有意的流式差异：ZUPT 要求固定 IMU 采样间隔，检测只看已经传播的前一采样；
传播 → ZUPT → 轮速 → 固定 NHC → 同历元 GNSS 的顺序与批量一致。离网格 GNSS 可以切分传播，
但批量 API 本身按 IMU 索引工作，仅网格输入承诺一致性。轮速必须位于传播后的 IMU 网格，
起始 IMU 历元不应用轮速。流式可在起始历元应用 GNSS；批量循环从下一历元开始，
因此做一致性回放应排除起始 GNSS。流式 GNSS 接收显式协方差，真实数据回放须由调用方
应用与批量相同的历元 σ 与地板。已过期数据不能回溯；迟到 GNSS 的拒绝语义保持不变。

固定网格一致性：`from_truth` 使用与批量相同的 nominal `truth.dt` 传播，避免浮点时间戳相减误差在长中断中放大；离网格 GNSS 切分仍使用实际子区间。NHC 与批量相同，仅在 `WINDOW` 后按 `pseudo_every` 应用（安装旋转、固定方差均保留）。默认 GNSS 停车保护与旧 NHC 基线未修改。

**SYNTHETIC（参考速度生成，非实测轮速）**：i2Nav 没有真实轮速日志。
本次仅用参考姿态/速度生成车体前向速度，研究外部零速证据能否解除中断停车确认瓶颈。
10 Hz，1% 比例偏差，独立高斯噪声 σ=`0.01+0.02*abs(v_forward)` m/s；
每 90 s 的第 40–45 s 加 10% 滑移并已知标为无效（乐观假设），随机种子 2026。
安装矩阵来自既有对半标定，合成轮速测点选在 IMU 中心，显式零杆臂。
测试参考信息直接进入合成观测，因此**不构成真实里程计精度或独立验证证据**。

4 IMU × 2 时间折 × InEKF/初值种子 0，**8 组匹配案例、24 条批量轨迹 + 8 条完整流式回放**，
共 48 个配对中断事件；仅使用现有数据。为匹配流式功能，本次不加 NHC，基线与此前 NHC 试验不同。
所有结果（含退化）、逐中断最大/末端误差、方差/数据哈希与复现命令见
[stream_motion/report.md](results/stream_motion/report.md) 和
[metrics.json](results/stream_motion/metrics.json)。

| 配置 | 各案例水平 RMSE 均值 [m] | 95% 水平覆盖率 | ANEES/3 | 中断内接受 ZUPT |
|---|---:|---:|---:|---:|
| GNSS 保护 ZUPT 基线（无 NHC） | 71.0856 | 0.7615 | 6.7861 | 0 |
| 基线 + SYNTHETIC 轮速 | 6.4053 | 0.4691 | 9.6400 | 0 |
| 同轮速 + 轮速零值停车确认 | 6.4273 | 0.4692 | 9.6991 | 1194 |

相对 GNSS 保护基线，轮速+确认的平均中断最大水平误差 **104.0914→14.6001 m**、
末端 **104.0914→8.8398 m**；最大误差 34 改善 / 0 相同 / **14 退化**。
主要收益来自连续合成速度，而不是停车确认，且覆盖率显著下降、过度自信加重。
隔离停车确认本身：相同合成轮速下平均最大误差 **14.7941→14.6001 m**，
末端 **8.8608→8.8398 m**（最大误差 12 改善 / 29 相同 / **7 退化**）。
全程平均 RMSE 反而 **6.4053→6.4273 m**；逐案例 6 改善 / 1 相同 / 1 退化，
ADIS16460/B **4.5441→5.4187 m**。这不是普遍精度改进。

参考速度模长 <0.1 m/s 且持续 ≥1 s、起点位于中断内的停车共 12 个传感器/折事件
（同一次行驶的 3 个实际停车，被 4 个共载传感器重复观察，并非独立样本）。
至少接受一次停车期间 ZUPT 的事件数从 **0→9/12**；ICM20602 两折仍未接受中断零速更新。
全部 ZUPT 中参考速度 >0.5 m/s 的接受数为 0，但这不能排除蠕行或未知横滑风险。
此前批次流式/批量全轨迹最大位置差 **6.93×10⁻⁷ m**，运动计数完全一致；速度/协方差差异见 JSON。

此前批次验证：**96 passed in 38.12s**（全部此前 83 测试 + 13 个新测试），
覆盖两类滤波器、非零安装/杆臂与地球旋转、位置/速度/协方差一致性、
中断停车确认、GNSS 拒绝运动证据仍撤销停车，以及轮速时序/有效性防护。
剩余限制：没有真实轮速精度评估；滑移标识假定已知，几何/同步/比例不确定性未估计，
默认 GNSS-only 保护仍无法确认中断中新停车；大漂移后的滤波速度防护和陀螺零偏仍会漏检。

### 主报告配置的流式/批量一致性复核

[完整报告](results/stream_batch_parity/report.md) / [逐配置结果](results/stream_batch_parity/metrics.json)：
冻结此前报告设置，运行 **88 组完整 batch + stream 配对 / 706,552 个日志历元**：
64 个 guarded motion 配置（4 IMU × 2 折 × 2 滤波器 × clean/outage × baseline/ZUPT），
加上 24 个 **NO-NHC、SYNTHETIC 轮速**实验配置。每第 10 个 IMU 历元检查位置、速度、
导航系位置协方差及全部 15×15 内部误差协方差（含速度/位置及其交叉块）；
所有最大绝对差均为 **0**，运动计数/接受时刻完全相同，容差为 `atol=1e-12, rtol=0`。
此前 batch RMSE、Coverage95、ANEES 数字完全不变，未修改默认 GNSS 停车确认或 NHC 基线。

修复了流式静默忽略 mounted NHC，以及固定网格浮点时间差与批量 nominal dt 不一致的问题。
离网格 GNSS 切分、起始历元 GNSS 等有意差异仍显式保留；流式不支持的批量选项会报错。
新增 16 个测试，最终完整验证 **112 passed in 42.98s**，此前 96 个全部通过。
这只证明数值一致，不修复 SYNTHETIC 轮速导致的覆盖率/ANEES 退化；
没有新增独立道路或真实轮速证据，也不主张轮速零值停车辅助具有普遍收益。

复现：`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m ccgins.parity_experiment --workers 4`
