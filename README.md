# cc-gins：一致性校准的学习辅助 GNSS/INS 组合导航

**cc-gins**（Consistency-Calibrated GNSS/INS）是一个可运行、可测试的松组合导航原型。它解决的问题是：在 GNSS 中断时，用学习得到的车辆运动伪观测来约束惯导漂移，同时保证滤波器给出的协方差仍然“可信”；在 GNSS 被慢漂移欺骗时，能够检测出来并把它剔除。

> ⚠️ 本仓库的数值结果全部由下文脚本实际运行得到，分三部分：(a) **仿真数据**（`ccgins/sim.py`）；(b) **KITTI raw 真实 OXTS 数据**（真实 IMU 与真值轨迹，但 GNSS 为合成：1 Hz、带中断/欺骗；并在 OXTS IMU 上注入了 MEMS 级误差）；(c) **i2Nav awesome-gins 真实数据**（真实 MEMS IMU×4 + 真实 RTK + 真值，第 6 节）。

调研文档位于 `docs/`：`literature_review.md`（文献综述）、`improvement_opportunities.md`（可改进点）、`project_proposal.md`（方案）。

## 已实现内容

| 模块 | 文件 | 说明 |
|---|---|---|
| SO(3) 工具 | `ccgins/geometry.py` | Exp/Log、左雅可比、欧拉角 |
| 仿真器 | `ccgins/sim.py` | 车辆轨迹（含停车与转弯，侧偏角）；MEMS IMU（零偏、零偏随机游走、ARW/VRW、随速度增大的振动）；1 Hz GNSS（多路径 Gauss-Markov 误差、杆臂、可配置中断、斜坡式慢漂移欺骗） |
| 惯导机械编排 | `ccgins/filters.py::mechanize` | 采用 ENU 导航系与 FLU 机体系，不考虑地球自转（局部、短时假设） |
| **稳健 GNSS 更新** | `NavFilter.update_gnss` | ESKF/InEKF 共用白化新息 Huber 权重与协方差膨胀；`Config.robust_gnss_delta=1.5` 启用，默认关闭 |
| ESKF 基线 | `ccgins/filters.py::ESKF` | 15 维误差状态，使用 Joseph 形式更新，支持 χ² 门限 |
| InEKF 松组合 | `ccgins/filters.py::InEKF` | 右不变 SE₂(3) 误差加可加性零偏，GNSS 观测带杆臂 |
| **零速检测 / ZUPT / 里程计** | `ccgins/zupt.py` | 陀螺/加速度滑窗方差阈值检测静止段（可设最短持续时间）；提供导航系零速 ZUPT 与机体系里程计速度更新，复用 `NavFilter.update` 的 Joseph 形式更新和 χ² 门限 |
| **实时流式封装** | `ccgins/stream.py` | `StreamingNavigator`：`push_imu` / `push_gnss` 增量处理，乱序 GNSS 缓存后按时间戳插入，`get_state` 返回位置、速度、姿态与协方差；复用现有 ESKF/InEKF |
| **自适应 GNSS 量测噪声 R** | `ccgins/adaptive.py` | 基于新息的 Sage-Husa 滑窗估计（遗忘因子加权），按 `[min_scale, max_scale] × R_nominal` 逐轴夹紧；`Config.adaptive_r=False`（默认）时不启用 |
| **配置驱动 CLI** | `ccgins/cli.py` | `ccgins run --config … --out …`（或 `python -m ccgins.cli`）跑仿真/中断场景，写出 `report.md` 与 `metrics.json`；示例配置 `configs/example_sim.yaml` |
| NHC 基线 | `pipeline.py`（`pseudo="nhc"`） | 侧向和垂向速度为 0，σ 固定为 0.1 m/s |
| 学习速度伪观测 | `ccgins/learning.py` | 1D-CNN（MotionNet）：输入 1 s 原始 IMU（100 个采样），输出机体系速度（前、左、上）的均值和方差，用 β-NLL 训练 |
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
python scripts/train_motionnet.py                     # 训练 MotionNet（约 1 分钟），生成 results/motionnet.pt 和 motionnet_val.json（含 n_eff）
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

滤波器推进时会按时间顺序应用缓冲的 GNSS；若 GNSS 历元落在两个 IMU 采样之间，会在该时刻切分积分区间，因此结果与 `pipeline.run` 的批处理顺序一致。`tests/test_stream.py` 验证了同一仿真数据上的批处理一致性（位置/速度/位置协方差）以及乱序 GNSS 的等价性。流式接口只封装 IMU+GNSS；伪观测、ACI、完整性监测与自适应 R 仍使用批量 `pipeline.run` 的选项。已经推进过去的 GNSS 历元无法回溯，会抛出 `ValueError`。

### 自适应 GNSS 量测噪声 R（可选，默认关闭）

`ccgins/adaptive.py` 提供 `AdaptiveR`：利用 GNSS 位置新息 `r = z − h` 的协方差 `S = H P Hᵀ + R`，在滑窗内对 `r rᵀ − H P Hᵀ` 做遗忘因子加权平均，得到逐轴 R 估计，并夹紧到 `[min_scale, max_scale] × R_nominal`，避免异常新息把 R 推到 0 或无穷。`gnss_update(..., estimator=None)` 与原有 `nav.update` 完全一致；主流程仅在 `Config(adaptive_r=True)` 时于 GNSS 更新前调用它，因此所有已发表结果不受影响。`tests/test_adaptive.py` 验证了 4 倍 R 噪声下的收敛与上下限夹紧。

### 配置驱动的命令行仿真（`ccgins.cli`）

`ccgins/cli.py` 读取 YAML（也接受 JSON）配置，运行一次仿真/中断场景，并把结果写入 `--out`：

```bash
python -m ccgins.cli run --config configs/example_sim.yaml --out results/cli_demo
# 安装后等价的 console script：
ccgins run --config configs/example_sim.yaml --out results/cli_demo
```

配置段包括 `scenario`、`imu`、`gnss`（含 `outages`）、`filter`（任意 `Config` 字段）、可选 `learning.model`，以及 `metrics.t_start`。`--seed` / `--duration` 可覆盖配置。输出：
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
- **ZUPT/里程计目前是独立模块**：静止检测使用固定滑窗方差阈值，未按 IMU 噪声或车辆振动自适应，也未接入 `pipeline.py` 的主实验；里程计速度更新假设无杆臂/安装角误差，且未建模轮速滑移。
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
