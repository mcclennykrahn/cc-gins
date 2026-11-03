# 项目方案：CC-GINS —— 一致性校准的学习辅助 GNSS/INS 组合导航

> **Consistency-Calibrated GNSS/INS**：在不变 EKF 松组合框架中，引入①学习型车体伪速度/NHC 伪观测，②基于自适应共形推断的在线一致性校准，③双通道 GNSS 完好性/欺骗监测。
> 运行环境：本机（这台 Linux box）本地 Python 环境，笔记本级 CPU 即可，不依赖 GPU/ROS。
> 建议仓库名：`cc-gins`（私有仓库，由主代理在获得批准后创建）。文献编号见 `literature_review.md`。

---

## 1. 要解决的问题

低成本 MEMS GNSS/INS 在城市里有两类失效：
1. **GNSS 中断**（隧道、高架、峡谷）：纯 INS 位置误差按 t²~t³ 发散，30 s 内可达几十米。
2. **GNSS 异常**（NLOS 跳变、多路径、缓慢拉偏式欺骗）：滤波器被错误观测带偏。

近年主流做法是用神经网络给出伪观测（E1–E5、F1、F3、F4）或噪声协方差（D1–D5）。**共同缺陷是网络输出的方差没有经验校准**：过度自信时滤波器被锁死在错误伪观测上，过度保守时又起不到作用，而且 P 阵不可信，完好性（保护级）无从谈起。

## 2. 创新点与已有工作对比

| 已有工作 | 做了什么 | 缺什么 | CC-GINS 的做法 |
|---|---|---|---|
| AI-IMU (F1)、NHC-LIO (F3)、Deep-inertial IEKF (F4) | 网络预测 NHC/速度伪观测及其协方差 | 协方差没有校准；F1 无 GNSS | 协方差经**在线共形校准**，覆盖率可验证 |
| CTGANN (E3)、Transformer/LSTM 桥接 (E1、E2、E4) | 中断期预测伪速度/伪位置 | 固定或启发式 R；离线训练 | 伪速度 + 校准方差，GNSS 可用时**自监督在线校准** |
| GANP 伪 GNSS (E5) | 神经过程预测协方差直接当 R | 无经验覆盖率保证 | 用 ACI 让 NIS 覆盖率收敛到名义值 |
| VB / Student-t 自适应滤波 (C1–C4) | 在线估计 R 并抗差 | 中断期无新息，自适应失效；超参敏感 | GNSS 侧同样用共形 NIS 校准 R，加上 Huber/Student-t 抗差 |
| CQKF / 共形 KF（ICASSP 2026；arXiv 2609.27506） | 校准滤波器**输出**置信区间 | 不处理输入伪观测，不影响状态估计 | 校准**进入更新的观测协方差**，改变的是估计本身 |
| 新息检验类欺骗检测 (G3、G4) | 单通道新息/多普勒 RAIM | 缓慢拉偏会被滤波器跟随 | **独立 DR 通道 + CUSUM**，且 DR 协方差已校准，阈值有统计意义 |

**一句话创新**：把自适应共形推断（ACI，Gibbs & Candès, NeurIPS 2021，https://arxiv.org/abs/2106.00170 ）引入不变 EKF 的**观测侧**，以 GNSS 为"老师"在线校准学习型车辆运动伪观测的不确定度，使中断桥接与完好性监测都建立在**统计一致**的协方差上。

预期可发表、可量化的结论：
- 相对未校准的学习伪观测（AI-IMU 式），中断期最大误差降低，且 ANEES 接近 1、95% 覆盖率接近 95%。
- 相对固定 NHC 和 VB 自适应，中断期误差更小。
- 对斜坡式欺骗，检测时延比单通道新息检验更短。

## 3. 算法

### 3.1 状态与坐标系
- 导航系 n：局部 ENU（以首个 GNSS 点为原点）；首期忽略地球自转（MEMS 级可接受），预留 `earth_rate` 开关。
- 主状态 X ∈ SE₂(3)：

  X = [[R, v, p], [0, 1, 0], [0, 0, 1]]，R ∈ SO(3)（b→n），v、p ∈ ℝ³

- 附加状态（欧氏空间，"不完美 InEKF"）：陀螺零偏 b_g、加计零偏 b_a、IMU→车体安装角小量 δm = [δroll, δpitch, δyaw]，可选里程计比例 s。
- 误差状态 ξ = [ξ_R, ξ_v, ξ_p, δb_g, δb_a, δm] ∈ ℝ¹⁸，使用**右不变误差** X = Exp(ξ_X)·X̂。

### 3.2 IMU 传播（100–200 Hz）
ω̄ = ω_m − b_g，ā = a_m − b_a：

- R_{k+1} = R_k Exp(ω̄Δt)
- v_{k+1} = v_k + (R_k ā + g)Δt
- p_{k+1} = p_k + v_kΔt + ½(R_k ā + g)Δt²

右不变误差线性化 ξ̇ = A ξ + G w，其中（3×3 分块，(·)^× 为反对称矩阵）：

```
       ξ_R    ξ_v   ξ_p    δb_g      δb_a   δm
ξ_R [   0      0     0     -R̂         0      0 ]
ξ_v [  g^×     0     0     -v̂^× R̂    -R̂     0 ]
ξ_p [   0      I     0     -p̂^× R̂     0      0 ]
δb_g[   0      0     0  -I/τ_g        0      0 ]
δb_a[   0      0     0      0      -I/τ_a    0 ]
δm  [   0      0     0      0         0      0 ]
```

零偏取一阶 Gauss-Markov（τ 来自 Allan 方差或厂商手册）。离散化：Φ ≈ I + AΔt + ½(AΔt)²，Q_d ≈ Φ G Q_c Gᵀ Φᵀ Δt。噪声参数 ARW、VRW、零偏不稳定性放在 `configs/imu/*.yaml`。

同时实现**ESKF 对照版**（误差 δθ 定义在 R = R̂ Exp(δθ)，标准 15/18 维），共享传播代码，只换误差定义和雅可比。

### 3.3 GNSS 位置/速度观测（1–10 Hz，松组合）
杆臂 l_b：y_p = p + R l_b + n_p

- 残差 r = y_p − (p̂ + R̂ l_b)
- 右不变雅可比：H_p = [ −(p̂ + R̂ l_b)^×, 0, I, 0, 0, 0 ]
- GNSS 速度（可选）：y_v = v + R(ω̄ × l_b)，H_v ≈ [ −v̂^×, I, 0, …]

**抗差更新**：
1. 快照门限：NIS_k = rᵀS⁻¹r，S = HPHᵀ + R̃_gnss；当 NIS_k > χ²₃(0.999) 时拒绝，并计入完好性统计。
2. 中等残差用 Huber/Student-t 等效权重：R̃ ← R̃ / w_k，w_k = (ν+3)/(ν+NIS_k)，ν 默认 5（参考 C1–C4，用单次迭代近似 VB，避免多次迭代）。
3. R̃_gnss = κ_g · R_nominal，其中 κ_g 由 §3.5 的共形校准在线更新（替代 Sage-Husa）。

更新使用 Joseph 形式；右不变注入 X̂⁺ = Exp(δξ_X) X̂，附加状态直接相加。

### 3.4 学习型车辆运动伪观测（10 Hz）
**观测模型**（车体系 v_b = [前, 左, 上]）：

v_b = C_m(δm) · Rᵀ v，C_m = Exp(δm) C̄_m

右不变误差下 Rᵀv ≈ R̂ᵀ(v̂ + ξ_v)（ξ_R 项相互抵消，这是选右不变误差的好处之一），因此：

H_vb = [ 0, C_m R̂ᵀ, 0, 0, 0, −(C_m R̂ᵀ v̂)^× ]

**网络 `MotionNet`**（PyTorch，CPU，约 5–10 万参数）：
- 输入：最近 1–2 s 的 IMU 窗口（100 Hz × 6 通道，零偏用滤波器当前估计扣除），再加上滤波器给出的 roll/pitch（用于重力投影），做通道标准化。
- 结构：3 层 1D-TCN（膨胀卷积）→ 全局池化 → MLP，两个输出头：
  - 均值 μ ∈ ℝ³：前向速度、侧向速度、垂向速度
  - 对数方差 log σ̂² ∈ ℝ³（异方差）
- 损失：高斯负对数似然 L = Σ_j [ (y_j − μ_j)² / σ̂_j² + log σ̂_j² ]，加 β-NLL 加权（β = 0.5）防止方差塌缩。
- 监督标签：KITTI OXTS 自带的 vf/vl/vu（前/左/上速度）；i2Nav 数据用参考真值姿态和速度换算到车体系。
- 退化模式（消融用）：只输出 NHC（μ_lat = μ_up = 0），方差由网络给出，等价于 AI-IMU。

### 3.5 核心创新：在线共形一致性校准（ACI-Kalman）
对每个伪观测通道 j，以及 GNSS 观测，维护缩放因子 κ_j，令 R̃_j = κ_j σ̂_j²。

**校准参考**：GNSS 健康（通过 §3.6 检验）且滤波收敛时，用 GNSS 约束后的后验 v̂_b⁺ 作参考，不需要真值，可以在线自监督。离线评估时也可以用真值做对照实验。

**非一致性分数**（扣除参考本身的不确定度）：

s_t,j = (μ_t,j − v̂⁺_b,t,j)² / (κ σ̂²_t,j + (H P⁺ Hᵀ)_jj)

**ACI 更新**（目标覆盖率 1−α，如 α = 0.05）：
1. 误差指示 err_t = 1[ s_t,j(κ_t) > χ²₁(1−α_t) ]
2. α_{t+1} = α_t + γ(α − err_t)，γ ≈ 0.005
3. κ_{t+1} = 在最近 N 个分数（如 N = 600，即 60 s）上解 1-D 方程：Quantile_{(1−α_{t+1})}(s(κ)) = χ²₁(1−α)（二分法，几十微秒级）

**GNSS 中断期**（无参考）：冻结 κ 并做保守膨胀 κ(τ) = κ₀ · (1 + β τ / T_ref)，τ 为中断时长，β 由仿真标定。这一点很关键：伪观测是在"老分布"上校准的，中断越久越要谨慎。

**GNSS 侧**用同样的机制：分数为 NIS_k / 3 的 χ²₃ 分位版本，从而在线得到 κ_g。

理论依据：ACI 在任意分布漂移下保证长期平均覆盖率收敛到 1−α（Gibbs & Candès 2021）。对 KF 来说，这意味着伪观测的 NIS 分布被"拉"到名义 χ² 附近，P 阵一致性随之改善。

### 3.6 双通道完好性 / 欺骗检测
- **通道 A**（主滤波）：IMU + GNSS + 伪观测。
- **通道 B**（DR 守卫）：IMU + 校准伪观测 + NHC，**不吃 GNSS**。每隔 T_reset = 30 s，且通道 A 在整窗都健康时，从 A 复制状态和协方差重置 B，保证窗内 B 独立于 GNSS。
- 统计量：d_t = y_p,t − (p_B + R_B l_b)，Σ_t = R̃_gnss + P_B,pp（校准后可信），q_t = d_tᵀ Σ_t⁻¹ d_t
- 检测器：
  - 突变：q_t > χ²₃(1−P_fa)
  - 缓慢拉偏：CUSUM g_t = max(0, g_{t−1} + q_t − 3 − k)，g_t > h 时报警（k、h 由仿真按目标虚警率标定）
- 响应：报警后通道 A 进入"类中断"模式（不用 GNSS，靠 B 的约束桥接），并输出告警事件；连续 M 个历元恢复一致后再接回 GNSS。
- 保护级：HPL_t = K_md · sqrt(λ_max(P_hh))，K_md 取 5.33（10⁻⁷ 量级）。报告"误差超界率"（HPE > HPL 的比例）。

### 3.7 可选扩展（二期）
- 1–5 s 固定滞后 RTS 平滑（吃 FGO 的重线性化红利，A1/A2）。
- UrbanNav 原始伪距紧耦合，加 NLOS 概率→Student-t 自由度映射（G1、C 类）。
- 在线微调 MotionNet 最后一层（GNSS 健康时 SGD，学习率很小，带回滚）。

## 4. 软件架构与模块

```
cc-gins/
├── README.md                    # 中文/英文说明、快速开始、结果表
├── pyproject.toml               # 依赖与 ruff/pytest 配置
├── configs/
│   ├── imu/{adis16465.yaml, consumer_mems.yaml}
│   ├── sim/{urban_loop.yaml, highway.yaml}
│   ├── filter/{eskf.yaml, inekf.yaml, cc_gins.yaml}
│   └── experiments/{kitti_outage.yaml, i2nav_outage.yaml, spoof_ramp.yaml}
├── ccgins/
│   ├── geometry/        so3.py, se23.py（Exp/Log/Jr/adjoint，numpy 向量化）
│   ├── sim/             trajectory.py（样条/车辆运动学轨迹）, imu_model.py（ARW/VRW/零偏不稳定性/比例因子）,
│   │                    gnss_model.py（噪声、中断、NLOS 跳变、斜坡欺骗注入）
│   ├── data/            kitti.py（OXTS 100 Hz 解析→IMU+真值，合成 GNSS）, i2nav.py（awesome-gins 格式读取）,
│   │                    urbannav.py（二期，使用 rosbags 读取 bag）, outages.py（中断/攻击场景生成器）
│   ├── filters/         base.py（接口：propagate/update/inject）, eskf.py, inekf.py,
│   │                    robust.py（χ² 门限、Huber/Student-t 权重）
│   ├── learning/        motionnet.py, dataset.py（滑窗）, train.py, export.py（TorchScript/ONNX）
│   ├── calib/           aci.py（ACI 校准器：分数缓存、二分求 κ、中断膨胀）
│   ├── integrity/       dual_channel.py（通道 B、CUSUM、HPL）
│   ├── pipeline.py      # 组装：传感器流→滤波器→校准器→完好性→日志
│   └── eval/            metrics.py, outage_eval.py, consistency.py（NEES/NIS/覆盖率）, plots.py
├── scripts/
│   ├── download_kitti.sh, download_i2nav.sh
│   ├── run_sim.py, train_motionnet.py, run_experiment.py, make_report.py
├── tests/                       # pytest：李群恒等式、雅可比数值校验、无噪声仿真收敛、ACI 覆盖率
├── notebooks/                   # 结果展示（可选）
└── results/                     # 自动生成的表格/图（.gitignore 大文件）
```

关键接口（示意）：

```python
class Filter(Protocol):
    def propagate(self, imu: ImuSample) -> None: ...
    def update(self, z: np.ndarray, h: MeasModel, R: np.ndarray, robust: RobustCfg | None) -> UpdateInfo: ...
    @property
    def state(self) -> NavState: ...          # R, v, p, bg, ba, dm, P

class ACICalibrator:
    def __init__(self, dim: int, alpha: float = 0.05, gamma: float = 0.005, window: int = 600): ...
    def scale(self, sigma2: np.ndarray, outage_time: float) -> np.ndarray: ...   # 返回 κ·σ²
    def observe(self, residual: np.ndarray, sigma2: np.ndarray, ref_var: np.ndarray) -> None: ...
```

## 5. 数据与仿真计划

### 5.1 仿真（M0，确保真值和可控性）
- 轨迹：城市环线 + 高速 + 掉头/急转（车辆运动学模型，NHC 有轻微侧滑）。
- IMU：两档 MEMS 参数（工业级如 ADIS16465 量级、消费级），包括 ARW、VRW、零偏不稳定性、比例因子、安装角误差。
- GNSS：1/10 Hz，σ = 0.5–3 m；场景包括中断 30/60/120 s、NLOS 跳变（5–30 m）、多路径有色噪声、斜坡欺骗（0.1–1 m/s 拉偏）。
- 用途：验证雅可比（数值差分）和 NEES 一致性，并标定 CUSUM 阈值和中断膨胀系数 β。

### 5.2 KITTI raw（主实验，真实 IMU）
- 下载：https://www.cvlibs.net/datasets/kitti/raw_data.php（需要注册；使用各序列的 **unsynced/extract** 版 OXTS，100 Hz）。
- OXTS 每行 30 个字段，含 lat/lon/alt、roll/pitch/yaw、vf/vl/vu、af/al/au、wf/wl/wu 等（解析可参考 pykitti：https://github.com/utiasSTARS/pykitti ）。
- 方法：IMU = af/al/au、wf/wl/wu；真值 = RTK/INS 解；**合成 GNSS** = 真值加噪声，并注入中断和异常。说明：KITTI 的 IMU 测量来自 RT3003 的高质量 IMU，必要时叠加 MEMS 噪声，模拟低成本器件。
- 划分：按驾驶序列做留出（训练/验证/测试序列不重叠），与 AI-IMU（F1）的划分保持一致，便于对比。

### 5.3 i2Nav awesome-gins-datasets（真实 MEMS + 真实 GNSS）
- 下载：https://github.com/i2Nav-WHU/awesome-gins-datasets （多款 MEMS IMU、GNSS RTK 解和高精度参考真值）。
- 用途：跨器件泛化（同一轨迹不同 IMU 等级），以及真实 GNSS 中断（人工屏蔽 GNSS 段）。格式与 KF-GINS（https://github.com/i2Nav-WHU/KF-GINS ）一致，可以直接用 KF-GINS 的输出做 ESKF 基线交叉验证。

### 5.4 UrbanNav（二期，真实城市异常）
- https://github.com/IPNL-POLYU/UrbanNavDataset （MIT 许可，含 RINEX、IMU、SPAN-CPT 真值，ROS bag），用 `pip install rosbags` 在无 ROS 环境下读取。

## 6. 基线与消融

| 编号 | 方法 | 说明 |
|---|---|---|
| B0 | ESKF-LC | KF-GINS 式 15 维 ESKF，中断期纯 INS |
| B1 | ESKF-LC + 固定 NHC | 侧/垂向速度为 0，R 固定（工程常用） |
| B2 | InEKF-LC + 固定 NHC | 检验不变误差本身的收益 |
| B3 | InEKF + AI-IMU 式学习 NHC 协方差（未校准） | 对应 F1 |
| B4 | InEKF + 学习伪速度，网络方差直接当 R（未校准） | 对应 E3/E5 思路 |
| B5 | InEKF + VB/Student-t 自适应 GNSS R | 对应 C1–C4 |
| **Ours** | InEKF + 学习伪速度 + ACI 校准 + 抗差 GNSS + 双通道监测 | 完整 CC-GINS |
| A1–A4 | 消融 | 去掉 ACI / 去掉中断膨胀 / ESKF 替换 InEKF / 单通道检测 |

## 7. 评价指标
- **精度**：全程水平/3D 位置 RMSE、速度 RMSE、航向 RMSE。
- **中断桥接**：每次中断（30/60/120 s）的最大水平误差、结束时误差、误差占行驶距离百分比（%DT）。
- **一致性 / 可信度**：
  - ANEES（理想值 1）
  - 伪观测和 GNSS 的平均 NIS 及其 χ² 拟合
  - 位置 95% 置信椭圆的经验覆盖率（理想值 95%）
  - HPE > HPL 的超界率
- **完好性 / 检测**：检测时延、漏检率、每小时虚警次数（阶跃/斜坡/NLOS 三类）。
- **效率**：每个 IMU 历元的耗时（µs），MotionNet 单次推理耗时（CPU），内存占用。
- 统计：每种设置至少 20 次蒙特卡洛（仿真）或全部测试序列 × 多个中断起点（真实数据），报告均值 ± 标准差。

预期目标（作为验收参考，不是承诺）：
- 60 s 中断最大水平误差比 B1 降低 ≥ 30%，比 B4 降低 ≥ 15%。
- ANEES ∈ [0.7, 1.5]，覆盖率 ∈ [90%, 98%]，而 B3/B4 显著偏离。
- 斜坡欺骗检测时延比单通道新息检验短 ≥ 30%，虚警率相同。
- 单历元滤波 < 200 µs（numpy），满足实时要求。

## 8. 环境（本机 Linux box）

```bash
python3 -m venv ~/venvs/ccgins && source ~/venvs/ccgins/bin/activate
pip install --upgrade pip
pip install numpy scipy pandas matplotlib pyyaml tqdm pytest ruff
pip install torch --index-url https://download.pytorch.org/whl/cpu
pip install rosbags   # 二期 UrbanNav 才需要
```
- 数据目录：`/workspace/data/{kitti,i2nav,urbannav}`（不进 git），仓库代码放在 `/workspace/cc-gins`。
- 全部实验只用 CPU；MotionNet 在 KITTI 上训练预计在笔记本级 CPU 上用时少于 1 小时（参数量小、窗口短）。

## 9. 里程碑

| 里程碑 | 内容 | 验收标准 | 估计工作量 |
|---|---|---|---|
| M0 | 仓库骨架、李群库、仿真器 | pytest 通过：Exp/Log 往返、雅可比数值校验误差 < 1e-6 | 1–2 天 |
| M1 | ESKF-LC 基线（B0/B1） | 无噪声仿真收敛；有噪声时 ANEES≈1；i2Nav 数据与 KF-GINS 结果误差接近 | 2 天 |
| M2 | InEKF-LC + NHC（B2） | 大初始航向误差（如 90°）下收敛快于 ESKF | 2 天 |
| M3 | MotionNet 训练 + 数据管线（B3/B4） | KITTI 测试序列伪速度 RMSE 报告；NLL 收敛 | 2–3 天 |
| M4 | ACI 校准器 + 中断膨胀 | 仿真覆盖率达到 95%±3%；KITTI 中断实验改进 | 2 天 |
| M5 | 抗差 GNSS + 双通道 CUSUM + HPL | 三类攻击场景的检测曲线 | 2 天 |
| M6 | 全量实验、消融、自动报告 | `scripts/make_report.py` 生成表格和图，README 给出结果 | 2 天 |
| M7（二期） | UrbanNav 紧耦合 / 固定滞后平滑 / 在线微调 | — | 视需要 |

## 10. 风险与对策
- **仅凭 IMU 估计前向速度是病态问题**：KITTI 上可能不准。对策：前向速度方差会被 ACI 自动放大，滤波器主要利用侧/垂向约束；有里程计的数据接入轮速作为前向速度观测。
- **时间序列不可交换**：ACI 只保证长期平均覆盖率。对策：滑窗分位数 + 中断膨胀，并在实验中报告逐段覆盖率。
- **参考值与伪观测相关**（参考是后验，本身用过伪观测）：对策：用 GNSS-only 的前验或滞后平滑速度作参考，或校准时把 HP⁺Hᵀ 换成先验协方差。消融里比较两者。
- **KITTI 的 GNSS 是合成的**：对策：用 i2Nav 真实 GNSS 补充，二期用 UrbanNav 的真实城市异常。

## 11. 与后续"私有仓库"任务的衔接
- 本方案可以直接交给编码代理：按 §4 目录建仓，按 §9 里程碑逐个提交，每个里程碑都有可运行脚本和 pytest。
- 仓库创建（GitHub 私有仓库）、推送等外部操作由主代理在获得用户确认后执行；本阶段只在本机 `/workspace/nav-research/` 产出文档。
