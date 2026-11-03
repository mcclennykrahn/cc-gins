# 卫星导航 / 惯性导航 / 组合导航 近期文献综述（2023–2026 为主）

> 检索时间：2026-11-03（UTC-6）。所有条目均通过 WebSearch/WebFetch 核实了真实链接（DOI / arXiv / 期刊页 / GitHub）。少数经典奠基工作（2019–2021）因为是本项目的直接基线，单独标注为"基线文献"。
> 说明：部分 IEEE/Elsevier 论文只核实了摘要页，作者信息未完整获取时标注"作者见链接"。

---

## A. 组合导航：滤波 vs 因子图（FGO）

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| A1 | Relative Accuracy of GNSS/INS Integration Based on Factor Graph Optimization | IEEE Sensors Journal, 2024 | https://doi.org/10.1109/jsen.2024.3451665 |
| A2 | Factor graph optimization for GNSS/INS integration: A comparison with the extended Kalman filter（基线文献） | Wen W., Pfeifer T., Bai X., Hsu L.-T.；NAVIGATION 68(2), 2021 | https://navi.ion.org/content/68/2/315 ；arXiv: https://arxiv.org/abs/2004.10572 |
| A3 | Second Place Winner of the Smartphone Decimeter Challenge: An Open-Source FGO Package for GNSS and IMU Integration in Smartphones | Taro Suzuki；ION GNSS+ 2024 | https://doi.org/10.33012/2024.19923 ；代码 https://github.com/taroz/gsdc2023 |
| A4 | GSDC 开源 FGO 包的相关 arXiv 论文 | Taro Suzuki；arXiv 2025 | https://arxiv.org/abs/2502.08158 |

- **A1** 核心：首次用 Allan 方差评估 FGO 组合导航的"相对精度"，短期相对精度比 EKF 提升 3D 位置 23.6%、速度 16.5%。局限：只看相对精度，未讨论实时计算量和异常值场景。
- **A2** 核心：在香港城市峡谷中系统对比 LC/TC 的 EKF 与 FGO，TC-FGO 2D 均值误差 3.64 m，而 TC-EKF 为 8.03 m；收益来自多次迭代/重线性化和时间相关性。局限：TC-FGO 处理全程耗时 75 s，远高于 EKF 的 0.1 s，嵌入式实时性存疑。
- **A3/A4** 核心：GSDC 2023 第二名，GNSS(伪距/多普勒/TDCP)+IMU 紧耦合 FGO，私榜 0.789 m，并已开源。局限：面向离线后处理（批量平滑），不是因果实时滤波。

## B. 组合导航：不变 EKF（InEKF / 李群滤波）

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| B1 | SINS/GNSS Integrated Navigation Based on Invariant Error Models in Inertial Frame | IEEE Sensors Journal, 2024 | https://doi.org/10.1109/jsen.2023.3346873 |
| B2 | Adaptive Invariant EKF-Based Tightly-Coupled SINS/RTK-Integrated Positioning for Rotor UAV | IEEE Trans. Instrum. Meas., 2024 | https://doi.org/10.1109/tim.2024.3449937 |
| B3 | A land vehicle's INS/GNSS integrated navigation system using left invariant EKF | Ibrahim A., Azouz A., Abosekeen A.；J. Phys.: Conf. Ser. 2616, 2023 | https://iopscience.iop.org/article/10.1088/1742-6596/2616/1/012023 |
| B4 | GNSS/INS Navigation System Based on Lie Group Right Invariant Kalman Filter | Ye X. 等；IEEE ICUS 2023 | https://doi.org/10.1109/icus58632.2023.10318318 |

- **B1** 核心：在惯性系中构造满足 group-affine 的李群状态，推导左/右不变误差模型，把初始对准和组合导航合并成一步。局限：只有船载试验，没有城市 GNSS 异常值场景。
- **B2** 核心：推导 TC RTK/SINS 的 LIEKF 与 RIEKF，两者可切换，并加入自适应机制，提升旋翼无人机场景下的收敛性和鲁棒性。局限：从摘要看，自适应仍基于传统统计，没有学习模块，也没有可信度（覆盖率）标定。
- **B3/B4** 核心：车载/无人机 LC 场景下 IEKF 对初值不敏感，B3 的 2D RMS 从 19.43 m 降到 3.32 m。局限：多为 LC、无车辆约束、无异常值处理，代码多不开源。

## C. 组合导航：自适应 / 鲁棒滤波（异常值、重尾噪声）

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| C1 | A novel Bayesian-based INS/GNSS integrated positioning method with both adaptability and robustness in urban environments (M-VBt) | Chinese Journal of Aeronautics, 2024（作者见链接） | https://doi.org/10.1016/j.cja.2023.11.024 |
| C2 | Variational Bayesian-based robust adaptive filtering for GNSS/INS tightly coupled positioning in urban environments | Measurement, 2023 | https://doi.org/10.1016/j.measurement.2023.113668 |
| C3 | Variational robust filter with a heavy-tailed mixture model for GNSS/INS tightly coupled integration | Meas. Sci. Technol., 2024 | https://doi.org/10.1088/1361-6501/ad95ac |
| C4 | Gaussian–Student's t Mixture Distribution-Based Robust KF for GNSS/INS/Odometer Data Fusion | Remote Sensing 16(24):4716, 2024 | https://doi.org/10.3390/rs16244716 |
| C5 | Integrity-Constrained Factor Graph Optimization for GNSS Positioning in Urban Canyons | Xia X., Wen W., Hsu L.-T.；NAVIGATION 71(3), 2024 | https://doi.org/10.33012/navi.660 |

- **C1–C4** 核心：用变分贝叶斯 + Student-t / 高斯–t 混合分布同时估计未知时变 R 并抑制异常值，C1 还加了马氏距离预剔除。局限：VB 迭代需要调超参（自由度、遗忘因子）；GNSS 中断时没有可用新息，自适应失效；没有和学习方法结合。
- **C5** 核心：在 FGO 中用 switch 变量加卡方检验约束代替直接剔除卫星，保留几何构型并给出保护级（HPL）。UrbanNav-Medium 上 HPE 从 11.8 m 降到 9.0 m。局限：只用 GNSS，作者自己说 switch 先验严重依赖调参，未来要加 IMU。

## D. 学习辅助的噪声协方差 / 卡尔曼增益

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| D1 | Dynamic Process Noise Covariance Adjustment in GNSS/INS Using GRU-SAC | IEEE T-ITS, 2025 | https://doi.org/10.1109/tits.2025.3540205 |
| D2 | A-KIT: Adaptive Kalman-Informed Transformer | arXiv 2024（作者见链接） | https://arxiv.org/abs/2401.09987 |
| D3 | Deep-Learning-Assisted Kalman Filter for GNSS/MEMS IMU Integration in GNSS-Denied Environments (DLKF) | NAVIGATION 73(1), 2026（作者见链接） | https://navi.ion.org/content/73/1/navi.787 |
| D4 | Practical Implementation of KalmanNet for Accurate Data Fusion in Integrated Navigation | IEEE Signal Processing Letters, 2024 | https://doi.org/10.1109/lsp.2024.3431443 |
| D5 | Towards Predicting the Measurement Noise Covariance with a Transformer and RDAE for GNSS/INS TC（2022，基线） | Remote Sensing 14(7):1691 | https://www.mdpi.com/2072-4292/14/7/1691 |

- **D1** 核心：把 Q 的调整建模成 POMDP，用 GRU + SAC 强化学习在线调 Q，LC 和 TC 都能用，中断期漂移更小。局限：RL 训练成本高，可解释性和可迁移性差。
- **D2** 核心：用 set-transformer 在线回归过程噪声 Q。局限：场景是水下 DVL；输出协方差没有一致性/覆盖率保证。
- **D3** 核心：DNN 输出卡尔曼增益和 IMU 误差，同时保留误差状态 KF 结构；仿真中断 30–50 s 时位置 RMS 1.43 m→0.63 m。局限：直接学增益，协方差 P 与真实误差的一致性没有保证，完好性无从谈起。
- **D4/D5** 核心：KalmanNet 和 Transformer 预测 R 的工程化实现。局限：同样缺少不确定度标定，对分布外场景敏感。

## E. GNSS 中断桥接（伪观测预测）

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| E1 | GNSS/INS Compensation Method Based on CNN–GRU + IRAKF Hybrid Model During GNSS Outages | IEEE TIM, 2024 | https://doi.org/10.1109/tim.2024.3369131 |
| E2 | Online Semi-Supervised Transformer for Resilient Vehicle GNSS/INS Navigation | IEEE TVT, 2024 | https://doi.org/10.1109/tvt.2024.3422680 |
| E3 | CTGANN: Channel-Mixed and Temporal Gated Attention NN … Predicting Pseudo-Velocity During GNSS Outages | IEEE Sensors Journal, 2025 | https://doi.org/10.1109/jsen.2025.3558968 |
| E4 | Enhanced GNSS/INS compensation using LSTM-FPN for bridging GNSS outages | Meas. Sci. Technol., 2024 | https://doi.org/10.1088/1361-6501/ad9cab |
| E5 | Uncertainty-Quantified Pseudo-GNSS via GANP for Robust INS/GNSS Navigation of UAVs During GNSS Outages | IEEE Sensors Journal, 2026 | https://doi.org/10.1109/jsen.2026.3680041 |

- **E1–E4** 核心：用网络预测 GNSS 伪位置、伪增量或伪速度，中断期作为观测更新。E3 指出伪速度比伪位置累积误差更小；E2 采用在线半监督 Transformer，40 s/100 s 中断最大误差分别为 6.02 m/9.81 m。局限：大多用固定或启发式 R，网络过度自信时会把滤波器"拉偏"；多为私有数据，难以复现。
- **E5** 核心：用神经过程（GANP）输出伪 GNSS 的预测分布协方差作为时变 R。局限：协方差来自模型本身，没有经验覆盖率校准；场景是无人机。

## F. 惯性导航：学习型 IMU / 车辆运动约束 / 标定

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| F1 | AI-IMU Dead-Reckoning（基线文献） | Brossard M., Barrau A., Bonnabel S.；IEEE T-IV 2020 | https://arxiv.org/abs/1904.06064 ；代码 https://github.com/mbrossar/ai-imu-dr |
| F2 | AirIMU: Learning Uncertainty Propagation for Inertial Odometry | Qiu Y., Wang C., …, Scherer S.；arXiv 2023/2024 | https://arxiv.org/abs/2310.04874 |
| F3 | NHC-LIO（CNN 预测 NHC 伪观测的车载 LiDAR-惯性里程计） | IEEE Sensors Journal, 2023 | https://doi.org/10.1109/jsen.2023.3317575 |
| F4 | Invariant EKF for Pedestrian Deep-Inertial Odometry | Bai S., Wen W., Yu Y., Hsu L.-T.；ISPRS Archives 2024 | https://isprs-archives.copernicus.org/articles/XLVIII-4-2024/607/2024/ |
| F5 | Wheel-GINS: A GNSS/INS Integrated Navigation System With a Wheel-Mounted IMU | i2Nav-WHU；IEEE T-ITS 2025 | https://arxiv.org/abs/2501.03079 ；代码 https://github.com/i2Nav-WHU/Wheel-GINS |
| F6 | Debiasing 6-DOF IMU via Hierarchical Learning of Continuous Bias Dynamics | Liu B., Lin T.-Y., Zhang W., Ghaffari M.；arXiv 2025 | https://arxiv.org/abs/2504.09495 |
| F7 | TinyGC-Net（超轻量 MEMS 陀螺标定/去噪网络） | arXiv 2024 | https://arxiv.org/html/2403.02618v2 |
| F8 | Rapid Gyroscope Calibration: A Deep Learning Approach | Stolero Y., Klein I.；arXiv 2024 | https://arxiv.org/abs/2409.00488 |

- **F1** 核心：IEKF 加"侧向/垂向速度≈0"伪观测，CNN 根据 IMU 动态给出伪观测协方差，KITTI 纯 IMU 平均平移误差 1.10%。局限：没有 GNSS，协方差只是为了降低误差而训练，并未标定，也不保证一致性。
- **F2** 核心：可微 IMU 积分加流形上协方差传播，联合学习噪声修正和不确定度，IMU-GPS 位姿图精度提升 31.6%。局限：偏离线、图优化；不确定度仍没有覆盖率保证。
- **F3/F4** 核心：用网络预测"可靠的 NHC 伪观测"（F3）或前向速度+NHC（F4），作为滤波/图优化中的观测。局限：没有和 GNSS 质量监测联动。
- **F5** 核心：轮载 IMU 提供轮速，加 NHC，ESKF 在线估计杆臂、安装角和轮径比例误差，开源。局限：需要额外硬件（轮载 IMU）。
- **F6/F7/F8** 核心：用学习方法显式建模 IMU 零偏动态（NODE 上李群）、做超轻量 MCU 级陀螺标定/去噪，或把标定时间最多缩短 89%。局限：多在 VIO 或转台数据上验证，未接入 GNSS/INS 滤波闭环。

## G. 卫星导航：城市 NLOS / 多路径、欺骗检测、高精度定位

| # | 论文 | 作者 / 期刊 / 年份 | 链接 |
|---|---|---|---|
| G1 | GNSS positioning enhancement based on NLOS signal detection using spatio-temporal learning in urban canyons | Li F., Zhai C., Xie T., Dai Z., Zhu X.；GPS Solutions, 2024 | https://link.springer.com/article/10.1007/s10291-024-01737-0 |
| G2 | Sky-GVIO（基于 FCN 天空分割的 GNSS/INS/视觉组合） | arXiv 2024 | https://arxiv.org/abs/2404.11070 |
| G3 | Doppler-Based RAIM for GNSS Spoofing Detection in Vehicular Applications | Zhou Z., Li H., Lu M.；IEEE TVT 74(7), 2025 | https://doi.org/10.1109/tvt.2025.3543612 |
| G4 | Spoofing Detection Algorithm for Tightly Coupled INS/GNSS Integration Model with Robust Averaging Measurement | IEEE ICSIDP 2024 | https://doi.org/10.1109/icsidp62679.2024.10868403 |
| G5 | Tightly coupled multi-frequency PPP-RTK/INS integration model and its application in an urban environment | Geo-spatial Information Science, 2023 | https://www.tandfonline.com/doi/full/10.1080/10095020.2023.2251535 |
| G6 | Advancing High-Precision Navigation: Leveraging Homogeneous Sensors in Tightly Coupled PPP-RTK/IMU Integration | IEEE Trans. Ind. Electron., 2024 | https://doi.org/10.1109/tie.2024.3363741 |

- **G1** 核心：CNN-LSTM 时空学习识别 NLOS，动态数据识别率 95.3%，加权 SPP 后 2D 精度提升 37.6%。局限：只做 SPP，NLOS 分类结果没有以概率形式进入组合滤波。
- **G2** 核心：鱼眼/天空分割识别 NLOS，再接入 GNSS/INS/视觉。局限：需要朝天相机。
- **G3/G4** 核心：G3 用多普勒定位残差做 RAIM，独立检测欺骗；G4 用 TC 新息加鲁棒均值检测缓慢拉偏式欺骗。局限：缓慢拉偏攻击的检测仍依赖阈值，且和 IMU 预测一致性的结合较弱。
- **G5/G6** 核心：多频 PPP-RTK/INS 紧耦合，城市中 96.1% 历元水平误差小于 10 cm，固定率 90.9%；G6 利用同构多 IMU 冗余增强紧耦合 PPP-RTK/IMU。局限：依赖改正数服务和载波相位，工程门槛高，笔记本级 Python 复现难度大。

## H. 公开数据集 / 开源基线（项目落地会用到）

| # | 资源 | 链接 |
|---|---|---|
| H1 | Hong Kong UrbanNav（Hsu L.-T. 等，NAVIGATION 70(4), 2023），含原始 RINEX、IMU、SPAN-CPT 真值 | https://doi.org/10.33012/navi.602 ；https://github.com/IPNL-POLYU/UrbanNavDataset |
| H2 | i2Nav awesome-gins-datasets（多款 MEMS IMU + RTK + 真值） | https://github.com/i2Nav-WHU/awesome-gins-datasets |
| H3 | KF-GINS（武大 i2Nav 开源 ESKF 松组合，C++） | https://github.com/i2Nav-WHU/KF-GINS |
| H4 | KITTI raw（OXTS RT3003 GNSS/IMU 100 Hz） | https://www.cvlibs.net/datasets/kitti/raw_data.php |
| H5 | GSDC 2023 开源方案 | https://github.com/taroz/gsdc2023 |

---

## 小结：文献中的共性趋势
1. **FGO 精度优于 EKF，但实时性和算力是瓶颈**；IEKF 在大失准角和一致性上优于 ESKF，计算量与 EKF 相当，是低成本平台更实际的选择。
2. **学习方法正在从"直接替代"转向"嵌入模型"**：学习 Q/R/增益（D 类）、学习伪观测（E、F 类）。
3. **普遍缺口在可信度**：几乎所有学习型协方差或伪观测都没有做**经验覆盖率 / 一致性（NEES/NIS）校准**，对分布外场景缺少保护。这正是组合导航完好性（integrity）最在意的部分。
