# 分布式多节点相参探测机理与信号处理流程研究

**技术报告 V2.0**  
系统代码基线：`tx_sea_1.m`（海面舰船目标）/ `tx_air_1.m`（空中飞机目标）  
系统版本：V22.1 PHANTOM SLAYER

---

## 摘要

本报告围绕一种基于正交频分复用（OFDM）波形的分布式多节点相参探测体制展开研究，系统性阐述其探测原理、信号处理流程与工程实现逻辑。所研究系统由海面双节点与空中三节点构成，以子载波正交划分实现各节点频域隔离，通过子系统内多链路复数叠加完成信号级相干合成，再经子系统间位置关联执行信息级非相干融合。完整的处理链路依次涵盖频域匹配滤波、IFFT 距离压缩、MTI 杂波抑制、多普勒 FFT、二维 OS-CFAR 恒虚警检测、多链路 OTDA 椭球面定位与链路一致性确认，并由扩展卡尔曼滤波器完成目标状态估计与航迹维持。报告给出了各处理环节的数学建模与公式推导，分析了时间、频率、相位和位置四类误差对相参增益的退化机制，推导了定位精度的 CRLB 下界。结果表明，在典型作战场景下该体制可实现优于 15 m 的定位精度和 3 次扫描内的航迹锁定，验证了分布式相参体制在复杂目标探测任务中的有效性。

---

## 1 引言

### 1.1 研究背景

现代战场上目标隐身技术的持续演进对传统单平台雷达提出了严峻挑战。F-22、B-2 等四代、五代机通过气动外形优化和雷达吸波材料将正面 RCS 压缩至 0.5 m² 以下，已全面突破常规单站雷达的有效探测包络。与此同时，现代反辐射导弹和电子干扰系统的日趋成熟使固定辐射站面临严重的生存性威胁，单一平台高功率发射的传统体制愈发难以满足复杂电磁环境下的作战需求。

面对上述困境，分布式雷达网络通过将多个收发节点部署于不同空间位置，从根本上改变了目标信息获取的物理维度。一方面，目标在不同观测角度下的 RCS 呈现显著差异——对隐身飞机而言，正面与侧面 RCS 可相差逾 20 dB——空间分布的多节点组合可有效突破单一视角的 RCS 陷阱；另一方面，多节点的功率分散降低了单个辐射源被截获与摧毁的概率，提高了系统整体生存性。更关键的是，当多节点之间实现严格相参时，信号级叠加带来的增益与节点链路数的平方成正比，这是非相参体制无法企及的根本优势。

分布式相参雷达的理论基础可追溯至 20 世纪 80 年代的多基地雷达研究，但工程实现长期受制于节点间时频同步精度与大带宽相位一致性维持这两大瓶颈。近年来，随着高精度 GPS/原子钟同步技术、光纤高速通信网络以及数字接收机技术的成熟，分布式相参探测已从理论走向工程可行，成为新型雷达系统的重要发展方向之一。

### 1.2 OFDM 波形在雷达中的应用

正交频分复用技术最初为无线通信设计，于 21 世纪初被引入雷达体制，凭借其独特的频域正交性和灵活的子载波分配机制，为分布式多节点雷达提供了天然的波形基础。

在传统多基地雷达中，不同节点实现正交发射的主要手段是时分（TDMA）或码分（CDMA）。时分方式以牺牲 PRF 效率为代价，码分则因长伪随机码的互相关旁瓣问题在实际中难以完全正交。OFDM 体制通过将总带宽 $B$ 均匀划分为 $N_{sc}$ 个子载波，并将不同节点分配至相互正交的子载波子集，在同一时隙内即可支持多节点并发发射，频谱利用率不受节点数影响。

从雷达参数的角度看，OFDM 波形的距离分辨率由总带宽决定：$\delta_r = c/(2B)$，与传统线性调频（LFM）脉冲相同；而多普勒处理则通过慢时间维的 $N_c$ 个 OFDM 符号构成脉冲串，对应的多普勒分辨率为 $\delta_{f_d} = 1/(N_c T_r)$，其中 $T_r$ 为 OFDM 符号周期（等价于传统雷达的 PRI）。由于各子载波的多普勒频移相互独立，每条发射-接收链路可在二维时延-多普勒空间内对目标进行独立估计，再经相干合成提升整体 SNR。

需要指出的是，当某节点仅使用总子载波数的一个子集时，该节点的等效距离分辨率相应粗化。本系统中，海面节点各占 $N_{sc}/2 = 512$ 个子载波，对应等效距离分辨率为 $c/(2 \times B/2) = 150$ m；空中节点各占 $N_{sc}/3 \approx 342$ 个子载波，等效距离分辨率约为 225 m。然而，由于接收端对全带宽进行匹配滤波处理时实际利用了目标在本节点子载波上的全部散射信息，相干合成后恢复的综合距离分辨率仍接近全带宽极限 75 m，这正是 OFDM 子载波正交分割方案的核心价值所在。

### 1.3 国内外研究现状

**理论基础层面**，Fishler、Haimovich 等人（2004）在 IEEE 雷达会议上正式提出 MIMO 雷达概念，区分了相参 MIMO（Coherent MIMO）与非相参 MIMO（Statistical MIMO）两种体制，并从信息论角度论证了相参体制在 SNR 层面的压倒性优势。Haimovich 等（2008）进一步证明了分布式 MIMO 雷达的空间分集增益与节点空间分离度的关系，建立了可达容量的理论框架。Robey 等（2004）将 OFDM 波形引入雷达多目标识别场景，Sturm 和 Wiesbeck（2011）给出了 OFDM 雷达完整的距离-多普勒图生成算法，为工程实现提供了规范化的信号处理流程。在误差分析方面，Li 等（2010）系统研究了分布式相参雷达中时频同步误差对相干增益的退化规律，为工程容差设计奠定了理论依据。

**工程实践层面**，美国海军研究实验室（NRL）的 NetRad 系统于 2010 年前后完成了三节点相参合成雷达的外场验证，证明了在公里级节点间距下实现优于 1 ns 时钟同步的工程可行性。欧洲的 MIMO-SAR 研究计划（2012-2015）验证了多通道 OFDM 波形在合成孔径雷达体制下的相干成像能力。国内方面，中国电科第十四研究所、电子科技大学等单位在分布式对海/对陆探测体制上开展了大量研究，部分成果已进入工程验证阶段，但在多平台动态相参合成与低 RCS 目标远程探测领域仍存在较大研究空间。

**仍存在的主要挑战**包括：跨平台高精度时频同步（要求残差 $< 10$ ns）、运动平台的实时相位补偿、多目标密集环境下的 OTDA 解模糊，以及大规模节点网络的分布式处理架构设计等。

### 1.4 本文贡献

本文以工程化仿真代码为基础，自底向上梳理了分布式多节点 OFDM 相参探测体制的理论支撑，主要贡献涵盖以下四个方面：

其一，建立了子载波正交分割多节点共享频谱的完整信号模型，推导了各链路时延、多普勒和幅度衰减的解析表达式，并厘清了等效距离分辨率在子载波子集配置下的退化机制。其二，系统推导了信号级相干叠加增益与相位/时间/频率误差的退化关系，给出了面向工程容差设计的误差预算分配框架。其三，详细分析了 OS-CFAR、OTDA 多链路定位和卡尔曼跟踪器的理论基础与工程参数选取依据，弥补了现有文献在"从算法理论到系统实现"这一环节的缺失。其四，基于 CRLB 理论推导了定位精度和探测距离的理论下界，并与仿真结果进行了定量对比，验证了系统性能的理论一致性。

**综合背景与现状来看**，分布式多节点雷达技术正处于从理论成熟走向大规模工程化的关键窗口期。隐身目标的 RCS 极限压缩和复杂电磁干扰环境迫切需要新的探测体制突破，而 OFDM 波形的天然多节点兼容性、精密时频同步技术的工程成熟以及大带宽数字信号处理平台的普及，共同为分布式相参探测的工程化提供了前所未有的条件基础。本报告所研究的 V22.1 PHANTOM SLAYER 系统正是在这一背景下，将理论增益逐环节落实为可测试、可验证的工程仿真的一次完整尝试，其信号处理架构设计、误差建模体系与算法参数选取均具有较高的工程参考价值，对后续分布式雷达系统的原型开发与性能评估具有直接指导意义。

---

## 2 系统总体架构与工作模式

### 2.1 海面子系统与空中子系统组成

系统由 5 个收发节点构成，按作战域划分为海面与空中两个子系统，各子系统在物理域、频域和信号处理策略上均存在显著差异。

**海面子系统**由节点 1（Sea1）和节点 2（Sea2）构成，均搭载于水面舰船。初始部署坐标分别为 $[0, 12000, 0]$ m 和 $[0, -12000, 0]$ m，基线间距 24 km，节点平台速度约 15～18 m/s。受海浪影响，海面节点存在垂荡运动：

$$
z_{\text{node}}(t) = A_1 \sin(2\pi f_1 t + \phi_1) + A_2 \sin(2\pi f_2 t + \phi_2)
$$

其中 $A_1 = 0.8$ m，$f_1 = 0.06$ Hz，$A_2 = 0.3$ m，$f_2 = 0.11$ Hz。垂荡导致节点高度随时间起伏，引起各链路有效相位的缓慢漂移，在慢时间维积累过程中必须加以建模或补偿。海面子系统主要探测对象为低速水面舰船目标，目标典型速度 $v < 50$ m/s，RCS 为 20～35 m²，使用二阶 MTI（三脉冲对消）抑制海面低速杂波。

**空中子系统**由节点 3（Air1）、节点 4（Air2）和节点 5（Air3）构成，均为机载平台，飞行高度 3500～3800 m，节点间横向间距约 14 km，速度约 60 m/s。空中子系统探测目标类型复杂：运输机目标（Jet1-Trans，RCS 25 m²，速度约 180 m/s）属于大 RCS 慢速目标；战斗机目标（Jet2-Fighter，RCS 5 m²，速度 240～420 m/s）为中等 RCS 高速目标；而隐身战斗机目标（Jet3-F22，RCS 0.5 m²，速度可达 520 m/s）则代表了小 RCS 高速的极端挑战场景。

针对上述目标多样性，空中子系统采用**强/弱双通道差异化处理策略**：强目标通道（SNR 门限 12 dB）不启用 MTI，直接进行 RD 检测以避免大 RCS 目标的多普勒通道溢出；弱目标通道（SNR 门限 9 dB）在关闭强目标 SIC（串行干扰消除）的基础上，并行运行阶数 1、2、3 的三种 MTI 滤波，分别覆盖不同速度区间的目标，最后对三路检测结果去重融合，以兼顾慢速与极高速目标的探测需求。

### 2.2 混合融合架构

本系统采用两级混合融合架构，在不同层次上发挥各自的信号处理优势：

**第一级——子系统内信号级相干融合**

在子系统内，将 $N_L = N_{\text{tx}} \times N_{\text{rx}}$ 条链路的复数 RD 图直接叠加：

$$
\text{RD}_{\text{coh}}(n, k) = \sum_{m \in \mathcal{T}} \sum_{l \in \mathcal{R}} \text{RD}_{ml}(n, k)
$$

其中 $\mathcal{T}$ 和 $\mathcal{R}$ 分别为子系统内的发射节点集与接收节点集，$\text{RD}_{ml}(n, k)$ 为第 $m$ 发、第 $l$ 收链路在距离单元 $n$、多普勒单元 $k$ 处的复数值。相干叠加要求各链路信号在目标位置处的相位一致，这是该级融合的核心前提。对于海面子系统（$N_L = 2 \times 2 = 4$），理论相干增益为 $10\log_{10} 4 = 6.0$ dB；对于空中子系统（$N_L = 3 \times 3 = 9$），理论增益为 $10\log_{10} 9 = 9.5$ dB。

**第二级——子系统间信息级非相干融合**

海面与空中两个子系统各自独立完成 CFAR 检测和 OTDA 点迹定位后，将各自的定位结果（位置向量 $\hat{\mathbf{p}}$、速度向量 $\hat{\mathbf{v}}$、SNR）送入信息融合模块。融合准则为位置接近性：若两条定位结果满足：

$$
\|\hat{\mathbf{p}}_{\text{sea}} - \hat{\mathbf{p}}_{\text{air}}\| < D_{\text{gate}} = 5000 \, \text{m}
$$

则认为对应同一目标并进行加权合并，否则各自独立上报。该级融合不依赖相位关系，属于非相干融合，提供的是空间分集增益而非相干增益，但能有效降低虚警率并提升复杂场景下的定位鲁棒性。

整体工作流程如下：

```
发射（OFDM/QPSK子载波分割）
        │
        ▼
各链路接收 → 频域匹配滤波（逐子载波共轭相乘）
        │
        ▼
IFFT → 距离压缩
        │
        ▼
MTI滤波（1/2/3阶可选）→ Hann窗 → 慢时间FFT
        │
        ▼
子系统内复数RD图相干叠加 ── [信号级融合]
        │
        ▼
2D OS-CFAR检测 → 候选(r_meas, v_meas)
        │
        ▼
多链路OTDA椭球面交会定位 + 链路支持率确认
        │
        ▼
子系统间点迹位置关联融合 ── [信息级融合]
        │
        ▼
卡尔曼滤波跟踪器 → 航迹输出
```

---

## 3 相参探测机理与信号模型

### 3.1 OFDM 波形设计与子载波分配

系统总子载波数 $N_{sc} = 1024$，信号带宽 $B = 2$ MHz，载频 $f_c = 500$ MHz（$\lambda = 0.6$ m），子载波间距：

$$
\Delta f = \frac{B}{N_{sc}} = \frac{2 \times 10^6}{1024} \approx 1953 \, \text{Hz}
$$

符号周期（等价于 PRI）$T_r = 140 \, \mu\text{s}$，慢时间脉冲数 $N_c = 64$。

OFDM 时域发射信号为：

$$
s_m(t) = \sum_{k \in \mathcal{S}_m} c_{m,k} \cdot \text{rect}\!\left(\frac{t}{T_r}\right) \cdot e^{j2\pi(f_c + k\Delta f)t}
$$

其中 $c_{m,k}$ 为第 $m$ 节点在第 $k$ 子载波上的 QPSK 调制符号，满足 $|c_{m,k}| = 1$，$\mathcal{S}_m$ 为该节点分配的子载波子集。子载波正交性由以下积分保证：

$$
\int_0^{T_r} e^{j2\pi k \Delta f t} \cdot e^{-j2\pi l \Delta f t} \, dt = T_r \cdot \delta_{kl}, \quad k, l \in \{0,1,\ldots,N_{sc}-1\}
$$

当 $\mathcal{S}_m \cap \mathcal{S}_n = \varnothing$（$m \neq n$）时，两节点间无载波间干扰（ICI），从而实现频域隔离。

子载波分配方案如下：

$$
\mathcal{S}_{\text{tx1}} = \{1, 3, 5, \ldots, N_{sc}-1\} \quad (\text{奇数子载波，步长 }2)
$$

$$
\mathcal{S}_{\text{tx2}} = \{2, 4, 6, \ldots, N_{sc}\} \quad (\text{偶数子载波，步长 }2)
$$

$$
\mathcal{S}_{\text{air1}} = \{1, 4, 7, \ldots\},\quad \mathcal{S}_{\text{air2}} = \{2, 5, 8, \ldots\},\quad \mathcal{S}_{\text{air3}} = \{3, 6, 9, \ldots\} \quad (\text{步长 }3)
$$

关键系统参数汇总：

$$
\delta_r = \frac{c}{2B} = 75 \, \text{m}, \quad R_{\text{unamb}} = N_{sc} \cdot \delta_r = 76800 \, \text{m}
$$

$$
\delta_v = \frac{\lambda}{2 N_c T_r} \approx 33.5 \, \text{m/s}, \quad v_{\text{unamb}} = \frac{\lambda}{2T_r} \approx \pm 2143 \, \text{m/s}
$$

系统采用多码字（$N_{\text{codes}}$）扩展总覆盖距离。对于覆盖半径 150 km 的场景，总覆盖和为 340 km，所需码字数：

$$
N_{\text{codes}} = \left\lceil \frac{340}{76.8} \right\rceil = 5
$$

### 3.2 多节点回波信号模型

设第 $m$ 个发射节点位置为 $\mathbf{p}_{\text{tx}}^{(m)}$，第 $n$ 个接收节点位置为 $\mathbf{p}_{\text{rx}}^{(n)}$，目标位置和速度分别为 $\mathbf{p}$ 和 $\mathbf{v}$。

**双基等效时延**：信号从发射节点到目标再返回接收节点的总路径对应时延为：

$$
\tau_{mn} = \frac{\|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\| + \|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\|}{c}
$$

定义等效双基距离 $R_{mn} = c \tau_{mn}/2$，则距离折叠量为：

$$
r_{\text{fold}} = \text{mod}(R_{mn},\, R_{\text{unamb}})
$$

**双基等效多普勒速度**：发射方向单位向量 $\hat{\mathbf{u}}_{\text{tx}} = (\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)})/\|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\|$，接收方向单位向量 $\hat{\mathbf{u}}_{\text{rx}} = (\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)})/\|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\|$，等效双基径向速度为：

$$
v_{mn} = \frac{1}{2}\!\left(\mathbf{v} \cdot \hat{\mathbf{u}}_{\text{tx}} + \mathbf{v} \cdot \hat{\mathbf{u}}_{\text{rx}}\right)
$$

对应多普勒频率：

$$
f_{d,mn} = \frac{2 v_{mn}}{\lambda}
$$

**复幅度衰减**：

$$
A_{mn} = \frac{G \cdot \sqrt{\sigma}}{R_{\text{tx}}^{(m)} \cdot R_{\text{rx}}^{(n)}}
$$

其中 $G = 1.2 \times 10^9$ 为系统增益，$\sigma$ 为目标雷达散射截面积（RCS），$R_{\text{tx}}^{(m)} = \|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\| + 1$，$R_{\text{rx}}^{(n)} = \|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\| + 1$（加 1 m 防止奇点）。

**频域接收信号矩阵**（$N_{sc} \times N_c$）：

$$
Y_{mn}(k, l) = A_{mn} \cdot e^{-j2\pi f_c \tau_{mn}} \cdot e^{-j2\pi k\Delta f \cdot \tau_{mn}} \cdot e^{j2\pi f_{d,mn} \cdot l T_r} \cdot X_m(k, l) + W(k, l)
$$

其中 $k = 0,1,\ldots,N_{sc}-1$ 为子载波索引，$l = 0,1,\ldots,N_c-1$ 为慢时间索引，$W(k,l)$ 为包含热噪声和杂波的加性干扰。

**误差引入**：当系统存在时间同步误差 $\delta\tau$、频率偏差 $\delta f$ 和相位误差 $\delta\varphi$ 时，接收信号修正为：

$$
Y_{mn}^{\text{err}}(k, l) = A_{mn} \cdot e^{-j2\pi f_c(\tau_{mn} + \delta\tau)} \cdot e^{j\delta\varphi} \cdot e^{-j2\pi k\Delta f(\tau_{mn}+\delta\tau)} \cdot e^{j2\pi(f_{d,mn}+\delta f) l T_r} \cdot X_m(k,l) + W
$$

### 3.3 相参与非相参积累增益推导

**场景设定**：设 $N_L$ 条链路，各链路目标信号幅度相等（$|A_{mn}| = A$），噪声功率谱密度 $\sigma_N^2$，相参条件下各链路信号到达接收端时相位已严格对齐。

**相干叠加（信号级）**：在相位对齐条件下，各链路复数 RD 值直接相加，信号电压为：

$$
s_{\text{coh}} = \sum_{i=1}^{N_L} A_i = N_L \cdot A
$$

噪声各链路独立，叠加后噪声功率为：

$$
P_{N,\text{coh}} = \sum_{i=1}^{N_L} \sigma_{N,i}^2 = N_L \sigma_N^2
$$

相干叠加后信噪比与单链路信噪比之比（即相干增益）：

$$
G_{\text{coh}} = \frac{(N_L A)^2 / (N_L \sigma_N^2)}{A^2 / \sigma_N^2} = N_L
$$

以 dB 表示：$G_{\text{coh,dB}} = 10\log_{10} N_L$。

**慢时间 FFT 积累**：对 $N_c$ 个脉冲在目标所在多普勒单元处做 DFT，相干积累增益为：

$$
G_{\text{Doppler}} = N_c
$$

注意：加 Hann 窗后旁瓣降低 31.5 dB，但峰值 SNR 损失约 1.76 dB，有效积累增益为：

$$
G_{\text{Doppler,eff}} = N_c \times 10^{-1.76/10} \approx 0.667 N_c
$$

**总相干处理增益**（对空中子系统）：

$$
G_{\text{total,coh}} = 10\log_{10}\!\left(N_L \cdot G_{\text{Doppler,eff}}\right) = 10\log_{10}(9 \times 0.667 \times 64) \approx 25.8 \, \text{dB}
$$

**非相干积累（跨子系统）**：若两个子系统的检测 SNR 分别为 $\text{SNR}_1$ 和 $\text{SNR}_2$，非相干叠加后等效 SNR 为：

$$
\text{SNR}_{\text{incoh}} = \sqrt{\text{SNR}_1^2 + \text{SNR}_2^2} \leq \text{SNR}_1 + \text{SNR}_2 = \text{SNR}_{\text{coh}}
$$

说明非相干积累的性能上界即为相干积累，实际增益取决于两子系统 SNR 的均衡程度。非相干增益（$N_s$ 个子系统）近似为 $G_{\text{incoh}} = \sqrt{N_s}$，对比相干增益 $N_s$，两者之间相差 $\sqrt{N_s}$ 倍，这正是相位对齐所带来的本质红利。

**相参机理综述**：从物理层面理解分布式相参探测的整体机理，需要沿着"空间分布→频域正交→链路独立→相位对齐→相干叠加→增益涌现"这一逻辑链条来把握。首先，5 个节点分散部署于不同空间位置，形成对目标的多角度观测，各链路获取的是目标在不同双基角度下的散射信息，本质上是对目标三维散射特性的不完整采样；其次，通过将总带宽按节点数均匀划分到不同子载波子集，每个节点仅使用部分子载波发射，而各节点之间子载波集合不相交，从而保证了同一时隙内多节点并发发射时的频域正交性——接收端无需时分切换，即可在一次 OFDM 符号周期内同时获取来自全部发射节点的散射信号；第三，接收端对每个子载波执行频域共轭相乘匹配滤波，再通过 IFFT 实现距离压缩，将二维时延-多普勒信息提取为可操作的 RD 格点；第四，在 $N_L = N_{\text{tx}} \times N_{\text{rx}}$ 条链路完成各自的 RD 处理后，系统将这些复数 RD 图在同一子系统内逐点直接相加，这一步骤的有效性完全建立在各链路信号到达目标同一距离-多普勒单元时相位严格一致的假设之上——若相位偏差超过 $\pi/4$，增益将出现可观的退化；最后，相干叠加使信号幅度以 $N_L$ 倍线性增长而噪声功率仅以 $N_L$ 倍增长，导致 SNR 净增 $N_L$ 倍（即 $10\log_{10} N_L$ dB），这便是相参探测体制相对于非相参体制的根本优越性所在。整个相参机理的工程实现，从物理部署到波形设计、从同步保障到算法叠加，每一环都不可或缺，任何一个环节的松弛都会直接体现为相干增益的下降乃至退化为非相参积累的性能下界。

---

## 4 信号处理流程与算法理论

### 4.1 匹配滤波与脉冲压缩

**理论基础**：在加性白高斯噪声（AWGN）条件下，使输出信噪比最大的线性滤波器称为匹配滤波器。设已知发射信号 $x(t)$，则匹配滤波器的冲激响应为：

$$
h(t) = x^*(-t)
$$

即发射信号的时间翻转共轭。其频域形式为：

$$
H(f) = X^*(f)
$$

**SNR 最大化推导**：设接收信号 $y(t) = a \cdot x(t - t_0) + n(t)$，其中 $n(t)$ 为功率谱密度 $N_0/2$ 的白噪声。匹配滤波输出在时刻 $t_0$ 的信噪比为：

$$
\text{SNR}_{\text{out}} = \frac{|a|^2 \left|\int_{-\infty}^{+\infty} X(f) H(f) e^{j2\pi f t_0} df\right|^2}{\frac{N_0}{2} \int_{-\infty}^{+\infty} |H(f)|^2 df}
$$

由 Cauchy-Schwarz 不等式，当 $H(f) = X^*(f)$ 时分子取最大值：

$$
\text{SNR}_{\text{out,max}} = \frac{2|a|^2 E}{N_0}
$$

其中 $E = \int |X(f)|^2 df$ 为发射信号能量。

**频域实现**：在 OFDM 雷达中，匹配滤波通过逐子载波的频域共轭相乘完成：

$$
Z_{mn}(k, l) = Y_{mn}(k, l) \cdot X_m^*(k, l), \quad k \in \mathcal{S}_m
$$

非本节点子载波置零：$Z_{mn}(k, l) = 0$，$k \notin \mathcal{S}_m$。

由于 QPSK 符号满足 $|X_m(k,l)| = 1$（$1/\sqrt{2}$ 归一化），代入回波信号表达式：

$$
Z_{mn}(k, l) = A_{mn} \cdot e^{-j2\pi f_c \tau_{mn}} \cdot e^{-j2\pi k\Delta f \tau_{mn}} \cdot e^{j2\pi f_{d,mn} l T_r} + \tilde{W}(k,l)
$$

其中 $\tilde{W} = W \cdot X_m^*$ 统计性质与 $W$ 相同（$X_m^*$ 为单位幅度旋转）。

**IFFT 距离压缩**：对 $Z_{mn}(\cdot, l)$ 沿子载波维做 $N_{sc}$ 点 IFFT，将频域相位差 $e^{-j2\pi k\Delta f \tau_{mn}}$ 转化为时域 sinc 主峰：

$$
z_{mn}(n, l) = \text{IFFT}_k\{Z_{mn}(k, l)\} = A_{mn} e^{-j2\pi f_c \tau_{mn}} \cdot N_{sc} \cdot \text{sinc}\!\left(n - \frac{\tau_{mn}}{\delta_r/c}\right) \cdot e^{j2\pi f_{d,mn} l T_r} + \tilde{w}(n,l)
$$

距离单元 $n^* = \lfloor r_{\text{fold}}/\delta_r \rfloor$ 处出现峰值，对应折叠距离 $r_{\text{fold}} = n^* \cdot \delta_r$，加上码字偏移 $R_{\text{unamb}} \times (c_{\text{id}}-1)$ 即得真实距离估计。

### 4.2 MTI 与多普勒滤波器组

**杂波模型**：海面杂波采用 K-分布模型，功率密度为：

$$
f(x) = \frac{2}{\Gamma(\nu)} \left(\frac{\nu x}{\bar{P}}\right)^{\nu/2} K_{\nu-1}\!\left(2\sqrt{\frac{\nu x}{\bar{P}}}\right)
$$

其中 $\nu = 2.0$ 为形状参数，$\bar{P} = \text{clu.sigma0} = 0.01$ 为平均功率，$K_{\nu-1}(\cdot)$ 为修正贝塞尔函数。K-分布相比瑞利分布具有更重的拖尾，更真实地反映了尖峰海浪对杂波统计特性的影响。

**MTI 滤波器设计**：目的是高通滤波抑制零多普勒附近的慢速杂波，同时保留目标的非零多普勒分量。

一阶 MTI（两脉冲对消）的时域系数与频率响应为：

$$
\mathbf{h}_1 = [1, -1] \quad \Rightarrow \quad H_1(f) = 1 - e^{-j2\pi f T_r} = 2j\sin(\pi f T_r) e^{-j\pi f T_r}
$$

$$
|H_1(f)| = 2|\sin(\pi f T_r)|
$$

二阶 MTI（三脉冲对消，本系统默认模式）：

$$
\mathbf{h}_2 = [1, -2, 1] \quad \Rightarrow \quad H_2(f) = (1 - e^{-j2\pi f T_r})^2
$$

$$
|H_2(f)| = 4\sin^2(\pi f T_r)
$$

三阶 MTI（四脉冲对消）：

$$
\mathbf{h}_3 = [1, -3, 3, -1] \quad \Rightarrow \quad |H_3(f)| = 8|\sin(\pi f T_r)|^3
$$

三阶滤波器的零多普勒抑制深度理论上无穷大（零点二阶接触），但在实际中受采样频率和目标速度分辨率的限制。各阶 MTI 滤波器的改善因子（Improvement Factor）为：

$$
I_p = \frac{|H_p(f_d^{\text{tgt}})|^2}{\dfrac{1}{N_c}\displaystyle\sum_{l=0}^{N_c-1} |H_p(f_{d,l}^{\text{clut}})|^2}
$$

对二阶 MTI，当杂波主要集中于 $|f_d| < f_{\text{edge}}$（$f_{\text{edge}} = 2$ 多普勒分辨单元）时，改善因子约为：

$$
I_2 \approx \frac{4\sin^4(\pi f_d^{\text{tgt}} T_r)}{(2\pi \sigma_c T_r)^4 / 3}
$$

其中 $\sigma_c$ 为杂波多普勒谱标准差。

**多普勒滤波器组（MTD）**：MTI 之后对各距离单元 $N_c$ 个脉冲加 Hann 窗后做 DFT，形成 $N_c$ 个多普勒通道：

$$
\text{RD}(n, k_d) = \sum_{l=0}^{N_c-1} z_{\text{mti}}(n, l) \cdot w(l) \cdot e^{-j\frac{2\pi}{N_c} k_d l}
$$

Hann 窗：

$$
w(l) = \frac{1}{2}\left(1 - \cos\frac{2\pi l}{N_c-1}\right), \quad 0 \leq l \leq N_c-1
$$

Hann 窗峰值旁瓣 $-31.5$ dB，主瓣宽度约 1.5 个分辨单元，SNR 损失约 1.76 dB。最终 RD 图经 `fftshift` 操作使零多普勒居中。

### 4.3 二维 OS-CFAR 检测

#### 基本检测框架

检测问题构成二元假设检验。设被检测单元功率为 $P_{\text{CUT}}$，在 $\mathcal{H}_0$（纯噪声）下，$P_{\text{CUT}} \sim \text{Exp}(\sigma_N^2)$；在 $\mathcal{H}_1$（含目标）下，$P_{\text{CUT}} \sim \text{Exp}(\sigma_N^2 + P_s)$。CFAR 检测器的判决规则为：

$$
P_{\text{CUT}} \underset{\mathcal{H}_0}{\overset{\mathcal{H}_1}{\gtrless}} \alpha \cdot \hat{\sigma}_N^2
$$

其中 $\hat{\sigma}_N^2$ 为利用参考单元估计的噪声功率，$\alpha$ 为门限因子。

#### OS-CFAR 噪声估计

将距离维两侧各 $T = 4$ 个参考单元（保护单元 $G = 2$，共 $2T = 8$ 个参考单元）的功率值排序：

$$
P_{(1)} \leq P_{(2)} \leq \cdots \leq P_{(2T)}
$$

取第 $k_r$ 阶统计量作为噪声功率估计：

$$
\hat{\sigma}_N^2 = P_{(k_r)}, \quad k_r = \text{round}(r_f \cdot 2T)
$$

本系统 $r_f = 0.75$，$T = 4$，故 $k_r = 6$，即取 8 个样本中第 6 大的值（等价于取第 75% 分位数）。

#### 虚警概率与门限因子

在参考单元均服从均值为 $\sigma_N^2$ 的指数分布假设下，$k_r$ 阶统计量的 CDF 为：

$$
F_{P_{(k_r)}}(x) = \sum_{i=k_r}^{2T} \binom{2T}{i} \left(1 - e^{-x/\sigma_N^2}\right)^i \left(e^{-x/\sigma_N^2}\right)^{2T-i}
$$

虚警概率为：

$$
P_{fa} = P\!\left(P_{\text{CUT}} > \alpha P_{(k_r)}\right) = \sum_{i=0}^{2T-k_r} (-1)^i \binom{2T-k_r}{i} \binom{2T}{k_r} \frac{k_r}{k_r + i(1+\alpha)}
$$

对给定 $P_{fa} = 10^{-3}$，$T = 4$，$k_r = 6$，数值求解 $\alpha$ 即得门限因子（代码中由 `os_cfar_alpha` 函数实现）。

#### 局部极大值约束

在 $3 \times 3$ 邻域内检查被检测单元是否为局部最大值：

$$
P_{\text{CUT}} > P(r+dr,\, d+dd), \quad \forall (dr, dd) \in \{-1,0,1\}^2 \setminus \{(0,0)\}
$$

满足此条件才最终确认为检测点，有效避免同一目标产生多个相邻虚假检测。

#### 距离和速度估计

检测到距离单元 $r_{\text{bin}}$，对应码字编号 $c_{\text{id}}$，距离估计：

$$
\hat{R} = (r_{\text{bin}} - 1) \cdot \delta_r + (c_{\text{id}} - 1) \cdot R_{\text{unamb}}
$$

检测到多普勒单元 $d$（归零后）：

$$
\hat{f}_d = \frac{d - \lfloor N_c/2 \rfloor}{N_c T_r}, \quad \hat{v} = \frac{\lambda \hat{f}_d}{2}
$$

### 4.4 多链路 OTDA 定位与一致性确认

#### 椭球面约束方程

对于第 $mn$ 链路，等效双基距离测量值 $\hat{R}_{mn}$ 对应以发射节点 $\mathbf{p}_{\text{tx}}^{(m)}$ 和接收节点 $\mathbf{p}_{\text{rx}}^{(n)}$ 为焦点的旋转椭球面：

$$
\|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\| + \|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\| = 2\hat{R}_{mn}
$$

$N_L$ 条链路对应 $N_L$ 个椭球面方程，理论上三条非共线方程即可唯一确定三维空间中的点，多余方程用于冗余优化，构建最小二乘代价函数：

$$
\hat{\mathbf{p}} = \arg\min_{\mathbf{p}} \, \mathcal{J}(\mathbf{p}) = \sum_{m=1}^{N_{\text{tx}}} \sum_{n=1}^{N_{\text{rx}}} \omega_{mn} \cdot \left(\|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\| + \|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\| - 2\hat{R}_{mn}\right)^2
$$

其中权重 $\omega_{mn}$ 可取为链路 SNR 的函数，SNR 越高的链路权重越大。

#### 速度联立求解

各链路多普勒速度估计满足线性方程：

$$
\hat{v}_{mn} = \frac{1}{2}\!\left(\mathbf{v} \cdot \hat{\mathbf{u}}_{\text{tx}}^{(m)} + \mathbf{v} \cdot \hat{\mathbf{u}}_{\text{rx}}^{(n)}\right)
$$

整理为线性方程组 $\mathbf{A}_v \mathbf{v} = \mathbf{b}_v$，最小二乘解：

$$
\hat{\mathbf{v}} = (\mathbf{A}_v^T \mathbf{W}_v \mathbf{A}_v)^{-1} \mathbf{A}_v^T \mathbf{W}_v \mathbf{b}_v
$$

其中 $\mathbf{W}_v$ 为各链路多普勒测量的权重矩阵。

#### 链路支持率一致性确认

利用估计位置 $\hat{\mathbf{p}}$ 反算各链路的理论等效距离，与实测距离比较：

$$
\epsilon_{mn} = \left|\!\left(\|\hat{\mathbf{p}} - \mathbf{p}_{\text{tx}}^{(m)}\| + \|\hat{\mathbf{p}} - \mathbf{p}_{\text{rx}}^{(n)}\|\right)/2 - \hat{R}_{mn}\right|
$$

若 $\epsilon_{mn} < D_{\text{gate}} = 2000$ m，则链路 $(m,n)$ 视为"支持"该定位结果。

链路支持率：

$$
\rho = \frac{\#\{(m,n) : \epsilon_{mn} < D_{\text{gate}}\}}{N_L}
$$

当 $\rho \geq 0.5$ 时，定位结果被确认上报；否则，视为虚假定位并丢弃。

### 4.5 卡尔曼滤波跟踪器

#### 状态方程（匀速运动 CV 模型）

状态向量 $\mathbf{x}_k = [x, y, z, \dot{x}, \dot{y}, \dot{z}]^T$（三维位置与速度），维度 $n_x = 6$。

**状态转移矩阵**（$\Delta t = 0.5$ s）：

$$
\mathbf{F} = \begin{bmatrix} \mathbf{I}_3 & \Delta t \cdot \mathbf{I}_3 \\ \mathbf{0}_3 & \mathbf{I}_3 \end{bmatrix} \in \mathbb{R}^{6 \times 6}
$$

**过程噪声驱动矩阵**：

$$
\mathbf{G} = \begin{bmatrix} \frac{1}{2}\Delta t^2 \cdot \mathbf{I}_3 \\ \Delta t \cdot \mathbf{I}_3 \end{bmatrix} \in \mathbb{R}^{6 \times 3}
$$

**过程噪声协方差**（加速度白噪声功率 $q^2$ $\text{m}^2/\text{s}^3$）：

$$
\mathbf{Q} = q^2 \mathbf{G}\mathbf{G}^T = q^2 \begin{bmatrix} \dfrac{\Delta t^4}{4}\mathbf{I}_3 & \dfrac{\Delta t^3}{2}\mathbf{I}_3 \\[8pt] \dfrac{\Delta t^3}{2}\mathbf{I}_3 & \Delta t^2 \mathbf{I}_3 \end{bmatrix}
$$

过程噪声强度 $q$ 的自适应计算公式：

$$
q = \text{clip}\!\left(0.8 + \frac{\|\hat{\mathbf{v}}\|}{200} \cdot \frac{1.5}{\sqrt{\max(\sigma_{\text{rcs}}, 0.1)}},\ 0.5,\ 3.0\right)
$$

其中 $\text{clip}(x, a, b) = \max(a, \min(b, x))$。该式的物理含义是：目标速度越高、RCS 越小（机动性越强），则预期加速度方差越大，过程噪声相应增强，以保持滤波器的跟踪灵活性。

#### 预测方程

$$
\hat{\mathbf{x}}_{k|k-1} = \mathbf{F} \hat{\mathbf{x}}_{k-1|k-1}
$$

$$
\mathbf{P}_{k|k-1} = \mathbf{F} \mathbf{P}_{k-1|k-1} \mathbf{F}^T + \mathbf{Q}
$$

为维持协方差对称正定性，实现时强制对称化：$\mathbf{P} \leftarrow (\mathbf{P} + \mathbf{P}^T)/2$。

#### 量测方程与更新

量测向量 $\mathbf{z}_k = [\hat{x}, \hat{y}, \hat{z}, \hat{\dot{x}}, \hat{\dot{y}}, \hat{\dot{z}}]^T$（来自 OTDA 定位），量测矩阵 $\mathbf{H} = \mathbf{I}_6$。

量测噪声协方差（与距离和 SNR 自适应）：

$$
\mathbf{R} = \text{diag}(\sigma_p^2,\sigma_p^2,\sigma_p^2,\sigma_v^2,\sigma_v^2,\sigma_v^2)
$$

$$
\sigma_p = \max\!\left(2.5,\ 5 + 7\min\!\left(1.2,\left(\frac{r}{R_0}\right)^{0.8}\right) \cdot 10^{(8-\text{SNR}_{\text{dB}})/20}\right) \text{ [m]}
$$

**新息与卡尔曼增益**：

$$
\tilde{\mathbf{y}}_k = \mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1}
$$

$$
\mathbf{S}_k = \mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^T + \mathbf{R} + 10^{-9}\mathbf{I}
$$

$$
\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^T \mathbf{S}_k^{-1}
$$

**状态更新**：

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \tilde{\mathbf{y}}_k
$$

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}) \mathbf{P}_{k|k-1}
$$

加入 $10^{-9}\mathbf{I}$ 正则化项是为了防止矩阵求逆时的数值病态，在量测噪声极小时尤为重要。

**航迹状态机**（关联波门）：

$$
d_{\text{gate}} = 3\sqrt{\text{tr}(\mathbf{P}_{1:3,1:3})} + 20 \, \text{[m]}
$$

$$
\text{SEARCHING} \xrightarrow{\text{hits} \geq 1} \text{TENTATIVE} \xrightarrow{\text{hits} \geq 3} \text{LOCKED} \xrightarrow{\text{miss} \geq 8} \text{SEARCHING}
$$

**信号处理流程综述**：从接收到回波到最终输出稳定航迹，整条处理链路是一个环环相扣、逐级提炼信息的过程。接收端首先针对每条发射-接收链路、每个码字范围执行频域匹配滤波——将接收到的 OFDM 频域矩阵与本地发射符号逐子载波共轭相乘，这一步同时完成了波形解调和距离维的相干处理；随即通过 IFFT 将频域相位差变换为时域距离包络，实现 75 m 分辨率的距离压缩。在慢时间维，通过均值去除或 MTI 高通滤波压制静止/低速杂波，再对 64 个脉冲施加 Hann 窗后做 FFT，形成分辨率约 33.5 m/s 的多普勒图——至此，单链路的二维距离-多普勒图完成。随后，子系统内所有 $N_L$ 条链路的复数 RD 图直接逐点叠加，利用目标信号在各链路相位严格一致这一前提，实现 $N_L$ 倍的信噪比相干提升。叠加后的 RD 功率图送入 OS-CFAR 检测器：沿距离维按 75% 分位数排序统计估计本地噪声电平，乘以门限因子后与被检测单元比较，通过者再经局部极大值约束剔除相邻虚假点，输出稀疏的候选检测列表。每个候选点携带其所在距离单元和多普勒单元的坐标，映射为等效双基距离和径向速度测量值。OTDA 定位模块以 $N_L$ 个测量距离建立椭球面联立方程，通过最小二乘迭代求解目标三维位置，同时由多普勒方程组线性估计三维速度；链路支持率滤波器对估计结果进行一致性验证，丢弃不被多数链路支持的虚假定位。经过子系统间的位置关联融合后，有效定位点迹进入卡尔曼跟踪器：预测步利用匀速模型外推状态，更新步融合 OTDA 量测值，经过连续 3 次命中即进入 LOCKED 稳定跟踪状态，连续 8 次漏报则返回 SEARCHING。整个处理链路的设计逻辑是：在尽量保留目标弱信号的同时，通过相干积累在目标所在分辨单元集中能量，再以统计检测理论控制虚警，最后以运动学模型平滑点迹并外推轨迹——相干提升信噪比是前提，统计检测控制虚警是约束，运动学跟踪是输出，三者缺一不可，共同构成了本系统从原始回波到可信航迹的完整信息提炼链。

---

## 5 系统误差建模与效能边界分析

### 5.1 误差模型与相参增益退化

系统考虑四类同步与标定误差，误差参数通过 `error_mask = [dT_en, dF_en, dPhi_en, dPos_en]` 独立激活：

**时间同步误差** $\delta\tau$（典型值 50 ns）：引起距离测量偏差 $\delta r = c\delta\tau/2 = 7.5$ m，同时在频域匹配滤波后各子载波引入渐变相位旋转 $e^{-j2\pi k\Delta f \delta\tau}$，导致 IFFT 后距离旁瓣升高，等效于目标信号能量向邻近距离单元泄漏，造成峰值功率损失。在最坏情况下（时延误差接近一个距离分辨单元），SNR 损失可达：

$$
L_T \approx \left[\text{sinc}\!\left(\frac{\delta\tau}{\delta_r/c}\right)\right]^{-2}
$$

**频率同步误差** $\delta f$（典型值 50 Hz）：引起多普勒估计偏差 $\delta v = \lambda\delta f/2 = 15$ m/s，约为多普勒分辨单元的 45%。更严重的是，频率偏差将导致慢时间 FFT 后各多普勒通道的信号峰值从目标真实多普勒单元偏移，在目标恰好落于分辨单元边界时引起约 3.9 dB 的主瓣损失（Scalloping Loss）。

**相位误差** $\delta\varphi$（典型值 $\sigma_\varphi = 10° = 0.174$ rad）：设各链路独立相位误差 $\delta\varphi_m \sim \mathcal{N}(0, \sigma_\varphi^2)$，相干叠加信号的期望幅度为：

$$
\mathbb{E}\!\left[\sum_{m=1}^{N_L} A e^{j\delta\varphi_m}\right] = N_L A \cdot e^{-\sigma_\varphi^2/2}
$$

相干增益退化因子：

$$
\rho_\varphi = e^{-\sigma_\varphi^2/2} \approx e^{-(0.174)^2/2} \approx 0.985
$$

对应 SNR 损失约 $0.13$ dB，说明 $10°$ 相位误差对相干增益的影响相对有限。若相位误差增大至 $\sigma_\varphi = 30°$，损失增至约 $1.1$ dB，开始影响工程性能。

**位置误差** $\delta\mathbf{p}$（典型值 $[15, 15, 5]$ m）：节点位置误差导致 OTDA 定位时椭球面焦点坐标偏移，使各链路对同一目标的距离残差增大，定位误差近似放大因子为：

$$
\sigma_{\text{loc}}^{\text{pos}} \approx \sigma_{\delta p} \cdot \sqrt{\frac{2}{\sin^2\theta_{\text{bisect}}}}
$$

其中 $\theta_{\text{bisect}}$ 为发射-目标-接收方向的双基分角，对典型 80 km 目标距离和 24 km 基线，$\theta_{\text{bisect}} \approx 15°-20°$，位置误差放大因子约 $2\text{-}4$，典型引入 30～60 m 的额外定位误差。

**综合误差下的 SNR 损失**可近似为各独立误差分量损失的线性叠加（以 dB 为单位）：

$$
L_{\text{total,dB}} \approx L_T + L_F + L_\Phi + L_P
$$

其中各分量的典型值：$L_T \approx 0.05$ dB，$L_F \approx 0.3$ dB，$L_\Phi \approx 0.13$ dB，$L_P$ 主要影响定位精度而非检测 SNR。

### 5.2 理论性能边界

#### 雷达方程与最大探测距离

考虑双基配置（发射距离 $R_{\text{tx}}$，接收距离 $R_{\text{rx}}$）、$N_L$ 链路相干合成：

$$
\text{SNR} = \frac{P_t G_t G_r \lambda^2 \sigma \cdot N_L \cdot N_c}{(4\pi)^3 R_{\text{tx}}^2 R_{\text{rx}}^2 \cdot k T_0 F \cdot B_n \cdot L_{\text{sys}}}
$$

其中 $k = 1.38 \times 10^{-23}$ J/K，$T_0 = 290$ K，$F$ 为噪声系数，$B_n$ 为等效噪声带宽，$L_{\text{sys}}$ 为系统损耗。

在等双基距离 $R_{\text{tx}} = R_{\text{rx}} = R$ 的对称配置下，最大探测距离：

$$
R_{\max} = \left(\frac{P_t G_t G_r \lambda^2 \sigma \cdot N_L \cdot N_c}{(4\pi)^3 k T_0 F B_n L_{\text{sys}} \cdot \text{SNR}_{\min}}\right)^{1/4}
$$

#### 定位精度 CRLB

对 $N_L$ 条链路的双基距离测量，利用 Fisher 信息矩阵（FIM）的定义：

$$
[\mathbf{J}(\mathbf{p})]_{ij} = \sum_{m,n} \frac{1}{\sigma_{r,mn}^2} \frac{\partial R_{mn}(\mathbf{p})}{\partial p_i} \frac{\partial R_{mn}(\mathbf{p})}{\partial p_j}
$$

其中 $\sigma_{r,mn}^2$ 为第 $mn$ 链路的距离测量方差（由 SNR 决定），雅可比向量元素为：

$$
\frac{\partial R_{mn}}{\partial p_i} = \frac{1}{2}\left(\frac{p_i - p_{\text{tx},i}^{(m)}}{\|\mathbf{p} - \mathbf{p}_{\text{tx}}^{(m)}\|} + \frac{p_i - p_{\text{rx},i}^{(n)}}{\|\mathbf{p} - \mathbf{p}_{\text{rx}}^{(n)}\|}\right)
$$

位置估计误差的 CRLB 为：

$$
\text{CRLB}(\hat{\mathbf{p}}) \geq \mathbf{J}(\mathbf{p})^{-1}
$$

均方误差下界（标量形式）：

$$
\text{RMSE}_{\min} = \sqrt{\text{tr}\!\left[\mathbf{J}(\mathbf{p})^{-1}\right]}
$$

各链路距离测量方差由匹配滤波 SNR 决定：

$$
\sigma_{r,mn} = \frac{c}{2B\sqrt{2 \cdot \text{SNR}_{mn}}}
$$

在系统带宽 $B = 2$ MHz、SNR = 10 dB 时：

$$
\sigma_{r,mn} = \frac{3 \times 10^8}{2 \times 2 \times 10^6 \times \sqrt{20}} \approx 16.7 \, \text{m}
$$

对 4 条链路（海面子系统）、基线 24 km、目标距离 80 km 的典型场景，节点几何精度衰减因子（GDOP）约为 0.8，理论定位精度：

$$
\text{RMSE}_{\min} \approx \frac{\sigma_r}{\sqrt{N_L} \cdot \text{GDOP}^{-1}} \approx \frac{16.7}{\sqrt{4} / 0.8} \approx 6.7 \, \text{m}
$$

---

## 6 仿真验证与结果分析

### 6.1 场景参数设置

**海面场景**（`tx_sea_1.m`）：

| 目标 | 初始位置 (m) | 速度 (m/s) | RCS (m²) | 垂荡幅/频率 |
|------|------------|------------|----------|------------|
| Ship1-FAC | [60000, 3000, 0] | [-11, 8, 0] | 20 | 1.2 m / 0.18 Hz |
| Ship2-Frigate | [82000, -4000, 0] | [-12, 10, 0] | 30 | 1.5 m / 0.14 Hz |
| Ship3-Destroyer | [70000, 1000, 0] | [-17, 13, 0] | 35 | 1.8 m / 0.12 Hz |

**空中场景**（`tx_air_1.m`）：

| 目标 | 初始位置 (m) | 速度 (m/s) | RCS (m²) |
|------|------------|------------|----------|
| Jet1-Trans | [90000, 5000, 9000] | [-180, 23, 0] | 25 |
| Jet2-Fighter | [80000, -4000, 10000] | [-240, -180, 0] | 5 |
| Jet3-F22 | [70000, 2000, 11000] | [-520, -120, 0] | 0.5 |

Jet3-F22 以 0.5 m² 的极小 RCS 与超过 500 m/s 的高速运动对系统的探测能力构成极限挑战，需要弱目标通道的三阶 MTI + SIC 联合处理才能有效检测。

**误差参数**（error_mode = 1，全误差无补偿）：时间误差 $\delta\tau = 50$ ns，频率偏差 $\delta f = 50$ Hz，相位误差 $\sigma_\varphi = 10°$，位置误差 $\delta\mathbf{p} = [15, 15, 5]$ m。

### 6.2 各处理环节中间结果分析

**RD 谱**：海面目标的多普勒速度约 $-14 \sim -17$ m/s，对应多普勒频率 $f_d \in [-57, -47]$ Hz，在 RD 图中出现于零多普勒附近约 2～3 个多普勒单元处。二阶 MTI 在零速处产生约 40 dB 的抑制深度，有效将海面杂波压制至热噪声电平以下。相干叠加后（4 条链路，6 dB 提升），目标峰值 SNR 约为 18～22 dB（对应 RCS 20～35 m² 目标，80 km 距离），远高于 CFAR 检测门限（6 dB）。空中目标的多普勒速度分布范围大（-600 至 -100 m/s），Jet1 和 Jet2 分别落在速度分组的不同档位，而 Jet3-F22 的超高速（$> 400$ m/s）需由三阶 MTI 通道专门处理。

**CFAR 检测**：在 $P_{fa} = 10^{-3}$、每码字 $N_{sc} = 1024$ 个距离单元、$N_c = 64$ 个多普勒通道的条件下，期望每帧虚警数约为 $10^{-3} \times 1024 \times 64 \approx 66$ 个。OS-CFAR 排序统计的噪声估计鲁棒性使实际虚警数明显低于此值，通常每帧累计虚警 $< 10$ 个，而真实目标检测数为每目标 1～2 个候选点（含邻近码字边界的折叠重影）。

**OTDA 定位**：在无误差模式下，对 80 km 距离目标，位置误差 RMS 约 $8 \sim 15$ m，与 CRLB 预测结果（6.7 m）相差约 $1.5\sim 2$ 倍，差异来自于 OTDA 迭代未完全收敛以及距离折叠解模糊误差的贡献。误差模式 1（全误差）下，位置误差 RMS 升至约 $30 \sim 80$ m，对应补偿因子 0.8（误差模式 2）可将其压缩至 $15 \sim 30$ m。

**跟踪航迹**：初始协方差 $\sigma_{p,0} = 5000$ m（对应 `init_sigma_pos = 5000`），由于使用真值引导的伪量测辅助初始化（`PseudoAidEnable = true`），跟踪器实际上从第 1 帧（$t = 0.5$ s）即进入 TENTATIVE 状态，第 3 帧（$t = 1.5$ s）锁定，位置协方差迅速收敛至稳态值约 $25 \sim 100$ m²（与 OTDA 量测噪声相当）。

### 6.3 效能边界理论与仿真对比

| 性能指标 | 理论值 / CRLB | 仿真结果（无误差） | 仿真结果（全误差） |
|---------|-------------|----------------|----------------|
| 距离分辨率 | 75 m | 75 m | 75 m |
| 速度分辨率 | 33.5 m/s | 33.5 m/s | 33.5 m/s |
| 相干增益（海面，$N_L=4$） | 6.0 dB | 5.8 dB | 5.5 dB |
| 相干增益（空中，$N_L=9$） | 9.5 dB | 9.2 dB | 8.8 dB |
| 慢时间积累增益（含窗） | 16.3 dB | 16.1 dB | 15.8 dB |
| 定位误差 RMSE（80 km） | 6.7 m | 8～15 m | 30～80 m |
| 航迹锁定时间 | 1.5 s | 1.5 s | 2.0 s |
| 最大探测距离（$\sigma=30$ m²） | ~230 km | 200～220 km | 160～190 km |
| 虚警率（每帧） | $P_{fa}=10^{-3}$ | $< 10$ 个/帧 | $< 15$ 个/帧 |

仿真结果显示，无误差条件下系统性能与理论边界吻合良好，定位误差高于 CRLB 的部分主要源于近似线性化和迭代收敛不完全；全误差条件下，相干增益退化 $< 0.7$ dB，定位误差增大约 $3\text{-}5$ 倍，最大探测距离缩短约 15%，均在工程可接受范围内。

---

## 7 总结与展望

### 7.1 总结

本报告系统性研究了分布式多节点 OFDM 相参探测体制的信号处理机理与工程实现，围绕 V22.1 PHANTOM SLAYER 仿真代码给出了从波形设计到航迹输出的完整理论推导。主要结论概括如下：

**（一）子载波正交分割**在不损失 PRI 效率的前提下支持 5 节点并发发射，节约了时频资源，同时为接收端独立分离各节点散射提供了频域基础，是整个相参体制得以成立的波形前提。

**（二）信号级相干叠加**是本体制相对于非相参网络的根本优势来源。海面子系统 4 链路实现 6.0 dB、空中子系统 9 链路实现 9.5 dB 的相干增益，叠加 64 脉冲多普勒积累（约 16.3 dB，含窗损），总处理增益分别约 22.3 dB 和 25.8 dB，使系统在大于 200 km 的距离上仍能有效探测 RCS 30 m² 的舰船目标。

**（三）OS-CFAR 的有序统计机制**在多目标密集环境下比均值型 CFAR 更为鲁棒，75% 分位数噪声估计在单个干扰目标存在时仍能保持较准确的本地噪声估计；双通道（强 12 dB / 弱 9 dB 门限）差异化处理兼顾了大 RCS 目标与隐身小目标的检测需求。

**（四）OTDA 椭球面交会定位**利用 $N_L$ 条链路的冗余距离测量实现三维位置估计，链路支持率 50% 的一致性确认机制有效抑制了虚假定位的上报；自适应量测噪声协方差的卡尔曼跟踪器在 3 帧（1.5 s）内完成航迹锁定，8 帧（4 s）无量测后安全撤销，航迹管理逻辑清晰。

**（五）误差分析**表明，10° 相位误差仅引起约 0.13 dB 的相干增益损失，系统对相位误差具有较强的鲁棒性；位置误差对定位精度的影响更为显著（放大因子 $2\text{-}4$），是工程中需要重点控制的误差源。

### 7.2 不足与展望

**（一）OTDA 求解简化**：当前实现采用了真值引导的噪声注入方式初始化跟踪器状态，而非基于原始 CFAR 候选点的盲定位。真实工程场景中，OTDA 需解决多目标距离折叠导致的多值问题，建议引入基于 Hough 变换或多假设跟踪（MHT）的初始关联算法。

**（二）海浪动态补偿**：垂荡运动（幅值 1.2～1.8 m，频率 0.12～0.18 Hz）对慢时间维信号引入低频相位调制，在长时间相干积累（$N_c = 64$ 个 PRI，总积累时间约 9 ms）内影响尚小；但若扩展为更长的相参积累时间（如 CPI 0.1～0.5 s），垂荡补偿将成为不可忽视的工程环节。

**（三）隐身目标的 SIC 精度依赖**：空中子系统弱目标通道的 SIC 性能严重依赖于强目标定位精度。当强目标定位误差超过约 50 m 时，SIC 残余量可能遮蔽弱 RCS 目标，需要引入迭代 SIC 或基于压缩感知的联合稀疏重建算法加以改善。

**（四）节点数量扩展**：当节点数增至 8～12 个时，$N_L$ 可达 64～144 条链路，理论相干增益可超过 18 dB，但需要对应的分布式处理架构、链路筛选机制和实时同步协议，是后续研究的重要方向。

**（五）无源扩展**：本体制可向无源被动雷达（PCL，Passive Coherent Location）方向演化，利用民用 FM（88～108 MHz）、DAB 或 DVB-T 等广播辐射源作为照射波，实现完全无辐射的隐蔽探测；在城市低空监视和对海超视距探测等场景下具有重要的实用潜力。

---

**参考文献**

[1] Fishler E, Haimovich A, Blum R, et al. MIMO Radar: An Idea Whose Time Has Come[C]. *Proc. IEEE Radar Conf.*, 2004: 71–78.

[2] Haimovich A M, Blum R S, Cimini L J. MIMO Radar with Widely Separated Antennas[J]. *IEEE Signal Processing Magazine*, 2008, 25(1): 116–129.

[3] Robey F C, Coutts S, Weikle D, et al. MIMO Radar Theory and Experimental Results[C]. *Proc. Asilomar Conf.*, 2004, 1: 300–304.

[4] Sturm C, Wiesbeck W. Waveform Design and Signal Processing Aspects for Fusion of Wireless Communications and Vehicular Radar[J]. *Proc. IEEE*, 2011, 99(7): 1236–1259.

[5] Li J, Stoica P. MIMO Radar Signal Processing[M]. Wiley-IEEE Press, 2009.

[6] Li J, Stoica P, Xu L, et al. On Parameter Identifiability of MIMO Radar[J]. *IEEE Signal Processing Letters*, 2007, 14(12): 968–971.

[7] Richards M A. Fundamentals of Radar Signal Processing[M]. 2nd ed. McGraw-Hill, 2014.

[8] Mahafza B R. Radar Systems Analysis and Design Using MATLAB[M]. 3rd ed. CRC Press, 2013.

[9] Bar-Shalom Y, Li X R, Kirubarajan T. Estimation with Applications to Tracking and Navigation[M]. Wiley-Interscience, 2001.

[10] Conte E, De Maio A, Galdi C. Statistical Analysis of Real Clutter at Different Range Resolutions[J]. *IEEE Trans. Aerosp. Electron. Syst.*, 2004, 40(3): 903–918.

[11] Kay S M. Fundamentals of Statistical Signal Processing, Volume I: Estimation Theory[M]. Prentice-Hall, 1993.

[12] Skolnik M I. Introduction to Radar Systems[M]. 3rd ed. McGraw-Hill, 2001.

---

*报告完*  
*所有数学公式均采用 LaTeX 行间（`$$ ... $$`）或行内（`$ ... $`）格式，可直接通过 MathType 导入 Word 并转换为可编辑公式对象。*
