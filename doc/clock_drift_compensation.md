# 时钟漂移闭环补偿原理

## 1. 问题根源

OAStream 的播放循环由 `asio::steady_timer` 驱动，周期标称为 `ti` ms（通常 10 ms）。
硬件声卡由独立的石英晶振控制采样时钟，两者之间存在不可消除的频率偏差（典型值 ±50～100 ppm）。

```
网络 → session FIFO → LocSampler → odevice（WASAPI/ALSA 软件缓冲）→ 硬件 DAC
         生产者              消耗者             软件计时          独立硬件时钟
```

两处时钟不对齐会导致软件缓冲水位单调漂移：

| 场景 | 结果 |
|---|---|
| timer 偏慢（<硬件速率）|  odevice 软件缓冲（wlatency）持续下降 → 欠载断声；session FIFO 持续上升 → 最终满载丢包 |
| timer 偏快（>硬件速率）| odevice 软件缓冲持续上升 → 延迟增大 |

---

## 2. 反馈量选取：`odevice->wlatency()`

`wlatency()` 返回 odevice 内部 KFifo 写端的 0.9 分位水位，单位毫秒。

```
wlatency() 低  →  软件侧写入慢（timer 偏慢）→ 硬件快，软件跟不上
wlatency() 高  →  软件侧写入快（timer 偏快）→ 硬件慢，软件超前
```

选择 odevice 而非 session FIFO 的原因：
- odevice 水位直接代表"软件计时 vs 硬件时钟"的净差，不受网络抖动干扰
- 一个反馈量统一控制所有 session，不需要每路独立 PI

目标水位设定为 `2 × ti` ms（即 2 个播放周期），在提供足够抗抖动余量的同时，保持较低的端到端延迟。

---

## 3. PI 控制器

每次 `process_data()` 被调用时执行一次 PI 更新。

### 3.1 偏差定义

```
e = target_ms - cur_ms
```

- `e > 0`：当前水位低于目标 → 需要更快排空 session FIFO（`adj > 1`）
- `e < 0`：当前水位高于目标 → 需要减慢排空（`adj < 1`）

### 3.2 积分项

```
integral += e × dt        (dt = ti / 1000 s)
integral = clamp(integral, −1000, +1000)   单位: ms·s
```

积分项消除稳态误差（例如系统性的固定频偏）。

### 3.3 输出

```
adj = 1.0 + Kp × e + Ki × integral
adj = clamp(adj, 0.995, 1.005)
```

| 参数 | 值 | 含义 |
|---|---|---|
| Kp | 1×10⁻⁴ | 比例增益；10 ms 偏差 → 0.1% 校正 |
| Ki | 1×10⁻⁴ | 积分增益；消除固定偏频，时间常数约 100 s |
| adj 范围 | [0.995, 1.005] | 对应 ±5‰ 调节幅度，远大于石英晶振百万分之百级偏差 |

---

## 4. 调节量的双重作用

单独调节 FIFO 消耗量或单独调节采样步进都会破坏下游缓冲区的尺寸约束（输出帧数变化）。
必须同时、同幅度地调节两处，使它们在 SincInterpolator 内部互消。

### 4.1 adj_frames：控制 FIFO 消耗量

```
adj_frames = round(src_fr × adj)
```

每次播放循环从 session FIFO 消耗 `adj_frames` 帧，而非固定的 `src_fr` 帧：

- `adj > 1` → 多消耗 → FIFO 排空加快 → session 缓冲水位下降，odevice 水位回升
- `adj < 1` → 少消耗 → FIFO 排空放慢 → session 缓冲水位上升，odevice 水位回落

### 4.2 effective_step：同步调节采样步进

SincInterpolator 的名义步进为：

```
step = src_fs / dst_fs
```

引入 PI 调节后，实际步进为：

```
effective_step = step × ratio_adjust_ = (src_fs / dst_fs) × adj
```

### 4.3 输出帧数恒定的证明

SincInterpolator 将 `adj_frames` 帧输入按 `effective_step` 步进采样，输出帧数为：

```
output = adj_frames / effective_step
       = (src_fr × adj) / ((src_fs / dst_fs) × adj)
       = src_fr × dst_fs / src_fs
       = dst_fr                          ← 恒定
```

`adj` 在分子分母完全互消，下游 `mix_buf`、`odevice->write()` 始终收到精确 `ps = dst_fr` 帧，无需任何缓冲区尺寸变动。

---

## 5. 缓冲区尺寸安全性

| 缓冲区 | 分配容量 | 实际写入上限 | 余量 |
|---|---|---|---|
| `analysis_ibuffer` | `src_fr × 1.01 + 2` | `adj_frames ≤ src_fr × 1.005 + 1` | > 0 ✓ |
| `analysis_obuffer` | `dst_fr × 1.01 + 2` | SincInterp 输出 `≈ dst_fr`（adj 互消）| > 0 ✓ |
| `mix_buf / oview` | `ps = dst_fr` 帧 | `interleave_f16_s16` 写 `ps` 帧 | 精确 ✓ |
| `odevice->write` | — | 始终 `ps × ch` 字节 | 不变 ✓ |

---

## 6. 收敛行为

### 启动预热

前 64 个播放周期（`oa_drift_warmup_ < 64`，约 640 ms @ 10 ms ti）内 `adj` 固定为 1.0。
原因：KFifo 的直方图统计需要至少 32 个样本（`BOOTSTRAP_MIN`）才能给出可靠分位数，预热期内水位数据不稳定。

### 稳态

设系统时钟偏差为 δ ppm（典型 |δ| < 100 ppm = 1×10⁻⁴）。

比例项在偏差产生的瞬间立即响应，积分项在约 `1/(Ki × dt)` = 100 s 后消除稳态余差。
稳态下 `|adj - 1| ≈ |δ| < 0.0001`，远低于 0.005 的限幅，不会触及边界。

### 日志观测

`query_latency()` 输出格式：

```
LAC 001 [w=18.32ms,r=4.21ms,adj=1.00023]
    00.001@0xC0A80101|0x00000000 [w=12.10ms,r=3.45ms]
```

- `w=`：odevice 软件缓冲写水位（应稳定在 `2×ti` ms 附近）
- `adj=`：当前 PI 输出（稳态应收敛至 `1.000xx`，绝对值 < 0.001）

---

## 7. 涉及代码位置

| 逻辑 | 文件 | 关键符号 |
|---|---|---|
| PI 状态 | `audio_stream.h` | `oa_drift_integral_`, `oa_drift_adj_`, `oa_drift_warmup_` |
| PI 计算 + adj_frames | `audio_stream.cpp` | `OAStream::process_data()` |
| effective_step | `audio_process.cpp` | `SincInterpolator::process()` |
| adj 传递路径 | `audio_stream.cpp` → `LocSampler::set_ratio_adjust()` → `SincInterpolator::set_ratio_adjust()` | |
| 实际输入帧传递 | `audio_process.cpp` | `LocSampler::convert_sample_rate(input, output, actual_src_frames)` |
| 缓冲区超配 | `audio_process.cpp` | `LocSampler::LocSampler()` 构造，`src_fr_max / dst_fr_max` |
