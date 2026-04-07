# 音频网络接收流程说明

## 概述

接收侧由 `NetWorker` 类驱动，通过 UDP 异步接收数据包，完成解码、丢包恢复并将 PCM 数据交付给注册的回调。整个链路分为四个主要阶段：

```
UDP 接收 → 包校验与路由 → 统计 & 序号分析 → 解码 / 丢包恢复 → 投递
```

---

## 一、UDP 异步接收循环

```
NetWorker::start_receive_loop()
    └─ socket.async_receive_from(...)
           └─ handle_receive(error, bytes, endpoint)
```

- `start_receive_loop()` 在 io_context 线程上投递一个异步读操作，读入 `receive_buffer`（最大 `NETWORK_MAX_BUFFER_SIZE` 字节）。
- 收到数据后进入 `handle_receive()`：
  - 若发生错误且非 `operation_aborted`，调用 `retry_receive_with_backoff()` 做指数退避（上限 30 s），然后重新挂起接收。
  - 正常包：校验首字节 magic number（`DataPacket::is_valid_magic_num()`），合法则解析为 `DataPacket` 并转入 `process_and_deliver_audio()`。
  - 完成处理后**立即**再次调用 `start_receive_loop()`，保持循环连续。

---

## 二、包头解析与 SourceUUID 构造

每个 UDP 包以固定长度的 `DataPacket` 头部开始（共 24 字节，内存对齐）：

| 字段 | 偏移 | 大小 | 说明 |
|---|---|---|---|
| `magic_num` | 0 | 1 B | 编解码类型 + 优先级 + 合法性标志 |
| `sender_id` | 1 | 1 B | 发送者 ID |
| `receiver_id` | 2 | 1 B | 目标接收者 token |
| `channels` | 3 | 1 B | 声道数 |
| `sample_rate` | 4 | 4 B | 采样率 |
| `sequence` | 8 | 4 B | 递增序列号（uint32，支持回绕） |
| `session_ip` | 12 | 4 B | 发送端本机 IP（用于区分 NAT 后的相同端口） |
| `timestamp` | 16 | 8 B | 发送时刻（ms，用于 jitter 计算） |

`magic_num` 编码方案（位域）：
```
bit 7-4 : 0xB（固定基础值）
bit 3-2 : 优先级 (00=LOW, 01=MEDIUM, 10=HIGH)
bit 1   : 编解码 (0=OPUS, 1=PCM)
bit 0   : 固定为 1（合法性标志）
```

接收侧从 `sender_endpoint`（网关 IP）+ `session_ip` + `sender_id` 三元组构造 `SourceUUID`，唯一标识一条上行流，支持 NAT 穿越场景。

---

## 三、统计与序号分析（`NetState`）

每条流（`SourceUUID`）对应一个 `DecoderContext`，内部持有一个 `NetState` 实例。

### 3.1 `NetState::update()` 逻辑

```
输入: sequence, timestamp
输出: stats.seq_gap (每次都写入)
      stats.packet_loss_rate / average_jitter / ... (仅周期上报时写入)
      返回值: true = 本次触发了周期上报
```

**序号差计算（支持 uint32 回绕）：**
```cpp
int32_t seq_diff = static_cast<int32_t>(sequence - highest_sequence_seen);
```
将无符号差强转为有符号 32 位整数，自动处理 `0xFFFF→0x0000` 等回绕情形。

**`seq_diff` 的含义：**

| 值 | 含义 |
|---|---|
| `< 0` | 乱序包（晚到的旧包） |
| `== 0` | 重复包（忽略） |
| `== 1` | 正常连续 |
| `>= 2` | 检测到丢包：丢失了 `seq_diff - 1` 个包 |

**Jitter 计算（参考 RFC 3550 §6.4.1）：**
- 仅对 `seq_diff >= 0` 的包计算（避免乱序包干扰）
- `jitter = |到达间隔 - 发送间隔|`，单位 ms

**周期上报（每 1 分钟一次）：**
- 统计窗口内的丢包率、平均 jitter、最大 jitter、乱序包数
- 上报后重置所有 period 计数器
- `AUDIO_INFO_PRINT` 打印 `NETSTATS` 行（仅当存在丢包或乱序时）

### 3.2 `NetStatInfos::seq_gap` 的特殊地位

`seq_gap` 是 `NetStatInfos` 中**唯一在每次调用时都有效**的字段，其他统计字段仅在函数返回 `true` 时有意义。`process_and_deliver_audio()` 使用 `seq_gap` 驱动 FEC/PLC 决策，两套系统共享同一份序号状态，无重复维护。

---

## 四、解码与丢包恢复

### 4.1 接收者查找（提前一次）

```cpp
auto receiver_it = receivers.find(receiver_id);
if (receiver_it == receivers.end()) { return; }  // 无接收者直接丢弃
const auto &deliver = receiver_it->second;       // 后续所有投递共用此引用
```

### 4.2 Opus 路径：FEC + PLC + 正常解码

丢包恢复策略由 `stats.seq_gap` 驱动，按顺序执行：

```
seq_gap <= 0  →  无动作（首包/重复/乱序）
seq_gap == 1  →  无动作（正常连续）
seq_gap >= 2  →  需要恢复：
                  ① FEC（seq_gap >= 2，始终执行）
                  ② PLC（仅当 seq_gap > 2 时，循环 seq_gap-2 次）
                  ③ 正常解码当前包
```

**① FEC（前向纠错）**

Opus 编码器在每个包中嵌入了前一个包的冗余副本（需开启 `OPUS_SET_INBAND_FEC`）。解码器可从当前包中提取这份冗余来恢复紧邻的丢失包：

```cpp
opus_decode(decoder, current_packet_data, current_packet_size,
            fec_buffer, last_frames, /*decode_fec=*/1);
```

- `last_frames`：必须与丢失包的帧数一致——因此 `last_frames` 只在正常顺序包（`seq_gap >= 0`）时更新，乱序包不允许覆盖此值。
- FEC 恢复的帧数（`fec_frames`）可能与 `last_frames` 不完全相同（取决于编码参数），以实际返回值为准。

**② PLC（包丢失隐藏）**

当 `seq_gap > 2` 时，除第一个丢包（已由 FEC 处理）外，其余丢失包用 PLC 补齐：

```cpp
// 循环 seq_gap - 2 次（i 从 1 到 seq_gap-2）
opus_decode(decoder, nullptr, 0, fec_buffer, last_frames, /*decode_fec=*/0);
```

传入 `null` 载荷通知解码器自行生成隐藏帧，保持内部状态连续，避免解码器状态与实际样本流脱节。

**③ 正常解码**

```cpp
opus_decode(decoder, opus_data, opus_size, decode_buffer, NETWORK_MAX_FRAMES, /*decode_fec=*/0);
```

解码成功后：
- 若 `seq_gap >= 0`（非乱序），更新 `ctx.last_frames = decoded_frames`
- 将解码帧投递给接收回调

### 4.3 PCM 路径

直接将 UDP payload 强转为 `int16_t*` 投递，不经过 Opus 解码器。

### 4.4 完整决策流程图

```
process_and_deliver_audio()
│
├─ 查找 receiver，未注册则 return
├─ get_decoder() — 按 SourceUUID 查找/创建 DecoderContext
├─ update_stats() — 更新 NetState，获取 seq_gap
│   └─ seq_gap 写入 stats（每次有效）
│   └─ 周期统计写入 stats（每分钟一次，返回 true）
│       └─ [若有丢包/乱序] AUDIO_INFO_PRINT NETSTATS
│
├─ [OPUS]
│   ├─ 检查 decoder 指针有效性
│   ├─ [seq_gap >= 2 && last_frames > 0]
│   │   ├─ FEC: opus_decode(..., decode_fec=1) → deliver
│   │   └─ PLC: loop(seq_gap-2 次) opus_decode(null,...) → deliver
│   ├─ 正常解码: opus_decode(..., decode_fec=0)
│   ├─ [seq_gap >= 0] 更新 last_frames
│   └─ deliver 当前帧
│
└─ [PCM]
    └─ deliver 原始 payload
```

---

## 五、关键设计决策说明

| 决策 | 原因 |
|---|---|
| `seq_gap` 使用有符号 int32_t | uint32_t 序列号相减再强转，自动正确处理回绕 |
| `last_sequence` 已删除 | `highest_sequence_seen` 已完整替代其功能，消除死字段 |
| 累计计数器（`packets_received` 等非 period 版）已删除 | 仅用于上报，period 版完全覆盖需求，避免无谓内存和分支 |
| `last_frames` 仅在 `seq_gap >= 0` 时更新 | 乱序包不代表"正常序列的最后一帧"，用乱序帧数做 FEC 参数会导致解码失败 |
| FEC 优先于 PLC | FEC 音质优于 PLC；且 Opus 协议保证 FEC 数据只能恢复紧邻前包，无法覆盖更早的丢包 |
| Receiver 查找提前到函数入口 | 若未注册 receiver 则整条路径（含统计、解码）均可跳过；消除 FEC/PLC 循环内的重复 map 查找 |
