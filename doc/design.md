# BettyAudio 设计文档

## 1. 架构概述

BettyAudio 是一个多功能音频处理库，提供完整的音频输入/输出流管理、设备监控、网络传输和音频处理功能。该库采用现代 C++ 设计，使用基于 token 的机制来识别和管理音频流，支持本地和远程音频处理。

## 2. 核心组件

BettyAudio 的架构由以下几个核心组件组成，每个组件负责特定的功能区域：

### 2.1 核心类图及依赖关系

#### 详细类图

```
                     +----------------+
                     |  AudioCenter   |
                     +-------+--------+
                             |
         +----------+--------+---------+------------+
         |          |        |         |            |
+--------v--+ +-----v----+ +-v------+ +v--------+ +-v-------+
| IAStream  | | OAStream | |AudioPlayer| |NetWorker| |AudioMonitor|
+---+---+---+ +---+---+--+ +--+-----+ +----+----+ +-----+-----+
    |   |         |   |       |           |            |
+---v-+ |     +---v-+ |    +--v----+      |       +----v-----+
|KFifo| |     |KFifo[]|    |WaveDevice    |       |Device    |
+-----+ |     +------+     +-------+      |       |Notification|
        |                                 |       +------------+
+-------v--+  +----------+               |
|AudioDevice|  |AudioDevice|              |
+-----+-----+  +-----+----+               |
      |              |                    |
      |              |                    |
+-----v------+       |                    |
|LocSampler  |-------+--------------------|
+-----+------+                            |
      |                                   |
      v                                   |
+-----+------+                            |
|DRCompressor|<---------------------------+
+------------+
```

#### 类之间的依赖关系图

```
                   +-------------+
                   | AudioCenter |
                   +------+------+
                          |
    +----------+----------+----------+----------+
    |          |          |          |          |
    v          v          v          v          v
+-------+ +--------+ +--------+ +--------+ +--------+
|IAStream| |OAStream| |AudioPlayer| |NetWorker| |AudioMonitor|
+---+---+ +---+----+ +----+----+ +----+----+ +----+----+
    |         |           |           |           |
    |         |           |           |           |
    v         v           v           v           v
+-------------+       +--------+  +--------+  +--------+
| AudioDevice |<------| KFifo  |  |NetEncoder/| |Device  |
+------+------+       +--------+  |NetDecoder | |Notification|
       |                          +--------+  +--------+
       |                               ^
       v                               |
+------+-----+                         |
|LocSampler  |-------------------------+
+------+-----+                         |
       |                               |
       v                               |
+------+-----+                         |
|DRCompressor|<------------------------+
+------------+
```

### 2.2 组件层次结构

1. **控制层**
   - `AudioCenter`: 整个音频系统的中央控制器
   
2. **网络层**
   - `NetWorker`: 网络传输组件
     - `NetEncoder`: 网络音频编码
     - `NetDecoder`: 网络音频解码
     - `NetPacketHeader`: 网络包头定义

3. **流处理层**
   - `IAStream`: 输入音频流
   - `OAStream`: 输出音频流
   - `SessionContext`: 会话上下文（在OAStream中用于管理每个输入连接）
   
4. **设备层**
   - `AudioDevice`: 音频设备抽象
     - `AlsaDriver`: Linux 下的 ALSA 设备实现
     - `WasapiDriver`: Windows 下的 WASAPI 设备实现
     - `WaveDevice`: WAV 文件设备
     - `EchoDevice`: 回声设备
     - `NullDevice`: 空设备
   - `AudioMonitor`: 设备监控
     - `IMMNotificationClient`: Windows 设备通知
     - `UdevNotificationHandler`: Linux 设备通知
   
5. **处理层**
   - `AudioPlayer`: 音频播放
   - `DRCompressor`: 动态范围压缩
   - `LocSampler`: 采样率转换
   
6. **基础设施层**
   - `BackgroundService`: 后台服务
   - `KFifo`: 循环缓冲区
   - `InfoLabel`: 连接状态信息

## 3. 组件详细设计

### 3.1 AudioCenter

中央控制器，负责整个音频系统的管理和协调。

**职责**:
- 管理音频流的生命周期
- 建立和断开流之间的连接
- 控制整个系统的启动和停止
- 提供音量控制和静音功能
- 播放音频文件
- 周期性报告连接状态

**核心成员变量**:
- `std::unique_ptr<AudioMonitor> monitor`: 设备监控器
- `std::shared_ptr<AudioPlayer> player`: 音频播放器
- `std::shared_ptr<NetWorker> net_mgr`: 网络管理器
- `std::map<IToken, std::shared_ptr<IAStream>> ias_map`: 输入流映射
- `std::map<OToken, std::shared_ptr<OAStream>> oas_map`: 输出流映射
- `std::atomic<State> center_state`: 中心状态

**主要接口**:
- `create()`: 创建输入/输出流
- `connect()/disconnect()`: 连接/断开流
- `start()/stop()`: 启动/停止音频处理
- `mute()/unmute()`: 静音/取消静音
- `play()`: 播放音频文件
- `report_connections()`: 报告连接状态

**状态管理**:
- `INIT`: 初始化状态
- `CONNECTING`: 连接状态
- `READY`: 就绪状态

**状态转换流程**:
1. 构造时设置为 `INIT` 状态
2. 调用 `prepare()` 转换为 `CONNECTING` 状态
3. 调用 `start()` 转换为 `READY` 状态
4. 调用 `stop()` 转换回 `INIT` 状态

### 3.2 IAStream (输入音频流)

管理从音频输入设备获取的音频数据。

**职责**:
- 从音频设备读取数据
- 处理音频数据（体积调整、采样率转换等）
- 将处理后的数据分发到连接的输出流
- 支持网络传输

**核心成员变量**:
- `IToken token`: 流标识符
- `std::unique_ptr<AudioDevice> device`: 音频设备
- `std::unique_ptr<LocSampler> sampler`: 采样率转换器
- `std::unique_ptr<DRCompressor> compressor`: 动态范围压缩器
- `std::atomic<bool> muted`: 静音状态
- `std::atomic<bool> running`: 运行状态
- `std::set<std::weak_ptr<OAStream>, std::owner_less<>> oas_set`: 输出流集合
- `KFifo<int16_t> fifo`: 缓冲区

**主要接口**:
- `start()/stop()`: 启动/停止流
- `connect()/disconnect()`: 连接/断开输出流
- `mute()/unmute()`: 静音/取消静音
- `reset()`: 重置设备
- `set_volume()`: 设置音量
- `name()`: 获取设备名
- `initialize_network()`: 初始化网络
- `register_callback()`: 注册回调
- `report_conns()`: 报告连接状态

**数据流程**:
1. 从 `AudioDevice` 读取原始音频数据
2. 通过 `LocSampler` 进行采样率转换（如需要）
3. 应用音量设置
4. 通过 `DRCompressor` 进行动态范围压缩
5. 将处理后的数据发送到所有连接的 `OAStream`

### 3.3 OAStream (输出音频流)

管理音频输出设备和播放过程。

**职责**:
- 接收来自输入流的数据
- 混合多个输入源的音频
- 将处理后的数据发送到输出设备
- 支持网络传输

**核心成员变量**:
- `OToken token`: 流标识符
- `std::unique_ptr<AudioDevice> device`: 音频设备
- `std::unique_ptr<DRCompressor> compressor`: 动态范围压缩器
- `std::atomic<bool> muted`: 静音状态
- `std::atomic<bool> running`: 运行状态
- `std::map<IToken, SessionContext> input_sessions`: 会话上下文
- `std::map<uint64_t, context_ptr> sessions`: 输入会话集合，使用复合ID作为键
- `KFifo <int16_t> fifo`: 基础缓冲区

**SessionContext结构**:
- `bool enabled`: 会话是否启用
- `SessionData session`: 会话数据缓冲区 (KFifo实例)
- `LocSampler sampler`: 采样率转换器

**主要接口**:
- `start()/stop()`: 启动/停止流
- `direct_push()`: 直接推送音频数据
- `mute()/unmute()`: 静音/取消静音
- `reset()`: 重置设备
- `set_volume()`: 设置音量
- `name()`: 获取设备名
- `initialize_network()`: 初始化网络
- `register_listener()`: 注册监听器
- `report_conns()`: 报告连接状态

**数据处理流程**:
1. 从各个会话接收数据，每个会话都有自己的KFifo缓冲区
2. 混合所有输入源的数据（考虑到每个源的音量和静音状态）
3. 通过 `DRCompressor` 处理混合后的数据
4. 将处理后的数据写入输出设备

### 3.4 BackgroundService

提供异步操作的基础设施。

**职责**:
- 管理 Asio 上下文和线程池
- 提供计时器功能
- 管理高分辨率计时器（Windows）

**核心成员变量**:
- `asio::io_context io_context`: Asio I/O 上下文
- `std::vector<std::thread> io_thds`: 工作线程池

**设计模式**:
- 单例模式，通过 `instance()` 访问实例
- 提供 `context()` 方法返回 `io_context` 引用

**关键方法**:
- `start()`: 启动服务
- `stop()`: 停止服务
- `schedule()`: 安排定时任务

### 3.5 AudioDevice

音频设备的抽象基类，提供统一的接口。

**派生类**:
- `AlsaDriver`: Linux 下的 ALSA 设备实现
- `WasapiDriver`: Windows 下的 WASAPI 设备实现
- `WaveDevice`: WAV 文件设备
- `EchoDevice`: 回声设备
- `NullDevice`: 空设备

**核心成员变量**:
- `AudioDeviceName name`: 设备名称
- `AudioConfig config`: 设备配置
- `std::atomic<bool> running`: 运行状态

**主要接口**:
- `open()`: 打开设备
- `start()`: 启动设备
- `stop()`: 停止设备
- `read()`: 读取数据
- `write()`: 写入数据
- `name()`: 获取设备名
- `reset()`: 重置设备
- `handle_error()`: 处理错误

**平台特定实现**:
- **Windows**: 使用 WASAPI 接口，支持独占和共享模式
- **Linux**: 使用 ALSA 接口，支持硬件和软件混合

### 3.6 AudioMonitor

监控系统音频设备的变化。

**职责**:
- 检测设备插拔
- 提供设备列表
- 提供设备变更回调

**核心成员变量**:
- `std::mutex callback_mutex`: 回调互斥锁
- `std::function<void(AudioDeviceEvent, const AudioDeviceInfo&)> callback`: 设备变更回调
- 平台特定成员

**主要接口**:
- `RegisterCallback()`: 注册回调
- `UnregisterCallback()`: 注销回调
- `GetDevices()`: 获取设备列表

**平台特定实现**:
- **Windows**: 使用 `IMMNotificationClient` 接口
- **Linux**: 使用 `UdevNotificationHandler` 监控设备变化

**设备通知流程**:
1. 监听系统设备事件
2. 当设备改变时调用 `notify()` 方法
3. 通过用户注册的回调通知应用程序
4. AudioCenter 接收通知并更新 dummy 流的设备

### 3.7 NetWorker

处理音频数据的网络传输。

**职责**:
- 建立网络连接
- 发送和接收音频数据
- 管理目标端点
- 处理网络错误和重连
- 提供音频编解码服务

**核心成员变量**:
- `uint16_t port`: 监听端口
- `std::atomic<bool> running`: 运行状态
- `asio::io_context &io_context`: I/O 上下文
- `std::unique_ptr<asio::ip::udp::socket> socket`: UDP 套接字
- `std::map<uint8_t, SenderContext> senders`: 发送者上下文
- `std::map<uint8_t, ReceiverContext> receivers`: 接收者回调
- `std::map<uint8_t, DecoderContext> decoders`: 解码器上下文

**主要接口**:
- `start()/stop()`: 启动/停止网络服务
- `add_destination()/del_destination()`: 添加/删除目标
- `send_audio()`: 发送音频数据
- `receive_audio()`: 接收音频数据
- `register_sender()/unregister_sender()`: 注册/注销发送者
- `register_receiver()/unregister_receiver()`: 注册/注销接收者
- `report_conns()`: 报告连接状态

**SenderContext结构**:
- `std::unique_ptr<NetEncoder> encoder`: 音频编码器
- `std::vector<Destination> destinations`: 目标端点列表
- `unsigned int channels`: 通道数
- `unsigned int sample_rate`: 采样率
- `unsigned int isequence`: 序列号计数器

**DecoderContext结构**:
- `std::unique_ptr<NetDecoder> decoder`: 音频解码器
- `统计信息`: 包含丢包率、抖动、接收包数、丢失包数等

**网络包格式**:
- `NetPacketHeader`: 网络包头
  - Magic Number: 用于验证包的有效性 (0xBA)
  - 版本号: 协议版本
  - 源 Token: 发送方标识
  - 目标 Token: 接收方标识
  - 时间戳: 音频数据的时间戳
  - 序列号: 数据包序列号
  - 标志位: 包类型、静音状态等标志
  - 数据大小: 音频数据大小

**数据处理流程**:
1. 音频数据通过 `NetEncoder` 压缩和编码
2. 添加包头信息形成完整数据包
3. 通过 UDP 发送到目标 IP 和端口
4. 接收端解析包头，验证有效性
5. 通过 `NetDecoder` 解码数据
6. 将解码后的数据通过回调传递给对应的 `OAStream`

### 3.8 AudioPlayer

管理音频文件的播放。

**职责**:
- 加载音频文件
- 将音频数据发送到输出流或网络
- 控制播放（开始/停止）

**核心成员变量**:
- `std::atomic<AudioToken> next_token`: 下一个令牌
- `std::map<std::string, std::shared_ptr<IAStream>> players`: 播放器映射

**主要接口**:
- `play()`: 播放音频文件
- `stop()`: 停止播放
- `create_player()`: 创建播放器

**播放流程**:
1. 加载并解析音频文件
2. 创建临时 `IAStream` 实例
3. 将该流连接到目标 `OAStream` 或配置网络目标
4. 从文件读取音频数据并推送到流
5. 完成播放或调用 `stop()` 时清理资源

### 3.9 工具类

#### LocSampler (本地采样器)
- **职责**: 执行采样率转换
- **接口**:
  - `resample()`: 转换采样率
  - `set_ratio()`: 设置转换比例
- **内部实现**:
  - 使用线性插值或更复杂的算法实现采样率转换
  - 支持多种质量级别的转换

#### DRCompressor (动态范围压缩器)
- **职责**:
  - 执行动态范围压缩
  - 防止音频过载
  - 优化音频质量
- **接口**:
  - `process()`: 处理音频数据
  - `set_threshold()`: 设置阈值
  - `set_ratio()`: 设置比例
- **内部实现**:
  - 使用预设阈值、比例和释放时间
  - 实时计算和应用增益

#### KFifo (循环缓冲区)
- **职责**:
  - 实现循环缓冲区
  - 支持高效的数据读写
  - 处理缓冲区溢出和下溢
- **模板参数**:
  - `T`: 数据类型
- **接口**:
  - `write()`: 写入数据
  - `read()`: 读取数据
  - `peek()`: 预览数据
  - `available()`: 可读数据量
  - `space()`: 可写空间

#### InfoLabel (连接信息标签)
- **职责**:
  - 存储和管理连接状态信息
  - 支持状态比较和排序
- **核心成员变量**:
  - `uint64_t ias_composite`: 输入流复合ID
  - `uint64_t oas_composite`: 输出流复合ID
  - `bool connected`: 连接状态
  - `bool ias_muted_flag`: 输入流静音状态
  - `bool oas_muted_flag`: 输出流静音状态
- **静态方法**:
  - `extract_token()`: 从复合ID提取令牌
  - `extract_ip()`: 从复合ID提取IP

## 4. 数据流

### 4.1 本地音频流

```
            读取                      推送                       写入
输入设备 ---------> IAStream --------------> OAStream ---------> 输出设备
         |            |                       |              |
         |            |                       |              |
     原始数据      处理数据                  混合数据       最终输出
         |        (采样率转换,             (多输入源)         |
         v        音量调整,                 |                 v
       KFifo      动态压缩)                 v               音频
                     |               +---SessionContext[]---+
                     |               |    每个会话包含:     |
                     |               | KFifo + LocSampler   |
                     v               +--------------------+
                  处理后音频              应用DRCompressor
```

**详细流程**:
1. `IAStream` 从输入设备读取数据
   - 使用 `AudioDevice::read()` 方法
   - 数据暂存在 `KFifo` 中
2. 应用处理（采样率转换、音量调整等）
   - 如果需要，通过 `LocSampler` 进行采样率转换
   - 应用音量设置（乘以音量系数）
   - 使用 `DRCompressor` 进行动态范围压缩
3. 将处理后的数据发送到连接的 `OAStream`
   - 遍历 `dests` 中的所有输出流
   - 调用每个 `OAStream::direct_push()` 方法
4. `OAStream` 接收数据到每个会话的独立KFifo
   - 每个会话上下文 (`SessionContext`) 包含自己的`KFifo`和`LocSampler`
   - 会话根据输入流ID和源IP进行唯一识别
5. `OAStream` 混合输入，应用处理
   - 从每个输入会话的KFifo获取数据
   - 混合所有输入源（考虑各自音量和静音状态）
   - 应用 `DRCompressor` 进行最终处理
6. 将最终数据写入输出设备
   - 使用 `AudioDevice::write()` 方法

### 4.2 远程音频流

```
                   +--------+
                   |IAStream|
                   +---+----+
                       |
                       v
+-------------+    +---+----+                                +----------+
|原始音频数据 |--->|NetEncoder|---> UDP发送 --> [网络] ---> UDP接收 --->|NetDecoder|
+-------------+    +--------+                                +----+-----+
                                                                 |
                                                                 v
                                                            +----+-----+
                                                            |OAStream  |
                                                            |Session   |
                                                            +----+-----+
                                                                 |
                                                                 v
                                                            +----+-----+
                                                            |输出设备  |
                                                            +----------+
```

**详细流程**:
1. `IAStream` 从输入设备读取数据
2. 通过 `NetWorker` 编码和传输数据
   - `IAStream` 检查是否有网络接收者
   - 调用 `NetWorker::send_audio()` 传递数据
   - `NetWorker` 使用 `NetEncoder` 压缩音频数据
   - 添加 `NetPacketHeader` 包头
   - 通过 UDP 发送到目标 IP 和端口
3. 远端 `NetWorker` 接收和解码数据
   - 接收 UDP 数据包
   - 验证包头有效性
   - 根据发送者ID获取或创建 `DecoderContext`
   - 使用 `NetDecoder` 解压数据
4. 数据传递给远端 `OAStream`
   - 通过回调函数调用对应接收者的 `direct_push()`
   - 在 `OAStream` 中创建会话上下文，包含独立的 `KFifo`
5. 远端 `OAStream` 正常处理和播放数据

## 5. 关键机制

### 5.1 Token 系统

BettyAudio 使用 Token 机制标识和管理不同的音频流:

- **Token 类型**:
  - `IToken`: 输入流标识（0-100）
  - `OToken`: 输出流标识（101-200）
  - `AudioToken`: 通用流标识（基类）

- **特殊 Token**:
  - `USR_DUMMY_IN` (0): 用于设备监控的虚拟输入流
  - `USR_DUMMY_OUT` (101): 用于设备监控的虚拟输出流

- **Token 操作符**:
  - `_itk` 后缀: 创建 `IToken`
  - `_otk` 后缀: 创建 `OToken`

- **复合 ID**:
  - 在 `InfoLabel` 中使用复合 ID 存储 Token 和 IP
  - 高32位: IP 地址
  - 低32位: Token 值和其他标志

### 5.2 状态管理

- **AudioCenter 状态机**:
  ```
  +-------+    prepare()    +------------+    start()    +-------+
  | INIT  | --------------> | CONNECTING | ------------> | READY |
  +-------+                 +------------+               +-------+
      ^                                                      |
      |                                                      |
      +---------------------- stop() -----------------------+
  ```

- **线程安全**:
  - 使用 `std::atomic<>` 确保状态变更的线程安全
  - 使用 `compare_exchange_strong` 进行原子状态转换
  - 状态检查确保操作在正确的状态下执行

- **流状态**:
  - `IAStream` 和 `OAStream` 各自维护 `running` 状态
  - 通过 `start()/stop()` 方法控制状态

### 5.3 异步处理

BettyAudio 使用 Asio 库实现异步 I/O 处理:

- **后台服务**:
  - `BackgroundService` 维护一个 `asio::io_context` 实例
  - 使用线程池并行处理 I/O 事件
  - 所有组件共享相同的 I/O 上下文

- **异步模式**:
  - 网络通信使用异步读写
  - 设备输入输出使用异步回调
  - 计时器使用异步等待

- **Strand**:
  - 使用 `asio::strand` 确保某些操作的序列化执行
  - 防止多线程访问共享资源时的竞争条件

### 5.4 设备自动恢复

BettyAudio 提供完善的设备错误处理和恢复机制:

- **设备热插拔**:
  - `AudioMonitor` 监控设备变化
  - 当设备插入或移除时通知 `AudioCenter`
  - `AudioCenter` 通过 dummy 流更新设备

- **错误恢复策略**:
  - 设备错误时自动尝试重新打开
  - 缓冲区溢出时自动调整处理策略
  - 支持回退到空设备或文件设备

- **容错机制**:
  - 网络传输使用 UDP 协议，允许包丢失
  - 丢包时使用前一帧数据或静音填充
  - 错误计数和日志记录便于诊断

## 6. 示例场景

### 6.1 创建本地音频回路

```cpp
AudioCenter center;
center.create(1_itk, {"hw:0,0", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
center.create(101_otk, {"hw:0,0", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
center.prepare();
center.connect(1_itk, 101_otk);
center.start();
```

**流程解析**:
1. 创建 `AudioCenter` 实例
2. 创建输入流，绑定到设备 "hw:0,0"
3. 创建输出流，绑定到相同设备
4. 准备音频中心，转换到 `CONNECTING` 状态
5. 连接输入流和输出流
6. 启动音频处理，转换到 `READY` 状态

### 6.2 网络音频传输

```cpp
// 本地端
AudioCenter center1(true);
center1.create(1_itk, {"hw:0,0", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
center1.prepare();
center1.connect(1_itk, 101_otk, "192.168.1.2");
center1.start();

// 远程端
AudioCenter center2(true);
center2.create(101_otk, {"hw:0,0", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2, true);
center2.prepare();
center2.start();
```

**流程解析**:
1. 本地端创建启用网络的 `AudioCenter`
2. 创建输入流并启用网络
3. 准备音频中心
4. 连接输入流到远程输出流（IP: 192.168.1.2）
5. 启动本地音频处理
6. 远程端创建启用网络的 `AudioCenter`
7. 创建输出流并启用网络
8. 远程端准备并启动
9. 音频数据自动通过网络从本地传输到远程

### 6.3 播放音频文件到设备

```cpp
AudioCenter center;
center.create(101_otk, {"hw:0,0", 0}, AudioBandWidth::Full, AudioPeriodSize::INR_20MS, 2);
center.prepare();
center.start();
center.play("/path/to/audio.wav", 1, 101_otk);
```

**流程解析**:
1. 创建 `AudioCenter` 实例
2. 创建输出流绑定到设备
3. 准备并启动音频中心
4. 播放指定音频文件到输出流
   - 内部创建临时 `IAStream` 连接到指定的 `OAStream`
   - 从文件读取数据并推送到输出流
   - 播放完成后自动清理临时流

## 7. 跨平台支持

BettyAudio 提供跨平台支持，通过条件编译和抽象接口实现：

- **Windows**:
  - 使用 WASAPI 接口访问音频设备
  - 使用 `IMMNotificationClient` 监控设备变化
  - 支持低延迟独占模式和共享模式
  - 专门的高精度计时器实现

- **Linux**:
  - 使用 ALSA 接口访问音频设备
  - 使用 Udev 监控设备变化
  - 支持硬件参数调整和软件混音

- **通用**:
  - 文件设备：在所有平台使用相同的实现
  - 回声设备：提供回环测试功能
  - 空设备：当实际设备不可用时的后备方案

- **预处理宏**:
  ```cpp
  #ifdef _WIN32
      // Windows 特定实现
  #elif defined(__linux__)
      // Linux 特定实现
  #else
      // 通用后备实现
  #endif
  ```

## 8. 性能考虑

BettyAudio 设计特别注重性能优化：

- **内存管理**:
  - 使用 `KFifo` 循环缓冲区避免频繁分配和复制
  - 预分配缓冲区减少运行时内存管理开销
  - 使用 `std::shared_ptr` 和 `std::weak_ptr` 避免循环引用

- **计算效率**:
  - 采用高效的采样率转换算法
  - 压缩算法选择考虑计算复杂度和压缩率平衡
  - 批处理音频数据减少函数调用开销

- **延迟优化**:
  - 缓冲区大小可配置，允许低延迟设置
  - 避免不必要的数据复制
  - 使用高精度计时器精确调度

- **资源使用**:
  - 使用单例模式避免资源重复创建
  - 关闭不使用的流以节省资源
  - 自动检测并处理资源泄漏

- **网络优化**:
  - 使用 UDP 协议减少延迟
  - 压缩编码减少带宽使用
  - 自适应缓冲区大小应对网络抖动

## 9. 扩展性

BettyAudio 设计注重扩展性：

- **设备扩展**:
  - 通过 `AudioDevice` 抽象基类支持新设备类型
  - 实现 `open()`、`start()`、`stop()`、`read()`、`write()` 等方法
  - 无需修改核心代码即可添加新设备类型

- **编解码器扩展**:
  - 通过抽象接口支持新的音频压缩格式
  - 网络传输的编解码可独立更新

- **处理器扩展**:
  - 可添加新的音频处理模块，如均衡器、混响等
  - 遵循统一的处理接口

- **网络协议扩展**:
  - 包头设计允许协议版本升级
  - 可添加新的控制消息类型

- **接口设计**:
  - Token 系统支持扩展更多流类型
  - 公共 API 设计考虑向后兼容性
  - 使用接口而非实现进行耦合，便于替换组件