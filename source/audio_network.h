#ifndef AUDIO_NETWORK_HEADER
#define AUDIO_NETWORK_HEADER

#include "asio.hpp"
#include "audio_interface.h"
#include <array>
#include <bitset>
#include <chrono>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

/*                    Packet Frame Format
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  sender id   |    channels   |  sample rate  |  magic number  |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  receiver id |                     reserved                   |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                            sequence                           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           timestamp                           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           timestamp                           |
+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
|                            payload                            |
|                             ....                              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
*/

struct NetPacketHeader
{
    uint8_t sender_id;
    uint8_t channels;
    uint8_t sample_rate;
    uint8_t magic_num;
    uint8_t receiver_id;
    uint8_t padding[3];
    uint32_t sequence;
    uint64_t timestamp;

    static bool validate(const char *data, size_t length);
};

struct NetStatInfos
{
    double packet_loss_rate;
    double average_jitter;
    double max_jitter;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t packets_out_of_order;
};

class NetState
{
  public:
    NetState();
    void update(uint32_t sequence, uint64_t timestamp);
    NetStatInfos get_period_stats();
    void reset();

  private:
    uint32_t last_sequence{0};
    uint64_t last_timestamp{0};
    uint64_t last_arrival_time{0};

    uint32_t packets_received{0};
    uint32_t packets_lost{0};
    double total_jitter{0.0};
    double max_jitter{0.0};

    uint32_t packets_out_of_order{0};

    uint32_t period_packets_received{0};
    uint32_t period_packets_lost{0};
    double period_total_jitter{0.0};
    uint32_t period_packets_out_of_order{0};

    uint32_t highest_sequence_seen{0};
    bool first_packet{true};

    std::chrono::steady_clock::time_point last_report_time;
};

template <typename T> constexpr std::underlying_type_t<T> enum2val(T e)
{
    return static_cast<std::underlying_type_t<T>>(e);
}

template <typename E> constexpr E val2enum(std::underlying_type_t<E> val)
{
    return static_cast<E>(val);
}

inline AudioBandWidth byte_to_bandwidth(uint8_t byte)
{
    switch (byte)
    {
    case 8:
        return AudioBandWidth::Narrow;
    case 16:
        return AudioBandWidth::Wide;
    case 24:
        return AudioBandWidth::SemiSuperWide;
    case 44:
        return AudioBandWidth::CDQuality;
    case 48:
        return AudioBandWidth::Full;
    default:
        return AudioBandWidth::Unknown;
    }
}

constexpr uint8_t NET_MAGIC_NUM = 0xBA;
constexpr uint8_t FEC_MAGIC_NUM = 0xFE;
constexpr unsigned int NETWORK_MAX_FRAMES = enum2val(AudioPeriodSize::INR_40MS) * enum2val(AudioBandWidth::Full) / 1000;
constexpr unsigned int NETWORK_MAX_BUFFER_SIZE = NETWORK_MAX_FRAMES + 2 * sizeof(NetPacketHeader);
constexpr unsigned int FEC_GROUP_SIZE = 8;         // 一个FEC组中包含的数据包数量
constexpr unsigned int FEC_MAX_PACKET_SIZE = 2048; // 单个FEC包最大大小

// FEC包头格式
struct FecPacketHeader
{
    uint8_t magic_num;     // 魔数 0xFE
    uint8_t sender_id;     // 发送者ID
    uint8_t group_id;      // FEC组ID
    uint8_t packet_count;  // 组中数据包数量
    uint32_t seq_base;     // 组中第一个数据包的序列号
    uint32_t fec_sequence; // FEC包序列号
    uint64_t timestamp;    // 时间戳

    static bool validate(const char *data, size_t length);
};

// FEC编码器
class FecEncoder
{
  public:
    FecEncoder(unsigned int max_packet_size = FEC_MAX_PACKET_SIZE);
    ~FecEncoder() = default;

    // 添加数据包到当前FEC组
    void add_packet(const char *data, size_t length, uint32_t sequence);

    // 获取FEC恢复包数据，如果组不完整则返回nullptr
    const char *generate_fec_packet(size_t &out_size);

    // 重置编码器状态
    void reset();

    // 设置FEC组大小
    void set_group_size(unsigned int size);

    // 获取当前FEC组大小
    unsigned int get_group_size() const;

    // 设置包缓冲大小
    void set_packet_size(unsigned int size);

  private:
    std::vector<char> fec_buffer;  // FEC包缓冲区
    std::vector<bool> packet_mask; // 记录已添加的包
    unsigned int packet_count;     // 当前组中的包数量
    unsigned int max_packet_size;  // 最大包大小
    uint8_t current_group_id;      // 当前FEC组ID
    uint32_t base_sequence;        // 组中第一个包的序列号
    unsigned int group_size;       // FEC组大小
};

// 存储待恢复的数据包
struct RecoveryPacket
{
    uint32_t sequence;                                  // 序列号
    std::chrono::steady_clock::time_point arrival_time; // 到达时间
    std::vector<char> data;                             // 包数据

    RecoveryPacket(uint32_t seq, const char *data_ptr, size_t size)
        : sequence(seq), arrival_time(std::chrono::steady_clock::now())
    {
        data.assign(data_ptr, data_ptr + size);
    }
};

// FEC解码器
class FecDecoder
{
  public:
    FecDecoder(unsigned int max_packet_size = FEC_MAX_PACKET_SIZE);
    ~FecDecoder() = default;

    // 处理普通数据包（需要为FEC恢复准备）
    void process_data_packet(uint32_t sequence, const char *data, size_t length);

    // 处理FEC包，尝试恢复丢失的包
    bool process_fec_packet(const FecPacketHeader *header, const char *fec_data, size_t fec_length);

    // 获取恢复的数据包，如果没有则返回nullptr
    const RecoveryPacket *get_recovered_packet();

    // 清理过期的包
    void cleanup(std::chrono::milliseconds max_age = std::chrono::milliseconds(500));

  private:
    bool try_recover_packet(uint32_t missing_seq, const char *fec_data,
                            const std::vector<const RecoveryPacket *> &group_packets);

    std::deque<RecoveryPacket> packet_buffer;     // 保存最近的数据包
    std::deque<RecoveryPacket> recovered_packets; // 已恢复的包队列
    unsigned int max_packet_size;                 // 最大包大小
};

/*                                                            Info Label Format
0                   1                   2                   3                   4                   5 6 0 1 2 3 4 5 6 7
8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           ias_ip (32 bits)                           |                               reserved |
ias_token
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           oas_ip (32 bits)                           |                               reserved |
oas_token
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                              |C|I|O| |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
C: Connected, 1-bit
I: IAS Muted, 1-bit
O: OAS Muted, 1-bit
*/

struct InfoLabel
{
    uint64_t ias_composite;
    uint64_t oas_composite;
    uint32_t flags;

  private:
    enum FlagPositions
    {
        CONNECTED_POS = 0,
        IAS_MUTED_POS = 1,
        OAS_MUTED_POS = 2
    };

    enum FlagMasks
    {
        CONNECTED_MASK = 1U << CONNECTED_POS,
        IAS_MUTED_MASK = 1U << IAS_MUTED_POS,
        OAS_MUTED_MASK = 1U << OAS_MUTED_POS
    };

  public:
    InfoLabel() : ias_composite(0), oas_composite(0), flags(0)
    {
    }

    InfoLabel(uint32_t ias_ip, uint8_t ias_token, uint32_t oas_ip, uint8_t oas_token, bool connected, bool ias_muted,
              bool oas_muted)
        : ias_composite(make_composite(ias_ip, ias_token)), oas_composite(make_composite(oas_ip, oas_token)), flags(0)
    {
        if (connected)
        {
            flags |= CONNECTED_MASK;
        }
        if (ias_muted)
        {
            flags |= IAS_MUTED_MASK;
        }
        if (oas_muted)
        {
            flags |= OAS_MUTED_MASK;
        }
    }

    InfoLabel(uint64_t ias_comp, uint64_t oas_comp, bool connected, bool ias_muted, bool oas_muted)
        : ias_composite(ias_comp), oas_composite(oas_comp), flags(0)
    {
        if (connected)
        {
            flags |= CONNECTED_MASK;
        }
        if (ias_muted)
        {
            flags |= IAS_MUTED_MASK;
        }
        if (oas_muted)
        {
            flags |= OAS_MUTED_MASK;
        }
    }

    uint32_t ias_ip() const
    {
        return extract_ip(ias_composite);
    }

    InfoLabel &set_ias_ip(uint32_t ip)
    {
        ias_composite = (ias_composite & 0xFFFFFFFF) | (static_cast<uint64_t>(ip) << 32);
        return *this;
    }

    uint8_t ias_token() const
    {
        return extract_token(ias_composite);
    }

    InfoLabel &set_ias_token(uint8_t token)
    {
        ias_composite = (ias_composite & ~0xFFULL) | token;
        return *this;
    }

    uint32_t oas_ip() const
    {
        return extract_ip(oas_composite);
    }

    InfoLabel &set_oas_ip(uint32_t ip)
    {
        oas_composite = (oas_composite & 0xFFFFFFFF) | (static_cast<uint64_t>(ip) << 32);
        return *this;
    }

    uint8_t oas_token() const
    {
        return extract_token(oas_composite);
    }

    InfoLabel &set_oas_token(uint8_t token)
    {
        oas_composite = (oas_composite & ~0xFFULL) | token;
        return *this;
    }

    bool connected() const
    {
        return (flags & CONNECTED_MASK) != 0;
    }

    InfoLabel &set_connected(bool connected)
    {
        if (connected)
        {
            flags |= CONNECTED_MASK;
        }
        else
        {
            flags &= ~CONNECTED_MASK;
        }
        return *this;
    }

    bool ias_muted() const
    {
        return (flags & IAS_MUTED_MASK) != 0;
    }

    InfoLabel &set_ias_muted(bool muted)
    {
        if (muted)
            flags |= IAS_MUTED_MASK;
        else
            flags &= ~IAS_MUTED_MASK;
        return *this;
    }

    bool oas_muted() const
    {
        return (flags & OAS_MUTED_MASK) != 0;
    }

    InfoLabel &set_oas_muted(bool muted)
    {
        if (muted)
            flags |= OAS_MUTED_MASK;
        else
            flags &= ~OAS_MUTED_MASK;
        return *this;
    }

    uint8_t itok() const
    {
        return ias_token();
    }

    InfoLabel &set_itok(uint8_t token)
    {
        return set_ias_token(token);
    }

    uint8_t otok() const
    {
        return oas_token();
    }

    InfoLabel &set_otok(uint8_t token)
    {
        return set_oas_token(token);
    }

    bool operator<(const InfoLabel &other) const
    {
        if (ias_composite < other.ias_composite)
        {
            return true;
        }
        else if (ias_composite > other.ias_composite)
        {
            return false;
        }
        else
        {
            return oas_composite < other.oas_composite;
        }
    }

    bool operator>(const InfoLabel &other) const
    {
        if (ias_composite > other.ias_composite)
        {
            return true;
        }
        else if (ias_composite < other.ias_composite)
        {
            return false;
        }
        else
        {
            return oas_composite > other.oas_composite;
        }
    }

    bool operator==(const InfoLabel &other) const
    {
        return ias_composite == other.ias_composite && oas_composite == other.oas_composite;
    }

    static uint32_t extract_ip(uint64_t composite)
    {
        return static_cast<uint32_t>(composite >> 32);
    }

    static uint8_t extract_token(uint64_t composite)
    {
        return static_cast<uint8_t>(composite & 0xFF);
    }

    static uint64_t make_composite(uint32_t ip, uint8_t token)
    {
        return (static_cast<uint64_t>(ip) << 32) | token;
    }
};

struct KFifo
{
  public:
    KFifo(size_t blk_sz, size_t blk_num, unsigned int channel);
    ~KFifo();

    KFifo(const KFifo &) = delete;
    KFifo &operator=(const KFifo &) = delete;

    bool store(const char *input_addr, size_t write_length);
    bool load(char *output_addr, size_t read_length);
    bool store_aside(size_t write_length);
    bool load_aside(size_t read_length);
    char *data() const
    {
        return buffer_addr;
    }

  public:
    const unsigned int chan;
    const size_t buf_length;
    const size_t max_length;
    unsigned int idle_count;

  private:
    size_t head;
    size_t tail;
    size_t length;
    char *memory_addr;
    char *buffer_addr;
    std::mutex io_mtx;

    size_t available_space() const
    {
        return max_length - length;
    }
};

class NetEncoder
{
  private:
    struct State
    {
        int32_t predictor{0};
        int step_index{0};
    };

  public:
    NetEncoder(unsigned int channels, unsigned int max_frames);
    const uint8_t *encode(const int16_t *pcm_data, unsigned int frames, size_t &out_size);
    void reset() noexcept;

  public:
    const unsigned int channels;
    const unsigned int max_frames;

  private:
    uint8_t encode_sample(int16_t sample, State &state);
    size_t calculate_encoded_size(unsigned int frames) const noexcept;

    std::vector<State> encode_states;
    std::unique_ptr<uint8_t[]> encode_buffer;
};

class NetDecoder
{
  private:
    struct State
    {
        int32_t predictor{0};
        int step_index{0};
    };

  public:
    NetDecoder(unsigned int channels, unsigned int max_frames);
    const int16_t *decode(const uint8_t *adpcm_data, size_t adpcm_size, unsigned int &out_frames);
    void reset() noexcept;

  public:
    const unsigned int channels;
    const unsigned int max_frames;

  private:
    int16_t decode_sample(uint8_t code, State &state);

    std::vector<State> decode_states;
    std::unique_ptr<int16_t[]> decode_buffer;
};

class NetWorker : public std::enable_shared_from_this<NetWorker>
{
  public:
    using ReceiveCallback = std::function<void(uint8_t sender_id, unsigned int channels, unsigned int frames,
                                               unsigned int sample_rate, const int16_t *data, uint32_t source_ip)>;
    using ReceiverContext = ReceiveCallback;

  private:
    struct Destination
    {
        asio::ip::udp::endpoint endpoint;
        uint8_t receiver_token;

        Destination(asio::ip::udp::endpoint ep, uint8_t token) : endpoint(std::move(ep)), receiver_token(token)
        {
        }

        bool operator==(const Destination &other) const
        {
            return endpoint == other.endpoint && receiver_token == other.receiver_token;
        }
    };

    struct SenderContext
    {
        std::unique_ptr<NetEncoder> encoder;
        std::vector<Destination> destinations;
        unsigned int channels;
        unsigned int sample_rate;
        unsigned int isequence;
        // FEC相关
        std::unique_ptr<FecEncoder> fec_encoder;
        bool fec_enabled;

        SenderContext(unsigned int ch, unsigned int sr)
            : encoder(std::make_unique<NetEncoder>(ch, NETWORK_MAX_FRAMES)), channels(ch), sample_rate(sr),
              isequence(0), fec_encoder(std::make_unique<FecEncoder>()), fec_enabled(false)
        {
        }
    };

    struct DecoderContext
    {
        std::unique_ptr<NetDecoder> decoder;
        std::unique_ptr<FecDecoder> fec_decoder;
        NetState stats;

        DecoderContext(unsigned int ch, unsigned int max_frames)
            : decoder(std::make_unique<NetDecoder>(ch, max_frames)), fec_decoder(std::make_unique<FecDecoder>())
        {
        }

        void update_stats(uint32_t sequence, uint64_t timestamp)
        {
            stats.update(sequence, timestamp);
        }

        NetStatInfos get_period_stats()
        {
            return stats.get_period_stats();
        }
    };

  public:
    explicit NetWorker(asio::io_context &io_context, uint16_t port);
    ~NetWorker();

    RetCode start();
    RetCode stop();

    RetCode register_sender(uint8_t sender_id, unsigned int channels, unsigned int sample_rate);
    RetCode unregister_sender(uint8_t sender_id);
    RetCode send_audio(uint8_t sender_id, const int16_t *data, unsigned int frames);

    // 启用或禁用FEC
    RetCode enable_fec(uint8_t sender_id, bool enable);
    // 设置FEC参数
    RetCode set_fec_params(uint8_t sender_id, unsigned int group_size);

    RetCode add_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port);
    RetCode del_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port);

    RetCode register_receiver(uint8_t token, ReceiveCallback callback);
    RetCode unregister_receiver(uint8_t token);

    void report_conns(std::vector<InfoLabel> &result);

  private:
    void report();
    void start_receive_loop();
    void start_stats_loop();
    void handle_receive(const asio::error_code &error, std::size_t bytes);
    void retry_receive_with_backoff();
    DecoderContext &get_decoder(uint8_t sender_id, unsigned int channels);

    // 处理FEC数据包
    void handle_fec_packet(const FecPacketHeader *header, const char *data, size_t bytes);
    // 发送FEC数据包
    void send_fec_packet(uint8_t sender_id, const char *data, size_t length);
    // 处理并分发已解码的音频数据
    void process_and_deliver_audio(uint8_t sender_id, uint8_t receiver_id, uint8_t channels, uint32_t sequence,
                                   uint64_t timestamp, const uint8_t *adpcm_data, size_t adpcm_size, uint32_t source_ip,
                                   AudioBandWidth sample_enum);

    bool is_ready() const
    {
        return running && socket && socket->is_open();
    }

  private:
    int retry_count;

    asio::io_context &io_context;
    std::atomic<bool> running;

    std::unique_ptr<asio::ip::udp::socket> socket;
    std::unique_ptr<char[]> receive_buffer;
    asio::ip::udp::endpoint sender_endpoint;
    std::map<uint8_t, SenderContext> senders;
    std::mutex senders_mutex;

    std::map<uint8_t, DecoderContext> decoders;
    std::mutex decoders_mutex;
    std::map<uint8_t, ReceiverContext> receivers;
    std::mutex receivers_mutex;

    asio::steady_timer stats_timer;
};

#endif