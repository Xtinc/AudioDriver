#ifndef AUDIO_NETWORK_HEADER
#define AUDIO_NETWORK_HEADER

#include "asio.hpp"
#include "audio_interface.h"
#include "opus.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Template utility functions
template <typename T> constexpr std::underlying_type_t<T> enum2val(T e)
{
    return static_cast<std::underlying_type_t<T>>(e);
}

template <typename E> constexpr E val2enum(std::underlying_type_t<E> val)
{
    return static_cast<E>(val);
}

struct SourceUUID
{
    uint32_t sender_ip;
    uint32_t gateway_ip;
    uint8_t sender_token;
    uint8_t receiver_token;

    bool operator==(const SourceUUID &other) const
    {
        return sender_ip == other.sender_ip && gateway_ip == other.gateway_ip && sender_token == other.sender_token &&
               receiver_token == other.receiver_token;
    }

    bool operator<(const SourceUUID &other) const
    {
        return std::tie(sender_ip, gateway_ip, sender_token, receiver_token) <
               std::tie(other.sender_ip, other.gateway_ip, other.sender_token, other.receiver_token);
    }
};

// Magic number encoding scheme:
// Bit 7-4: Base magic number (1011 = 0xB)
// Bit 3-2: Priority (00=LOW, 01=MEDIUM, 10=HIGH)
// Bit 1: Codec type (0=OPUS, 1=PCM)
// Bit 0: Fixed bit (1)

enum class AudioCodecType : uint8_t
{
    OPUS = 0,
    PCM = 1
};

constexpr unsigned int NETWORK_MAX_FRAMES = enum2val(AudioPeriodSize::INR_40MS) * enum2val(AudioBandWidth::Full) / 1000;
constexpr unsigned int NETWORK_MAX_BUFFER_SIZE = NETWORK_MAX_FRAMES * 4 + 320;

struct NetStatInfos
{
    double packet_loss_rate;
    double average_jitter;
    double max_jitter;
    uint32_t packets_received;
    uint32_t packets_lost;
    uint32_t packets_out_of_order;
    // Always valid (not limited to the periodic report window):
    // > 0  : in-order gap (number of missing packets before this one)
    // == 0 : consecutive, first packet, or duplicate
    // < 0  : out-of-order arrival
    int32_t seq_gap{0};
};

class NetState
{
  public:
    void update(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats);
    bool snapshot(NetStatInfos &stats);

  private:
    // Jitter tracking (RFC 3550): timestamps of the most recently processed packet
    uint64_t last_timestamp{0};
    uint64_t last_arrival_time{0};

    // Per-period counters — reset every report interval
    uint32_t period_packets_received{0};
    uint32_t period_packets_lost{0};
    double period_total_jitter{0.0};
    double period_max_jitter{0.0};
    uint32_t period_jitter_samples{0};
    uint32_t period_packets_out_of_order{0};

    uint32_t highest_sequence_seen{0};
    bool first_packet{true};
};

template <size_t N> class Histogram
{
    static_assert(N > 2, "Histogram must have at least 3 buckets");

    // Collect this many raw samples before switching to EMA mode.
    static constexpr size_t BOOTSTRAP_MIN = (N < 32) ? 32 : N;

    // EMA decay factor. Effective window ≈ 1/(1-DECAY) = 100 samples.
    static constexpr double DECAY = 0.99;

  public:
    Histogram() : count(0), boot_n(0), observed_min(0.0), observed_max(0.0)
    {
        bcnts.fill(0.0);
        scale.fill(0.0);
    }

    void add(double val)
    {
        if (boot_n < BOOTSTRAP_MIN)
        {
            collect_boot(val);
            return;
        }
        ema_update(val);
    }

    double quantile(double q) const
    {
        if (q < 0.0 || q > 1.0)
            return std::numeric_limits<double>::quiet_NaN();

        if (boot_n < BOOTSTRAP_MIN)
        {
            if (boot_n == 0)
                return std::numeric_limits<double>::quiet_NaN();
            std::array<double, BOOTSTRAP_MIN> s;
            std::copy(boot_vals.begin(), boot_vals.begin() + boot_n, s.begin());
            std::sort(s.begin(), s.begin() + boot_n);
            if (boot_n == 1)
                return s[0];
            const double pos = q * static_cast<double>(boot_n - 1);
            const size_t lo = static_cast<size_t>(pos);
            const size_t hi = std::min(lo + 1, boot_n - 1);
            return s[lo] + (pos - lo) * (s[hi] - s[lo]);
        }

        double cum = 0.0;
        for (size_t i = 0; i < N; i++)
        {
            const double next = cum + bcnts[i];
            if (q <= next || i == N - 1)
            {
                if (bcnts[i] < 1e-12)
                    return 0.5 * (scale[i] + scale[i + 1]);
                const double local = std::max(0.0, std::min(1.0, (q - cum) / bcnts[i]));
                return scale[i] + local * (scale[i + 1] - scale[i]);
            }
            cum = next;
        }
        return scale[N];
    }

  private:
    void collect_boot(double val)
    {
        observed_min = (boot_n == 0) ? val : std::min(observed_min, val);
        observed_max = (boot_n == 0) ? val : std::max(observed_max, val);
        boot_vals[boot_n++] = val;

        if (boot_n < BOOTSTRAP_MIN)
            return;

        build_scale(observed_min, observed_max);
        bcnts.fill(0.0);
        for (size_t i = 0; i < boot_n; i++)
            bcnts[bucket_of(boot_vals[i])] += 1.0;
        for (auto &c : bcnts)
            c /= static_cast<double>(boot_n); // normalize → sum == 1.0
    }

    void ema_update(double val)
    {
        if (val < scale[0])
            scale[0] = val;
        else if (val > scale[N])
            scale[N] = val;

        for (auto &c : bcnts)
            c *= DECAY;
        bcnts[bucket_of(val)] += 1.0 - DECAY;

        if (++count % 100 == 0)
            equalize();
    }

    size_t bucket_of(double val) const
    {
        if (val <= scale[0])
            return 0;
        if (val >= scale[N])
            return N - 1;
        size_t idx = 0;
        for (size_t i = 1; i < N; i++)
        {
            if (val < scale[i])
                break;
            idx = i;
        }
        return idx;
    }

    void equalize()
    {
        // Step 1: compute new interior boundaries via a single O(N) pass.
        std::array<double, N + 1> ns;
        ns[0] = scale[0];
        ns[N] = scale[N];

        double cum = 0.0;
        size_t qi = 1; // next interior boundary index to compute
        for (size_t i = 0; i < N && qi < N; i++)
        {
            const double next = cum + bcnts[i];
            while (qi < N && static_cast<double>(qi) / N <= next)
            {
                const double target = static_cast<double>(qi) / N;
                const double local = (bcnts[i] > 1e-12) ? (target - cum) / bcnts[i] : 0.0;
                ns[qi] = scale[i] + std::max(0.0, std::min(1.0, local)) * (scale[i + 1] - scale[i]);
                qi++;
            }
            cum = next;
        }
        while (qi < N)
            ns[qi++] = scale[N]; // tail guard for near-zero trailing buckets

        scale = ns;
        bcnts.fill(1.0 / N);
    }

    void build_scale(double lo, double hi)
    {
        if (hi - lo < 1e-10)
        {
            const double pad = std::max(1e-6, std::abs(lo) * 0.01);
            lo -= pad;
            hi += pad;
        }
        const double step = (hi - lo) / N;
        for (size_t i = 0; i <= N; i++)
            scale[i] = lo + i * step;
    }

    std::array<double, N> bcnts;
    std::array<double, N + 1> scale;
    uint64_t count;
    std::array<double, BOOTSTRAP_MIN> boot_vals;
    size_t boot_n;
    double observed_min;
    double observed_max;
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
    double write_water_level(double quantile = 0.9) const;
    double read_water_level(double quantile = 0.1) const;

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
    mutable std::mutex io_mtx;
    Histogram<10> whist;
    Histogram<10> rhist;

    size_t available_space() const
    {
        return max_length - length;
    }
};

struct DataPacket
{
    uint8_t magic_num;    // 1 byte  - offset 0
    uint8_t sender_id;    // 1 byte  - offset 1
    uint8_t receiver_id;  // 1 byte  - offset 2
    uint8_t channels;     // 1 byte  - offset 3
    uint32_t sample_rate; // 4 bytes - offset 4 (aligned)
    uint32_t sequence;    // 4 bytes - offset 8 (aligned)
    uint32_t session_ip;  // 4 bytes - offset 12 (aligned)
    uint64_t timestamp;   // 8 bytes - offset 16 (aligned)

    /**
     * @brief Generate magic number that encodes both codec type and priority
     * @param codec Audio codec type
     * @param priority Audio priority level
     * @return Encoded magic number
     */
    static constexpr uint8_t encode_magic_num(AudioCodecType codec, AudioPriority priority)
    {
        uint8_t base = 0xB0; // Base magic number: 1011 0000

        // Encode priority (bits 3-2)
        uint8_t priority_bits = 0;
        switch (priority)
        {
        case AudioPriority::LOW:
            priority_bits = 0;
            break; // 00
        case AudioPriority::MEDIUM:
            priority_bits = 1;
            break; // 01
        case AudioPriority::HIGH:
            priority_bits = 2;
            break; // 10
        }

        // Encode codec type (bit 1)
        uint8_t codec_bit = (codec == AudioCodecType::PCM) ? 1 : 0;

        // Combine: base | (priority << 2) | (codec << 1) | 1
        return base | (priority_bits << 2) | (codec_bit << 1) | 1;
    }

    /**
     * @brief Decode codec type from magic number
     * @param magic_num Encoded magic number
     * @return Audio codec type
     */
    static constexpr AudioCodecType decode_magic_codec(uint8_t magic_num)
    {
        return ((magic_num >> 1) & 1) ? AudioCodecType::PCM : AudioCodecType::OPUS;
    }

    /**
     * @brief Decode priority from magic number
     * @param magic_num Encoded magic number
     * @return Audio priority level
     */
    static constexpr AudioPriority decode_magic_priority(uint8_t magic_num)
    {
        uint8_t priority_bits = (magic_num >> 2) & 0x3;
        switch (priority_bits)
        {
        case 0:
            return AudioPriority::LOW;
        case 1:
            return AudioPriority::MEDIUM;
        case 2:
            return AudioPriority::HIGH;
        default:
            return AudioPriority::MEDIUM;
        }
    }

    /**
     * @brief Check if magic number is valid
     * @param magic_num Magic number to validate
     * @return True if valid
     */
    static constexpr bool is_valid_magic_num(uint8_t magic_num)
    {
        // Check format: base should be 0xB0, bit 0 should be 1
        return (magic_num & 0xF1) == 0xB1;
    }
};

class NetWorker : public std::enable_shared_from_this<NetWorker>
{
  public:
    using ReceiveCallback = std::function<void(unsigned int channels, unsigned int frames, unsigned int sample_rate,
                                               const int16_t *data, SourceUUID ssid, AudioPriority priority)>;
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
        ::OpusEncoder *encoder;
        std::unique_ptr<uint8_t[]> encode_buffer;
        std::vector<Destination> destinations;
        unsigned int channels;
        unsigned int sample_rate;
        unsigned int isequence;
        AudioCodecType codec_type;

        SenderContext(unsigned int ch, unsigned int sr, AudioCodecType codec = AudioCodecType::OPUS)
            : encoder(nullptr), channels(ch), sample_rate(sr), isequence(0), codec_type(codec)
        {
            if (codec == AudioCodecType::OPUS)
            {
                int error;
                encoder = opus_encoder_create(sr, ch, OPUS_APPLICATION_AUDIO, &error);
                if (error == OPUS_OK && encoder)
                {
                    encode_buffer = std::make_unique<uint8_t[]>(NETWORK_MAX_BUFFER_SIZE);
                    opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(1));
                    opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(5));
                }
            }
            else
            {
                encode_buffer = std::make_unique<uint8_t[]>(NETWORK_MAX_BUFFER_SIZE);
            }
        }

        ~SenderContext()
        {
            if (encoder)
            {
                opus_encoder_destroy(encoder);
            }
        }

        SenderContext(const SenderContext &) = delete;
        SenderContext &operator=(const SenderContext &) = delete;

        SenderContext(SenderContext &&other) noexcept
            : encoder(other.encoder), encode_buffer(std::move(other.encode_buffer)),
              destinations(std::move(other.destinations)), channels(other.channels), sample_rate(other.sample_rate),
              isequence(other.isequence), codec_type(other.codec_type)
        {
            other.encoder = nullptr;
        }

        SenderContext &operator=(SenderContext &&other) noexcept
        {
            if (this != &other)
            {
                encoder = other.encoder;
                encode_buffer = std::move(other.encode_buffer);
                destinations = std::move(other.destinations);
                channels = other.channels;
                sample_rate = other.sample_rate;
                isequence = other.isequence;
                codec_type = other.codec_type;

                other.encoder = nullptr;
            }
            return *this;
        }
    };

    struct DecoderContext
    {
        ::OpusDecoder *decoder;
        std::unique_ptr<int16_t[]> decode_buffer;
        std::unique_ptr<int16_t[]> fec_buffer;
        NetState stats;
        int last_frames{0};

        DecoderContext(unsigned int ch, unsigned int sr, AudioCodecType codec) : decoder(nullptr)
        {
            if (codec == AudioCodecType::OPUS)
            {
                int error;
                decoder = opus_decoder_create(sr, ch, &error);
                if (error == OPUS_OK && decoder)
                {
                    decode_buffer = std::make_unique<int16_t[]>(NETWORK_MAX_FRAMES * ch);
                    fec_buffer = std::make_unique<int16_t[]>(NETWORK_MAX_FRAMES * ch);
                }
            }
        }

        ~DecoderContext()
        {
            if (decoder)
            {
                opus_decoder_destroy(decoder);
            }
        }

        DecoderContext(const DecoderContext &) = delete;
        DecoderContext &operator=(const DecoderContext &) = delete;

        DecoderContext(DecoderContext &&other) noexcept
            : decoder(other.decoder), decode_buffer(std::move(other.decode_buffer)),
              fec_buffer(std::move(other.fec_buffer)), stats(std::move(other.stats)), last_frames(other.last_frames)
        {
            other.decoder = nullptr;
        }

        DecoderContext &operator=(DecoderContext &&other) noexcept
        {
            if (this != &other)
            {
                decoder = other.decoder;
                decode_buffer = std::move(other.decode_buffer);
                fec_buffer = std::move(other.fec_buffer);
                stats = std::move(other.stats);
                last_frames = other.last_frames;
                other.decoder = nullptr;
            }
            return *this;
        }

        void update_stats(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats_info)
        {
            stats.update(sequence, timestamp, stats_info);
        }
    };

  public:
    explicit NetWorker(asio::io_context &io_context, uint16_t port, const std::string &local_ip);
    ~NetWorker();

    RetCode start();
    RetCode stop();
    RetCode register_sender(uint8_t sender_id, unsigned int channels, unsigned int sample_rate,
                            AudioCodecType codec = AudioCodecType::OPUS);
    RetCode unregister_sender(uint8_t sender_id);
    RetCode send_audio(uint8_t sender_id, const int16_t *data, unsigned int frames, AudioPriority priority);

    RetCode add_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port);
    RetCode del_destination(uint8_t sender_id, uint8_t receiver_token, const std::string &ip, uint16_t port);
    RetCode register_receiver(uint8_t token, ReceiveCallback callback);
    RetCode unregister_receiver(uint8_t token);
    RetCode clear_all_destinations(uint8_t sender_id);
    void query_stats();

  private:
    void start_receive_loop();
    void handle_receive(const asio::error_code &error, std::size_t bytes_transferred);
    DecoderContext &get_decoder(SourceUUID sid, unsigned int channels, unsigned int sample_rate, AudioCodecType codec);

    void process_and_deliver_audio(const DataPacket *header, const uint8_t *opus_data, size_t opus_size,
                                   SourceUUID source_id);

    void send_data_packet(const Destination &dest, uint8_t sender_id, uint8_t receiver_id, uint32_t sequence,
                          uint64_t timestamp, const uint8_t *data, size_t size, uint8_t channels, uint32_t sample_rate,
                          AudioCodecType codec_type, AudioPriority priority);

  private:
    asio::strand<asio::io_context::executor_type> receive_strand;
    std::atomic<bool> running;
    uint32_t local_session_ip;

    std::unique_ptr<asio::ip::udp::socket> receive_socket;
    std::unique_ptr<asio::ip::udp::socket> send_socket;
    std::unique_ptr<char[]> receive_buffer;
    asio::ip::udp::endpoint receive_endpoint;
    std::map<uint8_t, SenderContext> senders;
    std::mutex senders_mutex;

    std::map<uint8_t, ReceiverContext> receivers;
    std::map<SourceUUID, DecoderContext> decoders;
};

#endif