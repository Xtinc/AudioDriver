#ifndef AUDIO_NETWORK_HEADER
#define AUDIO_NETWORK_HEADER

#include "asio.hpp"
#include "audio_interface.h"
#include "opus.h"
#include <array>
#include <chrono>
#include <deque>
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

    bool operator==(const SourceUUID &other) const
    {
        return sender_ip == other.sender_ip && gateway_ip == other.gateway_ip && sender_token == other.sender_token;
    }

    bool operator<(const SourceUUID &other) const
    {
        return std::tie(sender_ip, gateway_ip, sender_token) <
               std::tie(other.sender_ip, other.gateway_ip, other.sender_token);
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
};

class NetState
{
  public:
    NetState();
    bool update(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats);
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
        NetState stats;

        DecoderContext(unsigned int ch, unsigned int sr) : decoder(nullptr)
        {
            int error;
            decoder = opus_decoder_create(sr, ch, &error);
            if (error == OPUS_OK && decoder)
            {
                decode_buffer = std::make_unique<int16_t[]>(NETWORK_MAX_FRAMES * ch);
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
            : decoder(other.decoder), decode_buffer(std::move(other.decode_buffer)), stats(std::move(other.stats))
        {
            other.decoder = nullptr;
        }

        DecoderContext &operator=(DecoderContext &&other) noexcept
        {
            if (this != &other)
            {
                decoder = other.decoder;
                decode_buffer = std::move(other.decode_buffer);
                stats = std::move(other.stats);
                other.decoder = nullptr;
            }
            return *this;
        }

        bool update_stats(uint32_t sequence, uint64_t timestamp, NetStatInfos &stats_info)
        {
            return stats.update(sequence, timestamp, stats_info);
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

  private:
    void start_receive_loop();
    void handle_receive(const asio::error_code &error, std::size_t bytes_transferred,
                        const asio::ip::udp::endpoint &sender_endpoint);
    void retry_receive_with_backoff();
    DecoderContext &get_decoder(SourceUUID sid, unsigned int channels, unsigned int sample_rate);

    void process_and_deliver_audio(const DataPacket *header, const uint8_t *opus_data, size_t opus_size,
                                   SourceUUID source_id);

    void send_data_packet(const Destination &dest, uint8_t sender_id, uint8_t receiver_id, uint32_t sequence,
                          uint64_t timestamp, const uint8_t *data, size_t size, uint8_t channels, uint32_t sample_rate,
                          AudioCodecType codec_type, AudioPriority priority);

  private:
    int retry_count;

    asio::io_context &io_context;
    std::atomic<bool> running;
    uint32_t local_session_ip;

    std::unique_ptr<asio::ip::udp::socket> receive_socket;
    std::unique_ptr<asio::ip::udp::socket> send_socket;
    std::unique_ptr<char[]> receive_buffer;
    std::map<uint8_t, SenderContext> senders;
    std::mutex senders_mutex;

    std::map<uint8_t, ReceiverContext> receivers;
    std::map<SourceUUID, DecoderContext> decoders;
};

#endif