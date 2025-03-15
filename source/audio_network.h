#ifndef AUDIO_NETWORK_HEADER
#define AUDIO_NETWORK_HEADER

#include "asio.hpp"
#include "audio_interface.h"
#include <mutex>
#include <vector>

/*                    Packet Frame Format
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|  sender id   |    channels   |  sample rate  |  magic number  |
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
    uint32_t sequence;
    uint64_t timestamp;

    static bool validate(const char *data, size_t length);
};

constexpr uint8_t NET_MAGIC_NUM = 0xBA;

template <typename T> constexpr typename std::underlying_type<T>::type enum2val(T e)
{
    return static_cast<typename std::underlying_type<T>::type>(e);
}

template <typename E> constexpr E val2enum(typename std::underlying_type<E>::type val)
{
    return static_cast<E>(val);
}

inline uint8_t bandwidth_to_byte(AudioBandWidth bw)
{
    switch (bw)
    {
    case AudioBandWidth::Narrow:
        return 0x01;
    case AudioBandWidth::Wide:
        return 0x02;
    case AudioBandWidth::SemiSuperWide:
        return 0x03;
    case AudioBandWidth::CDQuality:
        return 0x04;
    case AudioBandWidth::Full:
        return 0x05;
    default:
        return 0x00;
    }
}

inline AudioBandWidth byte_to_bandwidth(uint8_t byte)
{
    switch (byte)
    {
    case 0x01:
        return AudioBandWidth::Narrow;
    case 0x02:
        return AudioBandWidth::Wide;
    case 0x03:
        return AudioBandWidth::SemiSuperWide;
    case 0x04:
        return AudioBandWidth::CDQuality;
    case 0x05:
        return AudioBandWidth::Full;
    default:
        return AudioBandWidth::Unknown;
    }
}

struct KFifo
{
  public:
    KFifo(size_t blk_sz, size_t blk_num, unsigned int channel);
    ~KFifo();

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

    KFifo(const KFifo &) = delete;
    KFifo &operator=(const KFifo &) = delete;

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

  private:
    uint8_t encode_sample(int16_t sample, State &state);
    size_t calculate_encoded_size(unsigned int frames) const noexcept;

    const unsigned int channels;
    const unsigned int max_frames;
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

  private:
    int16_t decode_sample(uint8_t code, State &state);

    const unsigned int channels;
    const unsigned int max_frames;
    std::vector<State> decode_states;
    std::unique_ptr<int16_t[]> decode_buffer;
};

#endif