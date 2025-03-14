#ifndef AUDIO_NETWORK_H
#define AUDIO_NETWORK_H

#include "audio_interface.h"
#include <mutex>
#include <vector>

template <typename T> constexpr typename std::underlying_type<T>::type enum2val(T e)
{
    return static_cast<typename std::underlying_type<T>::type>(e);
}

template <typename E> constexpr E val2enum(typename std::underlying_type<E>::type val)
{
    return static_cast<E>(val);
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
    struct State
    {
        int16_t predictor;
        int8_t step_index;
        State() : predictor(0), step_index(0)
        {
        }
    };

  public:
    NetEncoder(unsigned int channels, unsigned int max_frames);
    ~NetEncoder() = default;

    const uint8_t *encode(const int16_t *pcm_data, unsigned int frames, size_t &out_size);
    void reset();

  private:
    uint8_t encode_sample(int16_t sample, State &state);
    size_t calculate_encoded_size(unsigned int frames) const;

  private:
    unsigned int channels;
    unsigned int max_frames;
    std::vector<State> encode_states;
    std::unique_ptr<uint8_t[]> encode_buffer;
    size_t encode_buffer_size;
};

class NetDecoder
{
    struct State
    {
        int16_t predictor;
        int8_t step_index;
        State() : predictor(0), step_index(0)
        {
        }
    };

  public:
    NetDecoder(unsigned int channels, unsigned int max_frames);
    ~NetDecoder() = default;

    const int16_t *decode(const uint8_t *adpcm_data, size_t adpcm_size, unsigned int &out_frames);
    void reset();

  private:
    int16_t decode_sample(uint8_t code, State &state);

  private:
    unsigned int channels;
    unsigned int max_frames;
    std::vector<State> decode_states;
    std::unique_ptr<int16_t[]> decode_buffer;
    size_t decode_buffer_size;
};

#endif