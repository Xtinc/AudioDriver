#include "audio_network.h"
#include <cstring>

KFifo::KFifo(size_t blk_sz, size_t blk_num, unsigned int channel)
    : chan(channel), buf_length(channel * blk_sz), max_length(channel * blk_sz * blk_num), idle_count(0), head(0),
      tail(0), length(0), memory_addr(nullptr), buffer_addr(nullptr)
{

    memory_addr = new char[max_length];
    buffer_addr = new char[buf_length];
}

KFifo::~KFifo()
{
    delete[] buffer_addr;
    delete[] memory_addr;
}

bool KFifo::store(const char *input_addr, size_t write_length)
{
    if (!input_addr || write_length == 0)
    {
        return false;
    }

    std::lock_guard<std::mutex> lck(io_mtx);
    size_t space = available_space();
    size_t actual_write = std::min(write_length, space);

    if (actual_write == 0)
    {
        return false;
    }

    if (max_length - tail >= actual_write)
    {
        memcpy(memory_addr + tail, input_addr, actual_write);
        tail = (tail + actual_write == max_length) ? 0 : tail + actual_write;
    }
    else
    {
        size_t first_part = max_length - tail;
        memcpy(memory_addr + tail, input_addr, first_part);
        memcpy(memory_addr, input_addr + first_part, actual_write - first_part);
        tail = actual_write - first_part;
    }

    length += actual_write;
    return actual_write == write_length;
}

bool KFifo::load(char *output_addr, size_t read_length)
{
    std::lock_guard<std::mutex> lck(io_mtx);
    if (!output_addr || read_length > buf_length)
    {
        return false;
    }

    auto actual_read = std::min<size_t>(read_length, length);
    if (actual_read == 0)
    {
        return false;
    }

    if (max_length - head >= actual_read)
    {
        memcpy(output_addr, memory_addr + head, actual_read);
        head = (head + actual_read == max_length) ? 0 : head + actual_read;
    }
    else
    {
        size_t first_part = max_length - head;
        memcpy(output_addr, memory_addr + head, first_part);
        memcpy(output_addr + first_part, memory_addr, actual_read - first_part);
        head = actual_read - first_part;
    }

    length -= actual_read;
    return true;
}

bool KFifo::store_aside(size_t write_length)
{
    return store(buffer_addr, write_length);
}

bool KFifo::load_aside(size_t read_length)
{
    memset(buffer_addr, 0, buf_length);
    return load(buffer_addr, read_length);
}

// Standard IMA ADPCM index table and step table
static constexpr int ADPCM_INDEX_TABLE[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
static constexpr int ADPCM_STEP_TABLE[89] = {
    7,    8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,   28,
    31,   34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,  118,
    130,  143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,  494,
    544,  598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878, 2066,
    2272, 2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845, 8630,
    9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};
static constexpr size_t ADPCM_HEADER_SIZE_PER_CHANNEL = 4;

NetEncoder::NetEncoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    encode_states.resize(channels);
    size_t buffer_size = calculate_encoded_size(max_frames);
    encode_buffer = std::make_unique<uint8_t[]>(buffer_size);
    reset();
}

void NetEncoder::reset() noexcept
{
    for (auto &state : encode_states)
    {
        state.predictor = 0;
        state.step_index = 0;
    }
}

size_t NetEncoder::calculate_encoded_size(unsigned int frames) const noexcept
{
    return channels * ADPCM_HEADER_SIZE_PER_CHANNEL + (frames * channels + 1) / 2;
}

uint8_t NetEncoder::encode_sample(int16_t sample, NetEncoder::State &state)
{
    int diff = sample - state.predictor;
    uint8_t code = 0;

    if (diff < 0)
    {
        code = 8;
        diff = -diff;
    }

    int step = ADPCM_STEP_TABLE[state.step_index];
    int temp;

    temp = step;
    if (diff >= temp)
    {
        code |= 4;
        diff -= temp;
    }
    temp = step >> 1;
    if (diff >= temp)
    {
        code |= 2;
        diff -= temp;
    }
    temp = step >> 2;
    if (diff >= temp)
    {
        code |= 1;
    }

    diff = 0;
    if (code & 4)
        diff += step;
    if (code & 2)
        diff += step >> 1;
    if (code & 1)
        diff += step >> 2;
    diff += step >> 3;

    if (code & 8)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE[code & 15];

    if (state.step_index < 0)
        state.step_index = 0;
    else if (state.step_index > 88)
        state.step_index = 88;

    return code & 15;
}

const uint8_t *NetEncoder::encode(const int16_t *pcm_data, unsigned int frames, size_t &out_size)
{
    if (!pcm_data || frames == 0)
    {
        out_size = 0;
        return nullptr;
    }

    frames = std::min(frames, max_frames);
    out_size = calculate_encoded_size(frames);
    uint8_t *out = encode_buffer.get();

    for (unsigned int ch = 0; ch < channels; ch++)
    {
        int16_t predictor_value = static_cast<int16_t>(encode_states[ch].predictor);
        out[0] = predictor_value & 0xFF;
        out[1] = (predictor_value >> 8) & 0xFF;
        out += 2;

        *out++ = static_cast<uint8_t>(encode_states[ch].step_index);
        *out++ = 0; // Reserved byte
    }

    unsigned int samples_to_encode = frames * channels;
    for (unsigned int i = 0; i < samples_to_encode; i += 2)
    {
        unsigned int ch = i % channels;
        uint8_t nibble1 = encode_sample(pcm_data[i], encode_states[ch]);
        uint8_t nibble2 = 0;
        if (i + 1 < samples_to_encode)
        {
            ch = (i + 1) % channels;
            nibble2 = encode_sample(pcm_data[i + 1], encode_states[ch]);
        }
        *out++ = nibble1 | (nibble2 << 4);
    }

    return encode_buffer.get();
}

NetDecoder::NetDecoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    decode_states.resize(channels);
    decode_buffer = std::make_unique<int16_t[]>(max_frames * channels);
    reset();
}

void NetDecoder::reset() noexcept
{
    for (auto &state : decode_states)
    {
        state.predictor = 0;
        state.step_index = 0;
    }
}

int16_t NetDecoder::decode_sample(uint8_t code, NetDecoder::State &state)
{
    int step = ADPCM_STEP_TABLE[state.step_index];

    int diff = step >> 3;
    if (code & 1)
        diff += step >> 2;
    if (code & 2)
        diff += step >> 1;
    if (code & 4)
        diff += step;

    if (code & 8)
        state.predictor -= diff;
    else
        state.predictor += diff;

    if (state.predictor > 32767)
        state.predictor = 32767;
    else if (state.predictor < -32768)
        state.predictor = -32768;

    state.step_index += ADPCM_INDEX_TABLE[code & 15];

    if (state.step_index < 0)
        state.step_index = 0;
    else if (state.step_index > 88)
        state.step_index = 88;

    return static_cast<int16_t>(state.predictor);
}

const int16_t *NetDecoder::decode(const uint8_t *adpcm_data, size_t adpcm_size, unsigned int &out_frames)
{
    if (!adpcm_data || adpcm_size <= channels * ADPCM_HEADER_SIZE_PER_CHANNEL)
    {
        out_frames = 0;
        return nullptr;
    }

    size_t header_size = channels * ADPCM_HEADER_SIZE_PER_CHANNEL;
    size_t adpcm_data_size = adpcm_size - header_size;
    size_t adpcm_samples = adpcm_data_size * 2; // Each byte contains two 4-bit samples

    out_frames = static_cast<unsigned int>(adpcm_samples / channels);
    out_frames = std::min(out_frames, max_frames);

    std::memset(decode_buffer.get(), 0, out_frames * channels * sizeof(int16_t));

    const uint8_t *in = adpcm_data;
    for (unsigned int ch = 0; ch < channels; ch++)
    {
        int16_t predictor_value = in[0] | (in[1] << 8);
        decode_states[ch].predictor = predictor_value;
        in += 2;

        decode_states[ch].step_index = *in++;
        in++; // Skip reserved byte
    }

    unsigned int sample_index = 0;
    for (unsigned int i = 0; i < adpcm_data_size && sample_index < out_frames * channels; i++)
    {
        uint8_t byte = adpcm_data[header_size + i];

        unsigned int ch = sample_index % channels;
        decode_buffer[sample_index++] = decode_sample(byte & 0x0F, decode_states[ch]);
        if (sample_index < out_frames * channels)
        {
            ch = sample_index % channels;
            decode_buffer[sample_index++] = decode_sample((byte >> 4) & 0x0F, decode_states[ch]);
        }
    }

    return decode_buffer.get();
}