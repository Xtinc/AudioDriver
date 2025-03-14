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

// IMA ADPCM
static constexpr size_t ADPCM_HEADER_SIZE_PER_CHANNEL = 4;
static constexpr int8_t ADPCM_INDEX_TABLE[16] = {-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8};
static constexpr int16_t ADPCM_STEP_TABLE[89] = {
    7,    8,     9,     10,    11,    12,    13,    14,    16,    17,    19,    21,    23,    25,   28,
    31,   34,    37,    41,    45,    50,    55,    60,    66,    73,    80,    88,    97,    107,  118,
    130,  143,   157,   173,   190,   209,   230,   253,   279,   307,   337,   371,   408,   449,  494,
    544,  598,   658,   724,   796,   876,   963,   1060,  1166,  1282,  1411,  1552,  1707,  1878, 2066,
    2272, 2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,  5894,  6484,  7132,  7845, 8630,
    9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767};

NetEncoder::NetEncoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    encode_states.resize(channels);
    encode_buffer_size = calculate_encoded_size(max_frames);
    encode_buffer = std::make_unique<uint8_t[]>(encode_buffer_size);
    reset();
}

void NetEncoder::reset()
{
    for (auto &state : encode_states)
    {
        state.predictor = 0;
        state.step_index = 0;
    }
}

size_t NetEncoder::calculate_encoded_size(unsigned int frames) const
{
    return channels * ADPCM_HEADER_SIZE_PER_CHANNEL + (frames * channels + 1) / 2;
}

uint8_t NetEncoder::encode_sample(int16_t sample, NetEncoder::State &state)
{
    int diff = sample - state.predictor;
    uint8_t code = 0;

    if (diff < 0)
    {
        code = 0x8;
        diff = -diff;
    }

    int step = ADPCM_STEP_TABLE[state.step_index];

    int tempstep = step;
    if (diff >= tempstep)
    {
        code |= 0x4;
        diff -= tempstep;
    }
    tempstep >>= 1;

    if (diff >= tempstep)
    {
        code |= 0x2;
        diff -= tempstep;
    }
    tempstep >>= 1;

    if (diff >= tempstep)
    {
        code |= 0x1;
    }

    diff = ((step * ((code & 0x7) * 2 + 1)) >> 3);
    if (code & 0x8)
    {
        state.predictor -= diff;
    }
    else
    {
        state.predictor += diff;
    }

    state.predictor = std::max<int16_t>(-32768, std::min<int16_t>(32767, state.predictor));
    state.step_index += ADPCM_INDEX_TABLE[code & 0xf];
    state.step_index = std::max<int8_t>(0, std::min<int8_t>(88, state.step_index));

    return code & 0xf;
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

    std::memset(encode_buffer.get(), 0, encode_buffer_size);

    uint8_t *out = encode_buffer.get();

    for (unsigned int ch = 0; ch < channels; ch++)
    {
        *reinterpret_cast<int16_t *>(out) = encode_states[ch].predictor;
        out += 2;
        *out++ = static_cast<uint8_t>(encode_states[ch].step_index);
        *out++ = 0;
    }

    unsigned int samples_to_encode = frames * channels;
    for (unsigned int i = 0; i < samples_to_encode; i += 2)
    {
        uint8_t byte = 0;

        for (unsigned int j = 0; j < 2 && (i + j) < samples_to_encode; j++)
        {
            unsigned int idx = i + j;
            unsigned int ch = idx % channels;
            uint8_t nibble = encode_sample(pcm_data[idx], encode_states[ch]);
            if (j == 0)
            {
                byte = nibble;
            }
            else
            {
                byte |= (nibble << 4);
            }
        }
        *out++ = byte;
    }

    return encode_buffer.get();
}

NetDecoder::NetDecoder(unsigned int channels, unsigned int max_frames) : channels(channels), max_frames(max_frames)
{
    decode_states.resize(channels);
    decode_buffer_size = max_frames * channels;
    decode_buffer = std::make_unique<int16_t[]>(decode_buffer_size);
    reset();
}

void NetDecoder::reset()
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
    if (code & 0x1)
        diff += step >> 2;
    if (code & 0x2)
        diff += step >> 1;
    if (code & 0x4)
        diff += step;

    if (code & 0x8)
    {
        state.predictor -= diff;
    }
    else
    {
        state.predictor += diff;
    }

    state.predictor = std::max<int16_t>(-32768, std::min<int16_t>(32767, state.predictor));
    state.step_index += ADPCM_INDEX_TABLE[code & 0xf];
    state.step_index = std::max<int8_t>(0, std::min<int8_t>(88, state.step_index));

    return state.predictor;
}

const int16_t *NetDecoder::decode(const uint8_t *adpcm_data, size_t adpcm_size, unsigned int &out_frames)
{
    if (!adpcm_data || adpcm_size == 0)
    {
        out_frames = 0;
        return nullptr;
    }

    std::memset(decode_buffer.get(), 0, decode_buffer_size * sizeof(int16_t));
    size_t header_size = channels * ADPCM_HEADER_SIZE_PER_CHANNEL;
    if (adpcm_size < header_size)
    {
        out_frames = 0;
        return nullptr;
    }

    const uint8_t *in = adpcm_data;
    for (unsigned int ch = 0; ch < channels; ch++)
    {
        decode_states[ch].predictor = *reinterpret_cast<const int16_t *>(in);
        in += 2;
        decode_states[ch].step_index = *in++;
        in++;
    }

    size_t adpcm_data_size = adpcm_size - header_size;
    size_t adpcm_samples = adpcm_data_size * 2;
    out_frames = static_cast<unsigned int>(adpcm_samples / channels);

    out_frames = std::min(out_frames, max_frames);

    unsigned int sample_index = 0;
    for (unsigned int i = 0; i < adpcm_data_size && sample_index < out_frames * channels; i++)
    {
        uint8_t byte = adpcm_data[header_size + i];

        uint8_t code = byte & 0x0F;
        unsigned int ch = sample_index % channels;
        decode_buffer[sample_index++] = decode_sample(code, decode_states[ch]);

        if (sample_index >= out_frames * channels)
        {
            break;
        }

        code = (byte >> 4) & 0x0F;
        ch = sample_index % channels;
        decode_buffer[sample_index++] = decode_sample(code, decode_states[ch]);
    }

    return decode_buffer.get();
}