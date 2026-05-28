#ifndef AUDIO_WAVEFILE_HEADER
#define AUDIO_WAVEFILE_HEADER

#include <fstream>
#include <vector>
#include "audio_interface.h"

class ChunkList;

class WavFile
{
public:
    enum mode
    {
        in,
        out
    };

    struct RIFFChunk
    {
        char chunk_id[4];
        uint32_t chunk_size;
        char format[4];
    };

    struct FMTChunk
    {
        char sub_chunk_1_id[4];
        uint32_t sub_chunk_1_size;
        uint16_t audio_format;
        uint16_t num_channel;
        uint32_t sample_rate;
        uint32_t byte_rate;
        uint16_t byte_per_block;
        uint16_t bits_per_sample;
    };

    struct DataChunk
    {
        char sub_chunk_2_id[4];
        uint32_t sub_chunk_2_size;
    };

    struct Header
    {
        RIFFChunk riff;
        FMTChunk fmt;
        DataChunk data;
    };

public:
    WavFile();
    ~WavFile();

    RetCode open(const std::string &path, mode mode);

    // output should have a length more than frame_number*channel_number
    RetCode read(int16_t *output, uint64_t frame_number);

    RetCode read(std::vector<int16_t> &output, uint64_t frame_number);

    RetCode read(std::vector<int16_t> &output);

    RetCode write(const int16_t *input, uint64_t frame_number);

    RetCode write(const std::vector<int16_t> &input, uint64_t frame_number);

    RetCode write(const std::vector<int16_t> &input);

    RetCode seek(uint64_t frame_index);

    uint64_t tell() const;

    uint16_t channel_number() const;
    void set_channel_number(uint16_t channel_number);

    uint32_t sample_rate() const;
    void set_sample_rate(uint32_t sample_rate);

    uint16_t bits_per_sample() const;
    void set_bits_per_sample(uint16_t bits_per_sample);

    uint64_t frame_number() const;

private:
    RetCode write_header(uint64_t data_size);

    RetCode read_header(ChunkList *headers);

    uint64_t sample_number() const
    {
        auto bits_per_sample = header.fmt.bits_per_sample;
        auto bytes_per_sample = bits_per_sample / 8;

        auto total_data_size = header.data.sub_chunk_2_size;
        return total_data_size / bytes_per_sample;
    }

    uint64_t current_sample_index() const
    {
        auto bits_per_sample = header.fmt.bits_per_sample;
        auto bytes_per_sample = bits_per_sample / 8;
        uint64_t data_index;
        if (ostream.is_open())
        {
            data_index = static_cast<uint64_t>(ostream.tellp()) - data_offset;
        }
        else if (istream.is_open())
        {
            data_index = static_cast<uint64_t>(istream.tellg()) - data_offset;
        }
        else
        {
            return 0;
        }
        return data_index / bytes_per_sample;
    }

    void set_current_sample_index(uint64_t sample_idx)
    {
        auto bits_per_sample = header.fmt.bits_per_sample;
        auto bytes_per_sample = bits_per_sample / 8;

        std::streampos stream_index = data_offset + (sample_idx * bytes_per_sample);
        if (ostream.is_open())
        {
            ostream.seekp(stream_index);
        }
        else if (istream.is_open())
        {
            istream.seekg(stream_index);
        }
    }

private:
    mutable std::ifstream istream;
    mutable std::ofstream ostream;
    Header header{};
    uint64_t data_offset{};
};

// Internal class for combining multiple WAV segments with per-segment durations.
// Not part of the public API - used only within AudioDriver internals.
class WavCombiner
{
  public:
    WavCombiner() = default;
    WavCombiner(WavCombiner &&) = default;
    WavCombiner &operator=(WavCombiner &&) = default;

    // Add a segment: open path and play for duration_ms milliseconds.
    // All segments must share the same sample_rate and channel count.
    RetCode add(const std::string &path, unsigned int duration_ms);

    // Read frames sequentially across segments.
    // Returns INVSEEK when all segments are exhausted or a file ends early.
    RetCode read(int16_t *output, uint64_t frame_count);

    // Reset all segment cursors to the beginning (for cycle replay).
    RetCode seek_all();

    bool valid() const { return !segments.empty(); }
    uint32_t sample_rate() const { return native_fs; }
    uint16_t channels() const { return native_ch; }

  private:
    struct Segment
    {
        std::unique_ptr<WavFile> wav;
        uint64_t duration_frames; // how many frames to play from this segment
    };

    std::vector<Segment> segments;
    size_t current_seg{0};
    uint64_t seg_frames_played{0};
    uint32_t native_fs{0};
    uint16_t native_ch{0};
};

#endif