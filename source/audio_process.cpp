#include "audio_process.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static constexpr auto RS_BLKSIZE = 4;
static constexpr auto A_PI = 3.141592653589793;
static constexpr auto MAX_CONVERTER_RATIO = 8.0;

constexpr int16_t clamp_s16(int32_t v)
{
    return static_cast<int16_t>(v < -32768 ? -32768 : 32767 < v ? 32767 : v);
}

constexpr int16_t clamp_s16(double v)
{
    v *= 32767.0;
    return static_cast<int16_t>(v < -32768 ? -32768 : 32767 < v ? 32767 : v);
}

void mix_channels(const int16_t *ssrc, unsigned int chan, unsigned int frames_num, int16_t *output)
{
    for (unsigned int i = 0; i < frames_num * chan; i++)
    {
        auto res = static_cast<int32_t>(output[i]) + static_cast<int32_t>(ssrc[i]);
        output[i] = clamp_s16(res);
    }
}

inline static double sinc(double x)
{
    return x == 0 ? 1 : std::sin(A_PI * x) / (A_PI * x);
}

static double blackman_nuttall(double x, double N)
{
    return 0.3635819 - 0.4891775 * std::cos((2 * A_PI * x) / N) + 0.1365995 * std::cos((4 * A_PI * x) / N) -
           0.0106411 * std::cos((6 * A_PI * x) / N);
}

SincInterpolator::SincInterpolator(int _order, int _precision, int _chan, double _ratio)
    : order(_order), quan(_precision), chan(_chan), step(_ratio)
{
    left = new double[chan];
    memset(left, 0, sizeof(double) * chan);
    ready = new bool[chan];
    memset(ready, 0, sizeof(bool) * chan);
    double cutoff = (std::min)(1.0, 1.0 / step) * 0.98;
    prev = new double[order * chan * 2];
    memset(prev, 0, sizeof(double) * order * chan * 2);
    kern = new double[order * 2 * (quan + 2)];
    int idx = 0;
    for (int offset = 0; offset <= quan; offset++)
    {
        auto scale = 0.0;
        int start_idx = idx;
        for (int i = -order + 1; i <= order; i++)
        {
            kern[idx] = kernel_func(static_cast<double>(offset) / quan - i, cutoff);
            scale += kern[idx];
            idx++;
        }
        idx = start_idx;
        for (int i = -order + 1; i <= order; i++)
        {
            kern[idx++] /= scale;
        }
    }
}

SincInterpolator::~SincInterpolator()
{
    delete[] left;
    delete[] ready;
    delete[] prev;
    delete[] kern;
}

int SincInterpolator::process(const double *input, int n_input, double *output, int track)
{
    if (n_input < order * 2 || track > chan)
    {
        return 0;
    }

    double x = left[track] - (ready[track] ? order : 0);
    int wr_pos = track;
    while (x < n_input - order)
    {
        output[wr_pos] = interpolator(x, input, prev + track * order * 2);
        wr_pos += chan;
        x += step;
    }
    left[track] = x - (n_input - order);
    memcpy(prev + track * order * 2, &input[n_input - order * 2], sizeof(double) * order * 2);
    ready[track] = true;

    return wr_pos / chan;
}

double SincInterpolator::kernel_func(double x, double cutoff) const
{
    if (x > -order && x < order)
    {
        return sinc(cutoff * x) * blackman_nuttall(x + 1 - order, 2 * order - 1);
    }
    return 0.0;
}

double SincInterpolator::interpolator(double x, const double *input, const double *prev_pos)
{
    double fraction = std::abs(x - std::floor(x));
    int c_idx = (int)(fraction * quan + 0.5);
    double c_frac = fraction * quan - c_idx;
    const double *coffs1 = &kern[c_idx * (order * 2)];
    const double *coffs2 = &kern[(c_idx + 1) * (order * 2)];
    const int offset = (quan + 1) * order * 2;
    for (int i = 0; i < order; i++)
    {
        kern[offset + 2 * i] = (coffs2[2 * i] - coffs1[2 * i]) * c_frac + coffs1[2 * i];
        kern[offset + 2 * i + 1] = (coffs2[2 * i + 1] - coffs1[2 * i + 1]) * c_frac + coffs1[2 * i + 1];
    }
    const double *coffs = &kern[offset];

    double sum = 0.0;
    for (int i = static_cast<int>(floor(x)) - order + 1; i <= static_cast<int>(floor(x)) + order; i += RS_BLKSIZE)
    {
        const double *samp_ptr = nullptr;
        bool overlap = true;

        // sample index, [0..order*2] are in state->p, [order*2..n] are in input
        //                        state->p     input
        // total input vector = [0..order*2 order*2..n]
        auto sample_index = i + 2 * order - 1;
        // do we have blocksize samples in p?
        if (sample_index + RS_BLKSIZE < 2 * order)
        {
            samp_ptr = &prev_pos[sample_index];
            overlap = false;
        }
        else if (sample_index >= 2 * order)
        {
            samp_ptr = &input[sample_index - 2 * order];
            overlap = false;
        }

        // if the samples saved from the previous iteration overlap
        // with the ones in this one, do them separately
        if (overlap)
        {
            for (int j = 0; j < RS_BLKSIZE; j++)
            {
                auto sample = 0.0;

                if (sample_index + j >= 2 * order)
                {
                    sample = input[sample_index + j - (2 * order)];
                }
                else
                {
                    sample = prev_pos[sample_index + j];
                }
                sum += sample * coffs[j];
            }
        }
        else
        {
            for (int j = 0; j < RS_BLKSIZE; j++)
            {
                sum += samp_ptr[j] * coffs[j];
            }
        }
        coffs += RS_BLKSIZE;
    }

    return (std::max)(-1.0, (std::min)(1.0, sum));
}

LocSampler::LocSampler(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
                       unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap)
    : src_fs(src_fs), src_ch(src_ch), dst_fs(dst_fs), dst_ch(dst_ch), ratio(static_cast<double>(dst_fs) / src_fs),
      max_frames(max_frames), ichan_map(imap), ochan_map(omap), real_channel(std::min(2u, dst_ch)), valid(true)
{
    if (src_ch == 0 || dst_ch == 0 || max_frames == 0 || ratio <= 0.0 || ratio > MAX_CONVERTER_RATIO ||
        ichan_map[0] >= src_ch || ochan_map[0] >= dst_ch || ichan_map[1] >= src_ch || ochan_map[1] >= dst_ch)
    {
        valid = false;
    }
    else
    {
        if (src_fs != dst_fs)
        {
            resampler = std::make_unique<SincInterpolator>(64, 100, real_channel, static_cast<double>(src_fs) / dst_fs);
            ibuffer = std::make_unique<double[]>(max_frames);
            obuffer = std::make_unique<double[]>(static_cast<unsigned int>(max_frames * ratio) * real_channel);
            AUDIO_INFO_PRINT(
                "LocSampler: [fs] %u->%u, [ch] %u->%u, [L] (%u)--(%u), [R] (%u)--(%u)  ratio = %f, max_frames = %u",
                src_fs, dst_fs, src_ch, dst_ch, ichan_map[0], ochan_map[0], ichan_map[1], ochan_map[1], ratio,
                max_frames);
        }
        else
        {
            AUDIO_INFO_PRINT("ChnConvert: [ch] %u->%u, [L] (%u)--(%u), [R] (%u)--(%u)", src_ch, dst_ch, ichan_map[0],
                             ochan_map[0], ichan_map[1], ochan_map[1]);
        }
    }
}

RetCode LocSampler::process(const PCM_TYPE *input, unsigned int input_frames, PCM_TYPE *output,
                            unsigned int &output_frames)
{
    if (!valid || !input || !output || input_frames == 0 || input_frames > max_frames)
    {
        return RetCode::EPARAM;
    }

    if (src_fs == dst_fs)
    {
        if (src_ch == dst_ch && ichan_map[0] == ochan_map[0] && ichan_map[1] == ochan_map[1])
        {
            return RetCode::NOACTION;
        }

        convertChannels(input, input_frames, output);
        output_frames = input_frames;
        return RetCode::OK;
    }

    double *float_input = ibuffer.get();
    double *float_output = obuffer.get();

    for (unsigned int c = 0; c < real_channel; c++)
    {
        for (unsigned int i = 0; i < input_frames; i++)
        {
            float_input[i] = static_cast<double>(input[i * src_ch + ichan_map[c]]) / 32768.0;
        }

        output_frames = static_cast<unsigned int>(resampler->process(float_input, input_frames, float_output, c));
    }

    for (unsigned int i = 0; i < output_frames; i++)
    {
        for (unsigned int c = 0; c < real_channel; c++)
        {
            output[i * dst_ch + ochan_map[c]] = clamp_s16(float_output[i * real_channel + c]);
        }
    }

    return RetCode::OK;
}

void LocSampler::convertChannels(const PCM_TYPE *input, unsigned int frames, PCM_TYPE *output) const
{
    if (src_ch == 1 && dst_ch == 2)
    {
        const PCM_TYPE *in_ptr = input;
        PCM_TYPE *out_ptr = output;
        for (unsigned int i = 0; i < frames; i++)
        {
            *out_ptr++ = *in_ptr;
            *out_ptr++ = *in_ptr;
            in_ptr++;
        }
    }
    else if (src_ch == 2 && dst_ch == 1)
    {
        const PCM_TYPE *in_ptr = input;
        PCM_TYPE *out_ptr = output;
        for (unsigned int i = 0; i < frames; i++)
        {
            *out_ptr++ = in_ptr[0] / 2 + in_ptr[1] / 2;
            in_ptr += 2;
        }
    }
    else
    {
        for (unsigned int i = 0; i < frames; i++)
        {
            for (unsigned int c = 0; c < real_channel; c++)
            {
                output[i * dst_ch + ochan_map[c]] = input[i * src_ch + ichan_map[c]];
            }
        }
    }
}

DRCompressor::DRCompressor(float sample_rate, float _threshold, float _ratio, float _attack, float _release,
                           float _knee_width)
    : threshold(_threshold), ratio(_ratio), attack(_attack), release(_release), knee_width(_knee_width),
      current_gain(0.0f)
{
    attack_coeff = std::exp(-1.0f / (attack * sample_rate));
    release_coeff = std::exp(-1.0f / (release * sample_rate));

    knee_threshold_lower = threshold - knee_width / 2.0f;
    knee_threshold_upper = threshold + knee_width / 2.0f;
}

RetCode DRCompressor::process(PCM_TYPE *buffer, unsigned int frames, unsigned int channels, float gain)
{
    if (!buffer || frames == 0 || channels == 0)
    {
        return {RetCode::FAILED, "Invalid parameters"};
    }

    for (unsigned int i = 0; i < frames; i++)
    {
        PCM_TYPE maxSample = 0;
        for (unsigned int ch = 0; ch < channels; ++ch)
        {
            auto sample = buffer[i * channels + ch];
            if (sample < 0)
            {
                sample = -sample;
            }
            maxSample = (std::max)(maxSample, sample);
        }

        float inputLevelDB = sample2db(maxSample);
        float targetGain = compute_gain(inputLevelDB) + gain;

        if (targetGain < current_gain)
        {
            current_gain = attack_coeff * current_gain + (1.0f - attack_coeff) * targetGain;
        }
        else
        {
            current_gain = release_coeff * current_gain + (1.0f - release_coeff) * targetGain;
        }

        for (unsigned int ch = 0; ch < channels; ++ch)
        {
            auto result = (float)buffer[i * channels + ch] * db2sample(current_gain);
            result = std::max(-32768.0f, std::min(32767.0f, result));
            buffer[i * channels + ch] = (int16_t)result;
        }
    }

    return RetCode::OK;
}

void DRCompressor::reset()
{
    current_gain = 0.0f;
}

float DRCompressor::compute_gain(float inputLevelDB)
{
    float gainReduction = 0.0f;

    if (inputLevelDB < knee_threshold_lower)
    {
        gainReduction = 0.0f;
    }
    else if (inputLevelDB > knee_threshold_upper)
    {
        gainReduction = (threshold - inputLevelDB) * (1.0f - 1.0f / ratio);
    }
    else
    {
        const float overshoot = inputLevelDB - knee_threshold_lower;
        const float compressionFactor = overshoot / (2.0f * knee_width);
        gainReduction = overshoot * compressionFactor * (1.0f - 1.0f / ratio);
    }

    return gainReduction;
}

float DRCompressor::sample2db(PCM_TYPE sample) const
{
    const float normalizedSample = std::abs(static_cast<float>(sample)) / 32768.0f;
    if (normalizedSample < 1e-6f)
    {
        return -120.0f;
    }
    return 20.0f * std::log10(normalizedSample);
}

float DRCompressor::db2sample(float db) const
{
    return std::pow(10.0f, db / 20.0f);
}
