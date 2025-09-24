#include "audio_process.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static constexpr auto RS_BLKSIZE = 4;
static constexpr auto A_PI = 3.141592653589793;
static constexpr auto MAX_CONVERTER_RATIO = 8.0;
static constexpr auto DENOISE_SAMPLE_RATE = 16000;

static constexpr int16_t clamp_s16(int32_t v)
{
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

static float float2db(float sample)
{
    if (sample < 0)
    {
        sample = -sample;
    }

    return sample < 1e-6f ? -120.0f : 20.0f * std::log10(sample);
}

static float db2float(float db)
{
    return std::pow(10.0f, db / 20.0f);
}

static double sinc(double x)
{
    return x == 0 ? 1 : std::sin(A_PI * x) / (A_PI * x);
}

static double blackman_nuttall(double x, double N)
{
    return 0.3635819 - 0.4891775 * std::cos((2 * A_PI * x) / N) + 0.1365995 * std::cos((4 * A_PI * x) / N) -
           0.0106411 * std::cos((6 * A_PI * x) / N);
}

SincInterpolator::SincInterpolator(int _order, int _precision, unsigned int _chan, double _ratio)
    : order(_order), quan(_precision), chan(_chan), step(_ratio), left(chan, 0.0), ready(chan, false)
{
    const double cutoff = (std::min)(1.0, 1.0 / step) * 0.98;
    prev = std::make_unique<float[]>(order * chan * 2);
    memset(prev.get(), 0, sizeof(float) * order * chan * 2);
    kern = std::make_unique<float[]>(order * 2 * (quan + 2));
    int idx = 0;
    for (int offset = 0; offset <= quan; offset++)
    {
        auto scale = 0.0f;
        const int start_idx = idx;
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

SincInterpolator::~SincInterpolator() = default;

void SincInterpolator::process(ArrayView<const float> in, ArrayView<float> out, unsigned int ch)
{
    const auto isize = in.size();

    DBG_ASSERT_NE(step, 1.0);
    DBG_ASSERT_LE(isize / step, out.size() + 1u);
    DBG_ASSERT_LT(ch, chan);
    DBG_ASSERT_GE(isize, order * 2);

    const auto avail = static_cast<double>(isize - order);
    double x = left[ch] - (ready[ch] ? order : 0);
    size_t wr_pos = 0u;
    const auto prev_ptr = prev.get() + ch * order * 2;

    while (x < avail)
    {
        out[wr_pos++] = interpolator(x, in.data(), prev_ptr);
        x += step;
    }

    left[ch] = x - avail;
    memcpy(prev_ptr, &in[isize - order * 2], sizeof(float) * order * 2);
    ready[ch] = true;
}

float SincInterpolator::kernel_func(double x, double cutoff) const
{
    if (x > -order && x < order)
    {
        return static_cast<float>(sinc(cutoff * x) * blackman_nuttall(x + 1 - order, 2 * order - 1));
    }
    return 0.0f;
}

float SincInterpolator::interpolator(double x, const float *input, const float *prev_pos) const
{
    const double fraction = std::abs(x - std::floor(x));
    const int c_idx = static_cast<int>(fraction * quan + 0.5);
    const auto c_frac = static_cast<float>(fraction * quan - c_idx);
    const auto *coffs1 = &kern[c_idx * (order * 2)];
    const auto *coffs2 = &kern[(c_idx + 1) * (order * 2)];
    const int offset = (quan + 1) * order * 2;
    for (int i = 0; i < order; i++)
    {
        kern[offset + 2 * i] = (coffs2[2 * i] - coffs1[2 * i]) * c_frac + coffs1[2 * i];
        kern[offset + 2 * i + 1] = (coffs2[2 * i + 1] - coffs1[2 * i + 1]) * c_frac + coffs1[2 * i + 1];
    }
    const auto *coffs = &kern[offset];

    auto sum = 0.0f;
    for (int i = static_cast<int>(floor(x)) - order + 1; i <= static_cast<int>(floor(x)) + order; i += RS_BLKSIZE)
    {
        const float *samp_ptr = nullptr;
        bool overlap = true;

        // sample index, [0...order*2] are in state->p, [order*2...n] are in input
        //                        state->p     input
        // total input vector = [0...order*2 order*2..n]
        const auto sample_index = i + 2 * order - 1;
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
                auto sample = 0.0f;

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

    return (std::max)(-1.0f, (std::min)(1.0f, sum));
}

LocSampler::LocSampler(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
                       unsigned int max_iframe, const AudioChannelMap &imap, const AudioChannelMap &omap)
    : src_fs(src_fs), src_ch(src_ch), dst_fs(dst_fs), dst_ch(dst_ch), ratio(static_cast<double>(dst_fs) / src_fs),
      ti(max_iframe * 1000 / src_fs), ichan_map(imap), ochan_map(omap), real_ichan(std::min(2u, src_ch)),
      real_ochan(std::min(2u, dst_ch))
{
    // Validate input parameters
    DBG_ASSERT_GT(src_ch, 0u);
    DBG_ASSERT_GT(dst_ch, 0u);
    DBG_ASSERT_GT(ti, 0u);
    DBG_ASSERT_GT(ratio, 0.0);
    DBG_ASSERT_LE(ratio, MAX_CONVERTER_RATIO);
    DBG_ASSERT_LE(ichan_map[0], src_ch);
    DBG_ASSERT_LE(ichan_map[1], src_ch);
    DBG_ASSERT_LE(ochan_map[0], dst_ch);
    DBG_ASSERT_LE(ochan_map[1], dst_ch);

    // Allocate buffers for audio processing
    auto src_fr = src_fs * ti / 1000;
    auto dst_fr = dst_fs * ti / 1000;
    analysis_ibuffer = std::make_unique<ChannelBuffer<float>>(src_fr, std::max(src_ch, dst_ch));
    analysis_obuffer = std::make_unique<ChannelBuffer<float>>(dst_fr, real_ochan);
    compressor = std::make_unique<DRCompressor>(static_cast<float>(dst_fs), real_ochan);

    // Create resampler if needed
    if (src_fs != dst_fs)
    {
        resampler = std::make_unique<SincInterpolator>(48, 100, real_ochan, static_cast<double>(src_fs) / dst_fs);
        AUDIO_INFO_PRINT(
            "LocSampler: [fs] %u->%u, [ch] %u->%u, [L] (%u)--(%u), [R] (%u)--(%u)  ratio = %f, max_period = %ums",
            src_fs, dst_fs, src_ch, dst_ch, ichan_map[0], ochan_map[0], ichan_map[1], ochan_map[1], ratio, ti);
    }
    else if (ichan_map[0] != ochan_map[0] || ichan_map[1] != ochan_map[1])
    {
        // Channel conversion only
        AUDIO_INFO_PRINT("ChnConvert: [ch] %u->%u, [L] (%u)--(%u), [R] (%u)--(%u)", src_ch, dst_ch, ichan_map[0],
                         ochan_map[0], ichan_map[1], ochan_map[1]);
    }
}

LocSampler::~LocSampler() = default;

RetCode LocSampler::process(const InterleavedView<const PCM_TYPE> &input, const InterleavedView<PCM_TYPE> &output,
                            float gain) const
{
    // Calculate input and output frame counts
    const auto src_fr = ti * src_fs / 1000;
    const auto dst_fr = ti * dst_fs / 1000;
    const auto ibuffer = analysis_ibuffer.get();
    const auto obuffer = analysis_obuffer.get();
    ibuffer->set_num_channels(std::max(real_ochan, real_ichan));

    // Validate buffer sizes
    DBG_ASSERT_LE(input.size(), src_fr * src_ch);
    DBG_ASSERT_LE(output.size(), dst_fr * dst_ch);

    if (input.size() > src_fr * src_ch || output.size() > dst_fr * dst_ch)
    {
        return RetCode::EPARAM;
    }

    // Step 1: Deinterleave PCM data to float arrays
    // input is interleaved with src_ch & src_fr,
    // output is deinterleaved with src_ch & src_fr
    deinterleave_s16_f16(input, ibuffer);

    // Step 2: Perform channel conversion
    // Convert channels from src_ch & src_fr to dst_ch & src_fr
    if (real_ichan != real_ochan)
    {
        convert_channels(ibuffer);
        ibuffer->set_num_channels(real_ochan);
    }

    // Step 3: Perform sample rate conversion if needed
    // To Convert sample rate from src_fs to dst_fs
    ChannelBuffer<float> *buf_ptr = ibuffer;
    if (dst_fs != src_fs)
    {
        convert_sample_rate(ibuffer, obuffer);
        buf_ptr = analysis_obuffer.get();
    }

    // Step 5: Apply dynamic range compression
    compressor->process(buf_ptr, gain);

    // Step 6: Interleave float data back to PCM format
    interleave_f16_s16(buf_ptr, output);

    return RetCode::OK;
}

void LocSampler::deinterleave_s16_f16(const InterleavedView<const PCM_TYPE> &interleaved,
                                      ChannelBuffer<float> *deinterleaved) const
{
    // Ensure input and output have same number of samples per channel
    DBG_ASSERT_LE(SamplesPerChannel(interleaved), deinterleaved->num_frames());

    // Scale factor to normalize 16-bit PCM to float [-1.0, 1.0]
    constexpr float kScaling = 1.f / 32768.f;
    const auto samples_per_channel = SamplesPerChannel(interleaved);

    // Extract each channel from interleaved data based on channel map
    for (size_t i = 0; i < real_ichan; ++i)
    {
        MonoView<float> channel = deinterleaved->channels_view()[i];
        size_t interleaved_idx = ichan_map[i];
        for (size_t j = 0; j < samples_per_channel; ++j)
        {
            channel[j] = static_cast<float>(interleaved[interleaved_idx]) * kScaling;
            interleaved_idx += src_ch;
        }
    }
}

void LocSampler::interleave_f16_s16(ChannelBuffer<float> *deinterleaved,
                                    const InterleavedView<PCM_TYPE> &interleaved) const
{
    // Ensure input and output have same number of samples per channel
    DBG_ASSERT_LE(interleaved.samples_per_channel(), deinterleaved->num_frames());
    const auto samples_per_channel = interleaved.samples_per_channel();

    // Convert each channel from float to 16-bit PCM and interleave based on channel map
    for (size_t i = 0; i < real_ochan; ++i)
    {
        MonoView<const float> channel = deinterleaved->channels_view()[i];
        size_t interleaved_idx = ochan_map[i];
        for (size_t j = 0; j < samples_per_channel; ++j)
        {
            // Scale to 16-bit range and apply proper rounding
            auto v = channel[j] * 32768.f;
            v = std::min(v, 32767.f);
            v = std::max(v, -32768.f);
            interleaved[interleaved_idx] = static_cast<int16_t>(v + std::copysign(0.5f, v));
            interleaved_idx += dst_ch;
        }
    }
}

void LocSampler::convert_channels(ChannelBuffer<float> *io) const
{
    DBG_ASSERT_LE(real_ichan, io->num_channels());
    DBG_ASSERT_LE(real_ochan, io->num_channels());
    const auto frames = io->num_frames();

    // Case 1: Mono to stereo conversion - copy left channel to right channel
    if (real_ichan == 1 && real_ochan == 2)
    {
        auto left_channel = io->channels_view()[0];
        auto right_channel = io->channels_view()[1];
        for (size_t i = 0; i < frames; ++i)
        {
            right_channel[i] = left_channel[i]; // Left channel
        }
    }
    else if (real_ichan == 2 && real_ochan == 1)
    {
        // Case 3: Stereo to mono conversion - average both channels
        auto left_channel = io->channels_view()[0];
        auto right_channel = io->channels_view()[1];
        for (size_t i = 0; i < frames; ++i)
        {
            // Average of L+R
            left_channel[i] = 0.5f * (left_channel[i] + right_channel[i]);
        }
    }
}

void LocSampler::convert_sample_rate(ChannelBuffer<float> *input, ChannelBuffer<float> *output) const
{
    // Ensure input and output have the same number of channels
    DBG_ASSERT_EQ(input->num_channels(), output->num_channels());
    const auto channels = input->num_channels();

    // Apply resampling if source and destination sample rates differ
    for (size_t c = 0; c < channels; c++)
    {
        // Resample each channel separately
        resampler->process(input->channels_view()[c], output->channels_view()[c], static_cast<unsigned int>(c));
    }
}

DRCompressor::DRCompressor(float sample_rate, unsigned int chs, float _threshold, float _ratio, float _attack,
                           float _release, float _knee_width)
    : threshold(_threshold), ratio(_ratio), attack(_attack), release(_release), knee_width(_knee_width), channels(chs),
      current_gain(chs, 0.0f)
{
    attack_coeff = std::exp(-1.0f / (attack * sample_rate));
    release_coeff = std::exp(-1.0f / (release * sample_rate));

    knee_threshold_lower = threshold - knee_width / 2.0f;
    knee_threshold_upper = threshold + knee_width / 2.0f;
}

void DRCompressor::process(ChannelBuffer<float> *input, float gain)
{
    const auto frames = input->num_frames();
    DBG_ASSERT_EQ(channels, input->num_channels());

    if (gain < 0.1f && gain > -0.1f)
    {
        return;
    }

    for (unsigned int c = 0; c < channels; c++)
    {
        MonoView<float> channel = input->channels_view()[c];
        for (unsigned int i = 0; i < frames; i++)
        {
            float inputLevelDB = float2db(channel[i]);
            float targetGain = compute_gain(inputLevelDB) + gain;

            if (targetGain < current_gain[c])
            {
                current_gain[c] = attack_coeff * current_gain[c] + (1.0f - attack_coeff) * targetGain;
            }
            else
            {
                current_gain[c] = release_coeff * current_gain[c] + (1.0f - release_coeff) * targetGain;
            }

            auto result = channel[i] * db2float(current_gain[c]);
            result = std::max(-32768.0f, std::min(32767.0f, result));
            channel[i] = result;
        }
    }
}

float DRCompressor::compute_gain(float inputLevelDB) const
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
