#include "audio_process.h"
#include "audio_network.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static constexpr auto RS_BLKSIZE = 4;
static constexpr auto A_PI = 3.141592653589793;
static constexpr auto MAX_CONVERTER_RATIO = 8.0;

void mix_channels(const int16_t *ssrc, unsigned int chan, unsigned int frames_num, int16_t *output)
{
    // Start compressing when absolute sample > SOFT_THRESH
    // Compress excess by roughly 2: output = thresh + (excess/2)
    constexpr int32_t SOFT_THRESH = 30000;
    constexpr int32_t CLIP_POS = 32767;

    for (unsigned int i = 0; i < frames_num * chan; i++)
    {
        const int32_t a = static_cast<int32_t>(output[i]);
        const int32_t b = static_cast<int32_t>(ssrc[i]);
        int32_t sum = a + b;

        // fast path: within soft threshold -> normal sum (still clamped)
        int32_t abs_sum = sum < 0 ? -sum : sum;
        if (abs_sum <= SOFT_THRESH)
        {
            output[i] = static_cast<int16_t>(sum);
            continue;
        }

        // soft limiting: compress the amount above threshold using integer ops
        int32_t excess = abs_sum - SOFT_THRESH;
        // divide by 2 with rounding: (excess + 1) >> 1
        int32_t compressed_excess = (excess + 1) >> 1;
        int32_t limited = SOFT_THRESH + compressed_excess;

        if (limited > CLIP_POS)
        {
            limited = CLIP_POS;
        }

        int32_t outv = (sum < 0) ? -limited : limited;
        output[i] = static_cast<int16_t>(outv);
    }
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

LocSampler::LocSampler(unsigned int sfs, unsigned int sch, unsigned int dfs, unsigned int dch, unsigned int max_iframe,
                       const AudioChannelMap &imap, const AudioChannelMap &omap)
    : src_fs(sfs), src_ch(sch), dst_fs(dfs), dst_ch(dch), ratio(static_cast<double>(dfs) / sfs),
      ti(max_iframe * 1000 / sfs), ichan_map(imap), ochan_map(omap), real_ichan(std::min(2u, sch)),
      real_ochan(std::min(2u, dch))
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
    volume_controller = std::make_unique<VolumeController>(static_cast<float>(dst_fs), real_ochan);

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
                            unsigned int gain, FadeEffect *effetor) const
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

    // Step 4: Apply volume control
    volume_controller->process(buf_ptr, gain);

    if (effetor)
    {
        effetor->process(buf_ptr);
    }

    // Step 5: Interleave float data back to PCM format
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

VolumeController::VolumeController(float sample_rate_, unsigned int channels_, float smooth_time_ms_)
    : sample_rate(sample_rate_), channels(channels_), current_gain(channels, 0.0f), target_gain(0.0f)
{
    DBG_ASSERT_GT(sample_rate, 0.0f);
    DBG_ASSERT_GT(channels, 0u);
    DBG_ASSERT_GT(smooth_time_ms_, 0.0f);

    // Calculate smoothing coefficient for exponential smoothing
    // Time constant: tau = smooth_time_ms / 1000.0f
    // Coefficient: coeff = exp(-1.0f / (tau * sample_rate))
    float tau = smooth_time_ms_ / 1000.0f;
    smooth_coeff = std::exp(-1.0f / (tau * sample_rate));
}

void VolumeController::process(ChannelBuffer<float> *input, unsigned int gain)
{
    DBG_ASSERT_EQ(channels, input->num_channels());

    if (!input || input->num_frames() == 0)
    {
        return;
    }

    // Convert percentage gain to linear gain
    target_gain = convert_gain_to_linear(static_cast<float>(gain));

    const auto frames = input->num_frames();

    // Process each channel
    for (unsigned int c = 0; c < channels; c++)
    {
        MonoView<float> channel = input->channels_view()[c];

        for (unsigned int i = 0; i < frames; i++)
        {
            // Smooth volume changes to avoid clicks and pops
            current_gain[c] = smooth_coeff * current_gain[c] + (1.0f - smooth_coeff) * target_gain;

            // Apply volume control
            channel[i] *= current_gain[c];
        }
    }
}

float VolumeController::convert_gain_to_linear(float gain_percent) const
{
    // Clamp input to valid range
    gain_percent = std::max(0.0f, std::min(100.0f, gain_percent));

    if (gain_percent <= 0.0f)
    {
        return 0.0f; // Complete silence
    }

    // Android/iOS compatible logarithmic volume curve
    // Based on Android AudioFlinger and iOS Core Audio implementations
    // 0% -> -inf dB (silence)
    // 1% -> -58  dB (barely audible)
    // 50% -> -8  dB (perceived half volume)
    // 100% -> 0  dB (unity gain)

    float db_gain;
    if (gain_percent <= 1.0f)
    {
        // Handle very low volume range (0-1%) with steep slope to silence
        // This matches mobile device behavior where very low settings are nearly silent
        db_gain = -58.0f - (1.0f - gain_percent) * 40.0f; // Goes from -58dB to -98dB
    }
    else
    {
        // dB = -58 * (1 - (volume/100)^0.5)^2
        float normalized = gain_percent / 100.0f;

        float curve_factor = std::sqrt(normalized); // Square root for better mid-range control
        db_gain = -58.0f * std::pow(1.0f - curve_factor, 2.0f);

        // Slight adjustment to make middle range (30-70%) more usable
        if (gain_percent >= 20.0f && gain_percent <= 80.0f)
        {
            float mid_adjustment = 0.15f * std::sin((gain_percent - 50.0f) * static_cast<float>(A_PI) / 60.0f);
            db_gain += mid_adjustment;
        }
    }

    // Convert dB to linear gain
    return std::pow(10.0f, db_gain / 20.0f);
}

FadeEffect::FadeEffect(float _sample_rate)
    : sample_rate(_sample_rate), fade_type(FadeType::INIT), position(0), total_samples(0)
{
    DBG_ASSERT_GT(sample_rate, 0.0f);
}

void FadeEffect::start(FadeType type, float duration_ms)
{
    DBG_ASSERT_GE(duration_ms, 0.0f);

    fade_type = type;
    position = 0;

    // Calculate total samples needed for fade duration
    total_samples = static_cast<unsigned int>((duration_ms / 1000.0f) * sample_rate);
}

void FadeEffect::process(ChannelBuffer<float> *buffer)
{
    if (fade_type == FadeType::NONE)
    {
        return;
    }

    const auto frames = buffer->num_frames();
    const auto num_channels = buffer->num_channels();

    for (unsigned int i = 0; i < frames; i++)
    {
        // Check if fade is complete
        if (position >= total_samples)
        {
            if (fade_type == FadeType::FADE_OUT)
            {
                // Silence remaining samples after fade-out completes
                for (unsigned int c = 0; c < num_channels; c++)
                {
                    MonoView<float> channel = buffer->channels_view()[c];
                    for (unsigned int j = i; j < frames; j++)
                    {
                        channel[j] = 0.0f;
                    }
                }
            }
            return;
        }

        // Compute gain for current position
        float gain = compute_gain();

        // Apply gain to all channels
        for (unsigned int c = 0; c < num_channels; c++)
        {
            MonoView<float> channel = buffer->channels_view()[c];
            channel[i] *= gain;
        }

        position++;
    }
}

void FadeEffect::decide_fade_type(unsigned int highest_priority, unsigned int current_priority)
{
    switch (fade_type)
    {
    case FadeType::INIT:
        start(FadeType::FADE_IN, 500.0f);
        break;
    case FadeType::NONE:
        if (highest_priority >= enum2val(AudioPriority::HIGH) && current_priority <= enum2val(AudioPriority::LOW))
        {
            start(FadeType::FADE_OUT, 500.0f);
        }
        break;
    case FadeType::FADE_OUT:
        if (highest_priority < enum2val(AudioPriority::HIGH))
        {
            start(FadeType::FADE_IN, 2500.0f);
        }
        break;
    case FadeType::FADE_IN:
        if (position >= total_samples)
        {
            start(FadeType::NONE, 0.0f);
        }
        break;
    default:
        break;
    }
}

float FadeEffect::compute_gain() const
{
    if (position >= total_samples)
    {
        return fade_type == FadeType::FADE_IN ? 1.0f : 0.0f;
    }

    // Calculate linear position (0.0 to 1.0)
    float linear_position = static_cast<float>(position) / static_cast<float>(total_samples);

    // Use cosine-based fade curve (widely used in professional audio)
    // This provides more natural-sounding transitions that match human hearing perception
    float gain;
    if (fade_type == FadeType::FADE_IN)
    {
        // Fade In: gain = 0.5 * (1.0 - cos(π * position))
        gain = 0.5f * (1.0f - std::cos(static_cast<float>(A_PI) * linear_position));
    }
    else // FADE_OUT
    {
        // Fade Out: gain = 0.5 * (1.0 + cos(π * position))
        gain = 0.5f * (1.0f + std::cos(static_cast<float>(A_PI) * linear_position));
    }

    return gain;
}
