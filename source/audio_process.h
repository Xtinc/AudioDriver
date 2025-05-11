#ifndef AUDIO_PROCESS_HEADER
#define AUDIO_PROCESS_HEADER

#include "audio_dataview.h"
#include "audio_interface.h"
#include <array>
#include <memory>
#include <vector>

void mix_channels(const int16_t *ssrc, unsigned int chan, unsigned int frames_num, int16_t *output);

class SincInterpolator
{
  public:
    SincInterpolator(int order, int precision, unsigned int chan, double ratio);

    ~SincInterpolator();

    void process(ArrayView<const float> in, ArrayView<float> out, unsigned int ch);

  private:
    float kernel_func(double x, double cutoff) const;

    float interpolator(double x, const float *input, const float *prev_pos) const;

  private:
    const int order;
    const int quan;
    const unsigned int chan;
    const double step;

    std::vector<double> left;
    std::vector<bool> ready;
    std::unique_ptr<float[]> prev;
    std::unique_ptr<float[]> kern;
};

class DRCompressor
{
  public:
    explicit DRCompressor(float sample_rate, float threshold = -10.0f, float ratio = 2.0f, float attack = 0.006f,
                          float release = 0.1f, float knee_width = 6.0f);

    RetCode process(PCM_TYPE *buffer, unsigned int frames, unsigned int channels, float gain = 0.0);

    void reset();

  private:
    float compute_gain(float inputLevel) const;
    float sample2db(PCM_TYPE sample) const;
    float db2sample(float db) const;

  private:
    const float threshold;
    const float ratio;
    const float attack;
    const float release;
    const float knee_width;

    float current_gain;
    float attack_coeff;
    float release_coeff;

    float knee_threshold_lower;
    float knee_threshold_upper;
};

class LocSampler
{
  public:
    LocSampler(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
               unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap);

    RetCode process(const InterleavedView<const PCM_TYPE> &input, const InterleavedView<PCM_TYPE> &output) const;

  public:
    const unsigned int src_fs;
    const unsigned int src_ch;
    const unsigned int dst_fs;
    const unsigned int dst_ch;
    const double ratio;
    const unsigned int ti;
    const AudioChannelMap ichan_map;
    const AudioChannelMap ochan_map;
    const unsigned int real_ichan;
    const unsigned int real_ochan;

  private:
    std::unique_ptr<float[]> ibuffer;
    std::unique_ptr<float[]> obuffer;
    std::unique_ptr<SincInterpolator> resampler;

    void deinterleave_s16_f16(const InterleavedView<const PCM_TYPE> &interleaved,
                              const DeinterleavedView<float> &deinterleaved) const;

    void interleave_f16_s16(const DeinterleavedView<const float> &deinterleaved,
                            const InterleavedView<PCM_TYPE> &interleaved) const;

    void convert_channels(const DeinterleavedView<float> &input, const DeinterleavedView<float> &output) const;

    void convert_sample_rate(const DeinterleavedView<float> &input, DeinterleavedView<float> &output) const;
};

#endif
