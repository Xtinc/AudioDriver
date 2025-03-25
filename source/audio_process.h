#ifndef AUDIO_PROCESS_HEADER
#define AUDIO_PROCESS_HEADER

#include "audio_interface.h"
#include <array>
#include <memory>

void mix_channels(const int16_t *ssrc, unsigned int chan, unsigned int frames_num, int16_t *output);

class SincInterpolator
{
  public:
    SincInterpolator(int order, int precision, int chan, double ratio);
    ~SincInterpolator();

    int process(const double *input, int input_size, double *output, int track);

  private:
    double kernel_func(double x, double cutoff) const;

    double interpolator(double x, const double *input, const double *prev_pos);

  private:
    const int order;
    const int quan;
    const int chan;
    const double step;

    double *left;
    bool *ready;

    double *prev;
    double *kern;
};

class LocSampler
{
  public:
    LocSampler(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
               unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap);

    RetCode process(const PCM_TYPE *input, unsigned int input_frames, PCM_TYPE *output, unsigned int &output_frames);

    bool is_valid() const
    {
        return valid;
    }

  public:
    const unsigned int src_fs;
    const unsigned int src_ch;
    const unsigned int dst_fs;
    const unsigned int dst_ch;
    const double ratio;
    const unsigned int max_frames;
    const AudioChannelMap ichan_map;
    const AudioChannelMap ochan_map;
    const unsigned int real_channel;

  private:
    bool valid;
    std::unique_ptr<double[]> ibuffer;
    std::unique_ptr<double[]> obuffer;
    std::unique_ptr<SincInterpolator> resampler;

    void convertChannels(const PCM_TYPE *input, unsigned int frames, PCM_TYPE *output) const;
};

class DRCompressor
{
  public:
    DRCompressor(float sample_rate, float threshold = -4.0f, float ratio = 2.0f, float attack = 0.1f,
                 float release = 0.2f, float knee_width = 10.0f);

    RetCode process(PCM_TYPE *buffer, unsigned int frames, unsigned int channels, float gain = 0.0);

    void reset();

  private:
    float compute_gain(float inputLevel);
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

#endif