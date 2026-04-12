#ifndef AUDIO_PROCESS_HEADER
#define AUDIO_PROCESS_HEADER

#include "audio_filterbanks.h"
#include "audio_interface.h"
#include <array>
#include <memory>
#include <vector>

/**
 * @brief Mixes audio channels together
 *
 * @param ssrc Source audio buffer
 * @param chan Number of channels
 * @param frames_num Number of frames
 * @param output Output buffer (will be mixed with input)
 */
void mix_channels(const int16_t *ssrc, unsigned int chan, unsigned int frames_num, int16_t *output);

/**
 * @brief Generic PI controller for scalar feedback loops.
 *
 * Output is clamped to [out_min, out_max]. An optional warmup period suppresses
 * updates until enough samples have been collected for stable feedback.
 *
 * Usage per period:
 * @code
 *   double adj = pi.update(measured_value);
 * @endcode
 */
class PIController
{
  public:
    /**
     * @param kp          Proportional gain
     * @param ki          Integral gain
     * @param target      Desired setpoint
     * @param dt          Period length in seconds
     * @param out_min     Lower output clamp
     * @param out_max     Upper output clamp
     * @param integral_clamp  Symmetric integral anti-windup limit
     * @param warmup_cycles   Number of initial calls to skip (returns 1.0 during warmup)
     */
    PIController(double kp, double ki, double target, double dt, double out_min, double out_max, double integral_clamp,
                 unsigned int warmup_cycles = 64)
        : kp_(kp), ki_(ki), target_(target), dt_(dt), out_min_(out_min), out_max_(out_max),
          integral_clamp_(integral_clamp), warmup_cycles_(warmup_cycles)
    {
    }

    /**
     * @brief Feeds a new measurement and returns the updated output.
     *
     * @param measured Current feedback value
     * @return Clamped PI output (1.0 during warmup)
     */
    double update(double measured)
    {
        if (warmup_ < warmup_cycles_)
        {
            ++warmup_;
            return 1.0;
        }

        const double e = target_ - measured;
        integral_ += e * dt_;
        if (integral_ > integral_clamp_)
            integral_ = integral_clamp_;
        else if (integral_ < -integral_clamp_)
            integral_ = -integral_clamp_;

        const double out = 1.0 + kp_ * e + ki_ * integral_;
        last_output_ = out < out_min_ ? out_min_ : (out > out_max_ ? out_max_ : out);
        return last_output_;
    }

    double output() const
    {
        return last_output_;
    }

    void reset()
    {
        integral_ = 0.0;
        warmup_ = 0;
        last_output_ = 1.0;
    }

  private:
    const double kp_;
    const double ki_;
    const double target_;
    const double dt_;
    const double out_min_;
    const double out_max_;
    const double integral_clamp_;
    const unsigned int warmup_cycles_;

    double integral_ = 0.0;
    double last_output_ = 1.0;
    unsigned int warmup_ = 0;
};

/**
 * @brief Implements a sinc interpolation based resampler
 */
class SincInterpolator
{
  public:
    /**
     * @brief Constructs a sinc interpolation resampler
     *
     * @param order Filter order
     * @param precision Interpolation precision
     * @param chan Number of audio channels
     * @param ratio Resampling ratio
     */
    SincInterpolator(int order, int precision, unsigned int chan, double ratio);

    ~SincInterpolator();

    /**
     * @brief Processes audio data with the sinc interpolator
     *
     * @param in Input audio data
     * @param out Output resampled audio data
     * @param ch Channel number to process
     */
    void process(ArrayView<const float> in, ArrayView<float> out, unsigned int ch);

    /**
     * @brief Sets the ratio adjustment for clock drift compensation.
     *
     * Scales the nominal step by adj so that when the input is also scaled by adj,
     * the output frame count remains exactly dst_fr.
     * @param adj Multiplier for step (clamped to [0.995, 1.005])
     */
    void set_ratio_adjust(double adj);

    double get_ratio_adjust() const
    {
        return ratio_adjust_;
    }

  private:
    float kernel_func(double x, double cutoff) const;

    float interpolator(double x, const float *input, const float *prev_pos) const;

  private:
    const int order;
    const int quan;
    const unsigned int chan;
    const double step;          ///< Nominal step = src_fs / dst_fs
    double ratio_adjust_ = 1.0; ///< PI-controlled multiplier; effective_step = step * ratio_adjust_

    std::vector<double> left;
    std::vector<bool> ready;
    std::unique_ptr<float[]> prev;
    std::unique_ptr<float[]> kern;
};

/**
 * @brief Professional volume controller for audio processing
 *
 * Uses logarithmic volume scaling and smooth transitions to provide
 * professional-grade volume control that matches human hearing perception
 */
class VolumeController
{
  public:
    /**
     * @brief Constructs a volume controller
     *
     * @param sample_rate Audio sample rate in Hz
     * @param channels Number of audio channels
     * @param smooth_time_ms Time for volume changes to smooth out in milliseconds
     */
    explicit VolumeController(float sample_rate, unsigned int channels, float smooth_time_ms = 10.0f);

    /**
     * @brief Processes audio buffer with volume control
     *
     * @param input Audio buffer to process (modified in place)
     * @param gain Volume level (0-100, where 0=mute, 50=medium, 100=unity gain)
     */
    void process(ChannelBuffer<float> *input, unsigned int gain);

  private:
    float convert_gain_to_linear(float gain_percent) const;

  private:
    const float sample_rate;
    const unsigned int channels;

    std::vector<float> current_gain; ///< Current gain per channel (linear scale)
    float target_gain;               ///< Target gain (linear scale)
    float smooth_coeff;              ///< Smoothing coefficient for volume changes
};

/**
 * @brief Fade effect processor for audio streams
 *
 * Designed for periodic processing calls (5-40ms periods)
 * Performs fade-in or fade-out over a specified duration
 */
class FadeEffect
{
  public:
    /**
     * @brief Fade effect type
     */
    enum FadeType
    {
        INIT,    ///< Initial state before any fade
        NONE,    ///< No fade effect
        FADE_IN, ///< Fade from silence to full volume
        FADE_OUT ///< Fade from full volume to silence
    };

    /**
     * @brief Constructs a fade effect processor
     *
     * @param sample_rate Audio sample rate in Hz
     */
    FadeEffect(float sample_rate);

    /**
     * @brief Decides fade type based on stream priorities
     *
     * @param highest_priority Highest priority among active streams
     * @param current_priority Current stream's priority
     * @return FadeType Decided fade type
     */
    void decide_fade_type(unsigned int highest_priority, unsigned int current_priority);

    /**
     * @brief Processes audio buffer with fade effect
     *
     * Call this method periodically (every 5-40ms) to apply fade
     *
     * @param buffer Audio buffer to process (modified in place)
     */
    void process(ChannelBuffer<float> *buffer);

  private:
    void start(FadeType type, float duration_ms);
    float compute_gain() const;

  private:
    const float sample_rate;

    FadeType fade_type;
    unsigned int position;      ///< Current sample position in fade
    unsigned int total_samples; ///< Total samples for complete fade
};

/**
 * @brief Audio format conversion and processing class
 *
 * Handles sample rate conversion, channel mapping, and audio processing
 */
class LocSampler
{
  public:
    /**
     * @brief Constructs an audio sample converter and processor
     *
     * @param src_fs Source sample rate in Hz
     * @param src_ch Number of source channels
     * @param dst_fs Destination sample rate in Hz
     * @param dst_ch Number of destination channels
     * @param max_frames Maximum number of input frames to process
     * @param imap Input channel mapping configuration
     * @param omap Output channel mapping configuration
     */
    LocSampler(unsigned int src_fs, unsigned int src_ch, unsigned int dst_fs, unsigned int dst_ch,
               unsigned int max_frames, const AudioChannelMap &imap, const AudioChannelMap &omap);

    ~LocSampler();

    /**
     * @brief Processes audio data with format conversion
     *
     * Performs channel conversion, sample rate conversion,
     * and volume control
     *
     * @param input Input audio data (interleaved PCM)
     * @param output Output audio data buffer (interleaved PCM)
     * @param gain Volume level (0-100)
     * @return RetCode Status code indicating success or failure
     */
    RetCode process(const InterleavedView<const PCM_TYPE> &input, const InterleavedView<PCM_TYPE> &output,
                    unsigned int gain, FadeEffect *effetor = nullptr) const;

    /**
     * @brief Propagates ratio adjustment to the internal SincInterpolator.
     *
     * No-op when src_fs == dst_fs (no resampler allocated).
     */
    void set_ratio_adjust(double adj);
    double get_ratio_adjust() const;

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
    std::unique_ptr<SincInterpolator> resampler;
    std::unique_ptr<VolumeController> volume_controller;
    std::unique_ptr<ChannelBuffer<float>> analysis_ibuffer;
    std::unique_ptr<ChannelBuffer<float>> analysis_obuffer;

    /**
     * @brief Converts interleaved 16-bit PCM to deinterleaved float samples
     *
     * @param interleaved Input interleaved PCM data
     * @param deinterleaved Output deinterleaved float data
     */
    void deinterleave_s16_f16(const InterleavedView<const PCM_TYPE> &interleaved,
                              ChannelBuffer<float> *deinterleaved) const;

    /**
     * @brief Converts deinterleaved float samples to interleaved 16-bit PCM
     *
     * @param deinterleaved Input deinterleaved float data
     * @param interleaved Output interleaved PCM data
     */
    void interleave_f16_s16(ChannelBuffer<float> *deinterleaved, const InterleavedView<PCM_TYPE> &interleaved) const;

    /**
     * @brief Converts between different channel configurations
     *
     * @param io Buffer containing audio data to be converted in place
     */
    void convert_channels(ChannelBuffer<float> *io) const;

    /**
     * @brief Converts audio data between different sample rates
     *
     * @param input Input audio data at source sample rate
     * @param output Output audio data at destination sample rate
     */
    void convert_sample_rate(ChannelBuffer<float> *input, ChannelBuffer<float> *output, size_t actual_src_frames) const;
};

#endif
