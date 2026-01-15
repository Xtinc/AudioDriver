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

/**
 * @brief Dynamic Range Compressor for audio processing
 */
class DRCompressor
{
  public:
    /**
     * @brief Constructs a dynamic range compressor
     *
     * @param sample_rate Audio sample rate in Hz
     * @param chs Number of audio channels
     * @param threshold Compression threshold in dB
     * @param ratio Compression ratio
     * @param attack Attack time in seconds
     * @param release Release time in seconds
     * @param knee_width Width of the knee in dB
     */
    explicit DRCompressor(float sample_rate, unsigned int chs, float threshold = -6.0f, float ratio = 2.0f,
                          float attack = 0.006f, float release = 0.1f, float knee_width = 6.0f);

    /**
     * @brief Processes audio through the compressor
     *
     * @param input Audio buffer to process
     * @param gain Additional gain to apply in dB
     */
    void process(ChannelBuffer<float> *input, float gain);

  private:
    float compute_gain(float inputLevel) const;

  private:
    const float threshold;
    const float ratio;
    const float attack;
    const float release;
    const float knee_width;
    const unsigned int channels;

    std::vector<float> current_gain;
    float attack_coeff;
    float release_coeff;

    float knee_threshold_lower;
    float knee_threshold_upper;
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
    enum class FadeType
    {
        NONE,    ///< No fade effect
        FADE_IN, ///< Fade from silence to full volume
        FADE_OUT ///< Fade from full volume to silence
    };

    /**
     * @brief Constructs a fade effect processor
     *
     * @param sample_rate Audio sample rate in Hz
     * @param channels Number of audio channels
     */
    FadeEffect(float sample_rate, unsigned int channels);

    /**
     * @brief Starts a fade effect
     *
     * @param type Type of fade effect to apply
     * @param duration_ms Fade duration in milliseconds
     */
    void start(FadeType type, float duration_ms);

    /**
     * @brief Stops the current fade effect immediately
     */
    void stop();

    /**
     * @brief Checks if fade effect is currently active
     *
     * @return true if fade is in progress, false otherwise
     */
    bool is_active() const
    {
        return fade_type != FadeType::NONE;
    }

    /**
     * @brief Processes audio buffer with fade effect
     *
     * Call this method periodically (every 5-40ms) to apply fade
     *
     * @param buffer Audio buffer to process (modified in place)
     */
    void process(ChannelBuffer<float> *buffer);

  private:
    float compute_gain() const;

  private:
    const float sample_rate;
    const unsigned int channels;

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
     * and dynamic range compression
     *
     * @param input Input audio data (interleaved PCM)
     * @param output Output audio data buffer (interleaved PCM)
     * @param gain Gain to apply in dB
     * @return RetCode Status code indicating success or failure
     */
    RetCode process(const InterleavedView<const PCM_TYPE> &input, const InterleavedView<PCM_TYPE> &output,
                    float gain) const;

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
    std::unique_ptr<DRCompressor> compressor;
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
    void convert_sample_rate(ChannelBuffer<float> *input, ChannelBuffer<float> *output) const;
};

#endif
