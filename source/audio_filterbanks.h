#ifndef AUDIO_PROCESSING_FILTER_BANKS_H
#define AUDIO_PROCESSING_FILTER_BANKS_H

#include "audio_dataview.h"

/**
 * @brief An implementation of a 3-band FIR filter-bank with DCT modulation
 *
 * Similar to the proposed in "Multirate Signal Processing for Communication Systems" by
 * Fredric J Harris.
 *
 * The low-pass filter prototype has these characteristics:
 * * Pass-band ripple = 0.3dB
 * * Pass-band frequency = 0.147 (7kHz at 48kHz)
 * * Stop-band attenuation = 40dB
 * * Stop-band frequency = 0.192 (9.2kHz at 48kHz)
 * * Delay = 24 samples (500us at 48kHz)
 * * Linear phase
 *
 * This filter bank does not satisfy perfect reconstruction. The SNR after
 * analysis and synthesis (with no processing in between) is approximately 9.5dB
 * depending on the input signal after compensating for the delay.
 */
class ThreeBandFilterBank final
{
  public:
    static constexpr int kSparsity = 4;
    static constexpr int kStrideLog2 = 2;
    static constexpr int kStride = 1 << kStrideLog2;
    static constexpr int kNumZeroFilters = 2;
    static constexpr int kFilterSize = 4;
    static constexpr int kMemorySize = kFilterSize * kStride - 1;
    static_assert(kMemorySize == 15, "The memory size must be sufficient to provide memory for the "
                                     "shifted filters");
    static constexpr int kNumBands = 3;
    static constexpr int kFullBandSize = 480;
    static constexpr int kSplitBandSize = kFullBandSize / kNumBands;
    static constexpr int kNumNonZeroFilters = kSparsity * kNumBands - kNumZeroFilters;

    ThreeBandFilterBank();
    ~ThreeBandFilterBank();

    /**
     * @brief Splits the input signal into 3 downsampled frequency bands
     *
     * @param in Full band input signal of size kFullBandSize
     * @param out Array of output bands, each of size 160
     */
    void Analysis(ArrayView<const float, kFullBandSize> in, ArrayView<const ArrayView<float>, kNumBands> out);

    /**
     * @brief Merges 3 downsampled frequency bands into a full band signal
     *
     * @param in Array of input bands, each of size 160
     * @param out Full band output signal of size kFullBandSize
     */
    void Synthesis(ArrayView<const ArrayView<float>, kNumBands> in, ArrayView<float, kFullBandSize> out);

  private:
    std::array<std::array<float, kMemorySize>, kNumNonZeroFilters> state_analysis{};
    std::array<std::array<float, kMemorySize>, kNumNonZeroFilters> state_synthesis{};
};

/**
 * @brief An implementation of a 2-band QMF filter-bank
 */
class TwoBandFilterBank final
{
  public:
    static constexpr int kStateSize = 6;
    static constexpr int kNumBands = 2;
    static constexpr int kFullBandSize = 320;
    static constexpr int kSplitBandSize = kFullBandSize / kNumBands;

    TwoBandFilterBank();
    ~TwoBandFilterBank();

    /**
     * @brief Splits the input signal into 2 downsampled frequency bands
     *
     * @param in Full band input signal of size kFullBandSize
     * @param out Array of output bands, each of size 160
     */
    void Analysis(ArrayView<const float, kFullBandSize> in, ArrayView<const ArrayView<float>, kNumBands> out);

    /**
     * @brief Merges 2 downsampled frequency bands into a full band signal
     *
     * @param in Array of input bands, each of size 160
     * @param out Full band output signal of size kFullBandSize
     */
    void Synthesis(ArrayView<const ArrayView<float>, kNumBands> in, ArrayView<float, kFullBandSize> out);

  private:
    std::array<std::array<float, kStateSize>, kNumBands> state_analysis{};
    std::array<std::array<float, kStateSize>, kNumBands> state_synthesis{};
};

/**
 * @brief Splitting filter which is able to split into and merge from 2 or 3 frequency bands
 *
 * The number of channels needs to be provided at construction time.
 * For each block, Analysis() is called to split into bands and then Synthesis()
 * to merge these bands again. The input and output signals are contained in
 * ChannelBuffers and for the different bands an array of ChannelBuffers is used.
 */
class SplittingFilter
{
  public:
    /**
     * @brief Constructs a splitting filter
     *
     * @param num_channels Number of audio channels to process
     * @param num_bands Number of frequency bands (2 or 3)
     * @param num_frames Number of frames per block
     */
    SplittingFilter(size_t num_channels, size_t num_bands, size_t num_frames);

    /**
     * @brief Destructor
     */
    ~SplittingFilter();

    /**
     * @brief Splits the input signal into multiple frequency bands
     *
     * @param data Input channel buffer with full frequency range
     * @param bands Output channel buffer to store the split frequency bands
     */
    void Analysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands);

    /**
     * @brief Merges multiple frequency bands into a full band signal
     *
     * @param bands Input channel buffer containing the frequency bands
     * @param data Output channel buffer to store the merged signal
     */
    void Synthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data);

  private:
    void TwoBandsAnalysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands);
    void TwoBandsSynthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data);
    void ThreeBandsAnalysis(const ChannelBuffer<float> *data, ChannelBuffer<float> *bands);
    void ThreeBandsSynthesis(const ChannelBuffer<float> *bands, ChannelBuffer<float> *data);

    const size_t num_bands_;
    std::vector<TwoBandFilterBank> two_bands_states_;
    std::vector<ThreeBandFilterBank> three_band_filter_banks_;
};

/**
 * @brief An implementation of Least Mean Squares (LMS) adaptive filter
 *
 * The LMS filter is an adaptive digital filter that uses the least mean squares
 * algorithm to automatically adjust its coefficients to minimize the mean square
 * error between the desired and actual output signals.
 *
 * The filter implements the normalized LMS (NLMS) algorithm which provides
 * better convergence properties by normalizing the step size with the input
 * signal power. The adaptation equation is:
 *
 * w(n+1) = w(n) + μ/(ε + ||x(n)||²) * e(n) * x(n)
 *
 * Where:
 * - w(n) is the weight vector at time n
 * - μ is the step size parameter (controls adaptation speed)
 * - ε is the regularization constant (prevents division by zero)
 * - x(n) is the input signal vector
 * - e(n) is the error signal (desired - output)
 *
 * Features:
 * * Normalized step size for stable convergence
 * * Circular buffer for efficient memory usage
 * * Regularization to prevent numerical instability
 * * Reset capability for reinitialization
 */
class LMSFilter
{
  public:
    /**
     * @brief Constructs an LMS filter
     *
     * @param filter_length Length of the adaptive filter (number of taps)
     *                      Typical values: 32-512 samples
     * @param step_size Step size parameter for the LMS algorithm (μ)
     *                  Typical values: 0.01-1.0, smaller = more stable but slower
     */
    LMSFilter(size_t filter_length, float step_size);

    /**
     * @brief Destructor
     */
    ~LMSFilter() = default;

    /**
     * @brief Processes a sample and adapts the filter coefficients
     *
     * This function performs one iteration of the NLMS algorithm:
     * 1. Updates circular buffer with new input
     * 2. Computes filter output using current weights: y(n) = w^T(n) * x(n)
     * 3. Calculates error signal: e(n) = d(n) - y(n)
     * 4. Updates filter weights using NLMS rule
     *
     * @param input Input signal sample (x(n))
     * @param desired Desired signal sample (d(n)) - reference/target signal
     * @return Filtered output signal (y(n))
     */
    float Process(float input, float desired);

    /**
     * @brief Resets the filter state
     *
     * Clears all internal buffers and resets filter weights to zero.
     * Use when starting adaptation on a new signal or handling discontinuities.
     */
    void Reset();

    /**
     * @brief Gets the current error signal
     * @return Last computed error value
     */
    float GetError() const
    {
        return error_;
    }

    /**
     * @brief Gets current filter length
     * @return Number of filter taps
     */
    size_t GetLength() const
    {
        return length_;
    }

  private:
    const float step_size_;      ///< Adaptation step size parameter (μ)
    const float regularization_; ///< Regularization constant (ε) to prevent division by zero
    const size_t length_;        ///< Length of the adaptive filter (number of taps)

    std::vector<float> buffer_;  ///< Circular input buffer x(n), x(n-1), ..., x(n-L+1)
    std::vector<float> weights_; ///< Adaptive filter coefficients w0, w1, ..., wL-1
    size_t buffer_index_;        ///< Current index in the circular buffer
    float error_;                ///< Last computed error signal e(n) = d(n) - y(n)
};

/**
 * @brief Audio channel mapping for LMS filter processing
 *
 * First element: target channel index (channel to be filtered)
 * Second element: reference channel index (desired signal source)
 */
typedef std::array<unsigned int, 2> AudioChannelMap;

/**
 * @brief Bank of LMS filters for multi-channel audio processing
 *
 * This class manages multiple LMS filters, each configured with a specific
 * channel mapping. It's designed for real-time audio processing scenarios
 * such as:
 * - Acoustic echo cancellation (AEC)
 * - Noise cancellation between microphone channels
 * - Adaptive feedback suppression
 *
 * The filter bank processes interleaved audio data where each LMS filter
 * operates on a pair of channels defined by the channel mapping.
 */
class LMSFilterBank
{
  public:
    /**
     * @brief Constructs an LMS filter bank
     *
     * Creates one LMS filter for each channel mapping. Each filter adapts
     * independently based on its assigned channel pair.
     *
     * @param channel_maps Vector of channel mappings for each LMS filter
     *                     Each map contains [target_channel, reference_channel]
     * @param filter_length Length of each LMS filter (number of taps)
     * @param step_size Step size parameter for all LMS filters
     */
    LMSFilterBank(const std::vector<AudioChannelMap> &channel_maps, size_t filter_length, float step_size);

    /**
     * @brief Destructor
     */
    ~LMSFilterBank() = default;

    /**
     * @brief Processes interleaved audio data through the filter bank
     *
     * For each configured channel pair:
     * 1. Extracts target and reference channel samples
     * 2. Applies LMS filtering: filtered = target - lms_output
     * 3. Updates the target channel with filtered result
     * 4. Clamps output to prevent overflow
     *
     * @param input Interleaved audio buffer (modified in-place)
     *              Format: [ch0_sample0, ch1_sample0, ..., chN_sample0,
     *                       ch0_sample1, ch1_sample1, ..., chN_sample1, ...]
     */
    void process(InterleavedView<int16_t> input);

    /**
     * @brief Resets all LMS filters in the bank
     */
    void reset();

  private:
    bool available_;
    std::vector<AudioChannelMap> channel_maps_; ///< Channel mappings for each LMS filter
    std::vector<LMSFilter> lms_filters_;        ///< Collection of LMS filters
};

#endif