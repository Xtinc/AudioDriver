#ifndef AUDIO_PROCESSING_FILTER_BANKS_H
#define AUDIO_PROCESSING_FILTER_BANKS_H

#include "array_view.h"

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

#endif